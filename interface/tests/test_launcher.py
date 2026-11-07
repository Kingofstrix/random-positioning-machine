"""Tests for the launcher bridge, with a fake serial port instead of hardware."""

import http.client
import json
import os
import queue
import sys
import threading
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import launcher  # noqa: E402


class FakeSerial:
    """Feeds queued lines to readline() and records writes."""

    def __init__(self):
        self.written = []
        self.incoming = queue.Queue()
        self.closed = False

    def write(self, data):
        self.written.append(data)
        return len(data)

    def readline(self):
        try:
            item = self.incoming.get(timeout=0.05)
        except queue.Empty:
            return b""
        if isinstance(item, Exception):
            raise item
        return item

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


def port(device, vid=None):
    return SimpleNamespace(device=device, vid=vid)


# --- port detection --------------------------------------------------------------------

def test_find_port_prefers_ch340():
    ports = [port("/dev/cu.Bluetooth"), port("/dev/cu.wchusbserial1410", 0x1A86)]
    assert launcher.find_port(ports) == "/dev/cu.wchusbserial1410"


def test_find_port_falls_back_to_single_port():
    assert launcher.find_port([port("COM3", 0x2341)]) == "COM3"


def test_find_port_none_when_ambiguous_or_empty():
    assert launcher.find_port([port("a"), port("b")]) is None
    assert launcher.find_port([]) is None


def test_find_port_override_wins():
    assert launcher.find_port([port("x", 0x1A86)], override="COM7") == "COM7"


# --- HTTP ------------------------------------------------------------------------------

@pytest.fixture
def setup(tmp_path):
    """Running server + bridge; `ports` controls what the fake lister returns."""
    index = tmp_path / "index.html"
    index.write_text("<!doctype html><title>RPM</title>")
    fake = FakeSerial()
    ports = []
    bridge = launcher.Bridge(opener=lambda dev, baud: fake, lister=lambda: list(ports),
                             reset_wait=0, retry=0.05, sleep=lambda s: None)
    server = launcher.make_server(bridge, 0, str(index))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    bridge.start()
    yield SimpleNamespace(bridge=bridge, fake=fake, ports=ports,
                          port=server.server_address[1])
    bridge.stop()
    server.shutdown()
    server.server_close()


def request(port_no, method, path, body=None):
    conn = http.client.HTTPConnection("127.0.0.1", port_no, timeout=5)
    conn.request(method, path, body=body, headers={"Content-Type": "text/plain"})
    resp = conn.getresponse()
    return resp.status, resp.read().decode()


def wait_connected(bridge, timeout=3.0):
    for _ in range(int(timeout / 0.02)):
        if bridge.serial is not None:
            return
        threading.Event().wait(0.02)
    raise AssertionError("bridge never connected")


def test_get_root_serves_html(setup):
    status, body = request(setup.port, "GET", "/")
    assert status == 200
    assert "<title>RPM</title>" in body


def test_cmd_503_without_port(setup):
    status, _ = request(setup.port, "POST", "/cmd", "status")
    assert status == 503


def test_cmd_writes_line_with_newline(setup):
    setup.ports.append(port("/dev/fake", 0x1A86))
    wait_connected(setup.bridge)
    status, body = request(setup.port, "POST", "/cmd", "start")
    assert (status, body) == (200, "ok")
    assert setup.fake.written == [b"start\n"]


def read_event(resp):
    """Parse one SSE event (skipping keepalive comments) into (event, data)."""
    event = data = None
    while True:
        line = resp.fp.readline().decode().rstrip("\n")
        if line.startswith(":"):
            continue
        if line.startswith("event: "):
            event = line[7:]
        elif line.startswith("data: "):
            data = line[6:]
        elif line == "" and event:
            return event, data


def test_events_stream_port_then_lines(setup):
    conn = http.client.HTTPConnection("127.0.0.1", setup.port, timeout=5)
    conn.request("GET", "/events")
    resp = conn.getresponse()
    assert resp.status == 200
    assert resp.getheader("Content-Type") == "text/event-stream"

    assert read_event(resp) == ("port", json.dumps({"connected": False, "port": None}))

    setup.ports.append(port("/dev/fake", 0x1A86))
    event, data = read_event(resp)
    assert event == "port"
    assert json.loads(data) == {"connected": True, "port": "/dev/fake"}

    setup.fake.incoming.put(b"ok state=STOPPED gres=1.000\r\n")
    setup.fake.incoming.put(b"err unknown\n")
    assert read_event(resp) == ("line", "ok state=STOPPED gres=1.000")
    assert read_event(resp) == ("line", "err unknown")

    # Unplugging: read error -> disconnected event, then reconnect after the port returns.
    setup.ports.clear()
    setup.fake.incoming.put(OSError("device gone"))
    assert json.loads(read_event(resp)[1]) == {"connected": False, "port": None}
    setup.ports.append(port("/dev/fake", 0x1A86))
    assert json.loads(read_event(resp)[1])["connected"] is True
    conn.close()


# --- forwarding to the simulation (sim.hil) --------------------------------------------

class RecordingForwarder:
    def __init__(self):
        self.sent = []

    def send(self, direction, line):
        self.sent.append((direction, line))


def test_is_telemetry():
    assert launcher.is_telemetry("T 20 1454 1454 RUNNING")
    assert launcher.is_telemetry("C start clino3d")
    assert launcher.is_telemetry("C!")
    assert not launcher.is_telemetry("ok state=RUNNING mode=random")
    assert not launcher.is_telemetry("err unknown command: Cx (try help)")


def test_forward_copies_both_directions_and_hides_telemetry_from_page():
    fake = FakeSerial()
    fwd = RecordingForwarder()
    bridge = launcher.Bridge(opener=lambda d, b: fake, lister=lambda: [port("/dev/fake", 0x1A86)],
                             reset_wait=0, sleep=lambda s: None, forwarder=fwd)
    page = bridge.subscribe()
    assert bridge.try_connect()
    assert bridge.write("  start \n")
    for raw in (b"C start\r\n", b"ok state=RUNNING mode=random\r\n", b"T 20 1546 1454 RUNNING\r\n"):
        fake.incoming.put(raw)
        bridge.read_once()
    assert fwd.sent == [("tx", "start"), ("rx", "C start"), ("rx", "ok state=RUNNING mode=random"),
                        ("rx", "T 20 1546 1454 RUNNING")]
    shown = []
    while not page.empty():
        shown.append(page.get_nowait())
    assert ("line", "ok state=RUNNING mode=random") in shown
    assert not any(e == "line" and launcher.is_telemetry(d) for e, d in shown)


def test_forward_keeps_boot_lines_for_sim_only():
    # Without forwarding the bootloader noise is thrown away; sim.hil needs the boot banner.
    fake = FakeSerial()
    for raw in (b"\x00\xff\r\n", b"ok rpm_nano ready (send help) telemetry\r\n", b"T 20 1454 1454 RUNNING\r\n"):
        fake.incoming.put(raw)
    fwd = RecordingForwarder()
    bridge = launcher.Bridge(opener=lambda d, b: fake, lister=lambda: [port("/dev/fake", 0x1A86)],
                             reset_wait=0.3, forwarder=fwd)
    page = bridge.subscribe()
    page.get_nowait()  # initial port state
    assert bridge.try_connect()
    lines = [l for d, l in fwd.sent if d == "rx"]
    assert lines[1:] == ["ok rpm_nano ready (send help) telemetry", "T 20 1454 1454 RUNNING"]
    events = []
    while not page.empty():
        events.append(page.get_nowait()[0])
    assert events == ["port"]  # the page saw the connection, none of the boot lines


def test_sim_forwarder_waits_for_sim_hil_and_delivers_in_order():
    import socket

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    addr = "127.0.0.1:%d" % srv.getsockname()[1]
    fwd = launcher.SimForwarder(addr, retry=0.05)
    fwd.start()
    # sim.hil is not listening yet: sending must not block, lines wait in the queue.
    fwd.send("rx", "ok rpm_nano ready (send help) telemetry")
    fwd.send("tx", "status")
    srv.listen(1)
    conn, _ = srv.accept()
    conn.settimeout(3)
    fwd.send("rx", "C status")
    data = b""
    while data.count(b"\n") < 3:
        data += conn.recv(4096)
    fwd.stop()
    conn.close()
    srv.close()
    msgs = [json.loads(l) for l in data.decode().splitlines()]
    assert msgs == [{"dir": "rx", "line": "ok rpm_nano ready (send help) telemetry"},
                    {"dir": "tx", "line": "status"}, {"dir": "rx", "line": "C status"}]


def test_sim_forwarder_drops_instead_of_blocking_when_full():
    fwd = launcher.SimForwarder("127.0.0.1:9", maxsize=2)  # never started, never drains
    for i in range(5):
        fwd.send("rx", str(i))
    assert fwd.dropped == 3
