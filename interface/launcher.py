"""Laptop-side launcher for the RPM minimal UI.

Browsers cannot open a serial port without Web Serial (Chrome only, and only over https or
localhost), so this small bridge owns the Arduino's USB-serial port and hands it to the page
over plain HTTP on 127.0.0.1:

    GET  /        -> index.html next to this script
    GET  /events  -> Server-Sent Events: `port` (connection state) and `line` (raw serial lines)
    POST /cmd     -> one command line, written to serial with "\\n" (503 without a port)

With --forward it also copies every serial line, both directions, to the simulation's
hardware-in-the-loop checker (`python -m sim.hil` in simulation/) on 127.0.0.1:8766.

Only the standard library and pyserial are used so the start scripts stay trivial.
"""

import argparse
import json
import os
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CH340_VID = 0x1A86
DEFAULT_BAUD = 115200
DEFAULT_HTTP_PORT = 8765
HOST = "127.0.0.1"
KEEPALIVE_S = 15.0
RETRY_S = 1.0
# Opening the port toggles DTR, which resets an Arduino Nano. The bootloader then runs for
# about 1-2 s and swallows anything we send, so we wait before reporting "connected".
RESET_WAIT_S = 2.0
INDEX_HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html")
# sim.hil listens here (not 8765: that is this launcher's own HTTP port).
SIM_ADDRESS = "127.0.0.1:8766"
SIM_RETRY_S = 3.0
# Lines kept while sim.hil is not reachable yet: ~6 min of telemetry at 50 lines/s.
SIM_QUEUE_MAX = 20000


def is_telemetry(line):
    """Lines of the Nano's telemetry test build: `T ...` per tick, `C ...` per command.

    They are meant for sim.hil only; the page would drown in 50 lines per second.
    """
    s = line.strip()
    return s.startswith(("T ", "C ")) or s == "C!"


class SimForwarder:
    """Copies serial traffic to sim.hil: one JSON line {"dir": "rx"|"tx", "line": ...} per
    serial line over a local TCP connection.

    Never blocks the serial loop. Lines wait in a bounded queue until sim.hil is reachable, so
    the launcher may start first; it reconnects every few seconds if sim.hil goes away.
    """

    def __init__(self, address=SIM_ADDRESS, connect=socket.create_connection, retry=SIM_RETRY_S,
                 maxsize=SIM_QUEUE_MAX):
        host, _, port = address.rpartition(":")
        self.address = (host or "127.0.0.1", int(port))
        self.connect = connect
        self.retry = retry
        self.q = queue.Queue(maxsize=maxsize)
        self.dropped = 0
        self.connected = False
        self.stop_event = threading.Event()

    def send(self, direction, line):
        try:
            self.q.put_nowait(json.dumps({"dir": direction, "line": line}) + "\n")
        except queue.Full:
            self.dropped += 1

    def run(self):
        sock = None
        item = None
        while not self.stop_event.is_set():
            if sock is None:
                try:
                    sock = self.connect(self.address, timeout=2)
                    self.connected = True
                    print("forwarding to sim.hil at %s:%d" % self.address, flush=True)
                except OSError:
                    self.stop_event.wait(self.retry)
                    continue
            if item is None:
                try:
                    item = self.q.get(timeout=0.5)
                except queue.Empty:
                    continue
            try:
                sock.sendall(item.encode("utf-8"))
                item = None
            except OSError:
                # Keep the line and resend it after reconnecting.
                sock, self.connected = None, False
                print("sim.hil went away, retrying every %g s" % self.retry, flush=True)
        if sock is not None:
            sock.close()

    def start(self):
        t = threading.Thread(target=self.run, name="sim-forward", daemon=True)
        t.start()
        return t

    def stop(self):
        self.stop_event.set()


def list_ports():
    """Return pyserial's port list; imported lazily so tests run without pyserial."""
    from serial.tools import list_ports as lp
    return list(lp.comports())


def open_serial(device, baud):
    import serial
    return serial.Serial(device, baud, timeout=0.2)


def find_port(ports, override=None):
    """Pick the Arduino: explicit override, else a CH340, else the only port there is."""
    if override:
        return override
    for p in ports:
        if getattr(p, "vid", None) == CH340_VID:
            return p.device
    if len(ports) == 1:
        return ports[0].device
    return None


class Bridge:
    """Owns the serial port and fans its lines out to every SSE client.

    Serial opener, port lister and sleep are injectable so the tests run without hardware.
    """

    def __init__(self, baud=DEFAULT_BAUD, port_override=None, opener=open_serial,
                 lister=list_ports, reset_wait=RESET_WAIT_S, retry=RETRY_S, sleep=time.sleep,
                 forwarder=None):
        self.forwarder = forwarder  # SimForwarder or None
        self.baud = baud
        self.port_override = port_override
        self.opener = opener
        self.lister = lister
        self.reset_wait = reset_wait
        self.retry = retry
        self.sleep = sleep
        self.serial = None
        self.device = None
        self.clients = []
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()

    # --- SSE fan-out -------------------------------------------------------------------
    def subscribe(self):
        q = queue.Queue()
        with self.lock:
            self.clients.append(q)
            # A new page must learn the current state right away, not on the next change.
            q.put(("port", self.port_payload()))
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def broadcast(self, event, data):
        with self.lock:
            for q in self.clients:
                q.put((event, data))

    def port_payload(self):
        return json.dumps({"connected": self.serial is not None, "port": self.device})

    # --- serial ------------------------------------------------------------------------
    def write(self, line):
        """Send one command; returns False when no port is open."""
        ser = self.serial
        if ser is None:
            return False
        try:
            with self.write_lock:
                ser.write((line.strip() + "\n").encode("utf-8"))
        except Exception:
            self._drop()
            return False
        if self.forwarder:
            self.forwarder.send("tx", line.strip())
        return True

    def _drop(self):
        ser, self.serial, self.device = self.serial, None, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
            self.broadcast("port", self.port_payload())

    def try_connect(self):
        """One attempt to find and open the port; True on success."""
        try:
            device = find_port(self.lister(), self.port_override)
        except Exception:
            device = None
        if device is None:
            return False
        try:
            ser = self.opener(device, self.baud)
        except Exception:
            return False
        if self.forwarder:
            # sim.hil checks the Nano from its boot on, so the boot banner and first ticks must
            # not be thrown away: they go to sim.hil only, the page still starts clean.
            self._boot_to_sim(ser)
        else:
            self.sleep(self.reset_wait)  # Nano reset via DTR, see RESET_WAIT_S
            try:
                ser.reset_input_buffer()  # drop bootloader noise
            except Exception:
                pass
        self.serial, self.device = ser, device
        self.broadcast("port", self.port_payload())
        return True

    def _boot_to_sim(self, ser):
        """Spend the reset wait reading, and hand what the Nano says to sim.hil only."""
        deadline = time.monotonic() + self.reset_wait
        while time.monotonic() < deadline:
            try:
                raw = ser.readline()
            except Exception:
                return
            if raw:
                self.forwarder.send("rx", raw.decode("utf-8", errors="replace").rstrip("\r\n"))

    def read_once(self):
        """Read one line (or time out) and broadcast it; drops the port on I/O errors."""
        try:
            raw = self.serial.readline()
        except Exception:
            self._drop()
            return
        if raw:
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if self.forwarder:
                self.forwarder.send("rx", line)
            if not is_telemetry(line):
                self.broadcast("line", line)

    def run(self):
        """Background loop: connect, read, and reconnect every second after unplugging."""
        while not self.stop_event.is_set():
            if self.serial is None:
                if not self.try_connect():
                    self.stop_event.wait(self.retry)
                continue
            self.read_once()

    def start(self):
        t = threading.Thread(target=self.run, name="serial", daemon=True)
        t.start()
        return t

    def stop(self):
        self.stop_event.set()
        self._drop()


def make_handler(bridge, index_path=INDEX_HTML, keepalive=KEEPALIVE_S):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            pass  # the page polls every 500 ms; logging that would drown the terminal

        def _send(self, code, body, ctype="text/plain; charset=utf-8"):
            data = body.encode("utf-8") if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                try:
                    with open(index_path, "rb") as f:
                        self._send(200, f.read(), "text/html; charset=utf-8")
                except OSError:
                    self._send(404, "index.html not found")
            elif path == "/events":
                self._events()
            else:
                self._send(404, "not found")

        def do_POST(self):
            if self.path.split("?", 1)[0] != "/cmd":
                self._send(404, "not found")
                return
            length = int(self.headers.get("Content-Length") or 0)
            line = self.rfile.read(length).decode("utf-8", errors="replace")
            if bridge.write(line):
                self._send(200, "ok")
            else:
                self._send(503, "no serial port")

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            q = bridge.subscribe()
            try:
                while not bridge.stop_event.is_set():
                    try:
                        event, data = q.get(timeout=keepalive)
                        msg = "event: %s\ndata: %s\n\n" % (event, data)
                    except queue.Empty:
                        msg = ": keepalive\n\n"  # keeps proxies/browsers from timing out
                    self.wfile.write(msg.encode("utf-8"))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                bridge.unsubscribe(q)

    return Handler


def make_server(bridge, http_port=DEFAULT_HTTP_PORT, index_path=INDEX_HTML):
    """Bind to 127.0.0.1 only; fall back to a free port if the default is taken."""
    handler = make_handler(bridge, index_path)
    try:
        server = ThreadingHTTPServer((HOST, http_port), handler)
    except OSError:
        server = ThreadingHTTPServer((HOST, 0), handler)
    server.daemon_threads = True
    return server


def open_app_window(url):
    """Prefer a chromeless Chrome/Edge window (feels like an app); else the default browser."""
    arg = "--app=" + url
    candidates = []
    if sys.platform == "darwin":
        for app in ("Google Chrome", "Microsoft Edge", "Chromium"):
            if os.path.isdir("/Applications/%s.app" % app):
                candidates.append(["open", "-na", app, "--args", arg])
    elif sys.platform.startswith("win"):
        roots = [os.environ.get(k, "") for k in ("PROGRAMFILES", "PROGRAMFILES(X86)", "LOCALAPPDATA")]
        rels = [r"Google\Chrome\Application\chrome.exe", r"Microsoft\Edge\Application\msedge.exe"]
        for rel in rels:
            for root in roots:
                exe = os.path.join(root, rel)
                if root and os.path.isfile(exe):
                    candidates.append([exe, arg])
    else:
        for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser",
                     "microsoft-edge"):
            exe = shutil.which(name)
            if exe:
                candidates.append([exe, arg])
    for cmd in candidates:
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return
        except OSError:
            continue
    webbrowser.open(url)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Serve the RPM UI and bridge it to the Arduino.")
    ap.add_argument("--port", help="serial port (default: auto-detect CH340)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT)
    ap.add_argument("--no-browser", action="store_true", help="do not open a browser window")
    ap.add_argument("--forward", nargs="?", const=SIM_ADDRESS, metavar="HOST:PORT",
                    help="copy all serial lines to the simulation's checker, python -m sim.hil "
                         "(default %s)" % SIM_ADDRESS)
    args = ap.parse_args(argv)

    forwarder = SimForwarder(args.forward) if args.forward else None
    if forwarder:
        forwarder.start()
        print("forwarding serial lines to sim.hil at %s once it runs" % args.forward, flush=True)
    bridge = Bridge(baud=args.baud, port_override=args.port, forwarder=forwarder)
    server = make_server(bridge, args.http_port)
    url = "http://%s:%d/" % (HOST, server.server_address[1])
    bridge.start()
    print("RPM UI at %s  (Ctrl+C to quit)" % url, flush=True)
    if not args.no_browser:
        open_app_window(url)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        bridge.stop()
        if forwarder:
            forwarder.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
