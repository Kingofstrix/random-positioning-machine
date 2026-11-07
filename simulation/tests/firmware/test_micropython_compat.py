"""Static check that rpmcore only uses what MicroPython on the RP2040 provides.

This does not replace running tools/pico_selftest.py on a real Pico (docs/PLAN.md 10.1); it
catches the common slips early, on every test run.
"""

import ast
from pathlib import Path

import pytest

RPMCORE = Path(__file__).resolve().parents[2] / "firmware" / "rpmcore"

ALLOWED_DECORATORS = {"staticmethod", "property"}
BANNED_NAMES = {
    "dataclass", "dataclasses", "typing", "enum", "Enum", "IntEnum",
    "abc", "ABC", "abstractmethod", "NamedTuple", "TypedDict",
}
# Core set of the MicroPython math module. Optional extras (isclose, fsum, factorial, ...)
# depend on build flags, so they are not allowed.
MATH_ATTRS = {
    "e", "pi", "inf", "nan", "sqrt", "pow", "exp", "log", "log2", "log10",
    "sin", "cos", "tan", "asin", "acos", "atan", "atan2",
    "ceil", "floor", "trunc", "fabs", "fmod", "modf", "frexp", "ldexp", "copysign",
    "isfinite", "isinf", "isnan", "degrees", "radians",
}


def violations(source: str) -> list[str]:
    tree = ast.parse(source)
    found = []

    def add(node, msg):
        found.append("line %d: %s" % (getattr(node, "lineno", 0), msg))

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name != "math" and not alias.name.startswith("rpmcore."):
                    add(node, "import %s" % alias.name)
                elif alias.name == "math" and alias.asname:
                    # An alias would hide math.<attr> calls from the MATH_ATTRS check.
                    add(node, "import math as %s" % alias.asname)
        elif isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            if node.level:
                add(node, "relative import")
            elif mod != "rpmcore" and not mod.startswith("rpmcore."):
                add(node, "from %s import" % mod)
        elif isinstance(node, ast.JoinedStr):
            add(node, "f-string")
        elif isinstance(node, ast.AnnAssign):
            add(node, "variable annotation")
        elif isinstance(node, ast.arg) and node.annotation is not None:
            add(node, "argument annotation")
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.returns is not None:
                add(node, "return annotation")
            for dec in node.decorator_list:
                if not (isinstance(dec, ast.Name) and dec.id in ALLOWED_DECORATORS):
                    add(dec, "decorator %s" % ast.unparse(dec))
        elif isinstance(node, ast.ClassDef):
            for dec in node.decorator_list:
                add(dec, "class decorator %s" % ast.unparse(dec))
        elif isinstance(node, ast.Match):
            add(node, "match statement")
        elif isinstance(node, ast.Name) and node.id in BANNED_NAMES:
            add(node, "name %s" % node.id)
        elif isinstance(node, ast.Attribute):
            if node.attr in BANNED_NAMES:
                add(node, "attribute %s" % node.attr)
            if isinstance(node.value, ast.Name) and node.value.id == "math":
                if node.attr not in MATH_ATTRS:
                    add(node, "math.%s" % node.attr)
    return found


@pytest.mark.parametrize(
    "snippet",
    [
        "import random",
        "import time",
        "from machine import Pin",
        "from . import prng",
        "from typing import List",
        "x = f'{1}'",
        "x: int = 1",
        "def f(a: int): pass",
        "def f(a) -> int: pass",
        "@functools.lru_cache\ndef f(): pass",
        "@dataclass\nclass A: pass",
        "match x:\n    case 1: pass",
        "y = math.isclose(1, 2)",
        "import math as m\ny = m.isclose(1, 2)",
        "class A(Enum): pass",
    ],
)
def test_checker_catches(snippet):
    assert violations(snippet), snippet


@pytest.mark.parametrize(
    "snippet",
    [
        "import math\nx = math.floor(1.5)",
        "import rpmcore.prng",
        "from rpmcore import prng",
        "from rpmcore.prng import XorShift32",
        "class A:\n    @staticmethod\n    def f(): pass\n    @property\n    def g(self): return 1",
        "s = 'v=%d' % 3",
        "field = 3",
    ],
)
def test_checker_allows(snippet):
    assert violations(snippet) == [], snippet


def rpmcore_files():
    files = sorted(RPMCORE.glob("*.py"))
    assert files, "no files found in %s" % RPMCORE
    return files


@pytest.mark.parametrize("path", rpmcore_files(), ids=lambda p: p.name)
def test_rpmcore_is_micropython_compatible(path):
    assert violations(path.read_text(encoding="utf-8")) == []


def test_expected_modules_are_checked():
    names = {p.name for p in rpmcore_files()}
    assert {"__init__.py", "config.py", "prng.py", "servo_map.py"} <= names
