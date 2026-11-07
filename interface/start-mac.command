#!/bin/sh
# Double-click in Finder: sets up a local venv once, then starts the RPM UI launcher.
cd "$(dirname "$0")" || exit 1
if [ ! -x .venv/bin/python ]; then
  echo "First run: creating .venv and installing requirements..."
  python3 -m venv .venv || { echo "python3 not found - install Python 3 from python.org"; read _; exit 1; }
  .venv/bin/python -m pip install -q -r requirements.txt || { read _; exit 1; }
fi
exec .venv/bin/python launcher.py "$@"
