@echo off
rem Double-click: sets up a local venv once, then starts the RPM UI launcher.
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo First run: creating .venv and installing requirements...
  py -3 -m venv .venv || (echo Python 3 not found - install it from python.org & pause & exit /b 1)
  ".venv\Scripts\python.exe" -m pip install -q -r requirements.txt || (pause & exit /b 1)
)
".venv\Scripts\python.exe" launcher.py %*
if errorlevel 1 pause
