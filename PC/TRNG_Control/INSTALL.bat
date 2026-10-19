@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" goto install
where py >nul 2>nul
if errorlevel 1 goto usepython
py -3 -m venv .venv
if errorlevel 1 goto fail
goto install
:usepython
python -m venv .venv
if errorlevel 1 goto fail
:install
".venv\Scripts\python.exe" -m pip install --no-index --find-links="%~dp0vendor" -r requirements.txt
if errorlevel 1 goto fail
echo.
echo Cai dat xong. Mo RUN.bat de chay ung dung.
exit /b 0
:fail
echo.
echo Chua cai duoc. Hay cai Python 3.10 tro len tu python.org, kem Tcl/Tk va pip.
pause
exit /b 1
