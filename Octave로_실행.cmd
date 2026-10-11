@echo off
rem GNU Octave로 run_all 실행 (MATLAB 사용자는 MATLAB에서 이 폴더를 열고 run_all 실행)
chcp 65001 >nul
set "OH="
for /d %%D in ("C:\Program Files\GNU Octave\Octave-*") do set "OH=%%D"
if not defined OH (
  echo GNU Octave를 찾지 못했습니다. PowerShell에서 winget install GNU.Octave 로 설치하세요.
  pause
  exit /b 1
)
set "PATH=%OH%\mingw64\bin;%OH%\mingw64\qt6\bin;%OH%\usr\bin;%PATH%"
set "QT_PLUGIN_PATH=%OH%\mingw64\qt6\plugins"
cd /d "%~dp0"
"%OH%\mingw64\bin\octave-gui.exe" --no-gui -q --eval "run_all"
echo.
echo 결과는 out 폴더에 있습니다.
pause
