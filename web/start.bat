@echo off
rem ============================================================
rem  elf3 MuJoCo Web Viewer  --  Windows 启动脚本
rem  双击本文件即可启动本地服务器并打开浏览器
rem ============================================================
setlocal
cd /d "%~dp0"

set "NODE="
where node >nul 2>nul
if %errorlevel%==0 set "NODE=node"

if not defined NODE (
  set "CAND=%USERPROFILE%\.workbuddy\binaries\node\versions\22.22.2-6\node.exe"
  if exist "%CAND%" set "NODE=%CAND%"
)

if not defined NODE (
  echo.
  echo  [ERROR] 未找到 Node.js。
  echo  请安装 Node.js 18+ 后重试：https://nodejs.org/
  echo.
  pause
  exit /b 1
)

echo.
echo  使用 Node: %NODE%
"%NODE%" server.mjs --open %*
echo.
pause
