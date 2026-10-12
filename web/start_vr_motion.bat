@echo off
chcp 65001 >nul
rem ============================================================
rem  elf3 运动仿真 · 一键在网页端 / VR 头显里观看
rem
rem  和 start.bat 的区别：本脚本直接进入「运动仿真回放」模式
rem  （策略跟踪 fight.npz 的打拳动作），并自动准备好回放数据。
rem  start.bat / start.sh 保持原样，只做「静止站立」查看。
rem
rem  用法：双击本文件即可。命令行可选参数：
rem    --rebake        重新跑一遍本机 MuJoCo 仿真，刷新回放数据
rem    --http-port N   指定 HTTP 端口（默认 8080）
rem    --https-port N  指定 HTTPS 端口（默认 8443，VR 头显走这个）
rem    其它参数原样转给 server.mjs
rem ============================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

set "REBAKE="
set "ARGS="
:parse
if "%~1"=="" goto parsed
if /i "%~1"=="--rebake" (set "REBAKE=1") else (set "ARGS=!ARGS! %~1")
shift
goto parse
:parsed

set "NODE="
where node >nul 2>nul
if %errorlevel%==0 set "NODE=node"
if not defined NODE (
  set "CAND=%USERPROFILE%\.workbuddy\binaries\node\versions\22.22.2-6\node.exe"
  if exist "!CAND!" set "NODE=!CAND!"
)
if not defined NODE (
  echo.
  echo  [错误] 未找到 Node.js 18+，请先安装：https://nodejs.org/
  echo.
  pause
  exit /b 1
)

set "PY="
set "VENV=%USERPROFILE%\.workbuddy\binaries\python\envs\default\Scripts\python.exe"
if exist "!VENV!" set "PY=!VENV!"
if not defined PY (
  where python >nul 2>nul
  if !errorlevel!==0 set "PY=python"
)

set "MOTION=assets\motion_fight_policy.bin"
if defined REBAKE goto bake
if not exist "%MOTION%" goto bake_none
goto run

:bake
if not defined PY (
  echo  [提示] 没找到 Python，跳过重新录制（沿用已有回放数据）。
  goto run
)
echo.
echo  [1/3] 重新录制运动仿真数据（MuJoCo + policy.onnx 闭环，约 20~40 秒）…
echo.
"!PY!" "tools\bake_motion.py"
if errorlevel 1 echo  [警告] 录制失败，继续沿用已有回放数据。
goto run

:bake_none
echo.
echo  [1/3] 首次运行：本地录制运动仿真数据（MuJoCo + policy.onnx 闭环）…
echo.
if defined PY (
  "!PY!" "tools\bake_motion.py"
  if errorlevel 1 echo  [警告] 录制失败：页面会打开，但「运动仿真」可能加载失败。
) else (
  echo  [警告] 缺少回放数据 assets\motion_fight_policy.bin，且未找到 Python。
  echo         页面仍会打开，但「运动仿真」会提示加载失败。
  echo         补数据：python web\tools\bake_motion.py
)

:run
echo.
echo  [2/3] 启动本地服务器（HTTP + HTTPS 双端口）…
echo  [3/3] 浏览器会自动打开「运动仿真」页面。
echo        VR 头显：连同一个 Wi-Fi，在头显浏览器里打开下面打印的
echo        https 局域网地址（带 ?motion=fight_policy 的那一条）。
echo.
"!NODE!" server.mjs --open --open-path "/?motion=fight_policy"!ARGS!
echo.
pause
