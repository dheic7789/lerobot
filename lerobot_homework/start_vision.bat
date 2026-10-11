@echo off
chcp 65001 >nul
REM 2.2 视觉闭环一键启动（使用 lerobot 环境，勿用系统默认 python）
REM 用法：双击运行 = 视觉闭环；或在命令行追加参数（如 --mode camera）
"C:\Users\juyim\miniforge3\envs\lerobot\python.exe" "C:\Users\juyim\lerobot_homework\vision_closed_loop.py" %*
echo.
echo 程序已退出。按任意键关闭窗口...
pause >nul
