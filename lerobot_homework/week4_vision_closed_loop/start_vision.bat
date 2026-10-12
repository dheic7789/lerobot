@echo off
chcp 65001 >nul
REM 2.2 视觉闭环一键启动（使用 lerobot 环境，勿用系统默认 python）
REM 修复 OMP Error #15：torch 的 libomp5md.dll 与 placo 的 libomp.dll 双 OpenMP 运行时冲突
set KMP_DUPLICATE_LIB_OK=TRUE
REM 用法：双击运行 = 视觉闭环；或在命令行追加参数（如 --mode camera --camera-id 1）
"C:\Users\juyim\miniforge3\envs\lerobot\python.exe" "D:\lerobot_workspace\lerobot_homework\vision_closed_loop.py" %*
echo.
echo 程序已退出。按任意键关闭窗口...
pause >nul
