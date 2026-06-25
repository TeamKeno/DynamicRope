@echo off
setlocal
REM Launch the editor loading the project modules built in the DebugGameEditor configuration.
REM The -debug switch tells the Development UnrealEditor.exe to load the -DebugGame project DLLs.
REM Requires DebugGameEditor to be built; otherwise -debug fails to find the DLLs.

REM Resolve the engine path, most-reliable source first:
set "ENGINE="
REM 1) explicit override
if defined UE_ROOT set "ENGINE=%UE_ROOT%"
REM 2) default Epic Games Launcher install (immune to codepage / registry bitness)
if not defined ENGINE if exist "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" set "ENGINE=C:\Program Files\Epic Games\UE_5.7"
REM 3) registry fallback, forcing the 64-bit view (a 32-bit cmd would otherwise read empty WOW6432Node)
if not defined ENGINE for /f "tokens=2,*" %%a in ('reg query "HKLM\SOFTWARE\EpicGames\Unreal Engine\5.7" /v InstalledDirectory /reg:64 2^>nul ^| findstr /i "InstalledDirectory"') do set "ENGINE=%%b"

if not defined ENGINE (
  echo [!] Could not find the UE 5.7 install path. Set the UE_ROOT env var to your engine root.
  pause
  exit /b 1
)

if not exist "%ENGINE%\Engine\Binaries\Win64\UnrealEditor.exe" (
  echo [!] UnrealEditor.exe not found: "%ENGINE%\Engine\Binaries\Win64\UnrealEditor.exe"
  echo     Set the UE_ROOT env var to the correct engine root.
  pause
  exit /b 1
)

REM This batch lives in <repo>\Batch\, so the uproject is one folder up.
echo Launching DebugGameEditor (loading -DebugGame project modules)...
echo   Engine:  %ENGINE%
echo   Project: %~dp0..\DynamicRopeProject.uproject
REM 'start' launches the editor detached so this window does not block for the whole session; it closes on its own.
start "" "%ENGINE%\Engine\Binaries\Win64\UnrealEditor.exe" "%~dp0..\DynamicRopeProject.uproject" -debug
endlocal
