@echo off
REM Convenience wrapper for start_flyscreen_live.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_flyscreen_live.ps1" %*
