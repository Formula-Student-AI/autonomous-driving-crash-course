@echo off
REM Convenience wrapper for start_relay.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_relay.ps1" %*
