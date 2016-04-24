@echo off
cls
SET mypath=%~dp0

REM unregister driver
"%ProgramFiles(x86)%\Steam\steamapps\common\SteamVR\bin\win32\vrpathreg.exe" removedriver "%mypath:~0,-1%\hydra"
