@echo off
cls
SET mypath=%~dp0
for /f "tokens=2*" %%A in ('REG QUERY "HKCU\Software\Valve\Steam" /v SteamPath') DO set SteamPath=%%B

REM unregister driver
"%SteamPath%\steamapps\common\SteamVR\bin\win32\vrpathreg.exe" removedriver "%mypath:~0,-1%\hydra"
