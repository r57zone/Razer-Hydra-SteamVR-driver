@echo off
cls
SET mypath=%~dp0
for /f "tokens=2*" %%A in ('REG QUERY "HKCU\Software\Valve\Steam" /v SteamPath') DO set SteamPath=%%B

REM register driver
"%SteamPath%\steamapps\common\SteamVR\bin\win32\vrpathreg.exe" adddriver "%mypath:~0,-1%\hydra"

REM copy steamvr.vrsettings to steam config dir
copy "%mypath:~0,-1%\steamvr.vrsettings" "%SteamPath%\config\"
