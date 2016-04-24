@echo off
cls
SET mypath=%~dp0

REM register driver
"%ProgramFiles(x86)%\Steam\steamapps\common\SteamVR\bin\win32\vrpathreg.exe" adddriver "%mypath:~0,-1%\hydra"

REM copy steamvr.vrsettings to steam config dir
copy "%mypath:~0,-1%\steamvr.vrsettings" "%ProgramFiles(x86)%\Steam\config\"
