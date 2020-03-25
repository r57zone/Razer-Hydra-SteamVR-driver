[![EN](https://user-images.githubusercontent.com/9499881/33184537-7be87e86-d096-11e7-89bb-f3286f752bc6.png)](https://github.com/r57zone/steamvr_driver_hydra) 
[![RU](https://user-images.githubusercontent.com/9499881/27683795-5b0fbac6-5cd8-11e7-929c-057833e01fb1.png)](https://github.com/r57zone/steamvr_driver_hydra/blob/master/README.RU.md) 
# Razer Hydra Driver for SteamVR
The driver emulates HTC Vive controllers.

## Installation

1. [Download](https://github.com/r57zone/steamvr_driver_hydra/releases/) the latest driver.
2. Unpack archive to "..\Steam\steamapps\common\SteamVR\drivers".
3. [Add option] `"activateMultipleDrivers" : true,` (https://youtu.be/QCA3m4_3IJM?t=197) to config "steamvr.vrsettings", to "steamvr" section.
4. Change the dead zone if your stick goes to the side in config (..\Steam\steamapps\common\SteamVR\drivers\razer_hydra\hydra\resources\settings\default.vrsettings), option "joystickdeadzone".

For more detailed instructions please visit the [wiki](https://github.com/betavr/steamvr_driver_hydra/wiki).

## Building

1. [Download "openvr"](https://github.com/ValveSoftware/openvr) and unpack to "C:\openvr".
2. [Download "SixenseSDK_102215.zip"](https://github.com/r57zone/steamvr_driver_hydra/releases) and unpack to "C:\SixenseSDK_102215".
3. [Download Microsoft Visual Studio Code 2017](https://code.visualstudio.com/download) and compile.
