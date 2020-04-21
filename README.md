[![EN](https://user-images.githubusercontent.com/9499881/33184537-7be87e86-d096-11e7-89bb-f3286f752bc6.png)](https://github.com/r57zone/steamvr_driver_hydra) 
[![RU](https://user-images.githubusercontent.com/9499881/27683795-5b0fbac6-5cd8-11e7-929c-057833e01fb1.png)](https://github.com/r57zone/steamvr_driver_hydra/blob/master/README.RU.md) 
# Razer Hydra Driver for SteamVR
The driver emulates HTC Vive controllers.

## Buttons
Vive controller | Razer hydra
------------ | -------------
System button | Middle button
Application menu button | Button 4
Grip button | Bumper
Touchpad button | Joystick button


In some games, it is inconvenient to click Joystick, but this can be edited in SteamVR Bindings UI, if you go to the SteamVR settings, click "Advance Settings" -> "Show" and go to the controllers item.

### Action
Description | Razer Hydra Button
------------ | -------------
Sit down | Button 3


For HMD, you can use any driver that supports button squats (the button is configurable). For example, you can use the [TrueOpenVR and SteamVR bridge driver](https://github.com/TrueOpenVR) for HMD (FreeTrack for HMD from smartphones or ArduinoHMD for [full-fledged DIY HMD](https://github.com/TrueOpenVR/TrueOpenVR-DIY/blob/master/HMD/HMD.md)). While pressing button 3, on the Razer Hydra, the button is also pressed (the button is configurable). Squat settings can be found in the configuration file "default.vrsettings".

## Installation

1. [Download](https://github.com/r57zone/steamvr_driver_hydra/releases/) the latest driver.
2. Unpack archive to "..\Steam\steamapps\common\SteamVR\drivers".
3. [Add option](https://youtu.be/QCA3m4_3IJM?t=197) `"activateMultipleDrivers" : true,` to config "steamvr.vrsettings", to "steamvr" section.
4. Change the dead zone if your stick goes to the side in config (..\Steam\steamapps\common\SteamVR\drivers\razer_hydra\hydra\resources\settings\default.vrsettings), option "joystickdeadzone".

For more detailed instructions please visit the [wiki](https://github.com/betavr/steamvr_driver_hydra/wiki).

## Building

1. Download sources from [branch "steamvr_input_new"](https://github.com/r57zone/steamvr_driver_hydra/tree/r/steamvr_input_new) and unpack.
2. [Download "openvr"](https://github.com/ValveSoftware/openvr) and unpack to "C:\openvr".
3. [Download "SixenseSDK_102215.zip"](https://github.com/r57zone/steamvr_driver_hydra/releases) and unpack to "C:\SixenseSDK_102215".
4. [Download Microsoft Visual Studio Code 2017](https://code.visualstudio.com/download) and compile.
