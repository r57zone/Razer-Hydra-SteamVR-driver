[![EN](https://user-images.githubusercontent.com/9499881/33184537-7be87e86-d096-11e7-89bb-f3286f752bc6.png)](https://github.com/r57zone/steamvr_driver_hydra) 
[![RU](https://user-images.githubusercontent.com/9499881/27683795-5b0fbac6-5cd8-11e7-929c-057833e01fb1.png)](https://github.com/r57zone/steamvr_driver_hydra/blob/master/README.RU.md) 
# Razer Hydra Driver для SteamVR
Драйвер эмулирует HTC Vive контроллеры.

## Установка

1. [Загрузите](https://github.com/r57zone/steamvr_driver_hydra/releases/) последний драйвер.
2. Распакуйте архив в "..\Steam\steamapps\common\SteamVR\drivers".
3. [Добавьте параметр] `"activateMultipleDrivers" : true,` (https://youtu.be/QCA3m4_3IJM?t=197) в конфиг "steamvr.vrsettings", в раздел "steamvr".
4. Измените мертвую зону если ваш стик уходит в сторону, в конфиге (..\Steam\steamapps\common\SteamVR\drivers\razer_hydra\hydra\resources\settings\default.vrsettings), параметр "joystickdeadzone".

Дополнительная информация есть в [вики](https://github.com/betavr/steamvr_driver_hydra/wiki).

## Сборка

1. [Загрузите "openvr"](https://github.com/ValveSoftware/openvr) и распакуйте в "C:\openvr".
2. [Загрузите "SixenseSDK_102215.zip"](https://github.com/r57zone/steamvr_driver_hydra/releases) и распакуйте в "C:\SixenseSDK_102215".
3. [Загрузите Microsoft Visual Studio Code 2017](https://code.visualstudio.com/download) и скомпилируйте.
