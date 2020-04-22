[![EN](https://user-images.githubusercontent.com/9499881/33184537-7be87e86-d096-11e7-89bb-f3286f752bc6.png)](https://github.com/r57zone/steamvr_driver_hydra) 
[![RU](https://user-images.githubusercontent.com/9499881/27683795-5b0fbac6-5cd8-11e7-929c-057833e01fb1.png)](https://github.com/r57zone/steamvr_driver_hydra/blob/master/README.RU.md) 
# Razer Hydra Driver для SteamVR
Драйвер эмулирует HTC Vive контроллеры.

## Кнопки
Vive контроллер | Razer Hydra
------------ | -------------
Системная кнопка | Средняя кнопка
Кнопка меню | Кнопка 4
Кнопка захвата | Бампер
Нажатие тачпада | Нажатие джойстика


В некоторых играх неудобно нажимать Joystick, но это можно отредактировать в SteamVR Bindings UI, если перейти в настройки SteamVR, нажать "Advance Settings" -> "Show" и перейти пункт контроллеры.

### Действия
Описание | Razer Hydra кнопка
------------ | -------------
Присесть | Кнопка 3


Для HMD можно использовать любой драйвер с поддержкой приседания по кнопке. Например, можно использовать [TrueOpenVR и SteamVR мост драйвер](https://github.com/TrueOpenVR) для HMD (FreeTrack для HMD из смартфонов или ArduinoHMD для [полноценных DIY шлемов](https://github.com/TrueOpenVR/TrueOpenVR-DIY/blob/master/HMD/HMD.RU.md)). Во время нажатия кнопки 3, на Razer Hydra, также нажимается кнопка клавиатуры (кнопка настраивается). Настройки приседания можно найти в конфигурационном файле "default.vrsettings".

## Установка

1. [Загрузите](https://github.com/r57zone/steamvr_driver_hydra/releases/) последний драйвер.
2. Распакуйте архив в "..\Steam\steamapps\common\SteamVR\drivers".
3. [Добавьте параметр](https://youtu.be/QCA3m4_3IJM?t=197) `"activateMultipleDrivers" : true,` в конфиг "steamvr.vrsettings", в раздел "steamvr".
4. Измените мертвую зону если ваш стик уходит в сторону, в конфиге (..\Steam\steamapps\common\SteamVR\drivers\razer_hydra\hydra\resources\settings\default.vrsettings), параметр "JoyStickDeadZone".

Дополнительная информация есть в [вики](https://github.com/betavr/steamvr_driver_hydra/wiki).

## Сборка

1. Загрузите исходники из [ветки "steamvr_input_new"](https://github.com/r57zone/steamvr_driver_hydra/tree/r/steamvr_input_new) и распакуйте.
2. [Загрузите "openvr"](https://github.com/ValveSoftware/openvr) и распакуйте в "C:\openvr".
3. [Загрузите "SixenseSDK_102215.zip"](https://github.com/r57zone/steamvr_driver_hydra/releases/tag/1) и распакуйте в "C:\SixenseSDK_102215".
4. [Загрузите Microsoft Visual Studio Code 2017](https://code.visualstudio.com/download) и скомпилируйте.
