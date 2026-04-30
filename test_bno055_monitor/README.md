# BNO055 Monitor Test

Mini projet autonome pour lire un `BNO055` sur un `ESP32-S3` et afficher les angles en serial.

## Ce que le code affiche

- `heading`
- `roll`
- `pitch`
- l'etat de calibration `sys/gyro/accel/mag`
- le statut interne du BNO055
- et publie aussi ces infos en MQTT sur le meme broker que le remote moteur

## Structure

- `platformio.ini`
- `src/main.cpp`

## Commandes

Depuis ce dossier:

```bash
~/.platformio/penv/bin/pio run -e esp32s3
~/.platformio/penv/bin/pio run -e esp32s3 -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```

## Cablage minimal

- `3V3` -> `VIN` / `3V3` du module selon ton breakout
- `GND` -> `GND`
- `SDA` -> pin I2C ESP32-S3
- `SCL` -> pin I2C ESP32-S3

Adresse I2C attendue par defaut:

- `0x28`

Si ton module est strap sur l'autre adresse, change `BNO055_I2C_ADDRESS` dans `src/main.cpp` vers `0x29`.

## Pins I2C

Par defaut, le code fait `Wire.begin()` sans forcer les broches.

Si le capteur n'est pas detecte:

1. ouvre `src/main.cpp`
2. remplace:

```cpp
constexpr int BNO055_SDA_PIN = -1;
constexpr int BNO055_SCL_PIN = -1;
```

par les bonnes broches de ta carte, par exemple:

```cpp
constexpr int BNO055_SDA_PIN = 8;
constexpr int BNO055_SCL_PIN = 9;
```

## Remarque utile

Les axes `heading / roll / pitch` du BNO055 dependent de l'orientation physique du module. Pour l'utiliser ensuite sur le miroir, il faudra le fixer dans une orientation stable et noter clairement quelle face/pointe correspond a quel axe.

## MQTT

Le projet reutilise [remote_secrets.h](/Users/r/Documents/PlatformIO/Projects/heliostat%20esp32/include/remote_secrets.h) via `-I../include`.

Topics publies :

- `heliostat/remote1/bno055/state`
- `heliostat/remote1/bno055/availability`
