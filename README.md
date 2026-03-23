# Heliostat ESP32

Base de test pour piloter 2 servos avec un `ESP32-S3 DevKitM-1` et une manette Xbox en Bluetooth.

## Fonction actuelle

- Stick gauche horizontal: servo `pan`
- Stick gauche vertical: servo `tilt`
- Deplacement progressif avec zone morte
- Limites d'angle configurables dans `src/main.cpp`

## Fichiers

- Firmware principal: `src/main.cpp`
- Configuration PlatformIO: `platformio.ini`

## Cablage

- Servo pan sur `GPIO4`
- Servo tilt sur `GPIO5`
- Masse commune entre ESP32 et alimentation servos
- Alimente les servos avec une vraie source `5V`, pas depuis le `3.3V` de l'ESP32

## Remarques

- Les broches et limites mecaniques sont definies en haut de `src/main.cpp`.
- La bibliotheque choisie est `BLE-Gamepad-Client`, adaptee a une manette Xbox BLE recente.
- Si ta manette ne se connecte pas, il faudra verifier sa version / son mode Bluetooth.

## Commandes

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```
