# Test ST3020

Projet PlatformIO autonome pour commencer les essais avec les servos `ST3020`
et le module ESP/alim associe, en s'appuyant sur la librairie constructeur
fournie dans `ST Servo/SCServo`.

Objectif de cette premiere etape:

- valider l'alimentation
- valider le cablage `TX/RX`
- verifier que l'ESP parle bien au servo via `Serial1`
- pouvoir ping/lire/deplacer un servo par ID

Ce dossier ne touche pas au firmware principal du heliostat.

## Fichiers

- `platformio.ini`: environnement autonome ESP32-S3
- `src/main.cpp`: banc de test UART minimal

## Reglages a verifier avant upload

Dans `src/main.cpp`:

- `ST3020_UART_RX_PIN`
- `ST3020_UART_TX_PIN`
- `ST3020_UART_BAUD`

Le baud par defaut est maintenant cale sur l'exemple constructeur:

- `1000000`
- `RX = GPIO18`
- `TX = GPIO19`

## Commandes serie

```text
help
scan
ping
read
id?
id=1
setid=1,2
torque=on
torque=off
move=2047
move=2047,1500
move=2047,1500,100
stop
```

## Commandes PlatformIO

```bash
cd test_st3020
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```
