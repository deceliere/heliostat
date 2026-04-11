# Test ST3215

Projet de test autonome pour partir de zero avec le servo bus TTL `ST3215`.

## Point important

Le `ST3215` n'est pas un servo PWM classique.

- il ne se pilote pas avec `writeMicroseconds()`
- il utilise un bus serie TTL half-duplex sur la broche `A`

## Cablagе conseille cote ESP32

Pins proposes pour les essais:

- `GPIO17` : UART TX
- `GPIO18` : UART RX
- `GPIO16` : TX enable (`TXEN`)

Servo:

- `V` : alimentation servo externe positive
- `G` : masse commune
- `A` : ligne DATA vers une interface half-duplex

## Attention

La ligne `A` ne devrait pas etre reliee directement a une seule GPIO nue de l'ESP32 si tu veux un vrai dialogue bidirectionnel fiable.

Le schema Waveshare du bus servo montre une petite interface half-duplex entre:

- TX ESP32
- RX ESP32
- un signal `TXEN`
- la ligne `DATA` du servo

## Etat actuel du dossier

Pour l'instant, le sketch:

- initialise `UART1` a `1_000_000` bauds
- affiche les pins recommandees
- permet juste de forcer `TXEN` depuis le moniteur serie

Ce dossier sert de base propre avant d'ajouter:

- le protocole ST/SC
- le ping du servo
- la lecture de position
- les commandes de position

## Commandes

```bash
cd test_st3215
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```

Dans le moniteur serie:

```text
help
tx on
tx off
```
