# Test simple servo

Petit projet PlatformIO autonome pour faire balayer le servo `pan` de `0` a `180` puis de `180` a `0` en boucle.

## Broche

- Servo `pan`: `GPIO5`

## Reglage vitesse

Ouvrir le moniteur serie a `115200` bauds puis envoyer une ligne avec un nombre en `deg/s`.

Exemples:

```text
10
45
90
```

## Commandes

```bash
cd test_simple_servo
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```
