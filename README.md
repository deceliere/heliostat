# Lampe Aurel ESP32

Firmware ESP32-S3 pour piloter 2 bobines (MOSFET), un dimmer LED, un mode test par bobine, et une interface Web de reglage.

## Cible et stack
- Board: `esp32-s3-devkitm-1`
- Framework: Arduino (PlatformIO)
- Environment: `env:esp32s3`
- Librairie: `Adafruit NeoPixel`

## Broches
- Bobine 1 (PWM): `GPIO4`
- Bobine 2 (PWM): `GPIO5`
- Dimmer LED driver (PWM): `GPIO6`
- Switch activation Wi-Fi (lecture au boot): `GPIO7`
- Coil check LED (ON/OFF): `GPIO8`
- NeoPixel status: `GPIO38`

## PWM
- Bobines (`GPIO4`, `GPIO5`): `31 kHz`, resolution `8 bits`
- Dimmer (`GPIO6`): `1 kHz`, resolution `8 bits`

## Demarrage
Au boot, le firmware:
1. Initialise les sorties PWM.
2. Charge les parametres depuis `Preferences` (namespace `lampe`).
3. Tire aleatoirement la bobine de depart (phase initiale bobine 1 ou 2).
4. Active le Wi-Fi AP uniquement si `GPIO7` est `HIGH`.

## Wi-Fi / Web UI
Si `GPIO7 = HIGH` au boot:
- AP SSID: `Lampe-ESP32`
- Mot de passe: `lampe1234`
- URL: `http://192.168.4.1/`
- Routes:
  - `/` page de controle
  - `/set` application des parametres
  - `/help` page d'aide

## LED de statut (NeoPixel)
- Clignote rouge pendant une fenetre de sauvegarde de parametres.
- Violette en mode test (`test=1` ou `test=2`).
- Bleue fixe si Wi-Fi actif et hors mode test.
- Eteinte sinon.

Note: la LED est configuree en `NEO_GRB`, les composantes couleur sont donc mappees selon ce format.

## Mode normal vs mode test
- `test=1`: force uniquement bobine 1, sans avance des phases/mute.
- `test=2`: force uniquement bobine 2, sans avance des phases/mute.

## Parametres serie
Baudrate: `115200`

Commandes disponibles:
- `period`
- `d1`, `f1`, `p1`
- `d2`, `f2`, `p2`
- `led` (0..255)
- `coil_led` (0|1)
- `test` (0|1|2)
- `n1`, `n1_var`, `n1_mute`, `n1_mute_var`, `n1_off`
- `n2`, `n2_var`, `n2_mute`, `n2_mute_var`, `n2_off`
- `off` (0|1)
- `show`
- `help`

Exemples:
```text
show
led=180
test=1
n1_off=0
off=0
```

## Persistance (Preferences)
Sauvegardes automatiques avec delai (`PARAM_SAVE_DELAY_MS = 2000 ms`) apres modification.

Persistes:
- `period`
- `d1`, `f1`, `p1`
- `d2`, `f2`, `p2`
- `led`
- `n1`, `n1_var`, `n1_mute`, `n1_mute_var`
- `n2`, `n2_var`, `n2_mute`, `n2_mute_var`
- `out_en`
- `coil_led`

Non persiste:
- `test` (retour a la valeur par defaut au reboot)

## Build / Flash
Depuis le dossier projet:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## Fichiers utiles
- Code principal: `src/main.cpp`
- Config PlatformIO: `platformio.ini`
