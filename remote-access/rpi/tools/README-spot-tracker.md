# Spot Tracker Prototype

Prototype de suivi du faisceau sur flux `RTMP`.

## Objectif

Mesurer en live le drift du spot dans l'image, sans IA dans un premier temps.

Le script :

- lit un flux `RTMP`
- detecte le spot lumineux
- permet de definir une reference par clic
- permet de definir une zone active rectangulaire
- calcule `dx / dy` et la distance au viseur
- publie optionnellement l'etat en MQTT

## Fichiers

- [spot_tracker.py](/Users/r/Documents/PlatformIO/Projects/heliostat%20esp32/remote-access/rpi/tools/spot_tracker.py)
- [requirements-vision.txt](/Users/r/Documents/PlatformIO/Projects/heliostat%20esp32/remote-access/rpi/tools/requirements-vision.txt)

## Installation

Depuis `remote-access/rpi` :

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -r tools/requirements-vision.txt
```

## Exemple

```bash
cd remote-access/rpi
source .venv/bin/activate
python tools/spot_tracker.py \
  --stream-url "rtmp://example/live/stream" \
  --display \
  --publish-mqtt
```

## Commandes dans la fenetre

- clic gauche : definir la reference
- `Shift` + clic gauche + drag : definir la zone active
- `c` : effacer la reference
- `x` : effacer la zone active
- `q` : quitter

## Zone active

La zone active permet d'exclure le ciel ou d'autres regions parasites.

Le tracker ne garde alors que les blobs lumineux qui tombent dans ce rectangle.

La zone est persistante dans :

- `remote-access/rpi/data/vision_zone.json`

## MQTT

Topics publies :

- `heliostat/remote1/vision/state`
- `heliostat/remote1/vision/availability`

Payload type :

```json
{
  "stream_ok": true,
  "frame_width_px": 1280,
  "frame_height_px": 720,
  "reference_x_px": 640.0,
  "reference_y_px": 360.0,
  "spot_x_px": 628.4,
  "spot_y_px": 347.1,
  "dx_px": -11.6,
  "dy_px": -12.9,
  "distance_px": 17.4,
  "area_px": 42.0,
  "confidence": 0.93,
  "threshold_value": 228.0,
  "max_value": 252.0,
  "timestamp_unix_ms": 1777500000000
}
```

## Limites actuelles

- pas encore de stabilisation de scene
- pas encore d'integration backend/UI
- detection basee sur seuillage simple et centroide

Donc :

- il faut une camera fixe
- une exposition aussi stable que possible
- et un spot suffisamment plus lumineux que le fond
