---
description: Flasher un module ESP32 avec PlatformIO
---

# Flasher un module ESP32

## Prérequis
- PlatformIO installé (extension VS Code ou CLI)
- ESP32 connecté en USB
- Driver USB installé (CP2102 ou CH340 selon la carte)

## Étapes

1. Configurer les secrets (première fois uniquement)
```bash
cp include/secrets.h.example include/secrets.h
```

2. Éditer `include/secrets.h` avec les bonnes valeurs
```cpp
#define WIFI_SSID "VotreSSID"
#define WIFI_PASSWORD "VotreMotDePasse"
#define MODULE_ID "nom-du-module"
```

3. Vérifier le `platformio.ini` pour l'adresse MQTT broker
```ini
build_flags =
    -D MQTT_SERVER=\"192.168.1.xxx\"
```

// turbo
4. Compiler et uploader
```bash
pio run -t upload
```

// turbo
5. Ouvrir le moniteur série pour vérifier
```bash
pio device monitor
```

## Troubleshooting

- **Upload failed** : Appuyer sur le bouton BOOT de l'ESP32 pendant l'upload
- **Port not found** : Vérifier le driver USB, essayer un autre câble
- **WiFi connection failed** : Vérifier SSID/password dans secrets.h
- **MQTT refused** : Vérifier que le broker Mosquitto tourne et que l'IP est correcte
