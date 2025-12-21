# Air Quality ESP32 Firmware

Firmware ESP32 pour module de surveillance de qualité de l'air. Publie les données capteurs via MQTT.

## 🔧 Installation

### Prérequis

- [PlatformIO](https://platformio.org/) (VS Code extension recommandée)
- ESP32 DevKit

### Configuration

1. **Créer le fichier de secrets** :
   ```bash
   cp include/secrets.h.example include/secrets.h
   ```

2. **Éditer `include/secrets.h`** :
   ```cpp
   #define WIFI_SSID "VotreSSID"
   #define WIFI_PASSWORD "VotreMotDePasse"
   #define MODULE_ID "module-esp32-1"  // Identifiant unique du module
   ```

3. **Configurer le serveur MQTT** dans `platformio.ini` :
   ```ini
   build_flags = 
       -D MQTT_SERVER=\"192.168.1.162\"
   ```

### Compilation & Upload

```bash
# Via PlatformIO CLI
pio run -t upload

# Ou via VS Code: Bouton Upload (→)
```

---

## 🔌 Capteurs Supportés

| Capteur | Interface | Pins ESP32 | Mesures |
|---------|-----------|------------|---------|
| **DHT22** | 1-Wire | GPIO 4 | Température, Humidité |
| **SHT31** | I2C (Bus 1) | SDA: 32, SCL: 33 | Température, Humidité (alternative DHT22) |
| **BMP280** | I2C (Bus 0) | SDA: 21, SCL: 22 | Pression atmosphérique, Température |
| **SGP40** | I2C (Bus 1) | SDA: 32, SCL: 33 | Indice VOC (0-500) |
| **SGP30** | I2C (Bus 1) | SDA: 32, SCL: 33 | eCO2 (ppm), TVOC (ppb) |
| **MH-Z14A** | UART | RX: 25, TX: 26 | CO2 (ppm) |
| **SPS30** | UART | RX: 13, TX: 27 | PM1.0, PM2.5, PM4.0, PM10 (µg/m³) |

> **Note** : Deux bus I2C séparés pour isoler les capteurs sensibles (SGP40/SGP30/SHT31 sur Bus 1).

---

## 📡 Topics MQTT

### Format des Topics

Les mesures sont publiées en utilisant un format **hardware-aware** :

```
{moduleId}/{hardwareId}/{measurement}
```

**Exemples :**
- `croissance/dht22/temperature` → Température du DHT22
- `croissance/bmp280/temperature` → Température du BMP280
- `croissance/sps30/pm25` → PM2.5 du SPS30

### Topics de Mesures

| Hardware | Topic | Mesure | Unité |
|----------|-------|--------|-------|
| **dht22** | `{moduleId}/dht22/temperature` | Température | °C |
| **dht22** | `{moduleId}/dht22/humidity` | Humidité | % |
| **bmp280** | `{moduleId}/bmp280/temperature` | Température | °C |
| **bmp280** | `{moduleId}/bmp280/pressure` | Pression | hPa |
| **sgp40** | `{moduleId}/sgp40/voc` | Indice VOC | 0-500 |
| **sgp30** | `{moduleId}/sgp30/eco2` | eCO2 | ppm |
| **sgp30** | `{moduleId}/sgp30/tvoc` | TVOC | ppb |
| **mhz14a** | `{moduleId}/mhz14a/co2` | CO2 | ppm |
| **sps30** | `{moduleId}/sps30/pm1` | PM1.0 | µg/m³ |
| **sps30** | `{moduleId}/sps30/pm25` | PM2.5 | µg/m³ |
| **sps30** | `{moduleId}/sps30/pm4` | PM4.0 | µg/m³ |
| **sps30** | `{moduleId}/sps30/pm10` | PM10 | µg/m³ |
| **sht40** | `{moduleId}/sht40/temperature` | Température | °C |
| **sht40** | `{moduleId}/sht40/humidity` | Humidité | % |
| **mq7** | `{moduleId}/mq7/co` | Monoxyde de carbone | ppm |

### Topics Système

| Topic | Description |
|-------|-------------|
| `{moduleId}/sensors/status` | Statut JSON de tous les capteurs |
| `{moduleId}/system` | Infos système (IP, RSSI, Mémoire) |
| `{moduleId}/system/config` | Configuration système |
| `{moduleId}/logs` | Logs remote pour debug |

### Topics Souscrits (Commandes)

| Topic | Payload | Description |
|-------|---------|-------------|
| `{moduleId}/sensors/reset` | `{"sensor": "bmp280"}` | Reset un capteur spécifique |
| `{moduleId}/sensors/config` | `{"sensors": {...}}` | Configuration des intervalles |

---

## ⚡ Alimentation

**Consommation totale estimée : ~650mA (pic)**

| Composant | Consommation |
|-----------|--------------|
| ESP32 (WiFi TX) | ~260 mA |
| MH-Z14A (chauffage) | ~150 mA |
| SPS30 (ventilateur) | ~80-100 mA |
| SGP30 (chauffage) | ~48 mA |
| Autres capteurs | ~10 mA |

> ⚠️ **Recommandation** : Utilisez une alimentation USB 2A minimum (chargeur téléphone) plutôt que le port USB d'un PC (500mA max).

**Câblage alimentation :**
- MH-Z14A : **5V** (VIN direct)
- Autres capteurs : **3.3V** (pin 3V3 ESP32)

---

## 🏗️ Architecture du Code

```
src/
├── main.cpp              # Point d'entrée
├── AppController.cpp     # Orchestrateur principal
├── NetworkManager.cpp    # WiFi + MQTT
├── SensorReader.cpp      # Lecture capteurs
├── StatusPublisher.cpp   # Publication MQTT
├── MqttHandler.cpp       # Réception commandes MQTT
├── RemoteLogger.cpp      # Logs distants via MQTT
├── SystemInfoCollector.cpp
└── SystemInitializer.cpp

include/
├── AppController.h
├── SensorReader.h
├── NetworkManager.h
├── MqttHandler.h
├── RemoteLogger.h
├── StatusPublisher.h
├── SensorData.h
├── OtaManager.h          # Mises à jour OTA
├── MHZ14A.h              # Driver CO2
└── secrets.h             # Configuration WiFi (gitignored)
```

---

## 🔄 Reset Capteurs

Le système peut réinitialiser les capteurs sans redémarrer l'ESP32 :

- **BMP280** : Soft reset → Hard I2C recovery si échec
- **SGP40/SGP30** : Reset via librairie Adafruit
- **DHT22/SHT31** : Réinitialisation `begin()`
- **MH-Z14A** : Flush buffer UART

---

## 📦 Dépendances

Gérées automatiquement par PlatformIO :

- `adafruit/DHT sensor library`
- `adafruit/Adafruit SGP40 Sensor`
- `adafruit/Adafruit SGP30 Sensor`
- `adafruit/Adafruit BMP280 Library`
- `adafruit/Adafruit SHT31 Library`
- `sensirion/Sensirion UART SPS30`
- `bblanchon/ArduinoJson`
- `ottowinter/AsyncMqttClient-esphome`
- `tzapu/WiFiManager`

---

## 📝 License

MIT
