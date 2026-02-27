# IoT Mesurable - Module Air Benchmark

Firmware ESP32 complet pour la surveillance de qualité de l'air avec 7+ capteurs. C'est le module de production principal.

## Capteurs

| Capteur | Interface | Pins | Mesures |
|---------|-----------|------|---------|
| DHT22 | 1-Wire | GPIO 4 | Température, Humidité |
| SHT31 | I2C Bus 1 | SDA:32, SCL:33 | Température, Humidité |
| BMP280 | I2C Bus 0 | SDA:21, SCL:22 | Pression, Température |
| SGP40 | I2C Bus 1 | SDA:32, SCL:33 | VOC (0-500) |
| SGP30 | I2C Bus 1 | SDA:32, SCL:33 | eCO2 (ppm), TVOC (ppb) |
| MH-Z14A | UART | RX:25, TX:26 | CO2 (ppm) |
| SPS30 | UART | RX:13, TX:27 | PM1.0, PM2.5, PM4.0, PM10 |

## Architecture I2C (IMPORTANT)

**Deux bus I2C séparés** pour isoler les capteurs :
- **Bus 0** (GPIO 21/22) : BMP280 seul — capteur instable, son reset n'affecte pas les autres
- **Bus 1** (GPIO 32/33) : SGP40, SGP30, SHT31 — capteurs sensibles isolés ensemble

### Adresses I2C
- BMP280 : `0x76` (Bus 0)
- SGP40 : `0x59` (Bus 1)
- SGP30 : `0x58` (Bus 1)
- SHT31 : `0x44` (Bus 1)

## Alimentation

⚠️ **Consommation pic : ~650mA** → USB standard (500mA) INSUFFISANT !
- Utiliser alimentation **2A minimum**
- MH-Z14A et SPS30 : **5V** (VIN direct)
- Autres capteurs : **3.3V** (pin 3V3)

## Structure du code

```
src/
├── main.cpp               # Point d'entrée
├── AppController.cpp      # Orchestrateur principal
├── NetworkManager.cpp     # WiFi + MQTT
├── SensorReader.cpp       # Lecture capteurs
├── StatusPublisher.cpp    # Publication MQTT
├── MqttHandler.cpp        # Réception commandes
├── RemoteLogger.cpp       # Logs distants
├── SystemInfoCollector.cpp
└── SystemInitializer.cpp
```

## Problèmes connus et solutions

### SCD41 qui arrête de reporter
- Le capteur SCD41 peut arrêter de reporter après un certain temps
- Lié à des conflits I2C ou à un problème du cycle de mesure après réinitialisation
- Solution : implémenter un mécanisme de recovery I2C robuste

### Reset capteurs sans redémarrage
- **BMP280** : Soft reset → Hard I2C recovery si échec
- **SGP40/SGP30** : Reset via librairie Adafruit
- **DHT22/SHT31** : Réinitialisation `begin()`
- **MH-Z14A** : Flush buffer UART

### Perte de données après déconnexion WiFi
- Les données sont perdues pendant la déconnexion (pas de buffer local)
- La reconnexion WiFi/MQTT est automatique via la lib bootstrap
