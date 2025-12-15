#include "AppController.h"
#include "SystemInitializer.h"

// Static instance for callbacks
static AppController* instance = nullptr;

// Static callback wrappers
static void staticMqttCallback(char* topic, byte* payload, unsigned int length) {
    if (instance) instance->handleMqttMessage(topic, payload, length);
}

static void staticOnMqttConnected() {
    if (instance) instance->onMqttConnected();
}

AppController::AppController() 
    : co2Serial(2), 
      sps30Serial(1),
      coSerial(16, 17),  // SC16-CO on GPIO 16 (RX), 17 (TX)
      dht(4, DHT22), 
      sensorReader(co2Serial, sps30Serial, dht, Wire1, coSerial), 
      statusPublisher(network, co2Serial, dht),
      mqttHandler(sensorReader, sensorConfig),
      logger(nullptr) 
{
    instance = this;
}

// ================= INITIALIZATION =================

void AppController::initHardware() {
    SystemInitializer::initHardware(co2Serial, dht, network, ota);
}

void AppController::initNetwork() {
    network.onMqttConnectedCallback = staticOnMqttConnected;
    network.setCallback(staticMqttCallback);
}

void AppController::setupLogger() {
    logger = new RemoteLogger(network, String(network.getTopicPrefix()));
    sensorReader.setLogger(logger);
    mqttHandler.setLogger(logger);

    if (logger) {
        if (WiFi.status() == WL_CONNECTED) {
            char wifiInfo[128];
            snprintf(wifiInfo, sizeof(wifiInfo), "✓ WiFi connected - IP: %s, MAC: %s", 
                     WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str());
            logger->success(wifiInfo);
        }
        char moduleInfo[64];
        snprintf(moduleInfo, sizeof(moduleInfo), "Module: %s", network.getTopicPrefix());
        logger->info(moduleInfo);
    }
}

void AppController::initSensors() {
    SystemInitializer::configureSensor();
    
    // Give sensors time to power up (especially SPS30 which can take a few seconds)
    delay(2000);
    
    // Explicitly initialize I2C buses first
    Wire.begin(21, 22);      // Main I2C (BMP280)
    int devicesWire = sensorReader.scanI2C(Wire, "Wire (BMP280)");

    // Initialize second I2C bus for SGP40 with auto-correction
    Wire1.begin(32, 33);     // Default: SDA=32, SCL=33
    int devicesWire1 = sensorReader.scanI2C(Wire1, "Wire1 (SGP40) [32,33]");
    
    // If no devices found on Wire1, try swapping pins
    if (devicesWire1 == 0) {
        if (logger) logger->warn("⚠️ No devices on Wire1 (32,33), trying swap (33,32)...");
        Wire1.begin(33, 32); // Swap: SDA=33, SCL=32
        devicesWire1 = sensorReader.scanI2C(Wire1, "Wire1 (SGP40) [33,32] - SWAPPED");
        
        if (devicesWire1 > 0) {
            if (logger) logger->success("✅ Devices found after swapping pins! Please update wiring or config.");
        } else {
            // Revert to default if still nothing, to match documentation
            Wire1.begin(32, 33);
        }
    }

    // Initialize sensors
    bool bmpOk = sensorReader.initBMP();
    bool sgpOk = sensorReader.initSGP();
    bool sgp30Ok = sensorReader.initSGP30();
    bool sps30Ok = sensorReader.initSPS30();
    bool shtOk = sensorReader.initSHT();
    
    // Initialize SC16-CO (SoftwareSerial)
    coSerial.begin(9600);
    bool coOk = sensorReader.initCO();

    if (logger) {
        if (bmpOk) logger->success("✓ BMP280 initialized");
        else logger->error("✗ BMP280 init failed");

        if (sgpOk) logger->success("✓ SGP40 initialized");
        else logger->error("✗ SGP40 init failed");

        if (sgp30Ok) logger->success("✓ SGP30 initialized");
        else logger->error("✗ SGP30 init failed");

        if (sps30Ok) logger->success("✓ SPS30 (PM) initialized");
        else logger->error("✗ SPS30 init failed");

        if (shtOk) logger->success("✓ SHT3x initialized");
        else logger->error("✗ SHT3x init failed");
        
        if (coOk) logger->success("✓ SC16-CO initialized");
        else logger->warn("⚠ SC16-CO not detected (will retry)");
        
        logger->success("✓ MH-Z14A (CO2) Serial initialized");
        logger->success("✓ DHT22 Sensor initialized");
        logger->info("System ready - waiting for MQTT connection");
    }

    SystemInitializer::runWarmupSequence();

    // Reset timers for immediate read
    // Reset timers for staggered reads (offsets to prevent thundering herd)
    unsigned long now = millis();
    
    // CO2: Immediate
    lastCo2ReadTime = now - sensorConfig.co2Interval;
    
    // Temp/Hum (DHT): +2s
    lastTempReadTime = now - sensorConfig.tempInterval + 2000;
    lastHumReadTime = now - sensorConfig.humInterval + 2000;
    
    // VOC (SGP40): +4s
    lastVocReadTime = now - sensorConfig.vocInterval + 4000;
    
    // PM (SPS30): +6s
    lastPmReadTime = now - sensorConfig.pmInterval + 6000;
    
    // SHT3x: +8s
    lastShtReadTime = now - sensorConfig.shtInterval + 8000;
    
    // Pressure (BMP): +10s
    lastPressureReadTime = now - sensorConfig.pressureInterval + 10000;
    
    // SGP30: +12s (Publish), Read is 1Hz
    lastSgp30PublishTime = now - sensorConfig.eco2Interval + 12000;
    lastSgp30ReadTime = now - 1000;
    
    // SC16-CO (Carbon Monoxide): +14s
    lastCoReadTime = now - sensorConfig.coInterval + 14000;

    lastSystemInfoTime = now - 5000;
}

// ================= CORE UPDATE =================

void AppController::updateNetwork() {
    network.loop();
}

void AppController::updateOta() {
    ota.loop();
}

bool AppController::isNetworkReady() {
    return network.isConnected();
}

// ================= SENSOR HANDLERS =================

void AppController::handleMHZ14A() {
    unsigned long now = millis();
    if (now - lastCo2ReadTime >= sensorConfig.co2Interval) {
        lastCo2ReadTime = now;
        int ppm = sensorReader.readCO2();
        if (ppm >= 0) {
            lastCO2Value = ppm;
            statusCo2 = "ok";
            network.publishCO2(ppm);
            if (logger) {
                char msg[48]; snprintf(msg, sizeof(msg), "📤 CO2: %d ppm", ppm);
                logger->info(msg);
            }
        } else {
             errCo2++;
             if (errCo2 > 5) {
                 if (logger) logger->warn("⚠️ CO2 self-healing: Resetting sensor...");
                 sensorReader.resetCO2();
                 errCo2 = 0;
             }
            
            if (ppm == -1) {
                statusCo2 = "missing";
                if (logger) logger->error("✗ CO2 sensor missing (timeout)");
            } else if (ppm == -4) {
                statusCo2 = "error";
                if (logger) logger->error("✗ CO2 sensor checksum error");
            } else {
                statusCo2 = "error";
                if (logger) logger->error("✗ CO2 sensor read error (code: " + String(ppm) + ")");
            }
        }
    }
}

void AppController::handleDHT22() {
    unsigned long now = millis();
    bool readTemp = (now - lastTempReadTime >= sensorConfig.tempInterval);
    bool readHum = (now - lastHumReadTime >= sensorConfig.humInterval);
    
    if (readTemp || readHum) {
        DhtReading reading = sensorReader.readDhtSensors();
        lastTemperature = reading.temperature;
        lastHumidity = reading.humidity;
        statusDht = reading.valid ? "ok" : "error";

        if (reading.valid) {
            errDht = 0;
            if (readTemp) {
                lastTempReadTime = now;
                network.publishValue("/temperature", reading.temperature);
                if (logger) {
                    char msg[48]; snprintf(msg, sizeof(msg), "📤 Temperature: %.1f°C", reading.temperature);
                    logger->info(msg);
                }
            }
            if (readHum) {
                lastHumReadTime = now;
                network.publishValue("/humidity", reading.humidity);
                if (logger) {
                    char msg[48]; snprintf(msg, sizeof(msg), "📤 Humidity: %.1f%%", reading.humidity);
                    logger->info(msg);
                }
            }
        } else {
             errDht++;
             if (errDht > 10) {
                 if (logger) logger->warn("⚠️ DHT self-healing: Resetting sensor...");
                 sensorReader.resetDHT();
                 errDht = 0;
             }
        }
    }
}

void AppController::handleSGP40() {
    unsigned long now = millis();
    if (now - lastVocReadTime >= sensorConfig.vocInterval) {
        lastVocReadTime = now;
        int voc = sensorReader.readVocIndex();
        lastVocValue = voc;
        
        if (voc >= 0) {
            statusVoc = "ok";
            network.publishVocIndex(voc);
            if (logger) {
                char msg[32]; snprintf(msg, sizeof(msg), "📤 VOC: %d", voc);
                logger->info(msg);
            }
        } else {
            // Self-healing for SGP40
            errSgp40++;
            if (errSgp40 > 5) {
                if (logger) logger->warn("⚠️ SGP40 self-healing: Resetting sensor...");
                if (sensorReader.resetSGP()) {
                    errSgp40 = 0; 
                }
            }

            // voc == -1 (disconnected)
            if (voc == -1) {
                statusVoc = "missing";
                // Only log error if status just changed or periodically? 
                // For now, consistent with other sensors: logging error every attempt might be spammy but standard here.
                if (logger) logger->error("✗ SGP40 sensor missing (disconnected)");
            } else {
                statusVoc = "error";
                if (logger) logger->error("✗ SGP40 sensor processing error");
            }
        }
    }
}

void AppController::handleSGP30() {
    unsigned long now = millis();
    
    // 1. Hardware Read (Must be 1Hz for SGP30 baseline algorithm)
    if (now - lastSgp30ReadTime >= 1000) { 
        lastSgp30ReadTime = now;
        
        int eco2, tvoc;
        if (sensorReader.readSGP30(eco2, tvoc)) {
            lastEco2Value = eco2;
            lastTvocValue = tvoc;
            statusEco2 = "ok";
            statusTvoc = "ok";
        } else {
            // Only log error periodically or if it persists?
            // For now, keep it simple. If 1Hz logging is too much, user will see.
            // But since we only publish on interval, maybe we should suppress 1Hz error logs?
            // Let's keep it but maybe it will spam if sensor is missing.
            statusEco2 = "error";
            statusTvoc = "error";
            // if (logger) logger->error("✗ SGP30 read failed"); // Commented out to reduce spam if missing
            
            errSgp30++;
            if (errSgp30 > 10) {
                 if (logger) logger->warn("⚠️ SGP30 self-healing: Resetting sensor...");
                 sensorReader.resetSGP(); 
                 errSgp30 = 0;
            }
        }
    }

    // 2. Publish (User Configured Interval)
    if (now - lastSgp30PublishTime >= sensorConfig.eco2Interval) {
        lastSgp30PublishTime = now;
        
        if (statusEco2 == "ok") {
            network.publishValue("/eco2", lastEco2Value);
            network.publishValue("/tvoc", lastTvocValue);
            
            if (logger) {
                char msg[64]; snprintf(msg, sizeof(msg), "📤 eCO2: %d ppm, TVOC: %d ppb", lastEco2Value, lastTvocValue);
                logger->info(msg);
            }
        } else {
             if (logger) logger->error("✗ SGP30 not ready or read failed");
        }
    }
}

void AppController::handleSPS30() {
    unsigned long now = millis();
    if (now - lastPmReadTime >= sensorConfig.pmInterval) {
        lastPmReadTime = now;
        
        float pm1, pm25, pm4, pm10;
        if (sensorReader.readSPS30(pm1, pm25, pm4, pm10)) {
            lastPm1 = pm1;
            lastPm25 = pm25;
            lastPm4 = pm4;
            lastPm10 = pm10;
            statusPm = "ok";
            
            // Assuming network.publishValue is generic enough or we simply publish as floats
            network.publishValue("/pm1", pm1);
            network.publishValue("/pm25", pm25);
            network.publishValue("/pm4", pm4);
            network.publishValue("/pm10", pm10);
            
            if (logger) {
                char msg[64]; 
                snprintf(msg, sizeof(msg), "📤 PM2.5: %.1f, PM10: %.1f", pm25, pm10);
                logger->info(msg);
            }
        } else {
            statusPm = "error";
            if (logger) logger->error("✗ SPS30 read failed");
            
            if (errSps30 > 3) {
                if (logger) logger->warn("⚠️ SPS30 self-healing: Resetting sensor...");
                sensorReader.initSPS30(1, 100);
                errSps30 = 0;
            }
        }
    }
}

void AppController::handleBMP280() {
    unsigned long now = millis();
    if (now - lastPressureReadTime >= sensorConfig.pressureInterval) {
        lastPressureReadTime = now;
        float pressure = sensorReader.readPressure();
        float tempBmp = sensorReader.readBMPTemperature();
        lastPressure = pressure;
        lastTempBmp = tempBmp;
        statusPressure = isnan(pressure) ? "error" : "ok";
        statusTempBmp = isnan(tempBmp) ? "error" : "ok";
        
        if (!isnan(pressure)) {
            errBmp = 0;
            network.publishValue("/pressure", pressure);
            network.publishValue("/temperature_bmp", tempBmp);
            if (logger) {
                char msg[96]; snprintf(msg, sizeof(msg), "📤 Pressure: %.1f hPa, TempBMP: %.1f°C", pressure, tempBmp);
                logger->info(msg);
            }
        } else {
             errBmp++;
             if (errBmp > 5) {
                 if (logger) logger->warn("⚠️ BMP280 self-healing: Resetting sensor...");
                 sensorReader.resetBMP();
                 errBmp = 0;
             }
        }
    }
}

void AppController::handleSystemStatus() {
    unsigned long now = millis();
    
    if (mqttJustConnected) {
        mqttJustConnected = false;
        publishAllConfigs();
        if (lastCO2Value > 0) {
            statusPublisher.publishSystemInfo();
            statusPublisher.publishSensorStatus(lastCO2Value, statusCo2, 
                                                lastCoValue, statusCo,
                                                lastTemperature, lastHumidity, statusDht, 
                                                lastVocValue, statusVoc, lastPressure, statusPressure, lastTempBmp, statusTempBmp,
                                                lastPm1, lastPm25, lastPm4, lastPm10, statusPm,
                                                lastEco2Value, statusEco2, lastTvocValue, statusTvoc,
                                                lastTempSht, statusTempSht, lastHumSht, statusHumSht);
            if (lastVocValue >= 0) network.publishVocIndex(lastVocValue);
        }
    }

    if (now - lastSystemInfoTime >= 5000) {
        lastSystemInfoTime = now;
        statusPublisher.publishSystemInfo();
        statusPublisher.publishSensorStatus(lastCO2Value, statusCo2, 
                                            lastCoValue, statusCo,
                                            lastTemperature, lastHumidity, statusDht, 
                                            lastVocValue, statusVoc, lastPressure, statusPressure, lastTempBmp, statusTempBmp,
                                            lastPm1, lastPm25, lastPm4, lastPm10, statusPm,
                                            lastEco2Value, statusEco2, lastTvocValue, statusTvoc,
                                            lastTempSht, statusTempSht, lastHumSht, statusHumSht);
    }
}

// ================= HELPERS & LEGACY =================
void AppController::handleMqttMessage(char* topic, byte* payload, unsigned int length) {
    mqttHandler.handleMessage(topic, payload, length);
}

void AppController::onMqttConnected() {
    mqttJustConnected = true;
    if (logger) {
        logger->flushBufferedLogs();
        logger->success("✓ MQTT connected");
    }
    reconnectAttempts = 0;
}

void AppController::handleSHT3x() {
    unsigned long now = millis();
    if (now - lastShtReadTime >= sensorConfig.shtInterval) {
        lastShtReadTime = now;
        
        float temp, hum;
        if (sensorReader.readSHT(temp, hum)) {
            lastTempSht = temp;
            lastHumSht = hum;
            statusTempSht = "ok";
            statusHumSht = "ok";
            
            network.publishValue("/temp_sht", temp);
            network.publishValue("/hum_sht", hum);
            
            if (logger) {
                char msg[64]; snprintf(msg, sizeof(msg), "📤 SHT3x: %.1f°C, %.1f%%", temp, hum);
                logger->info(msg);
            }
        } else {
            statusTempSht = "error";
            statusHumSht = "error";
            // Set to NAN so frontend treats it as invalid/missing (Red pill)
            lastTempSht = NAN;
            lastHumSht = NAN;
            if (logger) logger->error("✗ SHT3x read failed");
            
            errSht++;
            if (errSht > 5) {
                 if (logger) logger->warn("⚠️ SHT3x self-healing: Resetting sensor...");
                 sensorReader.resetSHT();
                 errSht = 0;
            }
        }
    }
}

void AppController::publishAllConfigs() {
    statusPublisher.publishHardwareConfig();
    statusPublisher.publishSystemConfig();
    statusPublisher.publishSensorConfig();
    if (logger) logger->debug("Published device configurations");
}

void AppController::handleSC16CO() {
    unsigned long now = millis();
    if (now - lastCoReadTime >= sensorConfig.coInterval) {
        lastCoReadTime = now;
        int co = sensorReader.readCO();
        
        if (co >= 0) {
            lastCoValue = co;
            statusCo = "ok";
            errCo = 0;
            network.publishValue("/co", co);
            if (logger) {
                char msg[48]; snprintf(msg, sizeof(msg), "📤 CO: %d ppm", co);
                logger->info(msg);
            }
        } else if (co == -1) {
            // No data available - sensor might not be connected
            statusCo = "missing";
            errCo++;
            if (errCo > 10) {
                if (logger) logger->warn("⚠️ SC16-CO no data: sensor missing?");
                sensorReader.resetCOBuffer();
                errCo = 0;
            }
        } else {
            // Error reading
            statusCo = "error";
            errCo++;
            if (errCo > 5) {
                if (logger) logger->warn("⚠️ SC16-CO self-healing: Resetting buffer...");
                sensorReader.resetCOBuffer();
                errCo = 0;
            }
        }
    }
}
