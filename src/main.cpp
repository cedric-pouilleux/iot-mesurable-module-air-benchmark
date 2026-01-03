/**
 * @file main.cpp
 * @brief ESP32 Air Quality Monitor - Refactored with iot-mesurable
 *
 * Uses the IotMesurable library for MQTT, WiFi, status publishing,
 * and enable/disable features. SensorReader handles hardware communication.
 */

#include <Arduino.h>
#include <Wire.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <IotMesurable.h>
#include "SensorReader.h"
#include "secrets.h"

// Priority logic: if a broker is defined via build flags (-D MQTT_HUB_IP), use it.
// This avoids conflicts with MQTT_SERVER defined in secrets.h
#ifdef MQTT_HUB_IP
  #define REAL_MQTT_SERVER MQTT_HUB_IP
#else
  #define REAL_MQTT_SERVER MQTT_SERVER
#endif

// ============================================================================
// Hardware Configuration
// ============================================================================

#include "pins.h"  // Include pin definitions

// ============================================================================
// Hardware Configuration
// ============================================================================

#define DHT_TYPE DHT22
#define MODULE_ID "module-esp32-1"
#define MODULE_ID "module-esp32-1"

// ============================================================================
// Global Objects
// ============================================================================

// I2C buses
TwoWire wireSGP = TwoWire(1);

// Serial connections
HardwareSerial co2Serial(2);   // UART2 for MH-Z14A
HardwareSerial sps30Serial(1); // UART1 for SPS30
SoftwareSerial coSerial(PIN_UART_RX_CO_SC16, PIN_UART_TX_CO_SC16);

// DHT sensor
DHT_Unified dht(PIN_DHT, DHT_TYPE);

// Sensor reader (low-level hardware access)
SensorReader sensors(co2Serial, sps30Serial, dht, wireSGP, coSerial);

// IotMesurable - handles MQTT, WiFi, status, enable/disable
IotMesurable brain(MODULE_ID);

// ============================================================================
// Timing
// ============================================================================

// Read sensors every 5 seconds, automatic throttling controls publish rate
unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 5000;

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Air Quality Monitor (iot-mesurable) ===\n");
    
    // Initialize I2C
    Wire.begin(PIN_I2C_SDA_MAIN, PIN_I2C_SCL_MAIN);
    wireSGP.begin(PIN_I2C_SDA_SGP, PIN_I2C_SCL_SGP); // SGP bus
    
    // Initialize serial ports
    co2Serial.begin(9600, SERIAL_8N1, PIN_UART_RX_CO2, PIN_UART_TX_CO2);  // MH-Z14A
    sps30Serial.begin(115200, SERIAL_8N1, PIN_UART_RX_SPS30, PIN_UART_TX_SPS30); // SPS30
    coSerial.begin(9600);
    
    // Initialize brain (WiFi + MQTT)
    Serial.print("Connecting WiFi/MQTT...");
    brain.setBroker(REAL_MQTT_SERVER, 1883);
    if (!brain.begin(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println(" FAILED!");
    } else {
        Serial.println(" OK");
    }
    
    // Register all hardware and sensors
    brain.setModuleType("air-quality-bench");
    brain.registerHardware("mhz14a", "MH-Z14A CO2 Sensor");
    brain.addSensor("mhz14a", "co2");
    
    brain.registerHardware("dht22", "DHT22 Temp/Humidity");
    brain.addSensor("dht22", "temperature");
    brain.addSensor("dht22", "humidity");
    
    brain.registerHardware("sgp40", "SGP40 VOC Sensor");
    brain.addSensor("sgp40", "voc");
    
    brain.registerHardware("sgp30", "SGP30 eCO2/TVOC");
    brain.addSensor("sgp30", "eco2");
    brain.addSensor("sgp30", "tvoc");
    
    brain.registerHardware("sps30", "SPS30 Particulate");
    brain.addSensor("sps30", "pm1");
    brain.addSensor("sps30", "pm25");
    brain.addSensor("sps30", "pm4");
    brain.addSensor("sps30", "pm10");
    
    brain.registerHardware("bmp280", "BMP280 Pressure");
    brain.addSensor("bmp280", "pressure");
    brain.addSensor("bmp280", "temperature");
    
    brain.registerHardware("sht40", "SHT40 Temp/Humidity");
    brain.addSensor("sht40", "temperature");
    brain.addSensor("sht40", "humidity");
    
    brain.registerHardware("sc16co", "SC16-CO Carbon Monoxide");
    brain.addSensor("sc16co", "co");
    
    // Callbacks for debugging (throttling is automatic, no manual interval management needed)

    brain.onConnect([](bool connected) {
        Serial.printf("[MQTT] %s\n", connected ? "Connected" : "Disconnected");
    });
    
    brain.onResetChange([](const char* hw) {
        Serial.printf("[Reset] Request for: %s\n", hw);
        brain.log("info", "Reset request received");
        
        bool success = false;
        
        if (strcmp(hw, "mhz14a") == 0) {
            // MH-Z14A doesn't have a specific init, but we can try to re-setup serial or similar if needed.
            // For now, re-init isn't explicitly exposed in SensorReader for CO2, 
            // but we can add resetCO2 if we want to be thorough.
            // Let's rely on what's available. 
            // SensorReader has resetCO2() (void).
            sensors.resetCO2();
            success = true;
        }
        else if (strcmp(hw, "dht22") == 0) {
           sensors.resetDHT();
           success = true;
        }
        else if (strcmp(hw, "sgp40") == 0) {
            success = sensors.initSGP();
        }
        else if (strcmp(hw, "sgp30") == 0) {
            success = sensors.initSGP30();
        }
        else if (strcmp(hw, "sps30") == 0) {
            success = sensors.initSPS30();
        }
        else if (strcmp(hw, "bmp280") == 0) {
            success = sensors.initBMP();
        }
        else if (strcmp(hw, "sht40") == 0) {
            success = sensors.initSHT();
        }
        else if (strcmp(hw, "sc16co") == 0) {
            success = sensors.initCO();
        }
        else {
             char msg[64];
             snprintf(msg, sizeof(msg), "Unknown hardware: %s", hw);
             brain.log("warn", msg);
             return;
        }
        
        if (success) {
            char msg[64];
            snprintf(msg, sizeof(msg), "Hardware reset: %s", hw);
            brain.log("success", msg);
        } else {
             char msg[64];
             snprintf(msg, sizeof(msg), "Reset failed: %s", hw);
            brain.log("error", msg);
        }
    });
    
    // Initialize sensors
    Serial.println("Initializing sensors...");
    dht.begin();
    if (sensors.initBMP()) Serial.println(" - BMP280 OK");
    if (sensors.initSGP()) Serial.println(" - SGP40 OK");
    if (sensors.initSGP30()) Serial.println(" - SGP30 OK");
    if (sensors.initSPS30()) Serial.println(" - SPS30 OK");
    if (sensors.initSHT()) Serial.println(" - SHT40 OK");
    if (sensors.initCO()) Serial.println(" - SC16-CO OK");
    
    brain.log("info", "Module booted and connected");
    Serial.println("Setup complete!");
}



// ============================================================================
// Logging Helper
// ============================================================================

void logError(const char* msg) {
    // Log to Serial
    Serial.printf("[ERROR] %s\n", msg);
    
    // Log to MQTT via brain
    brain.log("error", msg);
}

// ============================================================================
// Loop
// ============================================================================

void loop() {
    brain.loop();
    
    unsigned long now = millis();
    
    // Read all sensors every READ_INTERVAL, throttling controls publish rate
    if (now - lastRead >= READ_INTERVAL) {
        lastRead = now;
        Serial.printf("[DEBUG] Reading sensors at %lu ms\n", now);
        
        // MH-Z14A (CO2)
        if (brain.isHardwareEnabled("mhz14a")) {
            int co2 = sensors.readCO2();
            if (co2 > 0) {
                Serial.printf("[PUBLISH] mhz14a CO2=%d\n", co2);
                brain.publish("mhz14a", "co2", co2);
            }
        }
        
        // DHT22 (Temp/Humidity)
        if (brain.isHardwareEnabled("dht22")) {
            DhtReading reading = sensors.readDhtSensors();
            if (reading.valid) {
                brain.publish("dht22", "temperature", reading.temperature);
                brain.publish("dht22", "humidity", reading.humidity);
            }
        }
        
        // SGP40 (VOC)
        if (brain.isHardwareEnabled("sgp40")) {
            int voc = sensors.readVocIndex();
            if (voc >= 0) {
                brain.publish("sgp40", "voc", voc);
            }
        }
        
        // SGP30 (eCO2/TVOC)
        if (brain.isHardwareEnabled("sgp30")) {
            int eco2, tvoc;
            if (sensors.readSGP30(eco2, tvoc)) {
                brain.publish("sgp30", "eco2", eco2);
                brain.publish("sgp30", "tvoc", tvoc);
            }
        }
        
        // SPS30 (PM)
        if (brain.isHardwareEnabled("sps30")) {
            float pm1, pm25, pm4, pm10;
            if (sensors.readSPS30(pm1, pm25, pm4, pm10)) {
                brain.publish("sps30", "pm1", pm1);
                brain.publish("sps30", "pm25", pm25);
                brain.publish("sps30", "pm4", pm4);
                brain.publish("sps30", "pm10", pm10);
            }
        }
        
        // BMP280 (Pressure/Temp)
        if (brain.isHardwareEnabled("bmp280")) {
            float pressure = sensors.readPressure();
            float temp = sensors.readBMPTemperature();
            
            if (!isnan(pressure)) {
                brain.publish("bmp280", "pressure", pressure);
            }
            if (!isnan(temp)) {
                brain.publish("bmp280", "temperature", temp);
            }
        }
        
        // SHT40 (Temp/Humidity)
        if (brain.isHardwareEnabled("sht40")) {
            float temp, hum;
            if (sensors.readSHT(temp, hum)) {
                brain.publish("sht40", "temperature", temp);
                brain.publish("sht40", "humidity", hum);
            }
        }
        
        // SC16-CO (Carbon Monoxide)
        if (brain.isHardwareEnabled("sc16co")) {
            int co = sensors.readCO();
            if (co >= 0) {
                brain.publish("sc16co", "co", co);
            }
        }
    }
}
