/**
 * @file main.cpp
 * @brief Main entry point for the ESP32 firmware.
 *
 * Provides a readable, high-level sequence of initialization 
 * and loop steps, delegating implementation to AppController.
 */

#include <Arduino.h>
#include "AppController.h"

// The main application controller instance
AppController app;

void setup() {
    app.initHardware();
    app.initNetwork();
    app.setupLogger();
    app.initSensors();
}

void loop() {
    // 1. Core Tasks
    app.updateNetwork(); // Maintain WiFi & MQTT
    app.updateOta();     // Handle Over-The-Air updates

    // 2. Sensor & System Updates (only if network is ready)
    if (app.isNetworkReady()) {
        app.handleMHZ14A();       // Measure CO2 (PPM)
        app.handleDHT22();        // Measure Temperature & Humidity
        app.handleSGP40();        // Measure VOC Index
        app.handleSGP30();        // Measure eCO2/TVOC (SGP30)
        app.handleSHT3x();        // Measure Temp/Hum (SHT3x)
        app.handleSPS30();        // Measure PM (SPS30)
        app.handleBMP280();       // Measure Pressure & Temperature
        app.handleSC16CO();       // Measure CO (Carbon Monoxide)
        app.handleSystemStatus(); // Publish System Info (RSSI, Uptime)
    }
}
