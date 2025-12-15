#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <DHT_U.h>
#include "NetworkManager.h"
#include "SensorReader.h"
#include "StatusPublisher.h"
#include "OtaManager.h"
#include "MqttHandler.h"
#include "RemoteLogger.h"
#include "SensorData.h"

class AppController {
public:
    AppController();

    void initHardware();
    void initNetwork();
    void setupLogger();
    void initSensors();
    
    // Core loop tasks
    void updateNetwork();
    void updateOta();
    
    // Granular Sensor Handlers (Hardware specific)
    void handleMHZ14A();   // CO2 Sensor
    void handleDHT22();    // Temp/Humidity Sensor
    void handleSGP40();    // VOC Sensor
    void handleSPS30();    // PM Sensor
    void handleSGP30();    // eCO2/TVOC Sensor (SGP30)
    void handleSHT3x();    // Temp/Hum Sensor (SHT3x)
    void handleBMP280();   // Pressure/Temp Sensor
    void handleSC16CO();   // CO Sensor (SC16-CO)
    void handleSystemStatus(); // System Info
    
    bool isNetworkReady();
    
    // Helpers
    void handleMqttMessage(char* topic, byte* payload, unsigned int length);
    void onMqttConnected();
    
    // Helpers
    NetworkManager& getNetwork() { return network; }

private:
    // Config
    SensorConfig sensorConfig;

    // Hardware Interface
    HardwareSerial co2Serial;
    HardwareSerial sps30Serial;
    SoftwareSerial coSerial;  // SC16-CO (Carbon Monoxide)
    DHT_Unified dht;

    // Subsystems
    NetworkManager network;
    OtaManager ota;
    SensorReader sensorReader;
    StatusPublisher statusPublisher;
    MqttHandler mqttHandler;
    
    // Logging (pointer as it relies on network topic)
    RemoteLogger* logger;

    // State / Timers
    unsigned long lastCo2ReadTime = 0;
    unsigned long lastTempReadTime = 0;
    unsigned long lastHumReadTime = 0;
    unsigned long lastVocReadTime = 0;
    unsigned long lastSgp30ReadTime = 0;
    unsigned long lastSgp30PublishTime = 0;
    unsigned long lastPmReadTime = 0;
    unsigned long lastShtReadTime = 0;
    unsigned long lastPressureReadTime = 0;
    unsigned long lastCoReadTime = 0;  // SC16-CO
    unsigned long lastSystemInfoTime = 0;
    
    bool mqttJustConnected = false;
    unsigned int reconnectAttempts = 0;

    // Self-healing counters
    int errCo2 = 0;
    int errDht = 0;
    int errSgp40 = 0;
    int errSgp30 = 0;
    int errSps30 = 0;
    int errBmp = 0;
    int errSht = 0;
    int errCo = 0;  // SC16-CO

    // Last Values
    int lastCO2Value = 0;
    int lastVocValue = 0;
    int lastEco2Value = 0;
    int lastTvocValue = 0;
    int lastCoValue = 0;  // SC16-CO
    float lastTemperature = 0.0;
    float lastHumidity = 0.0;
    float lastPressure = 0.0;
    float lastTempBmp = 0.0;
    float lastPm1 = 0.0;
    float lastPm25 = 0.0;
    float lastPm4 = 0.0;
    float lastPm10 = 0.0;
    float lastTempSht = 0.0;
    float lastHumSht = 0.0;

    
    // Detailed Status ("ok", "warning", "error", "missing")
    String statusDht = "init";
    String statusCo2 = "init";
    String statusVoc = "init";
    String statusEco2 = "init";
    String statusTvoc = "init";
    String statusPressure = "init";
    String statusTempBmp = "init";
    String statusTempSht = "init";
    String statusHumSht = "init";
    String statusPm = "init";
    String statusCo = "init";  // SC16-CO

    void publishAllConfigs();
};

#endif
