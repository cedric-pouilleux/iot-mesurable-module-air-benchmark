#include "StatusPublisher.h"
#include "SystemInfoCollector.h"
#include "esp_ota_ops.h"
#include <WiFi.h>

StatusPublisher::StatusPublisher(NetworkManager& network, HardwareSerial& co2Serial, DHT_Unified& dht) 
    : network(network), dht(dht) {
}

String StatusPublisher::buildSystemJson(const SystemInfo& sysInfo, const String& psramStr) {
    char systemMsg[512];
    snprintf(systemMsg, sizeof(systemMsg), 
        "{"
            "\"rssi\":%ld,"
            "\"memory\":{"
                "\"heapFreeKb\":%d,"
                "\"heapMinFreeKb\":%d"
            "%s"
            "}"
        "}",
        network.getRSSI(),
        sysInfo.heapFree,
        sysInfo.heapMinFree,
        psramStr.c_str()
    );
    return String(systemMsg);
}

void StatusPublisher::publishSystemConfig() {
    if (!network.isConnected()) {
        return;
    }

    IPAddress currentIP = network.getIP();
    String ipStr = currentIP.toString();
    String macStr = WiFi.macAddress();
    SystemInfo sysInfo = SystemInfoCollector::collect();
    String psramStr = SystemInfoCollector::buildPsramJson();

    char systemConfigMsg[768];
    snprintf(systemConfigMsg, sizeof(systemConfigMsg), 
        "{"
            "\"moduleType\":\"air-quality-bench\","
            "\"ip\":\"%s\","
            "\"mac\":\"%s\","
            "\"uptimeStart\":%lu,"
            "\"flash\":{"
                "\"usedKb\":%d,"
                "\"freeKb\":%d,"
                "\"systemKb\":%d"
            "},"
            "\"memory\":{"
                "\"heapTotalKb\":%d"
            "%s"
            "}"
        "}",
        ipStr.c_str(),
        macStr.c_str(),
        millis() / 1000,
        sysInfo.flashUsed,
        sysInfo.flashFree,
        sysInfo.flashSystemPartitions,
        sysInfo.heapTotal,
        psramStr.c_str()
    );
    
    network.publishMessage("/system/config", systemConfigMsg, true);
}

void StatusPublisher::publishHardwareConfig() {
    if (!network.isConnected()) {
        return;
    }

    SystemInfo sysInfo = SystemInfoCollector::collect();
    String psramStr = SystemInfoCollector::buildPsramJson();
    
    char hardwareConfigMsg[512];
    snprintf(hardwareConfigMsg, sizeof(hardwareConfigMsg), 
        "{"
            "\"chip\":{"
                "\"model\":\"%s\","
                "\"rev\":%d,"
                "\"flashKb\":%d,"
                "\"cpuFreqMhz\":%d"
            "}"
        "}",
        ESP.getChipModel(),
        ESP.getChipRevision(),
        sysInfo.flashTotal,
        ESP.getCpuFreqMHz()
    );
    
    network.publishMessage("/hardware/config", hardwareConfigMsg, true);
}

String StatusPublisher::buildSensorStatusJson(int lastCO2Value, const String& statusCo2, float lastTemperature, 
                                              float lastHumidity, const String& statusDht, int lastVocValue, const String& statusVoc,
                                              float lastPressure, const String& statusPressure, float lastTempBmp, const String& statusTempBmp,
                                              float lastPm1, float lastPm25, float lastPm4, float lastPm10, const String& statusPm,
                                              int lastEco2, const String& statusEco2, int lastTvoc, const String& statusTvoc,
                                              float lastTempSht, const String& statusTempSht, float lastHumSht, const String& statusHumSht) {
    
    char tempStr[16];
    if (isnan(lastTemperature)) strcpy(tempStr, "null");
    else snprintf(tempStr, sizeof(tempStr), "%.1f", lastTemperature);

    char humStr[16];
    if (isnan(lastHumidity)) strcpy(humStr, "null");
    else snprintf(humStr, sizeof(humStr), "%.1f", lastHumidity);

    char pressStr[16];
    if (isnan(lastPressure)) strcpy(pressStr, "null");
    else snprintf(pressStr, sizeof(pressStr), "%.1f", lastPressure);

    char tempBmpStr[16];
    if (isnan(lastTempBmp)) strcpy(tempBmpStr, "null");
    else snprintf(tempBmpStr, sizeof(tempBmpStr), "%.1f", lastTempBmp);

    char pm1Str[16], pm25Str[16], pm4Str[16], pm10Str[16];
    // No isnan for float PM values if they are 0.0 initialized, but assuming sensorReader returns valid floats.
    // If we want null, we need to key off statusPm.
    if (statusPm == "missing" || statusPm == "error" || statusPm == "init") {
        strcpy(pm1Str, "null");
        strcpy(pm25Str, "null");
        strcpy(pm4Str, "null");
        strcpy(pm10Str, "null");
    } else {
        snprintf(pm1Str, sizeof(pm1Str), "%.1f", lastPm1);
        snprintf(pm25Str, sizeof(pm25Str), "%.1f", lastPm25);
        snprintf(pm4Str, sizeof(pm4Str), "%.1f", lastPm4);
        snprintf(pm10Str, sizeof(pm10Str), "%.1f", lastPm10);
    }

    char tempShtStr[16];
    if (isnan(lastTempSht)) strcpy(tempShtStr, "null");
    else snprintf(tempShtStr, sizeof(tempShtStr), "%.1f", lastTempSht);

    char humShtStr[16];
    if (isnan(lastHumSht)) strcpy(humShtStr, "null");
    else snprintf(humShtStr, sizeof(humShtStr), "%.1f", lastHumSht);

    char sensorStatusMsg[1280]; // Increased buffer size to be safe
    snprintf(sensorStatusMsg, sizeof(sensorStatusMsg), 
        "{"
            "\"co2\":{"
                "\"status\":\"%s\","
                "\"value\":%d"
            "},"
            "\"temperature\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"humidity\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"pm1\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"pm25\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"pm4\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"pm10\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"voc\":{"
                "\"status\":\"%s\","
                "\"value\":%d"
            "},"
            "\"eco2\":{"
                "\"status\":\"%s\","
                "\"value\":%d"
            "},"
            "\"tvoc\":{"
                "\"status\":\"%s\","
                "\"value\":%d"
            "},"
            "\"pressure\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"temperature_bmp\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"temp_sht\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "},"
            "\"hum_sht\":{"
                "\"status\":\"%s\","
                "\"value\":%s"
            "}"
        "}",
        statusCo2.c_str(),
        lastCO2Value,
        statusDht.c_str(),
        tempStr,
        statusDht.c_str(),
        humStr,
        statusPm.c_str(), pm1Str, // PM1
        statusPm.c_str(), pm25Str, // PM2.5
        statusPm.c_str(), pm4Str, // PM4
        statusPm.c_str(), pm10Str, // PM10
        statusVoc.c_str(), 
        lastVocValue,
        statusEco2.c_str(), lastEco2,
        statusTvoc.c_str(), lastTvoc,
        statusPressure.c_str(),
        pressStr,
        statusTempBmp.c_str(),
        tempBmpStr,
        statusTempSht.c_str(),
        tempShtStr,
        statusHumSht.c_str(),
        humShtStr
    );
    return String(sensorStatusMsg);
}

void StatusPublisher::publishSensorConfig() {
    if (!network.isConnected()) {
        return;
    }
    
    // Publish only sensor models (without intervals which are managed by the backend)
    char sensorConfigMsg[512];
    snprintf(sensorConfigMsg, sizeof(sensorConfigMsg), 
        "{"
            "\"co2\":{\"model\":\"MH-Z14A\"},"
            "\"temperature\":{\"model\":\"DHT22\"},"
            "\"humidity\":{\"model\":\"DHT22\"},"
            "\"pm1\":{\"model\":\"SPS30\"},"
            "\"pm25\":{\"model\":\"SPS30\"},"
            "\"pm4\":{\"model\":\"SPS30\"},"
            "\"pm10\":{\"model\":\"SPS30\"},"
            "\"voc\":{\"model\":\"SGP40\"},"
            "\"eco2\":{\"model\":\"SGP30\"},"
            "\"tvoc\":{\"model\":\"SGP30\"},"
            "\"pressure\":{\"model\":\"BMP280\"},"
            "\"temperature_bmp\":{\"model\":\"BMP280\"},"
            "\"temp_sht\":{\"model\":\"SHT31\"},"
            "\"hum_sht\":{\"model\":\"SHT31\"}"
        "}"
    );
    
    // Publish with retained flag so the backend retrieves models on startup
    network.publishMessage("/sensors/config", sensorConfigMsg, true);
}

void StatusPublisher::publishSystemInfo() {
    if (!network.isConnected()) {
        return;
    }

    SystemInfo sysInfo = SystemInfoCollector::collect();
    String psramStr = SystemInfoCollector::buildPsramJson();
    String systemMsg = buildSystemJson(sysInfo, psramStr);
    
    // Do not use retained for dynamic data (rssi, memory change often)
    network.publishMessage("/system", systemMsg.c_str(), false);
}

void StatusPublisher::publishSensorStatus(int lastCO2Value, const String& statusCo2, float lastTemperature, 
                                          float lastHumidity, const String& statusDht, int lastVocValue, const String& statusVoc,
                                          float lastPressure, const String& statusPressure, float lastTempBmp, const String& statusTempBmp,
                                          float lastPm1, float lastPm25, float lastPm4, float lastPm10, const String& statusPm,
                                          int lastEco2, const String& statusEco2, int lastTvoc, const String& statusTvoc,
                                          float lastTempSht, const String& statusTempSht, float lastHumSht, const String& statusHumSht) {
    if (!network.isConnected()) {
        return;
    }

    String sensorStatusMsg = buildSensorStatusJson(lastCO2Value, statusCo2, lastTemperature, 
                                                   lastHumidity, statusDht, lastVocValue, statusVoc,
                                                   lastPressure, statusPressure, lastTempBmp, statusTempBmp,
                                                   lastPm1, lastPm25, lastPm4, lastPm10, statusPm,
                                                   lastEco2, statusEco2, lastTvoc, statusTvoc,
                                                   lastTempSht, statusTempSht, lastHumSht, statusHumSht);
    // Do not use retained for dynamic data (values change often)
    network.publishMessage("/sensors/status", sensorStatusMsg.c_str(), false);
}

