#ifndef STATUS_PUBLISHER_H
#define STATUS_PUBLISHER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <DHT_U.h>
#include "NetworkManager.h"
#include "SensorData.h"

class StatusPublisher {
public:
    StatusPublisher(NetworkManager& network, HardwareSerial& co2Serial, DHT_Unified& dht);
    
    void publishSystemInfo();
    void publishSystemConfig();
    void publishSensorStatus(int lastCO2Value, const String& statusCo2, 
                            int lastCoValue, const String& statusCo,
                            float lastTemperature, float lastHumidity, const String& statusDht, 
                            int lastVocValue, const String& statusVoc,
                            float lastPressure, const String& statusPressure, float lastTempBmp, const String& statusTempBmp,
                            float lastPm1, float lastPm25, float lastPm4, float lastPm10, const String& statusPm,
                            int lastEco2, const String& statusEco2, int lastTvoc, const String& statusTvoc,
                            float lastTempSht, const String& statusTempSht, float lastHumSht, const String& statusHumSht);
    void publishSensorConfig();
    void publishHardwareConfig();
    
private:
    NetworkManager& network;
    DHT_Unified& dht;
    
    String buildSystemJson(const SystemInfo& sysInfo, const String& psramStr);
    String buildSensorStatusJson(int lastCO2Value, const String& statusCo2, 
                                int lastCoValue, const String& statusCo,
                                float lastTemperature, float lastHumidity, const String& statusDht, 
                                int lastVocValue, const String& statusVoc,
                                float lastPressure, const String& statusPressure, float lastTempBmp, const String& statusTempBmp,
                                float lastPm1, float lastPm25, float lastPm4, float lastPm10, const String& statusPm,
                                int lastEco2, const String& statusEco2, int lastTvoc, const String& statusTvoc,
                                float lastTempSht, const String& statusTempSht, float lastHumSht, const String& statusHumSht);
};

#endif
