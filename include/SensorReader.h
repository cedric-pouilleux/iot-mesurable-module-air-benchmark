#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include <DHT_U.h>
#include <Adafruit_SGP40.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_SHT31.h>
#include <SensirionUartSps30.h>
#include <SoftwareSerial.h>
#include "SensorData.h"

class RemoteLogger; // Forward declaration

/**
 * @brief Handles communication with all connected sensors (BMP280, SGP40, SGP30, DHT, CO2, CO).
 * 
 * Responsible for:
 * - Initialization and re-initialization (reset)
 * - Raw data reading
 * - I2C bus recovery
 * - Remote logging of sensor status
 */
class SensorReader {
public:
    // SGP40 and SGP30 will use the second I2C bus (wireSGP)
    SensorReader(HardwareSerial& co2Serial, HardwareSerial& sps30Serial, DHT_Unified& dht, TwoWire& wireSGP, SoftwareSerial& coSerial);
    
    /**
     * @brief Injects the logger instance for remote reporting.
     */
    void setLogger(RemoteLogger* logger);

    /**
     * @brief Initializes the BMP280 sensor (Pressure/Temp).
     * @param maxAttempts Number of retries before failing.
     * @return true if successful, false otherwise.
     */
    bool initBMP(int maxAttempts = 3, int delayBetweenMs = 100);

    /**
     * @brief Initializes the SGP40 sensor (VOC).
     * @return true if successful, false otherwise.
     */
    bool initSGP(int maxAttempts = 3, int delayBetweenMs = 100);

    /**
     * @brief Initializes the SGP30 sensor (eCO2/TVOC).
     * @return true if successful, false otherwise.
     */
    bool initSGP30(int maxAttempts = 3, int delayBetweenMs = 100);

    /**
     * @brief Initializes the SPS30 sensor (PM) via UART.
     * @return true if successful, false otherwise.
     */
    bool initSPS30(int maxAttempts = 3, int delayBetweenMs = 100);

    /**
     * @brief Scans an I2C bus and logs found devices.
     * @return Number of devices found.
     */
    int scanI2C(TwoWire& wire, const char* busName);

    /**
     * @brief Reads CO2 concentration from MH-Z19/14A sensor via Serial.
     * @return CO2 ppm value, or negative error code (-1: timeout, -2: header error, -3: range error).
     */
    int readCO2();

    /**
     * @brief Checks if SGP40 is reachable on the I2C bus.
     */
    bool isSGPConnected();
    
    /**
     * @brief Checks if SGP30 is reachable on the I2C bus.
     */
    bool isSGP30Connected();

    /**
     * @brief Checks if BMP280 is reachable on the I2C bus.
     */
    bool isBMPConnected();

    /**
     * @brief Reads Voc Index from SGP40.
     * Returns -1 if sensor is disconnected.
     * Use DHT measurements for compensation if available.
     * @return VOC Index (0-500), or -1 on error.
     */
    int readVocIndex();

    /**
     * @brief Reads eCO2 and TVOC from SGP30.
     * @param eco2 Reference to store eCO2 value (ppm)
     * @param tvoc Reference to store TVOC value (ppb)
     * @return true if read successful, false otherwise
     */
    bool readSGP30(int& eco2, int& tvoc);

    /**
     * @brief Reads Pressure from BMP280.
     * Returns NAN if sensor is disconnected.
     */
    float readPressure();

    /**
     * @brief Reads Temperature from BMP280.
     * Returns NAN if sensor is disconnected.
     */
    float readBMPTemperature();

    /**
     * @brief Resets the BMP280 sensor including I2C bus recovery.
     */
    bool resetBMP();

    /**
     * @brief Resets the SGP40 sensor.
     */
    bool resetSGP();

    void resetDHT();
    void resetCO2();
    DhtReading readDhtSensors();

    /**
     * @brief Manual I2C bus recovery routine.
     * Bit-bangs SCL to drain any stuck slave holding SDA low.
     * @param sdaPin GPIO pin for SDA (default: 21)
     * @param sclPin GPIO pin for SCL (default: 22)
     */
    void recoverI2C(int sdaPin = 21, int sclPin = 22);

    /**
     * @brief Reads PM values from SPS30.
     * @param pm1 Reference to store PM1.0
     * @param pm25 Reference to store PM2.5
     * @param pm4 Reference to store PM4.0
     * @param pm10 Reference to store PM10
     * @return true if read successful, false otherwise
     */
    bool readSPS30(float& pm1, float& pm25, float& pm4, float& pm10);

    /**
     * @brief Initializes the SHT3x sensor (Temp/Hum).
     * @return true if successful, false otherwise.
     */
    bool initSHT(int maxAttempts = 3, int delayBetweenMs = 100);

    /**
     * @brief Checks if SHT3x is reachable on the I2C bus.
     */
    bool isSHTConnected();

    /**
     * @brief Reads Temp/Hum from SHT3x.
     * @param temp Reference to store Temperature
     * @param hum Reference to store Humidity
     * @return true if read successful, false otherwise
     */
    bool readSHT(float& temp, float& hum);

    /**
     * @brief Resets the SHT3x sensor.
     */
    void resetSHT();

    // ============ SC16-CO (Carbon Monoxide) ============
    
    /**
     * @brief Initializes the SC16-CO sensor (SoftwareSerial).
     * @return true if successful, false otherwise.
     */
    bool initCO();

    /**
     * @brief Reads CO concentration from SC16-CO sensor.
     * The sensor auto-uploads data every ~1 second.
     * @return CO ppm value, or -1 on error/no data.
     */
    int readCO();

    /**
     * @brief Clears the CO serial buffer.
     */
    void resetCOBuffer();
    
private:
    RemoteLogger* _logger = nullptr;
    HardwareSerial& co2Serial;
    HardwareSerial& sps30Serial;
    SoftwareSerial& _coSerial;
    DHT_Unified& dht;
    TwoWire& _wireSGP;
    Adafruit_SGP40 sgp;
    Adafruit_SGP30 sgp30;
    Adafruit_SHT31 sht;
    Adafruit_BMP280 bmp;
    SensirionUartSps30 sps30;
    static const uint8_t CO2_READ_CMD[9];
    
    // SC16-CO buffer for parsing auto-upload frames
    uint8_t _coBuffer[9];
    int _coBufferIndex = 0;
};

#endif
