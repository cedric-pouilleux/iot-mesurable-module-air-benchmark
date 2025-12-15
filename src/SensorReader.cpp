#include "SensorReader.h"
#include <Wire.h>
#include "RemoteLogger.h"

const uint8_t SensorReader::CO2_READ_CMD[9] = { 0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79 };

SensorReader::SensorReader(HardwareSerial& co2Serial, HardwareSerial& sps30Serial, DHT_Unified& dht, TwoWire& wireSGP, SoftwareSerial& coSerial) 
    : co2Serial(co2Serial), sps30Serial(sps30Serial), _coSerial(coSerial), dht(dht), _wireSGP(wireSGP), sht(&wireSGP) {
}

void SensorReader::setLogger(RemoteLogger* logger) {
    _logger = logger;
}

bool SensorReader::initBMP(int maxAttempts, int delayBetweenMs) {
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        if (bmp.begin(0x76)) {
            if (_logger) {
                if (attempt > 1) _logger->warn("BMP280 initialized after " + String(attempt) + " attempts");
                else _logger->success("BMP280 initialized successfully");
            }
            return true;
        }
        if (attempt < maxAttempts) delay(delayBetweenMs);
    }
    if (_logger) _logger->error("BMP280 not found");
    return false;
}

bool SensorReader::initSGP(int maxAttempts, int delayBetweenMs) {
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        if (sgp.begin(&_wireSGP)) {
            if (_logger) {
                if (attempt > 1) _logger->warn("SGP40 initialized after " + String(attempt) + " attempts");
                else _logger->success("SGP40 initialized successfully");
            }
            return true;
        }
        if (attempt < maxAttempts) delay(delayBetweenMs);
    }
    if (_logger) _logger->error("SGP40 not found");
    return false;
}

bool SensorReader::initSGP30(int maxAttempts, int delayBetweenMs) {
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        if (sgp30.begin(&_wireSGP)) {
            if (_logger) {
                if (attempt > 1) _logger->warn("SGP30 initialized after " + String(attempt) + " attempts");
                else _logger->success("SGP30 initialized successfully");
            }
            return true;
        }
        if (attempt < maxAttempts) delay(delayBetweenMs);
    }
    if (_logger) _logger->error("SGP30 not found");
    return false;
}

bool SensorReader::initSPS30(int maxAttempts, int delayBetweenMs) {
    sps30Serial.begin(115200, SERIAL_8N1, 13, 27);
    sps30.begin(sps30Serial);

    for (int attempts = 0; attempts < maxAttempts; attempts++) {
        sps30.wakeUp(); 
        sps30.stopMeasurement(); 
        delay(100);
        sps30.deviceReset();
        delay(1000);

        int8_t serialNumber[32];
        int16_t ret = sps30.readSerialNumber(serialNumber, 32);
        
        if (ret == 0) {
            if (_logger) _logger->success("SPS30 detected: " + String((char*)serialNumber));
            
            ret = sps30.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_FLOAT);
            if (ret == 0 || ret == 1347 || (ret & 0xFF00) == 0x0500) {
                delay(2000);
                float p1, p2, p4, p10;
                if (readSPS30(p1, p2, p4, p10)) {
                    if (_logger) _logger->success("SPS30 ready");
                    return true;
                }
            }
        }
        delay(delayBetweenMs);
    }
    
    if (_logger) _logger->error("SPS30 not found");
    return false;
}

int SensorReader::scanI2C(TwoWire& wire, const char* busName) {
    if (_logger) _logger->info(String("Scanning I2C bus: ") + busName + "...");
    
    int nDevices = 0;
    for (byte address = 1; address < 127; address++) {
        wire.beginTransmission(address);
        if (wire.endTransmission() == 0) {
            char msg[48];
            snprintf(msg, sizeof(msg), "I2C found: 0x%02X on %s", address, busName);
            if (_logger) _logger->success(msg);
            nDevices++;
        }
    }
    
    if (nDevices == 0 && _logger) {
        _logger->warn(String("No I2C devices found on ") + busName);
    } else if (_logger) {
        _logger->info("Scan complete on " + String(busName) + ", devices: " + String(nDevices));
    }
    return nDevices;
}

bool SensorReader::readSPS30(float& pm1, float& pm25, float& pm4, float& pm10) {
    float mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, typPartSize;
    
    for (int i = 0; i < 3; i++) {
        if (sps30.readMeasurementValuesFloat(mc1p0, mc2p5, mc4p0, mc10p0, 
                                            nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, typPartSize) == 0) {
            pm1 = mc1p0; pm25 = mc2p5; pm4 = mc4p0; pm10 = mc10p0;
            return true;
        }
        delay(100);
    }

    // Auto-recovery
    sps30.wakeUp();
    int16_t startRet = sps30.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_FLOAT);
    if (_logger) {
        if (startRet == 0) _logger->warn("SPS30 restarted (auto-recovery)");
        else _logger->error("SPS30 recovery failed: " + String(startRet));
    }
    return false;
}

bool SensorReader::resetBMP() {
    // Soft Reset
    Wire.beginTransmission(0x76);
    Wire.write(0xE0);
    Wire.write(0xB6);
    byte error = Wire.endTransmission();
    delay(100);

    if (error == 0 && bmp.begin(0x76)) {
        if (_logger) _logger->success("BMP280 soft reset OK");
        return true;
    }

    if (_logger) _logger->warn("Soft reset failed, I2C recovery...");
    recoverI2C(21, 22);
    
    Wire.beginTransmission(0x76);
    Wire.write(0xE0);
    Wire.write(0xB6);
    Wire.endTransmission();
    delay(100);
    
    if (bmp.begin(0x76)) {
        if (_logger) _logger->success("BMP280 reset OK (after I2C recovery)");
        return true;
    }
    
    if (_logger) _logger->error("BMP280 reset failed");
    return false;
}

bool SensorReader::resetSGP() {
    bool success = sgp.begin(&_wireSGP);
    if (!success) {
        if (_logger) _logger->error("SGP40 reset failed");
        recoverI2C(32, 33);
    } else {
        if (_logger) _logger->success("SGP40 reset OK");
    }
    return success;
}

void SensorReader::recoverI2C(int sdaPin, int sclPin) {
    if (_logger) _logger->warn("I2C recovery: SDA=" + String(sdaPin) + " SCL=" + String(sclPin));

    pinMode(sdaPin, INPUT);
    pinMode(sclPin, INPUT);
    delayMicroseconds(5);

    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, LOW);
    pinMode(sdaPin, INPUT);

    for (int i = 0; i < 9; i++) {
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(sclPin, LOW);
        delayMicroseconds(10);
    }

    // STOP condition
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(10);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(10);

    pinMode(sdaPin, INPUT);
    pinMode(sclPin, INPUT);
    delayMicroseconds(5);

    if (sdaPin == 21 && sclPin == 22) {
        Wire.begin(sdaPin, sclPin);
        Wire.setTimeOut(1000);
    }
    delay(100);
}

void SensorReader::resetDHT() {
    dht.begin();
    if (_logger) _logger->info("DHT reset");
}

void SensorReader::resetCO2() {
    while (co2Serial.available()) co2Serial.read();
    if (_logger) _logger->info("CO2 buffer cleared");
}

bool SensorReader::isSGPConnected() {
    _wireSGP.beginTransmission(0x59);
    return _wireSGP.endTransmission() == 0;
}

bool SensorReader::isSGP30Connected() {
    _wireSGP.beginTransmission(0x58);
    return _wireSGP.endTransmission() == 0;
}

bool SensorReader::isBMPConnected() {
    Wire.beginTransmission(0x76);
    return Wire.endTransmission() == 0;
}

bool SensorReader::readSGP30(int& eco2, int& tvoc) {
    if (!isSGP30Connected()) return false;
    if (!sgp30.IAQmeasure()) return false;
    eco2 = sgp30.eCO2;
    tvoc = sgp30.TVOC;
    return true;
}

int SensorReader::readVocIndex() {
    if (!isSGPConnected()) return -1;

    sensors_event_t temp, humidity;
    dht.temperature().getEvent(&temp);
    dht.humidity().getEvent(&humidity);
    
    float t = isnan(temp.temperature) ? 25.0 : temp.temperature;
    float h = isnan(humidity.relative_humidity) ? 50.0 : humidity.relative_humidity;
    
    return sgp.measureVocIndex(t, h);
}

float SensorReader::readPressure() {
    if (!isBMPConnected()) return NAN;
    return bmp.readPressure() / 100.0F;
}

float SensorReader::readBMPTemperature() {
    if (!isBMPConnected()) return NAN;
    return bmp.readTemperature();
}

int SensorReader::readCO2() {
    while (co2Serial.available()) co2Serial.read();

    co2Serial.write(CO2_READ_CMD, 9);
    unsigned long start = millis();
    while (co2Serial.available() < 9 && millis() - start < 500) {
        delay(10);
    }

    if (co2Serial.available() < 9) return -1;

    uint8_t buf[9];
    co2Serial.readBytes(buf, 9);

    if (buf[0] != 0xFF || buf[1] != 0x86) return -2;
    
    uint8_t checksum = 0;
    for (int i = 1; i < 8; i++) checksum += buf[i];
    checksum = 0xFF - checksum + 1;
    
    if (checksum != buf[8]) return -4;

    int ppm = buf[2] * 256 + buf[3];
    return (ppm >= 0 && ppm <= 10000) ? ppm : -3;
}

DhtReading SensorReader::readDhtSensors() {
    DhtReading reading = {0.0, 0.0, false};
    sensors_event_t event;

    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature)) {
        reading.temperature = event.temperature;
        reading.valid = true;
    }
    
    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity)) {
        reading.humidity = event.relative_humidity;
    }
    
    return reading;
}

bool SensorReader::initSHT(int maxAttempts, int delayBetweenMs) {
    _wireSGP.setClock(100000);
    _wireSGP.setTimeOut(150);

    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        if (sht.begin(0x44)) {
            sht.reset();
            delay(100);
            if (_logger) {
                if (attempt > 1) _logger->warn("SHT31 initialized after " + String(attempt) + " attempts");
                else _logger->success("SHT31 initialized successfully (100kHz)");
            }
            return true;
        }
        if (attempt < maxAttempts) delay(delayBetweenMs);
    }
    if (_logger) _logger->error("SHT31 not found");
    return false;
}

bool SensorReader::isSHTConnected() {
    _wireSGP.beginTransmission(0x44);
    return _wireSGP.endTransmission() == 0;
}

bool SensorReader::readSHT(float& temp, float& hum) {
    if (!isSHTConnected()) return false;
    
    _wireSGP.setClock(100000);

    for (int i = 0; i < 3; i++) {
        if (sht.readBoth(&temp, &hum)) {
            if (hum >= 0 && hum <= 100 && temp > -45 && temp < 130) {
                return true;
            }
        }
        if (i < 2) {
            sht.begin(0x44);
            delay(10);
        }
    }
    return false;
}

void SensorReader::resetSHT() {
    if (_logger) _logger->warn("Resetting SHT3x...");
    if (initSHT()) {
        if (_logger) _logger->success("SHT3x reset OK");
    } else {
        if (_logger) _logger->error("SHT3x reset failed");
        recoverI2C(32, 33);
    }
}

// ============ SC16-CO (Carbon Monoxide) ============

bool SensorReader::initCO() {
    // SoftwareSerial is already configured by AppController
    // Just clear the buffer and check if data is arriving
    _coBufferIndex = 0;
    memset(_coBuffer, 0, sizeof(_coBuffer));
    
    // Wait briefly and check if any data arrives (sensor auto-uploads)
    unsigned long start = millis();
    while (_coSerial.available() == 0 && millis() - start < 2000) {
        delay(100);
    }
    
    if (_coSerial.available() > 0) {
        // Sensor is responding
        while (_coSerial.available()) _coSerial.read(); // Clear buffer
        if (_logger) _logger->success("SC16-CO initialized successfully");
        return true;
    }
    
    if (_logger) _logger->warn("SC16-CO not detected (no auto-upload data)");
    return false;
}

int SensorReader::readCO() {
    // SC16-CO auto-uploads data every ~1 second
    // Frame format: 0xFF, 0x04, HIGH, LOW, FULL_HIGH, FULL_LOW, RESERVED, RESERVED, CHECKSUM
    // CO ppm = HIGH * 256 + LOW
    
    int bytesAvailable = _coSerial.available();
    if (bytesAvailable == 0) {
        return -1; // No data available
    }
    
    // Read all available bytes and look for valid frame
    while (_coSerial.available()) {
        uint8_t b = _coSerial.read();
        
        // Look for frame start (0xFF)
        if (_coBufferIndex == 0 && b != 0xFF) {
            continue; // Skip until we find start byte
        }
        
        _coBuffer[_coBufferIndex++] = b;
        
        // Check if we have a complete frame (9 bytes)
        if (_coBufferIndex >= 9) {
            // Validate frame: starts with 0xFF, 0x04 for CO concentration
            if (_coBuffer[0] == 0xFF && _coBuffer[1] == 0x04) {
                // Calculate checksum
                uint8_t checksum = 0;
                for (int i = 1; i < 8; i++) {
                    checksum += _coBuffer[i];
                }
                checksum = 0xFF - checksum + 1;
                
                if (checksum == _coBuffer[8]) {
                    // Valid frame - extract CO value
                    int co_ppm = _coBuffer[2] * 256 + _coBuffer[3];
                    _coBufferIndex = 0;
                    
                    // SC16-CO range is typically 0-1000 ppm
                    if (co_ppm >= 0 && co_ppm <= 1000) {
                        return co_ppm;
                    } else {
                        return -3; // Out of range
                    }
                }
            }
            
            // Invalid frame, reset and continue
            _coBufferIndex = 0;
        }
    }
    
    return -2; // Incomplete frame
}

void SensorReader::resetCOBuffer() {
    while (_coSerial.available()) _coSerial.read();
    _coBufferIndex = 0;
    if (_logger) _logger->info("CO buffer cleared");
}
