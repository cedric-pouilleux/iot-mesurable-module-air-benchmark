#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

struct DhtReading {
    float temperature;
    float humidity;
    bool valid;
};

struct SystemInfo {
    int flashTotal;
    int flashUsed;
    int flashFree;
    int flashSystemPartitions;
    int heapTotal;
    int heapFree;
    int heapMinFree;
    float heapUsedPercent;
};

struct SensorConfig {
    unsigned long co2Interval = 60000;
    unsigned long tempInterval = 60000;
    unsigned long humInterval = 60000;
    unsigned long vocInterval = 60000;
    unsigned long pressureInterval = 60000;
    unsigned long pmInterval = 60000;
    unsigned long eco2Interval = 60000;
    unsigned long tvocInterval = 60000;
    unsigned long shtInterval = 60000;
    unsigned long coInterval = 60000;  // SC16-CO (Carbon Monoxide)
};

#endif
