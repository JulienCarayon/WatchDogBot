#ifndef DHT_SENSOR_h
#define DHT_SENSOR_h

#include <Arduino.h>
#include <stdint.h>
#include <DHT.h>

class DHT_sensor
{
public:
    DHT_sensor(uint8_t DHT_pin, uint8_t DHType);
    void initDHT(void);
    float get_temperature_sensor(void);
    float temp = 0.0;

private:
    DHT _sensor;
    uint8_t _dht_pin;
    uint8_t _dhtype;
};

#endif