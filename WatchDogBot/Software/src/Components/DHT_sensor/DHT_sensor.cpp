#include "Components/DHT_sensor/DHT_sensor.hpp"
#include <Arduino.h>

DHT_sensor::DHT_sensor(uint8_t DHT_pin, uint8_t DHType)
{
    _dht_pin = DHT_pin;
    _dhtype = DHType;
    DHT sensor(_dht_pin, _dhtype, 1);
    _sensor = sensor;
}

void DHT_sensor::initDHT(void)
{
    _sensor.begin();
}

float DHT_sensor::get_temperature_sensor(void)
{
    temp = _sensor.readTemperature(false, false);
    Serial.println(String(temp));
    return temp;
}