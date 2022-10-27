#ifndef DHT_11_H
#define DHT_11_H
#include <Arduino.h>
#include "DHT.h"

class DHT_11
{
public:
    DHT_11(uint8_t DHT_pin, uint8_t DHT_type);
    void init(void);
    float return_temp(void);

private:
    DHT _dht;
    uint8_t _DHT_pin;
    uint8_t _DHT_type;
};
#endif