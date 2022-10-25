#include "Components/Project/DHT_11/DHT_11.hpp"

DHT_11::DHT_11(uint8_t DHT_pin, uint8_t DHT_type) : _DHT_pin(DHT_pin), _DHT_type(DHT_type)
{
}

void DHT_11::init(void)
{
  DHT dht(_DHT_pin, _DHT_type);
  _dht = dht;
  _dht.begin();
}

float DHT_11::return_temp(void)
{
  float temp = _dht.readTemperature();
  return temp;
}
