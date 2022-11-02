#ifndef ClientMQTT_H
#define ClientMQTT_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <iostream>
#include <string>

#include "Wire.h"
#include "WiFi.h"
#include "WiFiClient.h"

#include "../PubSubClient/PubSubClient.hpp"
#include "../Global/Global.hpp"

#define MQTT_USER "test"
#define MQTT_PASSWORD "test"

#define MQTT_SERIAL_PUBLISH_CH "DOG/PUBLISH"
#define MQTT_SERIAL_TEMP_CH "DOG/TEMP"
#define MQTT_SERIAL_SECURITY_LOGGER_CH "DOG/SECURITY"
#define MQTT_SERIAL_INFO_CH "DOG/INFO"
#define MQTT_SERIAL_MOTORS_CH "DOG/MOTORS"
#define MQTT_SERIAL_DUTYCYCLE_MOTORS_CH "DOG/DUTYCYCLE"
#define MQTT_SERIAL_GUARD_CH "DOG/GUARD"
#define MQTT_SERIAL_SOC_CH "DOG/SOC"
#define MQTT_SERIAL_STATE_CH "DOG/STATE"

using namespace std;

struct CALLBACK;

class ClientMQTT : public PubSubClient
{
public:
    ClientMQTT(const char *ssid, const char *password, const char *mqtt_server,
               uint16_t port, PubSubClient client);

    void setupWifiMQTT();
    void pubSubClientMQTT(WiFiClient wifiClient_obj);
    void reconnectMQTT();
    void setServerMQTT();
    void publishMQTT(const char *topic, const char *payload);
    void publishSerialDataMQTT(const char *topic, char *serialData);
    void loopMQTT();
    void setCallbackMQTT();
    static CALLBACK callbackMQTT2(char *topic, uint8_t *payload, unsigned int length);
    CALLBACK getCallbackReturn();
    static bool _guardmode_activate;

private:
    uint16_t _port;
    uint16_t _period;
    const char *_ssid;
    const char *_password;
    const char *_mqtt_server;
    unsigned long _time_now;
    WiFiClient _wifiClient;
    PubSubClient _clientMQTT;
    CALLBACK _callbackReturn;
};

#endif