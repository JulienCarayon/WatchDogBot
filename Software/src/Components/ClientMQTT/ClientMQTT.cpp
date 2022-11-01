#include "ClientMQTT.hpp"
#include "Arduino.h"

#define DEBUG_MODE
//#undef DEBUG_MODE

ClientMQTT::ClientMQTT(const char *ssid, const char *password, const char *mqtt_server,
                       uint16_t port, PubSubClient client)
{
    _ssid = ssid;
    _password = password;
    _mqtt_server = mqtt_server;
    _port = port;
    _clientMQTT = client;
    _period = 3000;
    _time_now = 0;
}

void ClientMQTT::pubSubClientMQTT(WiFiClient wifiClient_obj)
{
    PubSubClient client(wifiClient_obj);
    _clientMQTT = client;
}

void ClientMQTT::setupWifiMQTT()
{
    delay(10);

    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(_ssid);
    WiFi.begin(_ssid, _password);

    int timeout_counter = 0;

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(200);
        timeout_counter++;
        if (timeout_counter >= 10 * 5)
        {
            Serial.println("RESTART");
            ESP.restart();
        }
    }

    // print your WiFi shield's IP address:
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void ClientMQTT::reconnectMQTT()
{
    // Loop until we're reconnected
    while (!_clientMQTT.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (_clientMQTT.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD))
        {
            Serial.println(" connected");
            // Once connected, publish an announcement...
            _clientMQTT.publish(MQTT_SERIAL_INFO_CH, "LOG : CONNECTED");
            // ... and resubscribe
            _clientMQTT.subscribe(MQTT_SERIAL_TEMP_CH);
            _clientMQTT.subscribe(MQTT_SERIAL_MOTORS_CH);
            _clientMQTT.subscribe(MQTT_SERIAL_DUTYCYCLE_MOTORS_CH);
            _clientMQTT.subscribe(MQTT_SERIAL_GUARD_CH);
            _clientMQTT.subscribe(MQTT_SERIAL_SOC_CH);
            _clientMQTT.subscribe(MQTT_SERIAL_STATE_CH);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(_clientMQTT.state());
            Serial.println(" try again in 5 seconds");

            // Wait 5 seconds before retrying
            int i = 5;
            while (i > 0)
            {
                delay(1000);
                Serial.print(i);
                Serial.println("...");
                i--;
            }
        }
    }
}

void ClientMQTT::setServerMQTT()
{
    _clientMQTT.setServer(_mqtt_server, _port);
}

void ClientMQTT::publishMQTT(const char *topic, const char *payload)
{
    _clientMQTT.publish(topic, payload);
}

void ClientMQTT::publishSerialDataMQTT(const char *topic, char *serialData)
{
    if (!_clientMQTT.connected())
    {
        reconnectMQTT();
    }
    _clientMQTT.publish(topic, serialData);
    Serial.println("Publish information : " + String(serialData));
}

void ClientMQTT::loopMQTT()
{
    _callbackReturn = _clientMQTT.loop();

    if (millis() >= _time_now + _period)
    {
        _time_now += _period;

        Serial.println("Waiting commands ...");
    }
}

CALLBACK ClientMQTT::callbackMQTT2(char *topic, uint8_t *payload, unsigned int length)
{
    CALLBACK callbackReturnClass;

    Serial.print("\n*** MESSAGE ARRIVED [");
    Serial.print(topic);
    Serial.print("]\n");

    char in_message[16];
    uint8_t in_message_len = 0;

    for (int index = 0; index < 16; index++)
    {
        if (in_message[index] != '\0')
        {
            in_message[index] = '\0';
        }
    }
    in_message_len = length;

    for (int i = 0; i < length; i++)
    {
        in_message[i] = char(payload[i]);
    }

#ifdef DEBUG_MODE
    Serial.println("-----------------------------------");
    Serial.print("Received from mqtt : ");
    Serial.println(in_message);
    Serial.println("----------------------------------- ***\n");
#endif

    string in_message_string = convertToString(in_message, in_message_len);
    callbackReturnClass.topic = (string)topic;
    callbackReturnClass.message = in_message_string;

    return callbackReturnClass;
}

void ClientMQTT::setCallbackMQTT()
{
    _clientMQTT.setCallback(callbackMQTT2);
}

CALLBACK ClientMQTT::getCallbackReturn()
{
    return _callbackReturn;
}
