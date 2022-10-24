#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "Wire.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <string>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include "Components/ControlMotors/ControlMotors_L298n.hpp"
#include "Components/ClientMQTT/ClientMQTT.hpp"

using namespace std;

// Update these with values suitable for your network.
// const char *ssid = "HUAWEI P30 lite"; // Enter your WiFi name
// const char *password = "0123456789";  // Enter WiFi password
const char *ssid = "Livebox-0209";                   // Enter your WiFi name
const char *password = "4612EA4513FE6C618F8CCF7C70"; // Enter WiFi password
// const char *ssid = "SER@YNOV";                 // Enter your WiFi name
// const char *password = "Q156@QeRd^YuRgRouX7T"; // Enter WiFi password
const char *mqtt_server = "broker.emqx.io";
uint16_t port = 1883;

#define HC_SR501_PIN 2
#define HC_SR501_LED_PIN 15
#define HC_SR04_TRIGGER_PIN 14
#define HC_SR04_ECHO_PIN 12

const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const float SOUND_SPEED = 340.0 / 1000;

void test_motors(string message, ControlMotorsL298n motors);
void treatmentMQTTCommands(CALLBACK command);
float hcsr04_get_data(void);

CALLBACK callbackStruct;

uint8_t motor12_pin1 = 16;
uint8_t motor12_pin2 = 17;
uint8_t motor34_pin1 = 18;
uint8_t motor34_pin2 = 5;
uint8_t motorPWM12_pin = 4;
uint8_t motorPWM34_pin = 19;
uint8_t old_dutyCycle = 0;

string topic;
string message;

bool guardModeAcivate = false;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

ControlMotorsL298n motors(motor12_pin1, motor12_pin2, motor34_pin1, motor34_pin2, motorPWM12_pin, motorPWM34_pin);
ClientMQTT clientMQTT(ssid, password, mqtt_server, port, client);

void setup()
{
    Serial.begin(115200);

    // MOTOR INIT
    motors.init();

    // HC-SR501 MOUVEMENT CAPTOR
    pinMode(HC_SR501_PIN, INPUT);
    pinMode(HC_SR501_LED_PIN, OUTPUT);
    digitalWrite(HC_SR501_LED_PIN, LOW);

    // HC-SR04 US SENSOR
    pinMode(HC_SR04_TRIGGER_PIN, OUTPUT);
    digitalWrite(HC_SR04_TRIGGER_PIN, LOW);
    pinMode(HC_SR04_ECHO_PIN, INPUT);

    // WIFI INIT
    Serial.setTimeout(500);
    clientMQTT.setupWifiMQTT();

    // MQTT CLIENT INIT
    clientMQTT.setServerMQTT();
    clientMQTT.setCallbackMQTT();
    clientMQTT.reconnectMQTT();
}

void loop()
{
    clientMQTT.loopMQTT();
    callbackStruct = clientMQTT.getCallbackReturn();
    treatmentMQTTCommands(callbackStruct);

    if (guardModeAcivate)
    {
        // autonomous mode
    }
}

void test_motors(string message, ControlMotorsL298n motors)
{
    if (message == "right")
        motors.goRight();
    else if (message == "left")
        motors.goLeft();
    else if (message == "back")
        motors.goBack();
    else if (message == "forward")
        motors.goForward();
    else if (message == "no_left" || message == "no_right" || message == "no_back" || message == "no_forward" || message == "stop")
        motors.stop();
}

float hcsr04_get_data(void)
{
    // Sending 10 µs pulse
    digitalWrite(HC_SR04_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_SR04_TRIGGER_PIN, LOW);

    // Measuring time between sending the TRIG pulse and its echo
    long measure = pulseIn(HC_SR04_ECHO_PIN, HIGH, MEASURE_TIMEOUT);

    // Calculing the distance
    float distance_mm = measure / 2.0 * SOUND_SPEED;

    Serial.print(F("Distance: "));
    Serial.print(distance_mm);
    Serial.print(F("mm ("));
    Serial.print(distance_mm / 10.0, 2);
    Serial.print(F("cm, "));
    Serial.print(distance_mm / 1000.0, 2);
    Serial.println(F("m)"));

    return distance_mm;
}

void treatmentMQTTCommands(CALLBACK command)
{
    if (command.new_message)
    {
        // Serial.print("main.cpp  : ");
        // Serial.println(command.message.c_str());
        topic = command.topic;
        message = command.message;

        if (topic == "DOG/DUTYCYCLE")
        {
            if (old_dutyCycle != atoi(message.c_str()))
            {
                Serial.print("SET NEW MOTORS SPEED ....... ");
                old_dutyCycle = atoi(message.c_str());
                motors.setMotorsSpeed(atoi(message.c_str()));
                // motors.getMotorsSpeed();
                Serial.println("----> NEW MOTOR SPEED SET");
            }
        }
        else if (topic == "DOG/MOTORS")
        {
            test_motors(message, motors);
        }
        else if (topic == "DOG/TEMP")
        {
            Serial.println("*** FROM TOPIC [DOG/RECEIVE] ***");
            // need to publish temp
        }
        else if (topic == "DOG/SECURITY")
        {
            Serial.println("*** FROM TOPIC [DOG/SECURITY] ***");
        }
        else if (topic == "DOG/GUARD")
        {
            Serial.println("*** FROM TOPIC [DOG/GUARD] ***");
            if (message == "guard_on")
                guardModeAcivate = true;
            else if (message == "guard_off")
                guardModeAcivate = false;
        }
    }
}