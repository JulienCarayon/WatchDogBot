#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "Wire.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <string>
#include <Adafruit_Sensor.h>
#include <stdio.h>
#include <stdlib.h>
// Lib for upload with wifi
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// Lib for upload with webside
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <AsyncElegantOTA.h>
//
#include "Components/LCD_16x2/LCD_16x2.hpp"
#include "Components/ControlMotors/ControlMotors_L298n.hpp"
#include "Components/ClientMQTT/ClientMQTT.hpp"
#include "Components/WS2812B_LED_Controller/WS2812B_LED_Controller.hpp"
#include "Components/DHT_sensor/DHT_sensor.hpp"

using namespace std;

// Update these with values suitable for your network.

// const char *ssid = "Livebox-0209";                   // Enter your WiFi name
// const char *password = "4612EA4513FE6C618F8CCF7C70"; // Enter WiFi password
// const char *ssid = "SER@YNOV";
// const char *password = "Q156@QeRd^YuRgRouX7T";
// const char *ssid = "HUAWEI P30 lite";

// const char *ssid = "iPhone de Julien";
// const char *password = "0123456789";

const char *ssid = "Freebox-5C96C4";
const char *password = "9v6tvq6b2vqnx3zhq6w2f5";

// AsyncWebServer server(80);
const char *mqtt_server = "broker.emqx.io";
uint16_t port = 1883;

#define BUZZER_PIN 15

#define HC_SR501_PIN 23

#define HC_SR04_TRIGGER_PIN_LEFT 14
#define HC_SR04_ECHO_PIN_LEFT 12
#define HC_SR04_TRIGGER_PIN_RIGHT 26
#define HC_SR04_ECHO_PIN_RIGHT 27
#define HC_SR04_RIGHT "right"
#define HC_SR04_LEFT "left"

#define DHT_PIN 9
#define DHT_TYPE DHT11

#define BAT_PIN 36

#define LED_I2C_ADDRESS 0x20
#define LEDS_COUNT 10

#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMN_NUMBER 16
#define LCD_LINE_NUMBER 2

const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const float SOUND_SPEED = 340.0 / 1000;
HC_SR04_DATA_T hc_sr04_data_t;
FSM_T fsm_t = IDLE;
g_fsm_flags fsm_flags;

void test_motors(string message, ControlMotorsL298n motors);
void treatmentMQTTCommands(CALLBACK command);
uint8_t hcsr04_get_data(string sensor_position);
bool hcsr501_get_data(void);
uint16_t get_voltage_percentage();
void guard_mode();
void get_temperature_from_DHT(void);
void police_BIP(void);
void buzzer_off(void);

CALLBACK callbackStruct;

uint8_t motor12_pin1 = 16;
uint8_t motor12_pin2 = 17;
uint8_t motor34_pin1 = 18;
uint8_t motor34_pin2 = 5;
uint8_t motorPWM12_pin = 4;
uint8_t motorPWM34_pin = 19;
uint8_t old_dutyCycle = 0;

uint16_t soc_data = 0;
uint16_t old_soc = 0;
uint8_t soc = 0;
uint16_t adc_read = 0;
uint16_t div_voltage = 0;
uint16_t bat_voltage = 0;

string topic;
string message;

bool guardModeAcivate = false;
bool movement_detected = false;
bool leftMode = false;
bool rightMode = false;
bool backMode = false;

float data = 0.0;
unsigned long previousMillis;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
ControlMotorsL298n motors(motor12_pin1, motor12_pin2, motor34_pin1, motor34_pin2, motorPWM12_pin, motorPWM34_pin);
ClientMQTT clientMQTT(ssid, password, mqtt_server, port, client);
WS2812B_Controller LED_strip(LED_I2C_ADDRESS, LEDS_COUNT, TYPE_GRB);
LCD_16x2 lcd(LCD_I2C_ADDRESS, LCD_COLUMN_NUMBER, LCD_LINE_NUMBER);

DHT_sensor test(9, DHT11);

void initOTA()
{
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname("ESP32");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA
        .onStart([]()
                 {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

    ArduinoOTA.begin();
}

void setup()
{
    Serial.begin(115200);

    // INIT FOR BOOTING WITH WIFI
    Serial.println("Booting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    initOTA();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    //

    // BUZZER INIT
    pinMode(BUZZER_PIN, OUTPUT);

    tone(BUZZER_PIN, 600);
    delay(100);
    tone(BUZZER_PIN, 0);

    // LCD INIT
    lcd.initLCD();
    lcd.displayStringLCD("WatchDog BOT", 0, 0);
    delay(2000);
    lcd.clearLCD();

    // DHT INIT
    test.initDHT();

    // STRIP LED INIT
    LED_strip.begin();

    // MOTOR INIT
    motors.init();

    // HC-SR501 MOVEMENT CAPTOR
    pinMode(HC_SR501_PIN, INPUT);

    // HC-SR04 US SENSOR
    pinMode(HC_SR04_TRIGGER_PIN_LEFT, OUTPUT);
    pinMode(HC_SR04_ECHO_PIN_LEFT, INPUT);
    digitalWrite(HC_SR04_TRIGGER_PIN_LEFT, LOW);
    pinMode(HC_SR04_TRIGGER_PIN_RIGHT, OUTPUT);
    pinMode(HC_SR04_ECHO_PIN_RIGHT, INPUT);
    digitalWrite(HC_SR04_TRIGGER_PIN_RIGHT, LOW);

    // WIFI INIT
    Serial.setTimeout(500);
    clientMQTT.setupWifiMQTT();

    // MQTT CLIENT INIT
    clientMQTT.setServerMQTT();
    clientMQTT.setCallbackMQTT();
    clientMQTT.reconnectMQTT();
}

// void loop_tag_v1()
void loop()
{

    // FONCTION FOR UPDATE WITH WIFI//
    ArduinoOTA.handle();
    if (millis() - previousMillis >= 500)
    {
        previousMillis = millis();
        Serial.println(F("Code has been update"));
    }
    //

    clientMQTT.loopMQTT();
    callbackStruct = clientMQTT.getCallbackReturn();
    treatmentMQTTCommands(callbackStruct);

    if (guardModeAcivate)
    {
        Serial.println("GUARD MODE ACTIVATE");
        lcd.clearLineLCD(1);
        lcd.displayStringLCD("GUARD MODE : ON ", 0, 0);

        hc_sr04_data_t.distance_right = hcsr04_get_data(HC_SR04_RIGHT);
        hc_sr04_data_t.distance_left = hcsr04_get_data(HC_SR04_LEFT);

        if (hcsr501_get_data())
        {
            Serial.println("==================================== > Detected");
            LED_strip.policeGang();
            police_BIP();
        }
        Serial.println("Distance right : " + String(hc_sr04_data_t.distance_right));
        Serial.println("Distance left : " + String(hc_sr04_data_t.distance_left));
    }
    else
    {
        lcd.displayStringLCD("GUARD MODE : OFF", 0, 0);
        lcd.clearLineLCD(1);
        LED_strip.turnOFF();
        buzzer_off();
    }

    // test.get_temperature_sensor();

    /* blinking tests */
    if (leftMode)
        LED_strip.blinkingLeft();
    if (rightMode)
        LED_strip.blinkingRight();
    if (backMode)
        LED_strip.warning();
    // LED_strip.WeWillFuckYou();

    /* SOC tests*/
    soc_data = get_voltage_percentage();
}

void loop_tada()
{
    clientMQTT.loopMQTT();
    callbackStruct = clientMQTT.getCallbackReturn();
    treatmentMQTTCommands(callbackStruct);
}

void fsm_state()
{
    switch (fsm_t)
    {
    case IDLE:
        fsm_flags.idleState = true;
        fsm_flags.manualState = false;
        fsm_flags.guardingState = false;
        fsm_flags.detectedState = false;
        fsm_flags.errorState = false;
        break;

    case MANUAL:
        fsm_flags.idleState = false;
        fsm_flags.manualState = true;
        fsm_flags.guardingState = false;
        fsm_flags.detectedState = false;
        fsm_flags.errorState = false;
        break;

    case GUARDING:
        fsm_flags.idleState = false;
        fsm_flags.manualState = false;
        fsm_flags.guardingState = true;
        fsm_flags.detectedState = false;
        fsm_flags.errorState = false;
        break;

    case DETECTED:
        fsm_flags.idleState = false;
        fsm_flags.manualState = false;
        fsm_flags.guardingState = false;
        fsm_flags.detectedState = true;
        fsm_flags.errorState = false;
        break;

    case ERROR:
        fsm_flags.idleState = false;
        fsm_flags.manualState = false;
        fsm_flags.guardingState = false;
        fsm_flags.detectedState = false;
        fsm_flags.errorState = true;
        break;
    default:
        fsm_t = IDLE;
        break;
    }
}

void get_temperature_from_DHT(void)
{
    data = test.get_temperature_sensor();
    char buffer[5];
    int ret = snprintf(buffer, sizeof(buffer), "%f", data);
    clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_TEMP_CH, buffer);
}

uint16_t get_voltage_percentage()
{
    adc_read = analogRead(BAT_PIN);
    div_voltage = (uint16_t)map(adc_read, 0, 4095, 0, 3300);
    bat_voltage = (div_voltage * 1500) / 500;

    soc = (uint8_t)map(bat_voltage, 0, 8400, 0, 100);
    // Serial.println(soc);

    if (soc > (old_soc + 1) || soc < (old_soc - 1))
    {
        old_soc = soc;
        // Serial.println("SOC : " + soc);

        char buffer[64];
        int ret = snprintf(buffer, sizeof(buffer), "%d", soc);

        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_SOC_CH, buffer);

        return soc;
    }
    return 0;
}

// autonomous mode
void test_motors(string message, ControlMotorsL298n motors)
{
    if (message == "right")
    {
        motors.goRight();
        lcd.displayStringLCD("right", 0, 0);
        LED_strip.blinkingRight();
        rightMode = true;
        leftMode = false;
        backMode = false;
    }
    else if (message == "left")
    {
        motors.goLeft();
        lcd.displayStringLCD("left", 0, 0);
        LED_strip.blinkingLeft();
        leftMode = true;
        rightMode = false;
        backMode = false;
    }

    else if (message == "back")
    {
        motors.goBack();
        LED_strip.warning();
        backMode = true;
    }
    else if (message == "forward")
    {
        leftMode = false;
        rightMode = false;
        motors.goForward();
    }
    else if (message == "no_left" || message == "no_right" || message == "no_back" || message == "no_forward" || message == "stop")
    {
        motors.stop();
        backMode = false;
    }
}

uint8_t hcsr04_get_data(string sensor_position)
{
    long measure;

    // Sending 10 µs pulse
    if (sensor_position == "left")
    {
        digitalWrite(HC_SR04_TRIGGER_PIN_LEFT, HIGH);
        delayMicroseconds(10);
        digitalWrite(HC_SR04_TRIGGER_PIN_LEFT, LOW);

        // Measuring time between sending the TRIG pulse and its echo
        measure = pulseIn(HC_SR04_ECHO_PIN_LEFT, HIGH, MEASURE_TIMEOUT);
    }
    else if (sensor_position == "right")
    {

        digitalWrite(HC_SR04_TRIGGER_PIN_RIGHT, HIGH);
        delayMicroseconds(10);
        digitalWrite(HC_SR04_TRIGGER_PIN_RIGHT, LOW);

        // Measuring time between sending the TRIG pulse and its echo
        measure = pulseIn(HC_SR04_ECHO_PIN_RIGHT, HIGH, MEASURE_TIMEOUT);
    }

    // Calculing the distance
    float distance_mm = measure / 2.0 * SOUND_SPEED;

    // Serial.print(F("Distance: "));
    // Serial.print(distance_mm);
    // Serial.print(F("mm ("));
    // Serial.print(distance_mm / 10.0, 2);
    // Serial.print(F("cm, "));
    // Serial.print(distance_mm / 1000.0, 2);
    // Serial.intln(F("m)"));

    return (uint8_t)distance_mm;
}

bool hcsr501_get_data(void)
{
    return (bool)digitalRead(HC_SR501_PIN);
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
            Serial.println("*** FROM TOPIC [DOG/TEMP] ***");
            get_temperature_from_DHT();
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

void guard_mode()
{
}

void police_BIP(void)
{

    for (uint16_t i = 600; i < 800; i++)
    {
        tone(BUZZER_PIN, i);
        delay(1);
    }
    for (uint16_t i = 800; i > 600; i--)
    {
        tone(BUZZER_PIN, i);
        delay(1);
    }
}

void buzzer_off(void)
{
    tone(BUZZER_PIN, 0);
}