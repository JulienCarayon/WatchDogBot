/*
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

#define MQTT_PORT 1883
#define MQTT_USER "test"
#define MQTT_PASSWORD "test"
#define MQTT_SERIAL_PUBLISH_CH "DOG/PUBLISH"
#define MQTT_SERIAL_RECEIVER_CH "DOG/RECEIVE"
#define MQTT_SERIAL_SECURITY_LOGGER_CH "DOG/SECURITY"
#define MQTT_SERIAL_INFO_CH "DOG/INFO"
#define MQTT_SERIAL_MOTORS_CH "DOG/MOTORS"
#define MQTT_SERIAL_DUTYCYCLE_MOTORS_CH "DOG/DUTYCYCLE"

#define DHT_PIN 33
#define DHT_TYPE DHT11

#define MOTOR2_PIN1 16
#define MOTOR2_PIN2 17
#define MOTOR1_PIN1 18
#define MOTOR1_PIN2 19
#define MOTOR12_PWM 5

#define HC_SR501_PIN 2
#define HC_SR501_LED_PIN 15

#define HC_SR04_TRIGGER_PIN 14
#define HC_SR04_ECHO_PIN 12

const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const float SOUND_SPEED = 340.0 / 1000;

bool in_message_treatment(string message, LiquidCrystal_I2C lcd_obj, ControlMotorsL298n motors);
void test_motors(string message, ControlMotorsL298n motors);
void clear_lcd_line(uint8_t line, LiquidCrystal_I2C lcd_obj);
// void publishSerialData(const char *topic, char *serialData);
void hcsr04_get_data(void);

static char in_message[16];
uint8_t in_message_len = 0;
bool get_temperature = false;
uint8_t state_motor = 0;
uint8_t hc_sr501_state = 0;

CALLBACK callbackReturn;

uint8_t motor12_pin1 = 16;
uint8_t motor12_pin2 = 17;
uint8_t motor34_pin1 = 18;
uint8_t motor34_pin2 = 5;
uint8_t motorPWM12_pin = 4;
uint8_t motorPWM34_pin = 19;
uint8_t old_dutyCycle = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT_TYPE);
ControlMotorsL298n motors(motor12_pin1, motor12_pin2, motor34_pin1, motor34_pin2, motorPWM12_pin, motorPWM34_pin);
ClientMQTT clientMQTT(ssid, password, mqtt_server, port, client, callbackReturn);


string convertToString(char *character_array, int size)
{
    string convert_string = "";
    for (int i = 0; i < size; i++)
    {
        convert_string += character_array[i];
    }
    return convert_string;
}

void setup_wifi()
{
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    // print your WiFi shield's IP address:
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD))
        {
            Serial.println(" connected");
            // Once connected, publish an announcement...
            client.publish(MQTT_SERIAL_INFO_CH, "LOG : CONNECTED");
            // ... and resubscribe
            client.subscribe(MQTT_SERIAL_RECEIVER_CH);
            client.subscribe(MQTT_SERIAL_MOTORS_CH);
            client.subscribe(MQTT_SERIAL_DUTYCYCLE_MOTORS_CH);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
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

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]\n");

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

    Serial.println("-----------------------------------");
    Serial.print("Received from mqtt : ");
    Serial.println(in_message);
    Serial.println("-----------------------------------");

    if ((string)topic == "DOG/DUTYCYCLE")
    {
        string in_message_string = convertToString(in_message, in_message_len);
        Serial.println(in_message_string.c_str());

        if (old_dutyCycle != atoi(in_message))
        {
            old_dutyCycle = atoi(in_message);
            motors.setMotorsSpeed(atoi(in_message));
            motors.getMotorsSpeed();
            Serial.println("NEW MOTOR SPEED");
        }
        test_motors(in_message, motors);
    }
    else if ((string)topic == "DOG/MOTORS")
    {
        string in_message_string = convertToString(in_message, in_message_len);
        Serial.println(in_message_string.c_str());

        test_motors(in_message, motors);
    }
    else
    {
        string in_message_string = convertToString(in_message, in_message_len);
        clear_lcd_line(0, lcd);
        get_temperature = in_message_treatment(in_message, lcd, motors);

        if (get_temperature)
        {
            float temp = dht.readTemperature();
            Serial.println("temp : " + String(temp));

            char buffer[5];
            int ret = snprintf(buffer, sizeof(buffer), "%f", temp);

            if ((ret < 0) || (ret >= sizeof(buffer) + 64))
                // client.publish(MQTT_SERIAL_INFO_CH, "LOG : EXIT ERROR !");
                clientMQTT.publishMQTT(MQTT_SERIAL_INFO_CH, "LOG : EXIT ERROR !");
            else
            {
                string command_received = convertToString(buffer, sizeof(buffer));
                Serial.print("command received :");
                Serial.println(command_received.c_str());
                publishSerialData(MQTT_SERIAL_PUBLISH_CH, buffer);
            }
            get_temperature = false;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    // clientMQTT.pubSubClient(wifiClient);
    //  LCD INIT
    lcd.init();
    lcd.clear();
    lcd.backlight(); // Make sure backlight is on
    lcd.noCursor();
    lcd.setCursor(0, 0); // Set cursor to character 2 on line 0
    lcd.print("WatchDog BOT");
    delay(2000);
    lcd.clear();

    // DHT INIT
    dht.begin();

    // MOTOR INIT
    motors.init();
    // pinMode(MOTOR1_PIN1, OUTPUT);
    // pinMode(MOTOR1_PIN2, OUTPUT);
    // pinMode(MOTOR2_PIN1, OUTPUT);
    // pinMode(MOTOR2_PIN2, OUTPUT);
    // pinMode(MOTOR12_PWM, OUTPUT);

    // HC-SR501 MOUVEMENT CAPTOR
    pinMode(HC_SR501_PIN, INPUT);
    pinMode(HC_SR501_LED_PIN, OUTPUT);
    digitalWrite(HC_SR501_LED_PIN, LOW);

    // HC-SR04 US SENSOR
    pinMode(HC_SR04_TRIGGER_PIN, OUTPUT);
    digitalWrite(HC_SR04_TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
    pinMode(HC_SR04_ECHO_PIN, INPUT);

    // WIFI INIT
    Serial.setTimeout(500); // Set time out for
    // setup_wifi();
    clientMQTT.setupWifi();

    // MQTT CLIENT INIT
    clientMQTT.setServerMQTT();
    // client.setServer(mqtt_server, MQTT_PORT);

    // client.setCallback(callback);
    clientMQTT.setCallbackMQTT();

    // reconnect();
    clientMQTT.reconnect();
}
/*
void publishSerialData(const char *topic, char *serialData)
{
    if (!clientMQTT.connected())
    {
        // reconnect();
        clientMQTT.reconnect();
    }
    // client.publish(topic, serialData);
    clientMQTT.publishMQTT(topic, serialData);
    Serial.println("Publish information : " + String(serialData));
}

int period = 3000;
unsigned long time_now = 0;

void loop()
{
    void setValue(byte payload);
    // client.loop();
    clientMQTT.loopMQTT();
    motors.setMotorsSpeed(80);
    string str = "back";
    test_motors(str, motors);
    delay(1000);
    str = "stop";
    test_motors(str, motors);
    delay(1000);
}

bool in_message_treatment(string message, LiquidCrystal_I2C lcd_obj, ControlMotorsL298n motors)
{
    if (message == "right")
        lcd_obj.print("right");
    else if (message == "left")
    {
        Serial.println("left");
        lcd_obj.print("left");
    }
    else if (message == "back")
    {
        Serial.println("back");
        lcd_obj.print("back");
    }
    else if (message == "forward")
        lcd_obj.print("forward");
    else if (message == "stop")
        lcd_obj.print("stop");
    else if (message == "temp")
        return true;

    return false;
}

void test_motors(string message, ControlMotorsL298n motors)
{
    // Serial.println("Message : " + String(message.c_str()));
    if (message == "right")
        motors.goRight();
    else if (message == "left")
    {
        string str = "left";
        motors.goLeft();
    }
    else if (message == "back")
    {
        string str = "back";
        motors.goBack();
    }
    else if (message == "forward")
        motors.goForward();
    else if (message == "no_left" || message == "no_right" || message == "no_back" || message == "no_forward" || message == "stop")
        motors.stop();
}

void clear_lcd_line(uint8_t line, LiquidCrystal_I2C lcd_obj)
{
    for (int column = 0; column < 15; column++)
    {
        lcd_obj.setCursor(column, line);
        lcd_obj.print(" ");
    }
    lcd_obj.setCursor(0, line);
    // char *log = (char *)"INFO : LINE CLEAR !";
    // mqtt_client.publish(MQTT_SERIAL_INFO_CH, log);
}

void hcsr04_get_data(void)
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
}
*/