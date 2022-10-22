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
#include "Components/LCD_16x2/LCD_16x2.hpp"


using namespace std;

// Update these with values suitable for your network.
const char *ssid = "Maison Bieujac";                   // Enter your WiFi name
const char *password = "Bienvenue"; // Enter WiFi password
const char *mqtt_server = "broker.emqx.io";

#define MQTT_PORT 1883
#define MQTT_USER "test"
#define MQTT_PASSWORD "test"
#define MQTT_SERIAL_PUBLISH_CH "DOG/PUBLISH"
#define MQTT_SERIAL_RECEIVER_CH "DOG/RECEIVE"
#define MQTT_SERIAL_SECURITY_LOGGER_CH "DOG/SECURITY"
#define MQTT_SERIAL_INFO_CH "DOG/INFO"

#define DHT_PIN 4
#define DHT_TYPE DHT11

#define MOTOR2_PIN1 16
#define MOTOR2_PIN2 17
#define MOTOR1_PIN1 18
#define MOTOR1_PIN2 19
#define MOTOR12_PWM 5

#define HC_SR501_PIN 2
#define HC_SR501_LED_PIN 15

// bool in_message_treatment(string message, LCD_16x2 lcd_obj);
void publishSerialData(const char *topic, char *serialData);
string go_forward();
string go_back();
string go_left();
string go_right();

char in_message[16];
uint8_t in_message_len;
bool get_temperature = false;
uint8_t state_motor = 0;
uint8_t hc_sr501_state = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
DHT dht(DHT_PIN, DHT_TYPE);

// LiquidCrystal_I2C LiquidCrystalI2c(0x27, 16, 2);

LCD_16x2 lcd(0x27, 16, 2);
String convertToString(char *character_array, int size)
{
    String convert_string = "";
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
    lcd.displayStringLCD("Connecting to ",0,0);
    lcd.displayStringLCD(ssid,0,1);
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

    String in_message_string = convertToString(in_message, in_message_len);
    // lcd.clearLineLCD(0);
    // clear_lcd_line(0, lcd);
    // get_temperature = in_message_treatment(in_message, lcd);s

    if (get_temperature)
    {
        float temp = dht.readTemperature();
        Serial.println("temp : " + String(temp));

        char buffer[64];
        int ret = snprintf(buffer, sizeof buffer, "%f", temp);

        if ((ret < 0) || (ret >= sizeof buffer))
            client.publish(MQTT_SERIAL_INFO_CH, "LOG : EXIT ERROR !");
        else
        {
            String command_received = convertToString(buffer, 5);
            Serial.println("commande_received : " + command_received);
            publishSerialData(MQTT_SERIAL_PUBLISH_CH, buffer);
        }
        get_temperature = false;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("BOOTING WATCHDOG");
    lcd.initLCD();
    lcd.displayStringLCD("WatchDog BOT",0,0);
    delay(2000);
    lcd.clearLCD();

    // DHT INIT
    dht.begin();

    // MOTOR INIT
    pinMode(MOTOR1_PIN1, OUTPUT);
    pinMode(MOTOR1_PIN2, OUTPUT);
    pinMode(MOTOR2_PIN1, OUTPUT);
    pinMode(MOTOR2_PIN2, OUTPUT);
    pinMode(MOTOR12_PWM, OUTPUT);

    // HC-SR501 CAPTOR
    pinMode(HC_SR501_PIN, INPUT);
    pinMode(HC_SR501_LED_PIN, OUTPUT);
    digitalWrite(HC_SR501_LED_PIN, LOW);

    // WIFI INIT
    Serial.setTimeout(500); // Set time out for
    setup_wifi();

    // MQTT CLIENT INIT
    client.setServer(mqtt_server, MQTT_PORT);
    client.setCallback(callback);
    reconnect();
}

void publishSerialData(const char *topic, char *serialData)
{
    if (!client.connected())
    {
        reconnect();
    }
    client.publish(topic, serialData);
    Serial.println("Publish information : " + String(serialData));
}

int period = 3000;
unsigned long time_now = 0;

void loop()
{
    void setValue(byte payload);
    client.loop();
    if (millis() >= time_now + period)
    {
        time_now += period;

        Serial.println("Waiting commands ...");
        lcd.clearLineLCD(1);
        lcd.displayStringLCD("Waiting commands",0,0);
        

        hc_sr501_state = digitalRead(HC_SR501_PIN);

        if (hc_sr501_state)
        {
            Serial.println("WARNING : HC-SR501 has detected a movement");
            publishSerialData(MQTT_SERIAL_SECURITY_LOGGER_CH, "detected");
            digitalWrite(HC_SR501_LED_PIN, HIGH);
        }
        else
        {
            hc_sr501_state = 0;
            digitalWrite(HC_SR501_LED_PIN, LOW);
        }
    }
}

bool in_message_treatment(string message, LCD_16x2 lcd_obj)
{
    if (message == "right"){
      lcd_obj.displayStringLCD("right",0,1);
    }
    else if (message == "left"){
        lcd_obj.displayStringLCD("left",0,1);
    }
    else if (message == "back")
    {
        lcd_obj.displayStringLCD("back",0,1);
        digitalWrite(MOTOR12_PWM, HIGH);
        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        delay(3000);
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR2_PIN1, LOW);
    }
    else if (message == "forward")
    {
        lcd_obj.displayStringLCD("forward",0,1);
        digitalWrite(MOTOR12_PWM, HIGH);
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
        delay(3000);
        digitalWrite(MOTOR1_PIN2, LOW);
        digitalWrite(MOTOR2_PIN2, LOW);
    }

    else if (message == "temp"){
    lcd_obj.displayStringLCD("forward",0,1);
        return true;
    }

    return false;
}

// void clear_lcd_line(uint8_t line, LiquidCrystal_I2C lcd_obj)
// {
//     for (int column = 0; column < 15; column++)
//     {
//         lcd_obj.setCursor(column, line);
//         lcd_obj.print(" ");
//     }
//     lcd_obj.setCursor(0, line);
//     // char *log = (char *)"INFO : LINE CLEAR !";
//     // mqtt_client.publish(MQTT_SERIAL_INFO_CH, log);
// }
