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

using namespace std;

// Update these with values suitable for your network.
const char *ssid = "HUAWEI P30 lite"; // Enter your WiFi name
const char *password = "0123456789";  // Enter WiFi password
// const char *ssid = "iPhone de Dorian"; // Enter your WiFi name
// const char *password = "WifiEc1502";   // Enter WiFi password
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

#define HC_SR04_TRIGGER_PIN 14
#define HC_SR04_ECHO_PIN 12

const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const float SOUND_SPEED = 340.0 / 1000;

bool in_message_treatment(string message, LiquidCrystal_I2C lcd_obj);
void clear_lcd_line(uint8_t line, LiquidCrystal_I2C lcd_obj);
void publishSerialData(const char *topic, char *serialData);
void hcsr04_get_data(void);
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
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT_TYPE);

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
    clear_lcd_line(0, lcd);
    get_temperature = in_message_treatment(in_message, lcd);

    if (get_temperature)
    {
        float temp = dht.readTemperature();
        Serial.println("temp : " + String(temp));

        char buffer[5];
        int ret = snprintf(buffer, sizeof(buffer), "%f", temp);

        if ((ret < 0) || (ret >= sizeof(buffer) + 64))
            client.publish(MQTT_SERIAL_INFO_CH, "LOG : EXIT ERROR !");
        else
        {
            String command_received = convertToString(buffer, sizeof(buffer));
            Serial.println("commande_received : " + command_received);
            publishSerialData(MQTT_SERIAL_PUBLISH_CH, buffer);
        }
        get_temperature = false;
    }
}

void setup()
{
    Serial.begin(115200);

    // LCD INIT
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
    pinMode(MOTOR1_PIN1, OUTPUT);
    pinMode(MOTOR1_PIN2, OUTPUT);
    pinMode(MOTOR2_PIN1, OUTPUT);
    pinMode(MOTOR2_PIN2, OUTPUT);
    pinMode(MOTOR12_PWM, OUTPUT);

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
            publishSerialData(MQTT_SERIAL_SECURITY_LOGGER_CH, "no detected");
            digitalWrite(HC_SR501_LED_PIN, LOW);
        }
        // hcsr04_get_data();
    }
}

bool in_message_treatment(string message, LiquidCrystal_I2C lcd_obj)
{
    if (message == "right")
        lcd_obj.print("right");
    else if (message == "left")
        lcd_obj.print("left");
    else if (message == "back")
    {
        lcd_obj.print("back");
        digitalWrite(MOTOR12_PWM, HIGH);
        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
    }
    else if (message == "no_back")
    {
        lcd_obj.print("no back");
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR2_PIN1, LOW);
    }
    else if (message == "forward")
    {
        lcd_obj.print("forward");
        digitalWrite(MOTOR12_PWM, HIGH);
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
        delay(3000);
        digitalWrite(MOTOR1_PIN2, LOW);
        digitalWrite(MOTOR2_PIN2, LOW);
    }

    else if (message == "temp")
        return true;

    return false;
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