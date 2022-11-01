#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "Wire.h"
#include <WiFi.h>
#include <string>
#include <Adafruit_Sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include "Components/ClientMQTT/ClientMQTT.hpp"
#include "Components/PubSubClient/PubSubClient.h"

#include "global.hpp"

using namespace std;

char main_buffer_temp[10];
char main_buffer_soc[10];
char main_buffer_state[10];
TREATMENT_CMD_T treatment_cmd_t;
bool guarding_mode_activate = false;

/* WIFI & MQTT VAR*/
const char *ssid = "Livebox-0209";
const char *password = "4612EA4513FE6C618F8CCF7C70";
// const char *ssid = "SER@YNOV";
// const char *password = "Q156@QeRd^YuRgRouX7T";
// const char *ssid = "HUAWEI P30 lite";
// const char *ssid = "iPhone de Julien";
// const char *password = "0123456789";
const char *mqtt_server = "broker.emqx.io";
uint16_t port = 1883;
string topic;
string message;
CALLBACK callbackStruct;

/* FSM VAR */
FSM_T fsm_t = IDLE;
FSM_T old_fsm_t = IDLE;
char *fsm;
g_fsm_flags fsm_flags;

/* SOC VAR*/
uint16_t soc_data = 0;
uint16_t old_soc = 0;
uint8_t soc = 0;
uint16_t adc_read = 0;
uint16_t div_voltage = 0;
uint16_t bat_voltage = 0;
bool publish_soc = false;

bool publish_state = false;

/* DHT SENSOR VAR */
float temperature = 0.0;
float old_temperature = 0.0;
bool publish_temperature = false;

HC_SR04_DATA_T hc_sr04_data_t;

void guard_mode();
void reset_IHM(char *dutycycle);
void treatment_publish_flags(void);

/* CLASS OBJECTS*/
WiFiClient wifiClient;
PubSubClient client(wifiClient);
ControlMotorsL298n motors(MOTOR12_PIN1, MOTOR12_PIN2, MOTOR34_PIN1, MOTOR34_PIN2, MOTORPWM12_PIN, MOTORPWM34_PIN);
ClientMQTT clientMQTT(ssid, password, mqtt_server, port, client);
WS2812B_Controller LED_strip(LED_I2C_ADDRESS, LEDS_COUNT, TYPE_GRB);
LCD_16x2 lcd(LCD_I2C_ADDRESS, LCD_COLUMN_NUMBER, LCD_LINE_NUMBER);
DHT_sensor temperature_sensor(9, DHT11);

void setup()
{
    Serial.begin(115200);

    // BUZZER INIT
    pinMode(BUZZER_PIN, OUTPUT);
    init_buzzer_sound();

    // LCD INIT
    lcd.initLCD();
    lcd.displayStringLCD("WATCHDOG BOT", 2, 0);
    lcd.displayStringLCD("... INIT ...", 2, 1);
    delay(2000);
    lcd.clearLCD();

    // DHT INIT
    temperature_sensor.initDHT();

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

    // INIT IHM
    reset_IHM("0");
    init_lcd_ihm(lcd);

#ifdef DEBUG_MODE
    Serial.print("motors obj add 0 : ");
    std::cout << &motors << std::endl;
#endif
}

void loop()
{
    publish_temperature = get_temperature_from_DHT(temperature_sensor, main_buffer_temp, lcd);
    publish_soc = get_voltage_percentage(main_buffer_soc, lcd);

    clientMQTT.loopMQTT();
    callbackStruct = clientMQTT.getCallbackReturn();
    fsm_t = roooh_static_member_reference_muste_beeeeee(fsm_t, callbackStruct);

    delay(100);

    publish_state = fsm_state(main_buffer_state, fsm_t, &fsm_flags, lcd);
    treatment_cmd_t = treatment_MQTT_Commands(callbackStruct, fsm_flags, &motors, LED_strip);

    if (fsm_t == GUARDING && !guarding_mode_activate)
    {
        guarding_mode_activate = true;
        reset_IHM("30");
    }
    if (fsm_t == MANUAL && guarding_mode_activate)
    {
        guarding_mode_activate = false;
        reset_IHM("0");
    }
    treatment_publish_flags();
    LED_managment(treatment_cmd_t, LED_strip);
    // delay(1000);
}

void guard_mode()
{
    /* TO DO */
}

void reset_IHM(char *dutycycle)
{
    clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_right");
    clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_left");
    clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_back");
    clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_forward");
    motors.stop();
    motors.setMotorsSpeed(atoi(dutycycle));
    clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_DUTYCYCLE_MOTORS_CH, dutycycle);
}

void treatment_publish_flags(void)
{
    if (publish_state)
    {
#ifdef DEBUG_MODE
        Serial.print("STATE BUFFER : ");
        Serial.println(main_buffer_state);
#endif
        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_STATE_CH, main_buffer_state);
    }
    if (publish_temperature)
        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_TEMP_CH, main_buffer_temp);
    if (publish_soc)
        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_SOC_CH, main_buffer_soc);

    if (treatment_cmd_t.ihm_managment == "turn_off")
    {
        Serial.println("STATE MOTORS : ");
        Serial.print("old_state : ");
        Serial.println(String(treatment_cmd_t.old_state_motors.c_str()));
        Serial.print("actual state : ");
        Serial.println(String(treatment_cmd_t.actual_state_motors.c_str()));
        Serial.print("ihm mana : ");
        Serial.println(String(treatment_cmd_t.ihm_managment.c_str()));
        if (treatment_cmd_t.old_state_motors == "right")
            clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_right");
        if (treatment_cmd_t.old_state_motors == "left")
            clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_left");
        if (treatment_cmd_t.old_state_motors == "back")
            clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_back");
        if (treatment_cmd_t.old_state_motors == "forward")
            clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_MOTORS_CH, "no_forward");
    }
}