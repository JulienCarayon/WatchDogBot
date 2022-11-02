#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "Components/DHT_sensor/DHT_sensor.hpp"
#include "Components/LCD_16x2/LCD_16x2.hpp"
#include "Components/ControlMotors/ControlMotors_L298n.hpp"
#include "Components/WS2812B_LED_Controller/WS2812B_LED_Controller.hpp"

using namespace std;

#undef DEBUG_MODE

#define BUZZER_PIN 15

#define HC_SR501_PIN 23

#define HC_SR04_TRIGGER_PIN 26
#define HC_SR04_ECHO_PIN 27
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

#define MOTOR12_PIN1 16
#define MOTOR12_PIN2 17
#define MOTOR34_PIN1 18
#define MOTOR34_PIN2 5
#define MOTORPWM12_PIN 4
#define MOTORPWM34_PIN 19

typedef struct
{
    float distance_right;
    float distance_left;
} HC_SR04_DATA_T;

typedef enum
{
    IDLE = 0,
    MANUAL = 1,
    GUARDING = 2,
    DETECTED = 3,
    ERROR = 4
} FSM_T;

typedef struct
{
    bool idleState;
    bool manualState;
    bool guardingState;
    bool detectedState;
    bool errorState;
} g_fsm_flags;

struct CALLBACK
{
    bool new_message;
    string topic;
    string message;
    FSM_T fsm_state;
};

typedef struct
{
    bool first_movement = false;
    string old_state_motors;
    string actual_state_motors;
    string ihm_managment;
    bool go_right;
    bool go_left;
    bool go_back;
    bool go_forward;
    bool stop;
} TREATMENT_CMD_T;

const string state_mode[5] = {
    "IDLE",
    "MANUAL",
    "GUARDING",
    "DETECTED",
    "ERROR"
};

/* HC-SR04*/
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const float SOUND_SPEED = 340.0 / 1000;

/* LCD */
const string mode = " MODE";

string convertToString(char *character_array, int size);
void init_buzzer_sound(void);
void debug_buzzer_sound(void);
void buzzer_sound_reverse_beep(void);
void buzzer_off(void);
void police_BIP(void);
void init_lcd_ihm(LCD_16x2 lcd);
bool hcsr501_get_data(void);
bool get_temperature_from_DHT(DHT_sensor dht_sensor, char *main_buffer, LCD_16x2 lcd);
bool get_voltage_percentage(char *main_buffer, LCD_16x2 lcd);
float hcsr04_get_distance();
FSM_T roooh_static_member_reference_muste_beeeeee(FSM_T fsm_t, CALLBACK callbackReturnClass);
bool fsm_state(char *main_buffer_state, FSM_T fsm, g_fsm_flags *fsm_flags, LCD_16x2 lcd);
TREATMENT_CMD_T control_motors(string message, ControlMotorsL298n *motors, WS2812B_Controller LED_strip);
TREATMENT_CMD_T treatment_MQTT_Commands(CALLBACK callbackStruct, g_fsm_flags fsm_flags, ControlMotorsL298n *motors, WS2812B_Controller LED_strip);
void LED_managment(TREATMENT_CMD_T treatment_cmd, WS2812B_Controller LED_strip);

#endif