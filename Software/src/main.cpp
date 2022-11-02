#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "Wire.h"
#include <WiFi.h>
#include <string>
#include <Adafruit_Sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include "Components/ClientMQTT/ClientMQTT.hpp"
#include "Components/PubSubClient/PubSubClient.hpp"
#include "Components/Global/Global.hpp"



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

void reset_IHM(char *dutycycle, string state);
void treatment_publish_flags(void);
bool guard_mode();
void manageObstable();

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

    pinMode(HC_SR04_TRIGGER_PIN,      OUTPUT);
    pinMode(HC_SR04_ECHO_PIN,         INPUT);
    digitalWrite(HC_SR04_TRIGGER_PIN, LOW);

    // WIFI INIT
    Serial.setTimeout(500);
    clientMQTT.setupWifiMQTT();

    // MQTT CLIENT INIT
    clientMQTT.setServerMQTT();
    clientMQTT.setCallbackMQTT();
    clientMQTT.reconnectMQTT();

    // INIT IHM
    reset_IHM("0", "start");
    init_lcd_ihm(lcd);

#ifdef DEBUG_MODE
    Serial.print("motors obj add 0 : ");
    std::cout << &motors << std::endl;
#endif
}

void loop()
{
    // clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_PUBLISH_CH, std::to_string(hcsr04_get_distance("right")));
    publish_temperature = get_temperature_from_DHT(temperature_sensor, main_buffer_temp, lcd);
    publish_soc = get_voltage_percentage(main_buffer_soc, lcd);

    clientMQTT.loopMQTT();
    callbackStruct = clientMQTT.getCallbackReturn();
    if (fsm_t != DETECTED)
    {
        fsm_t = roooh_static_member_reference_muste_beeeeee(fsm_t, callbackStruct);
    }

    delay(100);

    publish_state = fsm_state(main_buffer_state, fsm_t, &fsm_flags, lcd);
    treatment_cmd_t = treatment_MQTT_Commands(callbackStruct, fsm_flags, &motors, LED_strip);

    if (fsm_t == GUARDING && !guarding_mode_activate)
    {
        guarding_mode_activate = true;
        reset_IHM("40", "guard");
    }
    if (fsm_t == MANUAL && guarding_mode_activate)
    {
        guarding_mode_activate = false;
        reset_IHM("0", "manual");
    }
    // Serial.println("Before treatment_publish_flags");
    treatment_publish_flags();
    // Serial.println("After treatment_publish_flags");

    LED_managment(treatment_cmd_t, LED_strip);
    // delay(1000);
    // Serial.println("E");
    if (fsm_t == GUARDING)
    {
        Serial.println("ENTERING fsm_t == GUARDING");
        bool detected = guard_mode();
        Serial.print("detected state : ");
        Serial.println(detected);
        if (detected != 0){
            LED_strip.policeGang();
            fsm_t = DETECTED;
        }
        else{
            fsm_t = GUARDING;
            // LED_strip.turnOFF();
            clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_SECURITY_LOGGER_CH, "RAS");


        }
        Serial.println("EXITING fsm_t == GUARDING");
    }
    if (fsm_t == DETECTED){
        LED_strip.policeGang();
        police_BIP();
        motors.stop();
        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_SECURITY_LOGGER_CH, "DETECTED");
    }
}

uint16_t STEP_TIME = 3;
uint16_t DEG_90_TIME = 350;
uint16_t HALF_TURN_TIME = (DEG_90_TIME*2);
uint16_t DETECTION_TRESHOLD = 200;
uint16_t ANGLE_VALUE = 90;
uint16_t BACKING_TIME = 300;

bool isObstacleDetected(void){
  if (hcsr04_get_distance() <= DETECTION_TRESHOLD){
      return true;
  }
  else {
    return false;
  }
}



bool guard_mode()
{
     /* TO DO */
    bool detected = false;
    motors.setMotorsSpeed(40);
    motors.goForward();
    Serial.println("ENTERING GUARD FUNCTION");
    if(isObstacleDetected()){
        
        manageObstable();
        int duration = 0;
        delay(6000);
        int i = 0;
        while(i<1000){
            detected = hcsr501_get_data();
            Serial.print("while : ");
            Serial.println(detected);
            if(detected){
                Serial.print("if detected : ");
                Serial.println(detected);
                break;
            }
            delay(1);
            i++;
        }
    }
    else{
        
        
    }
    Serial.println("EXITING GUARD FUNCTION");
    Serial.print("detected state : ");
    Serial.println(detected);
    return detected;
}

void manageObstable(){
    motors.stop();
    motors.setMotorsSpeed(20);
    motors.goBack();
    delay(BACKING_TIME);
    motors.stop();
    motors.setMotorsSpeed(80);
    motors.goLeft();
    delay(HALF_TURN_TIME);
    motors.stop();

    motors.goLeft();
    delay(DEG_90_TIME);
    motors.stop();

    uint16_t ultraSound[90];
    for(int i = 0; i<89;i++){
      motors.setMotorsSpeed(100);
      motors.goRight();
      delay(STEP_TIME);
      motors.stop();
      ultraSound[i]=hcsr04_get_distance();

    }


    // motors.goLeft();
    // delay(DEG_90_TIME/2);
    // motors.stop();

  uint16_t highest_distance = 0;
  uint16_t highest_distance_index = 0;


  for(int scan = 0;scan<89;scan++){
    if(ultraSound[scan]>highest_distance){
      highest_distance=ultraSound[scan];
      highest_distance_index=scan;
      Serial.print("LOOP scan : ");
      Serial.println(scan);
    }
  }
  Serial.print("HIGHEST DISTANCE : ");
  Serial.println(highest_distance);
  Serial.print("HIGHEST INDEX : ");
  Serial.println(highest_distance_index);
//   motors.setMotorsSpeed(50);
//   motors.goRight();
//   delay(DEG_90_TIME*(highest_distance_index/90));
    for(int step = 89; step > (89-highest_distance_index); step--){
      motors.setMotorsSpeed(100);
      motors.goRight();
    //   Serial.println("After right");
      delay(STEP_TIME);
      motors.stop();
    }
    // Serial.println("EXITING FOR LOOP");
    motors.setMotorsSpeed(20);
}

void reset_IHM(char *dutycycle, string state)
{
    if (state=="start")
    {
        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_GUARD_CH, "guard_off");
    }
    else{
        /* do nothing */
    }
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
    if(!(fsm_t == GUARDING)){
    if (publish_temperature)
        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_TEMP_CH, main_buffer_temp);
    if (publish_soc)
        clientMQTT.publishSerialDataMQTT(MQTT_SERIAL_SOC_CH, main_buffer_soc);
    }
}