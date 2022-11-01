#include <global.hpp>
#include <Arduino.h>
#include <iostream>

string convertToString(char *character_array, int size)
{
    string convert_string = "";
    for (int i = 0; i < size; i++)
    {
        convert_string += character_array[i];
    }
    return convert_string;
}

void debug_buzzer_sound(void)
{
    tone(BUZZER_PIN, 600);
    delay(50);
    tone(BUZZER_PIN, 0);
    delay(50);
    tone(BUZZER_PIN, 600);
    delay(50);
    tone(BUZZER_PIN, 0);
}

void init_buzzer_sound(void)
{
    tone(BUZZER_PIN, 700);
    delay(100);
    tone(BUZZER_PIN, 0);
    delay(20);
    tone(BUZZER_PIN, 700);
    delay(50);
    tone(BUZZER_PIN, 0);
}

void buzzer_sound_reverse_beep(void)
{
    tone(BUZZER_PIN, 700);
    delay(100);
    tone(BUZZER_PIN, 0);
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

void init_lcd_ihm(LCD_16x2 lcd)
{
    lcd.displayStringLCD("C", 5, 1);
    lcd.displayStringLCD("%", 15, 1);
}

bool hcsr501_get_data(void)
{
    return (bool)digitalRead(HC_SR501_PIN);
}

bool get_temperature_from_DHT(DHT_sensor dht_sensor, char *main_buffer, LCD_16x2 lcd)
{
    char local_buffer[5];
    static float old_temperature = 0.0;
    float temperature = dht_sensor.get_temperature_sensor();
    bool publish_temperature = false;
    if (temperature > (old_temperature + 0.2) || temperature < (old_temperature - 0.2))
    {
        old_temperature = temperature;
        publish_temperature = true;

        int ret = snprintf(local_buffer, sizeof(local_buffer), "%f", temperature);
        strcpy(main_buffer, local_buffer);

        lcd.displayStringLCD((string)main_buffer, 0, 1);
#ifdef DEBUG_MODE
        debug_buzzer_sound();
#endif
    }
    else
    {
        publish_temperature = false;
    }
    return publish_temperature;
}

bool get_voltage_percentage(char *main_buffer, LCD_16x2 lcd)
{
    char local_buffer[10];
    uint16_t adc_read = analogRead(BAT_PIN);
    uint16_t div_voltage = (uint16_t)map(adc_read, 0, 4095, 0, 3300);
    uint16_t bat_voltage = (div_voltage * 1500) / 500;
    bool publish_soc = false;
    static uint16_t old_soc = 0;

    uint8_t soc = (uint8_t)map(bat_voltage, 0, 8400, 0, 100);

    if (soc > (old_soc + 2) || soc < (old_soc - 2))
    {
        publish_soc = true;
        old_soc = soc;
#ifdef DEBUG_MODE
        Serial.println("NEW SOC : " + soc);
#endif

        int ret = snprintf(local_buffer, sizeof(local_buffer), "%d", soc);
        strcpy(main_buffer, local_buffer);
#ifdef DEBUG_MODE
        debug_buzzer_sound();
#endif
        lcd.displayStringLCD((string)local_buffer, 13, 1);
        return publish_soc;
    }
    else
    {
        publish_soc = false;
    }
    return publish_soc;
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

        measure = pulseIn(HC_SR04_ECHO_PIN_RIGHT, HIGH, MEASURE_TIMEOUT);
    }

    // Calculing the distance
    float distance_mm = measure / 2.0 * SOUND_SPEED;

#ifdef DEBUG_MODE
    Serial.print(F("Distance: "));
    Serial.print(distance_mm);
    Serial.print(F("mm ("));
    Serial.print(distance_mm / 10.0, 2);
    Serial.print(F("cm, "));
    Serial.print(distance_mm / 1000.0, 2);
    Serial.println(F("m)"));
#endif

    return (uint8_t)distance_mm;
}

FSM_T roooh_static_member_reference_muste_beeeeee(FSM_T fsm_t, CALLBACK callbackReturnClass)
{
    static bool guardModeAcivate = false;

    if (callbackReturnClass.topic == "DOG/DUTYCYCLE")
    {
        if (!guardModeAcivate)
        {
            fsm_t = MANUAL;
        }
    }
    else if (callbackReturnClass.topic == "DOG/MOTORS")
    {
#ifdef DEBUG_MODE
        Serial.println("*** FROM TOPIC [DOG/MOTORS] ***");
#endif
        if (!guardModeAcivate)
            fsm_t = MANUAL;
    }
    else if (callbackReturnClass.topic == "DOG/GUARD")
    {
#ifdef DEBUG_MODE
        Serial.println("*** FROM TOPIC [DOG/GUARD] ***");
#endif
        if (callbackReturnClass.message == "guard_on")
        {
            fsm_t = GUARDING;
            guardModeAcivate = true;
        }
        else if (callbackReturnClass.message == "guard_off")
        {
            fsm_t = MANUAL;
            guardModeAcivate = false;
        }
    }
    else if (callbackReturnClass.topic == "DOG/SECURITY")
    {
#ifdef DEBUG_MODE
        Serial.println("*** FROM TOPIC [DOG/SECURITY] ***");
#endif
        if (guardModeAcivate)
            fsm_t = DETECTED;
    }

    return fsm_t;
}

bool fsm_state(char *main_buffer_state, FSM_T fsm, g_fsm_flags *fsm_flags, LCD_16x2 lcd)
{
    char local_buffer[64];
    static string display_state_lcd = "";
    static char *current_state;
    static char *old_state;
    bool publish_new_state = false;

    switch (fsm)
    {
    case IDLE:
        fsm_flags->idleState = true;
        fsm_flags->manualState = false;
        fsm_flags->guardingState = false;
        fsm_flags->detectedState = false;
        fsm_flags->errorState = false;
        display_state_lcd = state_mode[0] + mode;
        current_state = "IDLE";
        break;

    case MANUAL:
        fsm_flags->idleState = false;
        fsm_flags->manualState = true;
        fsm_flags->guardingState = false;
        fsm_flags->detectedState = false;
        fsm_flags->errorState = false;
        display_state_lcd = state_mode[1] + mode;
        lcd.displayStringLCD(display_state_lcd, 0, 0);
        current_state = "MANUAL";
        break;

    case GUARDING:
        fsm_flags->idleState = false;
        fsm_flags->manualState = false;
        fsm_flags->guardingState = true;
        fsm_flags->detectedState = false;
        fsm_flags->errorState = false;
        display_state_lcd = state_mode[2] + mode;
        current_state = "GUARDING";
        break;

    case DETECTED:
        fsm_flags->idleState = false;
        fsm_flags->manualState = false;
        fsm_flags->guardingState = false;
        fsm_flags->detectedState = true;
        fsm_flags->errorState = false;
        display_state_lcd = state_mode[3] + mode;
        current_state = "DETECTED";
        break;

    case ERROR:
        fsm_flags->idleState = false;
        fsm_flags->manualState = false;
        fsm_flags->guardingState = false;
        fsm_flags->detectedState = false;
        fsm_flags->errorState = true;
        display_state_lcd = state_mode[4] + mode;
        current_state = "ERROR";
        break;

    default:
        fsm = IDLE;
        break;
    }

    if (current_state != old_state)
    {
        old_state = current_state;
        publish_new_state = true;
        lcd.clearLineLCD(0);
        lcd.displayStringLCD(display_state_lcd, 0, 0);

        int ret = snprintf(local_buffer, sizeof(local_buffer), "%s", current_state);
        strcpy(main_buffer_state, local_buffer);

        return publish_new_state;
    }
    else
    {
        publish_new_state = false;
        return publish_new_state;
    }
}

TREATMENT_CMD_T treatment_MQTT_Commands(CALLBACK callbackStruct, g_fsm_flags fsm_flags, ControlMotorsL298n *motors, WS2812B_Controller LED_strip)
{
    static uint8_t old_dutyCycle = 0;
    static TREATMENT_CMD_T treatment_cmd;
    static string old_message = "";

    if (fsm_flags.manualState)
    {
#ifdef DEBUG_MODE
        Serial.println("MANUAL MODE");
#endif
        if (callbackStruct.topic == "DOG/DUTYCYCLE")
        {
#ifdef DEBUG_MODE
            Serial.println("old  : " + old_dutyCycle);
            Serial.println("msg  : " + String(callbackStruct.message.c_str()));
#endif
            if (old_dutyCycle != atoi(callbackStruct.message.c_str()))
            {
#ifdef DEBUG_MODE
                Serial.print("SET NEW MOTORS SPEED ....... ");
#endif
                old_dutyCycle = atoi(callbackStruct.message.c_str());
                motors->setMotorsSpeed(atoi(callbackStruct.message.c_str()));
#ifdef DEBUG_MODE
                Serial.println("----> NEW MOTOR SPEED SET");
#endif
            }
        }
        if (callbackStruct.topic == "DOG/MOTORS")
        {
#ifdef DEBUG_MODE
            Serial.print("motors obj add 1 : ");
            std::cout << &motors << std::endl;
#endif
            if (old_message != callbackStruct.message)
            {
#ifdef DEBUG_MODE
                Serial.print("SET NEW MOTORS SPEED ....... ");
#endif
                old_message == callbackStruct.message;
                treatment_cmd = control_motors(callbackStruct.message, motors, LED_strip);
            }
        }
    }
    return treatment_cmd;
}

TREATMENT_CMD_T control_motors(string message, ControlMotorsL298n *motors, WS2812B_Controller LED_strip)
{
#ifdef DEBUG_MODE
    Serial.print("motors obj add 2 : ");
    std::cout << &motors << std::endl;
#endif

    // 1 = right
    //          ret = stop
    //          old_mes = right
    // 2 = left
    //          ret = right
    //          old_mes = left
    // 3 - left
    //          ret = left
    //          old_mes = left
    // 3 - back
    //          ret = left
    //          old_mes = back

    static TREATMENT_CMD_T treatment_cmd;
    Serial.println("COUCOU LES LOULOUS ");

    static string old_message = "init";
    static string return_old_message = "";
    static string temp = "";

    if (message == "right")
    {
        motors->goRight();
        LED_strip.blinkingRight();
        return_old_message = old_message;
        old_message = message;
        treatment_cmd.actual_state_motors = "right";
        treatment_cmd.go_right = true;
        treatment_cmd.go_left = false;
        treatment_cmd.go_back = false;
        treatment_cmd.go_forward = false;
        treatment_cmd.stop = false;
    }
    else if (message == "left")
    {
        motors->goLeft();
        LED_strip.blinkingLeft();
        return_old_message = old_message;
        old_message = message;
        treatment_cmd.actual_state_motors = "left";
        treatment_cmd.go_right = false;
        treatment_cmd.go_left = true;
        treatment_cmd.go_back = false;
        treatment_cmd.go_forward = false;
        treatment_cmd.stop = false;
    }

    else if (message == "back")
    {
        motors->goBack();
        LED_strip.warning();
        return_old_message = old_message;
        old_message = message;
        treatment_cmd.actual_state_motors = "back";
        treatment_cmd.go_right = false;
        treatment_cmd.go_left = false;
        treatment_cmd.go_back = true;
        treatment_cmd.go_forward = false;
        treatment_cmd.stop = false;
    }
    else if (message == "forward")
    {
        motors->goForward();
        return_old_message = old_message;
        old_message = message;
        treatment_cmd.actual_state_motors = "forward";
        treatment_cmd.go_right = false;
        treatment_cmd.go_left = false;
        treatment_cmd.go_back = false;
        treatment_cmd.go_forward = true;
        treatment_cmd.stop = false;
    }
    else if (message == "stop")
    {
        motors->stop();
        treatment_cmd.go_right = false;
        treatment_cmd.go_left = false;
        treatment_cmd.go_back = false;
        treatment_cmd.go_forward = false;
        treatment_cmd.stop = true;
    }
    return_old_message = old_message;
    old_message = message;
    if (old_message != return_old_message)
    {
        treatment_cmd.ihm_managment = "turn_off";
        treatment_cmd.old_state_motors = return_old_message;
        return_old_message = old_message;

        if (message.find("no") != std::string::npos)
        {
            treatment_cmd.ihm_managment = "turn_off";
            motors->stop();
            treatment_cmd.go_right = false;
            treatment_cmd.go_left = false;
            treatment_cmd.go_back = false;
            treatment_cmd.go_forward = false;
            treatment_cmd.stop = true;
        }
    }
    else
    {
        treatment_cmd.ihm_managment = "do_nothing";
    }

    return treatment_cmd;
}

void LED_managment(TREATMENT_CMD_T treatment_cmd, WS2812B_Controller LED_strip)
{
    if (treatment_cmd.go_left)
        LED_strip.blinkingLeft();
    if (treatment_cmd.go_right)
        LED_strip.blinkingRight();
    if (treatment_cmd.go_back)
    {
        LED_strip.warning();
        buzzer_sound_reverse_beep();
    }
    if (treatment_cmd.go_forward)
    {
        LED_strip.turnOFF(0, 1);
        LED_strip.frontCarHeadlight();
    }

    if (treatment_cmd.stop)
    {
        LED_strip.frontCarHeadlight();
        LED_strip.backCarHeadlight();
    }
}
