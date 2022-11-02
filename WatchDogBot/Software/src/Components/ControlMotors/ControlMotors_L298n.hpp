#ifndef ControlMotors_L298N_H
#define ControlMotors_L298N_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <iostream>
#include <string>

using namespace std;

class ControlMotorsL298n
{
public:
    ControlMotorsL298n(uint8_t motor12_pin1, uint8_t motor12_pin2, uint8_t motor34_pin1, uint8_t motor34_pin2, uint8_t motorsPWM12_pin, uint8_t motorsPWM34_pin);

    void init(void);
    void stop(void);
    void goLeft(void);
    void goBack(void);
    void goRight(void);
    void goForward(void);
    void setMotorsSpeed(uint8_t PWM_percentage);
    void getMotorsSpeed(void);
    void availableMotorsSpeedConfiguration(void);
    void disableMotorsSpeedConfiguration(void);

private:
    uint8_t _motor12_pin1;
    uint8_t _motor12_pin2;
    uint8_t _motor34_pin1;
    uint8_t _motor34_pin2;
    uint8_t _motorsPWM12_pin;
    uint8_t _motorsPWM34_pin;
    uint8_t _dutyCycle;
    bool _availableMotorsSpeedConfiguration;
};

#endif