#include "Components/ControlMotors/ControlMotors_L298n.hpp"
#include "Arduino.h"

ControlMotorsL298n::ControlMotorsL298n(uint8_t motor12_pin1, uint8_t motor12_pin2, uint8_t motor34_pin1, uint8_t motor34_pin2, uint8_t motorsPWM12_pin, uint8_t motorsPWM34_pin)
{
    _motor12_pin1 = motor12_pin1;
    _motor12_pin2 = motor12_pin2;
    _motor34_pin1 = motor34_pin1;
    _motor34_pin2 = motor34_pin2;
    _motorsPWM12_pin = motorsPWM12_pin;
    _motorsPWM34_pin = motorsPWM34_pin;
    _dutyCycle = 0;
    _availableMotorsSpeedConfiguration = false;
    setMotorsSpeed(_dutyCycle);
}

void ControlMotorsL298n::init(void)
{
    pinMode(_motor12_pin1, OUTPUT);
    pinMode(_motor12_pin2, OUTPUT);
    pinMode(_motor34_pin1, OUTPUT);
    pinMode(_motor34_pin2, OUTPUT);
    pinMode(_motorsPWM12_pin, OUTPUT);
    pinMode(_motorsPWM34_pin, OUTPUT);
}

void ControlMotorsL298n::setMotorsSpeed(uint8_t PWM_percentage)
{
    _dutyCycle = map(PWM_percentage, 0, 100, 90, 255);
    if (PWM_percentage > 0)
    {
        analogWrite(_motorsPWM12_pin, _dutyCycle);
        analogWrite(_motorsPWM34_pin, _dutyCycle);
        if (!_availableMotorsSpeedConfiguration)
            availableMotorsSpeedConfiguration();
    }
    else
    {
        disableMotorsSpeedConfiguration();
        stop();
    }
}

void ControlMotorsL298n::stop(void)
{
    digitalWrite(_motor12_pin1, LOW);
    digitalWrite(_motor12_pin2, LOW);
    digitalWrite(_motor34_pin1, LOW);
    digitalWrite(_motor34_pin2, LOW);
    _availableMotorsSpeedConfiguration = false;
}

void ControlMotorsL298n::goBack(void)
{
    if (_availableMotorsSpeedConfiguration)
    {
        digitalWrite(_motor12_pin1, HIGH);
        digitalWrite(_motor12_pin2, LOW);
        digitalWrite(_motor34_pin1, HIGH);
        digitalWrite(_motor34_pin2, LOW);
    }
}

void ControlMotorsL298n::goForward(void)
{
    if (_availableMotorsSpeedConfiguration)
    {
        digitalWrite(_motor12_pin1, LOW);
        digitalWrite(_motor12_pin2, HIGH);
        digitalWrite(_motor34_pin1, LOW);
        digitalWrite(_motor34_pin2, HIGH);
    }
}

void ControlMotorsL298n::goRight(void)
{
    if (_availableMotorsSpeedConfiguration)
    {
        digitalWrite(_motor12_pin1, LOW);
        digitalWrite(_motor12_pin2, HIGH);
        digitalWrite(_motor34_pin1, HIGH);
        digitalWrite(_motor34_pin2, LOW);
    }
}

void ControlMotorsL298n::goLeft(void)
{
    if (_availableMotorsSpeedConfiguration)
    {
        digitalWrite(_motor12_pin1, HIGH);
        digitalWrite(_motor12_pin2, LOW);
        digitalWrite(_motor34_pin1, LOW);
        digitalWrite(_motor34_pin2, HIGH);
    }
}

void ControlMotorsL298n::getMotorsSpeed(void)
{
    Serial.println("DutyCycle (uint) : " + String(_dutyCycle));
}

void ControlMotorsL298n::availableMotorsSpeedConfiguration(void)
{
    _availableMotorsSpeedConfiguration = true;
}

void ControlMotorsL298n::disableMotorsSpeedConfiguration(void)
{
    _availableMotorsSpeedConfiguration = false;
}