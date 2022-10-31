#include <Arduino.h>
#include <string>
// #include <stdio.h>
// #include <stdlib.h>
using std::string;
// #include <stdlib.h>
#include "LiquidCrystal_I2C.h"
// namespace LCD_16x2;
class LCD_16x2
{
public:
  LCD_16x2(uint8_t address, uint8_t x, uint8_t y);
  void displayStringLCD(string stringText, uint8_t x, uint8_t y);
  void setBacklightLCD(bool state);
  void clearLCD();
  void clearLineLCD(uint8_t line);
  void initLCD();

private:
  // LiquidCrystal_I2C lcd;

  uint8_t _address;
  string _stringText;
  uint8_t _x;
  uint8_t _y;
  LiquidCrystal_I2C _lcd;
  bool _backlightState;
};