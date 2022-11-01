#include "LCD_16x2.hpp"

LCD_16x2::LCD_16x2(uint8_t address, uint8_t x, uint8_t y) : _address(address), _x(x), _y(y), _lcd(_address, _x, _y)
{
  LiquidCrystal_I2C _lcd(_address, _x, _y);
}

void LCD_16x2::initLCD(void)
{
  _lcd.init();
  _lcd.clear();
  _lcd.backlight();
  _lcd.noCursor();
  _lcd.setCursor(0, 0);
}

void LCD_16x2::displayStringLCD(string stringText, uint8_t x, uint8_t y)
{
  _lcd.setCursor(x, y);
  _lcd.print(stringText.c_str());
}

void LCD_16x2::setBacklightLCD(bool state)
{
  if (state == true)
  {
    _lcd.backlight();
  }
  else
  {
    _lcd.noBacklight();
  }
}

void LCD_16x2::clearLCD()
{
  _lcd.clear();
}

void LCD_16x2::clearLineLCD(uint8_t line)
{
  for (uint8_t column = 0; column < 15; column++)
  {
    _lcd.setCursor(column, line);
    _lcd.print(" ");
  }
  _lcd.setCursor(0, line);
}
