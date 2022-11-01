#include "Components/WS2812B_LED_Controller/WS2812B_LED_Controller.hpp"
#include "Arduino.h"
#include <stdbool.h>

int WS2812B_Controller::writeRegOneByte(uint8_t val)
{
    Wire.beginTransmission(I2C_Address);
    Wire.write(val);
    int error = Wire.endTransmission();
    return error;
}

int WS2812B_Controller::writeReg(uint8_t cmd, uint8_t *value, uint8_t size_a)
{
    Wire.beginTransmission(I2C_Address);
    Wire.write(cmd);
    Wire.write(value, size_a);
    int error = Wire.endTransmission();
    return error;
}

int WS2812B_Controller::readReg(uint8_t cmd, char *recv, uint16_t count)
{
    // Wire.beginTransmission(I2C_Address);
    // Wire.write(cmd);
    // Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, count);
    // Wire.requestFrom(I2C_Address, count, cmd, 1, true);

    int i = 0;
    while (Wire.available())
    {
        recv[i++] = Wire.read();
        if (i == count)
        {
            break;
        }
    }
    int error = Wire.endTransmission();
    return error;
}

bool WS2812B_Controller::uartWriteDataToControllerWithAck(uint8_t param[5], bool isShowLed)
{
    _serial->flush();
    while (_serial->available() > 0)
    {
        _serial->read();
    }
    uint8_t arr[8] = {UART_START_BYTE, param[0], param[1], param[2], param[3], param[4], UART_END_BYTE, 0};
    arr[7] = (uint8_t)(arr[0] + arr[1] + arr[2] + arr[3] + arr[4] + arr[5] + arr[6]);
    _serial->write(arr, sizeof(arr));
    if (isShowLed)
    {
        if (uartWaitAckTime > 60000)
        {
            delay(uartWaitAckTime / 1000 + 1);
        }
        else
        {
            delayMicroseconds(uartWaitAckTime);
        }
    }
    else
    {
        delayMicroseconds(2000);
    }

    uint8_t ack;
    while (_serial->available() > 0)
    {
        ack = _serial->read();
    }
    if (ack == UART_ACK_BYTE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

WS2812B_Controller::WS2812B_Controller(uint8_t _address, uint16_t n, LED_TYPE t)
{
    commMode = I2C_COMMUNICATION_MODE;
    I2C_Address = (uint8_t)_address;
    ledCounts = n;
    uartWaitAckTime = (280 + ledCounts * 53) + 3000;
    setLedType(t);
}

WS2812B_Controller::WS2812B_Controller(HardwareSerial *serial_param, uint16_t n, LED_TYPE t, uint32_t _baudrate)
{
    uartBaudrateIndex = 0;
    for (int i = 0; i < 13; i++)
    {
        if (_BAUDRATE[i] == _baudrate)
        {
            uartBaudrateIndex = i;
            break;
        }
    }
    commMode = UART_COMMUNICATION_MODE;
    ledCounts = n;
    uartWaitAckTime = (280 + ledCounts * 53) + 3000;
    _serial = serial_param;
    setLedType(t);
}

bool WS2812B_Controller::begin()
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        Wire.begin();
    }
    else
    {
        _serial->begin(_BAUDRATE[uartBaudrateIndex]);
    }

    return setLedCount(ledCounts);
}

bool WS2812B_Controller::setLedCount(uint16_t n)
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        uint8_t arr[] = {n};
        return !writeReg(REG_LEDS_COUNTS, arr, sizeof(arr));
    }
    else
    {
        uint8_t arr[] = {REG_LEDS_COUNTS, n, 0, 0, 0};
        return uartWriteDataToControllerWithAck(arr);
    }
}

void WS2812B_Controller::setLedType(LED_TYPE t)
{
    rOffset = (t >> 4) & 0x03;
    gOffset = (t >> 2) & 0x03;
    bOffset = t & 0x03;
}

bool WS2812B_Controller::setLedColorData(uint8_t index, uint32_t rgb)
{
    return setLedColorData(index, rgb >> 16, rgb >> 8, rgb);
}

bool WS2812B_Controller::setLedColorData(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t p[3];
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        uint8_t arr[] = {index, p[0], p[1], p[2]};
        return !writeReg(REG_SET_LED_COLOR_DATA, arr, sizeof(arr));
    }
    else
    {
        uint8_t arr[] = {REG_SET_LED_COLOR_DATA, index, p[0], p[1], p[2]};
        return uartWriteDataToControllerWithAck(arr);
    }
}

bool WS2812B_Controller::setLedColor(uint8_t index, uint32_t rgb)
{
    return setLedColor(index, rgb >> 16, rgb >> 8, rgb);
    ;
}

bool WS2812B_Controller::setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t p[3];
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        uint8_t arr[] = {index, p[0], p[1], p[2]};
        return !writeReg(REG_SET_LED_COLOR, arr, sizeof(arr));
    }
    else
    {
        uint8_t arr[] = {REG_SET_LED_COLOR, index, p[0], p[1], p[2]};
        return uartWriteDataToControllerWithAck(arr, true);
    }
}

bool WS2812B_Controller::setAllLedsColorData(uint32_t rgb)
{
    setAllLedsColorData(rgb >> 16, rgb >> 8, rgb);
}

bool WS2812B_Controller::setAllLedsColorData(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t p[3];
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        uint8_t arr[] = {p[0], p[1], p[2]};
        return !writeReg(REG_SET_ALL_LEDS_COLOR_DATA, arr, sizeof(arr));
    }
    else
    {
        uint8_t arr[] = {REG_SET_ALL_LEDS_COLOR_DATA, p[0], p[1], p[2], 0};
        return uartWriteDataToControllerWithAck(arr);
    }
}

bool WS2812B_Controller::setAllLedsColor(uint32_t rgb)
{
    setAllLedsColor(rgb >> 16, rgb >> 8, rgb);
}

bool WS2812B_Controller::setAllLedsColor(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t p[3];
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        uint8_t arr[] = {p[0], p[1], p[2]};
        return !writeReg(REG_SET_ALL_LEDS_COLOR, arr, sizeof(arr));
    }
    else
    {
        uint8_t arr[] = {REG_SET_ALL_LEDS_COLOR, p[0], p[1], p[2], 0};
        return uartWriteDataToControllerWithAck(arr, true);
    }
}

bool WS2812B_Controller::show()
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        return !writeRegOneByte(REG_TRANS_DATA_TO_LED);
    }
    else
    {
        uint8_t arr[] = {REG_TRANS_DATA_TO_LED, 0, 0, 0, 0};
        return uartWriteDataToControllerWithAck(arr, true);
    }
}

uint8_t WS2812B_Controller::getLedsCountFromController()
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        char recv[1];
        readReg(REG_LEDS_COUNT_READ, recv, 1);
        return recv[0];
    }
    else
    {
        _serial->flush();
        while (_serial->available() > 0)
        {
            _serial->read();
        }
        uint8_t arr[8] = {UART_START_BYTE, REG_LEDS_COUNT_READ, 0, 0, 0, 0, UART_END_BYTE, 0};
        arr[7] = (uint8_t)(arr[0] + arr[1] + arr[2] + arr[3] + arr[4] + arr[5] + arr[6]);
        _serial->write(arr, sizeof(arr));
        delay(10);
        uint8_t recv = 0;
        while (_serial->available() > 0)
        {
            recv = _serial->read();
        }
        return recv;
    }
}

uint8_t WS2812B_Controller::getI2CAddress()
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        return I2C_Address;
    }
    else
    {
        _serial->flush();
        while (_serial->available() > 0)
        {
            _serial->read();
        }
        uint8_t arr[8] = {UART_START_BYTE, REG_READ_I2C_ADDRESS, 0, 0, 0, 0, UART_END_BYTE, 0};
        arr[7] = (uint8_t)(arr[0] + arr[1] + arr[2] + arr[3] + arr[4] + arr[5] + arr[6]);
        _serial->write(arr, sizeof(arr));
        delay(10);
        uint8_t i2c_Address;
        while (_serial->available() > 0)
        {
            i2c_Address = _serial->read();
        }
        return i2c_Address; // 7bit Address
    }
}

uint32_t WS2812B_Controller::getUartBaudrate()
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        char recv[1];
        readReg(REG_GET_UART_BAUDRATE, recv, 1);
        return _BAUDRATE[recv[0]];
    }
    else
    {
        return _BAUDRATE[uartBaudrateIndex];
    }
}

bool WS2812B_Controller::setUartBaudrate(uint32_t _baudrate)
{
    for (int i = 0; i < 13; i++)
    {
        if (_BAUDRATE[i] == _baudrate)
        {
            if (commMode == I2C_COMMUNICATION_MODE)
            {
                uint8_t arr[] = {SECTET_KEY_A, SECTET_KEY_B, i};
                return !writeReg(REG_SET_UART_BAUDRATE, arr, sizeof(arr));
            }
            else
            {
                uint8_t arr[] = {REG_SET_UART_BAUDRATE, SECTET_KEY_A, SECTET_KEY_B, i, 0};
                return uartWriteDataToControllerWithAck(arr);
            }
            break;
        }
    }
}

bool WS2812B_Controller::setI2CNewAddress(uint8_t addr)
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        uint8_t arr[] = {SECTET_KEY_A, SECTET_KEY_B, addr};
        if (writeReg(REG_SET_I2C_ADDRESS, arr, sizeof(arr)) == 0)
        {
            I2C_Address = addr;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        uint8_t arr[] = {REG_SET_I2C_ADDRESS, SECTET_KEY_A, SECTET_KEY_B, addr, 0};
        if (uartWriteDataToControllerWithAck(arr))
        {
            I2C_Address = addr;
            return true;
        }
        else
        {
            return false;
        }
    }
}

String WS2812B_Controller::getBrand()
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        char recv[STRING_BRAND_LENGTH];
        readReg(REG_GET_BRAND, recv, STRING_BRAND_LENGTH);
        return String(recv);
    }
    else
    {
        _serial->flush();
        while (_serial->available() > 0)
        {
            _serial->read();
        }
        uint8_t arr[8] = {UART_START_BYTE, REG_GET_BRAND, 0, 0, 0, 0, UART_END_BYTE, 0};
        arr[7] = (uint8_t)(arr[0] + arr[1] + arr[2] + arr[3] + arr[4] + arr[5] + arr[6]);
        _serial->write(arr, sizeof(arr));
        delay(10);
        char brandChar[STRING_BRAND_LENGTH] = {0};
        uint8_t i = 0;
        while (_serial->available() > 0)
        {
            brandChar[i++] = _serial->read();
            if (i == STRING_BRAND_LENGTH)
            {
                break;
            }
        }
        return String(brandChar);
    }
}

String WS2812B_Controller::getFirmwareVersion()
{
    if (commMode == I2C_COMMUNICATION_MODE)
    {
        char recv[STRING_VERSION_LENGTH];
        readReg(REG_GET_FIRMWARE_VERSION, recv, STRING_VERSION_LENGTH);
        return String(recv);
    }
    else
    {
        _serial->flush();
        while (_serial->available() > 0)
        {
            _serial->read();
        }
        uint8_t arr[8] = {UART_START_BYTE, REG_GET_FIRMWARE_VERSION, 0, 0, 0, 0, UART_END_BYTE, 0};
        arr[7] = (uint8_t)(arr[0] + arr[1] + arr[2] + arr[3] + arr[4] + arr[5] + arr[6]);
        _serial->write(arr, sizeof(arr));
        delay(10);
        char brandChar[STRING_VERSION_LENGTH] = {0};
        uint8_t i = 0;
        while (_serial->available() > 0)
        {
            brandChar[i++] = _serial->read();
            if (i == STRING_VERSION_LENGTH)
            {
                break;
            }
        }
        return String(brandChar);
    }
}

uint32_t WS2812B_Controller::Wheel(byte pos)
{
    uint32_t WheelPos = pos % 0xff;
    if (WheelPos < 85)
    {
        return ((255 - WheelPos * 3) << 16) | ((WheelPos * 3) << 8);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return (((255 - WheelPos * 3) << 8) | (WheelPos * 3));
    }
    WheelPos -= 170;
    return ((WheelPos * 3) << 16 | (255 - WheelPos * 3));
}

void WS2812B_Controller::WeWillFuckYou(void)
{
    delay(100);
    for (int i = 0; i < LEDS_COUNT; i++)
    {
        setLedColor(i, 255, 0, 0);
        show();
    }
    delay(100);
    for (int i = 0; i < LEDS_COUNT; i++)
    {
        setLedColor(i, 0, 0, 0);
        show();
    }
}

void WS2812B_Controller::blinkingLeft(void)
{
    int j = 9;
    for (int i = 9; i >= 5; i--)
    {
        setLedColor(i, 255, 140, 0);
        setLedColor(i - j, 255, 140, 0);
        show();

        delay(50);
        j -= 2;
    }

    j = 9;
    for (int i = 0; i <= 5; i++)
    {
        setLedColor(i, 0, 0, 0);
        setLedColor(i + j, 0, 0, 0);
        show();

        delay(50);
        j -= 2;
    }
}

void WS2812B_Controller::blinkingRight(void)
{
    int j = 1;

    for (int i = 5; i <= 9; i++)
    {
        setLedColor(i, 255, 140, 0);
        setLedColor(i - j, 255, 140, 0);
        show();

        delay(50);
        j += 2;
    }

    j = 1;
    for (int i = 5; i <= 9; i++)
    {
        setLedColor(i, 0, 0, 0);
        setLedColor(i - j, 0, 0, 0);
        show();

        delay(50);
        j += 2;
    }
}

void WS2812B_Controller::warning(void)
{
    int j = 9;
    for (int i = 9; i >= 8; i--)
    {
        setLedColor(i, 255, 140, 0);
        setLedColor(i - j, 255, 140, 0);
        show();

        delay(50);
        j -= 2;
    }

    j = 1;

    for (int i = 5; i <= 6; i++)
    {
        setLedColor(i, 255, 140, 0);
        setLedColor(i - j, 255, 140, 0);
        show();

        delay(50);
        j += 2;
    }

    j = 9;
    for (int i = 0; i <= 2; i++)
    {
        setLedColor(i, 0, 0, 0);
        setLedColor(i + j, 0, 0, 0);
        show();

        delay(50);
        j -= 2;
    }
    j = 1;
    for (int i = 5; i <= 6; i++)
    {
        setLedColor(i, 0, 0, 0);
        setLedColor(i - j, 0, 0, 0);
        show();

        delay(50);
        j += 2;
    }
}

void WS2812B_Controller::policeGang(void)
{
    int j = 9;
    for (int i = 9; i >= 8; i--)
    {
        setLedColor(i, 255, 0, 0);
        setLedColor(i - j, 0, 0, 255);
        show();

        delay(50);
        j -= 2;
    }

    j = 1;

    for (int i = 5; i <= 6; i++)
    {
        setLedColor(i, 255, 0, 0);
        setLedColor(i - j, 0, 0, 255);
        show();

        delay(50);
        j += 2;
    }

    j = 9;
    for (int i = 0; i <= 2; i++)
    {
        setLedColor(i, 0, 0, 0);
        setLedColor(i + j, 0, 0, 0);
        show();

        delay(50);
        j -= 2;
    }
    j = 1;
    for (int i = 5; i <= 6; i++)
    {
        setLedColor(i, 0, 0, 0);
        setLedColor(i - j, 0, 0, 0);
        show();

        delay(50);
        j += 2;
    }
}

void WS2812B_Controller::turnOFF(bool front, bool back)
{
    if (front)
    {
        for (int i = 0; i <= 4; i++)
        {
            setLedColor(i, 0, 0, 0);
            show();
        }
    }
    if (back)
    {
        for (int i = 5; i <= 9; i++)
        {
            setLedColor(i, 0, 0, 0);
            show();
        }
    }
}

void WS2812B_Controller::frontCarHeadlight(void)
{
    for (int i = 0; i <= 4; i++)
    {
        if (i != 2)
        {
            setLedColor(i, 255, 255, 255);
            show();
        }
    }
}

void WS2812B_Controller::backCarHeadlight(void)
{
    for (int i = 5; i <= 9; i++)
    {
        if (i != 7)
        {
            setLedColor(i, 255, 0, 0);
            show();
        }
    }
}
