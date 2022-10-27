// Freenove_WS2812B_RGBLED_Controller.h
/**
 * Brief	Apply to Freenove WS2812B RGBLED Controller.
 *			You can use I2C or UART to communicate.
 * Author	SuhaylZhao
 * Company	Freenove
 * Date		2019-08-03
 */

#ifndef WS2812B_LED_CONTROLLER_h
#define WS2812B_LED_CONTROLLER_h

#include <Arduino.h>
#include <stdint.h>

#include <iostream>
#include <string>
#include <Wire.h>

#define LEDS_COUNT 10

#define REG_LEDS_COUNTS 0
#define REG_SET_LED_COLOR_DATA 1
#define REG_SET_LED_COLOR 2
#define REG_SET_ALL_LEDS_COLOR_DATA 3
#define REG_SET_ALL_LEDS_COLOR 4
#define REG_TRANS_DATA_TO_LED 5

#define REG_LEDS_COUNT_READ 0xfa

#define REG_READ_I2C_ADDRESS 0xfb
#define REG_GET_UART_BAUDRATE 0xfb

#define REG_SET_UART_BAUDRATE 0xfc
#define REG_SET_I2C_ADDRESS 0xfd

#define REG_GET_BRAND 0xfe
#define REG_GET_FIRMWARE_VERSION 0xff

#define I2C_COMMUNICATION_MODE 0
#define UART_COMMUNICATION_MODE 1

#define STRING_BRAND_LENGTH 9
#define STRING_VERSION_LENGTH 16

#define SECTET_KEY_A 0xaa
#define SECTET_KEY_B 0xbb
#define UART_START_BYTE 0xcc
#define UART_END_BYTE 0xdd
#define UART_ACK_BYTE 0x06

enum LED_TYPE
{                    // R  G  B
    TYPE_RGB = 0x06, // 00 01 10
    TYPE_RBG = 0x09, // 00 10 01
    TYPE_GRB = 0x12, // 01 00 10
    TYPE_GBR = 0x21, // 10 00 01
    TYPE_BRG = 0x18, // 01 10 00
    TYPE_BGR = 0x24  // 10 01 00
};

const uint32_t _BAUDRATE[] = {115200, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 230400, 500000};

class WS2812B_Controller
{
protected:
    uint8_t I2C_Address;
    uint8_t uartBaudrateIndex;
    uint8_t commMode;
    uint16_t ledCounts;
    uint32_t uartWaitAckTime; // unit: us
    uint8_t rOffset;
    uint8_t gOffset;
    uint8_t bOffset;

    HardwareSerial *_serial;
    // HardwareSerial *hdSerial;
    // SoftwareSerial *swSerial;
    int writeRegOneByte(uint8_t val);
    int writeReg(uint8_t cmd, uint8_t *value, uint8_t size_a);
    int readReg(uint8_t cmd, char *recv, uint16_t count);
    // bool i2cWriteDataToController(uint8_t cmd, uint8_t *value, uint8_t size_a);
    bool uartWriteDataToControllerWithAck(uint8_t param[5], bool isShowLed = false);

public:
    WS2812B_Controller(uint8_t _address = 0x20, uint16_t n = 8, LED_TYPE t = TYPE_GRB);
    WS2812B_Controller(HardwareSerial *serial_param, uint16_t n = 8, LED_TYPE t = TYPE_GRB, uint32_t _baudrate = 115200);

    bool begin();
    bool setLedCount(uint16_t n);
    void setLedType(LED_TYPE t);

    bool setLedColorData(uint8_t index, uint32_t rgb);
    bool setLedColorData(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

    bool setLedColor(uint8_t index, uint32_t rgb);
    bool setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

    bool setAllLedsColorData(uint32_t rgb);
    bool setAllLedsColorData(uint8_t r, uint8_t g, uint8_t b);

    bool setAllLedsColor(uint32_t rgb);
    bool setAllLedsColor(uint8_t r, uint8_t g, uint8_t b);

    bool show();

    uint8_t getLedsCountFromController();
    uint8_t getI2CAddress();
    uint32_t getUartBaudrate();
    bool setUartBaudrate(uint32_t _baudrate);
    bool setI2CNewAddress(uint8_t addr);
    String getBrand();
    String getFirmwareVersion();

    uint32_t Wheel(byte pos);

    void WeWillFuckYou(void);
};

#endif
