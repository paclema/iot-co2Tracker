#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>


#define USB_VID            0x303A
#define USB_PID            0x8104
#define USB_MANUFACTURER   "paclema"
#define USB_PRODUCT        "Iot-PostBox v1"
#define USB_SERIAL         "" // Empty string for MAC adddress


#define EXTERNAL_NUM_INTERRUPTS 46
#define NUM_DIGITAL_PINS        48
#define NUM_ANALOG_INPUTS       20

#define analogInputToDigitalPin(p)  (((p)<20)?(analogChannelToDigitalPin(p)):-1)
#define digitalPinToInterrupt(p)    (((p)<48)?(p):-1)
#define digitalPinHasPWM(p)         (p < 46)

// #define LED_BUILTIN     -1

#define LDO2_EN_PIN         21
#define PIN_NEOPIXEL        17
#define NEOPIXEL_NUM        1               // number of neopixels
#define NEOPIXEL_POWER      LDO2_EN_PIN     // power pin
#define NEOPIXEL_POWER_ON   HIGH            // power pin state when on
// #define I2C_POWER           7     // I2C power pin
// #define PIN_I2C_POWER       7     // I2C power pin

#define VBUS_SENSE_PIN			1
#define VBAT_SENSE_PIN			2
#define VBAT_STAT_SENSE_PIN		3

#define SW1_PIN		4
#define SW2_PIN		5

#define RFM95W_DIO0_PIN		6
#define RFM95W_DIO1_PIN		7
#define RFM95W_DIO2_PIN		8
#define RFM95W_DIO3_PIN		9
#define RFM95W_NSS_PIN		10
#define RFM95W_MOSI_PIN		11
#define RFM95W_SCK_PIN		12
#define RFM95W_MISO_PIN		13
#define RFM95W_RESET_PIN		14
#define RFM95W_DIO4_PIN		15
#define RFM95W_DIO5_PIN		16

static const uint8_t TX = 43;
static const uint8_t RX = 44;

static const uint8_t SDA = 33;
static const uint8_t SCL = 38;

static const uint8_t SS    = 34;
static const uint8_t MOSI  = 35;
static const uint8_t MISO  = 37;
static const uint8_t SCK   = 36;

static const uint8_t A0 = 1;
static const uint8_t A1 = 2;
static const uint8_t A2 = 3;
static const uint8_t A3 = 4;
static const uint8_t A4 = 5;
static const uint8_t A5 = 6;
static const uint8_t A6 = 7;
static const uint8_t A7 = 8;
static const uint8_t A8 = 9;
static const uint8_t A9 = 10;
static const uint8_t A10 = 11;
static const uint8_t A11 = 12;
static const uint8_t A12 = 13;
static const uint8_t A13 = 14;
static const uint8_t A14 = 15;
static const uint8_t A15 = 16;
static const uint8_t A16 = 17;
static const uint8_t A17 = 18;
static const uint8_t A18 = 19;
static const uint8_t A19 = 20;

static const uint8_t T1 = 1;
static const uint8_t T2 = 2;
static const uint8_t T3 = 3;
static const uint8_t T4 = 4;
static const uint8_t T5 = 5;
static const uint8_t T6 = 6;
static const uint8_t T7 = 7;
static const uint8_t T8 = 8;
static const uint8_t T9 = 9;
static const uint8_t T10 = 10;
static const uint8_t T11 = 11;
static const uint8_t T12 = 12;
static const uint8_t T13 = 13;
static const uint8_t T14 = 14;

static const uint8_t DAC1 = 17;
static const uint8_t DAC2 = 18;

#endif /* Pins_Arduino_h */
