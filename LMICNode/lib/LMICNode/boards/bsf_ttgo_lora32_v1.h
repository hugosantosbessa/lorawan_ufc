/*******************************************************************************
 * 
 *  File:         bsf_ttgo_lora32_v1.h
 * 
 *  Function:     Board Support File for TTGO LoRa32 (aka T3) v1.3.
 * 
 *  Copyright:    Copyright (c) 2023 Hugo S. C. Bessa, Francisco Helder C. Santos
 * 
 *  License:      MIT License. See accompanying LICENSE file.
 * 
 *  Author:       Hugo S. C. Bessa, Francisco Helder C. Santos
 * 
 *  Description:  This board has onboard USB (provided by onboard USB to serial).
 *                It supports automatic firmware upload and serial over USB. 
 *                Has onboard display.
 * 
 *                The standard I2C pins defined in the BSP do not match the 
 *                GPIO pins that the display is connected to. Therefore the
 *                the I2C Wire object is explicitly initialized with the
 *                correct pins (see boardInit() below).
 * 
 *                Schematic diagram and and pinout diagram show no onboard 
 *                user programmable LED while LED_BUILTIN is defined in BSP.
 *                Definition in BSP is incorrect.
 * 
 *                OLED_RST and LORA_RST are defined in BSP but neither is connected to GPIO.
 *                Definitions in BSP are incorrect.               
 * 
 *                CONNECTIONS AND PIN DEFINITIONS:
 *                
 *                Indentifiers between parentheses are defined in the board's 
 *                Board Support Package (BSP) which is part of the Arduino core. 
 * 
 *                Leds                GPIO 
 *                ----                ----
 *                LED                 -            Incorrectly defined in BSP as LED_BUILTIN (2).
 * 
 *                I2C/Display         GPIO
 *                ---                 ---- 
 *                SDA   <――――――――――>  4   Not SDA! (OLED_SDA)
 *                SCL   <――――――――――>  15  Not SCL! (OLED_SCL)
 *                RST                     OLED_RST is defined in BSP but not connected to GPIO.
 *
 *                SPI/LoRa            GPIO
 *                ---                 ---- 
 *                MOSI  <――――――――――>  27  (MOSI) (LORA_MOSI)
 *                MISO  <――――――――――>  19  (MISO) (LORA_MISO)
 *                SCK   <――――――――――>   5  (SCK)  (LORA_SCK)
 *                NSS   <――――――――――>  18  (SS)   (LORA_CS)
 *                RST   <――――――――――>  14         (LORA_RST)
 *                DIO0  <――――――――――>  26         (LORA_IRQ)
 *                DIO1  <――――――――――>  33
 *                DIO2  <――――――――――>  32
 * 
 *                Button switches     GPIO
 *                ------              ---- 
 *                Button <―――――――――>  36  (V_SP) Active-low
 * 
 *                Battery measure     GPIO
 *                -------             ---- 
 *                VBAT  <――――――――――>  35  Battery voltage via 50% voltage divider
 * 
 *  Docs:         https://docs.platformio.org/en/latest/boards/espressif32/ttgo-lora32-v1.html
 *
 *  Identifiers:  LMIC-node
 *                    board:         ttgo_lora32_v1
 *                PlatformIO
 *                    board:         ttgo-lora32-v1
 *                    platform:      espressif32
 *                Arduino
 *                    board:         ARDUINO_TTGO_LoRa32_V1
 *                    architecture:  ARDUINO_ARCH_ESP32
 * 
 ******************************************************************************/
#ifdef BSF_TTGO_LORA32_V1
#pragma once

#ifndef BSF_TTGO_LORA32_V1_H_
#define BSF_TTGO_LORA32_V1_H_

#include "LMICNode.h"

#define DEVICEID_DEFAULT "ttgo-lora32-v1"  // Default deviceid value

// Wait for Serial
// Can be useful for boards with MCU with integrated USB support.
// #define WAITFOR_SERIAL_SECONDS_DEFAULT 10   // -1 waits indefinitely  

// LMIC Clock Error
// This is only needed for slower 8-bit MCUs (e.g. 8MHz ATmega328 and ATmega32u4).
// Value is defined in parts per million (of MAX_CLOCK_ERROR).
// #ifndef LMIC_CLOCK_ERROR_PPM
//     #define LMIC_CLOCK_ERROR_PPM 0
// #endif   


#ifdef USE_LED
    #include <EasyLed.h>
#endif

#ifdef USE_DISPLAY
    // Create U8x8 instance for SSD1306 OLED display (no reset) using hardware I2C.
    // U8X8_SSD1306_128X64_NONAME_HW_I2C display(/*rst*/ 16, /*scl*/ 15, /*sda*/ 4);
    #include <Adafruit_I2CDevice.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    #define SCREEN_ADDRESS 0x3c
    #define OLED_SDA 4
    #define OLED_SCL 15
    #define OLED_RST 16
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
#endif

class BSF{
public:
#ifdef USE_SERIAL
    static HardwareSerial& serial;
#endif  

#ifdef USE_LED
    static EasyLed led;
#endif

#ifdef USE_DISPLAY
    static Adafruit_SSD1306 display;
#endif

static bool boardInit(InitType initType);

};

#endif
#endif  // BSF_TTGO_LORA32_V1_H_