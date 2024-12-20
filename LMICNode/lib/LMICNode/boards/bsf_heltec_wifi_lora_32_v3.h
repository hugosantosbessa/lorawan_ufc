/*******************************************************************************
 * 
 *  File:         bsf_heltec_wifi_lora_32_v4.h
 * 
 *  Function:     Board Support File for Heltec WiFi LoRa 32 V3.
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
 *                CONNECTIONS AND PIN DEFINITIONS:
 *                
 *                Indentifiers between parentheses are defined in the board's 
 *                Board Support Package (BSP) which is part of the Arduino core. 
 * 
 *                Leds                GPIO 
 *                ----                ----
 *                LED                 35
 * 
 *                I2C/Display         GPIO
 *                ---                 ---- 
 *                SDA   <――――――――――>  17 
 *                SCL   <――――――――――>  18 
 *                RST   <――――――――――>  21               
 *
 *                SPI/LoRa            GPIO
 *                ---                 ---- 
 *                MOSI  <――――――――――>  10  (MOSI) (LORA_MOSI)
 *                MISO  <――――――――――>  11  (MISO) (LORA_MISO)
 *                SCK   <――――――――――>   9  (SCK)  (LORA_SCK)
 *                NSS   <――――――――――>   8  (SS)   (LORA_CS)
 *                RST   <――――――――――>  12         (LORA_RST)
 *                BUSY  <――――――――――>  13         (LORA_IRQ)
 *                DIO0  <――――――――――>  14         (LORA_IRQ)
 * 
 * 
 *  Docs:         https://docs.platformio.org/en/latest/boards/espressif32/heltec_wifi_lora_32_V3.html
 *
 *  Identifiers:  LMIC-node
 *                    board:         heltec_wifi_lora_32_v3
 *                PlatformIO
 *                    board:         heltec_wifi_lora_32_V3
 *                    platform:      espressif32
 *                Arduino
 *                    board:         ARDUINO_heltec_wifi_lora_32_V3
 *                    architecture:  ARDUINO_ARCH_ESP32
 * 
 ******************************************************************************/
#ifdef BSF_HELTEC_WIFI_LORA_32_V3
#pragma once

#ifndef BSF_HELTEC_WIFI_LORA_32_V3_
#define BSF_HELTEC_WIFI_LORA_32_V3_

#include "LMICNode.h"

#define DEVICEID_DEFAULT "heltec-wifi-lora-32-v3"  // Default deviceid value

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
#endif  // BSF_HELTEC_WIFI_LORA_32_V3_