/*******************************************************************************
 * 
 *  File:         bsf_pico_zero.h 
 * 
 *                Note: This file cannot be called pico.h due to conflict
 *                      with identical named file in Arduino-embed core.
 * 
 *  Function:     Board Support File for Waveshare Pico Zero
 *                with external SPI LoRa module.
 * 
 *  Copyright:    Copyright (c) 2023 Hugo S. C. Bessa, Francisco Helder C. Santos
 * 
 *  License:      MIT License. See accompanying LICENSE file.
 * 
 *  Author:       Hugo S. C. Bessa, Francisco Helder C. Santos
 * 
 *  Description:  This board has onboard USB (provided by the MCU).
 *                It supports automatic firmware upload and serial over USB.
 *                No onboard display. Optionally an external display con be connected.
 * 
 *                IMPORTANT information for firmware upload:
 *                ------------------------------------------
 *                Device must be in BOOTSEL mode to upload firmware.
 *                (To put in BOOTSEL mode: press BOOTSEL button, power on or reset board, release BOOTSEL button.)
 *                In platformio.ini:
 *                For Windows specify: upload_protocol = picotool (appears not needed for Mac and Linux).
 *                upload_port is operating system and hardware dependent. 
 *                For convenience, to set upload_port: set upload_port in [pico] section (on top).
 *                Examples:
 *                  Windows: upload_port = E:
 *                  Mac:     upload_port = /Volumes/RPI-RP2
 *                  Linux:   upload_port = /media/<user>/RSPI-RP2
 *                On Windows USB driver for Pico [RP2 Boot (interface 1)] needs be installed with Zadig,
 *                see: https://community.platformio.org/t/official-platformio-arduino-ide-support-for-the-raspberry-pi-pico-is-now-available/20792
 * 
 *                Connect the LoRa module and optional display
 *                according to below connection details.
 * 
 *                CONNECTIONS AND PIN DEFINITIONS:
 * 
 *                Indentifiers between parentheses are defined in the board's 
 *                Board Support Package (BSP) which is part of the Arduino core. 
 * 
 *                Leds                GPIO 
 *                ----                ----      
 *                LED   <――――――――――>  16   (LED_BUILTIN) (PIN_LED)
 * 
 *                I2C [display]       GPIO  
 *                ---                 ----
 *                SDA   <――――――――――>  4               (PIN_WIRE0_SDA)
 *                SCL   <――――――――――>  5               (PIN_WIRE0_SCL)
 *
 *                SPI/LoRa module     GPIO
 *                ---                 ----
 *                SCK   <――――――――――>  2    (SCK)      (PIN_SPI_SCK)
 *                MOSI  <――――――――――>  3    (MOSI)     (PIN_SPI_MOSI)
 *                MISO  <――――――――――>  4    (MISO)     (PIN_SPI_MISO)
 *                NSS   <――――――――――>  5    (SS)       (PIN_SPI_SS)
 *                RST   <――――――――――>  8 
 *                DIO0  <――――――――――>  14
 *                DIO1  <――――――――――>  15
 *                DIO2                -          Not needed for LoRa.
 * 
 *  Docs:         https://docs.platformio.org/en/latest/boards/raspberrypi/pico.html
 *
 *  Identifiers:  LMIC-node
 *                    board:         pico-zero
 *                PlatformIO
 *                    board:         waveshare_rp2040_zero
 *                    platform:      raspberrypi
 *                Arduino
 *                    board:         
 *                    architecture:  
 * 
 ******************************************************************************/
#ifdef BSF_PICO_ZERO
#pragma once

#ifndef BSF_PICO_ZERO_H_
#define BSF_PICO_ZERO_H_

#include "LMICNode.h"

#ifndef SDA
    #define SDA PIN_WIRE0_SDA
#endif    
#ifndef SCL
    #define SCL PIN_WIRE0_SCL
#endif

#define DEVICEID_DEFAULT "rpi-pico-zero"  // Default deviceid value

// Wait for Serial
// Can be useful for boards with MCU with integrated USB support.
#define WAITFOR_SERIAL_SECONDS_DEFAULT 10   // -1 waits indefinitely  

// LMIC Clock Error
// This is only needed for slower 8-bit MCUs (e.g. 8MHz ATmega328 and ATmega32u4).
// Value is defined in parts per million (of MAX_CLOCK_ERROR).
// #ifndef LMIC_CLOCK_ERROR_PPM
//     #define LMIC_CLOCK_ERROR_PPM 0
// #endif   

#ifdef USE_LED
    #include <EasyLed.h>
    #define LED_BUILTIN 16
#endif


class BSF{
public:
#ifdef USE_SERIAL
    static SerialUSB& serial;
#endif    

#ifdef USE_LED
    static EasyLed led;
#endif

#ifdef USE_DISPLAY
    // Create U8x8 instance for SSD1306 OLED display (no reset) using hardware I2C.
    // Adafruit_SSD1306 BSF::display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, RST_OLED);
#endif

static bool boardInit(InitType initType);
};

#endif
#endif  // BSF_PICO_H_