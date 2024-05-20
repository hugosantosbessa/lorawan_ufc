/*******************************************************************************
 *
 *  File:         LMIC-node.h
 * 
 *  Function:     LMIC-node main header file.
 * 
 *  Copyright:    Copyright (c) 2023 Hugo S. C. Bessa, Francisco Helder C. Santos
 *                Copyright (c) 2021 Leonel Lopes Parente
 *                Portions Copyright (c) 2018 Terry Moore, MCCI
 *
 *                Permission is hereby granted, free of charge, to anyone 
 *                obtaining a copy of this document and accompanying files to do, 
 *                whatever they want with them without any restriction, including,
 *                but not limited to, copying, modification and redistribution.
 *                The above copyright notice and this permission notice shall be 
 *                included in all copies or substantial portions of the Software.
 * 
 *                THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 * 
 *  License:      MIT License. See accompanying LICENSE file.
 * 
 *  Author:       Hugo S. C. Bessa, Francisco Helder C. Santos
 *                 
 ******************************************************************************/

#pragma once

#ifndef LMIC_NODE_H_
#define LMIC_NODE_H_

#include <Arduino.h>
#include "lmic.h"
#include "hal/hal.h"

enum class InitType { Hardware, PostInitSerial };
enum class PrintTarget { All, Serial, Display };
enum class ActivationMode {OTAA, ABP};

#include BSFILE // Include Board Support File
#include "../keyfiles/lorawan-keys.h"

#ifdef USE_DISPLAY
    #include <Wire.h>
#endif

#ifdef USE_LED
    #include "EasyLed.h"
#endif

#ifndef DO_WORK_INTERVAL_SECONDS            // Should be set in platformio.ini
    #define DO_WORK_INTERVAL_SECONDS 300    // Default 5 minutes if not set
#endif    

#define TIMESTAMP_WIDTH 12 // Number of columns to display eventtime (zero-padded)
#define MESSAGE_INDENT TIMESTAMP_WIDTH + 3

// Determine which LMIC library is used
#ifdef _LMIC_CONFIG_PRECONDITIONS_H_   
    #define MCCI_LMIC
#else
    #define CLASSIC_LMIC
#endif    

#if !defined(ABP_ACTIVATION) && !defined(OTAA_ACTIVATION)
    #define OTAA_ACTIVATION
#endif
    
#if defined(ABP_ACTIVATION) && defined(OTAA_ACTIVATION)
    #error Only one of ABP_ACTIVATION and OTAA_ACTIVATION can be defined.
#endif

// Allow WAITFOR_SERIAL_SECONDS to be defined in platformio.ini.
// If used it shall be defined in the [common] section.
// The common setting will only be used for boards that have 
// WAITFOR_SERIAL_SECONDS_DEFAULT defined (in BSP) with a value != 0
#if defined(WAITFOR_SERIAL_SECONDS_DEFAULT)  && WAITFOR_SERIAL_SECONDS_DEFAULT != 0
    #ifdef WAITFOR_SERIAL_SECONDS
        #define WAITFOR_SERIAL_S WAITFOR_SERIAL_SECONDS
    #else
        #define WAITFOR_SERIAL_S WAITFOR_SERIAL_SECONDS_DEFAULT
    #endif
#else
    #define WAITFOR_SERIAL_S 0
#endif 

#if defined(ABP_ACTIVATION) && defined(CLASSIC_LMIC)
    #error Do NOT use ABP activation when using the deprecated IBM LMIC framework library. \
           On The Things Network V3 this will cause a downlink message for EVERY uplink message \
           because it does properly handle MAC commands. 
#endif

#ifdef OTAA_ACTIVATION
    #if !defined(OTAA_DEVEUI) || !defined(OTAA_APPEUI) || !defined(OTAA_APPKEY)
        #error One or more LoRaWAN keys (OTAA_DEVEUI, OTAA_APPEUI, OTAA_APPKEY) are not defined.
    #endif 
#else
    // ABP activation
    #if !defined(ABP_DEVADDR) || !defined(ABP_NWKSKEY) || !defined(ABP_APPSKEY)
        #error One or more LoRaWAN keys (ABP_DEVADDR, ABP_NWKSKEY, ABP_APPSKEY) are not defined.
    #endif
#endif

// Determine if a valid region is defined.
// This actually has little effect because
// CLASSIC LMIC: defines CFG_eu868 by default,
// MCCI LMIC: if no region is defined it
// sets CFG_eu868 as default.
#if ( \
    ( defined(CLASSIC_LMIC) \
      && !( defined(CFG_eu868) \
            || defined(CFG_us915) ) ) \
    || \
    ( defined(MCCI_LMIC) \
      && !( defined(CFG_as923) \
            || defined(CFG_as923jp) \
            || defined(CFG_au915) \
            || defined(CFG_eu868) \
            || defined(CFG_in866) \
            || defined(CFG_kr920) \
            || defined(CFG_us915) ) ) \
)
    #Error No valid LoRaWAN region defined
#endif

#ifndef MCCI_LMIC
    #define LMIC_ERROR_SUCCESS 0
    typedef int lmic_tx_error_t;

    // In MCCI LMIC these are already defined.
    // This macro can be used to initalize an array of event strings
    #define LEGACY_LMIC_EVENT_NAME_TABLE__INIT \
                "<<zero>>", \
                "EV_SCAN_TIMEOUT", "EV_BEACON_FOUND", \
                "EV_BEACON_MISSED", "EV_BEACON_TRACKED", "EV_JOINING", \
                "EV_JOINED", "EV_RFU1", "EV_JOIN_FAILED", "EV_REJOIN_FAILED", \
                "EV_TXCOMPLETE", "EV_LOST_TSYNC", "EV_RESET", \
                "EV_RXCOMPLETE", "EV_LINK_DEAD", "EV_LINK_ALIVE"

    // If working on an AVR (or worried about memory size), you can use this multi-zero
    // string and put this in a single const F() string to store it in program memory.
    // Index through this counting up from 0, until you get to the entry you want or 
    // to an entry that begins with a \0.
    #define LEGACY_LMIC_EVENT_NAME_MULTISZ__INIT \
                "<<zero>>\0" \                                                           \
                "EV_SCAN_TIMEOUT\0" "EV_BEACON_FOUND\0" \
                "EV_BEACON_MISSED\0" "EV_BEACON_TRACKED\0" "EV_JOINING\0" \
                "EV_JOINED\0" "EV_RFU1\0" "EV_JOIN_FAILED\0" "EV_REJOIN_FAILED\0" \
                "EV_TXCOMPLETE\0" "EV_LOST_TSYNC\0" "EV_RESET\0" \
                "EV_RXCOMPLETE\0" "EV_LINK_DEAD\0" "EV_LINK_ALIVE\0"   
#endif // LMIC_MCCI   

/*
    This function must be defined in main.cpp
*/
void prepareUplink();

class LMICNode {
private:
    // Set LoRaWAN keys defined in lorawan-keys.h.
    #ifdef OTAA_ACTIVATION
        static const u1_t PROGMEM DEVEUI[8];
        static const u1_t PROGMEM APPEUI[8];
        static const u1_t PROGMEM APPKEY[16];
        // Below callbacks are used by LMIC for reading above values.
        void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
        void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
        void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }    
    #else
        // ABP activation
        static const u4_t DEVADDR;
        static const u1_t NWKSKEY[16];
        static const u1_t APPSKEY[16];
        // Below callbacks are not used be they must be defined.
        void os_getDevEui (u1_t* buf) { }
        void os_getArtEui (u1_t* buf) { }
        void os_getDevKey (u1_t* buf) { }

        dr_t abpDataRate = DefaultABPDataRate; 
        s1_t abpTxPower = DefaultABPTxPower;
        void setAbpParameters(dr_t dataRate, s1_t txPower); 
    #endif

    bit_t adrEnabled = 0;
    static volatile uint16_t counter_;
    
    
    void printHeader();
    void initDisplay();
    void displayStatus();

    static void resetCounter();

public:
    
    #if defined(ABP_ACTIVATION) && defined(ABP_DEVICEID)
        static const char deviceId[];
    #elif defined(DEVICEID)
        static const char deviceId[];
    #else
        static const char deviceId[];
    #endif

    #ifdef MCCI_LMIC   
        static const char * const lmicEventNames[];
        static const char * const lmicErrorNames[];
    #else
        static const char * const lmicEventNames[];
    #endif

    #ifdef OTAA_ACTIVATION
        static const ActivationMode activationMode = ActivationMode::OTAA;
    #else       
        static const ActivationMode activationMode = ActivationMode::ABP;
    #endif
    
    static const uint32_t doWorkIntervalSeconds = DO_WORK_INTERVAL_SECONDS;  // Change value in platformio.ini
    static const dr_t DefaultABPDataRate = DR_SF7;
    static const s1_t DefaultABPTxPower =  14;
    static uint8_t fPort;
    static uint8_t sizeData;
    static uint8_t mydata[256];
    
    void initHardware();
    void initLmic();
    void initTransmission();
    void setup_node();
    void loop_node();
    void setADR(bit_t adrEnabled);
#ifdef ABP_ACTIVATION
    void setDataRate(dr_t abpDataRate); 
    void setTxPower(s1_t abpTxPower);
#endif
    static void setTxIndicatorsOn(bool on = true);
    static uint16_t getCounterValue();
    static String getDR();
    static String getFreq();
    static int16_t getSnrTenfold();
    static int16_t getRssi(int8_t snr);

    
    static void printChars(Print& printer, char ch, uint8_t count, bool linefeed = false);
    static void printHex(Print& printer, uint8_t* bytes, size_t length = 1, bool linefeed = false, char separator = 0);
    static void printSpaces(Print& printer, uint8_t count, bool linefeed = false);
    static void printEvent(ostime_t timestamp, 
                const char * const message, 
                PrintTarget target = PrintTarget::All,
                bool clearDisplayStatusRow = true,
                bool eventLabel = false);
    static void printEvent(ostime_t timestamp, 
                ev_t ev, 
                PrintTarget target = PrintTarget::All, 
                bool clearDisplayStatusRow = true);
    static void printSessionKeys();
    static void printDownlinkInfo(void);
    static void printFrameCounters(PrintTarget target = PrintTarget::All);

    static void onLmicEvent(void *pUserData, ev_t ev);
    static void doWorkCallback(osjob_t* job);
    
    static void processWork(ostime_t doWorkJobTimeStamp);
    static void processDownlink(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t* data, uint8_t dataLength);
    static lmic_tx_error_t scheduleUplink(uint8_t fPort, uint8_t* data, uint8_t dataLength, bool confirmed = false);

};


#endif  // LMIC_NODE_H_
