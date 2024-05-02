/*******************************************************************************
 *
 *  File:          LMIC-node.cpp
 * 
 *  Function:      LMIC-node main application file.
 * 
 *  Copyright:     
 *                 Copyright (c) 2023 Hugo S. C. Bessa, Francisco Helder C. Santos
 *
 *                 Permission is hereby granted, free of charge, to anyone 
 *                 obtaining a copy of this document and accompanying files to do, 
 *                 whatever they want with them without any restriction, including,
 *                 but not limited to, copying, modification and redistribution.
 *                 The above copyright notice and this permission notice shall be 
 *                 included in all copies or substantial portions of the Software.
 * 
 *                 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 * 
 *  License:       MIT License. See accompanying LICENSE file.
 * 
 *  Author:        Leonel Lopes Parente
 * 
 *  Description:   To get LMIC-node up and running no changes need to be made
 *                 to any source code. Only configuration is required
 *                 in platform-io.ini and lorawan-keys.h.
 * 
 *                 If you want to modify the code e.g. to add your own sensors,
 *                 that can be done in the two area's that start with
 *                 USER CODE BEGIN and end with USER CODE END. There's no need
 *                 to change code in other locations (unless you have a reason).
 *                 See README.md for documentation and how to use LMIC-node.
 * 
 *                 LMIC-node uses the concepts from the original ttn-otaa.ino 
 *                 and ttn-abp.ino examples provided with the LMIC libraries.
 *                 LMIC-node combines both OTAA and ABP support in a single example,
 *                 supports multiple LMIC libraries, contains several improvements
 *                 and enhancements like display support, support for downlinks,
 *                 separates LoRaWAN keys from source code into a separate keyfile,
 *                 provides formatted output to serial port and display
 *                 and supports many popular development boards out of the box.
 *                 To get a working node up and running only requires some configuration.
 *                 No programming or customization of source code required.
 * 
 *  Dependencies:  External libraries:
 *                 MCCI LoRaWAN LMIC library  https://github.com/mcci-catena/arduino-lmic
 *                 IBM LMIC framework         https://github.com/matthijskooijman/arduino-lmic  
 *                 U8g2                       https://github.com/olikraus/u8g2
 *                 EasyLed                    https://github.com/lnlp/EasyLed
 *
 ******************************************************************************/

#include "LMIC-node.h"
#include <Wire.h>
#include <CurrentBoard.h>

unsigned long _TX_COUNT = 0;
unsigned long _TX_REAL = 0;
unsigned long _RX_COUNT = 0;
unsigned long _DW_COUNT = 0;
String _STATUS_JOIN = "ABP";
String _STATUS_DR = "";
String _STATUS_LMIC = "INICIANDO";

#ifdef USE_DISPLAY
unsigned long _TX_COUNT_old = _TX_COUNT;
unsigned long _RX_COUNT_old = _RX_COUNT;
unsigned long _DW_COUNT_old = _DW_COUNT;
unsigned long _TX_REAL_old = _TX_REAL;
String _STATUS_JOIN_old = _STATUS_JOIN;
String _STATUS_LMIC_old = _STATUS_LMIC;
String _STATUS_DR_old = _STATUS_DR;

void display_status(){
    if(
        (_STATUS_LMIC_old == _STATUS_LMIC) &&
        (_STATUS_JOIN_old == _STATUS_JOIN) &&
        (_TX_REAL_old == _TX_REAL) &&
        (_RX_COUNT_old == _RX_COUNT) &&
        (_TX_COUNT_old == _TX_COUNT) && 
        (_DW_COUNT_old == _DW_COUNT) &&
        (_STATUS_DR_old == _STATUS_DR)
    ) {
        return;
    }
    
    _TX_COUNT_old = _TX_COUNT;
    _RX_COUNT_old = _RX_COUNT;
    _TX_REAL_old = _TX_REAL;
    _STATUS_JOIN_old = _STATUS_JOIN;
    _STATUS_LMIC_old = _STATUS_LMIC;
    _DW_COUNT_old = _DW_COUNT;
    _STATUS_DR_old = _STATUS_DR;
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("DEV: "));
    display.println(ABP_DEVADDR, HEX);

    // https://github.com/sandeepmistry/arduino-LoRa/pull/291
    u1_t bw = getBw(LMIC.rps);
    display.setCursor(0, 10);
    display.print(F("RSSI: "));
    display.print(LMIC.rssi);
    display.println("  "+_STATUS_DR);

    display.setCursor(0, 20);
    display.print(F("TX(c): "));
    display.print(_TX_COUNT);
    display.print(F(" ["));
    display.print(_TX_REAL);
    display.println(F("]"));

    display.setCursor(0, 30);
    display.print(F("RX(c): "));
    display.print(_RX_COUNT);

    display.print(F("  DW(c): "));
    display.println(_DW_COUNT);

    display.setCursor(0, 40);
    display.print(F("JOIN: "));
    display.println(_STATUS_JOIN);

    display.setCursor(0, 50);
    display.print(F("STATUS: "));
    display.println(_STATUS_LMIC);
        
    display.display();
}

#endif

CurrentBoard currentBoard;
const uint8_t payloadBufferLength = 4;    // Adjust to fit max payload length
uint8_t mydata[256] = "";
char data[20] = "";

uint8_t payloadBuffer[payloadBufferLength];
static osjob_t doWorkJob;
uint32_t doWorkIntervalSeconds = DO_WORK_INTERVAL_SECONDS;  // Change value in platformio.ini

// Note: LoRa module pin mappings are defined in the Board Support Files.

// Set LoRaWAN keys defined in lorawan-keys.h.
#ifdef OTAA_ACTIVATION
    static const u1_t PROGMEM DEVEUI[8]  = { OTAA_DEVEUI } ;
    static const u1_t PROGMEM APPEUI[8]  = { OTAA_APPEUI };
    static const u1_t PROGMEM APPKEY[16] = { OTAA_APPKEY };
    // Below callbacks are used by LMIC for reading above values.
    void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
    void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
    void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }    
#else
    // ABP activation
    static const u4_t DEVADDR = ABP_DEVADDR ;
    static const PROGMEM u1_t NWKSKEY[16] = { ABP_NWKSKEY };
    static const u1_t PROGMEM APPSKEY[16] = { ABP_APPSKEY };
    // Below callbacks are not used be they must be defined.
    void os_getDevEui (u1_t* buf) { }
    void os_getArtEui (u1_t* buf) { }
    void os_getDevKey (u1_t* buf) { }
#endif

String getDR(){
    byte sf = LMIC.datarate;
    char v[10];
    snprintf(v, 10, "SF%d%c", (sf!=6? 12-sf:8), (sf!=6? ' ':'C'));
    return v;
}

String getFreq(){
    byte sf = LMIC.datarate;
    float freq = ((LMIC.freq) / 1000000.0f);
    bool impar = (LMIC.freq/100000)%2;
    unsigned canalkhz = (LMIC.freq-915200000)/200000;
    if(impar)
        canalkhz = ((LMIC.freq-915900000)/600000) + 64;

    unsigned int bps = (sf==0? 250 : ( // SF12 / 125 kHz
        sf==1? 440 : ( // SF11 / 125 kHz
            sf==2 || sf==8 ? 980 : ( // SF10 / 125 kHz | SF12 / 500 kHz
                sf==3 || sf==9 ? 1760 : ( //  SF9 / 125 kHz | SF11 / 500 kHz
                    sf==4? 3125 : ( // SF8 / 125 kHz
                        sf==5? 5470 : ( // SF7 / 125 kHz
                            sf==6 || sf==12 ? 1250 : ( // SF8 / 500 kHz
                                sf==10? 3900 : ( //  SF10 / 500 kHz
                                    sf==11? 7000 : ( // SF9 / 500 kHz
                                        21900 //sf==13 // SF7 / 500 kHz
                                    ) 
                                ) 
                            )
                        )
                    )
                )
            ) 
        )));
    char v1[15];
    if(bps < 1000)
        snprintf(v1, 15, "%d bps", bps);
    else {
        snprintf(v1, 15, "%.2f kbps", ((float)bps/1000.0f));
        if(v1[1]=='.') v1[1] = ',';
        else v1[2] = ',';
    }
    char v[55];
    snprintf(v, 55, "%.1f MHz (CH%d) (%d KHz) (DR_SF%d%c) (%s)", freq, canalkhz+1, impar?500:150, (sf!=6? 12-sf:8), (sf!=6? ' ':'C'), v1);
    return v;
}

int16_t getSnrTenfold()
{
    // Returns ten times the SNR (dB) value of the last received packet.
    // Ten times to prevent the use of float but keep 1 decimal digit accuracy.
    // Calculation per SX1276 datasheet rev.7 §6.4, SX1276 datasheet rev.4 §6.4.
    // LMIC.snr contains value of PacketSnr, which is 4 times the actual SNR value.
    return (LMIC.snr * 10) / 4;
}


int16_t getRssi(int8_t snr)
{
    // Returns correct RSSI (dBm) value of the last received packet.
    // Calculation per SX1276 datasheet rev.7 §5.5.5, SX1272 datasheet rev.4 §5.5.5.

    #define RSSI_OFFSET            64
    #define SX1276_FREQ_LF_MAX     525000000     // per datasheet 6.3
    #define SX1272_RSSI_ADJUST     -139
    #define SX1276_RSSI_ADJUST_LF  -164
    #define SX1276_RSSI_ADJUST_HF  -157

    int16_t rssi;

    #ifdef MCCI_LMIC
        rssi = LMIC.rssi - RSSI_OFFSET;
    #endif

    return rssi;
}


void         printEvent(ostime_t timestamp, 
                const char * const message, 
                PrintTarget target = PrintTarget::All,
                bool clearDisplayStatusRow = true,
                bool eventLabel = false)
{   
    #ifdef USE_SERIAL
        // Create padded/indented output without using printf().
        // printf() is not default supported/enabled in each Arduino core. 
        // Not using printf() will save memory for memory constrainted devices.
        String timeString(timestamp);
        uint8_t len = timeString.length();
        uint8_t zerosCount = TIMESTAMP_WIDTH > len ? TIMESTAMP_WIDTH - len : 0;

        if (target == PrintTarget::All || target == PrintTarget::Serial)
        {
            printChars(serial, '0', zerosCount);
            serial.print(timeString);
            serial.print(":  ");
            if (eventLabel)
            {
                serial.print(F("Event: "));
            }
            serial.println(message);
        }
    #endif   
}           

void printEvent(ostime_t timestamp, 
                ev_t ev, 
                PrintTarget target = PrintTarget::All, 
                bool clearDisplayStatusRow = true)
{
    #if defined(USE_DISPLAY) || defined(USE_SERIAL)
        printEvent(timestamp, lmicEventNames[ev], target, clearDisplayStatusRow, true);
    #endif
}


void printFrameCounters(PrintTarget target = PrintTarget::All)
{
    #ifdef USE_SERIAL
        if (target == PrintTarget::Serial || target == PrintTarget::All)
        {
            printSpaces(serial, MESSAGE_INDENT);
            serial.print(F("Up: "));
            serial.print(LMIC.seqnoUp);
            serial.print(F(",  Down: "));
            serial.println(LMIC.seqnoDn);        
        }
    #endif        
}      


void printSessionKeys()
{    
    #if defined(USE_SERIAL) && defined(MCCI_LMIC)
        u4_t networkId = 0;
        devaddr_t deviceAddress = 0;
        u1_t networkSessionKey[16];
        u1_t applicationSessionKey[16];
        LMIC_getSessionKeys(&networkId, &deviceAddress, 
                            networkSessionKey, applicationSessionKey);

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Network Id: "));
        serial.println(networkId, DEC);

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Device Address: "));
        serial.println(deviceAddress, HEX);

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Application Session Key: "));
        printHex(serial, applicationSessionKey, 16, true, '-');

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Network Session Key:     "));
        printHex(serial, networkSessionKey, 16, true, '-');
    #endif
}


void printDownlinkInfo(void)
{
    #if defined(USE_SERIAL) || defined(USE_DISPLAY)

        uint8_t dataLength = LMIC.dataLen;
        // bool ackReceived = LMIC.txrxFlags & TXRX_ACK;

        int16_t snrTenfold = getSnrTenfold();
        int8_t snr = snrTenfold / 10;
        int8_t snrDecimalFraction = snrTenfold % 10;
        int16_t rssi = getRssi(snr);

        uint8_t fPort = 0;        
        if (LMIC.txrxFlags & TXRX_PORT)
        {
            fPort = LMIC.frame[LMIC.dataBeg -1];
        }        

        #ifdef USE_SERIAL
            printSpaces(serial, MESSAGE_INDENT);    
            serial.println(F("Downlink received"));

            printSpaces(serial, MESSAGE_INDENT);
            serial.print(F("RSSI: "));
            serial.print(rssi);
            serial.print(F(" dBm,  SNR: "));
            serial.print(snr);                        
            serial.print(".");                        
            serial.print(snrDecimalFraction);                        
            serial.println(F(" dB"));

            printSpaces(serial, MESSAGE_INDENT);    
            serial.print(F("Port: "));
            serial.println(fPort);
   
            if (dataLength != 0)
            {
                printSpaces(serial, MESSAGE_INDENT);
                serial.print(F("Length: "));
                serial.println(LMIC.dataLen);                   
                printSpaces(serial, MESSAGE_INDENT);    
                serial.print(F("Data: "));
                printHex(serial, LMIC.frame+LMIC.dataBeg, LMIC.dataLen, true, ' ');
            }
        #endif
    #endif
} 


void printHeader(void)
{
    #ifdef USE_SERIAL
        serial.println(F("\n\nLMIC-node\n"));
        serial.print(F("Device-id:     "));
        serial.println(deviceId);            
        serial.print(F("LMIC library:  "));
        #ifdef MCCI_LMIC  
            serial.println(F("MCCI"));
        #else
            serial.println(F("Classic [Deprecated]")); 
        #endif
        serial.print(F("Activation:    "));
        #ifdef OTAA_ACTIVATION  
            serial.println(F("OTAA"));
        #else
            serial.println(F("ABP")); 
        #endif
        #if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
            serial.print(F("LMIC debug:    "));  
            serial.println(LMIC_DEBUG_LEVEL);
        #endif
        serial.print(F("Interval:      "));
        serial.print(doWorkIntervalSeconds);
        serial.println(F(" seconds"));
        if (activationMode == ActivationMode::OTAA)
        {
            serial.println();
        }
    #endif
}     


#ifdef ABP_ACTIVATION
    void setAbpParameters(dr_t dataRate = DefaultABPDataRate, s1_t txPower = DefaultABPTxPower) 
    {
        // Set static session parameters. Instead of dynamically establishing a session
        // by joining the network, precomputed session parameters are be provided.
        #ifdef PROGMEM
            // On AVR, these values are stored in flash and only copied to RAM
            // once. Copy them to a temporary buffer here, LMIC_setSession will
            // copy them into a buffer of its own again.
            uint8_t appskey[sizeof(APPSKEY)];
            uint8_t nwkskey[sizeof(NWKSKEY)];
            memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
            memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
            LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
        #else
            // If not running an AVR with PROGMEM, just use the arrays directly
            LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
        #endif

        #if defined(CFG_us915) || defined(CFG_au915)
            // NA-US and AU channels 0-71 are configured automatically
            // but only one group of 8 should (a subband) should be active
            // TTN recommends the second sub band, 1 in a zero based count.
            // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
            LMIC_selectSubBand(1);
        #endif

        // Disable link check validation
        LMIC_setLinkCheckMode(0);

        // TTN uses SF9 for its RX2 window.
        LMIC.dn2Dr = DR_SF9;

        // Set data rate and transmit power (note: txpow is possibly ignored by the library)
        LMIC_setDrTxpow(dataRate, txPower);    
    }
#endif //ABP_ACTIVATION


void initLmic(bit_t adrEnabled = 0,
              dr_t abpDataRate = DefaultABPDataRate, 
              s1_t abpTxPower = DefaultABPTxPower) 
{
    // ostime_t timestamp = os_getTime();

    // Initialize LMIC runtime environment
    os_init();
    // Reset MAC state
    LMIC_reset();

    #ifdef ABP_ACTIVATION
        setAbpParameters(abpDataRate, abpTxPower);
    #endif

    // Enable or disable ADR (data rate adaptation). 
    // Should be turned off if the device is not stationary (mobile).
    // 1 is on, 0 is off.
    LMIC_setAdrMode(adrEnabled);

    if (activationMode == ActivationMode::OTAA)
    {
        #if defined(CFG_us915) || defined(CFG_au915)
            // NA-US and AU channels 0-71 are configured automatically
            // but only one group of 8 should (a subband) should be active
            // TTN recommends the second sub band, 1 in a zero based count.
            // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
            LMIC_selectSubBand(1); 
        #endif
    }

    // Relax LMIC timing if defined
    #if defined(LMIC_CLOCK_ERROR_PPM)
        uint32_t clockError = 0;
        #if LMIC_CLOCK_ERROR_PPM > 0
            #if defined(MCCI_LMIC) && LMIC_CLOCK_ERROR_PPM > 4000
                // Allow clock error percentage to be > 0.4%
                #define LMIC_ENABLE_arbitrary_clock_error 1
            #endif    
            clockError = (LMIC_CLOCK_ERROR_PPM / 100) * (MAX_CLOCK_ERROR / 100) / 100;
            LMIC_setClockError(clockError);
        #endif

        #ifdef USE_SERIAL
            serial.print(F("Clock Error:   "));
            serial.print(LMIC_CLOCK_ERROR_PPM);
            serial.print(" ppm (");
            serial.print(clockError);
            serial.println(")");            
        #endif
    #endif

    #ifdef MCCI_LMIC
        // Register a custom eventhandler and don't use default onEvent() to enable
        // additional features (e.g. make EV_RXSTART available). User data pointer is omitted.
        LMIC_registerEventCb(&onLmicEvent, nullptr);
    #endif
}


#ifdef MCCI_LMIC 
void onLmicEvent(void *pUserData, ev_t ev)
#else
void onEvent(ev_t ev) 
#endif
{
    // LMIC event handler
    ostime_t timestamp = os_getTime(); 

    switch (ev) 
    {
#ifdef MCCI_LMIC
        // Only supported in MCCI LMIC library:
        case EV_RXSTART:
            // Do not print anything for this event or it will mess up timing.
            break;

        case EV_TXSTART:
            setTxIndicatorsOn();
            printEvent(timestamp, ev);
            #ifdef USE_SERIAL
            Serial.println(getFreq());    
            #endif     
            break;               

        case EV_JOIN_TXCOMPLETE:
        case EV_TXCANCELED:
            setTxIndicatorsOn(false);
            printEvent(timestamp, ev);
            break;               
#endif
        case EV_JOINED:
            setTxIndicatorsOn(false);
            printEvent(timestamp, ev);
            printSessionKeys();

            // Disable link check validation.
            // Link check validation is automatically enabled
            // during join, but because slow data rates change
            // max TX size, it is not used in this example.                    
            LMIC_setLinkCheckMode(0);

            // The doWork job has probably run already (while
            // the node was still joining) and have rescheduled itself.
            // Cancel the next scheduled doWork job and re-schedule
            // for immediate execution to prevent that any uplink will
            // have to wait until the current doWork interval ends.
            os_clearCallback(&doWorkJob);
            os_setCallback(&doWorkJob, doWorkCallback);
            break;

        case EV_TXCOMPLETE:
            // Transmit completed, includes waiting for RX windows.
            setTxIndicatorsOn(false);   
            printEvent(timestamp, ev);
            printFrameCounters();

            // Check if downlink was received
            if (LMIC.dataLen != 0 || LMIC.dataBeg != 0)
            {
                uint8_t fPort = 0;
                _RX_COUNT++;
                _DW_COUNT++;
                if (LMIC.txrxFlags & TXRX_PORT)
                {
                    fPort = LMIC.frame[LMIC.dataBeg-1];
                }
                if (LMIC.txrxFlags & TXRX_ACK){
                    //Serial.println(F("========> Recebido ACK"));
                }
                printDownlinkInfo();
                processDownlink(timestamp, fPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);                
            }
            _TX_REAL++;
            break;     
          
        // Below events are printed only.
        case EV_SCAN_TIMEOUT:
        case EV_BEACON_FOUND:
        case EV_BEACON_MISSED:
        case EV_BEACON_TRACKED:
        case EV_RFU1:                    // This event is defined but not used in code
        case EV_JOINING:        
        case EV_JOIN_FAILED:           
        case EV_REJOIN_FAILED:
        case EV_LOST_TSYNC:
        case EV_RESET:
        case EV_RXCOMPLETE:
        case EV_LINK_DEAD:
        case EV_LINK_ALIVE:
#ifdef MCCI_LMIC
        // Only supported in MCCI LMIC library:
        case EV_SCAN_FOUND:              // This event is defined but not used in code 
#endif
            printEvent(timestamp, ev);    
            break;

        default: 
            printEvent(timestamp, "Unknown Event");    
            break;
    }
}


static void doWorkCallback(osjob_t* job)
{
    // Event hander for doWorkJob. Gets called by the LMIC scheduler.
    // The actual work is performed in function processWork() which is called below.

    ostime_t timestamp = os_getTime();
    #ifdef USE_SERIAL
        serial.println();
        printEvent(timestamp, "doWork job started", PrintTarget::Serial);
    #endif    

    // Do the work that needs to be performed.
    processWork(timestamp);

    // This job must explicitly reschedule itself for the next run.
    ostime_t startAt = timestamp + sec2osticks((int64_t)doWorkIntervalSeconds);
    os_setTimedCallback(&doWorkJob, startAt, doWorkCallback);    
}


lmic_tx_error_t scheduleUplink(uint8_t fPort, uint8_t* data, uint8_t dataLength, bool confirmed = false)
{
    // This function is called from the processWork() function to schedule
    // transmission of an uplink message that was prepared by processWork().
    // Transmission will be performed at the next possible time

    ostime_t timestamp = os_getTime();
    printEvent(timestamp, "Packet queued");

    lmic_tx_error_t retval = LMIC_setTxData2(fPort, data, dataLength, confirmed ? 1 : 0);
    timestamp = os_getTime();

    if (retval == LMIC_ERROR_SUCCESS)
    {
        #ifdef CLASSIC_LMIC
            // For MCCI_LMIC this will be handled in EV_TXSTART        
            setTxIndicatorsOn();  
        #endif        
    }
    else
    {
        String errmsg; 
        #ifdef USE_SERIAL
            errmsg = "LMIC Error: ";
            #ifdef MCCI_LMIC
                errmsg.concat(lmicErrorNames[abs(retval)]);
            #else
                errmsg.concat(retval);
            #endif
            printEvent(timestamp, errmsg.c_str(), PrintTarget::Serial);
        #endif
        // #ifdef USE_DISPLAY
            errmsg = "LMIC Err: ";
            errmsg.concat(retval);
            printEvent(timestamp, errmsg.c_str(), PrintTarget::Display);
        // #endif         
    }
    return retval;    
}

static volatile uint16_t counter_ = 0;

uint16_t getCounterValue()
{
    // Increments counter and returns the new value.
    delay(50);         // Fake this takes some time
    return ++counter_;
}

void resetCounter()
{
    // Reset counter to 0
    counter_ = 0;
}


void processWork(ostime_t doWorkJobTimeStamp)
{
    // This function is called from the doWorkCallback() 
    // callback function when the doWork job is executed.

    // Uses globals: payloadBuffer and LMIC data structure.

    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    // Skip processWork if using OTAA and still joining.
    if (LMIC.devaddr != 0)
    {
        // Collect input data.
        // For simplicity LMIC-node uses a counter to simulate a sensor. 
        // The counter is increased automatically by getCounterValue()
        // and can be reset with a 'reset counter' command downlink message.

        uint16_t counterValue = getCounterValue();
        ostime_t timestamp = os_getTime();
        
        #ifdef USE_SERIAL
            printEvent(timestamp, "Input data collected", PrintTarget::Serial);
            printSpaces(serial, MESSAGE_INDENT);
            serial.print(F("COUNTER value: "));
            serial.println(counterValue);
        #endif    

        // For simplicity LMIC-node will try to send an uplink
        // message every time processWork() is executed.

        // Schedule uplink message if possible
        if (LMIC.opmode & OP_TXRXPEND)
        {
            // TxRx is currently pending, do not send.
            #ifdef USE_SERIAL
                printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
            #endif    
        }
        else
        {
            _TX_COUNT++;
            // Prepare uplink payload.
            uint8_t fPort = 10;
            payloadBuffer[0] = counterValue >> 8;
            payloadBuffer[1] = counterValue & 0xFF;
            uint8_t payloadLength = 2;

            //scheduleUplink(fPort, payloadBuffer, payloadLength);
            strcpy((char*)mydata, "c|");
            snprintf(data, sizeof(data), "%f", currentBoard.getElectricalConsumption());
            strcat((char*)mydata, data);
            Serial.printf("Mydata: %s\n", mydata);
            scheduleUplink(10, mydata, 240);
        }
    }
}    
 

void processDownlink(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t* data, uint8_t dataLength)
{
    // This function is called from the onEvent() event handler
    // on EV_TXCOMPLETE when a downlink message was received.

    // Implements a 'reset counter' command that can be sent via a downlink message.
    // To send the reset counter command to the node, send a downlink message
    // (e.g. from the TTN Console) with single byte value resetCmd on port cmdPort.

    const uint8_t cmdPort = 100;
    const uint8_t resetCmd= 0xC0;

    if (fPort == cmdPort && dataLength == 1 && data[0] == resetCmd)
    {
        #ifdef USE_SERIAL
            printSpaces(serial, MESSAGE_INDENT);
            serial.println(F("Reset cmd received"));
        #endif
        ostime_t timestamp = os_getTime();
        resetCounter();
        printEvent(timestamp, "Counter reset", PrintTarget::All, false);
    }          
}


void setup() 
{
    // boardInit(InitType::Hardware) must be called at start of setup() before anything else.
    bool hardwareInitSucceeded = boardInit(InitType::Hardware);
    delay(1000);
    #ifdef USE_DISPLAY 
        initDisplay();
    #endif

    #ifdef USE_SERIAL
        initSerial(MONITOR_SPEED, WAITFOR_SERIAL_S);
    #endif    

    boardInit(InitType::PostInitSerial);

    #if defined(USE_SERIAL) || defined(USE_DISPLAY)
        printHeader();
    #endif

    if (!hardwareInitSucceeded)
    {   
        #ifdef USE_SERIAL
            serial.println(F("Error: hardware init failed."));
            serial.flush();            
        #endif
        abort();
    }

    initLmic(0, DR_SF7, DefaultABPTxPower);

    // Place code for initializing sensors etc. here.

    resetCounter();

    if (activationMode == ActivationMode::OTAA)
    {
        LMIC_startJoining();
    }

    // Schedule initial doWork job for immediate execution.
    os_setCallback(&doWorkJob, doWorkCallback);
}



void loop() 
{   
    os_runloop_once();
    #ifdef USE_DISPLAY
        _STATUS_DR = getDR();
        _STATUS_LMIC=(
        LMIC.opmode&OP_JOINING? F("JOINING"):
            (LMIC.opmode&OP_TXDATA? F("TRANSMIT"):
                (LMIC.opmode&OP_POLL? F("SND_MPTY_UP"):
                    (LMIC.opmode&OP_NEXTCHNL? F("NEXTCHANNL"):
                        F("OUTRO")))));
        display_status();
    #endif
}
