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
 *  Author:        Hugo S. C. Bessa, Francisco Helder C. Santos
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
 *                 U8g2                       https://github.com/olikraus/u8g2
 *                 EasyLed                    https://github.com/lnlp/EasyLed
 *
 ******************************************************************************/

#include "LMICNode.h"

/*
    INITIALIZE STATIC VARIABLES OF CLASS LMICNode
*/
uint8_t LMICNode::fPort;
uint8_t LMICNode::mydata[];
uint8_t LMICNode::sizeData;
volatile uint16_t LMICNode::counter_ = 0;
#ifdef MCCI_LMIC   
    const char * const LMICNode::lmicEventNames[] = { LMIC_EVENT_NAME_TABLE__INIT };
    const char * const LMICNode::lmicErrorNames[] = { LMIC_ERROR_NAME__INIT };
#else
    const char * const LMICNode::lmicEventNames[] = { LEGACY_LMIC_EVENT_NAME_TABLE__INIT };
#endif

#if defined(ABP_ACTIVATION) && defined(ABP_DEVICEID)
    const char LMICNode::deviceId[] = ABP_DEVICEID;
#elif defined(DEVICEID)
    const char LMICNode::deviceId[] = DEVICEID;
#else
    const char LMICNode::deviceId[] = DEVICEID_DEFAULT;
#endif

#if defined(USE_SERIAL) || defined(USE_DISPLAY)
    
    void LMICNode::printChars(Print& printer, char ch, uint8_t count, bool linefeed)
    {
        for (uint8_t i = 0; i < count; ++i)
        {
            printer.print(ch);
        }
        if (linefeed)
        {
            printer.println();
        }
    }


    void LMICNode::printSpaces(Print& printer, uint8_t count, bool linefeed)
    {
        printChars(printer, ' ', count, linefeed);
    }


    void LMICNode::printHex(Print& printer, uint8_t* bytes, size_t length, bool linefeed, char separator)
    {
        for (size_t i = 0; i < length; ++i)
        {
            if (i > 0 && separator != 0)
            {
                printer.print(separator);
            }
            if (bytes[i] <= 0x0F)
            {
                printer.print('0');
            }
            printer.print(bytes[i], HEX);        
        }
        if (linefeed)
        {
            printer.println();
        }
    }


    void LMICNode::setTxIndicatorsOn(bool on)
    {
        if (on)
        {
            #ifdef USE_LED
                BSF::led.on();
            #endif
            #ifdef USE_DISPLAY
                // displayTxSymbol(true);
            #endif           
        }
        else
        {
            #ifdef USE_LED
                BSF::led.off();
            #endif
            #ifdef USE_DISPLAY
                // displayTxSymbol(false);
            #endif           
        }        
    }
#endif

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

void LMICNode::initDisplay()
    {
        // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
        if(!BSF::display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, false, false)) {
            Serial.println(F("SSD1306 allocation failed"));
            for(;;); // Don't proceed, loop forever
        }

        BSF::display.clearDisplay();

        BSF::display.setTextSize(2); // Draw 2X-scale text
        BSF::display.setTextColor(SSD1306_WHITE);
        BSF::display.setCursor(0, 0);
        BSF::display.println(F("LoRa OK"));
        BSF::display.display();

        BSF::display.setCursor(0, 40);
        BSF::display.setTextSize(1);
        BSF::display.println(F("Iniciado. Iniciando job."));
        BSF::display.display();

}

void LMICNode::displayStatus(){
    _STATUS_DR = getDR();
        _STATUS_LMIC=(
        LMIC.opmode&OP_JOINING? F("JOINING"):
            (LMIC.opmode&OP_TXDATA? F("TRANSMIT"):
                (LMIC.opmode&OP_POLL? F("SND_MPTY_UP"):
                    (LMIC.opmode&OP_NEXTCHNL? F("NEXTCHANNL"):
                        F("OUTRO")))));
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
    
    BSF::display.clearDisplay();
    BSF::display.setCursor(0, 0);
    BSF::display.print(F("DEV: "));
    BSF::display.println(ABP_DEVADDR, HEX);

    // https://github.com/sandeepmistry/arduino-LoRa/pull/291
    u1_t bw = getBw(LMIC.rps);
    BSF::display.setCursor(0, 10);
    BSF::display.print(F("RSSI: "));
    BSF::display.print(LMIC.rssi);
    BSF::display.println("  "+_STATUS_DR);

    BSF::display.setCursor(0, 20);
    BSF::display.print(F("TX(c): "));
    BSF::display.print(_TX_COUNT);
    BSF::display.print(F(" ["));
    BSF::display.print(_TX_REAL);
    BSF::display.println(F("]"));

    BSF::display.setCursor(0, 30);
    BSF::display.print(F("RX(c): "));
    BSF::display.print(_RX_COUNT);

    BSF::display.print(F("  DW(c): "));
    BSF::display.println(_DW_COUNT);

    BSF::display.setCursor(0, 40);
    BSF::display.print(F("JOIN: "));
    BSF::display.println(_STATUS_JOIN);

    BSF::display.setCursor(0, 50);
    BSF::display.print(F("STATUS: "));
    BSF::display.println(_STATUS_LMIC);
        
    BSF::display.display();
}
#endif

#ifdef USE_SERIAL
    // HardwareSerial& serial = Serial;
    bool initSerial(unsigned long speed, int16_t timeoutSeconds)
    {
        // Initializes the serial port.
        // Optionally waits for serial port to be ready.
        // Will display status and progress on display (if enabled)
        // which can be useful for tracing (e.g. ATmega328u4) serial port issues.
        // A negative timeoutSeconds value will wait indefinitely.
        // A value of 0 (default) will not wait.
        // Returns: true when serial port ready,
        //          false when not ready.

        BSF::serial.begin(speed);

        #if WAITFOR_SERIAL_S != 0
            if (timeoutSeconds != 0)
            {   
                bool indefinite = (timeoutSeconds < 0);
                uint16_t secondsLeft = timeoutSeconds; 
                #ifdef USE_DISPLAY
                    BSF::display.setCursor(0, ROW_1);
                    BSF::display.print(F("Waiting for"));
                    BSF::display.setCursor(0,  ROW_2);                
                    BSF::display.print(F("serial port"));
                #endif

                while (!BSF::serial && (indefinite || secondsLeft > 0))
                {
                    if (!indefinite)
                    {
                        #ifdef USE_DISPLAY
                            BSF::display.clearLine(ROW_4);
                            BSF::display.setCursor(0, ROW_4);
                            BSF::display.print(F("timeout in "));
                            BSF::display.print(secondsLeft);
                            BSF::display.print('s');
                        #endif
                        --secondsLeft;
                    }
                    delay(1000);
                }  
                #ifdef USE_DISPLAY
                    BSF::display.setCursor(0, ROW_4);
                    if (BSF::serial)
                    {
                        BSF::display.print(F("Connected"));
                    }
                    else
                    {
                        BSF::display.print(F("NOT connected"));
                    }
                #endif
            }
        #endif

        return BSF::serial;
    }
#endif

osjob_t  doWorkJob;

// Note: LoRa module pin mappings are defined in the Board Support Files.

// Set LoRaWAN keys defined in lorawan-keys.h.
#ifdef OTAA_ACTIVATION
    const u1_t  LMICNode::DEVEUI[8]  PROGMEM = { OTAA_DEVEUI } ;
    const u1_t  LMICNode::APPEUI[8]  PROGMEM = { OTAA_APPEUI };
    const u1_t LMICNode:: APPKEY[16] PROGMEM = { OTAA_APPKEY };
    // Below callbacks are used by LMIC for reading above values.
    void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
    void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
    void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }    
#else
    // ABP activation
    const u4_t LMICNode::DEVADDR = ABP_DEVADDR ;
    const u1_t LMICNode::NWKSKEY[16] PROGMEM = { ABP_NWKSKEY };
    const u1_t  LMICNode::APPSKEY[16] PROGMEM = { ABP_APPSKEY };
    // Below callbacks are not used be they must be defined.
    void os_getDevEui (u1_t* buf) { }
    void os_getArtEui (u1_t* buf) { }
    void os_getDevKey (u1_t* buf) { }
#endif


String LMICNode::getDR(){
    byte sf = LMIC.datarate;
    char v[10];
    snprintf(v, 10, "SF%d%c", (sf!=6? 12-sf:8), (sf!=6? ' ':'C'));
    return v;
}


String LMICNode::getFreq(){
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


int16_t LMICNode::getSnrTenfold()
{
    // Returns ten times the SNR (dB) value of the last received packet.
    // Ten times to prevent the use of float but keep 1 decimal digit accuracy.
    // Calculation per SX1276 datasheet rev.7 §6.4, SX1276 datasheet rev.4 §6.4.
    // LMIC.snr contains value of PacketSnr, which is 4 times the actual SNR value.
    return (LMIC.snr * 10) / 4;
}


int16_t LMICNode::getRssi(int8_t snr)
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


void    LMICNode::printEvent(ostime_t timestamp, 
                const char * const message, 
                PrintTarget target,
                bool clearDisplayStatusRow,
                bool eventLabel)
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
            printChars(BSF::serial, '0', zerosCount);
            BSF::serial.print(timeString);
            BSF::serial.print(":  ");
            if (eventLabel)
            {
                BSF::serial.print(F("Event: "));
            }
            BSF::serial.println(message);
        }
    #endif   
}           

void LMICNode::printEvent(ostime_t timestamp, 
                ev_t ev, 
                PrintTarget target, 
                bool clearDisplayStatusRow)
{
    #if defined(USE_DISPLAY) || defined(USE_SERIAL)
        printEvent(timestamp, lmicEventNames[ev], target, clearDisplayStatusRow, true);
    #endif
}


void LMICNode::printFrameCounters(PrintTarget target)
{
    #ifdef USE_SERIAL
        if (target == PrintTarget::Serial || target == PrintTarget::All)
        {
            printSpaces(BSF::serial, MESSAGE_INDENT);
            BSF::serial.print(F("Up: "));
            BSF::serial.print(LMIC.seqnoUp);
            BSF::serial.print(F(",  Down: "));
            BSF::serial.println(LMIC.seqnoDn);        
        }
    #endif        
}      


void LMICNode::printSessionKeys()
{    
    #if defined(USE_SERIAL) && defined(MCCI_LMIC)
        u4_t networkId = 0;
        devaddr_t deviceAddress = 0;
        u1_t networkSessionKey[16];
        u1_t applicationSessionKey[16];
        LMIC_getSessionKeys(&networkId, &deviceAddress, 
                            networkSessionKey, applicationSessionKey);

        LMICNode::printSpaces(BSF::serial, MESSAGE_INDENT);    
        BSF::serial.print(F("Network Id: "));
        BSF::serial.println(networkId, DEC);

        LMICNode::printSpaces(BSF::serial, MESSAGE_INDENT);    
        BSF::serial.print(F("Device Address: "));
        BSF::serial.println(deviceAddress, HEX);

        LMICNode::printSpaces(BSF::serial, MESSAGE_INDENT);    
        BSF::serial.print(F("Application Session Key: "));
        printHex(BSF::serial, applicationSessionKey, 16, true, '-');

        LMICNode::printSpaces(BSF::serial, MESSAGE_INDENT);    
        BSF::serial.print(F("Network Session Key:     "));
        printHex(BSF::serial, networkSessionKey, 16, true, '-');
    #endif
}


void LMICNode::printDownlinkInfo(void)
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
            printSpaces(BSF::serial, MESSAGE_INDENT);    
            BSF::serial.println(F("Downlink received"));

            printSpaces(BSF::serial, MESSAGE_INDENT);
            BSF::serial.print(F("RSSI: "));
            BSF::serial.print(rssi);
            BSF::serial.print(F(" dBm,  SNR: "));
            BSF::serial.print(snr);                        
            BSF::serial.print(".");                        
            BSF::serial.print(snrDecimalFraction);                        
            BSF::serial.println(F(" dB"));

            printSpaces(BSF::serial, MESSAGE_INDENT);    
            BSF::serial.print(F("Port: "));
            BSF::serial.println(fPort);
   
            if (dataLength != 0)
            {
                printSpaces(BSF::serial, MESSAGE_INDENT);
                BSF::serial.print(F("Length: "));
                BSF::serial.println(LMIC.dataLen);                   
                printSpaces(BSF::serial, MESSAGE_INDENT);    
                BSF::serial.print(F("Data: "));
                printHex(BSF::serial, LMIC.frame+LMIC.dataBeg, LMIC.dataLen, true, ' ');
            }
        #endif
    #endif
} 


void LMICNode::printHeader(void)
{
    #ifdef USE_SERIAL
        BSF::serial.println(F("\n\nLMIC-node\n"));
        BSF::serial.print(F("Device-id:     "));
        BSF::serial.println(LMICNode::deviceId);            
        BSF::serial.print(F("LMIC library:  "));
        #ifdef MCCI_LMIC  
            BSF::serial.println(F("MCCI"));
        #else
            BSF::serial.println(F("Classic [Deprecated]")); 
        #endif
        BSF::serial.print(F("Activation:    "));
        #ifdef OTAA_ACTIVATION  
            BSF::serial.println(F("OTAA"));
        #else
            BSF::serial.println(F("ABP")); 
        #endif
        #if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
            BSF::serial.print(F("LMIC debug:    "));  
            BSF::serial.println(LMIC_DEBUG_LEVEL);
        #endif
        BSF::serial.print(F("Interval:      "));
        BSF::serial.print(LMICNode::doWorkIntervalSeconds);
        BSF::serial.println(F(" seconds"));
        if (activationMode == ActivationMode::OTAA)
        {
            BSF::serial.println();
        }
    #endif
}     


void LMICNode::setADR(bit_t adrEnabled){
    this->adrEnabled = adrEnabled;
}


#ifdef ABP_ACTIVATION
    void LMICNode::setAbpParameters(dr_t dataRate, s1_t txPower) 
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

    void LMICNode::setDataRate(dr_t abpDataRate){
        this->abpDataRate = abpDataRate;
    }
    
    void LMICNode::setTxPower(s1_t abpTxPower){
        this->abpTxPower = abpTxPower;
    }
#endif //ABP_ACTIVATION

void LMICNode::initLmic() 
{
    // ostime_t timestamp = os_getTime();

    // Initialize LMIC runtime environment
    #ifdef BSF_HELTEC_WIFI_LORA_32_V3
        const Arduino_LMIC::HalConfiguration_t myConfig;
        const lmic_pinmap *pPinMap = Arduino_LMIC::GetPinmap_ThisBoard();
        os_init_ex(pPinMap);
    #else
        os_init();
    #endif
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
            BSF::serial.print(F("Clock Error:   "));
            BSF::serial.print(LMIC_CLOCK_ERROR_PPM);
            BSF::serial.print(" ppm (");
            BSF::serial.print(clockError);
            BSF::serial.println(")");            
        #endif
    #endif

    #ifdef MCCI_LMIC
        // Register a custom eventhandler and don't use default onEvent() to enable
        // additional features (e.g. make EV_RXSTART available). User data pointer is omitted.
        LMIC_registerEventCb(&LMICNode::onLmicEvent, nullptr);
    #endif
}

#ifdef MCCI_LMIC 
void LMICNode::onLmicEvent(void *pUserData, ev_t ev)
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
                printSpaces(BSF::serial, MESSAGE_INDENT);
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


void LMICNode::doWorkCallback(osjob_t* job)
{
    // Event hander for doWorkJob. Gets called by the LMIC scheduler.
    // The actual work is performed in function processWork() which is called below.

    ostime_t timestamp = os_getTime();
    #ifdef USE_SERIAL
        BSF::serial.println();
        printEvent(timestamp, "doWork job started", PrintTarget::Serial);
    #endif    

    // Do the work that needs to be performed.
    processWork(timestamp);

    // This job must explicitly reschedule itself for the next run.
    ostime_t startAt = timestamp + sec2osticks((int64_t)LMICNode::doWorkIntervalSeconds);
    os_setTimedCallback(&doWorkJob, startAt, doWorkCallback);    
}

void LMICNode::processWork(ostime_t doWorkJobTimeStamp) {
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

        uint16_t counterValue = LMICNode::getCounterValue();
        ostime_t timestamp = os_getTime();
        
        #ifdef USE_SERIAL
            printEvent(timestamp, "Input data collected", PrintTarget::Serial);
            printSpaces(BSF::serial, MESSAGE_INDENT);
            BSF::serial.print(F("COUNTER value: "));
            BSF::serial.println(counterValue);
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
            // Prepare uplink payload.
            prepareUplink();
        }
    }
}    

lmic_tx_error_t LMICNode::scheduleUplink(uint8_t fPort, uint8_t* data, uint8_t dataLength, bool confirmed)
{
    // This function is called from the prepareUplink() function to schedule
    // transmission of an uplink message that was prepared by processWork().
    // Transmission will be performed at the next possible time

    ostime_t timestamp = os_getTime();
    printEvent(timestamp, "Packet queued");
    _TX_COUNT++;

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
                errmsg.concat(LMICNode::lmicErrorNames[abs(retval)]);
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


uint16_t LMICNode::getCounterValue()
{
    // Increments counter and returns the new value.
    delay(50);         // Fake this takes some time
    return ++counter_;
}

void LMICNode::resetCounter()
{
    // Reset counter to 0
    counter_ = 0;
}

 

void LMICNode::processDownlink(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t* data, uint8_t dataLength)
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
            printSpaces(BSF::serial, MESSAGE_INDENT);
            BSF::serial.println(F("Reset cmd received"));
        #endif
        ostime_t timestamp = os_getTime();
        resetCounter();
        printEvent(timestamp, "Counter reset", PrintTarget::All, false);
    }          
}


void LMICNode::initHardware(){
    // boardInit(InitType::Hardware) must be called at start of setup() before anything else.
    bool hardwareInitSucceeded = BSF::boardInit(InitType::Hardware);
    #ifdef USE_DISPLAY 
        initDisplay();
    #endif

    #ifdef USE_SERIAL
        initSerial(MONITOR_SPEED, WAITFOR_SERIAL_S);
    #endif    

    BSF::boardInit(InitType::PostInitSerial);
    
    delay(3000);
    #if defined(USE_SERIAL) || defined(USE_DISPLAY)
        printHeader();
    #endif

    if (!hardwareInitSucceeded)
    {   
        #ifdef USE_SERIAL
            BSF::serial.println(F("Error: hardware init failed."));
            BSF::serial.flush();            
        #endif
        abort();
    }
}


void LMICNode::initTransmission(){

    resetCounter();

    if (activationMode == ActivationMode::OTAA)
    {
        LMIC_startJoining();
    }

    // Schedule initial doWork job for immediate execution.
    os_setCallback(&doWorkJob, doWorkCallback);
}

void LMICNode::loop_node() 
{
    os_runloop_once();
    #ifdef USE_DISPLAY
        displayStatus();
    #endif
}
