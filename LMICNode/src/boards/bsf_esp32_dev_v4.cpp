#ifdef BSF_ESP32

#include "bsf_esp32_dev_v4.h"

// Pin mappings for LoRa tranceiver
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 26,                      // See remark about LORA_RST above.
    .dio = { /*dio0*/ 14, /*dio1*/ 27, /*dio2*/ LMIC_UNUSED_PIN }
#ifdef MCCI_LMIC
    ,
    .rxtx_rx_active = 0,
    .rssi_cal = 10,
    .spi_freq = 8000000     /* 8 MHz */
#endif    
};

#ifdef USE_SERIAL
    HardwareSerial& BSF::serial = Serial;
#endif

#ifdef USE_LED
    EasyLed BSF::led(LED_BUILTIN, EasyLed::ActiveLevel::Low);
#endif

#ifdef USE_DISPLAY
    Adafruit_SSD1306 BSF::display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
#endif

bool BSF::boardInit(InitType initType)
{
    // This function is used to perform board specific initializations.
    // Required as part of standard template.

    // InitType::Hardware        Must be called at start of setup() before anything else.
    // InitType::PostInitSerial  Must be called after initSerial() before other initializations.    

    bool success = true;
    switch (initType)
    {
        case InitType::Hardware:
            // Note: Serial port and display are not yet initialized and cannot be used use here.

            #ifdef USE_DISPLAY
                // Initialize I2C Wire object with GPIO pins the display is connected to.
                // These pins will be remembered and will not change if any library
                // later calls Wire.begin() without parameters.
                Wire.begin(4, 15);
            #endif
            break;

        case InitType::PostInitSerial:
            // Note: If enabled Serial port and display are already initialized here.
            // No actions required for this board.
            break;
    }
    return success;
}
#endif