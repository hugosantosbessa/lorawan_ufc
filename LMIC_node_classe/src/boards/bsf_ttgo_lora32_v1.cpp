#include "bsf_ttgo_lora32_v1.h"

// Pin mappings for LoRa tranceiver
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,                      // See remark about LORA_RST above.
    .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 }
#ifdef MCCI_LMIC
    ,
    .rxtx_rx_active = 0,
    .rssi_cal = 10,
    .spi_freq = 8000000     /* 8 MHz */
#endif    
};

EasyLed BSF::led(LED_BUILTIN, EasyLed::ActiveLevel::Low);
Adafruit_SSD1306 BSF::display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
HardwareSerial& BSF::serial = Serial;

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
                Wire.begin();
            #endif
            break;

        case InitType::PostInitSerial:
            // Note: If enabled Serial port and display are already initialized here.
            break;
    }
    return success;
}
