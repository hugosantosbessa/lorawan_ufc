#ifdef BSF_HELTEC_WIFI_LORA_32_V3

#include "bsf_heltec_wifi_lora_32_v3.h"

#ifdef USE_SERIAL
    HardwareSerial& BSF::serial = Serial;
#endif

#ifdef USE_LED
    EasyLed BSF::led(LED, EasyLed::ActiveLevel::Low);
#endif

#ifdef USE_DISPLAY
    Adafruit_SSD1306 BSF::display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, RST_OLED);
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
                pinMode(RST_OLED, OUTPUT);
                digitalWrite(RST_OLED, LOW);
                delay(20);
                digitalWrite(RST_OLED, HIGH);

                Wire.begin(SDA_OLED, SCL_OLED);
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