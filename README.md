# project LMICNode

This repository contains a project that aims to automate the configuration of different end devices associated with SmartUFC's LoRaWAN communication network.

## Getting Started

This section is intended to get a user into a working state, starting with a machine that may have never installed LMICNode. It covers prerequisites, ways to get LMICNode, and ways to build LMICNode.


### Prerequisites

The following is required to use LMICNode:

- **A supported** LoRa **development board** or development board with external Semtech SX127*x*, HopeRF RFM9*x* or HPDTek HPD1*x*A SPI LoRa module.
- **A computer with [PlatformIO](https://platformio.org/) installed**. PlatformIO is used instead of Arduino IDE because more flexible, more powerful and it better supports cross-platform development. For installation see [PlatformIO installation instructions](https://docs.platformio.org/en/latest/integration/ide/vscode.html).
- **Internet connection**. PlatformIO will automatically download and install packages (Arduino cores, toolchains and libraries) as needed.  
If PlatformIO is freshly installed the downloading may take some time. Once installed it will be possible to work offline.
- **USB cable**. For boards without onboard USB also a **USB to serial adapter** is needed.
- **Wiring**: Some LoRa development boards require manual wiring of LoRa DIO1 (see table below). When using a development board with external LoRa module then everything must be manually wired. For wiring details see the board's Board Support File (BSF).
- **Skills**: You should already be familiar with compiling and uploading basic Arduino sketches to your board and how to use a serial monitor.

With this, you can clone this project and use LMICNode.


## Settings

### 4.1 Board selection

In platformio.ini the board must be selected. Select only a single board by uncommenting the line with its board-id. Comment the line starting with "\<platformio.ini board selector guard\>". The guard is explicitly added to prevent that PlatformIO will compile LMIC-node for ALL listed boards, when no board is selected.

Warnings:

- When the guard is disabled and no board is selected then PlatformIO will compile for ALL listed boards!
- The Serial Monitor can only be started when a board has been selected in `platformio.ini`.

```ini
[platformio]
default_envs = 
    <platformio.ini board selector guard> Comment this line and uncomment one board-id below:

    ; LoRa development boards with integrated LoRa support:

    ; Board-id                            Board name
    ;---------                            ----------
    ; ttgo_lora32_v1                    ; TTGO LoRa32 v1.3
    ; pico                              ; Raspberry Pi Pico
```
### Common settings

```ini
[common]

monitor_speed = 115200                 ; No need to change this.

build_flags =
    -D DO_WORK_INTERVAL_SECONDS=15

    -D ABP_ACTIVATION                ; Use ABP instead of OTAA activation.
    ;
    ; -D WAITFOR_SERIAL_SECONDS=10     ; Can be used to override the default value (10).
    ;                                    Is used only for boards with default set to != 0 in BSF.
    ;
    ; -D LMIC_CLOCK_ERROR_PPM=0        ; If not defined defines, otherwise overrides value defined in BSF.
    ;                                    Is for testing purposes only.
    ;                                    Do not enable this unless you explicitly know what you are doing.

lib_deps =
    olikraus/U8g2                      ; OLED display library
    lnlp/EasyLed                       ; LED library
    Wire
;   ███ Add additional libraries for User Code below this line ███
```

**monitor_speed**  
Sets the monitor speed for the serial port. 115200 bps should be fine for most purposes and does not need to be changed.

**DO_WORK_INTERVAL_SECONDS**  
Defines the interval for when the doWork job runs where the actual work is done.
Be aware that this is also the interval that uplink messages will be sent. The interval should not exceed TTN's fair use policy and regulatory constraints.

**ABP_ACTIVATION**  
If enabled will use ABP activation instead of OTAA activation (default).

**WAITFOR_SERIAL_SECONDS**  
Can be used to overrule the default value defined in BSF for testing purposes but normally not needed to change this. This setting only has effect for boards where a default value (>0) is already defined and will not have effect for other boards.
'Wait for serial' only is useful when USB functionality is provided by the MCU.

A 'wait for serial' delay of 10 seconds is configured by default in boards where USB support is integrated into the MCU (instead of using onboard USB to serial).
Waiting until the serial (over USB) port is actually ready (via 'wait for serial') prevents the first output printed to the serial port getting lost.

A positive value will wait with a countdown delay (visible on display) for the number of seconds specified. A value of 0 will not wait. A value of -1 will wait indefinitely.

Be aware that the serial port must not only be ready but also a serial monitor needs to be connected. The countdown gives some time to start the serial monitor if not already started. A timeout value prevents that the node waits indefinitely if not connected to a computer. It will continue when the countdown ends.

### LoRaWAN library settings

#### MCCI LoRaWAN LMIC library settings

```ini
[mcci_lmic]
; LMIC-node was tested with MCCI LoRaWAN LMIC library v3.3.0.
; Some changes have been announced for future versions of the MCCI library
; which may be incompatible with LMIC-node. In case of problems just
; use mcci-catena/MCCI LoRaWAN LMIC library@3.3.0 below which will
; explicitly use v3.3.0 of the library.
; Perform PlatformIO: Clean after changing library version and
; in case of issues remove the old version from .pio/libdeps/*.

; Note: LMIC_PRINTF_TO is defined for each board separately
;       in the board specific sections. Don't define it in this section.

lib_deps =
    ; Only ONE of below LMIC libraries should be enabled.
    mcci-catena/MCCI LoRaWAN LMIC library           ; MCCI LMIC library (latest release)
    ; mcci-catena/MCCI LoRaWAN LMIC library@3.3.0   ; MCCI LMIC library v3.3.0

build_flags =
    ; Use platformio.ini for settings instead lmic_project_config.h.
    -D ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS

    ; Ping and beacons not supported for class A, disable to save memory.
    -D DISABLE_PING
    -D DISABLE_BEACONS

    ; -D LMIC_DEBUG_LEVEL=1            ; 0, 1 or 2
    
    ; -D CFG_sx1272_radio=1            ; Use for SX1272 radio
    -D CFG_sx1276_radio=1              ; Use for SX1276 radio
    -D USE_ORIGINAL_AES                ; Faster but larger, see docs
    ; -D LMIC_USE_INTERRUPTS           ; Not tested or supported on many platforms
    ; -D LMIC_ENABLE_DeviceTimeReq=1   ; Network time support

    ; --- Regional settings -----
    ; Enable only one of the following regions:    
    ; -D CFG_as923=1
    ; -D CFG_as923jp=1   
    ; -D CFG_au915=1
    ; -D CFG_cn490=1                   ; Not yet supported
    ; -D CFG_cn783=1                   ; Not yet supported
    ; -D CFG_eu433=1                   ; Not yet supported
    -D CFG_eu868=1
    ; -D CFG_in866=1
    ; -D CFG_kr920=1
    ; -D CFG_us915=1
```

**Regional setting**  
--- Regional settings --- is where the regional setting needs to be selected.
Uncomment the line with the appropriate region setting (and comment the other region settings). Only one region may be selected. By default CFG_eu868 (Europe) is selected.

**LMIC_DEBUG_LEVEL**  
This can be uncommented to allow debug information from the LMIC library to be printed.
Possible values are 0, 1, 2 and 3 where 0 provides no debugging information and 3 provides the most information.  
Be aware that enabling debug will increase memory requirements.

Other settings are listed for information but are not further explained here.
For more information see the MCCI LoRaWAN LMIC library documentation.


### Board specific settings

All options listed below are specified for each board separately. This is done for flexibility and to provide the optimum configuration for each board out of the box.

Example board section for ESP32 LoRa 32 TTGO V1 board:

```ini
[env:ttgo_lora32_v1]
; TTGO LoRa32 v1.3 (ESP32)
; Onboard OLED display SSD1306 0.96" 128x64.
platform = espressif32
board = ttgo-lora32-v1
framework = arduino
upload_speed = 921600
monitor_speed = ${common.monitor_speed}
lib_deps =
    ${common.lib_deps}    
    ${mcci_lmic.lib_deps}
    adafruit/Adafruit BusIO @ ^1.14.5
    adafruit/Adafruit GFX Library@^1.11.3
    adafruit/Adafruit SSD1306@^2.5.7
build_flags =
    ${common.build_flags}
    ${esp32.build_flags}
    ${mcci_lmic.build_flags} 
    -D BSF_TTGO_LORA32_V1
    -D BSFILE=\"boards/bsf_ttgo_lora32_v1.h\"
    -D MONITOR_SPEED=${common.monitor_speed}
    -D LMIC_PRINTF_TO=Serial    
    -D USE_SERIAL
    -D USE_LED                 
    -D USE_DISPLAY
```

**USE_SERIAL**  
If enabled status output will be send to the serial port to be viewed on serial monitor.

**USE_LED**  
If enabled it will use the onboard LED. The LED will be on during Tx/Rx transmission and off otherwise. For some boards the onboard LED cannot be used because of a hardware conflict, because the LED type is not supported (e.g. WS2812 RGB LED) or because there is no onboard user LED. If the onboard LED cannot be used then this option is disabled by default and the line will be commented to make this clear.

**USE_DISPLAY**  
If enabled status output will be send to the display.

**upload_protocol**  
For some boards it may be necessary to change the upload protocol, e.g. for pico change it from `picotool`.

**LMIC library**  
By default all boards with 32-bit MCU are configured to use the MCCI LoRaWAN LMIC library (MCCI LMIC) because this is the library that should be used. It is the most LoRaWAN compliant LMIC library for Arduino and it is actively maintained.  


### 4.5 Board Support Files

Each BSF contains the following settings. Except for DEVICEID_DEFAULT these settings are not enabled for each board. Values shown are examples.

```cpp
#define DEVICEID_DEFAULT "rpi-pico"  // Default deviceid value

// Wait for Serial
// Can be useful for boards with MCU with integrated USB support.
#define WAITFOR_SERIAL_SECONDS_DEFAULT 10   // -1 waits indefinitely  
```

**DEVICEID_DEFAULT**  
If no `DEVICEID` is defined and no `ABP_DEVICEID` is defined when ABP activation is used then `DEVICEID_DEFAULT` is used.  
There is no need to change this value in the BSF. `DEVICEID` and `ABP_DEVICEID` can be defined in file `lorawan-keys.h` where they are kept nicely together with the LoRaWAN keys of the same device.

**WAITFOR_SERIAL_SECONDS_DEFAULT**  
Defines the default value used for the 'wait for serial' timeout. This value is only defined in the BSF for boards where USB support is integrated into the MCU. It has no use for other boards.  
If there is need to change this value then don't change the default value in the BSF and change `WAITFOR_SERIAL_SECONDS` in the `[common]` section in `platformio.ini` instead.

## Instructions

For prerequisites see [Prerequisites](###prerequisites)

###  Select your board

In `platformio.ini`:

**Step 1a**: Select exactly one supported board by uncommenting the line with its board-id and name.

**Step 1b**: Comment the guard line to disable it. This is the line that starts with `<platformio.ini board selector guard>`.

### Select your LoRaWAN region

For MCCI LMIC (default for 32-bit boards):  
**Step 2:** In `platformio.ini` select you LoRaWAN region.  

### Provide the LoRaWAN keys for your node

**Step 3a**: If this is the first time: In the `keyfiles` folder copy file `lorawan-keys_example.h` to `lorawan-keys.h`.

**Step 3b**: Optional: if preferred, set values for `DEVICEID` and `ABP_DEVICEID` in `lorawan-keys.h` (otherwise default value will be used).

### Compile and upload

**Step 4a**: Compile the code.

**Step 4b**: Start the serial monitor (if USE_SERIAL is enabled).

**Step 4c**: Upload the code (and manually reset the device if needed).

**Your node should now be up and running.**

The node will be running but to be able to see the counter value in uplink messages displayed on the console.

