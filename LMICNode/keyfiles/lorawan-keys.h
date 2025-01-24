/*******************************************************************************
 *
 *  File:          lorawan-keys.h
 * 
 *  Function:      Lorawan keys required by LMICNode.
 *
 *  Copyright:     Copyright (c) 2023 Hugo S. C. Bessa, Francisco Helder C. Santos
 *
 * 
 *  Decription:    lorawan-keys.h defines LoRaWAN keys needed by the LMIC library.
 *                 It can contain keys for both OTAA and for ABP activation.
 *                 Only the keys for the used activation type need to be specified.
 *
 ******************************************************************************/

#pragma once

#ifndef LORAWAN_KEYS_H_
#define LORAWAN_KEYS_H_

// Optional: If DEVICEID is defined it will be used instead of the default defined in the BSF.
// #define DEVICEID "<deviceid>"

// Keys required for OTAA activation:

// End-device Identifier (u1_t[8]) in lsb format
#define OTAA_DEVEUI 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

// Application Identifier (u1_t[8]) in lsb format
#define OTAA_APPEUI 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

// Application Key (u1_t[16]) in msb format
#define OTAA_APPKEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00


// -----------------------------------------------------------------------------

// Optional: If ABP_DEVICEID is defined it will be used for ABP instead of the default defined in the BSF.
// #define ABP_DEVICEID "<deviceid>"

// Keys required for ABP activation:

// End-device Address (u4_t) in uint32_t format. 
// Note: The value must start with 0x (current version of TTN Console does not provide this).
#define ABP_DEVADDR 0xFEDCBA98

// Network Session Key (u1_t[16]) in msb format
#define ABP_NWKSKEY 0xC3,0x0E,0xBD,0xC5,0xC5,0xF5,0x92,0xFF,0x87,0xBF,0xD7,0xB3,0xF6,0x04,0xB0,0x2E

// Application Session K (u1_t[16]) in msb format
#define ABP_APPSKEY 0xC8,0xD7,0x62,0xA4,0x58,0xF3,0x21,0xF6,0xF9,0x26,0x34,0x20,0x5E,0x72,0x8E,0x6D

#endif  // LORAWAN_KEYS_H_
