/** @file fram.h
* 
* @brief FRAM interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl.  All rights reserved.
*/ 

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

#define DATA_SIZE_BYTES 16

typedef union {
    uint32_t u32;
    uint8_t u8[4];
} u32_u8_t;

typedef union {
    uint16_t u16;
    uint8_t u8[2];
} u16_u8_t;

typedef union {
    int16_t i16;
    uint8_t u8[2];
} i16_u8_t;

typedef struct 
{
    u32_u8_t fram_counter;  // 4 bytes (total)
    i16_u8_t accel_x;       // 6
    i16_u8_t accel_y;       // 8
    i16_u8_t accel_z;       // 10
    i16_u8_t temp;          // 12
    i16_u8_t pressure;      // 14
    u16_u8_t id;            // 16
} we_power_data_t;

typedef union {
    we_power_data_t data_fields;
    uint8_t data_bytes[DATA_SIZE_BYTES];
} we_power_data_ble_adv_t;

#endif /* APP_FRAM_H */

