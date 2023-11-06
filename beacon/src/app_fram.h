/** @file fram.h
* 
* @brief FRAM interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl.  All rights reserved.
*/ 

#ifndef APP_FRAM_H
#define APP_FRAM_H

#define FRAM_ERROR -1
#define FRAM_SUCCESS 0

typedef struct
{
    uint32_t frame_counter; //
    uint32_t serial_number;
    uint8_t  type;
    uint8_t  frame_inteval;
    uint16_t frame_max_limits;
    uint16_t sleep_min_interval;
    uint16_t sleep_after_wake;
    uint16_t cmd;
} fram_data_t;

int app_fram_service(uint32_t *counter);
int app_fram_read_data(fram_data_t *dat);
int app_fram_write_data(fram_data_t *dat);
#endif /* APP_FRAM_H */

