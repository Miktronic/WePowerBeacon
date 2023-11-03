/** @file app_accell.h
* 
* @brief Accelerometer interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl.  All rights reserved.
*/ 

#ifndef APP_ACCEL_H
#define APP_ACCEL_H

#include <stdint.h>

#define ACCEL_ERROR -1
#define ACCEL_SUCCESS 0

typedef struct 
{
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
} accel_data_t;

int app_accel_service(accel_data_t *accel_data);

#endif /* APP_FRAM_H */

