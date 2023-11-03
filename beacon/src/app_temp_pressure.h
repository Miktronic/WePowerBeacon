/** @file app_temp_pressure.h
* 
* @brief Temp and Pressure interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl.  All rights reserved.
*/ 

#ifndef APP_TEMP_PRESSURE_H
#define APP_TEMP_PRESSURE_H

#include <stdint.h>

#define TEMP_PRESSURE_ERROR -1
#define TEMP_PRESSURE_SUCCESS 0

typedef struct 
{
    int16_t temp;
    int16_t pressure;
} temp_pressure_data_t;

int app_temp_pressure_service(temp_pressure_data_t *temp_pressure_data);

#endif /* APP_TEMP_PRESSURE_H */

