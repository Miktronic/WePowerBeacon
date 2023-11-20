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

enum POL_ACTIONS {
	NO_POL,		// do not use POL
	OUT_POL,	// raise POL at start and toggle on each beacon
	IN_POL,		// read POL once st start of main() as switch polarity
	CMP_POL,	// capture ACMP0 in EFR32MG24 as switch polarity
	ERR_POL=0xFF
};

typedef struct
{
    uint32_t event_counter; //
    uint32_t serial_number;
    uint8_t  type;
    uint8_t  event_inteval;
    uint16_t event_max_limits;
    uint16_t sleep_min_interval;
    uint16_t sleep_after_wake;
	uint8_t u8_voltsISL9122;
	uint8_t u8_POLmethod; 
	uint8_t cName[10];
} fram_data_t;

int app_fram_service(uint32_t *counter);
int app_fram_read_data(fram_data_t *dat);
int app_fram_write_data(fram_data_t *dat);
int app_fram_read_counter(fram_data_t *dat);
int app_fram_write_counter(fram_data_t *dat);
#endif /* APP_FRAM_H */

