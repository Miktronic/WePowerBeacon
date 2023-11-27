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

#define FRAM_COUNTER_ADDR 			0x0000
#define FRAM_COUNTER_NUM_BYTES 		4
#define SER_NUM_ADDR 				0x0004
#define SER_NUM_BYTES 				4
#define TYPE_ADDR					0x0008
#define TYPE_NUM_BYTES 				1
#define EV_INT_ADDR					0x0009
#define EV_INT_NUM_BYTES			1
#define EV_MAX_ADDR					0x000a
#define EV_MAX_NUM_BYTES			2
#define EV_SLP_ADDR					0x000c
#define EV_SLP_NUM_BYTES 			2
#define IN_SLP_ADDR					0x000e
#define IN_SLP_NUM_BYTES			2
#define ISL9122_ADDR				0x0010
#define ISL9122_NUM_BYTES			1
#define POL_MET_ADDR				0x0011
#define POL_MET_NUM_BYTES			1
#define ENCRYPTED_KEY_ADDR			0x0012
#define ENCRYPTED_KEY_NUM_BYTES		16
#define TX_DBM_ADDR					0x0022
#define TX_DBM_NUM_BYTES			1
#define NAME_ADDR					0x0023
#define NAME_NUM_BYTES				10

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
	uint8_t encrypted_key[16];
	uint8_t tx_dbm_10;
	uint8_t cName[10];
} fram_data_t;

enum FRAM_FIELDS {
       EV_CTR = 0,
       SER_NUM,
       TYPE,
       EV_INT,
       EV_MAX,
       EV_SLP,
       IN_SLP,
       ISL9122,
       POL_MET,
	   ENCRYPTED_KEY,
	   TX_DBM,
	   NAME,
       MAX_FRAM_FIELDS
};

int app_fram_read_field(uint8_t field, uint8_t *dat);
int app_fram_write_field(uint8_t field, uint8_t *dat);
int app_fram_service(uint32_t *counter);
int app_fram_read_data(fram_data_t *dat);
int app_fram_write_data(fram_data_t *dat);
int app_fram_read_counter(fram_data_t *dat);
int app_fram_write_counter(fram_data_t *dat);
#endif /* APP_FRAM_H */

