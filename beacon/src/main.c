/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <stdlib.h>
#include <math.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/adc.h>

#include "app_types.h"
#include "app_fram.h"
#include "app_encryption.h"
#include "app_accel.h"
#include "app_temp_pressure.h"
#include "zephyr/drivers/gpio.h"
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>


#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define DEVICE_ID 3

#define MANUF_DATA_CUSTOM_START_INDEX   2
#define PAYLOAD_TX_REPEAT_COUNTER_INDEX 18
#define PAYLOAD_DEVICE_ID_INDEX         19
#define PAYLOAD_STATUS_BYTE_INDEX       21
#define PAYLOAD_RFU_BYTE_INDEX          22
#define PAYLOAD_ENCRYPTION_STATUS_ENC   0
#define PAYLOAD_ENCRYPTION_STATUS_CLEAR 1
#define FRAME_LENGTH                    23

#define BLE_ADV_INTERVAL_MIN            (32) // N * 0.625. corresponds to dec. 32; 32*0.625 = 20ms
#define BLE_ADV_INTERVAL_MAX            (36) // N * 0.625. corresponds to dec. 36; 36*0.625 = 22.5ms
#define BLE_ADV_TIMEOUT                 (0)  // N * 10ms for advertiser timeout
#define BLE_ADV_EVENTS                  (1)


#define PACKET_INTERVAL_MIN (10) // ms repeat rate
#define PACKET_INTERVAL_MAX (50) // ms repeat rate

#define LOGS (0) // if printk doesn't handle it use if(LOGS) printk();

#define POL_GPIO_PIN                    13

#define FIELD_NAME_MAX_LENGTH       32
const char FRAM_FIELD_NAMES[MAX_FRAM_FIELDS][FIELD_NAME_MAX_LENGTH] = {
    "EVENT Counter", "SERIAL NUMBER", "TYPE", "EVENT INTERVAL", "EVENT MAXIMUM LIMITS", "EVENT MINIMAM SLEEP",
	"SLEEP AFTER WAKEUP", "VOLT of ISL9122", "POL METHOD", "Device NAME",};

const uint8_t FRAM_FIELD_LENGTH[MAX_FRAM_FIELDS] = {
    (FRAM_COUNTER_NUM_BYTES-1), SER_NUM_BYTES, TYPE_NUM_BYTES, EV_INT_NUM_BYTES, EV_MAX_NUM_BYTES, 
    EV_SLP_NUM_BYTES, IN_SLP_NUM_BYTES, ISL9122_NUM_BYTES, POL_MET_NUM_BYTES, NAME_NUM_BYTES,
};

const uint8_t bIsNumeric[MAX_FRAM_FIELDS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
const uint32_t MinValue[MAX_FRAM_FIELDS] = {0, 0, 0, 20, 1, 0, 0, 0, 0, 0};
const uint32_t MaxValue[MAX_FRAM_FIELDS] = {0xFFFFFF, 0xFFFF, 0xFF, 30, 0xF0, 100, 1000, 1700, 1, 0xFF};

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define UART_MSG_SIZE 256
/* queue to store up to 1 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, UART_MSG_SIZE, 1, 4);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static struct k_work process_command_task;

/* receive buffer used in UART ISR callback */
static uint8_t rx_buf[UART_MSG_SIZE];
static int rx_buf_pos;

static void timer_event_handler(struct k_timer *dummy);
K_TIMER_DEFINE(timer_event, timer_event_handler, NULL);

static struct k_work start_advertising_worker;
static struct k_work_delayable update_frame_work;

static we_power_data_ble_adv_t we_power_data;
static struct bt_data we_power_adv_data;
static struct bt_le_ext_adv *adv;

// variable to store the FRAM data
static fram_data_t fram_data = {0};
//static u16_u8_t device_id;
static uint8_t TX_Repeat_Counter;
static uint8_t TX_Repeat_Counter_Init = 1;

uint32_t serial_number = DEVICE_ID;
uint8_t  type = 1;
uint8_t  event_inteval = 20;
uint16_t event_max_limits = 240;
uint16_t sleep_min_interval = 200;
uint16_t sleep_after_wake = 0; // assumes sleep(0) is just a yield()
uint8_t u8Polarity = 0;

typedef struct
{
    uint8_t type;
    uint8_t field_index;
    uint8_t data_len;
    uint8_t data[(UART_MSG_SIZE - 3)];
}command_data_t;

command_data_t command_data = {0};
//                                                    |--------- ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ENCRYPTED ------------ |  cnt    id    id  status RFU
static uint8_t manf_data[FRAME_LENGTH] = { 0x50, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F};
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manf_data, FRAME_LENGTH),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x50, 0x57)
};

void parse_command(uint8_t *buf)
{
    uint8_t space_idx = 0;
    uint8_t data_idx = 0;

    memset(&command_data, 0x00, sizeof(command_data_t));

    for (uint8_t i=0; buf[i]!='\0'; i++){
        if ((buf[i] == ' ') && (space_idx < 2)){
            space_idx++;
        }
        else{
            if (space_idx == 0){
                command_data.type = buf[i];
            }
            else if (space_idx == 1){
                command_data.field_index = command_data.field_index * 10 + (buf[i] - 48);
            }
            else if(space_idx == 2){
                command_data.data[data_idx] = buf[i];
                data_idx++;
            }
        }
    }

    command_data.data_len = data_idx;
}

int process_command_fn(struct k_work *work)
{
    uint8_t response[256] = {0};
    uint32_t field_data = 0;

    printk("Parsed Command is Type: %c, field: %d, data: %s\n", command_data.type, command_data.field_index, command_data.data);

    if(command_data.field_index >= MAX_FRAM_FIELDS){
        sprintf(response, "FRAM field %d does not exist\n", command_data.field_index);
        printk("%s\n", response);

        return -EINVAL;
    }

    if ((command_data.type == 'G') || (command_data.type == 'g'))
    {
        if ( app_fram_read_field(command_data.field_index, command_data.data) == FRAM_SUCCESS){
            if (bIsNumeric[command_data.field_index] == 0){
                memcpy(&field_data, command_data.data, FRAM_FIELD_LENGTH[command_data.field_index]);
                sprintf(response, "FRAM field %s, length, %d, value is %d.\n", FRAM_FIELD_NAMES[command_data.field_index], FRAM_FIELD_LENGTH[command_data.field_index], field_data);
            }
            else{
                sprintf(response, "FRAM field %s, length, %d, value is %s.\n", FRAM_FIELD_NAMES[command_data.field_index], FRAM_FIELD_LENGTH[command_data.field_index], command_data.data);
            }
        }
        else{
            sprintf(response, "Access to FRAM failed.\n");
        }
    }
    else if ((command_data.type == 'S') || (command_data.type == 's')){
        if (bIsNumeric[command_data.field_index] == 0){
            for (uint8_t i = 0; i < command_data.data_len; i++){
                field_data = field_data * 10 + (command_data.data[i] - 48);
            }

            printk(" Numeric Data: %d\n", field_data);

            if(field_data < MinValue[command_data.field_index]){
                if (app_fram_read_field(command_data.field_index, &field_data) == FRAM_SUCCESS){
                    sprintf(response, "FRAM field %s, Min limit %d, Value is %d.\n", 
                        FRAM_FIELD_NAMES[command_data.field_index], MinValue[command_data.field_index], field_data);
                }else{
                    sprintf(response, "Access to FRAM failed.\n");
                }
            }
            else if (field_data > MaxValue[command_data.field_index]){
                if (app_fram_read_field(command_data.field_index, &field_data) == FRAM_SUCCESS){
                    sprintf(response, "FRAM field %s, Max limit %d exceeded, Value is %d.\n", 
                        FRAM_FIELD_NAMES[command_data.field_index], MaxValue[command_data.field_index], field_data);
                } else {
                    sprintf(response, "Access to FRAM failed.\n");
                }
            }
            else {
                if (app_fram_write_field(command_data.field_index, &field_data) == FRAM_SUCCESS){
                    sprintf(response, "FRAM field %s, Value is %d.\n", FRAM_FIELD_NAMES[command_data.field_index], field_data);
                }
                else {
                    sprintf(response, "Access to FRAM failed.\n");
                }
            }   
        }
        else{
            if (command_data.data_len > 10){
                if (app_fram_read_field(command_data.field_index, &command_data.data) == FRAM_SUCCESS){
                    sprintf(
                        response, 
                        "Type is alphanumeric, but number of characters exceeds maximum length of 10 bytes. FRAM field %s, length exceeded, Value is %s.\n", 
                        FRAM_FIELD_NAMES[command_data.field_index], command_data.data
                    );
                }
                else{
                    sprintf(response, "Access to FRAM failed.\n");
                }
            }
            else{
                if(app_fram_write_field(command_data.field_index, command_data.data) == FRAM_SUCCESS){
                    sprintf(
                        response, 
                        "Type is alphanumeric, number of characters does not exceed maximum length of 10 bytes. Fram field %s, length %d, value is %s.\n",
                        FRAM_FIELD_NAMES[command_data.field_index], FRAM_FIELD_LENGTH[command_data.field_index], command_data.data
                    );
                } else {
                    sprintf(response, "Access to FRAM failed.\n");
                }
            }
        }
    }
    else if ((command_data.type == 'C') || (command_data.type == 'c')){
        if (bIsNumeric[command_data.field_index] == 0){
            field_data = 0;
            if (app_fram_write_field(command_data.field_index, &field_data) == FRAM_SUCCESS){
                sprintf(response, "FRAM field %s, Value is %d.\n", FRAM_FIELD_NAMES[command_data.field_index], field_data);
            }
            else {
                sprintf(response, "Access to FRAM failed.\n");
            }
        }
        else{
            memset(command_data.data, 0x00, (UART_MSG_SIZE - 3));
            if(app_fram_write_field(command_data.field_index, command_data.data) == FRAM_SUCCESS){
                sprintf(response, "FRAM field %s, Value is \"%s\".\n", FRAM_FIELD_NAMES[command_data.field_index], field_data);
            }
            else{
                sprintf(response, "Access to FRAM failed.\n");
            }
        }
    }

    printk("%s\n",response);
    return 0;
}
/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

            if(1){
                printk("--------- UART Received Data -------------\n ");
                for (int i = 0; i < rx_buf_pos; i++){
                    printk("%c", rx_buf[i]);
                }
                printk("\n");
            }

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
            parse_command(rx_buf);
            /* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;

            k_work_submit(&process_command_task);

          
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

static int init_uart(void)
{
    if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

    printk("UART is initiated.\n");

    return 0;
}

static void init_gpio_fn(void)
{
    nrf_gpio_cfg_output(POL_GPIO_PIN);
    nrf_gpio_pin_toggle(POL_GPIO_PIN);
}

SYS_INIT(init_gpio_fn, POST_KERNEL, 0);

static void adv_sent(struct bt_le_ext_adv *instance,
		     struct bt_le_ext_adv_sent_info *info)
{	
    nrf_gpio_pin_toggle(POL_GPIO_PIN);
}

static int create_advertising(void)
{
	int err;
	struct bt_le_adv_param param =
		BT_LE_ADV_PARAM_INIT(
				     BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_USE_NAME,
				     BLE_ADV_INTERVAL_MIN,
				     BLE_ADV_INTERVAL_MAX,
				     NULL);

    static const struct bt_le_ext_adv_cb adv_cb = {
		.sent = adv_sent,
	};

	err = bt_le_ext_adv_create(&param, &adv_cb, &adv);
	if (err) {
		printk("Failed to create advertiser set (%d)\n", err);
		return err;
	}

	printk("Created adv: %p\n", adv);

	return 0;
}

void start_advertising(struct k_work *work)
{
	int err;

     if(LOGS){
        printk("Manufacturer Data: ");
        for (uint8_t i = 0; i < FRAME_LENGTH; i++){
            printk("%02X ", manf_data[i]);
        }
        printk("\n");
    }

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set advertising data (%d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_PARAM(BLE_ADV_TIMEOUT, BLE_ADV_EVENTS));
	if (err) {
		printk("Failed to start advertising set (%d)\n", err);
		return;
	}
}

static void bt_ready(void)
{
	int err = 0;

	printk("Bluetooth initialized\n");

	err = create_advertising();
	if (err) {
		printk("Advertising failed to create (err %d)\n", err);
		return;
	}
    
}

static void timer_event_handler(struct k_timer *dummy)
{
    TX_Repeat_Counter++;
    if (TX_Repeat_Counter <= fram_data.event_max_limits) // starts at 0, we send 1 if max is 1 by incrementing after the test.
    {
        manf_data[PAYLOAD_TX_REPEAT_COUNTER_INDEX] = TX_Repeat_Counter;
        k_work_submit(&start_advertising_worker);
    }
    else {
        printk(">>> Sent maximum packets.\n");
        TX_Repeat_Counter = TX_Repeat_Counter_Init;
        k_timer_stop(&timer_event);
        k_work_schedule(&update_frame_work, K_MSEC(fram_data.sleep_min_interval));
    }
}

static void measure_sensors(we_power_data_ble_adv_t *we_power_data)
{
     // Get the accelerometer data
    static accel_data_t accel_data;
    if(app_accel_service(&accel_data) == ACCEL_SUCCESS)
    {
        // Success!
        we_power_data->data_fields.accel_x.i16 = accel_data.x_accel;
        we_power_data->data_fields.accel_y.i16 = accel_data.y_accel;
        we_power_data->data_fields.accel_z.i16 = accel_data.z_accel;
    }
    else
    {
        we_power_data->data_fields.accel_x.i16 = 0xFFFF;
        we_power_data->data_fields.accel_y.i16 = 0xFFFF;
        we_power_data->data_fields.accel_z.i16 = 0xFFFF;
    }

    // Get temp and pressure
    temp_pressure_data_t temp_pressure_data;
    if(app_temp_pressure_service(&temp_pressure_data) == TEMP_PRESSURE_SUCCESS)
    {
        we_power_data->data_fields.pressure.i16 = temp_pressure_data.pressure;
        we_power_data->data_fields.temp.i16 = temp_pressure_data.temp;
    }
    else
    {
        we_power_data->data_fields.pressure.i16 = 0xFFFF;
        we_power_data->data_fields.temp.i16 = 0xFFFF;
    }

}

void encryptedData(uint8_t* clear_text_buf, uint8_t* encrypted_text_buf, uint8_t len )
{
#define ENCRYPT
#ifdef ENCRYPT
    // Not we want to encrypt the data
    if(app_encrypt_payload(clear_text_buf, len, encrypted_text_buf, len) == ENCRYPTION_ERROR)
    {
        memcpy(encrypted_text_buf, clear_text_buf, 16);
        manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_CLEAR;
    }
    else
    {
        manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_ENC;
    }
#else
    memcpy(cipher_text, encrypted_text_buf, 16);
    manf_data[PAYLOAD_STATUS_BYTE_INDEX] = PAYLOAD_ENCRYPTION_STATUS_CLEAR;
#endif
    if (LOGS){
        printk("Payload - Cleartext: ");
        for(int i = 0; i < len; i++)
        {
            printk("%02X ", clear_text_buf[i]);
        }
        printk("\n");

        printk("Payload - Encrypted: ");
        for(int i = 0; i < len; i++)
        {
            printk("%02X ", encrypted_text_buf[i]);
        }
        printk("\n");
    }
}
// only call this ONCE per EventCounter (FRAM[0:3])
void updateManufacturerData(void){
    printk(">>> Updating the Manufacturer Data\n");
   //Get sensor data
	switch (type)
	{
	default: 
		measure_sensors(&we_power_data);
		break;

	case 3: 
		we_power_data.data_bytes[4] = u8Polarity;
		memcpy(&we_power_data.data_bytes[5], fram_data.cName, 9);
		break;

	case 4: 
		memcpy(&we_power_data.data_bytes[4], fram_data.cName, 10);
		break;

	}

    // increase the FRAM Event counter and set first four bytes
	fram_data.event_counter++;
    app_fram_write_counter(&fram_data);

	we_power_data.data_fields.type = fram_data.type;
	we_power_data.data_fields.event_counter24[0] = (uint8_t)((fram_data.event_counter & 0x000000FF));
	we_power_data.data_fields.event_counter24[1] = (uint8_t)((fram_data.event_counter & 0x0000FF00)>>8);
	we_power_data.data_fields.event_counter24[2] = (uint8_t)((fram_data.event_counter & 0x00FF0000)>>16);
    we_power_data.data_fields.id.u16 = fram_data.serial_number & 0xFFFF;

    // initialize the TX counter

	TX_Repeat_Counter = TX_Repeat_Counter_Init;
	manf_data[PAYLOAD_TX_REPEAT_COUNTER_INDEX] = TX_Repeat_Counter;
   
    // Handle encryption
    static uint8_t cipher_text[DATA_SIZE_BYTES];

    encryptedData(we_power_data.data_bytes, cipher_text, DATA_SIZE_BYTES);
    // Build the BLE adv data
    memcpy(&manf_data[MANUF_DATA_CUSTOM_START_INDEX], cipher_text, DATA_SIZE_BYTES);
    memcpy(&manf_data[PAYLOAD_DEVICE_ID_INDEX], &(fram_data.serial_number), 2);

    we_power_adv_data.type = BT_DATA_SVC_DATA16;
    we_power_adv_data.data = manf_data;
    we_power_adv_data.data_len = sizeof(manf_data);

}

void update_frame_work_fn(struct k_work *work)
{
    updateManufacturerData();
    k_timer_start(&timer_event, K_MSEC(fram_data.event_inteval), K_MSEC(fram_data.event_inteval)); // Start 20ms timer event
}


int32_t read_vcc10(void)
{
    int8_t err;

    uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

    int32_t val_mv;

    if (!device_is_ready(adc_channel.dev)) {
		printk("ADC controller device %s not ready\n", adc_channel.dev->name);
		return -EINVAL;
	}

    err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
		printk("Could not setup channel (%d)\n", err);
		return -EINVAL;
	}

    printk("- %s, channel %d: ",
			       adc_channel.dev->name,
			       adc_channel.channel_id);
    (void)adc_sequence_init_dt(&adc_channel, &sequence);

    err = adc_read(adc_channel.dev, &sequence);
	if (err < 0) {
		printk("Could not read (%d)\n", err);
        return -EINVAL;
	}

    val_mv = (int32_t)buf;

    printk("%"PRId32, val_mv);

    err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);

	if (err < 0) {
		printk(" (value in mV not available)\n");
        return -EINVAL;
	} else {
		printk(" = %"PRId32" mV\n", val_mv);
	}
    return val_mv;

}
// Main Application
void main(void)
{
    int err;
    int32_t vext10_mv;

    // read VEXT10 ADC, decide to run this or the configuration app.
    vext10_mv = read_vcc10() * 11;
    nrf_gpio_pin_toggle(POL_GPIO_PIN);
	// if VEXT10 > 0.165V we have external power
    if (vext10_mv > 1700){
        //Init UART
        init_uart();
        //Process the UART Command
        k_work_init(&process_command_task, process_command_fn);
    }
    else{
        // Reading FRAM.
        if(app_fram_read_data(&fram_data) == FRAM_SUCCESS)
        {
            // Success! Add it to the payload

            if (0){
                printk(">> ------- FRAM Data -------\n");
		        printk(">>[FRAM INFO]->Event Counter: 0x%08X\n", fram_data.event_counter);
		        printk(">>[FRAM INFO]->Serial Number: 0x%08X\n", fram_data.serial_number);
		        printk(">>[FRAM INFO]->Device Type: %d\n", fram_data.type);
		        printk(">>[FRAM INFO]->Frame Interval: %d ms\n", fram_data.event_inteval);
		        printk(">>[FRAM INFO]->Frame Maximum Number: %d\n", fram_data.event_max_limits);
		        printk(">>[FRAM INFO]->Minimum Sleeping Interval: %d\n", fram_data.sleep_min_interval);
		        printk(">>[FRAM INFO]->Sleep time After Wake up: %d\n", fram_data.sleep_after_wake);
		        printk(">>[FRAM INFO]->Voltage of ISL9122: %d\n", fram_data.u8_voltsISL9122);
		        printk(">>[FRAM INFO]->POL Method: %d\n", fram_data.u8_POLmethod);
		        printk(">>[FRAM INFO]->cName: %s\n", fram_data.cName);
            }
		
            serial_number = (fram_data.serial_number?(fram_data.serial_number<0xFFFF? fram_data.serial_number:DEVICE_ID):DEVICE_ID);
		    type = (fram_data.type ? (fram_data.type < 0xFF ? fram_data.type : 1) : 1);
		    event_inteval = (fram_data.event_inteval >= PACKET_INTERVAL_MIN ? (fram_data.event_inteval <= PACKET_INTERVAL_MAX ? fram_data.event_inteval : PACKET_INTERVAL_MAX) : PACKET_INTERVAL_MIN); //
		    event_max_limits = (fram_data.event_max_limits >= 3 ? (fram_data.event_max_limits < 0xF0 ? fram_data.event_max_limits : 3) : 0xF0);
		    sleep_min_interval = (fram_data.sleep_min_interval >= 50 ? (fram_data.sleep_min_interval < 0xFFFF ? fram_data.sleep_min_interval : 0xFFF0) : 50);
		    sleep_after_wake = (fram_data.sleep_after_wake ? (fram_data.sleep_after_wake < 0xFFFF ? fram_data.sleep_after_wake : 20) : 0);
        
            fram_data.event_inteval=20;
            fram_data.event_max_limits = 0xF0;
            fram_data.serial_number = DEVICE_ID;
            fram_data.sleep_after_wake = 0;
            fram_data.sleep_min_interval = 50;
            fram_data.u8_POLmethod = OUT_POL;
            fram_data.u8_voltsISL9122 = 0;
        
            if (fram_data.u8_voltsISL9122 >= 72 && fram_data.u8_voltsISL9122 <= 132)
		    {// // write fram_data.u8_voltsISL9122 to addr 0x18, reg 0x11
		    };
			
		    if (fram_data.u8_POLmethod == OUT_POL) {// toggle output POL_GPIO_PIN
    			nrf_gpio_cfg_output(POL_GPIO_PIN);
			    nrf_gpio_pin_toggle(POL_GPIO_PIN);
		    }
		    else if (fram_data.u8_POLmethod == IN_POL) {// sleep sleep_after_wake then read GPIO --> u8Polarity
                // sleep sleep_after_wake
                k_sleep(K_MSEC(fram_data.sleep_after_wake));
			    nrf_gpio_cfg_input(POL_GPIO_PIN, NRF_GPIO_PIN_PULLUP);
			    u8Polarity = nrf_gpio_pin_read(POL_GPIO_PIN);
		    }
		    //else if (fram_data.u8_POLmethod == CMP_POL) {/* not possible on nRF design.  sleep sleep_after_wake then read ACMP0 on EFR32*/};
        }
        else
        {
		     fram_data.event_counter = 0;
        }
    
        // Init and run the BLE
	    err = bt_enable(NULL);
	    if (err) {
		    printk("Bluetooth init failed (err %d)\n", err);
		    return;
	    }
	
	    bt_ready();

        nrf_gpio_pin_toggle(POL_GPIO_PIN);

        updateManufacturerData();
        TX_Repeat_Counter_Init = 0;

        nrf_gpio_pin_toggle(POL_GPIO_PIN);

        // Init works for advertising and measuring data
        k_work_init(&start_advertising_worker, start_advertising);
        k_work_init_delayable(&update_frame_work, update_frame_work_fn);
        k_timer_start(&timer_event, K_MSEC(fram_data.event_inteval), K_MSEC(fram_data.event_inteval)); // Start 20ms timer event
        k_work_submit(&start_advertising_worker);// submit the first packet
    }
}
