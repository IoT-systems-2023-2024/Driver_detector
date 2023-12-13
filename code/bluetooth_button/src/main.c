/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
/* STEP 3 - Include the header file of the Bluetooth LE stack */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

#include "nrf.h"
#include "src\nrfx\hal\nrf_gpio.h"





LOG_MODULE_REGISTER(Lesson2_Exercise1, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define LED2_PIN 18
#define BUTTON1_PIN 13

typedef struct adv_data {
	uint16_t code; /* 0 = no driver detected; 1 = driver detected; 2 = driver detected + seatbelt on; 3 = driver detected, seatbelt not on */
} adv_data_type;

static struct bt_le_adv_param *adv_param =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_NONE, /* No options specified */
			800, /* Min Advertising Interval 500ms (800*0.625ms) */
			801, /* Max Advertising Interval 500.625ms (801*0.625ms) */
			NULL); /* Set to NULL for undirected advertising */

static adv_data_type adv_data = {0};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	/* STEP 3 - Include the Manufacturer Specific Data in the advertising packet. */
	BT_DATA(BT_DATA_UUID16_ALL, (unsigned char *)&adv_data, sizeof(adv_data)),
};

/* STEP 4.2.1 - Declare the scan response packet */
static struct bt_data sd[] = {
	/* 4.2.3 Include the URL data in the scan response packet */
	BT_DATA(BT_DATA_URI , "test", sizeof("test")),
};

void main(void)
{


	nrf_gpio_cfg_input(BUTTON1_PIN, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_output(LED2_PIN);


	// dk_leds_init();
	
	
	bt_enable(NULL);
	

	bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	while(1) {

		if (nrf_gpio_pin_read(BUTTON1_PIN) == 1) {
			adv_data.code = 1;
			bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
			nrf_gpio_pin_write(LED2_PIN, 1);
		}
		else {
			adv_data.code = 0;
			bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
			nrf_gpio_pin_write(LED2_PIN, 0);
		}


	}
}