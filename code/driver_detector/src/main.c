/*
 * NRF code uit https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-2-bluetooth-le-advertising/topic/blefund-lesson-2-exercise-1/
 * CH101 code uit https://invensense.tdk.com/wp-content/uploads/2020/08/AN-000175-SonicLib-Programmers-Guide-v1.0.pdf
 *  
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
/* STEP 3 - Include the header file of the Bluetooth LE stack */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

// #include <build\app\libapp.a>
// #include <build\zephyr\libzephyr.a>

#include "nrf.h"
#include "src\nrfx\hal\nrf_gpio.h"

#include <dk_buttons_and_leds.h>

#include <stdio.h>
// #include <src\drivers\chirpmicro\inc\soniclib.h>		// Chirp SonicLib sensor API definitions
// #include "src\board\config\chirp_board_config.h"	// required header with basic device counts etc.
// #include "src\application\smartsonic-hellochirp-example\inc\app_config.h"
// #include "src\application\smartsonic-hellochirp-example\inc\app_version.h"
// #include "src\drivers\chirpmicro\inc\chirp_bsp.h"			// board support package function definitions
// #include "src\board\config\chirp_smartsonic.h"
// #include "src\ultrasound\inc\ultrasound_display_config_info.h"

#include "src\nrfx\hal\nrf_twi.h"
#include "src\drivers_nrf\twi_master\nrf_drv_twi.h"
#include "src\libraries\util\app_error.h"
#include <zephyr/drivers/i2c.h>  //alles met deze libraries handmatig doen


// met board button 

/* CH101 */

#define TWI_INSTANCE_ID     0
#define TWI0_INSTANCE_INDEX 0

#define CONCAT_2(a, b)      a ## b
#define CONCAT_3(a, b, c)   a ## b ## c

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Chirp 101 I2C address
#define CHIRP_ADDR          0x45

// GPIO pin configuration for Chirp 101
#define CHIRP_RESET_PIN     21  // is not-reset
#define CHIRP_INTERRUPT_PIN 7

// Function to initialize GPIO pins for Chirp 101
void gpio_init(void)
{
    nrf_gpio_cfg_output(CHIRP_RESET_PIN);
    nrf_gpio_cfg_input(CHIRP_INTERRUPT_PIN, NRF_GPIO_PIN_NOPULL);
}


// Function to initialize the TWI peripheral
void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 25,
       .sda                = 26,
       .frequency          = NRF_TWI_FREQ_100K,  // 100 kHz I2C frequency?
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

// Function to write a byte to the Chirp 101 sensor
void chirp_write_byte(uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;

    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, CHIRP_ADDR, data, sizeof(data), false);
    APP_ERROR_CHECK(err_code);
}

// Function to read a byte from the Chirp 101 sensor
uint8_t chirp_read_byte(uint8_t reg)
{
    ret_code_t err_code;
    uint8_t data;

    err_code = nrf_drv_twi_tx(&m_twi, CHIRP_ADDR, &reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_tx(&m_twi, CHIRP_ADDR, data, sizeof(data), false);
    APP_ERROR_CHECK(err_code);

    return data;
}

// Function to read distance from the Chirp 101 sensor
uint16_t chirp_read_distance(void)
{
    // Assuming that the Chirp 101 sensor stores distance in a 16-bit register (example: register 0x01)
    uint8_t reg = 0x01;
    ret_code_t err_code;
    uint8_t data[2];

    // Send command to read distance data
    err_code = nrf_drv_twi_tx(&m_twi, CHIRP_ADDR, &reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);

    // Read distance data
    err_code = nrf_drv_twi_rx(&m_twi, CHIRP_ADDR, data, sizeof(data), false);

    APP_ERROR_CHECK(err_code);

    // Combine two bytes into a 16-bit distance value
    uint16_t distance = (data[1] << 8) | data[0];

    return distance;
}


/* EINDE CH101 */

LOG_MODULE_REGISTER(Lesson2_Exercise1, LOG_LEVEL_INF);

/* Buzzer en seatbelt dinges */

#define BUZZER_PIN 6
#define SEATBELT_EN_PIN 14
#define SEATBELT_OUT_PIN 15

/* Einde Buzzer en seatbelt*/



/* BLUETOOTH DINGES */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define SEND_INTERVAL 1000  // --> in ms, dus nu 1s, nog verhogen


/* STEP 4.1.1 - Declare the advertising packet */
static const struct bt_data ad[] = {
	/* STEP 4.1.2 - Set the advertising flags */
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	/* STEP 4.1.3 - Set the advertising packet data  --> Hier iets aan toevoegen om andere data te vestruren*/
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

/* Moet er een scan response geschreven worden??*/

/* STEP 4.2.1 - Declare the scan response packet */
static struct bt_data sd[] = {
	/* 4.2.3 Include the URL data in the scan response packet --> Hier iets anders van maken om andere data te vesturen*/ 
	BT_DATA(BT_DATA_INDOOR_POS, "Driver present", sizeof("Driver present")),
};
/* EINDE BLUETOOTH DINGES*/

/* CH101 uitlezen, als persoon aanwezig EN=1 (als die actief-hoog is),
(Als er geen stroom door sleutelcontact komt, is OUT=0 (?), moet zelfde resultaat geven als gordel die aan is)
Als OUt=1 is de seatbelt aan, dan gewoon via bluetooth sturen als dat bestuurder aanwezig is
Als OUT=0 is de seatbelt niet aan, dan buzzer=1 en bluetooth sturen dat seatbelt niet aan is
*/

void main(void)
{
	// int blink_status = 0;
	int err;

	// LOG_INF("Starting Lesson 2 - Exercise 1 \n");

	// err = dk_leds_init();
	// if (err) {
	// 	LOG_ERR("LEDs init failed (err %d)\n", err);
	// 	return;
	// }

	/* BUZZER EN SEATBELT DINGES*/

	nrf_gpio_cfg_output(BUZZER_PIN);
	nrf_gpio_cfg_output(SEATBELT_EN_PIN);
	nrf_gpio_cfg_input(SEATBELT_OUT_PIN, NRF_GPIO_PIN_PULLUP);

	

	/* EINDE BUZZER EN SEATBELT DINGES*/

	/* CH101 */

	// Initialize GPIO for Chirp 101
    gpio_init();

    // Initialize TWI peripheral
    twi_init();

    // Reset Chirp 101 sensor
    nrf_gpio_pin_set(CHIRP_RESET_PIN);
	k_msleep(10);
    // nrf_delay_ms(10);
    nrf_gpio_pin_clear(CHIRP_RESET_PIN);
	k_msleep(10);
    // nrf_delay_ms(10);

    // Initialize Chirp 101 sensor
    chirp_write_byte(0x80, 0x03); // Software reset
	k_msleep(10);
    // nrf_delay_ms(10); // Wait for reset to complete



	/* EINDE CH101 */

	

	/* BLUETOOTH DINGES */
	/* STEP 5 - Enable the Bluetooth LE stack */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return;
	}

	LOG_INF("Bluetooth initialized\n");

	/* STEP 6 - Start advertising --> zien of sd hier eerst aangepast kan worden met huidige waarden + Blijft hij advertisen of moet dit in de while-loop opgeroepen worden*/
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_INF("Advertising successfully started\n");

	/* EINDE BLUETOOTH DINGES*/

	/* MAIN LOOP*/
	while (1) {		/* LOOP FOREVER */

		// /* Check for sensor data-ready interrupt(s) */
		// if (taskflags & DATA_READY_FLAG) {

		// 	/* All sensors have interrupted - handle sensor data */
		// 	taskflags &= ~DATA_READY_FLAG;		// clear flag
		// 	handle_data_ready(grp_ptr);			// read and display measurement

		// 	if (chirp_data[0].range < THRESHOLD_DRIVER_MM) {
		// 		printf("Driver present\n");  /* Nog kunnen toevoegen aan sd van ble samen met seatbelt meting*/
		// 		if(nrf_gpio_pin_read(SEATBELT_OUT_PIN) == 1){
		// 			sd[0].data = "Driver present, seatbelt on";
		// 			nrf_gpio_pin_write(BUZZER_PIN, 0);
		// 		}
		// 		else{
		// 			sd[0].data = "Driver present, seatbelt off";
		// 			nrf_gpio_pin_write(BUZZER_PIN, 1);
		// 		}
		// 		sd[0].data = "Driver present";
		// 	} else {
		// 		printf("Driver not present\n");
		// 		sd[0].data = "Driver not present";
		// 	}

		uint16_t distance = chirp_read_distance();
		printf("Distance: %d mm\n", distance); //eens zien in welk formaat distance is
		k_sleep(K_MSEC(SEND_INTERVAL));  /* Een ander interval kiezen (+ alles van die LEDs uit de code halen)*/
		}


}

