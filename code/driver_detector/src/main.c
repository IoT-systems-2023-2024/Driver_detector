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

#include "nrf.h"
#include "src\nrfx\hal\nrf_gpio.h"

#include <dk_buttons_and_leds.h>

#include <src\drivers\chirpmicro\inc\soniclib.h>
#include <src\application\smartsonic-hellochirp-example\inc\app_config.h>

LOG_MODULE_REGISTER(Lesson2_Exercise1, LOG_LEVEL_INF);

/* Buzzer en seatbelt dinges */

#define BUZZER_PIN 6
#define SEATBELT_EN_PIN 14
#define SEATBELT_OUT_PIN 15

/* Einde Buzzer en seatbelt*/

/* CH101 DINGES*/
/* Bit flags used in main loop to check for completion of sensor I/O.  */
#define DATA_READY_FLAG		(1 << 0)
#define IQ_READY_FLAG		(1 << 1)

#define THRESHOLD_DRIVER_MM 	(500)	/* Minder dan 500mm (0.5m) --> bestuurder aanwezig*/

/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from 
 *   a sensor.  The data values include the measured range, the ultrasonic 
 *   signal amplitude, the number of valid samples (I/Q data pairs) in the 
 *   measurement, and (optionally) the full amplitude data and/or raw I/Q data 
 *   from the measurement.
 *
 *  The format of this data structure is specific to this application, so 
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor, 
 *  is declared in the main.c file.  The sensor's device number is 
 *  used to index the array.
 */
typedef struct {
	uint32_t		range;						// from ch_get_range()
	uint16_t		amplitude;					// from ch_get_amplitude()
	uint16_t		num_samples;				// from ch_get_num_samples()
#ifdef READ_AMPLITUDE_DATA
	uint16_t		amp_data[DATA_MAX_NUM_SAMPLES];	
												// from ch_get_amplitude_data()
#endif
#ifdef READ_IQ_DATA
	ch_iq_sample_t	iq_data[DATA_MAX_NUM_SAMPLES];	
												// from ch_get_iq_data()
#endif
} chirp_data_t;


/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t	chirp_data[CHIRP_MAX_NUM_SENSORS];		// MAX_NUM_SENSORS defined in app_config.h al instellen op 1?

/* Array of ch_dev_t device descriptors, one for each possible device */
ch_dev_t	chirp_devices[CHIRP_MAX_NUM_SENSORS];		

/* Configuration structure for group of sensors */
ch_group_t 	chirp_group;

/* Task flag word
 *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags 
 *   that are set in I/O processing routines.  The flags are checked in the 
 *   main() loop and, if set, will cause an appropriate handler function to 
 *   be called to process sensor data.  
 */
volatile uint32_t taskflags = 0;

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;

/* Number of connected sensors */
static uint8_t	num_connected_sensors = 0;

/* Number of sensors that use h/w triggering to start measurement */
static uint8_t	num_triggered_devices = 0;

#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
/* Count of non-blocking I/Q reads queued */
static uint8_t	num_io_queued = 0;
#endif


/* Forward declarations */
static void    sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num);
static void    io_complete_callback(ch_group_t *grp_ptr);
static uint8_t handle_data_ready(ch_group_t *grp_ptr);

#ifdef READ_IQ_DATA
static uint8_t display_iq_data(ch_dev_t *dev_ptr);
#ifdef READ_IQ_NONBLOCKING
static uint8_t handle_iq_data_done(ch_group_t *grp_ptr);
#endif
#endif


/* EINDE CH101 DINGES*/

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

	/* CH101 DINGES */

	ch_group_t	*grp_ptr = &chirp_group;
	uint8_t chirp_error = 0;
	uint8_t num_ports;
	uint8_t dev_num;

	chbsp_board_init(grp_ptr);

	/* Get the number of (possible) sensor devices on the board
	 *   Set by the BSP during chbsp_board_init() 
	 */
	num_ports = ch_get_num_ports(grp_ptr);

	/* Initialize sensor descriptors.
	 *   This loop initializes each (possible) sensor's ch_dev_t descriptor, 
	 *   although we don't yet know if a sensor is actually connected.
	 *
	 *   The call to ch_init() specifies the sensor descriptor, the sensor group
	 *   it will be added to, the device number within the group, and the sensor
	 *   firmware initialization routine that will be used.  (The sensor 
	 *   firmware selection effectively specifies whether it is a CH101 or 
	 *   CH201 sensor, as well as the exact feature set.)
	 */
	printf("Initializing sensor(s)... ");

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);	// init struct in array

		/* Init device descriptor 
		 *   Note that this assumes all sensors will use the same sensor 
		 *   firmware.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is defined in 
		 *   app_config.h and is used for all devices.
		 *
		 *   However, it is possible for different sensors to use different 
		 *   firmware images, by specifying different firmware init routines 
		 *   when ch_init() is called for each.
		 */
		chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, 
							   CHIRP_SENSOR_FW_INIT_FUNC);
	}

	/* Start all sensors.
	 *   The ch_group_start() function will search each port (that was 
	 *   initialized above) for a sensor. If it finds one, it programs it (with
	 *   the firmware specified above during ch_init()) and waits for it to 
	 *   perform a self-calibration step.  Then, once it has found all the 
	 *   sensors, ch_group_start() completes a timing reference calibration by 
	 *   applying a pulse of known length to the sensor's INT line.
	 */
	if (chirp_error == 0) {
		printf("starting group... ");
		chirp_error = ch_group_start(grp_ptr);
	}

	if (chirp_error == 0) {
		printf("OK\n");
	} else {
		printf("FAILED: %d\n", chirp_error);
	}
	printf("\n");

	/* Get and display the initialization results for each connected sensor.
	 *   This loop checks each device number in the sensor group to determine 
	 *   if a sensor is actually connected.  If so, it makes a series of 
	 *   function calls to get different operating values, including the 
	 *   operating frequency, clock calibration values, and firmware version.
 	 */
	printf("Sensor\tType \t   Freq\t\t RTC Cal \tFirmware\n");

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			printf("%d\tCH%d\t %u Hz\t%u@%ums\t%s\n", dev_num,
									ch_get_part_number(dev_ptr),
									(unsigned int) ch_get_frequency(dev_ptr),
									ch_get_rtc_cal_result(dev_ptr),
									ch_get_rtc_cal_pulselength(dev_ptr),
									ch_get_fw_version_string(dev_ptr));
		}
	}
	printf("\n");

	/* Register callback function to be called when Chirp sensor interrupts */
	ch_io_int_callback_set(grp_ptr, sensor_int_callback);

	/* Register callback function called when non-blocking I/Q readout completes
 	 *   Note, this callback will only be used if READ_IQ_DATA_NONBLOCK is 
	 *   defined to enable non-blocking I/Q readout in this application.
	 */
	ch_io_complete_callback_set(grp_ptr, io_complete_callback);

	/* Configure each sensor with its operating parameters 
	 *   Initialize a ch_config_t structure with values defined in the
	 *   app_config.h header file, then write the configuration to the 
	 *   sensor using ch_set_config().
	 */
	printf ("Configuring sensor(s)...\n");
	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_config_t dev_config;
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			/* Select sensor mode 
			 *   All connected sensors are placed in hardware triggered mode.
 	 		 *   The first connected (lowest numbered) sensor will transmit and 
			 *   receive, all others will only receive.
 	 		 */

			num_connected_sensors++;			// count one more connected
			active_devices |= (1 << dev_num);	// add to active device bit mask
			
			if (num_connected_sensors == 1) {	// if this is the first sensor
				dev_config.mode = CHIRP_FIRST_SENSOR_MODE;
			} else {									
				dev_config.mode = CHIRP_OTHER_SENSOR_MODE;
			}

			if (dev_config.mode != CH_MODE_FREERUN) {	// unless free-running
				num_triggered_devices++;				// will be triggered
			}

			/* Init config structure with values from app_config.h */
			dev_config.max_range       = CHIRP_SENSOR_MAX_RANGE_MM;
			dev_config.static_range    = CHIRP_SENSOR_STATIC_RANGE;

			/* If sensor will be free-running, set internal sample interval */
			if (dev_config.mode == CH_MODE_FREERUN) {
				dev_config.sample_interval = MEASUREMENT_INTERVAL_MS;
			} else {
				dev_config.sample_interval = 0;
			}

			/* Apply sensor configuration */
			chirp_error = ch_set_config(dev_ptr, &dev_config);

			/* Enable sensor interrupt if using free-running mode 
			 *   Note that interrupt is automatically enabled if using 
			 *   triggered modes.
			 */
			if ((!chirp_error) && (dev_config.mode == CH_MODE_FREERUN)) {
				chbsp_set_io_dir_in(dev_ptr);
				chbsp_io_interrupt_enable(dev_ptr);
			}

			/* Read back and display config settings */
			if (!chirp_error) {
				ultrasound_display_config_info(dev_ptr);
			} else {
				printf("Device %d: Error during ch_set_config()\n", dev_num);
			}

			/* Turn on an LED to indicate device connected */
			if (!chirp_error) {
				chbsp_led_on(dev_num);
			}
		}
	}

	printf("\n");

	/* Enable receive sensor pre-triggering, if specified */
	ch_set_rx_pretrigger(grp_ptr, RX_PRETRIGGER_ENABLE);

	/* Initialize the periodic timer we'll use to trigger the measurements.
 	 *   This function initializes a timer that will interrupt every time it 
	 *   expires, after the specified measurement interval.  The function also 
	 *   registers a callback function that will be called from the timer 
	 *   handler when the interrupt occurs.  The callback function will be 
	 *   used to trigger a measurement cycle on the group of sensors.
	 */
	if (num_triggered_devices > 0) {
		printf("Initializing sample timer for %dms interval... ", 
				MEASUREMENT_INTERVAL_MS);

		chbsp_periodic_timer_init(MEASUREMENT_INTERVAL_MS, 
							      periodic_timer_callback);

		/* Enable interrupt and start timer to trigger sensor sampling */
		chbsp_periodic_timer_irq_enable();
		chbsp_periodic_timer_start();

		printf("OK\n");
	}


	/* EINDE CH101 DINGES*/

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
		/*
		 * Put processor in light sleep mode if there are no pending tasks, but 
		 * never turn off the main clock, so that interrupts can still wake 
		 * the processor.
		 */
		if (taskflags==0) {
			chbsp_proc_sleep();			// put processor in low-power sleep mode

			/* We only continue here after an interrupt wakes the processor */
		}

		/* Check for sensor data-ready interrupt(s) */
		if (taskflags & DATA_READY_FLAG) {

			/* All sensors have interrupted - handle sensor data */
			taskflags &= ~DATA_READY_FLAG;		// clear flag
			handle_data_ready(grp_ptr);			// read and display measurement

			if (chirp_data[0].range < THRESHOLD_DRIVER_MM) {
				printf("Driver present\n");  /* Nog kunnen toevoegen aan sd van ble samen met seatbelt meting*/
				if(nrf_gpio_pin_read(SEATBELT_OUT_PIN) == 1){
					sd[0].data = "Driver present, seatbelt on";
					nrf_gpio_pin_write(BUZZER_PIN, 0);
				}
				else{
					sd[0].data = "Driver present, seatbelt off";
					nrf_gpio_pin_write(BUZZER_PIN, 1);
				}
				sd[0].data = "Driver present";
			} else {
				printf("Driver not present\n");
				sd[0].data = "Driver not present";
			}
		k_sleep(K_MSEC(SEND_INTERVAL));  /* Een ander interval kiezen (+ alles van die LEDs uit de code halen)*/
		}

#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
		/* Start any pending non-blocking I/Q reads */
		if (num_io_queued != 0) {
			ch_io_start_nb(grp_ptr);
			num_io_queued = 0;
		}

		/* Check for non-blocking I/Q read complete */
		if (taskflags & IQ_READY_FLAG) {

			/* All non-blocking I/Q readouts have completed */
			taskflags &= ~IQ_READY_FLAG;		// clear flag
			handle_iq_data_done(grp_ptr);		// display I/Q data
		}
#endif

	}	// end  while(1) main loop

	// for (;;) {
	// 	dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
	// 	k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	// }
}

/* CH101 DINGES */

/*
 * periodic_timer_callback() - periodic timer callback routine
 *
 * This function is called by the periodic timer interrupt when the timer 
 * expires.  Because the periodic timer is used to initiate a new measurement 
 * cycle on a group of sensors, this function calls ch_group_trigger() during 
 * each execution.
 *
 * This function is registered by the call to chbsp_periodic_timer_init() 
 * in main().
 */

void periodic_timer_callback(void) {

	if (num_triggered_devices > 0) {
		ch_group_trigger(&chirp_group);
	}
}


/*
 * sensor_int_callback() - sensor interrupt callback routine
 *
 * This function is called by the board support package's interrupt handler for 
 * the sensor's INT line every time that the sensor interrupts.  The device 
 * number parameter, dev_num, is used to identify the interrupting device
 * within the sensor group.  (Generally the device number is same as the port 
 * number used in the BSP to manage I/O pins, etc.)
 *
 * Each time this function is called, a bit is set in the data_ready_devices
 * variable to identify the interrupting device.  When all active sensors have
 * interrupted (found by comparing with the active_devices variable), the
 * DATA_READY_FLAG is set.  That flag will be detected in the main() loop.
 *
 * This callback function is registered by the call to ch_io_int_callback_set() 
 * in main().
 */
static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num) {
	ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

	data_ready_devices |= (1 << dev_num);		// add to data-ready bit mask

	if (data_ready_devices == active_devices) {
		/* All active sensors have interrupted after performing a measurement */
		data_ready_devices = 0;

		/* Set data-ready flag - it will be checked in main() loop */
		taskflags |= DATA_READY_FLAG;

		/* Disable interrupt unless in free-running mode
		 *   It will automatically be re-enabled by the next ch_group_trigger() 
		 */
		if (ch_get_mode(dev_ptr) == CH_MODE_FREERUN) {
			chbsp_set_io_dir_in(dev_ptr);				// set INT line as input
			chbsp_group_io_interrupt_enable(grp_ptr);
		} else {
			chbsp_group_io_interrupt_disable(grp_ptr);
		}
	}
}


/*
 * io_complete_callback() - non-blocking I/O complete callback routine
 *
 * This function is called by SonicLib's I2C DMA handling function when all 
 * outstanding non-blocking I/Q readouts have completed.  It simply sets a flag 
 * that will be detected and handled in the main() loop.
 *
 * This callback function is registered by the call to 
 * ch_io_complete_callback_set() in main().
 *
 *  Note: This callback is only used if READ_IQ_NONBLOCKING is defined to 
 *  select non-blocking I/Q readout in this application.
 */
static void io_complete_callback(ch_group_t __attribute__((unused)) *grp_ptr) {

	taskflags |= IQ_READY_FLAG;
}

/*
 * handle_data_ready() - get and display data from all sensors  
 *
 * This routine is called from the main() loop after all sensors have 
 * interrupted. It shows how to read the sensor data once a measurement is 
 * complete.  This routine always reads out the range and amplitude, and 
 * optionally will read out the amplitude data or raw I/Q for all samples
 * in the measurement.
 *
 * See the comments in app_config.h for information about the amplitude data
 * and I/Q readout build options.
 *
 */
static uint8_t handle_data_ready(ch_group_t *grp_ptr) {
	uint8_t 	dev_num;
	int 		num_samples = 0;
	uint8_t 	ret_val = 0;

	/* Read and display data from each connected sensor 
	 *   This loop will write the sensor data to this application's "chirp_data"
	 *   array.  Each sensor has a separate chirp_data_t structure in that 
	 *   array, so the device number is used as an index.
	 */

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			/* Get measurement results from each connected sensor 
			 *   For sensor in transmit/receive mode, report one-way echo 
			 *   distance,  For sensor(s) in receive-only mode, report direct 
			 *   one-way distance from transmitting sensor 
			 */
			
			if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
				chirp_data[dev_num].range = ch_get_range(dev_ptr, 
														CH_RANGE_DIRECT);
			} else {
				chirp_data[dev_num].range = ch_get_range(dev_ptr, 
														CH_RANGE_ECHO_ONE_WAY);
			}

			if (chirp_data[dev_num].range == CH_NO_TARGET) {
				/* No target object was detected - no range value */

				chirp_data[dev_num].amplitude = 0;  /* no updated amplitude */

				printf("Port %d:          no target found        ", dev_num);

			} else {
				/* Target object was successfully detected (range available) */

				 /* Get the new amplitude value - it's only updated if range 
				  * was successfully measured.  */
				chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);

				printf("Port %d:  Range: %0.1f mm  Amp: %u  ", dev_num, 
						(float) chirp_data[dev_num].range/32.0f,
					   	chirp_data[dev_num].amplitude);
			}

			/* Store number of active samples in this measurement */
			num_samples = ch_get_num_samples(dev_ptr);
			chirp_data[dev_num].num_samples = num_samples;

			/* Optionally read amplitude values for all samples */
#ifdef READ_AMPLITUDE_DATA
			uint16_t 	start_sample = 0;
			ch_get_amplitude_data(dev_ptr, chirp_data[dev_num].amp_data, 
								  start_sample, num_samples, CH_IO_MODE_BLOCK);

#ifdef OUTPUT_AMPLITUDE_DATA
			printf("\n");
			for (uint8_t count = 0; count < num_samples; count++) {

				printf("%d\n",  chirp_data[dev_num].amp_data[count]);
			}
#endif
#endif

			/* Optionally read raw I/Q values for all samples */
#ifdef READ_IQ_DATA
			display_iq_data(dev_ptr);
#endif
			/* If more than 2 sensors, put each on its own line */
			if (num_connected_sensors > 2) {
				printf("\n");
			}
		}
	}
	printf("\n");
	
	return ret_val;
}

/* EINDE CH101 DINGES */
