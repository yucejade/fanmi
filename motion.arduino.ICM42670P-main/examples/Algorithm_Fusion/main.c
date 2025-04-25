/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/* std */
#include <stdio.h>
#include "example_algo.h"
#include "ErrorHelper.h"

/*
 * Global variables
 */

/* Flag set from IMU interrupt handler */
static volatile int irq_from_device;

#if USE_MAG
/* Flag set when mag data is ready */
static volatile int irq_from_timer_mag_data_ready = 0;

/* Variable used to keep track of channel used for mag data ready */
static int mag_timer_ch = -1;

/* Mag timestamp filled up in interrupt handler */
volatile uint64_t timestamp_mag;

/* Indicates if mag was successfully initialized */
int mag_init_successful = 0;
#endif

/*
 * Static functions
 */

static void check_rc(int rc, const char *msg_context);

/*
 * Main
 */

int main(void)
{
	int                      rc = 0;
	struct inv_imu_serif     icm_serif;
#if USE_MAG
	uint32_t                 imu_sample_count = 0;
#endif

#if USE_MAG
	/* Initialize magnetometer */
	rc = setup_mag_device(&ist8306_serif);
	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_WARNING,
		        "Error while setting up Mag device: not support of mag-related sensors");
		mag_init_successful = 0;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "Mag successfully initialized");
		mag_init_successful = 1;
	}
#endif

	rc |= setup_imu_device(&icm_serif);
	rc |= init_agm_biases();
	rc |= init_agm_algo();
	rc |= configure_imu_device();
	check_rc(rc, "error during initialization");
	printf("IMU successfully initialized\n");

	printf("Start processing\n");

#if USE_MAG
		} else if (mag_init_successful && irq_from_timer_mag_data_ready) {
			uint8_t mag_drdy;
			/* 
			 * Check AKM interrupt
			 * Use `else if` instead of `if` to give priority to IMU's interrupt. 
			 * In case the processing of the IMU data takes longer than 1 ODR, both IMU and AKM 
			 * interrupt flag might be set. Since IMU's ODR is faster or equal to AKM's ODR, 
			 * let's process IMU first by restarting the loop. AKM will be processed in the 
			 * next iteration.
			 */

			rc = get_mag_drdy(&mag_drdy);
			check_rc(rc, "error while reading mag status");

			if (mag_drdy) {
				inv_disable_irq();
				irq_from_timer_mag_data_ready = 0;
				inv_enable_irq();

				rc = get_mag_data();
				check_rc(rc, "error while getting data from mag");
			} else {
				INV_MSG(INV_MSG_LEVEL_WARNING, "Mag is not yet ready.");
			}
		}
#endif

    return rc;
}

#if USE_MAG
static void int_timer_mag_data_rdy_cb(void *context)
{
	(void)context;

	timestamp_mag                 = inv_timer_get_counter(TIMEBASE_TIMER);
	irq_from_timer_mag_data_ready = 1;

	inv_timer_channel_stop(MAG_DATA_TIMER, mag_timer_ch);
}
#endif /* USE_MAG */

/* Helper function to check RC value and block program execution */
static void check_rc(int rc, const char *msg_context)
{
	if (rc < 0) {
		printf("%s: error %d (%s)\n", msg_context, rc, inv_error_str(rc));
	}
}
