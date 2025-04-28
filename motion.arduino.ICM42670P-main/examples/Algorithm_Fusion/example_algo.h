/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
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
#ifndef _EXAMPLE_ALGO_H_
#define _EXAMPLE_ALGO_H_

#include <stdint.h>

#include "imu/inv_imu_transport.h"
#include "imu/inv_imu_defs.h"
#include "imu/inv_imu_driver.h"

/*** Example configuration ***/

/* 
 * Select communication link between SmartMotion and IMU 
 */
//#define SERIF_TYPE UI_SPI4
#define SERIF_TYPE           UI_I2C

/*
 * Set this define to 0 to disable mag support
 * Recommended value: 1
 */
#define USE_MAG 0

/*
 * Set power mode flag
 * Set this flag to run example in low-noise mode.
 * Reset this flag to run example in low-power mode.
 * Note : low-noise mode is not available with sensor data frequencies less than 12.5Hz.
 */
#define IS_LOW_NOISE_MODE 1

/*
 * Accelerometer and gyroscope frequencies.
 * Possible values (same for accel, replace "GYRO" with "ACCEL"): 
 * - GYRO_CONFIG0_ODR_800_HZ  (800 Hz)
 * - GYRO_CONFIG0_ODR_400_HZ (400 Hz)
 * - GYRO_CONFIG0_ODR_200_HZ (200 Hz)
 * - GYRO_CONFIG0_ODR_100_HZ (100 Hz)
 * - GYRO_CONFIG0_ODR_50_HZ (50 Hz)
 */
#define GYRO_FREQ  GYRO_CONFIG0_ODR_400_HZ
#define ACCEL_FREQ ACCEL_CONFIG0_ODR_400_HZ

/*
 * Magnetometer output data rate in us
 * Only supported value: 10000 us (100 Hz)
 * Note: Not use if `USE_MAG` is set to 0
 */
#define MAG_ODR_US 10000

/*
 * Select Fifo resolution Mode (default is low resolution mode)
 * Low resolution mode : 16 bits data format
 * High resolution mode : 20 bits data format
 * Warning: Enabling High Res mode will force max FSR
 */
#define IS_HIGH_RES_MODE 0

/* 
 * Set of timers used throughout standalone applications 
 */
#define TIMEBASE_TIMER INV_TIMER1
#define DELAY_TIMER    INV_TIMER2
#define MAG_DATA_TIMER INV_TIMER4

/*
 * Enum for possible commands from user
 */
typedef enum {
	ALGORITHM_RESET = 0,
} inv_algo_commands;

/**
 * \brief This function is in charge of reseting and initializing IMU device. It should
 * be successfully executed before any access to IMU device.
 * 
 * \param[in] icm_serif : Serial interface object.
 * \return 0 on success, negative value on error.
 */
int setup_imu_device(const struct inv_imu_serif *icm_serif);

/**
 * \brief This function configures the device in order to output gyro and accelerometer.
 *
 * It initializes clock calibration module (this will allow to extend the 16 bits 
 * timestamp produced by IMU to a 64 bits timestamp).
 * Then function sets full scale range and frequency for both accel and gyro and it 
 * starts them in the requested power mode. (note that low-power mode is not available 
 * for gyroscope in IMU).
 *
 * \return 0 on success, negative value on error.
 */
int configure_imu_device(void);

/**
 * \This function clears biases and accuracies.
 *
 * \return 0 on success, negative value on error.
 */
int reset_agm_biases(void);

/**
 * \brief This function initializes biases and accuracies for accelerometer, gyroscope and magnetometer.
 *
 * \return 0 on success, negative value on error.
 */
int init_agm_biases(void);

/**
 * \brief This function initializes the AGM algorithm.
 *
 * \return 0 on success, negative value on error.
 */
int init_agm_algo(void);

/**
 * \brief This function extracts data from the IMU FIFO.
 *
 * The function just calls IMU driver function inv_imu_get_data_from_fifo.
 * But note that for each packet extracted from FIFO, a user defined function is called to 
 * allow custom handling of each packet. In this example custom packet handling function
 * is imu_callback.
 *
 * \return 0 on success, negative value on error.
 */
int get_imu_data(void);

/**
 * \brief This function is the custom handling packet function.
 *
 * It is passed in parameter at driver init time and it is called by 
 * inv_imu_get_data_from_fifo function each time a new valid packet is extracted 
 * from FIFO.
 * In this implementation, function extends packet timestamp from 16 to 64 bits and then
 * process data from packet and print them on UART.
 *
 * \param[in] event structure containing sensor data from one packet
 */
void imu_callback(inv_imu_sensor_event_t *event);

#if USE_MAG
/*!
 * \brief Set up magnetometer IST-8306
 * \param[in] ist_serif : Serial interface object.
 * \return 0 in case of success, negative value in case 
 */
int setup_mag_device(struct inv_ist8306_serif *ist_serif);

/*!
 * \brief Start a new data acquisition on magnetometer.
 * 
 * Magnetometer only works in one shot mode. Thus this function only initiates one data
 * acquisition.
 *
 * \return 0 in case of success, negative value in case 
 */
int start_mag_acquisition(void);

/*!
 * \brief Check if a new data is ready on the mag.
 * \param[out] drdy: data ready flag.
 * \return 0 in case of success, negative value otherwise 
 */
int get_mag_drdy(uint8_t *drdy);

/*!
 * \brief This function reads data from the Akm09915 and pipe them to the algorithms.
 * Finally it prints algorithms output on log uart.
 *
 * \return 0 on success, negative value on error.
 */
int get_mag_data(void);
#endif /* USE_MAG */

/**
 * \brief Helper to convert bitfield to odr value
 * \param[in] odr_bitfield enum containing bitfield
 *
 */
uint32_t bitfield_to_us(uint32_t odr_bitfield);

/**
 * \brief This function processes commands received from frontend
 * \param[in] cmd The command to be processed
 */
//void on_command_received(inv_algo_commands cmd);

#endif /* !_EXAMPLE_ALGO_H_ */
