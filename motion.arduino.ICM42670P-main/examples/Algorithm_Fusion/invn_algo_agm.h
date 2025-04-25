/*
 * Copyright (C) 2023 InvenSense Inc. All Rights Reserved.
 */

/** @defgroup AGM AGM Algo
 *  @brief Algorithm providing sensor calibration and device orientation. 
 *  @warning Supported sampling frequency [50 Hz-1000 Hz]
 *  @warning Supported gyroscope FSR [250 dps, 500 dps, 1000 dps, 2000 dps, 4000 dps]
 *  @warning Supported accelerometer FSR [1 g, 2 g, 4 g, 8 g, 16 g, 32 g]
 *  @{
 */

#ifndef _INVN_ALGO_AGM_H_
#define _INVN_ALGO_AGM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INVN_ALGO_AGM_DATA_STRUCTURE_SIZE 2150 /**< Size of the algo data structure */

/* Error code */
#define INVN_ALGO_AGM_ERROR_NULL_POINTER -127 /**< Pointer provided is NULL */
#define INVN_ALGO_AGM_ERROR_BAD_ARG      -126 /**< Invalid argument provided */

/*
 * Algorithm input masks
 * Indicates to the algo that the corresponding input have been updated.
 */
#define INVN_ALGO_AGM_INPUT_MASK_ACC (1 << 0) /**< Raw Accel update mask */
#define INVN_ALGO_AGM_INPUT_MASK_GYR (1 << 1) /**< Raw Gyro update mask */
#define INVN_ALGO_AGM_INPUT_MASK_MAG (1 << 2) /**< Raw Mag update mask */

/*
 * Algorithm output masks
 * Indicates that the algorithm updated the corresponding output.
 */
#define INVN_ALGO_AGM_OUTPUT_MASK_ACCEL_CAL (1 << 0) /**< Cal Accel update mask */
#define INVN_ALGO_AGM_OUTPUT_MASK_GYRO_CAL  (1 << 1) /**< Cal Gyro update mask */
#define INVN_ALGO_AGM_OUTPUT_MASK_MAG_CAL   (1 << 2) /**< Cal Mag update mask */
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AG   (1 << 3) /**< AG fusion update mask (GRV: Game Rotation Vector) */
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AM   (1 << 4) /**< AM fusion update mask (GeomagRV: Geomagnetic Rotation Vector) */
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AGM  (1 << 5) /**< AGM fusion update mask (RV: Rotation Vector) */
#define INVN_ALGO_AGM_OUTPUT_MASK_GRAVITY   (1 << 6) /**< Gravity update mask */
#define INVN_ALGO_AGM_OUTPUT_MASK_LINEARACC (1 << 7) /**< Linear acceleration update mask */

/** @brief List of available Mode ID to adapt algo to specific usage. */
typedef enum {
	INVN_ALGO_AGM_MOBILE     = 1, /**< Optimize for Mobile usage. */
	INVN_ALGO_AGM_AUTOMOTIVE = 2, /**< Optimize for Automotive usage. */
} InvnAlgoAGMModeId;

/** @brief Data structure holding internal algorithm state. 
 *         Ensure data buffer is aligned to 32 bits for 32-bits MCU.
 *         Other methods can be used to align memory using malloc or attribute((aligned, 4))
 */
typedef union InvnAlgoAGMStruct {
	uint8_t  data[INVN_ALGO_AGM_DATA_STRUCTURE_SIZE];
	uint32_t data32;
} InvnAlgoAGMStruct;

/** @brief AGM input structure */
typedef struct {
	int32_t mask;          /**< Mask to specify updated inputs */
	int64_t sRimu_time_us; /**< Accel and Gyro timestamp in us. Use 0 if not available. */
	int32_t sRacc_data[3]; /**< Raw accel coded over 20 bits (FSR g = 1 << 19 LSB) */
	int32_t sRgyr_data[3]; /**< Raw gyro coded over 20 bits (FSR dps = 1 << 19 LSB) */
	int16_t sRtemp_data;   /**< Raw temperature */
	int32_t sRmag_data[3]; /**< Raw mag */
	int64_t sRmag_time_us; /**< Mag timestamp in us. Use 0 if not available. */
} InvnAlgoAGMInput;

/** @brief AGM output structure */
typedef struct {
	int32_t mask; /**< Mask to specify updated outputs */

	int32_t acc_uncal_q16[3];  /**< Uncalibrated accel in g coded as Q16 */
	int32_t acc_cal_q16[3];    /**< Calibrated accel in g coded as Q16 */
	int32_t acc_bias_q16[3];   /**< Accel biases in g coded as Q16 */
	int8_t  acc_accuracy_flag; /**< Accel accuracy from 0 (not calibrated) to 3 (well calibrated) */

	int32_t gyr_uncal_q16[3];  /**< Uncalibrated gyro in dps coded as Q16 */
	int32_t gyr_cal_q16[3];    /**< Calibrated gyro in dps coded as Q16 */
	int32_t gyr_bias_q16[3];   /**< Gyro biases in dps coded as Q16 */
	int8_t  gyr_accuracy_flag; /**< Gyro accuracy from 0 (not calibrated) to 3 (well calibrated) */

	int32_t mag_uncal_q16[3];  /**< Uncalibrated mag in uT coded as Q16 */
	int32_t mag_cal_q16[3];    /**< Calibrated mag in uT coded as Q16 */
	int32_t mag_bias_q16[3];   /**< Mag biases in uT coded as Q16 */
	int8_t  mag_accuracy_flag; /**< Mag accuracy from 0 (not calibrated) to 3 (well calibrated) */

	int32_t grv_quat_q30[4]; /**< GRV quaternion (Accel and Gyro fusion) with WXYZ convention coded as Q30 */

	int32_t gmrv_quat_q30[4];         /**< GeomagRV quaternion (Accel and Mag fusion) with WXYZ convention coded as Q30 */
	int32_t gmrv_accuracy_q27;        /**< GeomagRV heading accuracy estimation (1-sigma) in rad coded as Q27 */
	int32_t gmrv_accuracy_3sigma_q27; /**< GeomagRV heading accuracy estimation (3-sigma) in rad coded as Q27 */

	int32_t rv_quat_q30[4];         /**< RV quaternion (Accel, Gyro and Mag fusion) with WZYZ convention coded as Q30 */
	int32_t rv_accuracy_q27;        /**< RV heading accuracy estimation (1-sigma) in rad coded as Q27 */
	int32_t rv_accuracy_3sigma_q27; /**< RV heading accuracy estimation (3-sigma) in rad coded as Q27 */

	int32_t gravity_q16[3];    /**< Gravity estimation in sensor frame */
	int32_t linear_acc_q16[3]; /**< Linear acceleration estimation in sensor frame */

	int32_t temp_degC_q16; /**< Temperature in degrees Celcius coded as Q16 */
} InvnAlgoAGMOutput;

/** @brief AGM configuration structure */
typedef struct {
	int32_t *acc_bias_q16; /**< Initial Accel biases coded as Q16 (will use {0,0,0} if pointer is NULL) */
	int32_t *gyr_bias_q16; /**< Initial Gyro biases coded as Q16 (will use {0,0,0} if pointer is NULL) */
	int32_t *mag_bias_q16; /**< Initial Mag biases coded as Q16 (will use {0,0,0} if pointer is NULL) */

	int8_t acc_accuracy; /**< Initial Accel biases accuracy (0 to 3) */
	int8_t gyr_accuracy; /**< Initial Gyro biases accuracy (0 to 3)  */
	int8_t mag_accuracy; /**< Initial Mag biases accuracy (0 to 3)  */

	int32_t acc_fsr; /**< Accel full scale range in g */
	int32_t gyr_fsr; /**< Gyro full scale range in dps */

	uint32_t acc_odr_us; /**< Accel output data rate in us */
	uint32_t gyr_odr_us; /**< Gyro output data rate in us */

	int32_t  mag_sc_q16; /**< Mag sensitivity in uT/LSB coded as Q16 */
	uint32_t mag_odr_us; /**< Mag output data rate in us */

	int32_t temp_sensitivity; /**< Temperature sensitivity in Celcius/LSB coded as Q30 */
	int32_t temp_offset;      /**<  Temperature offset in degrees Celcius coded as Q16 */

	int32_t gyr_cal_stationary_duration_us; /**< Duration in us for no motion window used for gyro bias calibration. Range: [0, 6000000] */
	int32_t gyr_cal_sample_number_log2;     /**< Define the number of gyro samples (N = 2^gyr_cal_sample_number_log2) used for calibration. Range: [6, 8] */
	int32_t gyr_cal_threshold_metric1;      /**< Stationary detection threshold of 1st metric for gyro calibration. Range [0, 1200] */
	int32_t gyr_cal_threshold_metric2;      /**< Stationary detection threshold of 2nd metric for gyro calibration. Range [0, 80000] */
} InvnAlgoAGMConfig;

/** @brief Returns library version x.y.z-suffix as a char array
 *  @return library version a char array "x.y.z-suffix"
 */
const char *invn_algo_agm_version(void);

/** @brief Generates default configuration.
 *  @param[out] config  Config structure to be updated.
 *  @param[in] mode_id  Desired algo mode.
 *  @return             0 on success, negative value on error.
 */
int invn_algo_agm_generate_config(InvnAlgoAGMConfig *config, InvnAlgoAGMModeId mode_id);

/** @brief Initializes algo according to config structure provided and reset internal states.
 *  @param[in] self    Pointer to algo structure.
 *  @param[in] config  Desired configuration.
 *  @return            0 on success, negative value on error.
 */
int invn_algo_agm_init(InvnAlgoAGMStruct *self, const InvnAlgoAGMConfig *config);

/** @brief Sets algo config structure.
 *  @param[in] self    Pointer to algo structure.
 *  @param[in] config  Desired configuration.
 *  @return            0 on success, negative value on error.
 */
int invn_algo_agm_set_config(InvnAlgoAGMStruct *self, const InvnAlgoAGMConfig *config);

/** @brief Enable or disable accel, gyro or mag calibration.
 *         This fonction can be called without having the re-init the algorithm.
 *  @param[in] self          Pointer to algo structure.
 *  @param[in] gyro_cal_en   Enable (1) or disable (0) Gyro calibration.
 *  @param[in] accel_cal_en  Enable (1) or disable (0) Accel calibration.
 *  @param[in] mag_cal_en    Enable (1) or disable (0) Mag calibration.
 *  @return                  0 on success, negative value on error.
 */
int invn_algo_agm_enable_calibration(InvnAlgoAGMStruct *self, const int8_t gyro_cal_en,
                                     const int8_t accel_cal_en, const int8_t mag_cal_en);

/** @brief Processes input data.
 *  @param[in] self     Pointer to algo structure.
 *  @param[in] input    Input to be processed by algo.
 *  @param[out] output  Output of the algo.
 *  @return             0 on success, negative value on error.
 */
int invn_algo_agm_process(InvnAlgoAGMStruct *self, const InvnAlgoAGMInput *input,
                          InvnAlgoAGMOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* _INVN_ALGO_AGM_H_ */

/** @} */