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
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <stddef.h>
#include "example_algo.h"
#include "imu/inv_imu_driver.h"
#include "Fusion.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/** AGM input structure */
typedef struct {
	int32_t mask;          /**< Mask to specify updated inputs */
	int64_t sRimu_time_us; /**< Accel and Gyro timestamp in us. Use 0 if not available. */
	int32_t sRacc_data[3]; /**< Raw accel coded over 20 bits (FSR g = 1 << 19 LSB) */
	int32_t sRgyr_data[3]; /**< Raw gyro coded over 20 bits (FSR dps = 1 << 19 LSB) */
	int16_t sRtemp_data;   /**< Raw temperature */
	int32_t sRmag_data[3]; /**< Raw mag */
	int64_t sRmag_time_us; /**< Mag timestamp in us. Use 0 if not available. */
} FMAGMInput;

typedef struct {
    FusionQuaternion quaternion;
}FMAGMOutput;

/* Bias previously stored in flash */
typedef struct sensor_biases
{
    int32_t bias_q16[ 3 ];
    uint8_t is_saved;
} sensor_biases_t;

/*
 * Printing period (prevent terminal overload)
 * Value expected in Hertz
 */
#define PRINTING_PERIOD_MS 1000

/* How often do we print the output */
static int iter_algo       = 0;
static int print_period_us = PRINTING_PERIOD_MS * 1000; /* 1 s */

/* IMU driver object */
static struct inv_imu_device icm_driver;

/* Full Scale Range */
#if IS_HIGH_RES_MODE
    #if ICM_HFSR_SUPPORTED
static const int32_t acc_fsr = 32;   /* 32 g */
static const int32_t gyr_fsr = 4000; /* 4000 dps */
    #else
static const int32_t acc_fsr = 16;   /* 16 g */
static const int32_t gyr_fsr = 2000; /* 2000 dps */
    #endif
#else
static const int32_t acc_fsr = 4;    /* 4 g */
static const int32_t gyr_fsr = 2000; /* 2000 dps */
#endif

#if USE_MAG
/* Ist-8306 driver object */
static inv_ist8306_t ist_driver;

/* Mag's timestamp (defined in main.c) */
extern volatile uint64_t timestamp_mag;

/* Variable to keep track if the mag has initialized successfully (defined in main.c) */
extern int mag_init_successful;
#endif

static FMAGMInput  input;
static FMAGMOutput output;

/* Accelerometer gyroscope and magnetometer biases */
static int32_t acc_bias[ 3 ];
static int32_t gyr_bias[ 3 ];
static int32_t mag_bias[ 3 ];

/*
 * Defines
 */

/* Mask to print data through UART */
#define MASK_PRINT_INPUT_DATA     0x01 /** algorithm inputs */
#define MASK_PRINT_ACC_DATA       0x02 /** accel data */
#define MASK_PRINT_GYR_DATA       0x04 /** gyro data */
#define MASK_PRINT_MAG_DATA       0x08 /** magnetometer data */
#define MASK_PRINT_6AXIS_DATA     0x10 /** 6-axis data */
#define MASK_PRINT_9AXIS_DATA     0x20 /** 9-axis data */
#define MASK_PRINT_GRAVITY_DATA   0x40 /** Gravity vector */
#define MASK_PRINT_LINEARACC_DATA 0x80 /** Linear acceleration vector */

/* Output data to print */
/*
 * Output data to print
 * Default behavior: only print accel, gyro and mag
 */
uint32_t data_to_print = MASK_PRINT_ACC_DATA | MASK_PRINT_GYR_DATA | MASK_PRINT_MAG_DATA | MASK_PRINT_6AXIS_DATA | MASK_PRINT_9AXIS_DATA;

/* --------------------------------------------------------------------------------------
 *  static function declaration
 * -------------------------------------------------------------------------------------- */
/* biases storage */
static int  retrieve_biases( int32_t acc_bias[ 3 ], int32_t gyr_bias[ 3 ], int32_t mag_bias[ 3 ] );
static void store_biases( void );

#if ! IS_HIGH_RES_MODE
static int gyro_fsr_dps_to_bitfield( int32_t fsr );
static int accel_fsr_g_to_bitfield( int32_t fsr );
#endif
static void print_data( uint64_t time, const FMAGMInput* input, const InvnAlgoAGMOutput* output );
static void print_inputs( uint64_t time, const FMAGMInput* input );
static void print_outputs( uint64_t time, const InvnAlgoAGMOutput* output );
static void fixedpoint_to_float( const int32_t* in, float* out, const uint8_t fxp_shift, const uint8_t dim );

/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */

int setup_imu_device( const struct inv_imu_serif* icm_serif )
{
    int     rc = 0;
    uint8_t who_am_i;

    /* Init device */
    rc = inv_imu_init( &icm_driver, icm_serif, imu_callback );
    if ( rc != INV_ERROR_SUCCESS )
    {
        printf( "Failed to initialize IMU!\n" );
        return rc;
    }

    /* Check WHOAMI */
    rc = inv_imu_get_who_am_i( &icm_driver, &who_am_i );
    if ( rc != INV_ERROR_SUCCESS )
    {
        printf( "Failed to read whoami!\n" );
        return rc;
    }

    if ( who_am_i != INV_IMU_WHOAMI )
    {
        printf( "Bad WHOAMI value!\n" );
        printf( "Read 0x%02x, expected 0x%02x\n", who_am_i, INV_IMU_WHOAMI );
        return INV_ERROR;
    }

    return rc;
}

int configure_imu_device( void )
{
    int                           rc = 0;
    GYRO_CONFIG1_GYRO_FILT_BW_t   gyro_bw;
    ACCEL_CONFIG1_ACCEL_FILT_BW_t accel_bw;

    /* FSR */
#if IS_HIGH_RES_MODE
    rc |= inv_imu_enable_high_resolution_fifo( &icm_driver );
#else
    rc |= inv_imu_set_accel_fsr( &icm_driver, ( ACCEL_CONFIG0_FS_SEL_t )accel_fsr_g_to_bitfield( acc_fsr ) );
    rc |= inv_imu_set_gyro_fsr( &icm_driver, ( GYRO_CONFIG0_FS_SEL_t )gyro_fsr_dps_to_bitfield( gyr_fsr ) );
#endif

    /* Frequencies */
    rc |= inv_imu_set_accel_frequency( &icm_driver, ACCEL_FREQ );
    rc |= inv_imu_set_gyro_frequency( &icm_driver, GYRO_FREQ );

    /* LPF configuration */
    switch ( ACCEL_FREQ )
    {
        case ACCEL_CONFIG0_ODR_800_HZ:
            accel_bw = ACCEL_CONFIG1_ACCEL_FILT_BW_180;
            break;
        case ACCEL_CONFIG0_ODR_400_HZ:
            accel_bw = ACCEL_CONFIG1_ACCEL_FILT_BW_121;
            break;
        case ACCEL_CONFIG0_ODR_200_HZ:
            accel_bw = ACCEL_CONFIG1_ACCEL_FILT_BW_53;
            break;
        case ACCEL_CONFIG0_ODR_100_HZ:
            accel_bw = ACCEL_CONFIG1_ACCEL_FILT_BW_25;
            break;
        case ACCEL_CONFIG0_ODR_50_HZ:
        case ACCEL_CONFIG0_ODR_25_HZ:
            accel_bw = ACCEL_CONFIG1_ACCEL_FILT_BW_16;
            break;
        default:
            printf( "Unsupported accel ODR\n" );
            return -1;
    }
    rc |= inv_imu_set_accel_ln_bw( &icm_driver, accel_bw );
    switch ( GYRO_FREQ )
    {
        case GYRO_CONFIG0_ODR_800_HZ:
            gyro_bw = GYRO_CONFIG1_GYRO_FILT_BW_180;
            break;
        case GYRO_CONFIG0_ODR_400_HZ:
            gyro_bw = GYRO_CONFIG1_GYRO_FILT_BW_121;
            break;
        case GYRO_CONFIG0_ODR_200_HZ:
            gyro_bw = GYRO_CONFIG1_GYRO_FILT_BW_53;
            break;
        case GYRO_CONFIG0_ODR_100_HZ:
            gyro_bw = GYRO_CONFIG1_GYRO_FILT_BW_25;
            break;
        case GYRO_CONFIG0_ODR_50_HZ:
        case GYRO_CONFIG0_ODR_25_HZ:
            gyro_bw = GYRO_CONFIG1_GYRO_FILT_BW_16;
            break;
        default:
            printf( "Unsupported gyro ODR\n" );
            return -1;
    }
    rc |= inv_imu_set_gyro_ln_bw( &icm_driver, gyro_bw );

    /* Enable sensors */
#if IS_LOW_NOISE_MODE
    rc |= inv_imu_enable_accel_low_noise_mode( &icm_driver );
#else
    rc |= inv_imu_enable_accel_low_power_mode( &icm_driver );
#endif
    rc |= inv_imu_enable_gyro_low_noise_mode( &icm_driver );

    return rc;
}

/*
 * This function clears biases and accuracies.
 */
int reset_agm_biases( void )
{
    memset( acc_bias, 0, sizeof( acc_bias ) );
    memset( gyr_bias, 0, sizeof( gyr_bias ) );
    memset( mag_bias, 0, sizeof( mag_bias ) );

    return 0;
}

/*
 * This function initializes biases and accuracies for accelerometer, gyroscope and magnetometer.
 */
int init_agm_biases( void )
{
    /* Retrieve stored biases */
    if ( retrieve_biases( acc_bias, gyr_bias, mag_bias ) == 0 )
    {
        printf( "   Biases loaded from flash:\n" );
        printf( "    - Accel: [%f %f %f]g\n", ( float )acc_bias[ 0 ] / ( 1 << 16 ), ( float )acc_bias[ 1 ] / ( 1 << 16 ), ( float )acc_bias[ 2 ] / ( 1 << 16 ) );
        printf( "    - Gyro:  [%f %f %f]dps\n", ( float )gyr_bias[ 0 ] / ( 1 << 16 ), ( float )gyr_bias[ 1 ] / ( 1 << 16 ), ( float )gyr_bias[ 2 ] / ( 1 << 16 ) );
        printf( "    - Mag:   [%f %f %f]uT\n", ( float )mag_bias[ 0 ] / ( 1 << 16 ), ( float )mag_bias[ 1 ] / ( 1 << 16 ), ( float )mag_bias[ 2 ] / ( 1 << 16 ) );
    }
    else
    {
        printf( "   No bias values retrieved\n" );
        memset( acc_bias, 0, sizeof( acc_bias ) );
        memset( gyr_bias, 0, sizeof( gyr_bias ) );
        memset( mag_bias, 0, sizeof( mag_bias ) );
    }

    return 0;
}

int init_agm_algo( void )
{
    return 0;
}

// TODO:什么时候调用？
int get_imu_data( void )
{
    /*
     * Extract packets from FIFO. Callback defined at init time (i.e.
     * imu_callback) will be called for each valid packet extracted from
     * FIFO.
     */
    return inv_imu_get_data_from_fifo( &icm_driver );
}

void imu_callback( inv_imu_sensor_event_t* event )
{
    int      rc            = 0;
    uint64_t irq_timestamp = 0;

    input.mask = 0;

    /*
     * Retrieve accel and gyro data
     */
    if ( event->sensor_mask & ( 1 << INV_SENSOR_ACCEL ) )
    {
#if IS_HIGH_RES_MODE
        input.sRacc_data[ 0 ] = ( ( ( int32_t )event->accel[ 0 ] << 4 ) ) | event->accel_high_res[ 0 ];
        input.sRacc_data[ 1 ] = ( ( ( int32_t )event->accel[ 1 ] << 4 ) ) | event->accel_high_res[ 1 ];
        input.sRacc_data[ 2 ] = ( ( ( int32_t )event->accel[ 2 ] << 4 ) ) | event->accel_high_res[ 2 ];
#else
        input.sRacc_data[ 0 ] = ( int32_t )event->accel[ 0 ] << 4;
        input.sRacc_data[ 1 ] = ( int32_t )event->accel[ 1 ] << 4;
        input.sRacc_data[ 2 ] = ( int32_t )event->accel[ 2 ] << 4;
#endif
        //TODO:input.mask |= INVN_ALGO_AGM_INPUT_MASK_ACC;
    }

    if ( event->sensor_mask & ( 1 << INV_SENSOR_GYRO ) )
    {
#if IS_HIGH_RES_MODE
        input.sRgyr_data[ 0 ] = ( ( ( int32_t )event->gyro[ 0 ] << 4 ) ) | event->gyro_high_res[ 0 ];
        input.sRgyr_data[ 1 ] = ( ( ( int32_t )event->gyro[ 1 ] << 4 ) ) | event->gyro_high_res[ 1 ];
        input.sRgyr_data[ 2 ] = ( ( ( int32_t )event->gyro[ 2 ] << 4 ) ) | event->gyro_high_res[ 2 ];
#else
        input.sRgyr_data[ 0 ] = ( int32_t )event->gyro[ 0 ] << 4;
        input.sRgyr_data[ 1 ] = ( int32_t )event->gyro[ 1 ] << 4;
        input.sRgyr_data[ 2 ] = ( int32_t )event->gyro[ 2 ] << 4;
#endif
        //TODO:input.mask |= INVN_ALGO_AGM_INPUT_MASK_GYR;
    }

    input.sRtemp_data   = event->temperature;
    input.sRimu_time_us = irq_timestamp;

    /* Process the AgmFusion Algo */
    // TODO:
    //rc |= invn_algo_agm_process( &agm, &input, &output );

    /* Check error */
    if ( rc < 0 )
    {
        printf( "Error while processing Accel/Gyro in AGM algorithm (rc=%d)\n", rc );
        return;
    }

    store_biases();

    /* Print data to temination for print of at least one of the input was provided */
    if ( input.mask != 0 )
        print_data( input.sRimu_time_us, &input, &output );
}

#if USE_MAG
int setup_mag_device( struct inv_ist8306_serif* ist_serif )
{
    int     rc;
    uint8_t whoami;

    printf( "Resetting IST-8306\n" );

    /* Reset Ist8306 driver states */
    inv_ist8306_reset_states( &ist_driver, ist_serif );

    /* Init Ist8306 device */
    rc = inv_ist8306_init( &ist_driver );
    if ( rc != INV_ERROR_SUCCESS )
    {
        printf( "Failed to initialize Ist8306.\n" );
        return rc;
    }

    /* Check WHOAMI */
    printf( "Check Ist8306 whoami value\n" );

    rc = inv_ist8306_get_whoami( &ist_driver, &whoami );
    if ( rc != INV_ERROR_SUCCESS )
    {
        printf( "Failed to read Ist8306 whoami value.\n" );
        return rc;
    }

    if ( whoami != IST8306_WHOAMI )
    {
        printf( "Bad WHOAMI value for mag device. Got 0x%02x\n", whoami );
        return INV_ERROR;
    }

    return rc;
}

int start_mag_acquisition( void )
{
    return inv_ist8306_trigger_one_measurement( &ist_driver );
}

int get_mag_drdy( uint8_t* drdy )
{
    return inv_ist8306_get_drdy( &ist_driver, drdy );
}

int get_mag_data()
{
    int      rc = 0;
    int16_t  raw_mag[ 3 ];
    uint64_t irq_timestamp = 0;

    inv_disable_irq();
    irq_timestamp = timestamp_mag;
    inv_enable_irq();

    /* Read Ist-8306 data */
    rc |= inv_ist8306_get_data( &ist_driver, raw_mag );
    if ( rc != 0 )
        return rc;

    input.sRmag_data[ 0 ] = ( int32_t )raw_mag[ 0 ];
    input.sRmag_data[ 1 ] = ( int32_t )raw_mag[ 1 ];
    input.sRmag_data[ 2 ] = ( int32_t )raw_mag[ 2 ];
    input.sRmag_time_us   = irq_timestamp;
    input.mask            = INVN_ALGO_AGM_INPUT_MASK_MAG;

    /* Apply the mounting matrix configuration to the mag data polled */
    apply_mounting_matrix( mag_matrix, input.sRmag_data );

    rc |= invn_algo_agm_process( &agm, &input, &output );

    /* Check error */
    if ( rc < 0 )
    {
        printf( "Error while processing Mag in AGM algorithm (rc=%d)\n", rc );
        return rc;
    }

    /* Print data to frontend for print */
    print_data( input.sRmag_time_us, &input, &output );

    return rc;
}
#endif /* USE_MAG */

#if 0  // Delete this section if not needed
void on_command_received(inv_algo_commands cmd)
{
	switch (cmd) {
	case ALGORITHM_RESET:
		reset_agm_biases();
		init_agm_algo();
		printf("Algorithm and sensors biases reset.\n");
		break;

	default:
		printf("Unknown command received\n");
		break;
	}
}
#endif

uint32_t bitfield_to_us( uint32_t odr_bitfield )
{
    switch ( ( GYRO_CONFIG0_ODR_t )odr_bitfield )
    {
        case GYRO_CONFIG0_ODR_1600_HZ:
            return 625;
        case GYRO_CONFIG0_ODR_800_HZ:
            return 1250;
        case GYRO_CONFIG0_ODR_400_HZ:
            return 2500;
        case GYRO_CONFIG0_ODR_200_HZ:
            return 5000;
        case GYRO_CONFIG0_ODR_100_HZ:
            return 10000;
        case GYRO_CONFIG0_ODR_50_HZ:
            return 20000;
        default:
            return 640000;
    }
}

/*
 * Static functions definition
 */

/*
 * \brief Read NV memory and retrieve stored biases
 * \param[out] acc_bias_q16 Previously stored acc bias
 * \param[out] gyr_bias_q16 Previously stored gyr bias
 * \param[out] mag_bias_q16 Previously stored mag bias
 * \return 0 on success, -1 if no bias are in NV, an error otherwise
 */
static int retrieve_biases( int32_t acc_bias[ 3 ], int32_t gyr_bias[ 3 ], int32_t mag_bias[ 3 ] )
{
    return -1;
}

/*
 * \brief Evaluate whether biases needs to be written to flash depending on accuracies value
 */
static void store_biases( void )
{
    return;
}

#if ! IS_HIGH_RES_MODE
static int gyro_fsr_dps_to_bitfield( int32_t fsr )
{
    switch ( fsr )
    {
    #if ! ICM_HFSR_SUPPORTED
        case 250:
            return GYRO_CONFIG0_FS_SEL_250dps;
    #endif
        case 500:
            return GYRO_CONFIG0_FS_SEL_500dps;
        case 1000:
            return GYRO_CONFIG0_FS_SEL_1000dps;
        case 2000:
            return GYRO_CONFIG0_FS_SEL_2000dps;
    #if ICM_HFSR_SUPPORTED
        case 4000:
            return GYRO_CONFIG0_FS_SEL_4000dps;
    #endif
        default:
            return -1;
    }
}

static int accel_fsr_g_to_bitfield( int32_t fsr )
{
    switch ( fsr )
    {
    #if ! ICM_HFSR_SUPPORTED
        case 2:
            return ACCEL_CONFIG0_FS_SEL_2g;
    #endif
        case 4:
            return ACCEL_CONFIG0_FS_SEL_4g;
        case 8:
            return ACCEL_CONFIG0_FS_SEL_8g;
        case 16:
            return ACCEL_CONFIG0_FS_SEL_16g;
    #if ICM_HFSR_SUPPORTED
        case 32:
            return ACCEL_CONFIG0_FS_SEL_32g;
    #endif
        default:
            return -1;
    }
}
#endif

static void print_data( uint64_t time, const FMAGMInput* input, const InvnAlgoAGMOutput* output )
{
    ( void )time;

    if ( iter_algo % ( ( int )( print_period_us / bitfield_to_us( GYRO_FREQ ) ) ) == 0 )
    {
        print_inputs( input->sRimu_time_us, input );
        print_outputs( input->sRimu_time_us, output );

        /* Print carriage return to ease reading, only if some data are printed */
        if ( data_to_print )
            printf( "\n" );
    }

    iter_algo++;
}

static void print_inputs( uint64_t time, const FMAGMInput* input )
{
    if ( data_to_print & MASK_PRINT_INPUT_DATA )
    {
        /* IMU data */
        printf( "%ld: INPUT  RAcc=[%d, %d, %d] RGyr=[%d, %d, %d] Rtemp=[%d]\n", time,
#if IS_HIGH_RES_MODE
                input->sRacc_data[ 0 ], input->sRacc_data[ 1 ], input->sRacc_data[ 2 ], input->sRgyr_data[ 0 ], input->sRgyr_data[ 1 ], input->sRgyr_data[ 2 ],
#else
                input->sRacc_data[ 0 ] >> 4, input->sRacc_data[ 1 ] >> 4, input->sRacc_data[ 2 ] >> 4, input->sRgyr_data[ 0 ] >> 4, input->sRgyr_data[ 1 ] >> 4, input->sRgyr_data[ 2 ] >> 4,
#endif
                ( int32_t )input->sRtemp_data );
#if USE_MAG
        if ( mag_init_successful )
        {
            /* Mag data */
            printf( "%ld: INPUT  RMag=[%d, %d, %d]\n", time, input->sRmag_data[ 0 ], input->sRmag_data[ 1 ], input->sRmag_data[ 2 ] );
        }
#endif
    }
}

static void print_outputs( uint64_t time, const InvnAlgoAGMOutput* output )
{
    float acc_g[ 3 ];
    float acc_bias[ 3 ];
    float gyr_dps[ 3 ];
    float gyr_bias[ 3 ];
    float temp;
    float grv_quat[ 4 ];
    float angles_deg_grv[ 3 ];
    float gravity[ 3 ];
    float linear_acc[ 3 ];
#if USE_MAG
    float mag_cal[ 3 ];
    float mag_bias[ 3 ];
    float rv_quat[ 4 ];
    float angles_deg_rv[ 3 ];
    float rv_heading_accuracy;
#endif

    /* Convert data to float before send it to the terminal */
    fixedpoint_to_float( output->acc_cal_q16, acc_g, 16, 3 );
    fixedpoint_to_float( output->acc_bias_q16, acc_bias, 16, 3 );
    fixedpoint_to_float( output->gyr_cal_q16, gyr_dps, 16, 3 );
    fixedpoint_to_float( output->gyr_bias_q16, gyr_bias, 16, 3 );
    fixedpoint_to_float( &output->temp_degC_q16, &temp, 16, 1 );
    fixedpoint_to_float( output->grv_quat_q30, grv_quat, 30, 4 );
    fixedpoint_to_float( output->gravity_q16, gravity, 16, 3 );
    fixedpoint_to_float( output->linear_acc_q16, linear_acc, 16, 3 );
    //TODO:quaternions_to_angles( grv_quat, angles_deg_grv );

#if USE_MAG
    fixedpoint_to_float( output->mag_cal_q16, mag_cal, 16, 3 );
    fixedpoint_to_float( output->mag_bias_q16, mag_bias, 16, 3 );
    fixedpoint_to_float( output->rv_quat_q30, rv_quat, 30, 4 );
    //TODO:quaternions_to_angles( rv_quat, angles_deg_rv );
#endif

    /* Print outputs */
    if ( data_to_print & MASK_PRINT_ACC_DATA )
    {
        printf( "%ld: OUTPUT Acc=[%.3f, %.3f, %.3f]g AccBias=[%.3f, %.3f, %.3f]mg Accuracy=%d\n", time, acc_g[ 0 ], acc_g[ 1 ], acc_g[ 2 ], acc_bias[ 0 ] * 1000, acc_bias[ 1 ] * 1000, acc_bias[ 2 ] * 1000, ( int32_t )output->acc_accuracy_flag );
    }

    if ( data_to_print & MASK_PRINT_GYR_DATA )
    {
        printf( "%ld: OUTPUT Gyr=[%.3f, %.3f, %.3f]dps GyrBias=[%.3f, %.3f, %.3f]dps Temp=[%.2f]C Accuracy=%d\n", time, gyr_dps[ 0 ], gyr_dps[ 1 ], gyr_dps[ 2 ], gyr_bias[ 0 ], gyr_bias[ 1 ], gyr_bias[ 2 ], temp, ( int32_t )output->gyr_accuracy_flag );
    }

#if USE_MAG
    if ( mag_init_successful )
    {
        if ( data_to_print & MASK_PRINT_MAG_DATA )
        {
            printf( "%ld: OUTPUT Mag=[%.3f, %.3f, %.3f]uT MagBias=[%.3f, %.3f, %.3f]uT Accuracy=%d\n", time, mag_cal[ 0 ], mag_cal[ 1 ], mag_cal[ 2 ], mag_bias[ 0 ], mag_bias[ 1 ], mag_bias[ 2 ], output->mag_accuracy_flag );
        }

        if ( data_to_print & MASK_PRINT_9AXIS_DATA )
        {
            printf( "%ld: OUTPUT 9Axis=[%f, %f, %f, %f] 9AxisAccuracy=[%f]deg\n", time, rv_quat[ 0 ], rv_quat[ 1 ], rv_quat[ 2 ], rv_quat[ 3 ], rv_heading_accuracy );

            printf( "%ld: OUTPUT Euler9Axis=[%.2f, %.2f, %.2f]deg 9AxisAccuracy=[%f]deg\n", time, angles_deg_rv[ 0 ], angles_deg_rv[ 1 ], angles_deg_rv[ 2 ], rv_heading_accuracy );
        }
    }
#endif

    if ( data_to_print & MASK_PRINT_6AXIS_DATA )
    {
        printf( "%ld: OUTPUT 6Axis=[%f, %f, %f, %f]\n", time, grv_quat[ 0 ], grv_quat[ 1 ], grv_quat[ 2 ], grv_quat[ 3 ] );

        printf( "%ld: OUTPUT Euler6Axis=[%.2f, %.2f, %.2f]deg\n", time, angles_deg_grv[ 0 ], angles_deg_grv[ 1 ], angles_deg_grv[ 2 ] );
    }

    if ( data_to_print & MASK_PRINT_GRAVITY_DATA )
    {
        printf( "%ld: OUTPUT Gravity=[%f, %f, %f] Accuracy=%d\n", time, gravity[ 0 ], gravity[ 1 ], gravity[ 2 ], output->acc_accuracy_flag );
    }
    if ( data_to_print & MASK_PRINT_LINEARACC_DATA )
    {
        printf( "%ld: OUTPUT LinearAcc=[%f, %f, %f] Accuracy=%d\n", time, linear_acc[ 0 ], linear_acc[ 1 ], linear_acc[ 2 ], output->acc_accuracy_flag );
    }
}

static void fixedpoint_to_float( const int32_t* in, float* out, const uint8_t fxp_shift, const uint8_t dim )
{
    float scale = 1.f / ( float )( 1 << fxp_shift );

    for ( uint8_t i = 0; i < dim; i++ )
        out[ i ] = scale * ( float )in[ i ];
}
