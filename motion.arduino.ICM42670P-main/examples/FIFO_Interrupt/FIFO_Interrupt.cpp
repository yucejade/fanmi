/*
 *
 * Copyright (c) [2022] by InvenSense, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <stdio.h>
#include "ICM42670P.h"

uint8_t irq_received = 0;
ICM42670 IMU;

void event_cb( inv_imu_sensor_event_t* evt )
{
    // Format data for Serial Plotter
    if ( IMU.isAccelDataValid( evt ) && IMU.isGyroDataValid( evt ) )
    {
        // Format data for Serial Plotter
        printf( "AccelX:%f,", evt->accel[ 0 ] / 2048.0);
        printf( "AccelY:%f,", evt->accel[ 1 ] / 2048.0);
        printf( "AccelZ:%f,", evt->accel[ 2 ] / 2048.0);
        printf( "GyroX:%f,", evt->gyro[ 0 ] / 16.4);
        printf( "GyroY:%f,", evt->gyro[ 1 ] / 16.4);
        printf( "GyroZ:%f,", evt->gyro[ 2 ] / 16.4);
        printf( "Temperature:%d", evt->temperature );
        printf( "\n" );
    }
}

void irq_handler( void )
{
    IMU.getDataFromFifo( event_cb );
}

int main( int argc, char* argv[] )
{
    ICM42670 IMU;
    int      ret;

    // Initializing the ICM42670
    ret = IMU.begin();
    if ( ret != 0 )
    {
        perror( "ICM42670 initialization failed: " );
        return -1;
    }

    // Enable interrupt on pin 2, Fifo watermark=10
    IMU.enableFifoInterrupt( 2, irq_handler, 10 );
    // Accel ODR = 100 Hz and Full Scale Range = 16G
    IMU.startAccel( 100, 16 );
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    IMU.startGyro( 100, 2000 );
}
