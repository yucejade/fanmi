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

#include "ICM42670P.h"
#include <stdio.h>
#include <chrono>
#include <thread>

void loop( ICM42670& IMU );

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

    // Accel ODR = 100 Hz and Full Scale Range = 16G
    IMU.startAccel( 100, 16 );
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    IMU.startGyro( 100, 2000 );
    // Wait IMU to start
    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

    while ( true )
        loop( IMU );

    return 0;
}

void loop( ICM42670& IMU )
{

    inv_imu_sensor_event_t imu_event;

    // Get last event
    IMU.getDataFromRegisters( imu_event );

    // Format data for Serial Plotter
    printf( "AccelX:%f,", imu_event.accel[ 0 ] / 2048.0);
    printf( "AccelY:%f,", imu_event.accel[ 1 ] / 2048.0);
    printf( "AccelZ:%f,", imu_event.accel[ 2 ] / 2048.0);
    printf( "GyroX:%f,", imu_event.gyro[ 0 ] / 16.4);
    printf( "GyroY:%f,", imu_event.gyro[ 1 ] / 16.4);
    printf( "GyroZ:%f,", imu_event.gyro[ 2 ] / 16.4);
    printf( "Temperature:%f", (imu_event.temperature / 128.0) + 25.0);
    printf( "\n" );

    // Run @ ODR 100Hz
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
}
