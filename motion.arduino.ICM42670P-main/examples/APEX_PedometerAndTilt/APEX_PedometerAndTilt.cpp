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
#include <chrono>
#include <stdio.h>
#include <thread>

ICM42670 IMU;

void irq_handler( void )
{
    uint32_t step_count=0;
    float step_cadence=0;
    const char* activity=nullptr;

    if(IMU.getPedometer(step_count,step_cadence,activity) == 0)
    {
        printf("Step count:%d\n", step_count);
        printf("Step cadence:%f(steps/sec)\n", step_cadence);
        printf("activity:%s\n", activity ? activity : "");
    }
    if(IMU.getTilt())
    {
        printf( "TILT:%s\n", "true");
    }
}

int main( int argc, char* argv[] )
{
    int ret;

    // Initializing the ICM42670
    ret = IMU.begin();
    if ( ret != 0 )
    {
        perror( "ICM42670 initialization failed: " );
        return -1;
    }

    // Pedometer enabled
    IMU.startPedometer();
    // Tilt enabled
    IMU.startTiltDetection();

    // Enable interrupt
    IMU.enableInterrupt(17, irq_handler);

    IMU.monitor.join();
    return 0;
}
