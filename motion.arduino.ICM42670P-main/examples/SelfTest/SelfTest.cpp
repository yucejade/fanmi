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

    ret = IMU.run_self_test();
    printf("Device self-test %s\n", (INV_ERROR_SUCCESS == ret) ? "OK" : "KO");

    return 0;
}
