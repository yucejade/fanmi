#pragma once
//
#include "Calculation/AhrsCalculation.h"
#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"

struct sensor_device
{
    MMC56x3*         sensor_mmc;
    ICM42670*        sensor_imu;
    AhrsCalculation* ahrs_calculation;
    //
    moodycamel::ConcurrentQueue< SENSOR_DB >* sensor_data_queue;
};
//

// ----------------------------------------------------------------------
static void* read_sensor( void* arg )
{
    struct sensor_device* pArg = ( struct sensor_device* )arg;
    //

    //
    while ( true )
    {
        std::string info = "Info:\n";
        SENSOR_DB   sensor_data;

        sensor_data.time = clock();
        info += "Time: " + std::to_string( sensor_data.time ) + "\n";

        // MMC56x3
        float x, y, z;
        bool  ret = pArg->sensor_mmc->getEvent( x, y, z );
        if ( ret )
        {
            sensor_data.mag_x = x;
            sensor_data.mag_y = y;
            sensor_data.mag_z = z;

            info += "Magnetic field: x = " + std::to_string( x ) + " uT, y = " + std::to_string( y ) + " uT, z = " + std::to_string( z ) + " uT\n";
            // std::cout << "Magnetic field: x = " << x << " uT, y = " << y << " uT, z = " << z << " uT, all = " << std::sqrt( std::pow( x, 2 ) + std::pow( y, 2 ) + std::pow( z, 2 ) ) << " uT" << std::endl;
        }
        else
        {
            std::cout << "Cannot read magnetic in continuous mode" << std::endl;
            continue;
        }

        float temp = pArg->sensor_mmc->readTemperature();
        if ( ! std::isnan( temp ) )
        {
            info += "Magnetic Temperature: " + std::to_string( temp ) + " C\n";
            // std::cout << "Temperature: " << temp << " C" << std::endl;
        }
        else
        {
            std::cout << "Cannot read temperature in continuous mode" << std::endl;
            continue;
        }

        // TDK42607
        inv_imu_sensor_event_t imu_event;

        // Get last event
        int rc = pArg->sensor_imu->getDataFromRegisters( imu_event );
        if ( INV_ERROR_SUCCESS == rc )
        {
            sensor_data.acc_x  = ( ( int32_t )imu_event.accel[ 0 ] ) / 2048.0;
            sensor_data.acc_y  = ( ( int32_t )imu_event.accel[ 1 ] ) / 2048.0;
            sensor_data.acc_z  = ( ( int32_t )imu_event.accel[ 2 ] ) / 2048.0;
            sensor_data.gyro_x = ( ( int32_t )imu_event.gyro[ 0 ] ) / 16.4;
            sensor_data.gyro_y = ( ( int32_t )imu_event.gyro[ 1 ] ) / 16.4;
            sensor_data.gyro_z = ( ( int32_t )imu_event.gyro[ 2 ] ) / 16.4;
            info += "Accelerometer: x = " + std::to_string( sensor_data.acc_x ) + " g, y = " + std::to_string( sensor_data.acc_y ) + " g, z = " + std::to_string( sensor_data.acc_z ) + " g\n";
            info += "Gyroscope: x = " + std::to_string( sensor_data.gyro_x ) + " dps, y = " + std::to_string( sensor_data.gyro_y ) + " dps, z = " + std::to_string( sensor_data.gyro_z ) + " dps\n";
        }
        else
        {
            std::cout << "Cannot read IMU data" << std::endl;
            continue;
        }

        //
        pArg->ahrs_calculation->SolveAnCalculation( &sensor_data );
        //
        info += "Quaternion: " + std::to_string( sensor_data.quate_x ) + " " + std::to_string( sensor_data.quate_y ) + " " + std::to_string( sensor_data.quate_z ) + " " + std::to_string( sensor_data.quate_w ) + "\n";
        info += "Roll: " + std::to_string( sensor_data.roll ) + " Pitch: " + std::to_string( sensor_data.pitch ) + " Yaw: " + std::to_string( sensor_data.yaw ) + "\n";
        info += "Pos X: " + std::to_string( sensor_data.pos_x ) + " Pos Y: " + std::to_string( sensor_data.pos_y ) + " Pos Z: " + std::to_string( sensor_data.pos_z ) + "\n";
        info += "Queue size: " + std::to_string( pArg->sensor_data_queue->size_approx() ) + "\n";

        //
        sensor_data.info = info;
        // 生成数据
        pArg->sensor_data_queue->enqueue( sensor_data );
        //
        URHO3D_LOGINFO( info.c_str() );
        // Run @ ODR 100Hz
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
        //
    }

    //
    return nullptr;
}
