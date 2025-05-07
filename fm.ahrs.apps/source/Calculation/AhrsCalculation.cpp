#include "AhrsCalculation.h"
#include <Urho3D/IO/Log.h>
#include <cstdio>
#include <time.h>
//
// 带 Context* 参数的构造函数实现
AhrsCalculation::AhrsCalculation( Urho3D::Context* context ) : Urho3D::Object( context )
{
    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings( &ahrs, &settings );
}

//
void AhrsCalculation::SolveAnCalculation( SENSOR_DB* sensor_data )
{
    // Acquire latest sensor data
    const clock_t timestamp     = sensor_data->time;
    FusionVector  gyroscope     = { sensor_data->gyro_x, sensor_data->gyro_y, sensor_data->gyro_z };
    FusionVector  accelerometer = { sensor_data->acc_x, sensor_data->acc_y, sensor_data->acc_z };
    FusionVector  magnetometer  = { sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z };
    //
    // Apply calibration
    gyroscope     = FusionCalibrationInertial( gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset );
    accelerometer = FusionCalibrationInertial( accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset );
    magnetometer  = FusionCalibrationMagnetic( magnetometer, softIronMatrix, hardIronOffset );

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate( &offset, gyroscope );

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static unsigned int previousTimestamp = timestamp;
    const float         deltaTime         = ( float )( timestamp - previousTimestamp ) / ( float )1000;
    if ( 0 == deltaTime )
        return;
    // TODO:previousTimestamp        = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime );

    // Print algorithm outputs
    const auto&        quate = FusionAhrsGetQuaternion( &ahrs );
    const FusionEuler  euler = FusionQuaternionToEuler( quate );
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs );
    //
    sensor_data->quate_x = quate.element.x;
    sensor_data->quate_y = quate.element.y;
    sensor_data->quate_z = quate.element.z;
    sensor_data->quate_w = quate.element.w;
    //
    sensor_data->roll  = euler.angle.roll;
    sensor_data->pitch = euler.angle.pitch;
    sensor_data->yaw   = euler.angle.yaw;

    sensor_data->eacc_x = earth.axis.x;
    sensor_data->eacc_y = earth.axis.y;
    sensor_data->eacc_z = earth.axis.z;

    static float previous_eacc_x = 0.0f;
    static float previous_eacc_y = 0.0f;
    static float previous_eacc_z = 0.0f;
    static float previous_vel_x  = 0.0f;
    static float previous_vel_y  = 0.0f;
    static float previous_vel_z  = 0.0f;
    static float previous_pos_x  = 0.0f;
    static float previous_pos_y  = 0.0f;
    static float previous_pos_z  = 0.0f;
    const float  kG              = 9.80665f;

    sensor_data->eacc_x = earth.axis.x;
    sensor_data->eacc_y = earth.axis.y;
    sensor_data->eacc_z = earth.axis.z;

    // 应用低通滤波减少噪声
    const float alpha   = 0.1f;
    sensor_data->eacc_x = alpha * sensor_data->eacc_x + ( 1.0f - alpha ) * previous_eacc_x;
    sensor_data->eacc_y = alpha * sensor_data->eacc_y + ( 1.0f - alpha ) * previous_eacc_y;
    sensor_data->eacc_z = alpha * sensor_data->eacc_z + ( 1.0f - alpha ) * previous_eacc_z;

    // 零速检测（静止时强制速度为零）
    const float stationary_threshold_x = 0.02f;
    const float stationary_threshold_y = 0.02f;
    const float stationary_threshold_z = 0.02f;
    bool is_stationary = ( fabs( sensor_data->eacc_x ) < stationary_threshold_x && fabs( sensor_data->eacc_y ) < stationary_threshold_y && fabs( sensor_data->eacc_z ) < stationary_threshold_z );
    if ( is_stationary )
    {
        sensor_data->eacc_x = 0.0f;
        sensor_data->eacc_y = 0.0f;
        sensor_data->eacc_z = 0.0f;

        previous_eacc_x = 0.0f;
        previous_eacc_y = 0.0f;
        previous_eacc_z = 0.0f;

        previous_vel_x  = 0.0f;
        previous_vel_y  = 0.0f;
        previous_vel_z  = 0.0f;
        printf( "Stationary detected, setting acceleration to zero.\n" );
    }

    // 使用梯形积分法计算速度（单位：m/s）
    sensor_data->vel_x = previous_vel_x + 0.5f * ( sensor_data->eacc_x + previous_eacc_x ) * kG * deltaTime;
    sensor_data->vel_y = previous_vel_y + 0.5f * ( sensor_data->eacc_y + previous_eacc_y ) * kG * deltaTime;
    sensor_data->vel_z = previous_vel_z + 0.5f * ( sensor_data->eacc_z + previous_eacc_z ) * kG * deltaTime;

    sensor_data->pos_x = previous_pos_x + 0.5 * ( sensor_data->vel_x + previous_vel_x ) * deltaTime;
    sensor_data->pos_y = previous_pos_y + 0.5 * ( sensor_data->vel_y + previous_vel_y ) * deltaTime;
    sensor_data->pos_z = previous_pos_z + 0.5 * ( sensor_data->vel_z + previous_vel_z ) * deltaTime;

    previous_pos_x = sensor_data->pos_x;
    previous_pos_y = sensor_data->pos_y;
    previous_pos_z = sensor_data->pos_z;

    previous_vel_x = sensor_data->vel_x;
    previous_vel_y = sensor_data->vel_y;
    previous_vel_z = sensor_data->vel_z;

    previous_eacc_x = sensor_data->eacc_x;
    previous_eacc_y = sensor_data->eacc_y;
    previous_eacc_z = sensor_data->eacc_z;

    previousTimestamp = timestamp;
}