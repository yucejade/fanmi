#include "AhrsCalculation.h"
#include <Urho3D/IO/Log.h>
#include <cstdio>
#include <time.h>
//
// 带 Context* 参数的构造函数实现
AhrsCalculation::AhrsCalculation( Urho3D::Context* context ) : Urho3D::Object( context )
{
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
    static clock_t previousTimestamp;
    const float    deltaTime = ( float )( timestamp - previousTimestamp ) / ( float )CLOCKS_PER_SEC;
    previousTimestamp        = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime );

    // Print algorithm outputs
    auto               quate = FusionAhrsGetQuaternion( &ahrs );
    const FusionEuler  euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion( &ahrs ) );
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
    //
    sensor_data->eacc_x = earth.axis.x;
    sensor_data->eacc_y = earth.axis.y;
    sensor_data->eacc_z = earth.axis.z;

    //
}

std::vector< float > AhrsCalculation::Integrate( const std::vector< float >& f, const std::vector< float >& t, float initial )
{
    std::vector< float > result( f.size() );
    result[ 0 ] = initial;

    for ( size_t i = 1; i < f.size(); ++i )
    {
        float dt   = t[ i ] - t[ i - 1 ];
        result[ i ] = result[ i - 1 ] + 0.5 * ( f[ i ] + f[ i - 1 ] ) * dt;
    }

    return result;
}

MotionData AhrsCalculation::AccelerationToDisplacement( const std::function< float( float ) >& a_func, float t_start, float t_end, size_t num_points, float v0, float s0 )
{
    MotionData data;

    // 生成时间数组
    data.time.resize( num_points );
    float dt = ( t_end - t_start ) / ( num_points - 1 );
    for ( size_t i = 0; i < num_points; ++i )
    {
        data.time[ i ] = t_start + i * dt;
    }

    // 计算加速度数组
    data.acceleration.resize( num_points );
    for ( size_t i = 0; i < num_points; ++i )
    {
        data.acceleration[ i ] = a_func( data.time[ i ] );
    }

    // 计算速度
    data.velocity = Integrate( data.acceleration, data.time, v0 );

    // 计算位移
    data.displacement = Integrate( data.velocity, data.time, s0 );

    return data;
}