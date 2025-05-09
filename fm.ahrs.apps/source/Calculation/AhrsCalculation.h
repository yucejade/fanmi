
//
#pragma once
//
#include "Fusion/Fusion.h"
#include "Urho3D/Core/Object.h"
#include "concurrentqueue/concurrentqueue.h"
#include <Urho3D/Core/CoreEvents.h>
#include <cctype>
#include <iostream>
#include <list>
#include <string>
//
#define SAMPLE_RATE ( 100 )  // replace this with actual sample rate
//
//
struct SENSOR_DB
{
    unsigned int time;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float quate_x;
    float quate_y;
    float quate_z;
    float quate_w;
    float roll;
    float pitch;
    float yaw;
    float eacc_x;
    float eacc_y;
    float eacc_z;
    float eacc_bias_x;
    float eacc_bias_y;
    float eacc_bias_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float pos_x;
    float pos_y;
    float pos_z;

    std::string info;
};

// 从加速度计算位移的函数
struct MotionData
{
    std::vector< float > time;
    std::vector< float > acceleration;
    std::vector< float > velocity;
    std::vector< float > displacement;
};

// 验证字符串是否为数字
static bool isNumber( const std::string& str )
{
    for ( char c : str )
    {
        if ( ! std::isdigit( c ) )
        {
            return false;
        }
    }
    return true;
}
//
using namespace Urho3D;
//
class AhrsCalculation : public Urho3D::Object
{
    URHO3D_OBJECT( AhrsCalculation, Urho3D::Object )
public:
    explicit AhrsCalculation( Context* context );
public:
    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment     = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    const FusionVector gyroscopeSensitivity      = { 1.0f, 1.0f, 1.0f };
    const FusionVector gyroscopeOffset           = { 0.0f, 0.0f, 0.0f };
    const FusionMatrix accelerometerMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    //const FusionVector accelerometerSensitivity  = { 0.101938f, 0.101715f, 0.102024f };
    const FusionVector accelerometerSensitivity  = { 1.0f, 1.0f, 1.0f };
    const FusionVector accelerometerOffset       = { -0.00494385f, -0.00185547f, 0.0249512f };
    //const FusionVector accelerometerOffset       = { 0.0f, 0.0f, 0.0f };
    const FusionMatrix softIronMatrix            = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    const FusionVector hardIronOffset            = { 0.0f, 0.0f, 0.0f };
public:
    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs   ahrs;
    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
        .convention            = FusionConventionNwu,
        .gain                  = 0.5f,
        .gyroscopeRange        = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection     = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };

public:
    void SolveAnCalculation( SENSOR_DB* sensor_data );

private:
    std::list<SENSOR_DB> recent_datas;
    int average_size = 0;
};
