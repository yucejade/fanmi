
#ifndef ICM42670_H
#define ICM42670_H

#include <thread>
extern "C" {
#include "imu/inv_imu_driver.h"
#include <stddef.h>
#undef ICM42670
}
//
#define ICM42670_I2C_ADDRESS 0x69
// This defines the handler called when retrieving a sample from the FIFO
typedef void ( *ICM42670_sensor_event_cb )( inv_imu_sensor_event_t* event );
// This defines the handler called when receiving an irq
typedef void ( *ICM42670_irq_handler )( void );

class ICM42670
{
public:
    ICM42670();
    ~ICM42670();

    int  begin( bool type = false, uint8_t i2c_addr = ICM42670_I2C_ADDRESS, const char* i2c_device = "/dev/i2c-1" );
    int  startAccel( uint16_t odr, uint16_t fsr );
    int  startGyro( uint16_t odr, uint16_t fsr );
    int  getDataFromRegisters( inv_imu_sensor_event_t& evt );
    int  enableFifoInterrupt( uint8_t intpin, ICM42670_irq_handler handler, uint8_t fifo_watermark );
    int  getDataFromFifo( ICM42670_sensor_event_cb event_cb );
    bool isAccelDataValid( inv_imu_sensor_event_t* evt );
    bool isGyroDataValid( inv_imu_sensor_event_t* evt );
    int  startTiltDetection( uint8_t intpin = 2, ICM42670_irq_handler handler = NULL );
    bool getTilt( void );
    int  startPedometer( uint8_t intpin = 2, ICM42670_irq_handler handler = NULL );
    int  getPedometer( uint32_t& step_count, float& step_cadence, const char*& activity );
    int  startWakeOnMotion( uint8_t intpin, ICM42670_irq_handler handler );
    int  updateApex( void );
    void enableInterrupt( uint8_t intpin, ICM42670_irq_handler handler );
    int  run_self_test( void );

    bool    which_use   = false;  // false:i2c，true:spi
    uint8_t i2c_address = 0;
    int     i2c_fd      = -1;
    int     spi_fd      = -1;  // TODO:未实现
    // 设置总线时钟频率不是所有平台都支持，该变量预留，暂不实现
    uint32_t                              clk_freq    = 0;
    struct gpiod_chip*                    gpio_chip   = NULL;
    struct gpiod_line*                    int_line    = NULL;
    uint8_t                               int_status3 = 0;
    std::thread                           monitor;
    std::chrono::steady_clock::time_point last_interrupt_time;
protected:
    struct inv_imu_device         icm_driver;
    inv_imu_interrupt_parameter_t int1_config;
    bool                          use_spi;
    bool                          open_i2c_device( const char* i2c_device, uint8_t address, uint32_t freq );
    void                          close_i2c_device();
    bool                          open_spi_device( const char* spi_device, uint8_t mode, uint32_t speed );
    void                          close_spi_device();
    ACCEL_CONFIG0_ODR_t           accel_freq_to_param( uint16_t accel_freq_hz );
    GYRO_CONFIG0_ODR_t            gyro_freq_to_param( uint16_t gyro_freq_hz );
    ACCEL_CONFIG0_FS_SEL_t        accel_fsr_g_to_param( uint16_t accel_fsr_g );
    GYRO_CONFIG0_FS_SEL_t         gyro_fsr_dps_to_param( uint16_t gyro_fsr_dps );
    int                           initApex( uint8_t intpin, ICM42670_irq_handler handler );
    uint32_t                      step_cnt_ovflw;
    bool                          apex_tilt_enable;
    bool                          apex_pedometer_enable;
};

#endif  // ICM42670_H
