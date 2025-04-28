#pragma once

#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define MMC56X3_DEFAULT_ADDRESS 0x30  //!< Default address
#define MMC56X3_CHIP_ID         0x10  //!< Chip ID from WHO_AM_I register

/*=========================================================================*/

/*!
 * @brief MMC56X3 I2C register address bits
 */
typedef enum
{
    MMC56X3_PRODUCT_ID = 0x39,
    MMC56X3_CTRL0_REG  = 0x1B,
    MMC56X3_CTRL1_REG  = 0x1C,
    MMC56X3_CTRL2_REG  = 0x1D,
    MMC56X3_STATUS_REG = 0x18,
    MMC56X3_OUT_TEMP   = 0x09,
    MMC56X3_OUT_X_L    = 0x00,
    MMC5603_ODR_REG    = 0x1A,

} mmc56x3_register_t;
/*=========================================================================*/

class MMC56x3
{
public:
    MMC56x3( int32_t sensorID = -1 );

    bool begin( uint8_t i2c_addr = MMC56X3_DEFAULT_ADDRESS, const char* i2c_device = "/dev/i2c-1" );

    bool getEvent( float& x, float& y, float& z );
    void getSensor( const char*& name, int& version, int& sensor_id, int& type, int& min_delay, float& max_value, float& min_value, float& resolution );

    void reset( void );
    void magnetSetReset( void );

    void setContinuousMode( bool mode );
    bool isContinuousMode( void );

    float readTemperature( void );

    uint16_t getDataRate();
    void     setDataRate( uint16_t rate );
private:
    int      _i2c_fd;
    uint16_t _odr_cache   = 0;
    uint8_t  _ctrl2_cache = 0;

    int32_t _x;  ///< x-axis raw data
    int32_t _y;  ///< y-axis raw data
    int32_t _z;  ///< z-axis raw data

    int32_t _sensorID;

    uint8_t readRegister( uint8_t reg );
    void    writeRegister( uint8_t reg, uint8_t value );
};
