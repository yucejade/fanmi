#include "fm_mmc5603nj.h"
#include "io/fm_i2c.h"
#include <cmath>
#include <cstring>
#include <iostream>
//

fm_mmc5603nj::fm_mmc5603nj( const std::string& i2cDevice, uint8_t deviceAddress ) : _i2cDevice( i2cDevice ), _deviceAddress( deviceAddress ), _i2cFile( -1 )
{
    _deviceAddress = deviceAddress;
    _i2cDevice     = i2cDevice;
}

fm_mmc5603nj::~fm_mmc5603nj()
{
    closeI2C( _i2cFile );
}

void fm_mmc5603nj::begin()
{
    openI2C( _i2cFile, _i2cDevice, _deviceAddress );
}

float fm_mmc5603nj::getMilliGaussX( void )
{
    int32_t mag = 0;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_XOUT0 ) << 12;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_XOUT1 ) << 4;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_XOUT2 ) << 0;
    return 0.0625 * ( mag - 524288 );
}

float fm_mmc5603nj::getMilliGaussY( void )
{
    int32_t mag = 0;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_YOUT0 ) << 12;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_YOUT1 ) << 4;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_YOUT2 ) << 0;
    return 0.0625 * ( mag - 524288 );
}

float fm_mmc5603nj::getMilliGaussZ( void )
{
    int32_t mag = 0;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_ZOUT0 ) << 12;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_ZOUT1 ) << 4;
    mag |= i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_ZOUT2 ) << 0;
    return 0.0625 * ( mag - 524288 );
}

void fm_mmc5603nj::getMilliGauss( float* magX, float* magY, float* magZ, float* magAbs )
{
    uint8_t mag[ 9 ] = { 0 };
    i2c_smbus_read_i2c_block_data( _i2cFile, MMC5603NJ_ADDR_XOUT0, 9, mag );
    // 计算磁场强度
    *magX   = ( ( mag[ 0 ] << 12 | mag[ 1 ] << 4 | mag[ 6 ] ) - 524288 ) * 0.0625;
    *magY   = ( ( mag[ 2 ] << 12 | mag[ 3 ] << 4 | mag[ 7 ] ) - 524288 ) * 0.0625;
    *magZ   = ( ( mag[ 4 ] << 12 | mag[ 5 ] << 4 | mag[ 8 ] ) - 524288 ) * 0.0625;
    *magAbs = std::sqrt( std::pow( *magX, 2 ) + std::pow( *magY, 2 ) + std::pow( *magZ, 2 ) );
}

void fm_mmc5603nj::setContinuousMode( uint8_t odr )
{
    i2c_smbus_write_byte_data( _i2cFile, MMC5603NJ_ADDR_ODR, odr );
    i2c_smbus_write_byte_data( _i2cFile, MMC5603NJ_ADDR_INTCTRL0, 0b10100000 );
    i2c_smbus_write_byte_data( _i2cFile, MMC5603NJ_ADDR_INTCTRL1, 0b00000011 );
    i2c_smbus_write_byte_data( _i2cFile, MMC5603NJ_ADDR_INTCTRL2, 0b00010000 );
}

void fm_mmc5603nj::clearContinuousMode( void )
{
    i2c_smbus_write_byte_data( _i2cFile, MMC5603NJ_ADDR_INTCTRL2, 0b00000000 );
}

uint8_t fm_mmc5603nj::readProductId( void )
{
    return i2c_smbus_read_byte_data( _i2cFile, MMC5603NJ_ADDR_PRODUCTID );
}

void fm_mmc5603nj::softwareReset()
{
    i2c_smbus_write_byte_data( _i2cFile, MMC5603NJ_ADDR_INTCTRL1, 0x80 );
}
