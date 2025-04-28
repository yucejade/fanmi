#pragma once
//
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <stdbool.h>
#include <stdexcept>
#include <stdint.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

//
static void openI2C( int& i2cFile, std::string& i2cDevice, uint8_t& deviceAddress )
{
    i2cFile = open( i2cDevice.c_str(), O_RDWR );
    if ( i2cFile < 0 )
    {
        std::cerr << "Failed to open I2C device" << std::endl;
    }

    if ( ioctl( i2cFile, I2C_SLAVE, deviceAddress ) < 0 )
    {
        close( i2cFile );
        std::cerr << "Failed to set I2C address" << std::endl;
    }
};

static void closeI2C( int& i2cFile )
{
    if ( i2cFile >= 0 )
    {
        close( i2cFile );
        i2cFile = -1;
    }
};
// 实现I2C SMBus协议的单字节读取功能
// 参数：
//   addr - I2C从设备地址
//   reg  - 要读取的寄存器地址
// 返回值：
//   成功返回读取的字节值（0-255），失败返回-1
static int i2c_smbus_read_byte_data( int i2cFile, int reg )
{
    // 写入要读取的寄存器地址
    if ( write( i2cFile, &reg, 1 ) != 1 )
    {
        std::cerr << "Failed to write register address" << std::endl;
        return -1;
    }

    // 读取寄存器值（单字节）
    unsigned char value;
    if ( read( i2cFile, &value, 1 ) != 1 )
    {
        std::cerr << "Failed to read register value" << std::endl;
        return -1;
    }

    // 返回读取的字节值（转换为int类型）
    return static_cast< int >( value );
}
// 实现I2C块数据读取功能（多字节读取）
// 参数：
//   addr - I2C从设备地址
//   reg  - 起始寄存器地址
//   len  - 要读取的字节数
//   buf  - 数据存储缓冲区（需预先分配足够空间）
// 返回值：
//   成功返回0，失败返回-1
static int i2c_smbus_read_i2c_block_data( int i2cFile, int reg, int len, unsigned char* buf )
{
    // 写入起始寄存器地址
    if ( write( i2cFile, &reg, 1 ) != 1 )
    {
        std::cerr << "Failed to write register address" << std::endl;
        return -1;
    }

    // 读取连续字节数据（从指定寄存器开始）
    if ( read( i2cFile, buf, len ) != len )
    {
        std::cerr << "Failed to read block data" << std::endl;
        return -1;
    }
    return 0;
}
// 实现I2C SMBus协议的单字节写入功能
// 参数：
//   addr  - I2C从设备地址
//   reg   - 目标寄存器地址
//   value - 要写入的字节值
// 返回值：
//   成功返回0，失败返回-1
static int i2c_smbus_write_byte_data( int i2cFile, int reg, unsigned char value )
{
    // 构造写入缓冲区：[寄存器地址, 写入值]
    unsigned char buf[ 2 ] = {
        static_cast< unsigned char >( reg ),  // 寄存器地址转无符号
        value                                 // 待写入的字节值
    };

    // 执行I2C写入操作
    if ( write( i2cFile, buf, 2 ) != 2 )
    {
        std::cerr << "Failed to write register value" << std::endl;
        return -1;
    }
    return 0;
}
