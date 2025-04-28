#include "MMC56x3.h"
#include <unistd.h>
#include <iostream>

MMC56x3::MMC56x3(int32_t sensorID) {
  _sensorID = sensorID;

  // Clear the raw mag data
  _x = 0;
  _y = 0;
  _z = 0;
}

bool MMC56x3::begin(uint8_t i2c_address, const char* i2c_device) {
  _i2c_fd = open(i2c_device, O_RDWR);
  if (_i2c_fd < 0) {
    std::cerr << "Failed to open I2C device" << std::endl;
    return false;
  }

  if (ioctl(_i2c_fd, I2C_SLAVE, i2c_address) < 0) {
    std::cerr << "Failed to set I2C address" << std::endl;
    close(_i2c_fd);
    return false;
  }

  // Check connection
  uint8_t id = readRegister(MMC56X3_PRODUCT_ID);
  if ((id != MMC56X3_CHIP_ID) && (id != 0x0)) {
    // No MMC56X3 detected ... return false
    std::cerr << "MMC56X3 not detected" << std::endl;
    close(_i2c_fd);
    return false;
  }

  reset();

  return true;
}

void MMC56x3::reset(void) {
  writeRegister(MMC56X3_CTRL1_REG, 0x80); // write only, set topmost bit
  usleep(20000);
  _odr_cache = 0;
  _ctrl2_cache = 0;
  magnetSetReset();
  setContinuousMode(false);
}

void MMC56x3::magnetSetReset(void) {
  writeRegister(MMC56X3_CTRL0_REG, 0x08); // turn on set bit
  usleep(1000);
  writeRegister(MMC56X3_CTRL0_REG, 0x10); // turn on reset bit
  usleep(1000);
}

void MMC56x3::setContinuousMode(bool mode) {
  if (mode) {
    writeRegister(MMC56X3_CTRL0_REG, 0x80); // turn on cmm_freq_en bit
    _ctrl2_cache |= 0x10;    // turn on cmm_en bit
  } else {
    _ctrl2_cache &= ~0x10; // turn off cmm_en bit
  }
  writeRegister(MMC56X3_CTRL2_REG, _ctrl2_cache);
}

bool MMC56x3::isContinuousMode(void) { return _ctrl2_cache & 0x10; }

float MMC56x3::readTemperature(void) {
  if (isContinuousMode())
    return NAN;

  writeRegister(MMC56X3_CTRL0_REG, 0x02); // TM_T trigger

  uint8_t status;
  status = readRegister(MMC56X3_STATUS_REG);
  if (!(status & 0x80))
    return false;

  float temp = readRegister(MMC56X3_OUT_TEMP);
  temp *= 0.8; //  0.8*C / LSB
  temp -= 75;  //  0 value is -75

  return temp;
}

bool MMC56x3::getEvent(float& x, float& y, float& z) {
  if (!isContinuousMode()) {
    writeRegister(MMC56X3_CTRL0_REG, 0x01); // TM_M trigger

    uint8_t status;
    status = readRegister(MMC56X3_STATUS_REG);
    if (!(status & 0x40))
      return false;
  }

  uint8_t buffer[9];
  buffer[0] = MMC56X3_OUT_X_L;
  if (write(_i2c_fd, buffer, 1) != 1) {
    std::cerr << "Failed to write to I2C device" << std::endl;
    return false;
  }
  if (read(_i2c_fd, buffer, 9) != 9) {
    std::cerr << "Failed to read from I2C device" << std::endl;
    return false;
  }

  _x = (uint32_t)buffer[0] << 12 | (uint32_t)buffer[1] << 4 | (uint32_t)buffer[6] >> 4;
  _y = (uint32_t)buffer[2] << 12 | (uint32_t)buffer[3] << 4 | (uint32_t)buffer[7] >> 4;
  _z = (uint32_t)buffer[4] << 12 | (uint32_t)buffer[5] << 4 | (uint32_t)buffer[8] >> 4;
  // fix center offsets
  _x -= (uint32_t)1 << 19;
  _y -= (uint32_t)1 << 19;
  _z -= (uint32_t)1 << 19;

  x = (float)_x * 0.00625; // scale to uT by LSB in datasheet
  y = (float)_y * 0.00625;
  z = (float)_z * 0.00625;

  return true;
}

void MMC56x3::setDataRate(uint16_t rate) {
  // only 0~255 and 1000 are valid, so just move any high rates to 1000
  if (rate > 255)
    rate = 1000;
  _odr_cache = rate;

  if (rate == 1000) {
    writeRegister(MMC5603_ODR_REG, 255);
    _ctrl2_cache |= 0x80; // turn on hpower bit
    writeRegister(MMC56X3_CTRL2_REG, _ctrl2_cache);
  } else {
    writeRegister(MMC5603_ODR_REG, rate);
    _ctrl2_cache &= ~0x80; // turn off hpower bit
    writeRegister(MMC56X3_CTRL2_REG, _ctrl2_cache);
  }
}

uint16_t MMC56x3::getDataRate(void) { return _ctrl2_cache; }

void MMC56x3::getSensor(const char*& name, int& version, int& sensor_id, int& type, int& min_delay, float& max_value, float& min_value, float& resolution) {
  name = "MMC5603";
  version = 1;
  sensor_id = _sensorID;
  type = 1; // SENSOR_TYPE_MAGNETIC_FIELD
  min_delay = 0;
  max_value = 3000;     // 30 gauss = 3000 uTesla
  min_value = -3000;    // -30 gauss = -3000 uTesla
  resolution = 0.00625; // 20 bit 0.00625 uT LSB
}

uint8_t MMC56x3::readRegister(uint8_t reg) {
  if (write(_i2c_fd, &reg, 1) != 1) {
    std::cerr << "Failed to write register address" << std::endl;
    return 0;
  }
  uint8_t value;
  if (read(_i2c_fd, &value, 1) != 1) {
    std::cerr << "Failed to read register value" << std::endl;
    return 0;
  }
  return value;
}

void MMC56x3::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  if (write(_i2c_fd, buffer, 2) != 2) {
    std::cerr << "Failed to write register" << std::endl;
  }
}