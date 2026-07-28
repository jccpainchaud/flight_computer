#ifndef SRC_ICM_20948_H_
#define SRC_ICM_20948_H_

#include "main.h"

typedef struct {
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
	int16_t x_magnet;
	int16_t y_magnet;
	int16_t z_magnet;
} icm_20948_data;

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

#define IMU_SPI             hspi1
#define IMU_CS_PORT         SPI1_CS_GPIO_Port
#define IMU_CS_PIN          SPI1_CS_Pin
#define GYRO_RANGE_VALUE    _gyro_250dps
#define ACCEL_RANGE_VALUE   _accel_2g

#define REG_BANK_SEL         0x7f

#define WHO_AM_I             0x00
#define USER_CTRL            0x03
#define LP_CONFIG            0x05
#define PWR_MGMT_1           0x06
#define ODR_ALIGN_EN         0x09

#define I2C_MST_CTRL         0x01
#define I2C_MST_ODR_CONFIG   0x00
#define I2C_SLV0_ADDR        0x03
#define I2C_SLV0_REG         0x04
#define I2C_SLV0_CTRL        0x05
#define I2C_SLV0_DO          0x06

#define AK_09916_ADDRESS     0x0c
#define MAG_CTRL2            0x31
#define MAG_CTRL3            0x32
#define MAG_DATA_ONSET       0x11

#define ACCEL_SMPLRT_DIV_1   0x10
#define ACCEL_SMPLRT_DIV_2   0x11
#define ACCEL_CONFIG_1       0x14

#define GYRO_SMPLRT_DIV      0x00
#define GYRO_CONFIG_1        0x01
#define GYRO_CONFIG_2        0x02

#define ACCEL_XOUT_H         0x2d
#define ACCEL_XOUT_L         0x2e
#define ACCEL_YOUT_H         0x2f
#define ACCEL_YOUT_L         0x30
#define ACCEL_ZOUT_H         0x31
#define ACCEL_ZOUT_L         0x32

#define GYRO_XOUT_H          0x33
#define GYRO_XOUT_L          0x34
#define GYRO_YOUT_H          0x35
#define GYRO_YOUT_L          0x36
#define GYRO_ZOUT_H          0x37
#define GYRO_ZOUT_L          0x38

#define XA_OFFS_H            0x14
#define XA_OFFS_L            0x15
#define YA_OFFS_H            0x17
#define YA_OFFS_L            0x18
#define ZA_OFFS_H            0x1a
#define ZA_OFFS_L            0x1b

#define XG_OFFS_USRH         0x03
#define XG_OFFS_USRL         0x04
#define YG_OFFS_USRH         0x05
#define YG_OFFS_USRL         0x06
#define ZG_OFFS_USRH         0x07
#define ZG_OFFS_USRL         0x08

#define MAG_X_BIAS           15
#define MAG_Y_BIAS           100
#define MAG_Z_BIAS           -30

typedef enum {
	_gyro_250dps,
	_gyro_500dps,
	_gyro_1000dps,
	_gyro_2000dps
} gyro_range;

typedef enum {
	_accel_2g,
	_accel_4g,
	_accel_8g,
	_accel_16g
} accel_range;

typedef enum {
	_bank0 = 0,
	_bank1 = 1 << 4,
	_bank2 = 2 << 4,
	_bank3 = 3 << 4
} user_bank;

void icm_20948_init();
void icm_20948_read_reg(user_bank ub, uint8_t address, uint8_t *data);
void icm_20948_write_reg(user_bank ub, uint8_t reg, uint8_t data);
void icm_20948_read_data(icm_20948_data *data);

#endif /* SRC_ICM_20948_H_ */
