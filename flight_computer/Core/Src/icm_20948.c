#include "icm_20948.h"
#include "main.h"
#include "stdio.h"

static void activate_imu() {
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); // CS pin pulled low to signal start of communication
}

static void deactivate_imu() {
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET); // CS pin pulled high to signal end of communication
}

static void sel_user_bank(user_bank ub) {
	uint8_t data = ub;
	uint8_t reg = REG_BANK_SEL;

	activate_imu();
	HAL_SPI_Transmit(&IMU_SPI, &reg, 1, 100);
	HAL_SPI_Transmit(&IMU_SPI, &data, 1, 100);
	deactivate_imu();
}

void remove_gyro_bias() {
	uint8_t samples = 200;
	icm_20948_data data;

	int16_t x_gyro_bias, y_gyro_bias, z_gyro_bias;
	int32_t x_bias = 0;
	int32_t y_bias = 0;
	int32_t z_bias = 0;

	sel_user_bank(_bank0);

	for (int i = 0; i < samples; i++) {
		icm_20948_read_data(&data);

		x_bias += data.x_gyro;
		y_bias += data.y_gyro;
		z_bias += data.z_gyro;

		HAL_Delay(2); // To get more varied and spread-out values
	}

	x_gyro_bias = -(int16_t)(x_bias / (samples * 4)); // Magic number
	y_gyro_bias = -(int16_t)(y_bias / (samples * 4));
	z_gyro_bias = -(int16_t)(z_bias / (samples * 4));

	icm_20948_write_reg(_bank2, XG_OFFS_USRH, (uint8_t)(x_gyro_bias >> 8));
	icm_20948_write_reg(_bank2, XG_OFFS_USRL, (uint8_t)(x_gyro_bias));
	icm_20948_write_reg(_bank2, YG_OFFS_USRH, (uint8_t)(y_gyro_bias >> 8));
	icm_20948_write_reg(_bank2, YG_OFFS_USRL, (uint8_t)(y_gyro_bias));
	icm_20948_write_reg(_bank2, ZG_OFFS_USRH, (uint8_t)(z_gyro_bias >> 8));
	icm_20948_write_reg(_bank2, ZG_OFFS_USRL, (uint8_t)(z_gyro_bias));
}

void icm_20948_write_reg(user_bank ub, uint8_t reg, uint8_t data) {
	sel_user_bank(ub);

	activate_imu();
	HAL_SPI_Transmit(&IMU_SPI, &reg, 1, 100);
	HAL_SPI_Transmit(&IMU_SPI, &data, 1, 100);
	deactivate_imu();
}

void icm_20948_read_reg(user_bank ub, uint8_t address, uint8_t *data) {
	sel_user_bank(ub);
	uint8_t temp_data = 0x80 | address; // MSB is R/W bit, read signaled by 1

	activate_imu();
	HAL_SPI_Transmit(&IMU_SPI, &temp_data, 1, 100);
	HAL_SPI_Receive(&IMU_SPI, data, 1, 100);
	deactivate_imu();
}

void icm_20948_read_data(icm_20948_data *data) {
	uint8_t rx_data[22];
	uint8_t temp_data = 0x80 | ACCEL_XOUT_H; // 0x80 is write instruction

	activate_imu();

	HAL_SPI_Transmit(&IMU_SPI, &temp_data, 1, 100);
	HAL_SPI_Receive(&IMU_SPI, rx_data, 22, 100);

	data -> x_accel = (int16_t) (rx_data[0] << 8) | rx_data[1];
	data -> y_accel = (int16_t) (rx_data[2] << 8) | rx_data[3];
	data -> z_accel = (int16_t) (rx_data[4] << 8) | rx_data[5];

	data -> x_gyro = (int16_t) (rx_data[6] << 8)  | rx_data[7];
	data -> y_gyro = (int16_t) (rx_data[8] << 8)  | rx_data[9];
	data -> z_gyro = (int16_t) (rx_data[10] << 8) | rx_data[11];

	data -> x_magnet = (int16_t) (rx_data[15] << 8 | rx_data[14]);
	data -> y_magnet = (int16_t) (rx_data[17] << 8 | rx_data[16]);
	data -> z_magnet = (int16_t) (rx_data[19] << 8 | rx_data[18]);

	deactivate_imu();
}

static void ak_09916_write_reg(uint8_t reg, uint8_t data) {
	icm_20948_write_reg(_bank3, I2C_SLV0_ADDR, AK_09916_ADDRESS);
	icm_20948_write_reg(_bank3, I2C_SLV0_REG, reg);
	icm_20948_write_reg(_bank3, I2C_SLV0_DO, data);
	icm_20948_write_reg(_bank3, I2C_SLV0_CTRL, 0x80 | 0x01);
	HAL_Delay(50);
}

static void ak_09916_read_reg(uint8_t onset_reg, uint8_t len) {
	icm_20948_write_reg(_bank3, I2C_SLV0_ADDR, 0x80 | AK_09916_ADDRESS);
	icm_20948_write_reg(_bank3, I2C_SLV0_REG, onset_reg);
	icm_20948_write_reg(_bank3, I2C_SLV0_CTRL, 0x80 | len);
	HAL_Delay(50);
}

static void ak_09916_init() {
	uint8_t temp_data;

	// I2C controller reset, page 36
	icm_20948_read_reg(_bank0, USER_CTRL, &temp_data);
	temp_data |= 0x02;
	icm_20948_write_reg(_bank0, USER_CTRL, temp_data);
	HAL_Delay(100); // I2C reset needs to wait a clock cycle

	// I2C controller enable, page 36
	icm_20948_read_reg(_bank0, USER_CTRL, &temp_data);
	temp_data |= 0x20;
	icm_20948_write_reg(_bank0, USER_CTRL, temp_data);
	HAL_Delay(10);

	// I2C clock frequency recommendation as per data sheet, page 68
	temp_data = 0x07;
	icm_20948_write_reg(_bank3, I2C_MST_CTRL, temp_data);
	HAL_Delay(10);

	// LP_CONFIG: ODR is determined by I2C_MST_ODR_CONFIG register, page 37
	temp_data = 0x40;
	icm_20948_write_reg(_bank0, LP_CONFIG, temp_data);
	HAL_Delay(10);

	// 1.1 kHz / (2^3) = 136 Hz, page 68
	temp_data = 0x03;
	icm_20948_write_reg(_bank3, I2C_MST_ODR_CONFIG, temp_data);
	HAL_Delay(10);

	// Magnetometer reset, page 80
	ak_09916_write_reg(MAG_CTRL3, 0x01);
	HAL_Delay(100);

	// Continuous measurement mode 4: 100Hz, page 79
	ak_09916_write_reg(MAG_CTRL2, 0x08);

}

void icm_20948_init() {
	uint8_t temp_data;

	// Resets device and selects best available clock, page 37
	icm_20948_write_reg(_bank0, PWR_MGMT_1, 0x81);
	HAL_Delay(100); // The IMU needs time after resetting, as per the datasheet

	// Exits sleep mode, page 37
	icm_20948_write_reg(_bank0, PWR_MGMT_1, 0x01);

	// Enables ODR, page 63
	icm_20948_write_reg(_bank2, ODR_ALIGN_EN, 0x01);

	// Set gyroscope sample rate divider to 0, page 59
	icm_20948_write_reg(_bank2, GYRO_SMPLRT_DIV, 0x00);

	// Set gyro range and enable DLPF, page 59
	icm_20948_write_reg(_bank2, GYRO_CONFIG_1, (GYRO_RANGE_VALUE << 1) | 0x01);

	// Set accelerometer sample rate divider to 0, page 63
	icm_20948_write_reg(_bank2, ACCEL_SMPLRT_DIV_1, 0x00);
	icm_20948_write_reg(_bank2, ACCEL_SMPLRT_DIV_2, 0x00);

	// Set accelerometer range and enable DLPF, page 64
	icm_20948_write_reg(_bank2, ACCEL_CONFIG_1, (ACCEL_RANGE_VALUE << 1) | 0x01);

	// Put the serial interface in SPI mode only, page 36
	icm_20948_read_reg(_bank0, USER_CTRL, &temp_data);
	temp_data |= 0x10;
	icm_20948_write_reg(_bank0, USER_CTRL, temp_data);

	ak_09916_init();
	ak_09916_read_reg(MAG_DATA_ONSET, 8);

	remove_gyro_bias();

	sel_user_bank(_bank0); // Bank 0 to enable values
}
