#include "bmp_280.h"

static bmp_280_calib_t calib;

static void activate_baro() {
	HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_RESET); // CS pin pulled low to signal start of communication
}

static void deactivate_baro() {
	HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET); // CS pin pulled high to signal end of communication
}

void bmp_280_read_reg(uint8_t address, uint8_t *data) {
	uint8_t temp_data = 0x80 | address;

	activate_baro();
	HAL_SPI_Transmit(&BARO_SPI, &temp_data, 1, 100);
	HAL_SPI_Receive(&BARO_SPI, data, 1, 100);
	deactivate_baro();
}

void bmp_280_write_reg(uint8_t reg, uint8_t data) {
	activate_baro();
	HAL_SPI_Transmit(&BARO_SPI, &reg, 1, 100);
	HAL_SPI_Transmit(&BARO_SPI, &data, 1, 100);
	deactivate_baro();
}

static void bmp_280_read_calib_values() {
	uint8_t temp_data = 0x80 | 0x88; // 0x88 is dig_T1 LSB
	uint8_t buffer[24];

	activate_baro();

	HAL_SPI_Transmit(&BARO_SPI, &temp_data, 1, 100);
	HAL_SPI_Receive(&BARO_SPI, buffer, 24, 100);

	calib.dig_T1 = (uint16_t) (buffer[1] << 8) | (buffer[0]);
	calib.dig_T2 = (int16_t)  (buffer[3] << 8) | (buffer[2]);
	calib.dig_T3 = (int16_t)  (buffer[5] << 8) | (buffer[4]);

	calib.dig_P1 = (uint16_t) (buffer[7] << 8) | (buffer[6]);
	calib.dig_P2 = (int16_t)  (buffer[9] << 8) | (buffer[8]);
	calib.dig_P3 = (int16_t) (buffer[11] << 8) | (buffer[10]);
	calib.dig_P4 = (int16_t) (buffer[13] << 8) | (buffer[12]);
	calib.dig_P5 = (int16_t) (buffer[15] << 8) | (buffer[14]);
	calib.dig_P6 = (int16_t) (buffer[17] << 8) | (buffer[16]);
	calib.dig_P7 = (int16_t) (buffer[19] << 8) | (buffer[18]);
	calib.dig_P8 = (int16_t) (buffer[21] << 8) | (buffer[20]);
	calib.dig_P9 = (int16_t) (buffer[23] << 8) | (buffer[22]);

	deactivate_baro();
}

void bmp_280_init() {
	uint8_t temp_data;

	temp_data = (PRESS_OVERSAMPLING << 5) | (TEMP_OVERSAMPLING << 2) | POWER_MODE;
	bmp_280_write_reg(CTRL_MEAS, temp_data);

	temp_data = (STANDBY_TIME_MS << 5) | (FILTER_COEFF << 2);
	bmp_280_write_reg(CONFIG, temp_data);

	bmp_280_read_calib_values();
}



