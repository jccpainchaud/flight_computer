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

	deactivate_baro();

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
}

void bmp_280_init() {
	uint8_t temp_data;

	bmp_280_read_calib_values();

	temp_data = (STANDBY_TIME_MS << 5) | (FILTER_COEFF << 2);
	bmp_280_write_reg(CONFIG, temp_data);

	temp_data = (PRESS_OVERSAMPLING << 5) | (TEMP_OVERSAMPLING << 2) | POWER_MODE;
	bmp_280_write_reg(CTRL_MEAS, temp_data);

	HAL_Delay(10);

}

void bmp_280_read_data(bmp_280_data_t *data) {
	uint8_t buffer[6];
	uint8_t temp_data = 0x80 | PRESS_MSB;

	activate_baro();

	HAL_SPI_Transmit(&BARO_SPI, &temp_data, 1, 100);
	HAL_SPI_Receive(&BARO_SPI, buffer, 6, 100);

	deactivate_baro();

	// Raw pressure and temperature values
	int32_t adc_P = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | ((int32_t)buffer[2] >> 4);
	int32_t adc_T =  ((int32_t)buffer[3] << 12) | ((int32_t)buffer[4] << 4) | ((int32_t)buffer[5] >> 4);

	// Temperature compensation
	int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
	int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
	int32_t t_fine = var1 + var2;

	int32_t temp = (t_fine * 5 + 128) >> 8;

	// Pressure compensation
	int64_t p_var1;
	int64_t p_var2;
	int64_t pressure;

	p_var1 = ((int64_t)t_fine) - 128000;
	p_var2 = p_var1 * p_var1 * (int64_t)calib.dig_P6;
	p_var2 += (p_var1 * (int64_t)calib.dig_P5) << 17;
	p_var2 += ((int64_t)calib.dig_P4) << 35;

	p_var1 = ((p_var1 * p_var1 * (int64_t)calib.dig_P3) >> 8) + ((p_var1 * (int64_t)calib.dig_P2) << 12);
	p_var1 = (((((int64_t)1) << 47) + p_var1) * (int64_t)calib.dig_P1) >> 33;

	if (p_var1 == 0) { // Avoid division by zero
		pressure = 0;
	} else {
		pressure = 1048576 - adc_P;
		pressure = (((pressure << 31) - p_var2) * 3125) / p_var1;

		p_var1 = ((int64_t)calib.dig_P9 * (pressure >> 13) * (pressure >> 13)) >> 25;
		p_var2 = ((int64_t)calib.dig_P8 * pressure) >> 19;

		pressure = ((pressure + p_var1 + p_var2) >> 8) + ((int64_t)calib.dig_P7 << 4);
	}

	data->temp = temp;
	data->press = pressure;
}



