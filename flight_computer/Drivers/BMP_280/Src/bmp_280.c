#include "bmp_280.h"

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

