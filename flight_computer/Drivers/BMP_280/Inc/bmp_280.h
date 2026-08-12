#ifndef BMP_280_INC_BMP_280_H_
#define BMP_280_INC_BMP_280_H_

#include <main.h>

typedef struct {
	int32_t temp;
	int32_t press;
} bmp_280_data_t;

extern SPI_HandleTypeDef hspi1;

#define BARO_SPI       hspi1
#define BARO_CS_PORT   SPI1_BARO_CS_GPIO_Port
#define BARO_CS_PIN    SPI1_BARO_CS_Pin

#define ID           0xd0
#define RESET        0xe0
#define STATUS       0xf3

#define CONFIG       0xf5
#define CTRL_MEAS    0xf4

#define TEMP_MSB     0xfa
#define TEMP_LSB     0xfb
#define TEMP_XLSB    0xfc

#define PRESS_MSB    0xf7
#define PRESS_LSB    0xf8
#define PRESS_XLSB   0xf9

void bmp_280_init();
void bmp_280_read_reg(uint8_t address, uint8_t *data);
void bmp_280_write_reg(uint8_t reg, uint8_t data);
void bmp_280_read_data(bmp_280_data_t *data);

#endif /* BMP_280_INC_BMP_280_H_ */
