#ifndef BMP_280_INC_BMP_280_H_
#define BMP_280_INC_BMP_280_H_

#include <main.h>

typedef struct {
	int32_t temp;
	int32_t press;
} bmp_280_data_t;

typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} bmp_280_calib_t;

extern SPI_HandleTypeDef hspi1;

#define BARO_SPI             hspi1
#define BARO_CS_PORT         SPI1_BARO_CS_GPIO_Port
#define BARO_CS_PIN          SPI1_BARO_CS_Pin

#define PRESS_OVERSAMPLING   _press_x4
#define TEMP_OVERSAMPLING    _temp_x1
#define POWER_MODE           _normal

#define FILTER_COEFF         _coeff_4
#define STANDBY_TIME_MS      _125

#define ID                   0xd0
#define RESET                0xe0
#define STATUS               0xf3

#define CONFIG               0xf5
#define CTRL_MEAS            0xf4

#define TEMP_MSB             0xfa
#define TEMP_LSB             0xfb
#define TEMP_XLSB            0xfc

#define PRESS_MSB            0xf7
#define PRESS_LSB            0xf8
#define PRESS_XLSB           0xf9

typedef enum {
	_press_skipped,
	_press_x1,
	_press_x2,
	_press_x4,
	_press_x8,
	_press_x16
} press_oversampling;

typedef enum {
	_temp_skipped,
	_temp_x1,
	_temp_x2,
	_temp_x4,
	_temp_x8,
	_temp_x16
} temp_oversampling;

typedef enum {
	_sleep = 0x00,
	_forced = 0x01,
	_normal = 0x03
} power_mode;

typedef enum {
	_off,
	_coeff_2,
	_coeff_4,
	_coeff_8,
	_coeff_16,
} filter_coefficient;

typedef enum {
	_0p5, // p represents a period, so 0p5 = 0.5
	_62p5,
	_125,
	_250,
	_500,
	_1000,
	_2000,
	_4000
} standby_time_ms;

void bmp_280_init();
void bmp_280_read_reg(uint8_t address, uint8_t *data);
void bmp_280_write_reg(uint8_t reg, uint8_t data);
void bmp_280_read_data(bmp_280_data_t *data);

#endif /* BMP_280_INC_BMP_280_H_ */
