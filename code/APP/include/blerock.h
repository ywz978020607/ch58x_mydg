#ifndef BLE_ROCK_DEF_H_
#define BLE_ROCK_DEF_H_

#include "CH58x_common.h"

// #define adc_scalar ADC_PGA_1_4 // ADC_PGA_0
// /*
// ADC_PGA_1_4 - 3.3V 测试发现居中1.6v大概是2338，摇杆拉满3.2V大概是3104，拉低0.2V大概是1612
// */
#define adc_scalar ADC_PGA_1_2 // ADC_PGA_0
/*
ADC_PGA_1_2 - 3.3V 测试发现居中1.6v大概是2630，摇杆拉满3.2V大概是4100，拉低0.2V大概是1320

设置dt = 1400
each_val - middle_val

*/
typedef enum _AIO_STATE
{
  GPIO_PORTA,
  GPIO_PORTB
} PinPORT;

#define rocker_key 0 //摇杆自带
#define pointer_key 1 //摇杆自带
#define s_key_1 2

extern const uint8_t ROCKER_KEY_PULL; //静息时电平状态(上下拉)
extern const uint8_t POINTER_KEY_PULL; //静息时电平状态(上下拉)
extern const uint8_t S_KEY_PULL;


extern int key_pin[];
extern PinPORT key_port[];

#define idx_rocker_x 1
#define idx_rocker_y 0
#define idx_pointer_x 3
#define idx_pointer_y 2
// battery adc: PA5

extern uint8_t adc_chs[];
extern volatile int16_t adc_bias_rocker[];

// -----------function define
#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif
// -----------function variable

extern volatile uint8_t temp_val_1;
extern volatile uint8_t temp_val_2;
extern volatile uint8_t temp_val_3;
extern volatile uint8_t temp_val_4;
extern volatile uint8_t last_temp_val_1;
extern volatile uint8_t last_temp_val_2;
extern volatile uint8_t last_temp_val_3;
extern volatile uint8_t last_temp_val_4;
extern const int16_t rank_num;
extern volatile uint8_t mode; // 1: scroll 0: direction

uint8_t digitalRead(int pin_idx);
int16_t _read_ADC(uint8_t ch, uint8_t avg_num);
uint8_t calc_aout(uint8_t ch_idx);

void adcInit();

// debug
// extern signed short RoughCalib_Value;
extern const int16_t level_val;

#endif
