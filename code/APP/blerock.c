
#include "blerock.h"

int key_pin[] = {GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_4}; // rocker_key, pointer_key, s_key_1
PinPORT key_port[] = {GPIO_PORTB,GPIO_PORTB,GPIO_PORTA};
const uint8_t ROCKER_KEY_PULL = 1; //静息时电平状态(上下拉)
const uint8_t POINTER_KEY_PULL = 1; //静息时电平状态(上下拉)
const uint8_t S_KEY_PULL = 1; //静息时电平状态(上下拉)

int adc_pin[] = {GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15}; // rx, ry, px, py - PortA
uint8_t adc_chs[] = {2, 3, 4, 5}; // rx, ry, px, py
volatile int16_t adc_bias_rocker[] = {0, 0, 0, 0}; // rx, ry, px, py
uint8_t adc_oppo[] = {1, 0, 0, 1}; // rx, ry, px, py
const int16_t rank_num = 4; // 正负分成rank_num档
const int16_t rank_val = 1500; // 单边偏离的原始采集设置范围1400
const int16_t level_val = rank_val * 2 / (2 * 4 + 1); // 共2*rank_num+1个区间

volatile uint8_t temp_val_1 = 0;
volatile uint8_t temp_val_2 = 0;
volatile uint8_t temp_val_3 = 0;
volatile uint8_t temp_val_4 = 0;
volatile uint8_t last_temp_val_1 = 0;
volatile uint8_t last_temp_val_2 = 0;
volatile uint8_t last_temp_val_3 = 0;
volatile uint8_t last_temp_val_4 = 0;
volatile uint8_t mode = 0; // 0: direction  1: scroll

signed short RoughCalib_Value = 0; // ADC粗调偏差值

uint8_t digitalRead(int pin_idx){
    return (key_port[pin_idx]==GPIO_PORTB)?((GPIOB_ReadPortPin( key_pin[pin_idx] )==0)?0:1):((GPIOA_ReadPortPin( key_pin[pin_idx] )==0)?0:1);
}

int16_t _read_ADC(uint8_t ch, uint8_t avg_num){
    ADC_ChannelCfg(ch);
    if(avg_num == 1)
        return ADC_ExcutSingleConver(); // + RoughCalib_Value;
    else{
        int16_t avg_val = 0;
        for(uint8_t ii = 0; ii < avg_num; ++ii){
            avg_val += ADC_ExcutSingleConver();
            // delay(50);
        }
        avg_val = avg_val/avg_num; // + RoughCalib_Value;
        return avg_val;
    }
}

uint8_t calc_aout(uint8_t ch_idx){
    uint8_t res = (adc_oppo[ch_idx])?(-((_read_ADC(adc_chs[ch_idx], 1) - adc_bias_rocker[ch_idx]) / (float)rank_val)*128 + 128):(((_read_ADC(adc_chs[ch_idx], 1) - adc_bias_rocker[ch_idx]) / (float)rank_val)*128 + 128);
    return res;
};


void adcInit(){
    for(uint8_t idx = 0; idx < 4; ++idx){
        GPIOA_ModeCfg(adc_pin[idx], GPIO_ModeIN_Floating);
        ADC_ExtSingleChSampInit(SampleFreq_3_2, ADC_PGA_1_2);
        adc_bias_rocker[idx] = _read_ADC(adc_chs[idx], 5) - level_val/2; // 中间值 - 半挡位区间
    }

    GPIOA_ModeCfg(key_pin[s_key_1], S_KEY_PULL?GPIO_ModeIN_PU:GPIO_ModeIN_PD);
    GPIOB_ModeCfg(key_pin[rocker_key], ROCKER_KEY_PULL?GPIO_ModeIN_PU:GPIO_ModeIN_PD); // PortB
    GPIOB_ModeCfg(key_pin[pointer_key], POINTER_KEY_PULL?GPIO_ModeIN_PU:GPIO_ModeIN_PD);// PortB

    RoughCalib_Value = ADC_DataCalib_Rough();
}





