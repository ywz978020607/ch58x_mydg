/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : ��������Ӧ��������������ϵͳ��ʼ��
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#include "CONFIG.h"
#include "HAL.h"
#include "hiddev.h"
#include "hidkbd.h"

#include "blerock.h"

// DEBUG define at settings - symbols
/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   ��ѭ��
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline))
void Main_Circulation()
{
    while(1)
    {
        TMOS_SystemProcess();
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   ������
 *
 * @return  none
 */
int main(void)
{
#if(defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif
    SetSysClock(CLK_SOURCE_PLL_60MHz);
#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif
#ifdef DEBUG
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
#endif
    PRINT("%s\n", VER_LIB);
    CH58X_BLEInit();
    HAL_Init();
    GAPRole_PeripheralInit();
    HidDev_Init();
    HidEmu_Init();

    // add init
    adcInit();

    Main_Circulation();
}

/******************************** endfile @ main ******************************/
/*********************************************************************
 * @fn      GPIOA_IRQHandler
 *
 * @brief   GPIOA中断函数
 *
 * @return  none
 *
GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU );   //部分GPIO中断引脚初始化 |GPIO_Pin_15|GPIO_Pin_14
GPIOA_ITModeCfg( GPIO_Pin_4, GPIO_ITMode_FallEdge ); // 低电平中断
PFIC_EnableIRQ( GPIO_A_IRQn );//开启GPIOA中断
PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, Long_Delay);
 */
__INTERRUPT
__HIGH_CODE
void GPIOA_IRQHandler(void)
{
    GPIOA_ClearITFlagBit(GPIO_Pin_4);
}
