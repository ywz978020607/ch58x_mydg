/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidkbd.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : ��������Ӧ�ó��򣬳�ʼ���㲥���Ӳ�����Ȼ��㲥��ֱ�����������󣬶�ʱ�ϴ���ֵ
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hidkbdservice.h"
#include "hiddev.h"
#include "hidkbd.h"
#include "CH58x_common.h"

#include "blerock.h"

/*********************************************************************
 * MACROS
 */
#define MOUSE_BUTTON_NONE                    0x00
#define MOUSE_BUTTON_LEFT                    0x01
#define MOUSE_BUTTON_RIGHT                   0x02

#define CHAR_KEY_UP_ARROW       0x52
#define CHAR_KEY_DOWN_ARROW     0x51
#define CHAR_KEY_LEFT_ARROW     0x50
#define CHAR_KEY_RIGHT_ARROW    0x4f
#define KEY_MEDIA_VOLUME_UP     0x80
#define KEY_MEDIA_VOLUME_DOWN   0x81
// #define KEY_MEDIA_NEXT_TRACK        0x
// #define KEY_MEDIA_PREVIOUS_TRACK
#define KEY_MEDIA_PLAY_PAUSE_SPACE   0x2c

/*
上翻页 0x4b
下翻页 0x4e
home 0x4a
end 0x4d
*/

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN              8
#define HID_MOUSE_IN_RPT_LEN                 4

// HID LED output report length
#define HID_LED_OUT_RPT_LEN                  1

/*********************************************************************
 * CONSTANTS
 */
// Param update delay
#define START_PARAM_UPDATE_EVT_DELAY         12800

// Param update delay
#define START_PHY_UPDATE_DELAY               1600

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT             60000

// Minimum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    8

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    8

// Slave latency to use if parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms)
#define DEFAULT_DESIRED_CONN_TIMEOUT         500

// Default passcode
#define DEFAULT_PASSCODE                     0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                 GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                    FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                 TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES              GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL          6

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
volatile uint8_t aliveFlag = 0; // 判断可睡眠的标志

// Task ID
static uint8_t hidEmuTaskId = INVALID_TASK_ID;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
static uint8_t mouseButtonVal = MOUSE_BUTTON_NONE;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

// GAP Profile - Name attribute for SCAN RSP data
static uint8_t scanRspData[] = {
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // service UUIDs
    0x05, // length of this data
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
    LO_UINT16(BATT_SERV_UUID),
    HI_UINT16(BATT_SERV_UUID),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// Advertising data
static uint8_t advertData[] = {
    // flags
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03, // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(0x03C0),
    HI_UINT16(0x03C0),
    // LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
    // HI_UINT16(GAP_APPEARE_HID_KEYBOARD),

    0x0D,                           // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE, // AD Type = Complete local name
    'H',
    'I',
    'D',
    ' ',
    'K',
    'e',
    'y',
    'b',
    'r',
    'o',
    'a',
    'd',  // connection interval range
};

// Device name attribute value
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "HID Keyboard";

// HID Dev configuration
static hidDevCfg_t hidEmuCfg = {
    DEFAULT_HID_IDLE_TIMEOUT, // Idle timeout
    HID_FEATURE_FLAGS         // HID feature flags
};

static uint16_t hidEmuConnHandle = GAP_CONNHANDLE_INIT;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t trigger_task_main();

static void    hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void    hidEmuSendKbdReport(uint8_t keycode);
static void    hidEmuSendMouseReport(uint8_t buttons, uint8_t X_data, uint8_t Y_data, uint8_t W_data);
static void    MOUSE_PRESS(uint8_t buttons);
static void    MOUSE_RELEASE(uint8_t buttons);
static uint8_t IS_MOUSE_PRESS(uint8_t buttons);
static void    MOUSE_CLICK(uint8_t buttons);
static void    MOUSE_MOVE(uint8_t X_data, uint8_t Y_data, uint8_t W_data);


static uint8_t hidEmuRcvReport(uint8_t len, uint8_t *pData);
static uint8_t hidEmuRptCB(uint8_t id, uint8_t type, uint16_t uuid,
                           uint8_t oper, uint16_t *pLen, uint8_t *pData);
static void    hidEmuEvtCB(uint8_t evt);
static void    hidEmuStateCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);


static void SendGamepadReport(uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry) {
    uint8_t buf[HID_MOUSE_IN_RPT_LEN];

    buf[0] = lx;
    buf[1] = ly;
    buf[2] = rx;
    buf[3] = ry;

    HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                  HID_MOUSE_IN_RPT_LEN, buf);

}

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidEmuHidCBs = {
    hidEmuRptCB,
    hidEmuEvtCB,
    NULL,
    hidEmuStateCB};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmu_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void HidEmu_Init()
{
    hidEmuTaskId = TMOS_ProcessEventRegister(HidEmu_ProcessEvent);

    // Setup the GAP Peripheral Role Profile
    {
        uint8_t initial_advertising_enable = TRUE;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);

        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    }

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), (void *)attDeviceName);

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = DEFAULT_PASSCODE;
        uint8_t  pairMode = DEFAULT_PAIRING_MODE;
        uint8_t  mitm = DEFAULT_MITM_MODE;
        uint8_t  ioCap = DEFAULT_IO_CAPABILITIES;
        uint8_t  bonding = DEFAULT_BONDING_MODE;
        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Setup Battery Characteristic Values
    {
        uint8_t critical = DEFAULT_BATT_CRITICAL_LEVEL;
        Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof(uint8_t), &critical);
    }

    // Set up HID keyboard service
    Hid_AddService();

    // Register for HID Dev callback
    HidDev_Register(&hidEmuCfg, &hidEmuHidCBs);

    // Setup a delayed profile startup
    tmos_set_event(hidEmuTaskId, START_DEVICE_EVT);

    // Add: Setup a task to check sleep
    tmos_start_task(hidEmuTaskId, START_CHECK_TO_SLEEP, CHECK_TO_SLEEP_INTERVAL);
}

/*********************************************************************
 * @fn      HidEmu_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t HidEmu_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(hidEmuTaskId)) != NULL)
        {
            hidEmu_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & START_DEVICE_EVT)
    {
        return (events ^ START_DEVICE_EVT);
    }

    if(events & START_PARAM_UPDATE_EVT)
    {
        // Send connect param update request
        GAPRole_PeripheralConnParamUpdateReq(hidEmuConnHandle,
                                             DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                             DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                             DEFAULT_DESIRED_SLAVE_LATENCY,
                                             DEFAULT_DESIRED_CONN_TIMEOUT,
                                             hidEmuTaskId);
        aliveFlag = 1; // Refresh alive
        return (events ^ START_PARAM_UPDATE_EVT);
    }

    if(events & START_PHY_UPDATE_EVT)
    {
        // start phy update
        PRINT("Send Phy Update %x...\n", GAPRole_UpdatePHY(hidEmuConnHandle, 0,
                    GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE));

        return (events ^ START_PHY_UPDATE_EVT);
    }

    if(events & START_REPORT_EVT)
    {
        trigger_task_main();

        tmos_start_task(hidEmuTaskId, START_REPORT_EVT, TRIGGER_INTERVAL);
        return (events ^ START_REPORT_EVT);
    }

    if(events & START_CHECK_TO_SLEEP)
    {
        if(aliveFlag == 0){
            // 配置中断源
            GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU );   //部分GPIO中断引脚初始化 |GPIO_Pin_15|GPIO_Pin_14
            GPIOA_ITModeCfg( GPIO_Pin_4, GPIO_ITMode_FallEdge ); // 低电平中断
            PFIC_EnableIRQ( GPIO_A_IRQn );//开启GPIOA中断
            PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, Long_Delay);
            // 进入睡眠
            PRINT("shut down mode sleep \n");
            DelayMs(2);
            LowPower_Shutdown(0); //全部断电，唤醒后复位
            /*
            此模式唤醒后会执行复位，所以下面代码不会运行，
            注意要确保系统睡下去再唤醒才是唤醒复位，否则有可能变成IDLE等级唤醒
            */
            HSECFG_Current(HSE_RCur_100); // 降为额定电流(低功耗函数中提升了HSE偏置电流)
            PRINT("wake.. \n");
            DelayMs(500);
        }
        aliveFlag = 0; // Reset alive

        //
        tmos_start_task(hidEmuTaskId, START_CHECK_TO_SLEEP, CHECK_TO_SLEEP_INTERVAL);
        return (events ^ START_CHECK_TO_SLEEP);
    }

    return 0;
}

/*********************************************************************
 * @fn      hidEmu_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        default:
            break;
    }
}

/*********************************************************************
 * @fn      hidEmuSendKbdReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void hidEmuSendKbdReport(uint8_t keycode)
{
    uint8_t buf[HID_KEYBOARD_IN_RPT_LEN];

    buf[0] = 0;       // Modifier keys
    buf[1] = 0;       // Reserved
    buf[2] = keycode; // Keycode 1
    buf[3] = 0;       // Keycode 2
    buf[4] = 0;       // Keycode 3
    buf[5] = 0;       // Keycode 4
    buf[6] = 0;       // Keycode 5
    buf[7] = 0;       // Keycode 6

    HidDev_Report(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                  HID_KEYBOARD_IN_RPT_LEN, buf);
}

static void hidEmuSendMouseReport(uint8_t buttons, uint8_t X_data, uint8_t Y_data, uint8_t W_data)
{
    uint8_t buf[HID_MOUSE_IN_RPT_LEN];

    buf[0] = buttons; // Buttons
    buf[1] = X_data;  // X right
    buf[2] = Y_data;  // Y down
    buf[3] = W_data;  // Wheel

    HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                  HID_MOUSE_IN_RPT_LEN, buf);
}

static void MOUSE_PRESS(uint8_t buttons)
{
    uint8_t buf[HID_MOUSE_IN_RPT_LEN];

    mouseButtonVal |= buttons;

    buf[0] = mouseButtonVal; // Buttons
    buf[1] = 0;  // X right
    buf[2] = 0;  // Y down
    buf[3] = 0;  // Wheel

    HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                  HID_MOUSE_IN_RPT_LEN, buf);
}

static void MOUSE_RELEASE(uint8_t buttons)
{
    uint8_t buf[HID_MOUSE_IN_RPT_LEN];

    mouseButtonVal = mouseButtonVal & (~buttons);

    buf[0] = mouseButtonVal; // Buttons
    buf[1] = 0;  // X right
    buf[2] = 0;  // Y down
    buf[3] = 0;  // Wheel

    HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                  HID_MOUSE_IN_RPT_LEN, buf);
}

static uint8_t IS_MOUSE_PRESS(uint8_t buttons)
{
    if((mouseButtonVal & buttons) > 0){
        return 1;
    }
    else{
        return 0;
    }
}

static void MOUSE_CLICK(uint8_t buttons)
{
    uint8_t buf[HID_MOUSE_IN_RPT_LEN];

    mouseButtonVal = buttons;
    buf[0] = mouseButtonVal; // Buttons
    buf[1] = 0;  // X right
    buf[2] = 0;  // Y down
    buf[3] = 0;  // Wheel
    HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                  HID_MOUSE_IN_RPT_LEN, buf);

    mouseButtonVal = MOUSE_BUTTON_NONE;
    buf[0] = mouseButtonVal; // Buttons
    buf[1] = 0;  // X right
    buf[2] = 0;  // Y down
    buf[3] = 0;  // Wheel
    HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                  HID_MOUSE_IN_RPT_LEN, buf);
}

static void MOUSE_MOVE(uint8_t X_data, uint8_t Y_data, uint8_t W_data)
{
    uint8_t buf[HID_MOUSE_IN_RPT_LEN];

    buf[0] = mouseButtonVal; // Buttons
    buf[1] = X_data;  // X right
    buf[2] = Y_data;  // Y down
    buf[3] = W_data;  // Wheel

    HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                  HID_MOUSE_IN_RPT_LEN, buf);
}


/*********************************************************************
 * @fn      hidEmuStateCB
 *
 * @brief   GAP state change callback.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void hidEmuStateCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
        {
            uint8_t ownAddr[6];
            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddr);
            GAP_ConfigDeviceAddr(ADDRTYPE_STATIC, ownAddr);
            PRINT("Initialized..\n");
        }
        break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Advertising..\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

                // get connection handle
                hidEmuConnHandle = event->connectionHandle;
                tmos_start_task(hidEmuTaskId, START_PARAM_UPDATE_EVT, START_PARAM_UPDATE_EVT_DELAY);
                PRINT("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Connected Advertising..\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                PRINT("Advertising timeout..\n");
            }
            // Enable advertising
            {
                uint8_t initial_advertising_enable = TRUE;
                // Set the GAP Role Parameters
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
            }
            break;

        case GAPROLE_ERROR:
            PRINT("Error %x ..\n", pEvent->gap.opcode);
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      hidEmuRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8_t hidEmuRcvReport(uint8_t len, uint8_t *pData)
{
    // verify data length
    if(len == HID_LED_OUT_RPT_LEN)
    {
        // set LEDs
        return SUCCESS;
    }
    else
    {
        return ATT_ERR_INVALID_VALUE_SIZE;
    }
}

/*********************************************************************
 * @fn      hidEmuRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8_t hidEmuRptCB(uint8_t id, uint8_t type, uint16_t uuid,
                           uint8_t oper, uint16_t *pLen, uint8_t *pData)
{
    uint8_t status = SUCCESS;

    // write
    if(oper == HID_DEV_OPER_WRITE)
    {
        if(uuid == REPORT_UUID)
        {
            // process write to LED output report; ignore others
            if(type == HID_REPORT_TYPE_OUTPUT)
            {
                status = hidEmuRcvReport(*pLen, pData);
            }
        }

        if(status == SUCCESS)
        {
            status = Hid_SetParameter(id, type, uuid, *pLen, pData);
        }
    }
    // read
    else if(oper == HID_DEV_OPER_READ)
    {
        status = Hid_GetParameter(id, type, uuid, pLen, pData);
    }
    // notifications enabled
    else if(oper == HID_DEV_OPER_ENABLE)
    {
        tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 500);
    }
    return status;
}

/*********************************************************************
 * @fn      hidEmuEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidEmuEvtCB(uint8_t evt)
{
    // process enter/exit suspend or enter/exit boot mode
    return;
}

/*********************************************************************
*********************************************************************/
static uint8_t trigger_task_main(){
//    SendGamepadReport(10,50,255,0);

    temp_val_1 = calc_aout(idx_pointer_x);
    temp_val_2 = calc_aout(idx_pointer_y);
    temp_val_3 = calc_aout(idx_rocker_x);
    temp_val_4 = calc_aout(idx_rocker_y);

    if(ABS(temp_val_1 - last_temp_val_1) > 50 || ABS(temp_val_2 - last_temp_val_2) > 50){
        aliveFlag = 1; // refresh alive

        last_temp_val_1 = temp_val_1;
        last_temp_val_2 = temp_val_2;
        last_temp_val_3 = temp_val_3;
        last_temp_val_4 = temp_val_4;
    }

    SendGamepadReport(temp_val_1, temp_val_2, temp_val_3, temp_val_4);
    return 0;
}

