/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "CAN_receive.h"
#include "main.h"
#include "can.h"

// Motor data read macro
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

// Motor data array for 6DOF arm: 0-2 GM6020, 3-5 M2006
static motor_measure_t motor_arm[6];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    // Handle only CAN2 (hcan2) for 6DOF arm, IDs 0x201-0x206
    if (hcan == &hcan2) {
        switch (rx_header.StdId)
        {
            case CAN_M1_6020_ID:  // 0x201 (GM6020_1)
            case CAN_M2_6020_ID:  // 0x202 (GM6020_2)
            case CAN_M3_6020_ID:  // 0x203 (GM6020_3)
            case CAN_M4_2006_ID:  // 0x204 (M2006_1)
            case CAN_M5_2006_ID:  // 0x205 (M2006_2)
            case CAN_M6_2006_ID:  // 0x206 (M2006_3)
            {
                uint8_t i = rx_header.StdId - CAN_M1_6020_ID; // Map 0x201-0x206 to 0-5
                if (i < 6) { // Only handle 0-5 for 6 motors
                    get_motor_measure(&motor_arm[i], rx_data);
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }
}

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x208)
  * @param[in]      yaw: (0x201) 6020 motor control current, range [-30000,30000]
  * @param[in]      pitch: (0x202) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x203) 6020 motor control current, range [-30000,30000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set motor to quick ID setting
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          send control current of motor (0x204, 0x205, 0x206, 0x207)
  * @param[in]      motor1: (0x204) 2006 motor control current, range [-10000,10000]
  * @param[in]      motor2: (0x205) 2006 motor control current, range [-10000,10000]
  * @param[in]      motor3: (0x206) 2006 motor control current, range [-10000,10000]
  * @param[in]      motor4: (0x207) reserved
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the GM6020_1 motor data point (0x201)
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_arm[0]; // GM6020_1 (0x201)
}

/**
  * @brief          return the GM6020_2 motor data point (0x202)
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_arm[1]; // GM6020_2 (0x202)
}

/**
  * @brief          return the GM6020_3 motor data point (0x203)
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_arm[2]; // GM6020_3 (0x203)
}

/**
  * @brief          return the M2006 motor data point (0x204-0x206)
  * @param[in]      i: motor number, range [0,2]
  * @retval         motor data point
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_arm[(i & 0x03) + 3]; // M2006: 3-5 (0x204-0x206)
}
