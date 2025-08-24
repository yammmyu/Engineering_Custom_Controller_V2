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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "main.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_M1_6020_ID = 0x201,
    CAN_M2_6020_ID = 0x202,
    CAN_M3_6020_ID = 0x203,
    CAN_M4_2006_ID = 0x204,
    CAN_M5_2006_ID = 0x205,
    CAN_M6_2006_ID = 0x206,
    CAN_GIMBAL_ALL_ID = 0x1FF, // Added for GM6020 control
} can_msg_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x208)
  * @param[in]      yaw: (0x201) 6020 motor control current, range [-30000,30000]
  * @param[in]      pitch: (0x202) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x203) 6020 motor control current, range [-30000,30000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set motor to quick ID setting
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x204, 0x205, 0x206, 0x207)
  * @param[in]      motor1: (0x204) 2006 motor control current, range [-10000,10000]
  * @param[in]      motor2: (0x205) 2006 motor control current, range [-10000,10000]
  * @param[in]      motor3: (0x206) 2006 motor control current, range [-10000,10000]
  * @param[in]      motor4: (0x207) reserved
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the GM6020_1 motor data point (0x201)
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the GM6020_2 motor data point (0x202)
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the GM6020_3 motor data point (0x203)
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the M2006 motor data point (0x204-0x206)
  * @param[in]      i: motor number, range [0,2]
  * @retval         motor data point
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
