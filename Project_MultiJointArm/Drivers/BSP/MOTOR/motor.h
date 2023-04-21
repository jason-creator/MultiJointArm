/**
 ****************************************************************************************************
 * @file        motor.h
 * @author      LiuZhao
 * @version     V1.0
 * @date        2023-2-16
 * @brief       Motor controller for ZeroErr eRob70H80I-BM-18CN
 ****************************************************************************************************
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/CAN/can.h"
#include "./BSP/CONTROL/control.h"


/******************************************************************************************/

#define ID_ElbowBendMotor								0xA
#define ID_ElbowPronationMotor					0x0B

#define Last														0x00
#define Current													0x01
#define Desire													0x02
#define UpLimitation										0x03
#define DownLimitation									0x04

// Modes of operation
#define OperatingMode_ProfilePosition 	0x01
#define OperatingMode_ProfileVelocity 	0x03
#define OperatingMode_ProfileTorque 		0x04

#define OperatingMode_CyclicPosition 		0x08
#define OperatingMode_CyclicVelocity 		0x09
#define OperatingMode_CyclicTorque 			0x0A

uint8_t Motor_Init_SDO(uint8_t mode, uint16_t acceleration, uint16_t deceleration, int speed, uint16_t communication_cycle_period);
uint8_t Motor_Init(uint8_t mode, uint16_t acceleration, uint16_t deceleration, int speed, uint16_t communication_cycle_period);
uint8_t Motor_Init_DIY(void);

uint8_t SetVelocity(int speed);
uint8_t SetPosition(int position);

uint8_t SetTorque(int torque);
uint8_t SetTorque_SDO(int torque);
uint8_t SetTorque_DIY(int current, uint32_t limited_speed);

uint8_t DisableMotor(void);
uint8_t DisableMotor_DIY(void);

uint8_t StopMotor(void);
uint8_t StopMotor_DIY(void);

uint32_t Get_CurrentPosition(void);
int Get_CurrentVelocity(void);

#endif

