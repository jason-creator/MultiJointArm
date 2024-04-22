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

#define ID_ElbowBendMotor								0x02
#define ID_ElbowPronationMotor					0x0A
#define ID_WristBendMotor					      0x05

#define ElbowBendMotor								  0x00
#define ElbowPronationMotor					    0x01
#define WristBendMotor					        0x02

#define Kp								  						0x00
#define Ki					    								0x01
#define Kd					        						0x02

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
uint8_t Motor_Init(uint8_t motor_id, uint8_t mode, uint16_t acceleration, uint16_t deceleration, int speed, uint16_t communication_cycle_period);
uint8_t Motor_Init_DIY(void);
uint8_t Motor_Init_Maxon(uint8_t mode, uint16_t acceleration, uint16_t deceleration);

uint8_t SetVelocity(uint8_t motor_id, int speed);
uint8_t SetPosition(uint8_t motor_id, int position);

uint8_t ProfileVelocity(int speed);  // Maxon

uint8_t SetTorque(uint8_t motor_id, int torque);
uint8_t SetTorque_SDO(int torque);
uint8_t SetTorque_DIY(int current, uint32_t limited_speed);

uint8_t DisableMotor(uint8_t motor_id);
uint8_t DisableMotor_DIY(void);
uint8_t DisableMotor_Maxon(void);

uint8_t StopMotor(uint8_t motor_id);
uint8_t StopMotor_DIY(void);
uint8_t StopMotor_Maxon(void);
 
int Get_CurrentPosition(uint8_t motor_id);
int Get_CurrentVelocity(uint8_t motor_id);

int Get_CurrentPosition_Maxon(void);
int Get_CurrentVelocity_Maxon(void);


#endif

