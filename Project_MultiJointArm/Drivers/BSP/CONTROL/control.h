/**
 ****************************************************************************************************
 * @file        control.h
 * @author      LiuZhao
 * @version     V1.0
 * @date        2023-3-2
 * @brief       Controlling algorithm
 ****************************************************************************************************
 */
 
#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/MOTOR/motor.h"
#include "./BSP/TorqueSensor/TorqueSensor.h"

typedef struct  
{
	int Torque[3];
	int Velocity[3];
	int Position[5];
	int Current_mA[3];
	int limited_speed;
	uint16_t acceleration;
	uint16_t deceleration;
	uint16_t communication_cycle_period;
}JOINT_INFO;

uint8_t active_control(JOINT_INFO *joint);
uint8_t assist_control(JOINT_INFO *joint);
uint8_t active_control_speed_limitation(JOINT_INFO *joint);

int torque2velocity(JOINT_INFO *joint);

#endif

