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

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) > (b) ? (b) : (a))

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/MOTOR/motor.h"
#include "./BSP/TorqueSensor/TorqueSensor.h"
#include "./SYSTEM/usart/usart.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>


typedef struct  
{
	int Torque[3][3];
	int Velocity[3][3];
	int Position[3][5];
	int Current_mA[3][3];
	int limited_speed[3];
	int range[3];
	float PID[3][3];
	float last_error;
	float integral;
	uint16_t acceleration;
	uint16_t deceleration;
	uint16_t communication_cycle_period;
}JOINT_INFO;

//float PID_integral[3];
//float PID_last_error[3];
int sgn(int x);

void PID_Reset(JOINT_INFO *joint);

int free_move(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint);

int active_control(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint);
uint8_t assist_control(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint);
uint8_t active_control_position_limitation(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint);
void send_joint_data_to_pc(int32_t velocity, int32_t position, int32_t torque);

uint8_t free_move_Maxon(uint8_t motor_num, JOINT_INFO *joint);
uint8_t active_control_Maxon(uint8_t motor_num, JOINT_INFO *joint);
uint8_t active_control_position_limitation_Maxon(uint8_t motor_num, JOINT_INFO *joint);
int calculate_velocity_reduction_factor(uint8_t motor_num, int position_difference, JOINT_INFO *joint, int a);

int torque2velocity(uint8_t motor_num, JOINT_INFO *joint);  // 力矩速度、加速度映射关系方程
int torque2velocity_Maxon(uint8_t motor_num, JOINT_INFO *joint);

uint8_t state_choice_func(void);  // 状态机选择函数


#endif

