/**
 ****************************************************************************************************
 * @file        control.c
 * @author      LiuZhao
 * @version     V1.0
 * @date        2023-3-2
 * @brief       Controlling algorithm
 ****************************************************************************************************
 */
 
 #include "./BSP/CONTROL/control.h"
 #include "./BSP/MOTOR/motor.h"
 #include "stdlib.h"
 


uint8_t assist_control(JOINT_INFO *joint)
{

	if (joint->Torque[Current] < -3)
		SetPosition(joint->Position[Last] + 65534);
	if (joint->Torque[Current] > 3)
		SetPosition(joint->Position[Last] - 65534);

	return 0;
	
}

/********************* For the CAN_DIY communication function **************
uint8_t active_control(JOINT_INFO *joint)
{

//	if (joint->Torque[Current] < -5)
//		SetTorque(joint->Torque[Desire]);
//	if (joint->Torque[Current] > 5)
//		SetTorque(-joint->Torque[Desire]);
	
	if (joint->Torque[Current] < -5)
		SetTorque_DIY(joint->Current_mA[Desire], joint->limited_speed);
	if (joint->Torque[Current] > 5)
		SetTorque_DIY(-joint->Current_mA[Desire], joint->limited_speed);

	return 0;

}

uint8_t active_control_speed_limitation(JOINT_INFO *joint)
{
	if (joint->Position[Current]-joint->Position[DownLimitation] > 10)
	{
		SetTorque_DIY(-joint->Current_mA[Desire], joint->limited_speed);
		printf("In the down limitation\n");
	}
	else if(joint->Position[Current]-joint->Position[UpLimitation] < -10)
	{
		SetTorque_DIY(joint->Current_mA[Desire], joint->limited_speed);
		printf("In the up limitation\n");
	}
	else
	{
		active_control(joint);
	}
	
	return 0;
}
****************** For the CAN_DIY communication function *****************/

int torque2velocity(JOINT_INFO *joint)
{
	joint->Velocity[Desire] = -146 * joint->Torque[Current];  // ignore the gravity
	joint->acceleration = 56 * joint->Torque[Current] + 5566;
	joint->deceleration = 56 * joint->Torque[Current] + 5566;

	return 0;
}

uint8_t free_move(JOINT_INFO *joint)
{
	if (joint->Torque[Current] < -5)
		SetVelocity(joint->Velocity[Desire]);
	if (joint->Torque[Current] > 5)
		SetVelocity(joint->Velocity[Desire]);
	
	return 0;
}

uint8_t active_control(JOINT_INFO *joint)
{
	printf("Desired velocity:%d\n", joint->Velocity[Desire]);
	
	if ((joint->limited_speed - joint->Velocity[Desire] < 5) && (joint->Velocity[Current] > 0))
	{
		SetVelocity(joint->limited_speed);
		printf("I am controlling the speed1\n");
	}
	else if ((joint->limited_speed + joint->Velocity[Desire] < 5) && (joint->Velocity[Current] < 0))
	{
		SetVelocity(-joint->limited_speed);
		printf("I am controlling the speed2\n");
	}
	else
	{
		free_move(joint);
	}	

	return 0;

}

uint8_t active_control_speed_limitation(JOINT_INFO *joint)
{
	if (joint->Position[Current]-joint->Position[DownLimitation] > 10 && joint->Torque[Current] < -5)
	{
		SetVelocity(0);
		printf("In the down limitation\n");
	}
	else if(joint->Position[Current]-joint->Position[UpLimitation] < -10 && joint->Torque[Current] > 5)
	{
		SetVelocity(0);
		printf("In the up limitation\n");
	}
	else
	{
		active_control(joint);
	}
	
	return 0;
}
