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

int sgn(int x) {
    return (x > 0) - (x < 0);
}


void PID_Reset(JOINT_INFO *joint) {
    joint->integral = 0.0f;
    joint->last_error = 0.0f; 
}

float pid_control(int error, JOINT_INFO *joint) 
{
    joint->integral += error;
    float derivative = error - joint->last_error;
    float output = joint->PID[ElbowBendMotor][Kp]*error + joint->PID[ElbowBendMotor][Ki]*joint->integral + joint->PID[ElbowBendMotor][Kd]*derivative;
    joint->last_error = error;
		if(abs(error) < 5)
			PID_Reset(joint);
    return output;
}


uint8_t assist_control(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint)
{
	for(uint8_t i = 0; i < 3; i++)
		{
		if (joint->Torque[i][Current] < -3)
			SetPosition(motor_id, joint->Position[i][Last] + 65534);
		if (joint->Torque[i][Current] > 3)
			SetPosition(motor_id, joint->Position[i][Last] - 65534);
		}

	return 0;
	
}

// ������������ٶȡ��ٶȵ�ӳ���ϵ
int torque2velocity(uint8_t motor_num, JOINT_INFO *joint)
{
	// ����˶������������ش������������෴
	float SupportingArm = -190.0;   // horizontal supporting arm torque 1.33 Nm, the encoder is 77600
//	float radians = -(joint->Position[motor_num][Current] - joint->Position[motor_num][DownLimitation]) /(3.1415 * 524288);
	
	joint->Torque[motor_num][Current] = (int)(joint->Torque[motor_num][Current] + SupportingArm * cos(9.375e-6 * joint->Position[motor_num][Current] - 21.032 ));
	
	switch(motor_num)
	{
		case 0:
			if(abs(joint->Torque[motor_num][Current]) > 10){
				joint->Velocity[motor_num][Desire] = -joint->Torque[motor_num][Current] * 140;  
			}
			else{
				joint->Velocity[motor_num][Desire] = 0;
			}
			joint->acceleration = 5566;
			joint->deceleration = 5566;
			break;
		
		case 1:
			joint->Velocity[motor_num][Desire] = -sgn(joint->Torque[motor_num][Current]) * joint->limited_speed[motor_num];  // ignore the gravity
			joint->acceleration = 5566;
			joint->deceleration = 5566;
			break;
		
		case 2:
			joint->Velocity[motor_num][Desire] = -sgn(joint->Torque[motor_num][Current]) * joint->limited_speed[motor_num];  // ignore the gravity
			joint->acceleration = 5566;
			joint->deceleration = 5566;
			break;
		
		default:
			joint->Velocity[motor_num][Desire] = 0;
			break;
	}
	
	if(joint->Velocity[motor_num][Desire] > 0)
		joint->Velocity[motor_num][Desire] = min(joint->Velocity[motor_num][Desire], joint->limited_speed[motor_num]);
	if(joint->Velocity[motor_num][Desire] < 0)
		joint->Velocity[motor_num][Desire] = -min(abs(joint->Velocity[motor_num][Desire]), joint->limited_speed[motor_num]);
	

	return 0;
}

// �����˶�ģʽ
int free_move(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint)
{
	int velocity_reduction_factor[3];
	if (joint->Torque[motor_num][Current] < -10){
		velocity_reduction_factor[motor_num] = joint->Velocity[motor_num][Current] + pid_control(joint->Velocity[motor_num][Current]-joint->Velocity[motor_num][Desire], joint);
	}
	if (joint->Torque[motor_num][Current] > 10){
		velocity_reduction_factor[motor_num] = joint->Velocity[motor_num][Current] + pid_control(joint->Velocity[motor_num][Current]-joint->Velocity[motor_num][Desire], joint);
	}
	if (joint->Torque[motor_num][Current] < 10 && joint->Torque[motor_num][Current] > -10){
		velocity_reduction_factor[motor_num] = joint->Velocity[motor_num][Current] + pid_control(joint->Velocity[motor_num][Current], joint);
	}
	return velocity_reduction_factor[motor_num];
}

// ���Ƽ����ٶ�
int active_control(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint)
{
//	printf("Desired velocity:%d\n", joint->Velocity[motor_num][Desire]);  // �������ش����������õ�Ŀ���ٶ�
	int velocity_reduction_factor[3];
	// �����ʱ����ת����ǰ�ٶ�Ϊ��ֵ���������Ϊ��ֵ��Ŀ���ٶ�Ϊ��ֵ
	if ((joint->limited_speed[motor_num] - joint->Velocity[motor_num][Desire] < 5) && (joint->Velocity[motor_num][Current] > 0))
	{
		velocity_reduction_factor[motor_num] = joint->Velocity[motor_num][Current] + pid_control(joint->Velocity[motor_num][Current]-joint->limited_speed[motor_num], joint);
//		printf("I am controlling the speed1\n");
	}
	// ���˳ʱ����ת����ʱ�ٶ�Ϊ��ֵ���������Ϊ��ֵ��Ŀ���ٶ�Ϊ��ֵ
	else if ((joint->limited_speed[motor_num] + joint->Velocity[motor_num][Desire] < 5) && (joint->Velocity[motor_num][Current] < 0))
	{
		velocity_reduction_factor[motor_num] = joint->Velocity[motor_num][Current] + pid_control(joint->Velocity[motor_num][Current]+joint->limited_speed[motor_num], joint);
//		printf("I am controlling the speed2\n");
	}
	else
	{
		velocity_reduction_factor[motor_num] = free_move(motor_id, motor_num, joint);
	}	

	return velocity_reduction_factor[motor_num];

}


//int calculate_velocity_reduction_factor(uint8_t motor_num, int position_difference, JOINT_INFO *joint, int a)
//{
//    int distance = abs(position_difference); 
//    double velocity_change = sqrt(2 * abs(a) * distance); 
//    int factor = (int)velocity_change; 
//    factor = min(factor, joint->limited_speed[motor_num]); 
//    return position_difference; 
//}

void update_motor_speed(uint8_t motor_id, uint8_t motor_num, float pid_out, JOINT_INFO *joint) 
{
    int velocity = (int)pid_out;
		if(velocity >= 0)
			velocity = min(velocity, joint->limited_speed[motor_num]);
		if(velocity < 0)
			velocity = -min(abs(velocity), joint->limited_speed[motor_num]);
		 
		SetVelocity(motor_id, velocity);
}

uint8_t active_control_position_limitation(uint8_t motor_id, uint8_t motor_num, JOINT_INFO *joint)
{
    float position_difference_down = min(joint->Position[motor_num][Current] - joint->Position[motor_num][DownLimitation],0);
    float position_difference_up = max(joint->Position[motor_num][Current] - joint->Position[motor_num][UpLimitation],0);
    float pid_out;
		float position_difference = 5 * 1456.35;  // 1 du = 1456.35 num
    if (position_difference_down >= -position_difference && joint->Torque[motor_num][Current] < 0)
    {
				pid_out = joint->Velocity[motor_num][Current] + pid_control(joint->Velocity[motor_num][Current], joint);
    }
    else if (position_difference_up <= position_difference && joint->Torque[motor_num][Current] > 0)
    {
				pid_out = joint->Velocity[motor_num][Current] + pid_control(joint->Velocity[motor_num][Current], joint);
    }
    else
    {
        pid_out = active_control(motor_id, motor_num, joint);
    }
		
		update_motor_speed(motor_id, motor_num, pid_out, joint);
		
    return 0;
}



////////////////////////////////////////Maxon/////////////////////////////////////////////

// ������������ٶȡ��ٶȵ�ӳ���ϵ
int torque2velocity_Maxon(uint8_t motor_num, JOINT_INFO *joint)
{
	// ����˶������������ش����������� same
	joint->Velocity[motor_num][Desire] = 50 * joint->Torque[motor_num][Current];  // ignore the gravity
	joint->acceleration = 10 * joint->Torque[motor_num][Current] + 2500;
	joint->deceleration = 10 * joint->Torque[motor_num][Current] + 2500;


	return 0;
}

// �����˶�ģʽ
uint8_t free_move_Maxon(uint8_t motor_num, JOINT_INFO *joint)
{

	if (joint->Torque[motor_num][Current] < -2)
	ProfileVelocity(joint->Velocity[motor_num][Desire]);
	if (joint->Torque[motor_num][Current] > 2)
	ProfileVelocity(joint->Velocity[motor_num][Desire]);
	
	return 0;
}

// ���Ƽ����ٶ�
uint8_t active_control_Maxon(uint8_t motor_num, JOINT_INFO *joint)
{
//	printf("Desired velocity:%d\n", joint->Velocity[motor_num][Desire]);  // �������ش����������õ�Ŀ���ٶ�
	// ���nishizhen����ǰ�ٶ�Ϊ��ֵ���������Ϊ+��Ŀ���ٶ�Ϊ+
	if ((joint->limited_speed[motor_num] - joint->Velocity[motor_num][Desire] < -50) && (joint->Velocity[motor_num][Current] > 0))
	{
		ProfileVelocity(joint->limited_speed[motor_num]);
//		printf("I am controlling the speed1\n");
	}
	// ���shunshizhen����ʱ�ٶ�Ϊ��ֵ���������Ϊ-��Ŀ���ٶ�Ϊ-
	else if ((joint->limited_speed[motor_num] + joint->Velocity[motor_num][Desire] < -50) && (joint->Velocity[motor_num][Current] < 0))
	{
		ProfileVelocity(-joint->limited_speed[motor_num]);
//		printf("I am controlling the speed2\n");
	}
	else
	{
		free_move_Maxon(motor_num, joint);
	}	
	
	return 0;

}

// ���Ƽ���λ��
uint8_t active_control_position_limitation_Maxon(uint8_t motor_num, JOINT_INFO *joint)
{
	// ���ü���λ��, wrist
	if (joint->Position[motor_num][Current]-joint->Position[motor_num][DownLimitation] < 0 && joint->Torque[motor_num][Current] < -2)
	{
//		ProfileVelocity(0);  // λ�ڼ���λ�ã�����Ҫ�����ŷ����˶����������ٶ�Ϊ�� maxon
		StopMotor_Maxon();
//		printf("In the down limitation\n");
	}
	// ���ü���λ�ã�wrist
	else if(joint->Position[motor_num][Current]-joint->Position[motor_num][UpLimitation] > 0 && joint->Torque[motor_num][Current] > 2)
	{
//		ProfileVelocity(0);  // λ�ڼ���λ�ã�����Ҫ�����������˶����������ٶ�Ϊ�� maxon
		StopMotor_Maxon();
//		printf("In the up limitation\n");
	}
	else
	{
		active_control_Maxon(motor_num, joint);  // ���ڹ����ռ���ʱ��������������ģʽ
	}
	
	return 0;
}


//////////////////////////////////////////Maxon end///////////////////////////////////////////////////////

//void send_joint_data_to_pc(int32_t velocity, int32_t position, int32_t torque)
//{
//    // ������ݣ�������ת��Ϊ�ַ��������ö��ŷָ�
//    char buffer[64];
//    sprintf(buffer, "VEL:%d,POS:%d,TOR:%d\n", velocity, position, torque);  // ����ʾ��ʽ����λ��ͬ���������������޸�
//    // ͨ��UART��������
//    HAL_UART_Transmit(&g_uart1_handle, (uint8_t *)buffer, strlen(buffer), 50);
//}

void send_joint_data_to_pc(int32_t velocity, int32_t position, int32_t torque) {
    static char buffer[64]; 
    int len = snprintf(buffer, sizeof(buffer), "VEL:%d,POS:%d,TOR:%d\n", velocity, position, torque);
	
    if (len > 0 && len < sizeof(buffer)) {
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&g_uart1_handle, (uint8_t*)buffer, len);
        if (status != HAL_OK) {
            printf("DMA Transmission Failed, Error Code: %d\n", status);
        }
    }
}


uint8_t state_choice_func(void)
{
		// ���ü����ٶ�
		if (strncmp((char*)g_usart_rx_buf, "lsv:", 4) == 0)
				return 1;
		// ������չ����λ��
		if (strncmp((char*)g_usart_rx_buf, "ERS:", 4) == 0)
				return 2;
		// ������������λ��
		if (strncmp((char*)g_usart_rx_buf, "IRS:", 4) == 0)
				return 3;
		// �������Ӹ���ѡ��
		
		// ���ص������ѡ��
		return *g_usart_rx_buf;

}


