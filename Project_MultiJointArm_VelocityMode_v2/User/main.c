/**
 ****************************************************************************************************
 * @file        main.c
 * @author      LiuZhao
 * @version     V1.0
 * @date        2023-2-26
 * @brief       Multiple Joints Arm 
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/RS485/rs485.h"
#include "./BSP/TorqueSensor/TorqueSensor.h"
#include "./BSP/DMA/dma.h"
#include "./BSP/MOTOR/motor.h"
#include "./BSP/CAN/can.h"
#include "./BSP/CONTROL/control.h"


int main(void)
{
		JOINT_INFO joint;
		uint8_t len;
    uint8_t t = 0;
		uint8_t flag_active_control[3] = {0,0,0};


		uint8_t state_choice = 0;  // 0~255 ״̬������ѡ��
		int limited_speed_buf = 43689;
	
		int ERS_buf = 0;
		int IRS_buf = 0;
	
		/* ����������� */
		joint.acceleration = 5566;
		joint.deceleration = 5566;
		joint.communication_cycle_period = 1000;
	
		joint.Velocity[ElbowBendMotor][Desire] = 0;
		joint.Velocity[ElbowPronationMotor][Desire] = 0;
		joint.Velocity[WristBendMotor][Desire] = 0;
	
		joint.Torque[ElbowBendMotor][Desire] = 80;
		joint.Torque[ElbowPronationMotor][Desire] = 80;
		joint.Torque[WristBendMotor][Desire] = 80;
	
		joint.Current_mA[ElbowBendMotor][Desire] = 400;
		joint.Current_mA[ElbowPronationMotor][Desire] = 400;
		joint.Current_mA[WristBendMotor][Desire] = 400;

		joint.limited_speed[ElbowBendMotor] = 14560;  // 1456 * 10
		joint.limited_speed[ElbowPronationMotor] = 14560;  // 1456 * 10
		joint.limited_speed[WristBendMotor] = 1781;  // 890.4 * 2
		
		joint.range[ElbowBendMotor] = 174763;  // 120
		joint.range[ElbowPronationMotor] = 131067;  // 90
		joint.range[WristBendMotor] = 26712;  // 30
		
		joint.PID[ElbowBendMotor][Kp] = -0.4;
		joint.PID[ElbowBendMotor][Ki] = -0.01;
		joint.PID[ElbowBendMotor][Kd] = 0.0;
		joint.last_error = 0.0;
		joint.integral = 0.0;


    HAL_Init();                                 /* ��ʼ��HAL�� */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* ����ʱ��, 72Mhz */
    delay_init(72);                             /* ��ʱ��ʼ�� */
    usart_init(115200);                         /* ���ڳ�ʼ��Ϊ115200 */
    led_init();                                 /* ��ʼ��LED */
    rs485_init(19200);                          /* ��ʼ��RS485,set the baudrate 19200 */
		
		key_init();                                                            /* ��ʼ������ */
		can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 2, CAN_MODE_NORMAL); 		/* CAN��ʼ��, ����ģʽ, ������1Mbps, Ϊʵ�ָò����ʣ�ֱ�ӽ���Ƶϵ����Ϊ2���� */
//		printf("can init finished!\n");
		
//		delay_ms(20);
		

    while (1)
    {
			if (g_usart_rx_sta & 0x8000) 
			{
					len = g_usart_rx_sta & 0x3fff;  /* �õ��˴ν��յ������ݳ��� */
					HAL_UART_Transmit(&g_uart1_handle,(uint8_t*)g_usart_rx_buf, len, 50);    /* ���ͽ��յ������� */
					while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC) != SET);           /* �ȴ����ͽ��� */
					
					printf("\r\n");             /* ���뻻�� */
				
					g_usart_rx_buf[len] = '\0';  // ���ý��ջ��������һ��Ԫ��Ϊ'\0'�������
				
					state_choice = state_choice_func();  // ״̬������

					switch (state_choice)
					{
						// ���ü����ٶ�
						case 1:
								sscanf((char*)g_usart_rx_buf+4, "%u", &(limited_speed_buf));  // +4Ϊ������ "lsv:" �ĸ��ַ�������ȡʣ��ֵ����limited_speed_buf��
								joint.limited_speed[ElbowBendMotor] = 1456*limited_speed_buf;
								break;
						// ������չ����λ��
						case 2:
								sscanf((char*)g_usart_rx_buf+4, "%u", &(ERS_buf));  // +4Ϊ������ "ERS:" �ĸ��ַ�������ȡʣ��ֵ����ERS_buf��
								joint.Position[ElbowBendMotor][DownLimitation] = joint.Position[ElbowBendMotor][DownLimitation] + 1456*ERS_buf;
								break;
						//������������λ��
						case 3:
								sscanf((char*)g_usart_rx_buf+4, "%u", &(IRS_buf));  // +4Ϊ������ "IRS:" �ĸ��ַ�������ȡʣ��ֵ����IRS_buf��
								joint.Position[ElbowBendMotor][UpLimitation] = joint.Position[ElbowBendMotor][UpLimitation] - 1456*IRS_buf;
								break;
						
				
						/* �ֱ�ʹ��a,b,c,d		����GBK���룬ת��Ϊuint16_tΪ 97,98,99,100 */
						// ���1��ʼ��
						case 97: 									
								Motor_Init(ID_ElbowBendMotor, OperatingMode_CyclicVelocity, joint.acceleration, joint.deceleration, joint.Velocity[ElbowBendMotor][Desire], joint.communication_cycle_period);
//								printf("motor init finished!\n");
								break;
						// ���1ʹ��
						case 98:
								flag_active_control[ElbowBendMotor] = 1;
								joint.Position[ElbowBendMotor][Last] = joint.Position[ElbowBendMotor][Current];
//								joint.Position[ElbowBendMotor][DownLimitation] = joint.Position[ElbowBendMotor][Last];
								joint.Position[ElbowBendMotor][DownLimitation] = 77600;
								joint.Position[ElbowBendMotor][UpLimitation] = joint.Position[ElbowBendMotor][DownLimitation] - joint.range[ElbowBendMotor];
//								joint.Position[ElbowBendMotor][DownLimitation] = 696510;
//								joint.Position[ElbowBendMotor][UpLimitation] = 435000;
//								printf("joint.Position[ElbowBendMotor][UpLimitation]: %d\n", joint.Position[ElbowBendMotor][UpLimitation]);
//								printf("joint.Position[ElbowBendMotor][DownLimitation]: %d\n", joint.Position[ElbowBendMotor][DownLimitation]);
								SetVelocity(ID_ElbowBendMotor, joint.Velocity[ElbowBendMotor][Desire]);
//								printf("Start the control mode\n");
								break;
						// ���1ʧ��
						case 99:
								DisableMotor(ID_ElbowBendMotor);
								flag_active_control[ElbowBendMotor] = 0;
								break;
						// ���1ͣת
						case 100:
								StopMotor(ID_ElbowBendMotor);
								break;

						/* �ֱ�ʹ��e,f,g,h		����GBK���룬ת��Ϊuint16_tΪ 101,102,103,104 */
						// ���2��ʼ��
						case 101: 									
								Motor_Init(ID_ElbowPronationMotor, OperatingMode_CyclicVelocity, joint.acceleration, joint.deceleration, joint.Velocity[ElbowPronationMotor][Desire], joint.communication_cycle_period);
//								printf("motor init finished!\n");
								break;
						// ���2ʹ��
						case 102:
								flag_active_control[ElbowPronationMotor] = 1;
								joint.Position[ElbowPronationMotor][Last] = joint.Position[ElbowPronationMotor][Current];
								joint.Position[ElbowPronationMotor][DownLimitation] = joint.Position[ElbowPronationMotor][Last];
						
								joint.Position[ElbowPronationMotor][UpLimitation] = joint.Position[ElbowPronationMotor][DownLimitation] - 0.5*joint.range[ElbowPronationMotor];
								joint.Position[ElbowPronationMotor][DownLimitation] = joint.Position[ElbowPronationMotor][DownLimitation] + 0.5*joint.range[ElbowPronationMotor];
						
//								printf("joint.Position[ElbowPronationMotor][UpLimitation]: %d\n", joint.Position[ElbowPronationMotor][UpLimitation]);
//								printf("joint.Position[ElbowPronationMotor][DownLimitation]: %d\n", joint.Position[ElbowPronationMotor][DownLimitation]);
								SetVelocity(ID_ElbowPronationMotor, joint.Velocity[ElbowPronationMotor][Desire]);
//								printf("Start the control mode\n");
								break;
						// ���2ʧ��
						case 103:
								DisableMotor(ID_ElbowPronationMotor);
								flag_active_control[ElbowPronationMotor] = 0;
								break;
						// ���2ͣת
						case 104:
								StopMotor(ID_ElbowPronationMotor);
								break;
						
						/* �ֱ�ʹ��i,j,k,l		����GBK���룬ת��Ϊuint16_tΪ 105,106,107,108 */
						// ���3��ʼ��
						case 105: 									
								Motor_Init_Maxon(OperatingMode_ProfileVelocity, joint.acceleration, joint.deceleration);
//								printf("motor init finished!\n");
								break;
						// ���3ʹ��
						case 106:
								flag_active_control[WristBendMotor] = 1;
								joint.Position[WristBendMotor][Last] = joint.Position[WristBendMotor][Current];
								joint.Position[WristBendMotor][DownLimitation] = joint.Position[WristBendMotor][Last];
								joint.Position[WristBendMotor][UpLimitation] = joint.Position[WristBendMotor][DownLimitation] + joint.range[WristBendMotor];  // Maxon 360du = 4000*80.14, 40070 = 45du
//								printf("joint.Position[WristBendMotor][UpLimitation]: %d\n", joint.Position[WristBendMotor][UpLimitation]);
//								printf("joint.Position[WristBendMotor][DownLimitation]: %d\n", joint.Position[WristBendMotor][DownLimitation]);
								ProfileVelocity(joint.Velocity[WristBendMotor][Desire]);
//								printf("Start the control mode\n");
								break;
						// ���3ʧ��
						case 107:
								DisableMotor_Maxon();
								flag_active_control[WristBendMotor] = 0;
								break;
						// ���3ͣת
						case 108:
								StopMotor_Maxon();
								break;
						
						
						
						default:
								break;
						}

						g_usart_rx_sta = 0;
			}	
			
				joint.Position[ElbowBendMotor][Current] = Get_CurrentPosition(ID_ElbowBendMotor);
				joint.Torque[ElbowBendMotor][Current] = Get_Torque(ID_ElbowBendSensor);	
				joint.Velocity[ElbowBendMotor][Current] = Get_CurrentVelocity(ID_ElbowBendMotor);
			
//				joint.Position[ElbowPronationMotor][Current] = Get_CurrentPosition(ID_ElbowPronationMotor);
//				joint.Torque[ElbowPronationMotor][Current] = Get_Torque(ID_ElbowPronationSensor);
//			  joint.Velocity[ElbowPronationMotor][Current] = Get_CurrentVelocity(ID_ElbowPronationMotor);
			
//				joint.Velocity[WristBendMotor][Current] = Get_CurrentVelocity_Maxon();
//				joint.Position[WristBendMotor][Current] = Get_CurrentPosition_Maxon();
//				joint.Torque[WristBendMotor][Current] = Get_Torque_SBT(ID_WristBendMotor);

			
//				printf("joint.Position[ElbowBendMotor][UpLimitation]: %d\n", joint.Position[ElbowBendMotor][UpLimitation]);
//				printf("joint.Position[ElbowBendMotor][DownLimitation]: %d\n", joint.Position[ElbowBendMotor][DownLimitation]);
				
				if (flag_active_control[ElbowBendMotor]){
					torque2velocity(ElbowBendMotor, &joint);
					active_control_position_limitation(ID_ElbowBendMotor, ElbowBendMotor, &joint);
				}
				
//				if (flag_active_control[ElbowPronationMotor]){
//					torque2velocity(ElbowPronationMotor, &joint);
//					active_control_position_limitation(ID_ElbowPronationMotor, ElbowPronationMotor, &joint);
//				}
				
//				if (flag_active_control[WristBendMotor]){
//					torque2velocity_Maxon(WristBendMotor, &joint);
//					active_control_position_limitation_Maxon(WristBendMotor, &joint);
//				}
				
//				printf("MOTOR1 p: %d, v: %d, T: %d\n", joint.Position[ElbowBendMotor][Current], joint.Velocity[ElbowBendMotor][Current], joint.Torque[ElbowBendMotor][Current]);
//				printf("MOTOR2 p: %d, v: %d, T: %d\n", joint.Position[ElbowPronationMotor][Current], joint.Velocity[ElbowPronationMotor][Current], joint.Torque[ElbowPronationMotor][Current]);
//				printf("MOTOR3 p: %d, v: %d, T: %d\n", joint.Position[WristBendMotor][Current], joint.Velocity[WristBendMotor][Current], joint.Torque[WristBendMotor][Current]);

				send_joint_data_to_pc(joint.Velocity[ElbowBendMotor][Current], joint.Position[ElbowBendMotor][Current], joint.Torque[ElbowBendMotor][Current]);

				
				t++;
			
				if (t == 10)
				{
						LED1_TOGGLE();  /* LED0��˸, ��ʾϵͳ�������� */
						t = 0;
				}
				
//				delay_ms(15);
//				delay_ms(5);
				

    }
}



















