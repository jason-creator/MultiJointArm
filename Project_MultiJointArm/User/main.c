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

extern DMA_HandleTypeDef  g_dma_handle;     /* DMA句柄 */

int main(void)
{
		JOINT_INFO joint;
    uint8_t key;
    uint8_t t = 0;
		uint8_t flag_active_control = 0;
	
		/* 电机参数设置 */
		joint.acceleration = 5566;
		joint.deceleration = 5566;
		joint.communication_cycle_period = 1000;
		joint.Velocity[Desire] = 5000;
		joint.Torque[Desire] = 80;
		joint.Current_mA[Desire] = 400;
		joint.limited_speed = 43689;
		

    HAL_Init();                                 /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* 设置时钟, 72Mhz */
    delay_init(72);                             /* 延时初始化 */
    usart_init(115200);                         /* 串口初始化为115200 */
    led_init();                                 /* 初始化LED */
    rs485_init(19200);                          /* 初始化RS485,set the baudrate 19200 */
		
		key_init();                                                            /* 初始化按键 */
		can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 2, CAN_MODE_NORMAL); 		/* CAN初始化, 正常模式, 波特率1Mbps, 为实现该波特率，直接将分频系数改为2即可 */
		printf("can init finished!\n");
		
//		Motor_Init_DIY();		/* 电机初始化 */
		Motor_Init(OperatingMode_CyclicVelocity, joint.acceleration, joint.deceleration, joint.Velocity[Desire], joint.communication_cycle_period);
		printf("motor init finished!\n");
		
		delay_ms(20);
		

    while (1)
    {
				key = key_scan(0);

				switch (key)
				{
					case KEY0_PRES:
//							StopMotor_DIY();
							StopMotor();
//							printf("I am here\n");
							break;
					case KEY1_PRES:
//							SetTorque_DIY(joint.Current_mA[Desire], joint.limited_speed);
							flag_active_control = 1;
							joint.Position[Last] = joint.Position[Current];
							joint.Position[DownLimitation] = joint.Position[Last];
							joint.Position[UpLimitation] = joint.Position[DownLimitation] - 131067;
							printf("joint.Position[UpLimitation]: %d\n", joint.Position[UpLimitation]);
							printf("joint.Position[DownLimitation]: %d\n", joint.Position[DownLimitation]);
							SetVelocity(joint.Velocity[Desire]);
//							SetPosition(joint.Position[Last] + 131067);
//							SetTorque(joint.Torque[Desire]);
							printf("Start the control mode\n");
							break;
					case WKUP_PRES:
//							DisableMotor_DIY();
							DisableMotor();
							break;
					default:
							break;
				}
				
				joint.Velocity[Current] = Get_CurrentVelocity();
				joint.Position[Current] = Get_CurrentPosition();
				joint.Torque[Current] = Get_Torque();		
				
				printf("Vel:%d, Pos:%d, Tor:%d\n",joint.Velocity[Current], joint.Position[Current], joint.Torque[Current]);
				
				if (flag_active_control){
					torque2velocity(&joint);
					active_control_speed_limitation(&joint);
				}
				
				t++;
			
				if (t == 30)
				{
						LED1_TOGGLE();  /* LED0闪烁, 提示系统正在运行 */
						t = 0;
				}
				delay_ms(20);
				

    }
}



















