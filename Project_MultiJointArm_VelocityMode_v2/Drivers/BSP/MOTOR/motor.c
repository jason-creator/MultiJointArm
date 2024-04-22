/**
 ****************************************************************************************************
 * @file        motor.c
 * @author      LiuZhao
 * @version     V1.0
 * @date        2023-2-16
 * @brief       Motor controller for ZeroErr eRob70H80I-BM-18CN
 ****************************************************************************************************
 */
 
 #include "./BSP/MOTOR/motor.h"
 
 uint8_t Motor_Init_DIY(void)
 {
	 uint8_t res;
	 uint8_t CommandData[6];
	 
 		CommandData[0]=0x00; 
		CommandData[1]=0x4E; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x01; 
		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		printf("stp1: %d", res);
	 
 		CommandData[0]=0x01; 
		CommandData[1]=0x12; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x00; 
		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		printf("stp2: %d", res);
	 
 		CommandData[0]=0x01; 
		CommandData[1]=0xFD; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x00; 
		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		printf("stp3: %d", res);

 		CommandData[0]=0x01; 
		CommandData[1]=0xFE; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x01;
		CommandData[5]=0x2C;
		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		printf("stp4: %d", res);
	 
	 // set the limited speed
	 	CommandData[0]=0x02; 
		CommandData[1]=0x04; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x75;
		CommandData[5]=0x30;
		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		printf("stp5: %d", res);
	 
		// enable
 		CommandData[0]=0x01; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x01; 
		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		printf("stp6: %d\n", res);


		
		return 0;
 }
 
 uint8_t Motor_Init_SDO(uint8_t mode, uint16_t acceleration, uint16_t deceleration, int speed, uint16_t communication_cycle_period)
 {
		uint8_t res;
	  uint8_t CommandData[8];
	 
 		CommandData[0]=0x2F; 
		CommandData[1]=0x60; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=mode;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_ElbowBendMotor, CommandData, 8);		
	 
 		CommandData[0]=0x40; 
		CommandData[1]=0x61; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_ElbowBendMotor, CommandData, 8);		

 		CommandData[0]=0x2B; 
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0x80;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_ElbowBendMotor, CommandData, 8);		
		
 		CommandData[0]=0x2B; 
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0x06;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_ElbowBendMotor, CommandData, 8);	
		
 		CommandData[0]=0x2B; 
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0x07;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_ElbowBendMotor, CommandData, 8);	
		
 		CommandData[0]=0x2B; 
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0x0F;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_ElbowBendMotor, CommandData, 8);	
 
		return res;
 }

 
 uint8_t Motor_Init(uint8_t motor_id, uint8_t mode, uint16_t acceleration, uint16_t deceleration, int speed, uint16_t communication_cycle_period)
 {
		uint8_t res;
	  uint8_t CommandData[8];
	 
//		delay_ms(10);
	 
		// Stop remote node
		CommandData[0]=0x02; 
		CommandData[1]=motor_id; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x00, CommandData, 2);		// res = 0, success
//		printf("The stop remote node result: %d\n", res);
	 
		// Reset communication
		CommandData[0]=0x82; 
		CommandData[1]=motor_id; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x00, CommandData, 2);		
//		printf("The Reset communication result: %d\n", res);

		// Set operating Mode
		CommandData[0]=0x2F; 
		CommandData[1]=0x60; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=mode;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Set operating Mode result: %d\n", res);
		

		// Check operating mode
		CommandData[0]=0x40; 
		CommandData[1]=0x61; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Check operating mode result: %d\n", res);
		
		if (mode!=OperatingMode_ProfileTorque || mode!=OperatingMode_CyclicTorque)
		{
				// Set initial speed 
				if (mode==OperatingMode_CyclicPosition || mode==OperatingMode_ProfilePosition)
				{
					CommandData[0]=0x23; 
					CommandData[1]=0x81; 
					CommandData[2]=0x60; 
					CommandData[3]=0x00; 
					CommandData[4]=speed & 0XFF;						
					CommandData[5]=(speed >> 8) & 0XFF; 		
					CommandData[6]=(speed >> 16) & 0XFF; 
					CommandData[7]=(speed >> 24) & 0XFF;
					res = can_send_msg(0x600+motor_id, CommandData, 8);
//					printf("The Set initial speed result: %d\n", res);
				}
				
				// Set initial acceleration 
				CommandData[0]=0x23; 
				CommandData[1]=0x83; 
				CommandData[2]=0x60; 
				CommandData[3]=0x00; 
				CommandData[4]=acceleration & 0XFF;						// low 8 
				CommandData[5]=(acceleration >> 8) & 0XFF; 		// high 8
				CommandData[6]=0x00; 
				CommandData[7]=0x00;	
				res = can_send_msg(0x600+motor_id, CommandData, 8);
//				printf("The Set initial acceleration result: %d\n", res);
				
				// Set initial deceleration 
				CommandData[0]=0x23; 
				CommandData[1]=0x84; 
				CommandData[2]=0x60; 
				CommandData[3]=0x00; 
				CommandData[4]=deceleration & 0XFF;						// low 8 
				CommandData[5]=(deceleration >> 8) & 0XFF; 		// high 8
				CommandData[6]=0x00; 
				CommandData[7]=0x00;	
				res = can_send_msg(0x600+motor_id, CommandData, 8);
//				printf("The Set initial deceleration result: %d\n", res);
		
		}

		
		// Disable cob-id sync
		CommandData[0]=0x23; 
		CommandData[1]=0x05; 
		CommandData[2]=0x10; 
		CommandData[3]=0x00; 
		CommandData[4]=0x80;			
		CommandData[5]=0x00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Disable cob-id sync result: %d\n", res);
		
		// Set communication cycle period
		CommandData[0]=0x23; 
		CommandData[1]=0x06; 
		CommandData[2]=0x10; 
		CommandData[3]=0x00; 
		CommandData[4]=communication_cycle_period & 0XFF;			
		CommandData[5]=(communication_cycle_period >> 8) & 0XFF; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Set communication cycle period result: %d\n", res);
		
		// Disable TPDO
		CommandData[0]=0x23; 
		CommandData[1]=0x00; 
		CommandData[2]=0x18; 
		CommandData[3]=0x01; 
		CommandData[4]=0X80+motor_id;			
		CommandData[5]=0X01; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x80;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Disable TPDO result: %d\n", res);
		
		// Defines the transmission type
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x18; 
		CommandData[3]=0x02; 
		CommandData[4]=0X01;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Defines the transmission type result: %d\n", res);
		
		
		// Defines the number of valid entries in the mapping record
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x1A; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Defines the number of valid result: %d\n", res);
		
		
		// 1A00h-01h maps 60410010h(status word)
		CommandData[0]=0x23; 
		CommandData[1]=0x00; 
		CommandData[2]=0x1A; 
		CommandData[3]=0x01; 
		CommandData[4]=0X10;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x41; 
		CommandData[7]=0x60;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The 1A00h-01h map result: %d\n", res);

		// 1A00h-02h maps 606C0020h(actual position, speed, torque)
		switch (mode)
		{
			case OperatingMode_ProfilePosition: case OperatingMode_CyclicPosition:	
				CommandData[0]=0x23; 
				CommandData[1]=0x00; 
				CommandData[2]=0x1A; 
				CommandData[3]=0x02; 
				CommandData[4]=0X20;			
				CommandData[5]=0X00; 
				CommandData[6]=0x64; 
				CommandData[7]=0x60;
				res = can_send_msg(0x600+motor_id, CommandData, 8);
//				printf("The 1A00h-02h maps result: %d\n", res);
				break;		
			case OperatingMode_ProfileVelocity: case OperatingMode_CyclicVelocity:
				CommandData[0]=0x23; 
				CommandData[1]=0x00; 
				CommandData[2]=0x1A; 
				CommandData[3]=0x02; 
				CommandData[4]=0X20;			
				CommandData[5]=0X00; 
				CommandData[6]=0x6C; 
				CommandData[7]=0x60;
				res = can_send_msg(0x600+motor_id, CommandData, 8);
//				printf("The 1A00h-02h maps result: %d\n", res);
				break;		
			case OperatingMode_ProfileTorque:	case OperatingMode_CyclicTorque:
				CommandData[0]=0x23; 
				CommandData[1]=0x00; 
				CommandData[2]=0x1A; 
				CommandData[3]=0x02; 
				CommandData[4]=0X10;			
				CommandData[5]=0X00; 
				CommandData[6]=0x77; 
				CommandData[7]=0x60;
				res = can_send_msg(0x600+motor_id, CommandData, 8);
//				printf("The 1A00h-02h maps result: %d\n", res);
			
				// actual current
				CommandData[0]=0x23; 
				CommandData[1]=0x00; 
				CommandData[2]=0x1A; 
				CommandData[3]=0x03; 
				CommandData[4]=0X10;			
				CommandData[5]=0X00; 
				CommandData[6]=0x78; 
				CommandData[7]=0x60;
				res = can_send_msg(0x600+motor_id, CommandData, 8);
//				printf("The 1A00h-02h maps result: %d\n", res);
				break;
			
			default:
				break;
		}

		
		// 1A00h-00h:the number of valid entries in the mapping record:2/3
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x1A; 
		CommandData[3]=0x00; 
		if (mode==OperatingMode_ProfileTorque || mode==OperatingMode_CyclicTorque){
			CommandData[4]=0X03;
		}
		else{
			CommandData[4]=0X02;
		}
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The the number of valid entries result: %d\n", res);
		
		// Enable TPDO
		CommandData[0]=0x23; 
		CommandData[1]=0x00; 
		CommandData[2]=0x18; 
		CommandData[3]=0x01; 
		CommandData[4]=0X80+motor_id;			
		CommandData[5]=0X01; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Enable TPDO result: %d\n", res);

		// Disable RPDO
		CommandData[0]=0x23; 
		CommandData[1]=0x00; 
		CommandData[2]=0x14; 
		CommandData[3]=0x01; 
		CommandData[4]=motor_id;			
		CommandData[5]=0X02; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x80;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Disable RPDO result: %d\n", res);
		
		// Defines the transmission type
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x14; 
		CommandData[3]=0x02; 
		CommandData[4]=0X01;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Defines the transmission type result: %d\n", res);
		
		// Defines the number of valid entries in the mapping record
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x16; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Defines the number of valid entries result: %d\n", res);
		
		// 1600h-01h maps 60400010h(control word)
		CommandData[0]=0x23; 
		CommandData[1]=0x00; 
		CommandData[2]=0x16; 
		CommandData[3]=0x01; 
		CommandData[4]=0X10;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x40; 
		CommandData[7]=0x60;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The control word result: %d\n", res);
		
		// 1600h-02h maps 60FF0020h(target position, speed, torque)
		switch (mode)
		{
			case OperatingMode_ProfilePosition: case OperatingMode_CyclicPosition:	
				CommandData[0]=0x23; 
				CommandData[1]=0x00; 
				CommandData[2]=0x16; 
				CommandData[3]=0x02; 
				CommandData[4]=0X20;			
				CommandData[5]=0X00; 
				CommandData[6]=0x7A; 
				CommandData[7]=0x60;
				break;		
			case OperatingMode_ProfileVelocity: case OperatingMode_CyclicVelocity:
				CommandData[0]=0x23; 
				CommandData[1]=0x00; 
				CommandData[2]=0x16; 
				CommandData[3]=0x02; 
				CommandData[4]=0X20;			
				CommandData[5]=0X00; 
				CommandData[6]=0xFF; 
				CommandData[7]=0x60;
				break;		
			case OperatingMode_ProfileTorque:	case OperatingMode_CyclicTorque:
				CommandData[0]=0x23; 
				CommandData[1]=0x00; 
				CommandData[2]=0x16; 
				CommandData[3]=0x02; 
				CommandData[4]=0X10;			
				CommandData[5]=0X00; 
				CommandData[6]=0x71; 
				CommandData[7]=0x60;
				break;
			
			default:
				break;
		}	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The 1600h-02h maps result: %d\n", res);
		
		// 1600h-00h:the number of valid entries in the mapping record:2
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x16; 
		CommandData[3]=0x00; 
		CommandData[4]=0X02;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The mapping record result: %d\n", res);
		
		// Ensable RPDO
		CommandData[0]=0x23; 
		CommandData[1]=0x00; 
		CommandData[2]=0x14; 
		CommandData[3]=0x01; 
		CommandData[4]=motor_id;			
		CommandData[5]=0X02; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+motor_id, CommandData, 8);
//		printf("The Ensable RPDO result: %d\n", res);
		
		// Start remote node
		CommandData[0]=0x01; 
		CommandData[1]=motor_id; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x00, CommandData, 2);
//		printf("The Start remote node result: %d\n", res);
		
		// Heartbeat command
		CommandData[0]=0x00; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x700+motor_id, CommandData, 8);
//		res = can_receive_msg(0x700+motor_id, CommandData);  // heartbeat detection
//		printf("The Heartbeat is: %d\n", CommandData[0]);
		
		// Sync command
		CommandData[0]=0x00; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x80, CommandData, 8);
//		res = can_receive_msg(0x180+motor_id, CommandData);  // sync
//		printf("The Sync command result: %d\n", res);
		
		
		CommandData[0]=0x80; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x200+motor_id, CommandData, 8);
//		printf("The result: %d\n", res);
		
		CommandData[0]=0x06; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x200+motor_id, CommandData, 8);
//		printf("The Disable TPDO result: %d\n", res);
		
		CommandData[0]=0x07; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x200+motor_id, CommandData, 8);
//		printf("The result: %d\n", res);
		
		// Enable
		CommandData[0]=0x0F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x200+motor_id, CommandData, 8);
//		printf("The Enable result: %d\n", res);
		
		return res;
 }
 
  uint8_t Motor_Init_Maxon(uint8_t mode, uint16_t acceleration, uint16_t deceleration)
 {
		uint8_t res;
	  uint8_t CommandData[8];
	
		// Set operating Mode
		CommandData[0]=0x2F; 
		CommandData[1]=0x60; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=mode;
		CommandData[5]=0x00; 
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		printf("The Set operating Mode result: %d\n", res);

		
		// Set initial acceleration 
		CommandData[0]=0x2B;
		CommandData[1]=0x83; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=acceleration & 0XFF;						// low 8 
		CommandData[5]=(acceleration >> 8) & 0XFF; 		// high 8
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		printf("The Set initial acceleration result: %d\n", res);
		
		// Set initial deceleration 
		CommandData[0]=0x2B;
		CommandData[1]=0x84; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=deceleration & 0XFF;						// low 8 
		CommandData[5]=(deceleration >> 8) & 0XFF; 		// high 8
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		printf("The Set initial deceleration result: %d\n", res);
		
		// Enable
		CommandData[0]=0x2B;
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0X06;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		printf("The Enable result: %d\n", res);
		
		CommandData[0]=0x2B;
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0X0F;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		printf("The Enable result: %d\n", res);
		
		return 0;
 }

uint8_t SetVelocity(uint8_t motor_id, int speed)
{
		uint8_t CommandData[8];
		uint8_t res;
	 
	
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=speed & 0XFF; 
		CommandData[3]=(speed >> 8) & 0XFF; 
		CommandData[4]=(speed >> 16) & 0XFF;			
		CommandData[5]=(speed >> 24) & 0XFF;
		CommandData[6]=0x00; 
		CommandData[7]=0x00;

		res = can_send_msg(0x200+motor_id, CommandData, 6);
//		printf("Velocity result: %d\n", res);
		return res;
 }
 
 
  uint8_t ProfileVelocity(int speed)
 {
		uint8_t CommandData[8];
		uint8_t res;
	 
	 
//		CommandData[0]=0x2F; 
		CommandData[0]=0x23; 
		CommandData[1]=0xFF; 
		CommandData[2]=0X60; 
		CommandData[3]=0X00; 
		CommandData[4]=speed & 0XFF; 			
		CommandData[5]=(speed >> 8) & 0XFF; 	
		CommandData[6]=(speed >> 16) & 0XFF; 
		CommandData[7]=(speed >> 24) & 0XFF;
//		res = can_send_msg(0x200+ID_WristBendMotor, CommandData, 6);
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
//		printf("Profile velocity result: %d\n", res);
	 
		delay_ms(5);
	 
//		CommandData[0]=0x0F; 
	 	CommandData[0]=0x2B;
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0X0F;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
//		res = can_send_msg(0x200+ID_WristBendMotor, CommandData, 8);
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		return res;
 }
 
 
 uint8_t SetPosition(uint8_t motor_id, int position)
{
		uint8_t CommandData[8];
		uint8_t res;
	 
	
		CommandData[0]=0x1F; 
		CommandData[1]=0x00; 
		CommandData[2]=position & 0XFF; 
		CommandData[3]=(position >> 8) & 0XFF; 
		CommandData[4]=(position >> 16) & 0XFF;			
		CommandData[5]=(position >> 24) & 0XFF;
		CommandData[6]=0x00; 
		CommandData[7]=0x00;

		res = can_send_msg(0x200+motor_id, CommandData, 6);
//		printf("Position result: %d\n", res);
		return res;
 }

 
 uint8_t SetTorque(uint8_t motor_id, int torque)
{
		uint8_t CommandData[8];
		uint8_t res;
	 
	
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=torque & 0XFF; 
		CommandData[3]=(torque >> 8) & 0XFF; 
		CommandData[4]=(torque >> 16) & 0XFF;			
		CommandData[5]=(torque >> 24) & 0XFF;
		CommandData[6]=0x00; 
		CommandData[7]=0x00;

		res = can_send_msg(0x200+motor_id, CommandData, 6);
//		printf("Torque result: %d\n", res);
		return res;
 }

 uint8_t SetTorque_DIY(int current, uint32_t limited_speed)
 {
		uint8_t res;
		uint8_t CommandData[8];
	 	 
	 
	 // set the target current
 		CommandData[0]=0x01; 
		CommandData[1]=0xFE; 
		CommandData[2]=(current >> 24) & 0XFF; 
		CommandData[3]=(current >> 16) & 0XFF; 
		CommandData[4]=(current >> 8) & 0XFF;
		CommandData[5]=current & 0XFF; 

		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
	 
	 // set the limited speed
	 	CommandData[0]=0x02; 
		CommandData[1]=0x04; 
		CommandData[2]=(limited_speed >> 24) & 0XFF; 
		CommandData[3]=(limited_speed >> 16) & 0XFF; 
		CommandData[4]=(limited_speed >> 8) & 0XFF;
		CommandData[5]=limited_speed & 0XFF;

		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
	 
		// enable
 		CommandData[0]=0x01; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0x00;
		CommandData[5]=0x01; 
		res=can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
 
		return res;
 }

 uint8_t SetTorque_SDO(int torque)
{
		uint8_t CommandData[8];
		uint8_t res;
	 
	
		CommandData[0]=0x2B; 
		CommandData[1]=0x71; 
		CommandData[2]=0x60;
		CommandData[3]=0x00;
		CommandData[4]=0x50;
		CommandData[5]=0x00;
		CommandData[6]=0x00;	
		CommandData[7]=0x00;

		res = can_send_msg(0x600+ID_ElbowBendMotor, CommandData, 6);
//		printf("Torque result: %d\n", res);
		return res;
 }

 
 uint8_t StopMotor(uint8_t motor_id)
{
		uint8_t CommandData[8];
		uint8_t res;
	 
		CommandData[0]=0x2F; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;
		res = can_send_msg(0x200+motor_id, CommandData, 6);
		
		return res;
}

 uint8_t StopMotor_Maxon(void)
{
		uint8_t CommandData[8];
		uint8_t res;
	 
		CommandData[0]=0x2B;
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0X0F;			
		CommandData[5]=0X01; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
	
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		
		return res;
}

 uint8_t StopMotor_DIY(void)
{
		uint8_t res;
		uint8_t CommandData[6];
	 
		CommandData[0]=0x01; 
		CommandData[1]=0xFE; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	

		res = can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		
		return res;
}

uint8_t DisableMotor(uint8_t motor_id)
{
		uint8_t CommandData[8];
		uint8_t res;
	 
		CommandData[0]=0x06; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;
		res = can_send_msg(0x200+motor_id, CommandData, 6);
		
		return res;
}

uint8_t DisableMotor_Maxon(void)
{
 		uint8_t CommandData[8];
		uint8_t res;
	 
		CommandData[0]=0x2B;
		CommandData[1]=0x40; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	
		CommandData[6]=0x00; 
		CommandData[7]=0x00;	
	 
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);
		
		return res;

}

uint8_t DisableMotor_DIY(void)
{
		uint8_t CommandData[6];
		uint8_t res;
	 
		CommandData[0]=0x01; 
		CommandData[1]=0x00; 
		CommandData[2]=0x00; 
		CommandData[3]=0x00; 
		CommandData[4]=0X00;			
		CommandData[5]=0X00; 	

		res = can_send_msg(0x640+ID_ElbowBendMotor, CommandData, 6);
		
		return res;
}
 
//uint32_t Get_CurrentPosition(uint8_t motor_id)
//{
//		uint8_t res;
//		uint8_t CommandData[8];
//		int Current_position = 0;
//	
//		CommandData[0]=0x40; 
//		CommandData[1]=0x64; 
//		CommandData[2]=0x60; 
//		CommandData[3]=0x00;
//		CommandData[4]=0x00;	       
//		CommandData[5]=0x00; 
//		CommandData[6]=0x00;	
//		CommandData[7]=0x00;	

//		res = can_send_msg(0x600+motor_id, CommandData, 8);	
//		delay_ms(20);
//		res = can_receive_msg(0x0580+motor_id, CommandData);
//		
//		if (res != 0)
//		{
//			Current_position =  (CommandData[7] << 24 | CommandData[6] << 16) | (CommandData[5] << 8 | CommandData[4]);

//		}
//		
//		return Current_position;
//}

int Get_CurrentPosition(uint8_t motor_id)
{
	uint8_t res;
	uint8_t CommandData[8];
	int Current_position = 0;
	int timeout = 5;  // for example, 100 ms
	
	CommandData[0]=0x40; 
	CommandData[1]=0x64; 
	CommandData[2]=0x60; 
	CommandData[3]=0x00;
	CommandData[4]=0x00;	       
	CommandData[5]=0x00; 
	CommandData[6]=0x00;	
	CommandData[7]=0x00;	

	can_send_msg(0x600+motor_id, CommandData, 8);

	do {
		delay_ms(1);  // Wait 10 ms each loop iteration
		res = can_receive_msg(0x0580+motor_id, CommandData);
	} while ((res == 0 || CommandData[0] != 0x43 || CommandData[1] != 0x64 || CommandData[2] != 0x60) && --timeout > 0);

	if (timeout > 0 && res != 0)
	{
		Current_position = (CommandData[7] << 24 | CommandData[6] << 16) | (CommandData[5] << 8 | CommandData[4]);
	}
	
	return Current_position;
}

int Get_CurrentVelocity(uint8_t motor_id)
{
	uint8_t res;
	uint8_t CommandData[8];
	int Current_velocity = 0;
	int timeout = 5;  // for example, 100 ms
	
	CommandData[0]=0x40; 
	CommandData[1]=0x69; 
	CommandData[2]=0x60; 
	CommandData[3]=0x00;
	CommandData[4]=0x00;	       
	CommandData[5]=0x00; 
	CommandData[6]=0x00;	
	CommandData[7]=0x00;	

	can_send_msg(0x600+motor_id, CommandData, 8);

	do {
		delay_ms(1);  // Wait 1 ms each loop iteration
		res = can_receive_msg(0x0580+motor_id, CommandData);
	} while ((res == 0 || CommandData[0] != 0x43 || CommandData[1] != 0x69 || CommandData[2] != 0x60) && --timeout > 0);

	if (timeout > 0 && res != 0)
	{
		Current_velocity = (CommandData[7] << 24 | CommandData[6] << 16) | (CommandData[5] << 8 | CommandData[4]);
	}
	
	return Current_velocity;
}



int Get_CurrentPosition_Maxon(void)
{
		uint8_t res;
		uint8_t CommandData[8];
		int Current_position = 0;
		int timeout = 100;  // for example, 100 ms
	
		CommandData[0]=0x40; 
		CommandData[1]=0x64; 
		CommandData[2]=0x60; 
		CommandData[3]=0x00;
		CommandData[4]=0x00;	       
		CommandData[5]=0x00; 
		CommandData[6]=0x00;	
		CommandData[7]=0x00;	

		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);	
		do {
			delay_ms(5);  // Wait 10 ms each loop iteration
			res = can_receive_msg(0x0580+ID_WristBendMotor, CommandData);
		} while ((res == 0 || CommandData[0] != 0x43 || CommandData[1] != 0x64 || CommandData[2] != 0x60) && --timeout > 0);

		if (timeout > 0 && res != 0)
		{
			Current_position = (CommandData[7] << 24 | CommandData[6] << 16) | (CommandData[5] << 8 | CommandData[4]);
//			printf("Current position:%d \n", Current_position); 
		}
		
		
		return Current_position;
}


int Get_CurrentVelocity_Maxon(void)
{
		uint8_t res;
		uint8_t CommandData[8];
		int Current_velocity = 0;
		int timeout = 100;  // for example, 100 ms
	
		CommandData[0]=0x40; 
		CommandData[1]=0xD3; 
		CommandData[2]=0x30; 
		CommandData[3]=0x01;
		CommandData[4]=0x00;	       
		CommandData[5]=0x00; 
		CommandData[6]=0x00;	
		CommandData[7]=0x00;	
		
		res = can_send_msg(0x600+ID_WristBendMotor, CommandData, 8);	
		do {
			delay_ms(5);  // Wait 10 ms each loop iteration
			res = can_receive_msg(0x0580+ID_WristBendMotor, CommandData);
		} while ((res == 0 || CommandData[1] != 0xD3 || CommandData[2] != 0x30) && --timeout > 0);

		if (timeout > 0 && res != 0)
		{
			Current_velocity = (CommandData[7] << 24 | CommandData[6] << 16) | (CommandData[5] << 8 | CommandData[4]);
//			printf("Current velocity:%d \n", Current_velocity); 
		}
		
		
		return Current_velocity;
}




