/**
 ****************************************************************************************************
 * @file        TorqueSensor.c
 * @author      LiuZhao
 * @version     V1.0
 * @date        2023-2-21
 * @brief       RS485-TorqueSensor
 ****************************************************************************************************
 */
 
 #include "./BSP/TorqueSensor/TorqueSensor.h"


 
int Get_Torque(void)
{
	  uint8_t CommandData[8];
		uint8_t ReceiveData[9];
		uint8_t res = 1;
		int TorqueValue = 0;
		uint16_t crc;
	
	 
		// Sending
		CommandData[0]=0x01; 
		CommandData[1]=0x03; 
		CommandData[2]=0x06; 
		CommandData[3]=0x06; 
		CommandData[4]=0x00;
		CommandData[5]=0x02; 
		
	 // CRC校验值计算
		crc = ModbusCRC_CheckTable(CommandData, 6);
//		printf("The crc is: %d\n", crc);
		
		CommandData[6]=crc & 0x00ff;   
		CommandData[7]=crc >> 8;	
		
		rs485_send_data(CommandData, 8);		
		
		rs485_receive_data(ReceiveData, &res);
		if (res)
		{
			TorqueValue = (ReceiveData[3] << 24 | ReceiveData[4] << 16)|(ReceiveData[5] << 8 | ReceiveData[6]);
			return TorqueValue;
		}
		else
		{
			printf("No received data\n");
			return 0;
		}

		
} 







