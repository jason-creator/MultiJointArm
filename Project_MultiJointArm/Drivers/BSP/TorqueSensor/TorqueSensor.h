/**
 ****************************************************************************************************
 * @file        TorqueSensor.h
 * @author      LiuZhao
 * @version     V1.0
 * @date        2023-2-21
 * @brief       RS485-TorqueSensor
 ****************************************************************************************************
 */

#ifndef __TORQUESENSOR_H
#define __TORQUESENSOR_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/RS485/rs485.h"
#include "./BSP/CRC/crc.h"


#define ID_ElbowBendSensor								0x01
#define ID_ElbowPronationSensor						0x02

int Get_Torque(void);

#endif

