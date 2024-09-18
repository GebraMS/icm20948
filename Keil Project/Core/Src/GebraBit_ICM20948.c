/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#include "GebraBit_ICM20948.h"
#include <math.h>
extern SPI_HandleTypeDef hspi1;
	
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of ICM20948
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t	GB_ICM20948_Read_Reg_Data ( uint8_t regAddr, ICM20948_Bank_Sel regBank, uint8_t* data)
{	
	uint8_t txBuf[2] = {regAddr|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	GB_ICM20948_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		*data = rxBuf[1];
	}
	return stat;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of ICM20948 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20948_Read_Reg_Bits (uint8_t regAddr, ICM20948_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;

	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (GB_ICM20948_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of ICM20948 that reading multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20948_Burst_Read(uint8_t regAddr, ICM20948_Bank_Sel regBank, uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	uint8_t status = HAL_ERROR;
	GB_ICM20948_Bank_Selection(regBank);
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of ICM20948
 * @param     regBank Register Bank number .
 * @param     data    Value that will be writen to register .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20948_Write_Reg_Data(uint8_t regAddr, ICM20948_Bank_Sel regBank, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr|0x00 , data}; //Write operation: set the 8th-bit to 0.
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	GB_ICM20948_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}

/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of ICM20948 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20948_Write_Reg_Bits(uint8_t regAddr, ICM20948_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}
	if (GB_ICM20948_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)	
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of ICM20948 that writing multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     byteQuantity Quantity of data that we want to write .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20948_Burst_Write		( uint8_t regAddr, ICM20948_Bank_Sel regBank, uint8_t *data, 	uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1]; // +1 is for register address that is 1 byte
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;
	GB_ICM20948_Bank_Selection(regBank);
	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity); // +1 is for set the address of data from [1]th position of array

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return status;
}
/*=========================================================================================================================================
 * @brief     Select Register Bank.
 * @param     bsel   Bank number
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_Bank_Selection( ICM20948_Bank_Sel bsel)
{
  uint8_t rtxBuf[2] = {ICM20948_BANK_SEL|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rrxBuf[2];
	uint8_t wtxBuf[2];
	uint8_t wrxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	uint8_t tempData = 0;
	uint8_t start_bit = START_MSB_BIT_AT_5 ;
	uint8_t len = BIT_LENGTH_2 ;
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, rtxBuf, rrxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		tempData = rrxBuf[1];
	}
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		bsel <<= (start_bit - len + 1); // shift data into correct position
		bsel &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= bsel; // combine data with existing byte

		wtxBuf[0] = ICM20948_BANK_SEL;
		wtxBuf[1] = tempData;
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		stat = (HAL_SPI_TransmitReceive(&hspi1, wtxBuf, wrxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}
/*=========================================================================================================================================
 * @brief     Reset ICM20948
 * @param     ICM20948   ICM20948 Struct RESET  variable
 * @param     ICM20948   ICM20948 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_Soft_Reset ( GebraBit_ICM20948 * ICM20948 )
{
	do 
	 {
		GB_ICM20948_Write_Reg_Bits(ICM20948_PWR_MGMT_1,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , 1);
		HAL_Delay(50);
		GB_ICM20948_Read_Reg_Bits (ICM20948_PWR_MGMT_1,BANK_0 ,START_MSB_BIT_AT_7, BIT_LENGTH_1, &ICM20948->RESET);
		if ( ICM20948->RESET == DONE )
			break;
	 }while(1);
}
/*=========================================================================================================================================
 * @brief     Get Who am I Register Value From Sensor
 * @param     ICM20948     ICM20948 Struct WHO_AM_I variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_ICM20948_Who_am_I(GebraBit_ICM20948 * ICM20948)
{
	GB_ICM20948_Read_Reg_Data( ICM20948_WHO_AM_I, BANK_0,&ICM20948->WHO_AM_I);
}	

/*=========================================================================================================================================
 * @brief     Select SPI 4 Wire as interface
 * @param     ICM20948   ICM20948 Struct INTERFACE  variable
 * @param     spisel Determines SPI 4 Wire as interface or not 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_Select_SPI4_Interface(GebraBit_ICM20948 * ICM20948 , ICM20948_Interface spisel)
{
 GB_ICM20948_Write_Reg_Bits( ICM20948_USER_CTRL,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , spisel);
 ICM20948->INTERFACE = spisel ; 
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable DMP and Set it to Low Power mode if needed.
 * @param     ICM20948   ICM20948 Struct DMP  variable
 * @param     dmp    Enable Or Disable DMP
 * @param     dmp_lp Determines DMP in 	DMP_LOW_POWER or NOT_DMP_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/

void GB_ICM20948_DMP(GebraBit_ICM20948* ICM20948 ,ICM20948_Ability dmp,ICM20948_DMP_LP dmp_lp)
{
	if(dmp_lp==NOT_DMP_LOW_POWER)
	{
		GB_ICM20948_Write_Reg_Bits (ICM20948_USER_CTRL ,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , dmp);
	  GB_ICM20948_Write_Reg_Bits (ICM20948_MOD_CTRL_USR ,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , dmp_lp);
	}
	else if(dmp_lp==DMP_LOW_POWER)
	{
		GB_ICM20948_Write_Reg_Bits (ICM20948_USER_CTRL ,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , dmp);
	  GB_ICM20948_Write_Reg_Bits (ICM20948_MOD_CTRL_USR ,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , dmp_lp);
	}
	ICM20948->DMP = dmp ;
}

/*=========================================================================================================================================
 * @brief     Reset DMP
 * @param     rst     Determines reset DMP or not
 * @return    Nothing
 ========================================================================================================================================*/ 

void GB_ICM20948_DMP_Reset(GebraBit_ICM20948* ICM20948 ,ICM20948_Ability rst)
{
  GB_ICM20948_Write_Reg_Bits (ICM20948_USER_CTRL ,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , rst);
}

/*=========================================================================================================================================
 * @brief     Set DMP interrupt on which Pin set
 * @param     pin    Determines INTERRUPT_ON_PIN_1 or INTERRUPT_ON_PIN_2
 * @param     interrupt Enable Or Disable interrupt
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_DMP_Interrupt(ICM20948_Ability interrupt)///DMP_INTERRUPT---->STRUCT
{
	GB_ICM20948_Write_Reg_Bits (ICM20948_INT_ENABLE ,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_1 , interrupt);
}
/*=========================================================================================================================================
 * @brief     SET ICM20948 Sleep or Awake
 * @param     ICM20948   ICM20948 Struct IS_ICM20948_Sleep  variable
 * @param     working   Determines ICM20948_AWAKE or ICM20948_SLEEP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Sleep_Awake (GebraBit_ICM20948 * ICM20948, ICM20948_Sleep  working  ) 
{
  GB_ICM20948_Write_Reg_Bits (ICM20948_PWR_MGMT_1 ,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1 , working);
  ICM20948->IS_ICM20948_SLEEP = working ;
}

/*=========================================================================================================================================
 * @brief     Set ICM20948 Accelerometer Power Mode
 * @param     ICM20948   ICM20948 Struct ACCEL_POWER_MODE  variable
 * @param     pmode        Determines ICM20948 Accelerometer Power Mode in ICM20948_LOW_NOISE or ICM20948_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
 void GB_ICM20948_ACCEL_Power_Mode(GebraBit_ICM20948* ICM20948 ,ICM20948_Power_Mode pmode)
{
	GB_ICM20948_Write_Reg_Bits (ICM20948_LP_CONFIG ,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , pmode);
	ICM20948->ACCEL_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set ICM20948 Gyroscope Power Mode
 * @param     ICM20948   ICM20948 Struct GYRO_POWER_MODE  variable
 * @param     pmode        Determines ICM20948 Gyroscope Power Mode in ICM20948_LOW_NOISE or ICM20948_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_GYRO_Power_Mode(GebraBit_ICM20948* ICM20948 ,ICM20948_Power_Mode pmode)
{
	GB_ICM20948_Write_Reg_Bits (ICM20948_LP_CONFIG ,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , pmode);
	ICM20948->GYRO_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set ICM20948 Clock Source
 * @param     ICM20948   ICM20948 Struct CLOCK_SOURCE  variable
 * @param     clk    Determines between INTERNAL_20MHZ_OSCILLATOR , AUTO_SELECT and CLOCK_STOP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Set_Clock_Source(GebraBit_ICM20948 * ICM20948 , ICM20948_CLK clk)
{ 
 GB_ICM20948_Write_Reg_Bits( ICM20948_PWR_MGMT_1,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_3 , clk);
 ICM20948->CLOCK_SOURCE = clk ;
} 
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature Sensor
 * @param     ICM20948   ICM20948 Struct TEMPERATURE  variable
 * @param     temp     Determines DISABLE or ENABLE Temperature Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Temperature(GebraBit_ICM20948* ICM20948 ,ICM20948_Ability temp)
{
	GB_ICM20948_Write_Reg_Bits (ICM20948_PWR_MGMT_1 ,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , !temp);
  ICM20948->TEMPERATURE = temp ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Accelerometer Sensor
 * @param     ICM20948   ICM20948 Struct ACCEL  variable
 * @param     accel     Determines DISABLE or ENABLE Accelerometer Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 	
/*
M403Z 
*/
void GB_ICM20948_Accelerometer(GebraBit_ICM20948 * ICM20948 , ICM20948_Sensor accel)
{
	GB_ICM20948_Write_Reg_Bits (ICM20948_PWR_MGMT_2 ,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_3 , accel);
  ICM20948->ACCEL = accel ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope Sensor
 * @param     ICM20948   ICM20948 Struct GYRO  variable
 * @param     gyro     Determines DISABLE or ENABLE Gyroscope Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Gyroscope(GebraBit_ICM20948 * ICM20948 , ICM20948_Sensor gyro)
{
	GB_ICM20948_Write_Reg_Bits (ICM20948_PWR_MGMT_2 ,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_3 , gyro);
  ICM20948->GYRO = gyro ; 
}
/*=========================================================================================================================================
 * @brief     Configure hardware interrupt pin (INT1) 
 * @param     ICM20948  ICM20948 struct INT1_PIN_LEVEL , INT1_PIN_TYPE and INT1_PIN_LATCH  variables
 * @param     level   ACTIVE_HIGH or  ACTIVE_LOW 
 * @param     type    PUSH_PULL   or  OPEN_DRAIN
 * @param     latch   _50_US      or  HELD_STATUS_CLEAR
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_Set_INT1_Pin(GebraBit_ICM20948 * ICM20948 , ICM20948_INT_Level level ,ICM20948_INT_Type type , ICM20948_Latch_Type latch )
{
  GB_ICM20948_Write_Reg_Bits( ICM20948_INT_PIN_CFG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , level);
	GB_ICM20948_Write_Reg_Bits( ICM20948_INT_PIN_CFG,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1 , type);
	GB_ICM20948_Write_Reg_Bits( ICM20948_INT_PIN_CFG,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , latch);
	ICM20948->INT1_PIN_LEVEL = level ; 
	ICM20948->INT1_PIN_TYPE  = type  ;
	ICM20948->INT1_PIN_LATCH = latch ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Interrupt Status
 * @param     ICM20948  ICM20948 struct INTERRUPT  variable 
 * @param     interrupt    DISABLE or ENABLE
 * @return    Nothing
 ========================================================================================================================================*/ 
void Interrupt_Status_Enable(GebraBit_ICM20948 * ICM20948 , ICM20948_Ability interrupt )
{
  GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_CFG ,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , interrupt);
	//ICM20948->INTERRUPT = interrupt ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Access Serial Interface To FIFO
 * @param     ICM20948  ICM20948 struct INTERFACE_ACCESS_FIFO  variable
 * @param     interface_access_fifo    DISABLE or ENABLE
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Access_Serial_Interface_To_FIFO(GebraBit_ICM20948 * ICM20948 , ICM20948_Ability interface_access_fifo) 
{ 
	GB_ICM20948_Write_Reg_Bits (ICM20948_USER_CTRL , BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1,  interface_access_fifo);
  ICM20948->INTERFACE_ACCESS_FIFO = interface_access_fifo ;  
}
/*=========================================================================================================================================
 * @brief     Check if FIFO Overflow
 * @param     ICM20948    Store FIFO Overflow status on ICM20948 Struct FIFO_OVERFLOW variable
 * @return    NOT_FIFO_OVERFLOW or FIFO_OVERFLOW
 ========================================================================================================================================*/ 
ICM20948_FIFO_Overflow GB_ICM20948_Check_FIFO_Overflow(GebraBit_ICM20948 * ICM20948)
{
  GB_ICM20948_Read_Reg_Bits (ICM20948_INT_STATUS_2,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, &ICM20948->FIFO_OVERFLOW); 
	return ICM20948->FIFO_OVERFLOW;
}
/*=========================================================================================================================================
 * @brief     Check if Data is ready
 * @param     ICM20948    Store data ready status on ICM20948 Struct DATA_STATUS variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
ICM20948_Preparation GB_ICM20948_Check_Data_Preparation(GebraBit_ICM20948 * ICM20948)
{
  GB_ICM20948_Read_Reg_Bits (ICM20948_INT_STATUS_1,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1, &ICM20948->DATA_STATUS); 
	return ICM20948->DATA_STATUS;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE accelerometer to be written on FIFO
 * @param     ICM20948  ICM20948 struct ACCEL_TO_FIFO  variable  
 * @param     accel_fifo Determines accelerometer write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Write_ACCEL_FIFO(GebraBit_ICM20948 * ICM20948 , ICM20948_Ability accel_fifo )
{
   GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_2,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1,accel_fifo); 
	 ICM20948->ACCEL_TO_FIFO = accel_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope to be written on FIFO
 * @param     ICM20948  ICM20948 struct GYRO_TO_FIFO  variable  
 * @param     gyro_fifo  Determines Gyroscope write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Write_GYRO_FIFO(GebraBit_ICM20948 * ICM20948 , ICM20948_Ability gyro_fifo )
{
   GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_2,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_3,(uint8_t)(gyro_fifo*7)); 
	 ICM20948->GYRO_TO_FIFO = gyro_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature to be written on FIFO
 * @param     ICM20948  ICM20948 struct TEMP_TO_FIFO  variable 
 * @param     temp_fifo  Determines Temperature write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20948_Write_TEMP_FIFO(GebraBit_ICM20948 * ICM20948 , ICM20948_Ability temp_fifo )
{
   GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_2,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1,temp_fifo); 
	 ICM20948->TEMP_TO_FIFO = temp_fifo ;
}

void GB_ICM20948_Write_Slave_FIFO(GebraBit_ICM20948 * ICM20948 ,ICM20948_I2C_Slave slave , ICM20948_Ability slv_fifo )
{	 
	switch(slave)
	 {
	  case SLAVE_0:
    GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_1,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1,slv_fifo); 
    break;
		case SLAVE_1:
    GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_1,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_1,slv_fifo);
    break;	
		case SLAVE_2:
    GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_1,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_1,slv_fifo);
    break;	
		case SLAVE_3: 
    GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_1,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1,slv_fifo); 
    break;	
		default:
    GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_EN_1,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1,slv_fifo);
	 } 
	ICM20948->SLAVE_TO_FIFO = slv_fifo ; 
	ICM20948->SLAVE_CHANNEL_FIFO = slave ;
}

/*=========================================================================================================================================
 * @brief     Set FIFO MODE
 * @param     ICM20948  ICM20948 struct FIFO_MODE  variable 
 * @param     mode     Determines FIFO MODE BYPASS ,  STREAM_TO_FIFO , STOP_ON_FULL_FIFO_SNAPSHOT
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_FIFO_Mode(GebraBit_ICM20948 * ICM20948 , ICM20948_FIFO_Mode fifo_mode )
{
  GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_MODE,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, fifo_mode); 
  ICM20948->FIFO_MODE = fifo_mode;
}
/*=========================================================================================================================================
 * @brief     Set FIFO reset. Assert and hold to set FIFO size to 0. Assert and de-assert to reset FIFO.
 * @param     ICM20948  ICM20948 struct FIFO_RESET  variable 
 * @param     fifo_rst     Determines NOT_FIFO_REST or FIFO_REST
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_FIFO_Reset(void ) 
{
  GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_RST,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, FIFO_ASSERT); 
	//HAL_Delay(5);
	GB_ICM20948_Write_Reg_Bits (ICM20948_FIFO_RST,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, FIFO_DE_ASSERT); 
	HAL_Delay(1);
  //ICM20948->FIFO_RESET = fifo_rst;
}
/*=========================================================================================================================================
 * @brief     Get FIFO Count  
 * @param     ICM20948   ICM20948 struct  FIFO_COUNT variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_GET_FIFO_Count (GebraBit_ICM20948 * ICM20948 ) 
{
	uint8_t count_h , count_l;
  GB_ICM20948_Read_Reg_Bits( ICM20948_FIFO_COUNTH, BANK_0,START_MSB_BIT_AT_4, BIT_LENGTH_5, &count_h); 
	GB_ICM20948_Read_Reg_Data( ICM20948_FIFO_COUNTL, BANK_0, &count_l );
	ICM20948->FIFO_COUNT = (uint16_t)((count_h << 8) | count_l);////13_Bit
}
/*=========================================================================================================================================
 * @brief     Read Data Directly from FIFO
 * @param     ICM20948  ICM20948 struct FIFO_DATA variable
 * @param     qty    Determine hoe many Data Byte to read
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Read_FIFO(GebraBit_ICM20948 * ICM20948 , uint16_t qty)  
{
  GB_ICM20948_Burst_Read( ICM20948_FIFO_R_W,BANK_0,ICM20948->FIFO_DATA, qty);
}
/*=========================================================================================================================================
 * @brief     Check if Data is Copied TO FIFO
 * @param     ICM20948    Store FIFO Copy status on ICM20948 Struct DATA_COPY_FIFO variable
 * @return    COPY_TO_FIFO or NOT_COPY_FIFO
 ========================================================================================================================================*/ 
ICM20948_Data_Copy_FIFO GB_ICM20948_Check_Data_Copy_TO_FIFO(GebraBit_ICM20948 * ICM20948)
{
  GB_ICM20948_Read_Reg_Bits (ICM20948_DATA_RDY_STATUS,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_4, &ICM20948->DATA_COPY_FIFO); 
	return ICM20948->DATA_COPY_FIFO;
}

/*=========================================================================================================================================
 * @brief     Set Gyroscope Full Scale Range and select Gyroscope SCALE FACTOR
 * @param     ICM20948   ICM20948 Struct GYRO_FULL_SCALE and GYRO_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among FS_500_DPS , FS_1000_DPS , FS_2000_DPS , FS_4000_DPS
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_GYRO_Full_Scale ( GebraBit_ICM20948 * ICM20948 , ICM20948_Gyro_Fs_Sel fs ) 
{
  GB_ICM20948_Write_Reg_Bits (ICM20948_GYRO_CONFIG_1,BANK_2 , START_MSB_BIT_AT_2, BIT_LENGTH_2, fs);
	ICM20948->GYRO_FULL_SCALE = fs ; 
	switch(fs)
	 {
	  case FS_250_DPS:
		ICM20948->GYRO_SCALE_FACTOR = SCALE_FACTOR_131_LSB_DPS ;
		ICM20948->PRECISE_GYRO_SF   =  131 ;
    break;
		case FS_500_DPS:
		ICM20948->GYRO_SCALE_FACTOR = SCALE_FACTOR_65p5_LSB_DPS ;
		ICM20948->PRECISE_GYRO_SF   =  65.5 ;
    break;	
		case FS_1000_DPS:
		ICM20948->GYRO_SCALE_FACTOR = SCALE_FACTOR_32p8_LSB_DPS ;
		ICM20948->PRECISE_GYRO_SF   =  32.8 ;
    break;	
		case FS_2000_DPS:
		ICM20948->GYRO_SCALE_FACTOR = SCALE_FACTOR_16p4_LSB_DPS ;
		ICM20948->PRECISE_GYRO_SF   =  16.4 ;
    break;			
		default:
		ICM20948->GYRO_SCALE_FACTOR = SCALE_FACTOR_131_LSB_DPS ;
    ICM20948->PRECISE_GYRO_SF   =  131 ;		
	 }
}
/*=========================================================================================================================================
 * @brief     Enable Or Bypass GYRO Low Pass Filter
 * @param     ICM20948     ICM20948 Struct GYRO_FCHOICEB variable
 * @param     bypass       Determines ENABLE_DLPF_FCHOICEB_1 or BYPASS_DLPF_FCHOICEB_0
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20948_GYRO_Low_Pass_Filter  (GebraBit_ICM20948 * ICM20948 ,  ICM20948_FCHOICEB bypass )
{
	if ( bypass == BYPASS_DLPF_FCHOICEB  )
		ICM20948->GYRO_SAMPLE_RATE = _9_KHz ;
	GB_ICM20948_Write_Reg_Bits(ICM20948_GYRO_CONFIG_1,BANK_2 , START_MSB_BIT_AT_0, BIT_LENGTH_1,  bypass);
	ICM20948->GYRO_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set GYRO Low Pass Filter value
 * @param     ICM20948     ICM20948 Struct GYRO_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 		
void GB_ICM20948_GYRO_Low_Pass_Filter_Value  (GebraBit_ICM20948 * ICM20948 , ICM20948_GYRO_DLPF dlpf )
{
	  GB_ICM20948_Write_Reg_Bits(ICM20948_GYRO_CONFIG_1,BANK_2 , START_MSB_BIT_AT_5, BIT_LENGTH_3,  dlpf);
	  ICM20948->GYRO_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  GYRO Averaging Filter
 * @param     ICM20948  ICM20948 Struct GYRO_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_ICM20948_GYRO_LP_Averaging_Filter  (GebraBit_ICM20948 * ICM20948 , ICM20948_GYRO_Averaging_Filter avg )
{
	  GB_ICM20948_Write_Reg_Bits(ICM20948_GYRO_CONFIG_2,BANK_2 , START_MSB_BIT_AT_2, BIT_LENGTH_3,  avg);
	  ICM20948->GYRO_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Gyroscope Sensor Output Sample Rate that controls sensor data output rate, FIFO sample rate
 * @param     ICM20948   ICM20948 struct GYRO_SAMPLE_RATE and GYRO_SAMPLE_DEVIDE variable
 * @param     rate_hz    Sample Rate in Hz
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_GYRO_Output_Sample_Rate (GebraBit_ICM20948 * ICM20948 , uint16_t rate_hz)
{
	uint8_t gfchoice , gdlpf ;
	GB_ICM20948_Read_Reg_Bits(ICM20948_GYRO_CONFIG_1,BANK_2, START_MSB_BIT_AT_0, BIT_LENGTH_1 , &gfchoice);
  GB_ICM20948_Read_Reg_Bits(ICM20948_GYRO_CONFIG_1,BANK_2, START_MSB_BIT_AT_5, BIT_LENGTH_3 , &gdlpf);
	if((gfchoice==1)&&(0<gdlpf)&&(gdlpf<7))
	{
		ICM20948->GYRO_SAMPLE_RATE = rate_hz ;
    ICM20948->GYRO_SAMPLE_DEVIDE=(INTERNAL_SAMPLE_RATE/ICM20948->GYRO_SAMPLE_RATE)-1;
		GB_ICM20948_Write_Reg_Data( ICM20948_GYRO_SMPLRT_DIV ,BANK_2,ICM20948->GYRO_SAMPLE_DEVIDE); 
	}
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Full Scale Range and select sensor SCALE FACTOR
 * @param     ICM20948   ICM20948 struct ACCEL_FULL_SCALE and ACCEL_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among  4g , 8g , 16g , 30g
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_ACCEL_Full_Scale ( GebraBit_ICM20948 * ICM20948 , ICM20948_Accel_Fs_Sel fs ) 
{
  GB_ICM20948_Write_Reg_Bits( ICM20948_ACCEL_CONFIG,BANK_2, START_MSB_BIT_AT_2, BIT_LENGTH_2 , fs);
	ICM20948->ACCEL_FULL_SCALE =  fs ;
	switch(fs)
	 {
	  case FULL_SCALE_2g:
		ICM20948->ACCEL_SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;
    break;
		case FULL_SCALE_4g:
		ICM20948->ACCEL_SCALE_FACTOR = SCALE_FACTOR_8192_LSB_g ;
    break;	
		case FULL_SCALE_8g:
		ICM20948->ACCEL_SCALE_FACTOR = SCALE_FACTOR_4096_LSB_g ;
    break;	
		case FULL_SCALE_16g: 
		ICM20948->ACCEL_SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;
    break;			
		default:
		ICM20948->ACCEL_SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;		
	 }
}

/*=========================================================================================================================================
 * @brief     Enable Or Bypass Accelerometer Low Pass Filter
 * @param     ICM20948     ICM20948 Struct ACCEL_FCHOICEB variable
 * @param     bypass       Determines ENABLE_DLPF_FCHOICEB_1 or BYPASS_DLPF_FCHOICEB_0
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20948_ACCEL_Low_Pass_Filter  (GebraBit_ICM20948 * ICM20948 ,  ICM20948_FCHOICEB bypass )
{
	if ( bypass == BYPASS_DLPF_FCHOICEB  )   
	 ICM20948->ACCEL_SAMPLE_RATE = _4_5_KHz ;
	GB_ICM20948_Write_Reg_Bits(ICM20948_ACCEL_CONFIG,BANK_2 , START_MSB_BIT_AT_0, BIT_LENGTH_1,  bypass);
	ICM20948->ACCEL_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Low Pass Filter value
 * @param     ICM20948     ICM20948 Struct ACCEL_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20948_ACCEL_Low_Pass_Filter_Value  (GebraBit_ICM20948 * ICM20948 , ICM20948_ACCEL_DLPF dlpf )
{
	  GB_ICM20948_Write_Reg_Bits(ICM20948_ACCEL_CONFIG,BANK_2 , START_MSB_BIT_AT_5, BIT_LENGTH_3,  dlpf);
	  ICM20948->ACCEL_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  Accelerometer Averaging Filter
 * @param     ICM20948  ICM20948 Struct ACCEL_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_ICM20948_ACCEL_LP_Averaging_Filter  (GebraBit_ICM20948 * ICM20948 , ICM20948_ACCEL_Averaging_Filter avg )
{
	  GB_ICM20948_Write_Reg_Bits(ICM20948_ACCEL_CONFIG_2,BANK_2 , START_MSB_BIT_AT_1, BIT_LENGTH_2,  avg);
	  ICM20948->ACCEL_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Sensor Output Sample Rate that controls Accelerometer data output rate, FIFO sample rate
 * @param     ICM20948   ICM20948 struct ACCEL_SAMPLE_RATE and ACCEL_SAMPLE_DEVIDE variable
 * @param     rate_hz    Sample Rate in Hz
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_ACCEL_Output_Sample_Rate (GebraBit_ICM20948 * ICM20948 , uint16_t rate_hz)
{
	uint8_t afchoice , adlpf ;
	GB_ICM20948_Read_Reg_Bits(ICM20948_ACCEL_CONFIG,BANK_2, START_MSB_BIT_AT_0, BIT_LENGTH_1 , &afchoice);
  GB_ICM20948_Read_Reg_Bits(ICM20948_ACCEL_CONFIG,BANK_2, START_MSB_BIT_AT_5, BIT_LENGTH_3 , &adlpf);
	if((afchoice==1)&&(0<adlpf)&&(adlpf<7))
	{
		ICM20948->ACCEL_SAMPLE_RATE = rate_hz ;
    ICM20948->ACCEL_SAMPLE_DEVIDE=(INTERNAL_SAMPLE_RATE/ICM20948->ACCEL_SAMPLE_RATE)-1;
	  GB_ICM20948_Write_Reg_Data( ICM20948_ACCEL_SMPLRT_DIV_1 ,BANK_2,(ICM20948->ACCEL_SAMPLE_DEVIDE >> 8) & 0xFF);
		GB_ICM20948_Write_Reg_Data( ICM20948_ACCEL_SMPLRT_DIV_2 ,BANK_2,ICM20948->ACCEL_SAMPLE_DEVIDE & 0xFF); 
	}
}
/*=========================================================================================================================================
 * @brief     Set Temperature Low Pass Filter value
 * @param     ICM20948     ICM20948 Struct TEMP_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20948_TEMP_Low_Pass_Filter_Value  (GebraBit_ICM20948 * ICM20948 , ICM20948_TEMP_DLPF tdlpf )
{
	  GB_ICM20948_Write_Reg_Bits(ICM20948_TEMP_CONFIG,BANK_2 , START_MSB_BIT_AT_2, BIT_LENGTH_3,  tdlpf);
	  ICM20948->TEMP_DLPF =  tdlpf ;
}
/*=========================================================================================================================================
 * @brief     Configure FIFO
 * @param     ICM20948       ICM20948 Struct FIFO variable
 * @param     fifo           Configure ICM20689 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_FIFO_Configuration ( GebraBit_ICM20948 * ICM20948 , ICM20948_FIFO_Ability fifo  )
{
	ICM20948->FIFO_PACKET_QTY = FIFO_DATA_BUFFER_SIZE / BYTE_QTY_IN_ONE_PACKET ;  
	if( fifo==Enable )  
	{
		ICM20948->FIFO = FIFO_ENABLE  ;
		GB_ICM20948_FIFO_Reset();
		GB_ICM20948_Access_Serial_Interface_To_FIFO( ICM20948 , Enable );
		GB_ICM20948_FIFO_Mode ( ICM20948 , STOP_ON_FULL_FIFO_SNAPSHOT );
		GB_ICM20948_Write_GYRO_FIFO ( ICM20948 , Enable );
	  GB_ICM20948_Write_ACCEL_FIFO( ICM20948 , Enable );
		GB_ICM20948_Write_TEMP_FIFO ( ICM20948 , Enable );
		//GB_ICM20948_Write_Slave_FIFO( ICM20948 , SLAVE_0 , Enable );
		Interrupt_Status_Enable( ICM20948 , Disable  );
	}
	else if ( fifo == Disable )
	{
		ICM20948->FIFO = FIFO_DISABLE  ;
		GB_ICM20948_Write_GYRO_FIFO ( ICM20948 , Disable );
	  GB_ICM20948_Write_ACCEL_FIFO( ICM20948 , Disable );
		GB_ICM20948_Write_TEMP_FIFO ( ICM20948 , Disable );
		GB_ICM20948_Access_Serial_Interface_To_FIFO( ICM20948 , Disable );
		GB_ICM20948_FIFO_Reset( );
	}
}
/*=========================================================================================================================================
 * @brief     controls the I2C Master’s transition from one slave read to the next slave if There is a restart between reads or stop
 * @param     ICM20948    ICM20948 Struct 
 * @param     bet       select RESTART or  STOP
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_AUX_I2C_Between_Reads(GebraBit_ICM20948 * ICM20948,ICM20948_Aux_Between bet )
{
	GB_ICM20948_Write_Reg_Bits(ICM20948_I2C_MST_CTRL      ,BANK_3, START_MSB_BIT_AT_4, BIT_LENGTH_1 , bet); 
}
/*=========================================================================================================================================
 * @brief     Set External Sensor sample rate
 * @param     ICM20948    ICM20948 Struct 
 * @param     devider     ODR=1.1 kHz/(2^((odr_config[3:0])) )  where  ODR==odr_config[3:0]
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_External_Sensor_ODR(GebraBit_ICM20948 * ICM20948,uint8_t devider)
{
	GB_ICM20948_Write_Reg_Bits(ICM20948_I2C_MST_ODR_CONFIG,BANK_3, START_MSB_BIT_AT_3, BIT_LENGTH_4 , devider );////ODR=1.1 kHz/(2^((odr_config[3:0])) )
	ICM20948->EXTERNAL_SENSOR_ODR = (EXTERNAL_SENSOR_SAMPLE_RATE)/(pow(2,devider)) ; 
}
/*=========================================================================================================================================
 * @brief     Initialize ICM20948 AUX I2C 
 * @param     ICM20948    ICM20948 Struct  
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_AUX_Initialize(GebraBit_ICM20948 * ICM20948)
{
	GB_ICM20948_AUX_I2C_Between_Reads(ICM20948,STOP );
	GB_ICM20948_External_Sensor_ODR(ICM20948, 1);
}
/*=========================================================================================================================================
 * @brief     stop AUX I2C for slave channel
 * @param     ICM20948    ICM20948 Struct 
 * @param     slave       AUX I2C slave channel 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_Stop_AUX_I2C(GebraBit_ICM20948 * ICM20948,ICM20948_I2C_Slave slave) 
{
  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_CTRL,BANK_3 ,0);
		switch(slave)
	 {
	  case SLAVE_0:
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_CTRL,BANK_3 ,0); 
    break;
		case SLAVE_1:
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_CTRL,BANK_3 ,0); 
    break;	
		case SLAVE_2:
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_CTRL,BANK_3 ,0); 
    break;	
		case SLAVE_3: 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV3_CTRL,BANK_3 ,0); 
    break;
		case SLAVE_4: 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_CTRL,BANK_3 ,0); 
    break;		
		default:
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_CTRL,BANK_3 ,0); 
	 }
}
void GB_ICM20948_AUX_I2C(GebraBit_ICM20948 * ICM20948,ICM20948_Ability aux)
{
  GB_ICM20948_Write_Reg_Bits(ICM20948_USER_CTRL,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , aux);
}
/*=========================================================================================================================================
 * @brief     set AUX I2C for Reading data
 * @param     ICM20948    ICM20948 Struct 
 * @param     slave       AUX I2C slave channel 
 * @param     slave_addr  AUX I2C slave address 
 * @param     slave_reg   AUX I2C slave register address 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_Set_AUX_I2C_Read(GebraBit_ICM20948 * ICM20948, ICM20948_I2C_Slave slave, uint8_t slave_addr, uint8_t slave_reg, uint8_t data_len)
{ 
	switch(slave)
	 {
	  case SLAVE_0:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_ADDR,BANK_3 ,AUX_I2C_TRANSFER_IS_READ|slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_REG ,BANK_3 ,slave_reg); 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|data_len); 
    break;
		case SLAVE_1:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_ADDR,BANK_3 ,AUX_I2C_TRANSFER_IS_READ|slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_REG ,BANK_3 ,slave_reg); 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|data_len);
    break;	
		case SLAVE_2:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_ADDR,BANK_3 ,AUX_I2C_TRANSFER_IS_READ|slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_REG ,BANK_3 ,slave_reg); 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|data_len);
    break;	
		case SLAVE_3: 
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV3_ADDR,BANK_3 ,AUX_I2C_TRANSFER_IS_READ|slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV3_REG ,BANK_3 ,slave_reg); 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|data_len);
    break;
		case SLAVE_4: 
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_ADDR,BANK_3 ,AUX_I2C_TRANSFER_IS_READ|slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_REG ,BANK_3 ,slave_reg); 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|data_len);
    break;		
		default:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_ADDR,BANK_3 ,AUX_I2C_TRANSFER_IS_READ|slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_REG ,BANK_3 ,slave_reg); 
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|data_len); 
	 }
}
/*=========================================================================================================================================
 * @brief     Read data thorough AUX I2C  
 * @param     ICM20948    ICM20948 Struct 
 * @param     slave       AUX I2C slave channel 
 * @param     slave_addr  AUX I2C slave address 
 * @param     slave_reg   AUX I2C slave register address 
 * @param     data        data that is read with AUX I2C from slave
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_AUX_I2C_Read_Data(GebraBit_ICM20948 * ICM20948, ICM20948_I2C_Slave slave, uint8_t slave_addr, uint8_t slave_reg, uint8_t data_len, uint8_t * data)
{
	GB_ICM20948_Set_AUX_I2C_Read(ICM20948,slave,slave_addr,slave_reg,data_len);
	GB_ICM20948_AUX_I2C(ICM20948,Enable);
	HAL_Delay(60);
  GB_ICM20948_AUX_I2C(ICM20948,Disable);     
  GB_ICM20948_Burst_Read( ICM20948_EXT_SLV_SENS_DATA_00,BANK_0,data, data_len); 
	GB_ICM20948_Stop_AUX_I2C(ICM20948, slave);  
}
/*=========================================================================================================================================
 * @brief     set AUX I2C in write mode
 * @param     ICM20948    ICM20948 Struct 
 * @param     slave       AUX I2C slave channel 
 * @param     slave_addr  AUX I2C slave address 
 * @param     slave_reg   AUX I2C slave register address 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20948_Set_AUX_I2C_Write(GebraBit_ICM20948 * ICM20948, ICM20948_I2C_Slave slave, uint8_t slave_addr, uint8_t slave_reg, uint8_t data)
{ 
	switch(slave)
	 {
	  case SLAVE_0:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_ADDR,BANK_3 ,slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_REG ,BANK_3 ,slave_reg); 
		GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_DO  ,BANK_3 ,data);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|1); 
    break;
		case SLAVE_1:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_ADDR,BANK_3 ,slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_REG ,BANK_3 ,slave_reg); 
		GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_DO  ,BANK_3 ,data);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV1_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|1);
    break;	
		case SLAVE_2:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_ADDR,BANK_3 ,slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_REG ,BANK_3 ,slave_reg); 
		GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_DO  ,BANK_3 ,data);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|1);
    break;	
		case SLAVE_3: 
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV3_ADDR,BANK_3 ,slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV3_REG ,BANK_3 ,slave_reg); 
		GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV3_DO  ,BANK_3 ,data);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV2_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|1);
    break;
		case SLAVE_4: 
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_ADDR,BANK_3 ,slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_REG ,BANK_3 ,slave_reg); 
		GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_DO  ,BANK_3 ,data);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV4_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|1);
    break;		
		default:
	  GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_ADDR,BANK_3 ,slave_addr);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_REG ,BANK_3 ,slave_reg);
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_DO  ,BANK_3 ,data);		
    GB_ICM20948_Write_Reg_Data(ICM20948_I2C_SLV0_CTRL,BANK_3 ,AUX_I2C_TRANSFER_ENABLE|1); 
	 }
}
/*=========================================================================================================================================
 * @brief     write data thorough AUX I2C  
 * @param     ICM20948    ICM20948 Struct 
 * @param     slave       AUX I2C slave channel 
 * @param     slave_addr  AUX I2C slave address 
 * @param     slave_reg   AUX I2C slave register address 
 * @param     data        data that is going to be write with AUX I2C on slave
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_AUX_I2C_Write_Data(GebraBit_ICM20948 * ICM20948, ICM20948_I2C_Slave slave, uint8_t slave_addr, uint8_t slave_reg, uint8_t  data)
{
	GB_ICM20948_Set_AUX_I2C_Write(ICM20948,slave,slave_addr,slave_reg,data);
	GB_ICM20948_AUX_I2C(ICM20948,Enable);
	HAL_Delay(60);
  GB_ICM20948_AUX_I2C(ICM20948,Disable);     
	GB_ICM20948_Stop_AUX_I2C(ICM20948, slave);  
}
/*=========================================================================================================================================
 * @brief     Get_Device ID of AK09916 internal Magnetometer sensor
 * @param     ICM20948    Store on ICM20948 Struct AK09916_ID variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_AK09916_Device_ID(GebraBit_ICM20948 * ICM20948) 
{
 GB_ICM20948_AUX_I2C_Read_Data(ICM20948, SLAVE_0,ICM20948_AK09916_I2C_ADDRESS,ICM20948_AK09916_DEVICE_ID,BIT_LENGTH_1,&ICM20948->AK09916_ID); 
}
/*=========================================================================================================================================
 * @brief     Check if Data is ready in AK09916 internal Magnetometer sensor
 * @param     ICM20948    Store data ready status on ICM20948 Struct AK09916_DATA_STATUS variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
ICM20948_Preparation GB_ICM20948_Check_AK09916_Data_Preparation(GebraBit_ICM20948 * ICM20948)
{
	GB_ICM20948_AUX_I2C_Read_Data(ICM20948, SLAVE_0,ICM20948_AK09916_I2C_ADDRESS,ICM20948_AK09916_STATUS_1,BIT_LENGTH_1,&ICM20948->AK09916_DATA_STATUS);
	ICM20948->AK09916_DATA_STATUS= (ICM20948_Preparation)(ICM20948->AK09916_DATA_STATUS * 0X01);
	return ICM20948->AK09916_DATA_STATUS; 
}
/*=========================================================================================================================================
 * @brief     Set AK09916 internal Magnetometer sensor Power Management
 * @param     ICM20948   ICM20948 Struct ACCEL_POWER_MODE and AK09916_POWER_MODE  variable
 * @param     pmode        Determines  Power Mode in AK09916_POWER_DOWN or AK09916_SINGLE_MEASUREMENT or AK09916_CONTINUOUS_MEASUREMENT or AK09916_SELF_TEST
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Set_AK09916_Magnetometer_Power_Mode(GebraBit_ICM20948 * ICM20948 , AK09916_Power_Mode pmode)  
{
  GB_ICM20948_AUX_I2C_Write_Data(ICM20948, SLAVE_1,ICM20948_AK09916_I2C_ADDRESS,ICM20948_AK09916_CONTROL_2,pmode); 
  ICM20948->AK09916_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     When reset, all registers are initialized.  AK09916 internal Magnetometer sensor
 * @param     ICM20948  Configure ICM20948 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Set_AK09916_Soft_Reset(GebraBit_ICM20948 * ICM20948)  
{
  GB_ICM20948_AUX_I2C_Write_Data(ICM20948, SLAVE_1,ICM20948_AK09916_I2C_ADDRESS,ICM20948_AK09916_CONTROL_3,0X01); 
}
/*=========================================================================================================================================
 * @brief     Configure AK09916 internal Magnetometer sensor
 * @param     ICM20948  Configure ICM20948 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Config_AK09916_Magnetometer(GebraBit_ICM20948 * ICM20948)
{
	ICM20948->AK09916_MAG_SCALE_FACTOR = 0.15;
  GB_ICM20948_Set_AK09916_Soft_Reset(ICM20948) ;
	GB_ICM20948_Get_AK09916_Device_ID( ICM20948);
	GB_ICM20948_Set_AK09916_Magnetometer_Power_Mode(ICM20948,AK09916_CONTINUOUS_MEASUREMENT) ;
}
/*=========================================================================================================================================
 * @brief     ST2 register has a role as data reading end register
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Read_AK09916_Status_2(GebraBit_ICM20948 * ICM20948)
{
	uint8_t cache;
  GB_ICM20948_AUX_I2C_Read_Data(ICM20948, SLAVE_0,ICM20948_AK09916_I2C_ADDRESS,ICM20948_AK09916_STATUS_2,BIT_LENGTH_1,&cache);  
}
/*=========================================================================================================================================
 * @brief     Set ICM20948 Power Management
 * @param     ICM20948   ICM20948 Struct ACCEL_POWER_MODE and GYRO_POWER_MODE  variable
 * @param     pmode        Determines ICM20948 Accelerometer Power Mode in ICM20948_LOW_NOISE or ICM20689_LOW_POWER or ICM20689_SLEEP_OFF
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Set_Power_Management(GebraBit_ICM20948 * ICM20948 , ICM20948_Power_Mode pmode) 
{	
	
 GB_ICM20948_Temperature(ICM20948 , Enable );
 GB_ICM20948_Accelerometer(ICM20948 , SENSOR_ENABLE );
 GB_ICM20948_Gyroscope(ICM20948 , SENSOR_ENABLE );
 if(pmode==ICM20948_LOW_POWER)
 {
	GB_ICM20948_Sleep_Awake(ICM20948 , ICM20948_AWAKE );
  GB_ICM20948_GYRO_Power_Mode (ICM20948 , ICM20948_LOW_POWER );
  GB_ICM20948_ACCEL_Power_Mode(ICM20948 , ICM20948_LOW_POWER );		 
 }
  else if(pmode==ICM20948_LOW_NOISE)
 {
	GB_ICM20948_Sleep_Awake(ICM20948 , ICM20948_AWAKE );
  GB_ICM20948_GYRO_Power_Mode (ICM20948 , ICM20948_LOW_NOISE );
  GB_ICM20948_ACCEL_Power_Mode(ICM20948 , ICM20948_LOW_NOISE );	  
 }
 else if (pmode==ICM20948_SLEEP_OFF)
 {
	ICM20948->ACCEL_POWER_MODE = ICM20948_SLEEP_OFF ;
  ICM20948->ACCEL_POWER_MODE = ICM20948_SLEEP_OFF ;
	GB_ICM20948_Temperature(ICM20948 , Disable );
	GB_ICM20948_Accelerometer(ICM20948 , SENSOR_DISABLE );
	GB_ICM20948_Gyroscope(ICM20948 , SENSOR_DISABLE );
	GB_ICM20948_Sleep_Awake(ICM20948 , ICM20948_SLEEP );
 }
 GB_ICM20948_Write_Reg_Bits (ICM20948_PWR_MGMT_1,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , pmode);//reduce current in low power
 HAL_Delay(1);
}

/*=========================================================================================================================================
 * @brief     initialize ICM20948
 * @param     ICM20948     initialize ICM20948
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_initialize( GebraBit_ICM20948 * ICM20948 )
{
  HAL_Delay(3);
  GB_ICM20948_Who_am_I(ICM20948);
	GB_ICM20948_Soft_Reset(ICM20948);
	GB_ICM20948_DMP( ICM20948 ,Disable ,DMP_LOW_POWER );
	GB_ICM20948_Select_SPI4_Interface(ICM20948 , IS_SPI);
	GB_ICM20948_Set_Power_Management( ICM20948 , ICM20948_LOW_NOISE );
	GB_ICM20948_Set_Clock_Source( ICM20948 , AUTO_SELECT );
	GB_ICM20948_GYRO_Low_Pass_Filter (ICM20948,ENABLE_DLPF_FCHOICEB);
	GB_ICM20948_ACCEL_Low_Pass_Filter(ICM20948,ENABLE_DLPF_FCHOICEB);
	GB_ICM20948_GYRO_Low_Pass_Filter_Value (ICM20948,ICM20948_GYRO_DLPF_152);
	GB_ICM20948_ACCEL_Low_Pass_Filter_Value(ICM20948,ICM20948_ACCEL_DLPF_246);
	GB_ICM20948_TEMP_Low_Pass_Filter_Value (ICM20948,ICM20948_TEMP_DLPF_218 );
	GB_ICM20948_FIFO_Configuration ( ICM20948 ,FIFO_DISABLE ) ;
	GB_ICM20948_Set_INT1_Pin( ICM20948 , ACTIVE_LOW  , OPEN_DRAIN  ,  HELD_STATUS_CLEAR );
	GB_ICM20948_AUX_Initialize(ICM20948);
  //Interrupt_Status_Enable( ICM20948 , Enable  );
}
/*=========================================================================================================================================
 * @brief     Configure ICM20948
 * @param     ICM20948  Configure ICM20948 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Configuration(GebraBit_ICM20948 * ICM20948, ICM20948_FIFO_Ability fifo)
{
	GB_ICM20948_GYRO_Output_Sample_Rate (ICM20948,GYRO_ODR_HZ ); 
	GB_ICM20948_ACCEL_Output_Sample_Rate(ICM20948,ACCEL_ODR_HZ );
	GB_ICM20948_GYRO_Full_Scale ( ICM20948 ,FS_1000_DPS ) ;
	GB_ICM20948_ACCEL_Full_Scale( ICM20948 ,FULL_SCALE_4g ) ; 
	GB_ICM20948_FIFO_Configuration ( ICM20948 ,fifo ) ;
	HAL_Delay(20);	
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature from Register 
 * @param     ICM20948  store Raw Data Of Temprature in GebraBit_ICM20948 Staruct REGISTER_RAW_TEMP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_Temp_Register_Raw_Data(GebraBit_ICM20948 * ICM20948)
{
	uint8_t temp_msb , temp_lsb;
  GB_ICM20948_Read_Reg_Data(ICM20948_TEMP_OUT_H, BANK_0, &temp_msb);
	GB_ICM20948_Read_Reg_Data(ICM20948_TEMP_OUT_L, BANK_0, &temp_lsb);
	ICM20948->REGISTER_RAW_TEMP = (int16_t)((temp_msb << 8) | temp_lsb);
}

/*=========================================================================================================================================
 * @brief     Get Valid Data Of Temprature Base on Datasheet Formula 
 * @param     ICM20948  store Valid Data Of Temprature in GebraBit_ICM20948 Staruct VALID_TEMP_DATA
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_Temp_Valid_Data(GebraBit_ICM20948 * ICM20948)
{ 
  ICM20948->VALID_TEMP_DATA =(ICM20948->REGISTER_RAW_TEMP / 333.87 ) + 21-ROOM_TEMPERATURE_OFFSET;///25 - 8 PCS OFSET!!!
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis GYRO from Register 
 * @param     ICM20948  store Raw Data Of X Axis GYRO DATA in GebraBit_ICM20948 Staruct REGISTER_RAW_GYRO_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_GYRO_X_Register_Raw_DATA(GebraBit_ICM20948 * ICM20948)
{
	uint8_t gyrox_msb , gyrox_lsb;
  GB_ICM20948_Read_Reg_Data( ICM20948_GYRO_XOUT_H, BANK_0, &gyrox_msb);
	GB_ICM20948_Read_Reg_Data( ICM20948_GYRO_XOUT_L, BANK_0, &gyrox_lsb );
	ICM20948->REGISTER_RAW_GYRO_X = (int16_t)((gyrox_msb << 8) | gyrox_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis GYRO from Register 
 * @param     ICM20948  store Raw Data Of Y Axis GYRO DATA in GebraBit_ICM20948 Staruct REGISTER_RAW_GYRO_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_GYRO_Y_Register_Raw_DATA(GebraBit_ICM20948 * ICM20948)
{
	uint8_t gyroy_msb , gyroy_lsb;
  GB_ICM20948_Read_Reg_Data( ICM20948_GYRO_YOUT_H, BANK_0, &gyroy_msb);
	GB_ICM20948_Read_Reg_Data( ICM20948_GYRO_YOUT_L, BANK_0, &gyroy_lsb );
	ICM20948->REGISTER_RAW_GYRO_Y = (int16_t)((gyroy_msb << 8) | gyroy_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis GYRO from Register 
 * @param     ICM20948  store Raw Data Of Z Axis GYRO DATA in GebraBit_ICM20948 Staruct REGISTER_RAW_GYRO_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_GYRO_Z_Register_Raw_DATA(GebraBit_ICM20948 * ICM20948)
{
	uint8_t gyroz_msb , gyroz_lsb;
  GB_ICM20948_Read_Reg_Data( ICM20948_GYRO_ZOUT_H, BANK_0, &gyroz_msb);
	GB_ICM20948_Read_Reg_Data( ICM20948_GYRO_ZOUT_L, BANK_0, &gyroz_lsb );
	ICM20948->REGISTER_RAW_GYRO_Z = (int16_t)((gyroz_msb << 8) | gyroz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis GYRO Base on GebraBit_ICM20948 Staruct SCALE_FACTOR 
 * @param     ICM20948  store Valid Data Of X Axis GYRO in GebraBit_ICM20948 Staruct VALID_GYRO_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_GYRO_DATA_X_Valid_Data(GebraBit_ICM20948 * ICM20948)
{
  ICM20948->VALID_GYRO_DATA_X =(ICM20948->REGISTER_RAW_GYRO_X /ICM20948->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis GYRO Base on GebraBit_ICM20948 Staruct SCALE_FACTOR 
 * @param     ICM20948  store Valid Data Of Y Axis GYRO in GebraBit_ICM20948 Staruct VALID_GYRO_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_GYRO_DATA_Y_Valid_Data(GebraBit_ICM20948 * ICM20948)
{
  ICM20948->VALID_GYRO_DATA_Y =(ICM20948->REGISTER_RAW_GYRO_Y /ICM20948->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis GYRO Base on GebraBit_ICM20948 Staruct SCALE_FACTOR 
 * @param     ICM20948  store Valid Data Of Z Axis GYRO in GebraBit_ICM20948 Staruct VALID_GYRO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_GYRO_DATA_Z_Valid_Data(GebraBit_ICM20948 * ICM20948)
{
  ICM20948->VALID_GYRO_DATA_Z =(ICM20948->REGISTER_RAW_GYRO_Z /ICM20948->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis ACCEL from Register 
 * @param     ICM20948  store Raw Data Of X Axis ACCEL DATA in GebraBit_ICM20948 Staruct REGISTER_RAW_ACCEL_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_X_Register_Raw_DATA(GebraBit_ICM20948 * ICM20948)
{
	uint8_t accelx_msb , acclx_lsb;
  GB_ICM20948_Read_Reg_Data( ICM20948_ACCEL_XOUT_H, BANK_0, &accelx_msb);
	GB_ICM20948_Read_Reg_Data( ICM20948_ACCEL_XOUT_L, BANK_0, &acclx_lsb );
	ICM20948->REGISTER_RAW_ACCEL_X = (int16_t)((accelx_msb << 8) | acclx_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis ACCEL from Register 
 * @param     ICM20948  store Raw Data Of Y Axis ACCEL DATA in GebraBit_ICM20948 Staruct REGISTER_RAW_ACCEL_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_Y_Register_Raw_DATA(GebraBit_ICM20948 * ICM20948)
{
	uint8_t accely_msb , accly_lsb;
  GB_ICM20948_Read_Reg_Data( ICM20948_ACCEL_YOUT_H, BANK_0, &accely_msb);
	GB_ICM20948_Read_Reg_Data( ICM20948_ACCEL_YOUT_L, BANK_0, &accly_lsb );
	ICM20948->REGISTER_RAW_ACCEL_Y = (int16_t)((accely_msb << 8) | accly_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis ACCEL from Register 
 * @param     ICM20948  store Raw Data Of Z Axis ACCEL DATA in GebraBit_ICM20948 Staruct REGISTER_RAW_ACCEL_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_Z_Register_Raw_DATA(GebraBit_ICM20948 * ICM20948)
{
	uint8_t accelz_msb , acclz_lsb;
  GB_ICM20948_Read_Reg_Data( ICM20948_ACCEL_ZOUT_H, BANK_0, &accelz_msb);
	GB_ICM20948_Read_Reg_Data( ICM20948_ACCEL_ZOUT_L, BANK_0, &acclz_lsb );
	ICM20948->REGISTER_RAW_ACCEL_Z = (int16_t)((accelz_msb << 8) | acclz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis ACCEL Base on GebraBit_ICM20948 Staruct SCALE_FACTOR 
 * @param     ICM20948  store Valid Data Of X Axis ACCEL in GebraBit_ICM20948 Staruct VALID_ACCEL_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_DATA_X_Valid_Data(GebraBit_ICM20948 * ICM20948)
{
	float scale_factor = ICM20948->ACCEL_SCALE_FACTOR;
  ICM20948->VALID_ACCEL_DATA_X =(ICM20948->REGISTER_RAW_ACCEL_X /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis ACCEL Base on GebraBit_ICM20948 Staruct SCALE_FACTOR 
 * @param     ICM20948  store Valid Data Of Y Axis ACCEL in GebraBit_ICM20948 Staruct VALID_ACCEL_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_DATA_Y_Valid_Data(GebraBit_ICM20948 * ICM20948)
{
	float scale_factor = ICM20948->ACCEL_SCALE_FACTOR;
  ICM20948->VALID_ACCEL_DATA_Y =(ICM20948->REGISTER_RAW_ACCEL_Y /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis ACCEL Base on GebraBit_ICM20948 Staruct SCALE_FACTOR 
 * @param     ICM20948  store Valid Data Of Z Axis ACCEL in GebraBit_ICM20948 Staruct VALID_ACCEL_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_DATA_Z_Valid_Data(GebraBit_ICM20948 * ICM20948)
{
	float scale_factor = ICM20948->ACCEL_SCALE_FACTOR;
  ICM20948->VALID_ACCEL_DATA_Z =(ICM20948->REGISTER_RAW_ACCEL_Z /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Temprature Directly 
 * @param     ICM20948       GebraBit_ICM20948 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_Temperature(GebraBit_ICM20948 * ICM20948)
{
  GB_ICM20948_Get_Temp_Register_Raw_Data  (ICM20948);
	GB_ICM20948_Get_Temp_Valid_Data(ICM20948);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     ICM20948       GebraBit_ICM20948 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_XYZ_GYROSCOPE(GebraBit_ICM20948 * ICM20948)
{
	GB_ICM20948_Get_GYRO_X_Register_Raw_DATA(ICM20948);
	GB_ICM20948_Get_GYRO_DATA_X_Valid_Data(ICM20948);
	GB_ICM20948_Get_GYRO_Y_Register_Raw_DATA(ICM20948);
	GB_ICM20948_Get_GYRO_DATA_Y_Valid_Data(ICM20948);
	GB_ICM20948_Get_GYRO_Z_Register_Raw_DATA(ICM20948);
	GB_ICM20948_Get_GYRO_DATA_Z_Valid_Data(ICM20948);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     ICM20948       GebraBit_ICM20948 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_XYZ_ACCELERATION(GebraBit_ICM20948 * ICM20948)
{
	GB_ICM20948_Get_ACCEL_X_Register_Raw_DATA(ICM20948);
	GB_ICM20948_Get_ACCEL_DATA_X_Valid_Data(ICM20948);
	GB_ICM20948_Get_ACCEL_Y_Register_Raw_DATA(ICM20948);
	GB_ICM20948_Get_ACCEL_DATA_Y_Valid_Data(ICM20948);
	GB_ICM20948_Get_ACCEL_Z_Register_Raw_DATA(ICM20948);
	GB_ICM20948_Get_ACCEL_DATA_Z_Valid_Data(ICM20948);
}
/*=========================================================================================================================================
 * @brief     Get XYZ Magnetometer Directly 
 * @param     ICM20948       GebraBit_ICM20948 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_XYZ_AK09916_Magnetometer(GebraBit_ICM20948 * ICM20948)
{
	uint8_t mag_data[6];
  GB_ICM20948_AUX_I2C_Read_Data(ICM20948, SLAVE_0,ICM20948_AK09916_I2C_ADDRESS,ICM20948_AK09916_MAG_XOUT_L,BIT_LENGTH_6,mag_data);
	ICM20948->REGISTER_RAW_MAG_X = (int16_t)((mag_data[1] << 8) | mag_data[0]); 
	ICM20948->REGISTER_RAW_MAG_Y = (int16_t)((mag_data[3] << 8) | mag_data[2]); 
	ICM20948->REGISTER_RAW_MAG_Z = (int16_t)((mag_data[5] << 8) | mag_data[4]); 
	ICM20948->VALID_MAG_DATA_X = (float)(ICM20948->REGISTER_RAW_MAG_X * ICM20948->AK09916_MAG_SCALE_FACTOR); 
	ICM20948->VALID_MAG_DATA_Y = (float)(ICM20948->REGISTER_RAW_MAG_Y * ICM20948->AK09916_MAG_SCALE_FACTOR);  
	ICM20948->VALID_MAG_DATA_Z = (float)(ICM20948->REGISTER_RAW_MAG_Z * ICM20948->AK09916_MAG_SCALE_FACTOR);  
	GB_ICM20948_Read_AK09916_Status_2( ICM20948);//////ST2 register has a role as data reading end register, also. When any of measurement data register (HXL to TMPS) is read in Continuous measurement mode 1, 2, 3, 4, it means data reading start and taken as data reading until ST2 register is read. Therefore, when any of measurement data is read, be sure to read ST2 register at the end.
}

/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION , GYRO and Temprature From FIFO
 * @param     ICM20948       GebraBit_ICM20948 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_GYRO_TEMP_MAG_From_Registers(GebraBit_ICM20948 * ICM20948)
{
  if (IS_Ready==GB_ICM20948_Check_Data_Preparation(ICM20948))
	 {
		 ICM20948->GET_DATA =  FROM_REGISTER ; 
	   GB_ICM20948_Get_Temperature( ICM20948 );
	   GB_ICM20948_Get_XYZ_ACCELERATION( ICM20948);
		 GB_ICM20948_Get_XYZ_GYROSCOPE( ICM20948);
	 }
	 GB_ICM20948_Get_XYZ_AK09916_Magnetometer(ICM20948);
}

/*=========================================================================================================================================
 * @brief     Separate XYZ ACCELERATION , GYROSCOPE and Temprature Data From FIFO and caculate Valid data
 * @param     ICM20948  store Valid Data Of XYZ ACCEL Axis and temp from FIFO TO GebraBit_ICM20948 Staruct VALID_FIFO_DATA_X , VALID_FIFO_DATA_Y ,VALID_FIFO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_FIFO_Data_Partition_ACCEL_GYRO_MAG_XYZ_TEMP(GebraBit_ICM20948 * ICM20948)
{
	uint16_t i,offset=0;
  float accel_scale_factor = ICM20948->ACCEL_SCALE_FACTOR;
 if ( (ICM20948->TEMP_TO_FIFO == Enable ) && ( ICM20948->ACCEL_TO_FIFO == Enable ) && ( ICM20948->GYRO_TO_FIFO == Enable ) )
	{
	 for ( i = 0 ; i < (PACKET_QTY_IN_FULL_FIFO-1) ; i++ )	
		{
			ICM20948->VALID_FIFO_ACCEL_X[i] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2; 
			ICM20948->VALID_FIFO_ACCEL_Y[i] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20948->VALID_FIFO_ACCEL_Z[i] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20948->VALID_FIFO_GYRO_X[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
			offset += 2; 
			ICM20948->VALID_FIFO_GYRO_Y[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
			offset += 2;
			ICM20948->VALID_FIFO_GYRO_Z[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
			offset += 2;
			ICM20948->VALID_FIFO_TEMP[i]    = (((int16_t)( (ICM20948->FIFO_DATA[offset] << 8)| ICM20948->FIFO_DATA[offset+1]))/ 333.87) + 25-ROOM_TEMPERATURE_OFFSET ;
			offset += 2;
		} 
			ICM20948->VALID_FIFO_ACCEL_X[292] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2; 
			ICM20948->VALID_FIFO_ACCEL_Y[292] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20948->VALID_FIFO_ACCEL_Z[292] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20948->VALID_FIFO_GYRO_X[292]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
	}
	//	if ( (ICM20948->TEMP_TO_FIFO == Enable ) && ( ICM20948->ACCEL_TO_FIFO == Enable ) && ( ICM20948->GYRO_TO_FIFO == Enable )&& ( ICM20948->SLAVE_TO_FIFO == Enable ))
//	{
//	 for ( i = 0 ; i < (PACKET_QTY_IN_FULL_FIFO-1) ; i++ )	
//		{
//			ICM20948->VALID_FIFO_ACCEL_X[i] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
//			offset += 2; 
//			ICM20948->VALID_FIFO_ACCEL_Y[i] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
//			offset += 2;
//			ICM20948->VALID_FIFO_ACCEL_Z[i] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
//			offset += 2;
//			ICM20948->VALID_FIFO_GYRO_X[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
//			offset += 2; 
//			ICM20948->VALID_FIFO_GYRO_Y[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
//			offset += 2;
//			ICM20948->VALID_FIFO_GYRO_Z[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
//			offset += 2;
//			ICM20948->VALID_FIFO_TEMP[i]    = (((int16_t)( (ICM20948->FIFO_DATA[offset] << 8)| ICM20948->FIFO_DATA[offset+1]))/ 333.87) + 25-ROOM_TEMPERATURE_OFFSET ;
//			offset += 2;
//			ICM20948->VALID_FIFO_MAG_X[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))*ICM20948->AK09916_MAG_SCALE_FACTOR ;
//			offset += 2;
//			ICM20948->VALID_FIFO_MAG_Y[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))*ICM20948->AK09916_MAG_SCALE_FACTOR ;
//			offset += 2;
//			ICM20948->VALID_FIFO_MAG_Z[i]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))*ICM20948->AK09916_MAG_SCALE_FACTOR ;
//			offset += 2;
//		}
//			ICM20948->VALID_FIFO_ACCEL_X[204] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
//			offset += 2; 
//			ICM20948->VALID_FIFO_ACCEL_Y[204] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
//			offset += 2;
//			ICM20948->VALID_FIFO_ACCEL_Z[204] = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/accel_scale_factor ;
//			offset += 2;
//			ICM20948->VALID_FIFO_GYRO_X[204]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
//			offset += 2; 
//			ICM20948->VALID_FIFO_GYRO_Y[204]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
//			offset += 2;
//			ICM20948->VALID_FIFO_GYRO_Z[204]  = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))/ICM20948->PRECISE_GYRO_SF ;
//			offset += 2;
//			ICM20948->VALID_FIFO_TEMP[204]    = (((int16_t)( (ICM20948->FIFO_DATA[offset] << 8)| ICM20948->FIFO_DATA[offset+1]))/ 333.87) + 25-ROOM_TEMPERATURE_OFFSET ;
//			offset += 2;
//			ICM20948->VALID_FIFO_MAG_X[204]   = ((int16_t)( (ICM20948->FIFO_DATA[offset] << 8) | ICM20948->FIFO_DATA[offset+1]))*ICM20948->AK09916_MAG_SCALE_FACTOR ;
//	}
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION , GYRO and Temprature From FIFO
 * @param     ICM20948       GebraBit_ICM20948 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_ACCEL_GYRO_MAG_TEMP_From_FIFO(GebraBit_ICM20948 * ICM20948)
{	  
	if (IS_Ready==GB_ICM20948_Check_Data_Preparation(ICM20948))
	{
		  if (FIFO_OVERFLOW == GB_ICM20948_Check_FIFO_Overflow(ICM20948))
		  {
        GB_ICM20948_GET_FIFO_Count(ICM20948);
				GB_ICM20948_Read_FIFO(ICM20948,FIFO_DATA_BUFFER_SIZE);				
				GB_ICM20948_FIFO_Data_Partition_ACCEL_GYRO_MAG_XYZ_TEMP(ICM20948); 
        //memset(ICM20948->FIFO_DATA , 0, FIFO_DATA_BUFFER_SIZE*sizeof(uint8_t));				
				GB_ICM20948_FIFO_Reset();
				ICM20948->GET_DATA =  FROM_FIFO ;
		  } 
	}	
}
/*=========================================================================================================================================
 * @brief     Get Data From ICM20948
 * @param     ICM20948       GebraBit_ICM20948 Staruct
 * @param     get_data       Determine Method of reading data from sensoe : FROM_REGISTER or FROM_FIFO
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20948_Get_Data(GebraBit_ICM20948 * ICM20948 , ICM20948_Get_DATA get_data)
{
 if( (get_data == FROM_REGISTER)&&(ICM20948->FIFO == Disable) )
	 GB_ICM20948_Get_ACCEL_GYRO_TEMP_MAG_From_Registers(ICM20948);
 else if ((get_data == FROM_FIFO)&&(ICM20948->FIFO == Enable)) 
	GB_ICM20948_Get_ACCEL_GYRO_MAG_TEMP_From_FIFO(ICM20948); 
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/


