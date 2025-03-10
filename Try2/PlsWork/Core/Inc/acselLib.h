/*
 * mems.h
 */

#ifndef APPLICATION_USER_CORE_MEMS_H_
#define APPLICATION_USER_CORE_MEMS_H_
#define READWRITE_CMD ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD ((uint8_t)0x40)
#define CS_ON HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define CS_OFF HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define DUMMY_BYTE ((uint8_t)0x00)
#define LIS3DSH_CTRL_REG4_ADDR	0X20
#define LIS3DSH_CTRL_REG5_ADDR	0X24
#endif /* APPLICATION_USER_CORE_MEMS_H_ */

#include "stm32f3xx_hal.h"
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

void Error(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
}

void Success(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

static uint8_t I2C_Read(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
	if(status != HAL_OK) Error();
	else Success();
	return value;
}

static void I2C_Write(uint16_t Addr, uint8_t Reg,  uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
	if(status != HAL_OK) Error();
	else Success();
}

uint8_t I2C_ReadID(uint16_t Addr)
{
	uint8_t ctrl = 0x00;
	ctrl = I2C_Read(Addr, 0x0F);
	return ctrl;
}

void Accel_Ini(void)
{
	uint8_t ctrl;
	if (I2C_ReadID(0x33)==0x33)
	{
		ctrl=0b10010111;
		I2C_Write(0x32,0x20,ctrl);
		ctrl=0b00010000;
		I2C_Write(0x32,0x22,ctrl);
		ctrl=0b00001000;
		I2C_Write(0x32,0x23,ctrl);
		ctrl=0b00000000;
		I2C_Write(0x32,0x25,ctrl);
	}
	HAL_Delay(500);
}

void Accel_GetXYZ(int16_t* pData)
{
	uint8_t buffer[6];
	uint8_t i=0;

	buffer[0] = I2C_Read(0x32,0x28);
	buffer[1] = I2C_Read(0x32,0x29);
	buffer[2] = I2C_Read(0x32,0x2A);
	buffer[3] = I2C_Read(0x32,0x2B);
	buffer[4] = I2C_Read(0x32,0x2C);
	buffer[5] = I2C_Read(0x32,0x2D);

	for(i=0;i<3;i++)
	{
		pData[i]=((int16_t)((uint16_t)buffer[2*i+1]<<8)+buffer[2*i]);
	}
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		Error();
	}
	else
	{
		Success();
	}
}

void Gyro_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if(NumByteToRead>0x01)
	{
		ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
	}
	else
	{
		ReadAddr |= (uint8_t)READWRITE_CMD;
	}
	CS_ON;
	SPIx_WriteRead(ReadAddr);
	while(NumByteToRead > 0x00)
	{
		*pBuffer=SPIx_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}
	CS_OFF;
}

void Gyro_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	CS_OFF;
	if(NumByteToWrite>0x01)
	{
		WriteAddr |= (uint8_t) MULTIPLEBYTE_CMD;
	}
	CS_ON;
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite > 0x00)
	{
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	CS_OFF;
}

uint8_t Gyro_ReadID(void)
{
	uint8_t ctrl = 0;
	Gyro_IO_Read(&ctrl,0x0F,1);
	return ctrl;
}

void Gyro_Ini(void)
{
	uint8_t ctrl = 0x00;

	if(Gyro_ReadID() == 0xD4)
	{
		ctrl=0b01111111;
		Gyro_IO_Write(&ctrl, 0x20,1);
		ctrl=0b00000000;
		Gyro_IO_Write(&ctrl, 0x21,1);

		ctrl=0b00001000;
		Gyro_IO_Write(&ctrl, 0x22,1);

		ctrl=0b00010000;
		Gyro_IO_Write(&ctrl, 0x23,1);
		ctrl=0b00010000;
		Gyro_IO_Write(&ctrl, 0x24,1);
	}
	else Error();
	HAL_Delay(500);
}

void Gyro_GetXYZ(int16_t* pData)
{
	uint8_t buffer[6];
	float valueinfloat = 0;

	Gyro_IO_Read((uint8_t*)&buffer[0], 0x28,1);
	Gyro_IO_Read((uint8_t*)&buffer[1], 0x29,1);
	Gyro_IO_Read((uint8_t*)&buffer[2], 0x2A,1);
	Gyro_IO_Read((uint8_t*)&buffer[3], 0x2B,1);
	Gyro_IO_Read((uint8_t*)&buffer[4], 0x2C,1);
	Gyro_IO_Read((uint8_t*)&buffer[5], 0x2D,1);

	for(int i=0;i<3;i++)
	{
		valueinfloat = ((buffer[2*i+1] << 8) + buffer[2*i]);
		pData[i]=(int16_t)valueinfloat;
	}
}
