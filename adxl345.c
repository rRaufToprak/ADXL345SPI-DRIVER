/*
 * adxl345.c
 *
 *  Created on: 19.11.2023
 *  Author: rRaufToprak
 */

#include "adxl345.h"

ADXL345_HandleTypeDef adxl345;

uint8_t testID(void)
{
	uint8_t id;
	uint8_t addr = 0x00;
	cs_pin_low();
	HAL_SPI_Transmit(&hspi1, &addr, 1, 2000);
	HAL_SPI_Receive(&hspi1, &id, 1, 2000);
	cs_pin_high();

	return id;
}
void ADXL345_Init(void)
{
	ADXL345_write_reg(DATA_FORMAT, 0x01);
	ADXL345_write_reg(POWER_CTL, 0x00);
	ADXL345_write_reg(POWER_CTL, 0x08);
}
uint8_t ADXL345_read_reg(uint8_t addr)
{
	uint8_t data;
	cs_pin_low();
	HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
	HAL_SPI_Receive(&hspi1, &data, 1, 1000);
	cs_pin_high();
	return data;
}
void ADXL345_write_reg(uint8_t addr, uint8_t data)
{
	uint8_t tx_buff[2];
	addr |= 0x40;
	tx_buff[0] = addr;
	tx_buff[1] = data;

	cs_pin_low();
	HAL_SPI_Transmit(&hspi1, tx_buff, 2, 1000);
	cs_pin_high();
}
void cs_pin_low(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void cs_pin_high(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void ADXL345_readAxisData(void)
{
	uint8_t data[6];
	uint8_t addr = 0x32;
	addr |= 0x80;
	addr |= 0x40;
	cs_pin_low();
	HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
	HAL_SPI_Receive(&hspi1, data, 6, 1000);
	cs_pin_high();
	adxl345.x = data[1]<<8 | data[0];
	adxl345.y = data[3]<<8 | data[2];
	adxl345.z = data[5]<<8 | data[4];
}
void ADXL345_tapDetectionEnable(uint8_t x, uint8_t y,uint8_t z)
{
	uint8_t data = 0;
	if(x==1){
		data |= 0x04;
	}
	if(y==1){
		data |= 0x02;
	}
	if(z==1){
		data |= 0x01;
	}
	ADXL345_write_reg(TAP_AXES, 0x00);
	ADXL345_write_reg(TAP_AXES, data);
	
	ADXL345_write_reg(THRESH_TAP, 32);
	//Set DUR to 0.01 second
	ADXL345_write_reg(DUR, 16);
	//Set Latent to 0.1 second 
	ADXL345_write_reg(Latent, 80);
	//Set Window to 0.2 second 
	ADXL345_write_reg(Window, 160);
	//Set INT_ENABLE 
	ADXL345_write_reg(INT_ENABLE, 0x60);
	//Set INT_MAP
	ADXL345_write_reg(INT_MAP, 0x00);
}
uint8_t ADXL345_tapDetection(void)
{
	uint8_t tap;
	uint8_t data;
	
	tap = ADXL345_read_reg(INT_SOURCE);
	if((tap>>6) & 0x01){
		data =  1;//single tap detection
	}else if((tap>>5) & 0x01){
		data = 2;//double tap detection
	}else if((tap>>5) &(tap>>6)& 0x01){
	data=3;
	} 
	else{
		data = 0;
	}
	return data;
}
void ADXL345_freeFallDetectionEnable(void)
{
	uint8_t data;
	ADXL345_write_reg(THRESH_FF, 0x05);
	ADXL345_write_reg(TIME_FF, 0x14);
	data = ADXL345_read_reg(INT_ENABLE);
	data |= 1 << 2;
	ADXL345_write_reg(INT_ENABLE, data);
}
uint8_t ADXL345_freeFallDetection(void)
{
	uint8_t free_fall;
	free_fall = ADXL345_read_reg(INT_SOURCE);
	if((free_fall>>2) & 0x01){
		return 1;
	}else{
		return 0;
	}
}

