/*
 * adxl345.h
 *
 *  Created on: 19.11.2023
 *  Author: rRaufToprak
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdint.h"
#include "stdbool.h"

#define DEVID 0x00
#define THRESH_TAP 0x1D
#define	OFSX 0x1E
#define	OFSY 0x1F
#define OFSZ 0x20
#define DUR 0x21
#define	Latent 0x22
#define Window 0x23
#define THRESH_ACT 0x24
#define THRESH_INACT 0x25
#define TIME_INACT 0x26
#define ACT_INACT_CTL 0x27
#define THRESH_FF 0x28
#define TIME_FF 0x29
#define TAP_AXES 0x2A
#define ACT_TAP_STATUS 0x2B
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37
#define FIFO_CTL 0x38
#define FIFO_STATUS 0x39

extern SPI_HandleTypeDef hspi1;

typedef struct{
	int16_t x,y,z;
	float xg, yg, zg;
}ADXL345_HandleTypeDef;

uint8_t testID(void);
void ADXL345_Init(void);
uint8_t ADXL345_read_reg(uint8_t addr);
void ADXL345_write_reg(uint8_t addr, uint8_t data);
void cs_pin_low(void);
void cs_pin_high(void);
void ADXL345_readAxisData(void);
uint8_t ADXL345_tapDetection(void);
void ADXL345_tapDetectionEnable(uint8_t x, uint8_t y,uint8_t z);
void ADXL345_freeFallDetectionEnable(void);
uint8_t ADXL345_freeFallDetection(void);


#endif

