/*
 * lcd.c
 *
 *  Created on: 12 maj 2016
 *      Author: postep
 */

#ifndef HEADER_LCD
#define HEADER_LCD
#include "stm32f1xx_hal.h"
#include "font5x8.h"


void LCD_writeControlValue(char valueDI, char valueRW){
	HAL_GPIO_WritePin(LCD_DI_GPIO_Port, LCD_DI_Pin, valueDI);
	HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, valueRW);
}

void LCD_wait(){
	asm("nop");asm("nop");asm("nop");asm("nop");
}

void LCD_enablePulse(){

	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	LCD_wait();
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	LCD_wait();
}

void LCD_writeCS(char cs1, char cs2){
	HAL_GPIO_WritePin(LCD_CS1_GPIO_Port, LCD_CS1_Pin, cs1);
	HAL_GPIO_WritePin(LCD_CS2_GPIO_Port, LCD_CS2_Pin, cs2);
}

void LCD_writeDataByte(char byte){
	HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, (byte&0x01));
	HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, (byte&0x02));
	HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, (byte&0x04));
	HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, (byte&0x08));
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, (byte&0x10));
	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, (byte&0x20));
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, (byte&0x40));
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, (byte&0x80));
}

void LCD_setStartLine(char line){
	LCD_writeControlValue(0, 0);
	LCD_writeCS(0, 0);
	LCD_writeDataByte(0xc0 | line);
	LCD_enablePulse();
}

void LCD_init(){
	osDelay(500);
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
	LCD_writeCS(0, 0);
	LCD_writeControlValue(0, 0);
	LCD_writeDataByte(0x3f);
	LCD_enablePulse();
	LCD_setStartLine(0);
}

void LCD_goTo(char x, char y){
	if(x < 64){
		LCD_writeCS(0, 1);
	}else{
		LCD_writeCS(1, 0);
		x -= 64;
	}

	LCD_writeControlValue(0, 0);

	y = (y | 0xb8 ) & 0xbF;
	LCD_writeDataByte(y);
	LCD_enablePulse();

	x = (x | 0x40 ) & 0x7f;
	LCD_writeDataByte(x);
	LCD_enablePulse();
}
void LCD_writeLine(char y, char * string);

void LCD_clear(){
	for (char i = 0; i < 8; i++){
		LCD_writeLine(i,"");
	}
}

void LCD_write(char x, char y, char byte){
	LCD_goTo(x, y);
	LCD_writeControlValue(1, 0);
	LCD_writeDataByte(byte);
	LCD_enablePulse();
}

void LCD_helpWriteChar(char x, char y, char ch){
	ch -= 32;
	char i = 0;
	LCD_writeControlValue(1, 0);
	for (i = 0; i < 5; i++){
		if(x == 64){
			LCD_goTo(x, y);
			LCD_writeControlValue(1, 0);
		}
		LCD_writeDataByte(font5x8[ch*5+i]);
		LCD_enablePulse();
		x++;
	}
}
void LCD_writeChar(char x, char y, char ch){
	x *= 5;
	LCD_goTo(x, y);
	LCD_helpWriteChar(x, y, ch);
}

void LCD_writeLine(char y, char * string){
	char i = 0;
	LCD_goTo(0, y);
	while(string[i] != '\0' && i < 25){
		LCD_helpWriteChar(i*5, y, string[i]);
		i++;
	}
	while(i < 25){
		LCD_helpWriteChar(i*5, y, ' ');
		i++;
	}
	LCD_writeDataByte(0);
	LCD_enablePulse();
	LCD_writeDataByte(0);
	LCD_enablePulse();
	LCD_writeDataByte(0);
	LCD_enablePulse();
}
#endif
