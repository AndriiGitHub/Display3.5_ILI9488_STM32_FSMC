/*
 * LCD.h
 *
 *  Created on: 20 апр. 2021 г.
 *      Author: PC3
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f2xx_hal.h"
#include "main.h"
#include "stdlib.h"
#include "stdio.h"

	#define LCD_CMD_ADDR 	((uint32_t)0x60000000) // с.1264 FSMC memory banks_Reference manual STM32F207
	#define LCD_DATA_ADDR 	((uint32_t)0x60020000) // с.1264 FSMC memory banks_Reference manual STM32F207


	#define LCD_WIDTH  480
	#define LCD_HEIGHT 320
	#define LCD_SCREEN_SIZE (LCD_WIDTH*LCD_HEIGHT)

	#define TRANSPERENT			0xffffffff
	#define BLACK           0x0000      /*   0,   0,   0 */
	#define NAVY            0x000F      /*   0,   0, 128 */
	#define DGREEN          0x03E0      /*   0, 128,   0 */
	#define DCYAN           0x03EF      /*   0, 128, 128 */
	#define MAROON          0x7800      /* 128,   0,   0 */
	#define PURPLE          0x780F      /* 128,   0, 128 */
	#define OLIVE           0x7BE0      /* 128, 128,   0 */ //ARC-021.4 затемнить  31C0
	#define LGRAY           0xC618      /* 192, 192, 192 *///ARC-021.4 затемнить   4A49
	#define DGRAY           0x7BEF      /* 128, 128, 128 *///ARC-021.4 затемнить   2104
	#define BLUE            0x001F      /*   0,   0, 255 */
	#define GREEN           0x07E0      /*   0, 255,   0 */ //ARC-021.4 затемнить  0620
	#define CYAN            0x07FF      /*   0, 255, 255 *///ARC-021.4 затемнить   0638
	#define RED             0xF800      /* 255,   0,   0 */
	#define MAGENTA         0xF81F      /* 255,   0, 255 */
	#define YELLOW          0xFFE0      /* 255, 255,   0 *///ARC-021.4 затемнить   C640
	#define WHITE           0xFFFF      /* 255, 255, 255 */
	#define ORANGE          0xFD20      /* 255, 165,   0 */
	#define GREENYELLOW     0xAFE5      /* 173, 255,  47 */

	extern SRAM_HandleTypeDef hsram1;

typedef struct
	{
		uint16_t CurX;
		uint16_t CurY;
		uint32_t 	DMA_TrSize;
		uint8_t 	DMA_Busy;
		uint32_t  BackColor;
		uint16_t  FontColor;
		//MaxFontMicro *CurFont;
		uint32_t Arddr;
	}SettTypeDef;

	void LCD_WriteCMD(unsigned int reg);
	void LCD_WriteDATA(unsigned int data);
		void LCD_Init (void);
		void LCD_MX_FSMC_Init(void);
		void LCD_Fill (uint16_t Color);
		void LCD_Fill2 (uint16_t Color);
		void LCD_Fill3 (uint16_t Color);
		void LCD_SetWindow(uint16_t x1,uint16_t x2, uint16_t y1, uint16_t y2);
		void LCD_DrawRectangle(uint16_t x1,uint16_t x2, uint16_t y1,uint16_t y2, uint32_t Color);
		void LCD_DrawPixel(uint16_t x,uint16_t y, uint32_t Color);
		void LCD_DrawLine(uint16_t start_x,uint16_t start_y,uint16_t width_x,char l[8], uint32_t Color, uint32_t Background);
		void LCD_WaitDMA_Opration(void);

#endif /* INC_LCD_H_ */
