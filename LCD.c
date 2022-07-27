/*
 * LCD.c
 *
 *  Created on: 20 апр. 2021 г.
 *      Author: PC3
 */

#include "LCD.h"

SettTypeDef Sett;

void LCD_WriteCMD(unsigned int reg)
{
    *(uint16_t *) (LCD_CMD_ADDR) = reg;
}

void LCD_WriteDATA(unsigned int data)
{
    *(uint16_t *) (LCD_DATA_ADDR)= data;
}

void LCD_Init (void)
{

	//HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
	/*GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);*/
	//Sett.CurFont=(MaxFontMicro *)resname_Century16_En_f;
	//Sett.CurX=0;
	//Sett.CurY=0;
	//Sett.FontColor=WHITE;
	//Sett.BackColor=BLACK;

	LCD_MX_FSMC_Init();

	//LCD_MX_TIM13_Init();
	//HAL_TIM_Base_Start(&htim13);
	HAL_GPIO_WritePin(LCDRST_GPIO_Port, LCDRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCDRST_GPIO_Port, LCDRST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	//HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);

	/*LCD_WriteCMD(0XF2); //Adjust Control 2 (F2h) c.272 ILI9488 //@RC-021.3
	LCD_WriteDATA(0x58); //(0x18)
	LCD_WriteDATA(0xA3);
	LCD_WriteDATA(0x12);
	LCD_WriteDATA(0x02);
	LCD_WriteDATA(0XB2);
	LCD_WriteDATA(0x12);
	LCD_WriteDATA(0xFF);
	LCD_WriteDATA(0x10);
	LCD_WriteDATA(0x00);*/

	//LCD_WriteCMD(0x01);//Software Reset

	LCD_WriteCMD(0xCF);
	LCD_WriteDATA(0x00);
	LCD_WriteDATA(0x83);
	LCD_WriteDATA(0x30);

	/*LCD_WriteCMD(0XF8); //Adjust Control 4 (F8h) c.277 ILI9488
	LCD_WriteDATA(0x21);
	LCD_WriteDATA(0x04);

	LCD_WriteCMD(0XF9);
	LCD_WriteDATA(0x00);
	LCD_WriteDATA(0x08);*/

	LCD_WriteCMD(0x36); //0b00110110//0x36//Memory Access Control (36h) c.192 ILI9488, без неё только шум
	LCD_WriteDATA(0b00101000);//0b00101000//0x28

	LCD_WriteCMD(0xB4);//Display Inversion Control (B4h) c.225 ILI9488 //можно не использовать
	LCD_WriteDATA(0x00);//(0x00)

	/*LCD_WriteCMD(0xC1); //Power Control 2 (C1h) c.239 ILI9488
	LCD_WriteDATA(0x44); //(0x41)*/

	/*LCD_WriteCMD(0xC5); //VCOM Control (C5h) c.243 ILI9488
	LCD_WriteDATA(0x00);
	LCD_WriteDATA(0x53);*/

   /*LCD_WriteCMD(0xE0); //PGAMCTRL (Positive Gamma Control) (E0h) c.265 ILI9488 //можно не использовать
	LCD_WriteDATA(0x0F);
	LCD_WriteDATA(0x1B);
	LCD_WriteDATA(0x18);
	LCD_WriteDATA(0x0B);
	LCD_WriteDATA(0x0E);
	LCD_WriteDATA(0x09);
	LCD_WriteDATA(0x47);
	LCD_WriteDATA(0x94);
	LCD_WriteDATA(0x35);
	LCD_WriteDATA(0x0A);
	LCD_WriteDATA(0x13);
	LCD_WriteDATA(0x05);
	LCD_WriteDATA(0x08);
	LCD_WriteDATA(0x03);
	LCD_WriteDATA(0x00);*/

	LCD_WriteCMD(0x26);
	LCD_WriteDATA(0x01);

	LCD_WriteCMD(0xE0); //PGAMCTRL (Positive Gamma Control) (E0h) c.265 ILI9488 //можно не использовать
		LCD_WriteDATA(0x1F);
		LCD_WriteDATA(0x1A);
		LCD_WriteDATA(0x18);
		LCD_WriteDATA(0x0A);
		LCD_WriteDATA(0x0F);
		LCD_WriteDATA(0x06);
		LCD_WriteDATA(0x45);
		LCD_WriteDATA(0x87);
		LCD_WriteDATA(0x32);
		LCD_WriteDATA(0x0A);
		LCD_WriteDATA(0x07);
		LCD_WriteDATA(0x02);
		LCD_WriteDATA(0x07);
		LCD_WriteDATA(0x05);
		LCD_WriteDATA(0x00);

	/*LCD_WriteCMD(0XE1); // NGAMCTRL (Negative Gamma Control) (E1h) c.266 ILI9488 //можно не использовать
	LCD_WriteDATA(0x0F);//()
	LCD_WriteDATA(0x3A);
	LCD_WriteDATA(0x37);
	LCD_WriteDATA(0x0B);
	LCD_WriteDATA(0x0C);
	LCD_WriteDATA(0x05);
	LCD_WriteDATA(0x4A);
	LCD_WriteDATA(0x24);
	LCD_WriteDATA(0x39);
	LCD_WriteDATA(0x07);
	LCD_WriteDATA(0x10);
	LCD_WriteDATA(0x04);
	LCD_WriteDATA(0x27);
	LCD_WriteDATA(0x25);
	LCD_WriteDATA(0x00);*/
		LCD_WriteCMD(0XE1); // NGAMCTRL (Negative Gamma Control) (E1h) c.266 ILI9488 //можно не использовать
			LCD_WriteDATA(0x00);//()
			LCD_WriteDATA(0x25);
			LCD_WriteDATA(0x27);
			LCD_WriteDATA(0x05);
			LCD_WriteDATA(0x10);
			LCD_WriteDATA(0x09);
			LCD_WriteDATA(0x3A);
			LCD_WriteDATA(0x78);
			LCD_WriteDATA(0x4D);
			LCD_WriteDATA(0x05);
			LCD_WriteDATA(0x18);
			LCD_WriteDATA(0x0D);
			LCD_WriteDATA(0x38);
			LCD_WriteDATA(0x3A);
			LCD_WriteDATA(0x1F);

	/*LCD_WriteCMD(0XE2);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);
	LCD_WriteDATA(0b11110000);

	LCD_WriteCMD(0X38);

	LCD_WriteCMD(0X13);*/


	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12))
	{
		LCD_WriteCMD(0x21); //Display Inversion ON (21h) ILI9488
	}
	else if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11))
	{
		LCD_WriteCMD(0x36); //Memory Access Control (36h) c.192 ILI9488, без неё только шум
		LCD_WriteDATA(0x70);
	}
	else if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10))
	{
		LCD_WriteCMD(0x36);
		LCD_WriteDATA((0xa0));//LCD_WriteDATA(0xa0);
	}

	else if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))
		{
			LCD_WriteCMD(0x36);
			LCD_WriteDATA(0b11101000);//LCD_WriteDATA(0xa0);
		}



	LCD_WriteCMD(0x3A); //Interface Pixel Format (3Ah) c.200 ILI9488, без неё только шум
	LCD_WriteDATA(0b01010101);//(0x55)

	LCD_WriteCMD(0x11); //Sleep OUT (11h) c.166 ILI9488, без неё только подсветка
	HAL_Delay(120);
	LCD_WriteCMD(0x29); //Display ON (29h) c.174 ILI9488, без неё только подсветка





}

void LCD_MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 3;
  Timing.AddressHoldTime =3;
  Timing.DataSetupTime = 3;
  Timing.BusTurnAroundDuration = 3;
  Timing.CLKDivision = 3;
  Timing.DataLatency = 3;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

}

void LCD_Fill (uint16_t Color)
{
	//LCD_WaitDMA_Opration();
	LCD_SetWindow(0,LCD_WIDTH-1,0,LCD_HEIGHT-1);
	//Sett.BackColor=Color;
	//Sett.DMA_TrSize=85000;
uint32_t Size=LCD_SCREEN_SIZE;

	while(Size--)
	{
		*(uint16_t *) (0x60020000)=Color;
	}

}

void LCD_DrawRectangle(uint16_t x1,uint16_t x2, uint16_t y1,uint16_t y2, uint32_t Color)// отрисовка прямоугольника
{
	LCD_SetWindow(x1,x2-1,y1,y2-1);
	uint32_t Size=(x2-x1)*(y2-y1);

		while(Size--)
		{
			*(uint16_t *) (0x60020000)=Color;
		}
}

/*void LCD_DrawChar(uint16_t x1, uint16_t y1,uint16_t size, uint32_t Color)// {0x0E,0x11,0x11,0x11,0x1F,0x11,0x11,0x00}
{
 uint16_t simbol[8]={0x0E,0x11,0x11,0x11,0x1F,0x11,0x11,0x00};


}*/

void LCD_DrawPixel(uint16_t x,uint16_t y, uint32_t Color)// отрисовка пикселя
{

	LCD_SetWindow(x,x,y,y);
	uint32_t SizePix=5;


		while(SizePix--)
		{

			*(uint16_t *) (0x60020000)=Color;

		}
}


void LCD_DrawLine(uint16_t start_x,uint16_t start_y,uint16_t width_x,char l[8], uint32_t Color, uint32_t Background)// отрисовка линии
{


	LCD_SetWindow(start_x,start_x+width_x-1,start_y,start_y);


for (uint8_t i=0; i<8; i++)
{
	uint16_t Size=width_x/8;
	if ((1 <<(7-i))&l[0])

	{
		while(Size--)
		{
	*(uint16_t *) (0x60020000)=Color;
	}

		}

	else

	{
		while(Size--)
				{
	*(uint16_t *) (0x60020000)=Background;

				}
	}

}
}






void LCD_Fill2 (uint16_t Color)
{
	//LCD_WaitDMA_Opration();
	//LCD_SetWindow(0,LCD_WIDTH-1,0,LCD_HEIGHT-1);
	//Sett.BackColor=Color;
	//Sett.DMA_TrSize=85000;
uint32_t Size=LCD_SCREEN_SIZE;

	while(Size--)
	{
		*(uint16_t *) (0x60020000)=Color;
	}

}

void LCD_Fill3 (uint16_t Color)
{
	//LCD_WaitDMA_Opration();
	LCD_SetWindow(50,LCD_WIDTH-1,10,LCD_HEIGHT-1);
	//Sett.BackColor=Color;
	//Sett.DMA_TrSize=85000;
uint32_t Size=LCD_SCREEN_SIZE;

	while(Size--)
	{
		*(uint16_t *) (0x60020000)=Color;
	}

}

void LCD_SetWindow(uint16_t x1,uint16_t x2, uint16_t y1, uint16_t y2)
{
	LCD_WriteCMD(0x2a); //Column Address Set (2Ah) c.175 ILI9488 //столбци
	LCD_WriteDATA(x1>>8); //>>8
	LCD_WriteDATA(x1);
	LCD_WriteDATA(x2>>8);//>>8
	LCD_WriteDATA(x2);

	LCD_WriteCMD(0x2b); //Page Address Set (2Bh) c.177 ILI9488 //строки
	LCD_WriteDATA(y1>>8);//>>8
	LCD_WriteDATA(y1);
	LCD_WriteDATA(y2>>8);//>>8
	LCD_WriteDATA(y2);

	LCD_WriteCMD(0x2c); //Memory Write (2Ch) c.179  ILI9488

}

void LCD_WaitDMA_Opration(void)
{
		while(Sett.DMA_Busy);
}
