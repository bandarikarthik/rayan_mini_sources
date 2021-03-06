/*
 * lcd.c
 *
 *  Created on: 01-Nov-2021
 *      Author: bandari karthik kumar
 *
 */
#include"lcd.h"

static void lcd_enable(void);
static void write_4_bits(uint8_t value);



void lcd_send_command(uint8_t cmd)
{
	/* RS=0 for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/*R/nW = 0, for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(cmd >> 4);
	write_4_bits(cmd & 0x0F);

}

void lcd_init(void)

{
	//1. config GPIO pins
	GPIO_Handle_t lcd_signal;



	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS ;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode =   GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType =  GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);


	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);


	    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

		mdelay(100);
		lcd_send_command(0x33);
		lcd_send_command(0x32);
		lcd_send_command(0x0C);
		lcd_send_command(0x01);

}
/* writes 4 bits of data/command on to D4,D5,D6,D7 lines */
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D4, ((value >> 0) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D5, ((value >> 1) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D6, ((value >> 2) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT,LCD_GPIO_D7, ((value >> 3) & 0x1) );

	lcd_enable();

}


static void lcd_enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100);/* execution time > 37 micro seconds */
}


 void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

 void udelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1); i++);
}


void lcd_print_char(uint8_t data)
{
	/* RS=1 for LCD user data */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	/*R/nW = 0, for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(data >> 4);  /*Higher nibble*/
	write_4_bits(data & 0x0F); /*Lower nibble*/

}


void lcd_print_string(char *message)
{

      do
      {
          lcd_print_char((uint8_t)*message++);
      }
      while (*message != '\0');

}



