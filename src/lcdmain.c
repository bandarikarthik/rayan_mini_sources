/*
 * lcd_main.c
 *
 *  Created on: 01-Nov-2021
 *      Author: bandari karthik kumar
 */


#include"lcd.h"

int main()
{
	GPIO_PeriClockControl(LCD_GPIO_PORT, ENABLE);
	lcd_init();
while(1)
{
	lcd_print_string("karthik");

	lcd_init();  //after printing re initialse the LCD refresh it by calling lcd_init();

	mdelay(200);

}
	return 0;

}
