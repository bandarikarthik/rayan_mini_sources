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
	mdelay(1000);


	lcd_print_string("karthik");


	mdelay(1000);mdelay(1000);mdelay(1000);

	lcd_send_command(0xC0);

	lcd_print_string("manjula");


	mdelay(1000);mdelay(1000);mdelay(1000);

   lcd_send_command(0x01);


	mdelay(1000);


}
	return 0;

}
