/*
 * lcd.h
 *
 *  Created on: 01-Nov-2021
 *      Author: bandari karthik kumar
 */

#ifndef LCD_H_
#define LCD_H_

#include"stm32f401xx.h"
#include"stm32f401xx_gpio_driver.h"


void lcd_init(void);
void lcd_send_command(uint8_t cmd);

void lcd_print_string(char *message);
void lcd_print_char(uint8_t data);

void udelay(uint32_t cnt);
void mdelay(uint32_t cnt);


#define LCD_GPIO_PORT    GPIOB
#define LCD_GPIO_RS      GPIO_PIN_NO_4
#define LCD_GPIO_RW      GPIO_PIN_NO_5
#define LCD_GPIO_EN      GPIO_PIN_NO_8

#define LCD_GPIO_D4      GPIO_PIN_NO_0
#define LCD_GPIO_D5      GPIO_PIN_NO_1
#define LCD_GPIO_D6      GPIO_PIN_NO_2
#define LCD_GPIO_D7      GPIO_PIN_NO_3



#endif /* LCD_H_ */
