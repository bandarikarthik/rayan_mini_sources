/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Oct 27, 2021
 *      Author: bandari karthik
 */


#include "stm32f401xx_gpio_driver.h"



void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{


	if(EnorDi == ENABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}
		}
	else
	{
		//
	}

}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	 uint32_t temp=0; //temp. register

	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	 {
		 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		 pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		 pGPIOHandle->pGPIOx->MODER |= temp;

	 }
	 else
	 {


	 }
	    //2. configure the speed
	 	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	 	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	 	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	 	//3. configure the pupd settings
	 	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	 	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	 	pGPIOHandle->pGPIOx->PUPDR |= temp;


	 	//4. configure the optype
	 	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	 	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	 	pGPIOHandle->pGPIOx->OTYPER |= temp;




}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber)
{

    uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x0000001);
	return value;

}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx ,uint8_t pinNumber,uint8_t value)
{

	if(value==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<pinNumber);

	}else
	{
		pGPIOx->ODR &= ~(1<<pinNumber);

	}

}

void GPIO_ToggleOutputpin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber)
{

	pGPIOx->ODR ^= (1<<pinNumber);

}

void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi)
{

}

void GPIO_IRQHandling(uint8_t pinNumber)
{

}



