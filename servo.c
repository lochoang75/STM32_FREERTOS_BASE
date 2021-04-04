#include "servo.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

extern GPIO_InitTypeDef GPIO_InitStruct;
void InitServo(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock to GPIO-B for PB6 and PB7
    // Set GPIOB Pins Parameters, PB6 and PB7 
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;  //PB6: TIM4_CH1 and PB7: TIM4_CH2
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //Alternate Function Push Pull Mode 
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No Pull-up or Pull-down activation
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // Assign those pins alternate function in TIM4 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Init GPIOB		
}
void setServoAngle(int angle){
	TIM3->CCR1 = angle;
	vTaskDelay(100);
}

void resetServoAngle(){
	TIM3->CCR1 = 500;
	vTaskDelay(100);
}
