/* This FreeRTOS STM32F4 based on:
https://www.instructables.com/Setting-Up-FreeRTOS-From-Scratch-on-STM32F407-Disc/ 
Remarks:
This template is upgraded to the latest FreeRTOS  202012.00 (10.4.3)

FreeRTOSConfig has been added into \FreeRTOS_STM32F4\FreeRTOSv202012.00\FreeRTOS\Source\portable\RVDS\ARM_CM4F

heap_4.c is included in the project (STM32f407VG board) to provide the memory allocation required by the RTOS kernel

System clock is based on HSE configuration

PendSV, SysTcik and SVC handlers "define" have been added to port.c
#define  xPortPendSVHandler      PendSV_Handler 
#define  xPortSysTickHandler     SysTick_Handler
#define  vPortSVCHandler         SVC_Handler
*/

/* FreeRTOS includes. */
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

/* Global Variables */
GPIO_InitTypeDef GPIO_InitStruct; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.

/* The task functions prototype*/
void vTaskBlueLED( void *pvParameters );
void vTaskOrangeLED( void *pvParameters );
void vTaskButton( void *pvParameters );
///////////////////PREDEFINED FUNCTION////////////////////////
void turnOnRedLED(void);
void turnOnBlueLED(void);
void turnOnOrangeLED(void);
void turnOnGreenLED(void);
void turnOnAllLEDS(void);
void turnOffAllLEDS(void);




TaskHandle_t xBlueLEDHandler = NULL;
TaskHandle_t xButtonHandler = NULL;

/** ISR Function **/
void EXTI0_IRQHandler(void) 
{
	// Reset existing interrupt at GPIO_PIN_0
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0); //alternative: HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0)
  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	// Resume the button task from the ISR
	BaseType_t xYieldRequiredCheck = xTaskResumeFromISR(xButtonHandler);
	portYIELD_FROM_ISR(xYieldRequiredCheck); // Yield to avoid delayed until the next time the scheduler
}

void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  // Enable Power Control clock
  __PWR_CLK_ENABLE();

  // The voltage scaling allows optimizing the power consumption when the
  // device is clocked below the maximum system frequency, to update the
  // voltage scaling value regarding system frequency refer to product
  // datasheet.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE Oscillator and activate PLL with HSE as source
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  // Based on the STM32F407 HSE clock configuration
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  // clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void InitLED(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable clock to GPIO-D for LEDs
    // Set GPIOD Pins Parameters 
    GPIO_InitStruct.Pin     = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;  // I/O PD12, PD13, PD14, and PD15
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);    // Init GPIOD 
}

void InitButton(void)
{
	  __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock to GPIO-A for button
    // Set GPIOA Pin Parameters, pin 0 for button
    GPIO_InitStruct.Pin     = GPIO_PIN_0;
	  // Specifies the trigger signal active edge for the EXTI lines. (e.g. Rising = button is pressed)
    GPIO_InitStruct.Mode    = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Init GPIOA
		HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn); // Enable GPIO_PIN_0 interrupt at IRQ-Level
}	
/*-----------------------------------------------------------*/

int main( void )
{
  
	/* essential Board initializations */
	SystemInit();
	HAL_Init();
	/* This function initializes the MCU clock, PLL will be used to generate Main MCU clock */
  SystemClock_Config();
	
	/* Initialize the serial I/O(console ), This function configures Due's CONSOLE_UART */
	
	InitLED();
	InitButton();
	printf("FreeRTOS running on STM32F407 Discovery Board\n");

	/* Create one of the two tasks. */
	xTaskCreate(	vTaskBlueLED,		/* Pointer to the function that implements the task. */
					"vTaskBlueLED",	/* Text name for the task.  This is to facilitate debugging only. */
					STACK_SIZE_MIN,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					tskIDLE_PRIORITY,			/* This task will run at priority 1. */
					&xBlueLEDHandler );		/* We are not using the task handle. */

	/* Create the other task in exactly the same way. */
	xTaskCreate( vTaskOrangeLED, "vTaskBlueLED", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );

	xTaskCreate( vTaskButton, "vTaskButton", STACK_SIZE_MIN*2, NULL, tskIDLE_PRIORITY, &xButtonHandler );
	
	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	
	for( ;; );
}
/*-----------------------------------------------------------*/

void vTaskBlueLED( void *pvParameters ) // Blue LED
{
const char *pcTaskName = "vTaskBlueLED is running\n";
volatile unsigned long ul;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task in debug */
    printf("%s\n",pcTaskName);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		vTaskDelay(500);
	}
} 

void vTaskOrangeLED( void *pvParameters ) // Orange LED
{
	const char *pcTaskName = "vTaskOrangeLED is running\n";
	volatile unsigned long ul;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task in debug */
		printf("%s\n",pcTaskName);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		vTaskDelay(1000);
	}
}

void vTaskButton( void *pvParameters ) // Button Task (interrupt)
{
	const char *pcTaskName = "vTaskButton is running\n";
	
	for(;;)
	{
		vTaskSuspend(NULL); //Suspend itself until resume from ISR
		/* Print out the name of this task in debug */
		printf("%s\n",pcTaskName);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		vTaskDelay(200); //Debounce using FreeRTOS software timer
		/* 
		// Suspend and resume a task 
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
			vTaskSuspend(xBlueLEDHandler);
		}
		else {
			vTaskResume(xBlueLEDHandler);
		}
		*/
	
	}
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}

//ADDED BY DUY, 2021/03/12
int curToneId = 0; // up to 7
buttonType button = NONE;

void vTaskIterateTones(void){
	for(;;)
	{
		vTaskSuspend(NULL); //Suspend itself until resume from ISR, mean waiting for a button press
		if(button==SINGLE_PRESS){
				printf("Iterating tone\n");
		
				//we have 8 tones in total
				switch(curToneId){
					//tone ID = 0
					case 0:
						turnOnOrangeLED();
						//play a tone, this is a missing function 
						break;
					case 1:
						turnOnRedLED();
						break;
					case 2:
						turnOnBlueLED();
						break;
					case 3:
						turnOnGreenLED();
						break;
					case 4:
						turnOnOrangeLED();
						break;
					case 5:
						turnOnRedLED();
						break;
					case 6:
						turnOnBlueLED();
						break;
					case 7:
						turnOnGreenLED();
						break;
				}
				vTaskDelay(200); //Debounce using FreeRTOS software timer
		}
		

		
		
		/* 
		// Suspend and resume a task 
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
			vTaskSuspend(xBlueLEDHandler);
		}
		else {
			vTaskResume(xBlueLEDHandler);
		}
		*/
	
	}
}

void vTaskConfirmTone(void){
	for(;;){
		vTaskSuspend(NULL); //waiting the button press to re-trigger the remaining code
		//confirm when DOUBLE_PRESS
		if(button == DOUBLE_PRESS){
			
			for(int i=0;i<4;i++){
				turnOffAllLEDS();
				if(curToneId==0||curToneId==4){
					if(i%2==0){
						turnOnOrangeLED();
					}
					else
						turnOffAllLEDS();
				}
				else if(curToneId==1||curToneId==5){
					if(i%2==0){
						turnOnRedLED();
					}
					else
						turnOffAllLEDS();
				}
				else if(curToneId==2||curToneId==6){
					if(i%2==0){
						turnOnBlueLED();
					}
					else
						turnOffAllLEDS();
				}
				else if(curToneId==3||curToneId==7){
					if(i%2==0){
						turnOnGreenLED();
					}
					else
						turnOffAllLEDS();
				}
				HAL_Delay(200);
			}
		}
	}
}
toneLength length = HALF_SEC;
void vTaskIterateLength(void){
	for(;;){
		vTaskSuspend(NULL); //waiting the button press to re-trigger the remaining code
		//confirm when SINGLE
		if(button == SINGLE_PRESS){
			//4 types of length in total. 0.5s, 1.0s, 1.5s, 2.0s
			turnOffAllLEDS();
			switch(length){
				case HALF_SEC:
					//play tone in a predefined length, missing this fucntion
					turnOnOrangeLED();
					break;
				case ONE_SEC:
					turnOnRedLED();
					break;
				case ONE_HALF_SEC:
					turnOnBlueLED();
					break;
				case TWO_SEC:
					turnOnGreenLED();
					break;
			}
		}
	}
}

int selectedTonesConfirm = 0;
void vTaskConfirm(void){
	for(;;){
		vTaskSuspend(NULL); //waiting the button press to re-trigger the remaining code
		//confirm when DOUBLE_PRESS
		if(button == DOUBLE_PRESS){
			selectedTonesConfirm++;
			for(int i=0;i<2;i++){
				turnOffAllLEDS();
				HAL_Delay(200);
				turnOnAllLEDS();
				HAL_Delay(200);
			}
			turnOffAllLEDS();
		}
	}
}
// id = 0 -> 7
// id = 1 : 280Hz
// id = 2 : 330Hz
// id = 3 : 450Hz
// id = 4 : 550Hz
// id = 5 : 650Hz
// id = 6 : 750Hz
// id = 7 : 850Hz
void playToneDefaultLength(){
	
}
void playToneGivenLength(){

}



void turnOnRedLED(void){
	//Turn on pin 14
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET);
	//12,13,15
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);
}

void turnOnBlueLED(void){
	//Turn on pin 15
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET);
	//12,13,14 off
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);
}

void turnOnOrangeLED(void){
	//Turn on pin 13
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET);
	//12,14,15 off
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);
}

void turnOnGreenLED(void){
	//Turn on pin 12
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
	//13,14,15 off
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);
}

void turnOnAllLEDS(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
	//13,14,15 off
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET);
}

void turnOffAllLEDS(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);
}

