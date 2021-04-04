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
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"
#include "MY_CS43L22.h"
#include "custom_queue.h"
#include "main.h"
#include "coffee_task.h"
#include "task_queue.h"
#include "task_schedule.h"
#include "servo.h"

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

#define DEFAULT_DURATION 500

//const int servoAngleIterateTask[TASK_SIZE]={400,600,800,1000,1200,1400,1600,1800};


//const float toneArr[TONE_ARR_SIZE] = {E6,F6,E6,F6,E6,F6,E6,F6};

//const int servoAngleIterateTone[TONE_ARR_SIZE]={400,600,800,1000,1200,1400,1600,1800};
//const int servoAngleIterateLength[LENGTH_ARR_SIZE]={400,800,1200,1600};
//const int length[LENGTH_ARR_SIZE]={500,1000,1500,2000};

/* The task functions prototype*/
void vTaskBlueLED( void *pvParameters );
void vTaskOrangeLED( void *pvParameters );
void vTaskBeep( void *pvParameters ) ;

/* Global Variables */
GPIO_InitTypeDef GPIO_InitStruct; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.
TIM_HandleTypeDef TIM_InitStruct;  // Initiate the structure that contains the configuration information for the timers
TIM_HandleTypeDef TIM_InitStruct_2;

/////////
system_state_t system_state = sys_state_idle;
static volatile bool button_trigger_flag = 0;
button_action_t btn_action = button_no_action;
volatile bool timeOut=false;
//select_tone_t toneList[MAX_TONE_ID];
//int toneId = 0;


bool initialTimer=false;

//////////

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TaskHandle_t xBlueLEDHandler = NULL;
TaskHandle_t xButtonHandler = NULL;
TaskHandle_t xScheduleHandler = NULL;
QueueHandle_t userInputEventQueue;
QueueHandle_t playerEventQueue;
//added by DUY
TaskHandle_t xButtonHandler_1 = NULL;
TaskHandle_t xButtonHandler_2 = NULL;
TaskHandle_t xButtonHandler_3 = NULL;

TIM_HandleTypeDef TIM3_InitStruct;
TIM_OC_InitTypeDef TIM3_OCInitStructure;



float mySinVal;
float sample_dt;
uint16_t sample_N;
uint16_t i_t;
uint32_t myDacVal;
buttonType button = NONE;


int16_t dataI2S[100];
bool soundOn;

void vTaskBlueLED( void *pvParameters );
void vTaskOrangeLED( void *pvParameters );
///////////////////PREDEFINED FUNCTION////////////////////////
void turnOnRedLED(void);
void turnOnBlueLED(void);
void turnOnOrangeLED(void);
void turnOnGreenLED(void);
void turnOnAllLEDS(void);
void turnOffAllLEDS(void);
void vDetectButtonType(void*);
static void buttonIrqCallback(void);
void turnOnCurLEDS(led);
void blinkLEDS(led);
void vTaskDetectUserState(void *pvParameters);
void vTaskPlaySong(void *pvParameters);
void playTone(float tone, int time_ms);
void vTaskRunSchedule(void *pvParameters);

/*
Use SysTick_Handler to wake xPortSysTickHandler when the FreeRTOS's 
scheduler is started. SysTick_Handler (HAL_IncTick) is needed by 
HAL driver such as DAC that depends on system tick but it's "conflict" 
with FreeRTOS xPortSysTickHandler because the SysTimer is used by FreeRTOS. 
*/

void counting500ms(){
		__HAL_RCC_TIM5_CLK_ENABLE(); // Enable clock to TIM2 from APB1 bus (42Mhz max)xPLL_P = 84MHz
		TIM_InitStruct_2.Instance = TIM5;
    TIM_InitStruct_2.Init.Prescaler   = 1400-1;
		TIM_InitStruct_2.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_InitStruct_2.Init.Period = 12000-1;		
		TIM_InitStruct_2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TIM_InitStruct_2.Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(&TIM_InitStruct_2); // Init TIM2 
		//HAL_TIM_Base_Start_IT(&TIM_InitStruct); // Enable timer-2 IRQ interrupt
		HAL_NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt at IRQ-Level
    //HAL_TIM_Base_Start(&TIM_InitStruct); // Start TIM2
}


void SysTick_Handler(void)
{
  HAL_IncTick();
	
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
			xPortSysTickHandler();
  }
	
}

/** ISR Function **/
void EXTI0_IRQHandler(void) 
{
	// Reset existing interrupt at GPIO_PIN_0
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0); //alternative: HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0)
	// Resume the button task from the ISR
	//BaseType_t xYieldRequiredCheck = xTaskResumeFromISR(xButtonHandler);
	//portYIELD_FROM_ISR(xYieldRequiredCheck); // Yield to avoid delayed until the next time the scheduler
	buttonIrqCallback();
	return;
}


void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	
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
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
	
	// Select peripheral clock for I2S
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
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

void InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level LEDs and DAC output */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}



void InitTimer3(void)
{
		//__TIM1_CLK_ENABLE();
		__HAL_RCC_TIM3_CLK_ENABLE(); // Enable clock to TIM3 from APB2 bus (48Mhz max)xPLL_P = 84MHz
		 
	// TIM4 is configure to 50hz: 50 times in 1s or 1000000us
		//Tim_Tick_Freq = 1Mhz. Tim_Freq = 50Hz

    TIM3_InitStruct.Instance = TIM3;
		TIM3_InitStruct.Init.Period = 20000-1;
    TIM3_InitStruct.Init.Prescaler   = 84-1;
		TIM3_InitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
		TIM3_InitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TIM3_InitStruct.Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(&TIM3_InitStruct); // Init TIM3
		
		/*//if you would like to enable interrupt
		//HAL_TIM_Base_Start_IT(&TIM3_InitStruct); // Enable timer-3 IRQ interrupt
		//HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		//HAL_NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt at IRQ-Level
		*/
    HAL_TIM_Base_Start(&TIM3_InitStruct); // Start TIM3
}
void SetupPWM_TIM3()
{
	HAL_TIM_PWM_Init(&TIM3_InitStruct);
	
	TIM3_OCInitStructure.OCMode = TIM_OCMODE_PWM1; //Set output capture as PWM mode
  TIM3_OCInitStructure.Pulse = 0; // Initial duty cycle at 0%
  TIM3_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH; // HIGH output compare active
  TIM3_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE; // output compare disable
	HAL_TIM_PWM_ConfigChannel(&TIM3_InitStruct, &TIM3_OCInitStructure, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM3_InitStruct, TIM_CHANNEL_1); // Start PWM at channel 1
	HAL_TIM_PWM_ConfigChannel(&TIM3_InitStruct, &TIM3_OCInitStructure, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM3_InitStruct, TIM_CHANNEL_2); // Start PWM at channel 2
}

void InitButton(void)
{
	  __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock to GPIO-A for button
    // Set GPIOA Pin Parameters, pin 0 for button
    GPIO_InitStruct.Pin     = GPIO_PIN_0;
	  // Specifies the trigger signal active edge for the EXTI lines. (e.g. Rising = button is pressed)
    GPIO_InitStruct.Mode    = GPIO_MODE_IT_FALLING; //ori GPIO_MODE_IT_FALLING
    GPIO_InitStruct.Pull    = GPIO_NOPULL; //ori GPIO_NOPULL
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
	
	InitGPIO();
	//InitLED();
	InitButton();
	InitServo();
	InitTimer3();
	SetupPWM_TIM3();
	soundOn = true;

	counting500ms();//inititate the 500ms timer
	// user input event queue
	userInputEventQueue = xQueueCreate(2, sizeof(button_action_t));
	playerEventQueue = xQueueCreate(2, sizeof(player_event_t));
	

	/* Create one of the two tasks. */
	
	//xTaskCreate(	vTaskBlueLED,		/* Pointer to the function that implements the task. */
	//				"vTaskBlueLED",	/* Text name for the task.  This is to facilitate debugging only. */
	//				STACK_SIZE_MIN,		/* Stack depth in words. */
	//				NULL,		/* We are not using the task parameter. */
	//				tskIDLE_PRIORITY,			/* This task will run at priority 1. */
	//				&xBlueLEDHandler );		/* We are not using the task handle. */

	//xTaskCreate( vTaskOrangeLED, "vTaskBlueLED", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );

	//xTaskCreate( vTaskButton, "vTaskButton", STACK_SIZE_MIN*2, NULL, tskIDLE_PRIORITY, &xButtonHandler );
	xTaskCreate(vDetectButtonType, "vDetectButtonType",STACK_SIZE_MIN,NULL,3,&xButtonHandler_1);
	xTaskCreate(vTaskDetectUserState, "vDetectUserState",STACK_SIZE_MIN,NULL,3,&xButtonHandler_2);
	xTaskCreate(vTaskRunSchedule, "vTaskPlaySong",STACK_SIZE_MIN,NULL,3,&xButtonHandler_3);
	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/
static void buttonIrqCallback() {
    button_trigger_flag = 1;
		//HAL_Delay(10);
}
static inline bool readGPIOPIn() {
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)){
		return 1; //TODO change this to return value of GPIO
	}
	return 0;
}
static inline void buttonFlagReset() {
    // disable irq first
    button_trigger_flag = 0;
    // enble irq
}
void TIM5_IRQHandler(void){
	HAL_TIM_IRQHandler(&TIM_InitStruct_2);
	if(__HAL_TIM_GET_FLAG(&TIM_InitStruct_2, TIM_FLAG_CC1) != RESET){
		timeOut=true;
	}
	__HAL_TIM_CLEAR_FLAG(&TIM_InitStruct_2, TIM_SR_UIF);
	return;
}
void startTimer500ms(){
	HAL_TIM_Base_Start_IT(&TIM_InitStruct_2);
	HAL_TIM_Base_Start(&TIM_InitStruct_2);
}
void stopTimer500ms(){
	HAL_TIM_Base_Stop_IT(&TIM_InitStruct_2);
	HAL_TIM_Base_Stop(&TIM_InitStruct_2);
}
//task 1
void vDetectButtonType(void *pvParameters){
	while(1){
		vTaskDelay(10);
		switch (system_state) {
            case sys_state_idle:{
                if (button_trigger_flag == 1) {
                    system_state = sys_state_wait_first_pressed_done;
                    //timeout_cnt = LONG_PRESSED_TIMEOUT;
                    btn_action = button_single_pressed;
										//printf("first press is done \n");
                } else {
                    /* Idle state delay for 10ms */
                    //delayms(10);
										//printf("waiting for first press\n");	
										vTaskDelay(10); // Use vtaskDelay
                }
                break;
							}
            case sys_state_wait_first_pressed_done:
								//printf("WE ARE IN WAIT FIRST PRESS DONE, WE ARE IN WAIT FIRST PRESS DONE\n");
                if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0) {
										//printf("0\n");
										buttonFlagReset();
										system_state = sys_state_wait_timeout;										
								} 
								//else
									//printf("1\n");
                break;
            case sys_state_wait_timeout:
								//printf("WE ARE IN WAITING TIME OUT, WE ARE IN WAITING TIME OUT,WE ARE IN WAITING TIME OUT\n");
                if (button_trigger_flag == 0) {
										if(initialTimer==false){
											initialTimer=true;
											//counting500ms();
											startTimer500ms();
										}
										if(timeOut==true){
											//end of the waiting, shut down the timer, SINGLE PRESS
											//stop timer
											stopTimer500ms();
											//reset variables
											initialTimer=false;
											//reset timeout Status
											timeOut=false;
											btn_action = button_single_pressed;
                      system_state = sys_state_idle;
											//printf("single press is done,single press is done,single press is done,single press is done\n");
											//PUSH BUTTON ACTION TO QUEUE
											printf("pushed single to queue\n");
											xQueueSend(userInputEventQueue, &btn_action, portMAX_DELAY);
											break;
										}
                } else {
                    // found trigger
                    btn_action = button_double_pressed;
                    system_state = sys_state_idle;
										//stop timer
										//HAL_TIM_Base_Stop(&TIM_InitStruct);
										//HAL_TIM_Base_Stop_IT(&TIM_InitStruct);
										stopTimer500ms();
										//reset variables
										initialTimer=false;
										//reset timeOut status
										timeOut=false;
										//printf("double press is done,double press is done,double press is done\n");
										//PUSH BUTTON ACTION TO QUEUE
										printf("pushed double to queue\n");
										xQueueSend(userInputEventQueue, &btn_action, portMAX_DELAY);
										//disableInteruptBtn();
										buttonFlagReset();
                    break;
                }
                break;
					
		}
	}
}

schedule_type_t schedule = task_schedule_unknow;
volatile schedule_flag_t schedule_flag = reset;
//TASK 2, for detecting user state
void vTaskDetectUserState(void *pvParameters){
	button_action_t button = button_no_action;
	schedule_type_t tempSchedule = task_schedule_unknow;
	//user_action_t userState = user_select_schedule;
	//select_tone_t tone = {.tone=0,.duration=0};
	//player_event_t player_event = player_stop;
	int durationId = -1;
	led curLedId = none;
	int tonePosition = -1;
	while(1){
		//pop event from message queue
		xQueueReceive(userInputEventQueue, &button, portMAX_DELAY);
		
				switch(button){
					//single press for iterating tones
					case button_single_pressed:
						printf("Single pressed\n");
						if(tempSchedule==task_schedule_unknow){
							tempSchedule = task_schedule_priority;
						}
						else if(tempSchedule==task_schedule_priority){
							tempSchedule = task_schedule_custom;	
						}
						else{
							tempSchedule = task_schedule_priority;
						}				
						curLedId++;
						if(curLedId==green+1){
							curLedId = orange;
						}
						//TODO : SET LED ACCORDING TO SCHEDULE TYPE . DONEE
						turnOffAllLEDS();
						turnOnCurLEDS(curLedId);
						break;
					case button_double_pressed: 
						printf("double pressed\n");
						turnOffAllLEDS();
						blinkLEDS(curLedId);
						//setSchedulerType(schedule);
						turnOffAllLEDS();
						turnOnCurLEDS(curLedId);
						schedule_flag = reset;
						//curLedId=orange;
						schedule = tempSchedule;
						break;
					default:
						break;
				}						
		}
		//check if the queue size = 8. If so, start the task 3 (Playing music)	
		//PUSH EVENT PLAYING MUSIC TO QUEUE
	}
//TASK 3, playing song
void vTaskRunSchedule(void *pvParameters){
	bool finished= false;
	while(1){
		switch(schedule_flag){
			case on:
				finished = startSchedule();
				if( finished&& (schedule_flag==on)){
					schedule_flag=off;
				}
				break;
			case off:
				//idle tasks
				printf("idle task\n");
				resetServoAngle();
				vTaskDelay(1500);
				break;
			case reset:
				if(schedule!=task_schedule_unknow){
					resetTaskQueue();
					setSchedulerType(schedule);
					schedule_flag=on;
				}			
				break;
		}
	}
}





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
	//HAL_IncTick();
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
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

void turnOnCurLEDS(led curLed){
	switch(curLed){
		case orange:
			turnOnOrangeLED();
			break;
		case green:
			turnOnGreenLED();
			break;
		default:
			break;
	}
}


void blinkLEDS(led curLed){
	for(int i=0;i<2;i++){
		switch(curLed){
			case green:
				turnOnGreenLED();
				break;
			case orange:
				turnOnOrangeLED();
				break;
			default:
				break;
		}
		vTaskDelay(200);
		turnOffAllLEDS();
		vTaskDelay(200);
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


