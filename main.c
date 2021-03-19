
#include "stm32f4xx_hal.h" // Device header using HAL library
#define set_value(i, value) (*((&(TIM4->CCR1)) + i) = value) //macro function to update the channel value at timer 4

#define PWM_MAX 5000
/* Global Variables */
GPIO_InitTypeDef GPIO_InitStruct; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.
GPIO_InitTypeDef GPIO_InitStruct_PWM; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.
TIM_HandleTypeDef TIM_InitStruct;  // Initiate the structure that contains the configuration information for the timers
TIM_HandleTypeDef TIM4_InitStruct;
TIM_HandleTypeDef TIM3_InitStruct;
TIM_OC_InitTypeDef TIM4_OCInitStructure; // Initiate the structure that contains the configuration information for the specified output capture
TIM_OC_InitTypeDef TIM3_OCInitStructure;

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

  // Based on the STM32F407 clock configuration
	// https://stm32f4-discovery.net/2015/01/properly-set-clock-speed-stm32f4xx-devices/
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
    GPIO_InitStruct.Pin     = GPIO_PIN_15;  //BLUE 
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);    // Init GPIOD 
}

void InitLEDPWM(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable clock to GPIO-D for LEDs
    // Set GPIOD Pins Parameters 
		// GREEN / ORANGE/RED
    GPIO_InitStruct_PWM.Pin     = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14; //PD12: TIM4_CH1, PD13: TIM4_CH2, PD14: TIM4_CH3, 
    GPIO_InitStruct_PWM.Mode    = GPIO_MODE_AF_PP;
    GPIO_InitStruct_PWM.Pull    = GPIO_PULLDOWN;
    GPIO_InitStruct_PWM.Speed   = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct_PWM.Alternate = GPIO_AF2_TIM4; // Assign those pins alternate function in TIM3 
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct_PWM);    // Init GPIOD 
}

void InitButton(void)
{
	  __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable clock to GPIO-D for LEDs
    // Set GPIOA Pin Parameters, pin 0 for button
    GPIO_InitStruct.Pin     = GPIO_PIN_0;
    GPIO_InitStruct.Mode    = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull    = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Init GPIOA
}	

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

void InitTimer2(void)
{
		__HAL_RCC_TIM2_CLK_ENABLE(); // Enable clock to TIM2 from APB1 bus (42Mhz max)xPLL_P = 84MHz 
	  // TIM2 is configure to 1hz: 1 times in 1s or 1000000us
		// Tim_Tick_Freq = 10Khz. => Time_Freq = 2Hz
    TIM_InitStruct.Instance = TIM2;
		TIM_InitStruct.Init.Period = 5000-1;	
    TIM_InitStruct.Init.Prescaler   = 8400-1;
		TIM_InitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
		TIM_InitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TIM_InitStruct.Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(&TIM_InitStruct); // Init TIM2 
	
		HAL_TIM_Base_Start_IT(&TIM_InitStruct); // Enable timer-2 IRQ interrupt
		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt at IRQ-Level
		
    HAL_TIM_Base_Start(&TIM_InitStruct); // Start TIM2

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

void InitTimer4(void)
{
		//__TIM1_CLK_ENABLE();
		__HAL_RCC_TIM4_CLK_ENABLE(); // Enable clock to TIM4 from APB2 bus (48Mhz max)xPLL_P = 84MHz
		 
	// TIM4 is configure to 50hz: 50 times in 1s or 1000000us
	
		//Tim_Tick_Freq = 1Mhz. Tim_Freq = 50Hz
    TIM4_InitStruct.Instance = TIM4;
		TIM4_InitStruct.Init.Period = 20000-1;
    TIM4_InitStruct.Init.Prescaler   = 84-1;
		TIM4_InitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
		TIM4_InitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TIM4_InitStruct.Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(&TIM4_InitStruct); // Init TIM4
	
		/*//if you would like to enable interrupt
		HAL_TIM_Base_Start_IT(&TIM4_InitStruct); // Enable timer-4 IRQ interrupt
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt at IRQ-Level
		*/
    HAL_TIM_Base_Start(&TIM4_InitStruct); // Start TIM4
}

//Control the Servo using timer 3 instead
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

/*To control LED (only at TIM4)*/
//PD12: TIM4_CH1, PD13: TIM4_CH2, PD14: TIM4_CH3
void SetupPWM_TIM4()
{
	HAL_TIM_PWM_Init(&TIM4_InitStruct);
	
	TIM4_OCInitStructure.OCMode = TIM_OCMODE_PWM1; //Set output capture as PWM mode
  TIM4_OCInitStructure.Pulse = 0; // Initial duty cycle at 0%
  TIM4_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH; // HIGH output compare active
  TIM4_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&TIM4_InitStruct, &TIM4_OCInitStructure, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM4_InitStruct, TIM_CHANNEL_1); // Start PWM at channel 1
	HAL_TIM_PWM_ConfigChannel(&TIM4_InitStruct, &TIM4_OCInitStructure, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM4_InitStruct, TIM_CHANNEL_2); // Start PWM at channel 2
	HAL_TIM_PWM_ConfigChannel(&TIM4_InitStruct, &TIM4_OCInitStructure, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM4_InitStruct, TIM_CHANNEL_3); // Start PWM at channel 3
}



int main () {
    
    HAL_Init(); // Initialize HAL Library.
		SystemClock_Config();
    InitLED();
		InitLEDPWM();	
		InitButton();
		InitServo();
		
		InitTimer2(); //For the blue LED flashing continuously
		InitTimer3(); // For the servo
		InitTimer4(); //For the 3 LEDS flashing up and down
		
		SetupPWM_TIM4();
		SetupPWM_TIM3();
		
		for (;;) {
		
    	//Control two servos using timer 3 PWM, channel 1&2 
    	TIM3->CCR1 = 500; // 900 us
			TIM3->CCR2 = 500; // 900 us
			HAL_Delay(2000);     
			TIM3->CCR1 = 1500; // 1500 us
			TIM3->CCR2 = 1500; // 1500 us
			HAL_Delay(2000);
			//ADDED ON 2/3/2021
			TIM3->CCR1 = 2500; 
			TIM3->CCR2 = 2500;
			HAL_Delay(2000);
			
			//////////////////
			TIM3->CCR1 = 1500; // 2100 us
			TIM3->CCR2 = 1500; // 2100 us
			HAL_Delay(2000);
			
			// Control Green, Orange and Red LEDs using timer 4 PWM (update using a macro fuinction)
			/*for (int i=0; i<3; i++)// at channel 1, 2, and 3
			{
				for (int j = 0; j <= 5000; j += 100) //increase brightness
				{
					set_value(i, j);
					HAL_Delay(10);
				}

				for (int j = 5000; j > -1; j -= 100) //decrease brightness
				{
					set_value(i, j);
					HAL_Delay(10);
				}
			}*/
		}
	}


