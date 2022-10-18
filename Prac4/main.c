/* USER CODE BEGIN Header */
/**
*******************
Info:		STM32 DMA and PWM with HAL
Author:		Amaan Vally
*******************
In this practical you will to use PWM using DMA on the STM32 using the HAL.
We also set up an interrupt to switch the waveform between various LUTs.

  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//TO DO:
//TASK 2
//Assign values to NS, TIM2CLK and F_SIGNAL
#define TIM2CLK 48000000
#define F_SIGNAL 4000
#define NS 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;
int check = 0;
int prev = 0;

/* USER CODE BEGIN PV */

//TO DO:
//TASK 1
//Create global variables for LUTs

//sin wave LUT (values from 0 - 1023 , frequency 1Hz)
uint32_t sin_LUT[NS] = {512,537,562,587,611,636,660,684,707,730,753,774,796,816,836,855,
                        873,890,907,922,937,950,963,974,
                        984,993,1001,1008,1013,1017,1021,1022,
                        1023,1022,1021,1017,1013,1008,1001,993,
                        984,974,963,950,937,922,907,890,
                        873,855,836,816,796,774,753,730,
                        707,684,660,636,611,587,562,537,
                        512,486,461,436,412,387,363,339,
                        316,293,270,249,227,207,187,168,
                        150,133,116,101,86,73,60,49,
                        39,30,22,15,10,6,2,1,
                        0,1,2,6,10,15,22,30,
                        39,49,60,73,86,101,116,133,
                        150,168,187,207,227,249,270,293,
                        316,339,363,387,412,436,461,486};

//sin wave LUT (values from 0 - 1023 , frequency 1Hz)
uint32_t saw_LUT[NS] = {0 , 8.05511811  , 16.11023622  , 24.16535433 ,  32.22047244,
                        40.27559055 ,  48.33070866  , 56.38582677  , 64.44094488  , 72.49606299,
                        80.5511811 ,   88.60629921  , 96.66141732 , 104.71653543 , 112.77165354,
                        120.82677165 , 128.88188976 , 136.93700787  ,144.99212598 , 153.04724409,
                        161.1023622  , 169.15748031,  177.21259843  ,185.26771654 , 193.32283465,
                        201.37795276 , 209.43307087 , 217.48818898  ,225.54330709 , 233.5984252,
                        241.65354331 , 249.70866142 , 257.76377953  ,265.81889764 , 273.87401575,
                        281.92913386 , 289.98425197 , 298.03937008 , 306.09448819 , 314.1496063,
                        322.20472441 , 330.25984252 , 338.31496063  ,346.37007874 , 354.42519685,
                        362.48031496 , 370.53543307 , 378.59055118  ,386.64566929 , 394.7007874,
                        402.75590551 , 410.81102362,  418.86614173  ,426.92125984 , 434.97637795,
                        443.03149606 , 451.08661417 , 459.14173228 , 467.19685039 , 475.2519685,
                        483.30708661 , 491.36220472 , 499.41732283 , 507.47244094 , 515.52755906,
                        523.58267717 , 531.63779528 , 539.69291339 , 547.7480315  , 555.80314961,
                        563.85826772 , 571.91338583 , 579.96850394 , 588.02362205 , 596.07874016,
                        604.13385827 , 612.18897638 , 620.24409449 , 628.2992126  , 636.35433071,
                        644.40944882 , 652.46456693 , 660.51968504 , 668.57480315 , 676.62992126,
                        684.68503937 , 692.74015748 , 700.79527559 , 708.8503937  , 716.90551181,
                        724.96062992 , 733.01574803 , 741.07086614 , 749.12598425 , 757.18110236,
                        765.23622047 , 773.29133858 , 781.34645669 , 789.4015748  , 797.45669291,
                        805.51181102 , 813.56692913 , 821.62204724 , 829.67716535 , 837.73228346,
                        845.78740157 , 853.84251969 , 861.8976378  , 869.95275591 , 878.00787402,
                        886.06299213 , 894.11811024 , 902.17322835 , 910.22834646 , 918.28346457,
                        926.33858268 , 934.39370079 , 942.4488189 ,  950.50393701 , 958.55905512,
                        966.61417323 , 974.66929134 , 982.72440945 , 990.77952756 , 998.83464567,
                        1006.88976378 ,1014.94488189 , 0};

//sin wave LUT (values from 0 - 1023 , frequency 1Hz)
uint32_t triangle_LUT[NS] = {16,32,48,64,80,96,112,128,
                             144,160,176,192,208,224,240,256,
                             272,288,304,320,336,352,368,384,
                             400,416,432,448,464,480,496,512,
                             527,543,559,575,591,607,623,639,
                             655,671,687,703,719,735,751,767,
                             783,799,815,831,847,863,879,895,
                             911,927,943,959,975,991,1007,1023,
                             1007,991,975,959,943,927,911,895,
                             879,863,847,831,815,799,783,767,
                             751,735,719,703,687,671,655,639,
                             623,607,591,575,559,543,527,512,
                             496,480,464,448,432,416,400,384,
                             368,352,336,320,304,288,272,256,
                             240,224,208,192,176,160,144,128,
                             112,96,80,64,48,32,16,0};

//TO DO:
//TASK 3
//Calculate TIM2_Ticks

//TIM2_TICKS = (TIM2CLK/F_SIGNAL)*NS
uint32_t TIM2_Ticks = ((TIM2CLK/F_SIGNAL)*NS)/1000; // adjust the 100 based on what it looks like
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //TO DO:
  //TASK 4
  //Start TIM3 in PWM mode on channel 1
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  //Start TIM2 in Output Compare mode on channel 1.
  HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);
  //Start the DMA in interrupt mode.
  uint32_t DestAddress = (uint32_t) &(TIM3->CCR1);
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) &sin_LUT,DestAddress,128);
  //Start the DMA transfer
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


	  //No need to do anything in the main loop for this practical


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1; //To make the frequency what we want it to be
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	//TO DO:
		//TASK 5
		uint32_t tick = HAL_GetTick();
        //debounce delay
		if((tick-prev)>100)
		{
		  prev = tick;
          //Disable DMA transfer
		  HAL_DMA_Abort(&hdma_tim2_ch1);
		  uint32_t DestAddress = (uint32_t) &(TIM3->CCR1);

          //"check" keeps track of clicks of the button and changes the waveform accordingly
          //the first time the button is clicked, the sinwave will be generated
          //the second time, the triangle wave
          //the third time, the sawtooth wave
          //then back to sine, in a circular loop

		  if(check == 0) //
		  {
              //set check to ensure triangle wave is dispalyed on next click
		  	check = 1;
             //start DMA in IT mode with new source and re-enable transfer
		    HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) &saw_LUT,DestAddress,128);
		  	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);


		   }
		  else if(check == 1){
             //set check to ensure sawtooth wave is dispalyed on next click
		  	check = 2;
            //start DMA in IT mode with new source and re-enable transfer
		  	HAL_DMA_Start_IT(&hdma_tim2_ch1,(uint32_t) &triangle_LUT,DestAddress,128);
		  	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
		   }
		  else if(check == 2)
		  {
               //set check to ensure sine  wave is dispalyed on next click
		  	check = 0;
		   //start DMA in IT mode with new source and re-enable transfer
           HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) &sin_LUT,DestAddress,128);
		  	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
		  }
		}

	   //Remember to debounce using HAL_GetTick()


		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); // Clear interrupt flags
	}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */