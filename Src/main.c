/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
 int pulse;
double rate;
char rateConvertString[15], degConvertString[15];
double T = 0.01;
double rate_mm;
signed long Error, pre_Error = 0, pre_pre_Error = 0;
double P_part, I_part, D_part, Kp, Kd, Ki;
double Output, pre_Output = 0;
double duty;
signed long speed;
signed long position;
float deg_mm,deg,vitri;
unsigned int RUN = 1, forward_reverse = 1, rLeft;
int rate_mm_int=0,deg_mm_int=0;

char Rx_data[20];
int dex1 = 0;
int dex2 = 0;
unsigned int checkData;
char dataSend[1];
char Rx_data_temp[20];
int i = 0, j = 0;
int Start_Process = 0;
int pre_pulse = 0;
int display = 0;
int start_data=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void chieuquay(int forward_reverse)
{
    if(forward_reverse==1)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);
    }   
    else if(forward_reverse==0)
    {    
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_RESET);
    }   
}       
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  HAL_UART_Receive_IT(&huart5,(uint8_t *)Rx_data,1);      //Nhan du lieu va luu vao Rx_data
    checkData=Rx_data[0];                                   //byte[0] cua Rx_data -> checkData
    if(checkData==37)                                       //check xem byte[0]== '%' (ki tu bat dau frame)
		{                
		 dataSend[0]='A';                                       // Neu dung frame -> gui ACK len PC
		 HAL_UART_Transmit_IT(&huart5,(uint8_t *)dataSend,1);
		 start_data=1;
		}
	if (checkData==101)                // Ki tu ket thuc chuoi 'e'=101
    {
        i = 0;
        Start_Process=1;                 //   % data e
			  start_data=0;
    }
	 if(start_data==1)
	 {
		 if(checkData!=37)                   
			{ 
				Rx_data_temp[i]=checkData;   //luu vao mang Rx_data_temp[]
				i+=1;
			}
		}
	 
}
void send_data(void)
{
    if (speed==1)
    {
        sprintf(rateConvertString,"%0.0f\r\n", rate);       
			  while (dataSend[0]!=10)                      // ki tu xuong dong \r\n ='10'
        {
        dataSend[0]=(int)rateConvertString[j];             
        j+=1;
        HAL_UART_Transmit(&huart5,(uint8_t *)dataSend,1,100);
        }
    }
    else
    {
			  deg=vitri*360/800;
        sprintf(degConvertString,"%0.0f\r\n",deg);
			  while(dataSend[0]!=10)
        {
        dataSend[0]=(int)degConvertString[j];
        j+=1;
        HAL_UART_Transmit(&huart5,(uint8_t *)dataSend,1,100);
        }
    }
    j=0;
    dataSend[0]=0;
}
void Stop()
{  
  	rate_mm=0;
    deg_mm=0;
    RUN=0;
    position=0;
    speed=0;
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,0);   
}

void convert()
{
    pulse =__HAL_TIM_GET_COUNTER(&htim3);
    if(RUN==0)
        htim3.Instance->CNT=0;   
		if(speed==1)
		{
		    htim3.Instance->CNT=0;
		    rate=(float)pulse*60*100/800;   
		}            
    if(position==1)
    {
        rate=(float)(pulse-pre_pulse)*60*100/800;
        pre_pulse=pulse;
    }         
}
void PID_Speed(int rate_mm,TIM_HandleTypeDef *htim)
{
   // Kp=0.001;
	//  Ki=0.8;
	//  Kd=0.01;			
    Error=rate_mm-rate;
	
    P_part=Kp*(Error-pre_Error);
    I_part=0.5*Ki*T*(Error + pre_Error);
    D_part=Kd/T*(Error-2*pre_Error+pre_pre_Error);
    Output=Output+P_part+I_part+D_part;
	  pre_pre_Error=pre_Error;
    pre_Error=Error;
    
    if(RUN==0)
    {
      Output=0;
    }
    if(Output>=999)
       Output=999;
    if(Output<=0)
       Output=0;
    forward_reverse=1;
    chieuquay(forward_reverse);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,Output);           //PWM
}

void PID_Position(int deg_mm,TIM_HandleTypeDef *htim)
{
 
 //  Kp=0.08;
 //  Ki=0.5;
 //  Kd=0.9;
  // 	Kp=0.1;        
	//	Ki=2;         
	//	Kd=0.03;
    vitri=pulse;
    Error=deg_mm-vitri;   
    P_part=Kp*(Error-pre_Error);
    I_part=0.5*Ki*T*(Error+pre_Error);
    D_part=Kd/T*(Error-2*pre_Error+pre_pre_Error);
    Output=Output+P_part+I_part+D_part;
		
    pre_pre_Error=pre_Error;
    pre_Error=Error;
		if (Error==0)
			 Output=0;
		if(Error>0)
			 forward_reverse=1;
		else if(Error<0)
		   forward_reverse=0;
		
    if (Output>220)
        Output=220;
    if (Output<-220)
        Output=-220;
    chieuquay(forward_reverse);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,Output);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim2.Instance)
    {
       convert();
       display+=1;
		   if(RUN==1)
			 {
			    if(speed==1)
             PID_Speed(rate_mm,&htim3);
          else if(position==1)
             PID_Position(deg_mm,&htim3);
		   }
     }
}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    
	speed = 0;
  position = 0;
  deg_mm = 0;
  rate_mm = 0;
  rate = 0;
  HAL_UART_Receive_IT(&huart5,(uint8_t *)Rx_data,1);
  HAL_UART_Transmit_IT(&huart5,(uint8_t *)dataSend,1);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {   
    if (Start_Process==1)
    {
      Start_Process=0;
			switch(Rx_data_temp[0])                // Kiem tra  Rx_data_temp[0]== lenh yeu cau              
			{
				case 73:                             // 'I'=73
					if (speed==1)
					{
            rate_mm+=100;
            PID_Speed(rate_mm,&htim3);
					}
					else if(position==1)
					{  
					  deg_mm+=60;
					  PID_Position(deg_mm,&htim3);
					}
				break;
				case 68:                               // 'D'=68
					if (speed==1)
					{
            rate_mm-=100;
            PID_Speed(rate_mm,&htim3);
					}
					else if(position==1)
					{ 
						deg_mm-=60;
						PID_Position(deg_mm,&htim3);
					}
				break;
				case 115:                               // 's'=115
						Stop();
            RUN=1;
            position=0;
            speed=1;
            rate_mm=(Rx_data_temp[1]-48)*1000+(Rx_data_temp[2]-48)*100+(Rx_data_temp[3]-48)*10+(Rx_data_temp[4]-48);
				    
				    Kp=(Rx_data_temp[5]-48)+(Rx_data_temp[7]-48)*0.1+(Rx_data_temp[8]-48)*0.01+(Rx_data_temp[9]-48)*0.001;    
				    Ki=(Rx_data_temp[10]-48)+(Rx_data_temp[12]-48)*0.1+(Rx_data_temp[13]-48)*0.01+(Rx_data_temp[14]-48)*0.001;
				    Kd=(Rx_data_temp[15]-48)+(Rx_data_temp[17]-48)*0.1+(Rx_data_temp[18]-48)*0.01+(Rx_data_temp[19]-48)*0.001;
				    
            PID_Speed(rate_mm,&htim3);
				break;
				case 112:                               // 'p'=112
						Stop();
            RUN=1;
            position=1;
            speed=0;
            deg_mm=(Rx_data_temp[1]-48)*1000+(Rx_data_temp[2]-48)*100+(Rx_data_temp[3]-48)*10+(Rx_data_temp[4]-48);
				
				    Kp=(Rx_data_temp[5]-48)+(Rx_data_temp[7]-48)*0.1+(Rx_data_temp[8]-48)*0.01+(Rx_data_temp[9]-48)*0.001;    
				    Ki=(Rx_data_temp[10]-48)+(Rx_data_temp[12]-48)*0.1+(Rx_data_temp[13]-48)*0.01+(Rx_data_temp[14]-48)*0.001;
				    Kd=(Rx_data_temp[15]-48)+(Rx_data_temp[17]-48)*0.1+(Rx_data_temp[18]-48)*0.01+(Rx_data_temp[19]-48)*0.001;
				
            PID_Position(deg_mm, &htim3);
				break;
				case 83:                               // 'S'=83
						Stop();
				break;
		
			}
   
     }
     	
    if (display>=10)                         // 5*10=50ms gui du lieu ve PC
    {
      send_data();
      display = 0;
    }       
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
