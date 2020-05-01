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
unsigned short rxfreq[]={//439.725Mhz
0x52BE,0xF850,0x0000};
unsigned short txfreq[]={
0x52C5,0x925F,0x0000};
unsigned short rxreg[]={ 
0x0300,
0x8e04,
0xF140,
0xED00,
0x17E0,
0xe0e0,
0x8543,
0x0700,
0xA066,
0xFFFF,
0xFFE0,
0x07a0,
0x9E3C,
0x1F00,
0xD1c1,
0x200F,
0x01FF,
0xE000,
0x0339
 };
unsigned short txreg[]={
0x7C00,
0x0c04,
0xF140,
0xED00,
0x3fE0,
0xe0e0,
0x8543,
0x0700,
0xA066,
0xFFFF,
0xffe0,
0x061f,
0x9e3c,
0x1f00,
0xd1C1,
0x200f,
0x01FF,
0xE000,
0x0c00
};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void delay1(unsigned short int ms)
{
	unsigned short int i;
	i=100;
	while(ms--)
		while(i--);
}
/* USER CODE END PTD */
unsigned short s;
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
unsigned char HIGHBYTE(unsigned short bi)//Get higer 8 bit
	 {
		 unsigned char highER ;
		 highER=bi>>8;
		 return highER;
	 }
unsigned char 	LOWBYTE(unsigned short bi)//Get lower 8 bit
	 {
		 unsigned char lowER;
		 lowER=bi&0xff;
		 return lowER;
	 }

/* USER CODE END PD */
void IIC_Start()
{
  HAL_GPIO_WritePin(GPIOA,CLK_Pin, 1);// SCL = high;		
  HAL_GPIO_WritePin(GPIOA, DATA_Pin, 1);// SDA = high;
  HAL_GPIO_WritePin(GPIOA, DATA_Pin, 0); //SDA = low;
  HAL_GPIO_WritePin(GPIOA, CLK_Pin, 0); //SCL = low;
}
void IIC_Stop()
{
  HAL_GPIO_WritePin(GPIOA, CLK_Pin, 0); // SCL = low;
  HAL_GPIO_WritePin(GPIOA, DATA_Pin, 0); // SDA = low;
  HAL_GPIO_WritePin(GPIOA,CLK_Pin, 1);// SCL = high;
  HAL_GPIO_WritePin(GPIOA, DATA_Pin, 1);//  SDA = high;
}
void Write_IIC_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		if(IIC_Byte & 0x80)
			HAL_GPIO_WritePin(GPIOA, DATA_Pin, 1);//SDA=high;
		else
			HAL_GPIO_WritePin(GPIOA, DATA_Pin, 0); //SDA=low;
		HAL_GPIO_WritePin(GPIOA,CLK_Pin, 1);// SCL=high;
		HAL_GPIO_WritePin(GPIOA, CLK_Pin, 0); //SCL=low;
		IIC_Byte<<=1;
	}
	HAL_GPIO_WritePin(GPIOA, DATA_Pin, 1);// SDA=1;
	HAL_GPIO_WritePin(GPIOA,CLK_Pin, 1);//SCL=1;
	HAL_GPIO_WritePin(GPIOA, CLK_Pin, 0); //SCL=0;
}
void writing(unsigned char ICaddforwrite,unsigned char hadd,unsigned char hdata,unsigned char ldata)//WRITE TO  BK4802N
	 {

    IIC_Start();	 //??????
    Write_IIC_Byte(ICaddforwrite);                          //??????+???
    Write_IIC_Byte(hadd); 
	  Write_IIC_Byte(hdata); 
		Write_IIC_Byte(ldata); 
		IIC_Stop();
		delay1(1);
	 }
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
	unsigned char i;
	unsigned char k=0;
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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, LED_Pin, 1);//LED falshing test
	delay1(100);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, 0);
	delay1(100);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, 1);
	delay1(100);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, 0);
	delay1(100);
  /* USER CODE END 2 */
  writing(0x90,23,0xa8,0XD0);
	for(i=4;i<=22;i++)
	    {
				writing(0x90,i,HIGHBYTE(rxreg[i-4]),LOWBYTE(rxreg[i-4]));
			}
			
		 for(i=5;i>2;i--)
		 {
			 writing(0x90,i-3,HIGHBYTE(rxfreq[i-3]),LOWBYTE(rxfreq[i-3]));
		 }
		 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  HAL_GPIO_WritePin(GPIOA, LED_Pin, 1);
	if( HAL_GPIO_ReadPin(GPIOB,PTT_Pin)==1&&k==0)//Set to TX if PTT =HIGH.
			{
				
				delay1(50);
				if(HAL_GPIO_ReadPin(GPIOB,PTT_Pin)==1)
				{
					
					  for(i=4;i<=22;i++)
	       {
				   writing(0x90,i,HIGHBYTE(txreg[i-4]),LOWBYTE(txreg[i-4]));
			    }
			
		      for(i=5;i>2;i--)
		      {
			     writing(0x90,i-3,HIGHBYTE(txfreq[i-3]),LOWBYTE(txfreq[i-3]));
		      }
					delay1(50);
				
					k=1;
				}
			}
				if(HAL_GPIO_ReadPin(GPIOB,PTT_Pin)==0&&k==1)//Set to RX.
				{
					
					delay1(50);
				if(HAL_GPIO_ReadPin(GPIOB,PTT_Pin)==0)
				{
					
					writing(0x90,23,0xa8,0XD0);
					  for(i=4;i<=22;i++)
	       {
				   writing(0x90,i,HIGHBYTE(rxreg[i-4]),LOWBYTE(rxreg[i-4]));
			    }
			
		      for(i=5;i>2;i--)
		      {
			     writing(0x90,i-3,HIGHBYTE(rxfreq[i-3]),LOWBYTE(rxfreq[i-3]));
		      }
					delay1(50);
					
					k=0;
			  }
		   }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|DATA_Pin|CLK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_Pin DATA_Pin CLK_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DATA_Pin|CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PTT_Pin */
  GPIO_InitStruct.Pin = PTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PTT_GPIO_Port, &GPIO_InitStruct);

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
