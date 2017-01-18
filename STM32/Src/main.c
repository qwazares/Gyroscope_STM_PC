/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
L3GD20_Init_Struct Gyro_Init;
L3GD20_out_reg_axes Gyro_data;
Gyro_out_axes  to;

uint8_t data;
char uart_data[8];

uint8_t flaga;
uint8_t conf = 0;
int time = 0;
double freq = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART5_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void MAIN_GYRO_INIT(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI5_Init();
  MX_TIM7_Init();
  MX_UART5_Init();

  /* USER CODE BEGIN 2 */

 // HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  MAIN_GYRO_INIT();								//Gyroscope initialization

  HAL_TIM_Base_Start(&htim7);					//Start Tim7, data ready frequency
  L3GD20_Read_Axes(&Gyro_data, &Gyro_Init);		//First data read

  //HAL_UART_Transmit(&huart5,"AT+UART=38400,0,0,\r\n",22,500);

  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  HAL_UART_Receive_IT(&huart5, &data, 1);

	  if(flaga & (1<<BUTTON_int_flag) && flaga & (1<<gyro_int_flag) &&  ~flaga & (1<<conf_int_flag))
	  {
		  HAL_UART_Receive_IT(&huart5, &data, 1);
		  flaga ^= 1<<gyro_int_flag;
		  //flaga ^= 1<<BUTTON_int_flag;
		  L3GD20_Read_Axes(&Gyro_data, &Gyro_Init);

		  uart_data[0] = (1<<START_BIT) | (Gyro_data.out_x_l & 1<<7)>>(7-BIT_X_L) | (Gyro_data.out_x_h & 1<<7)>>(7-BIT_X_H)
				  | (Gyro_data.out_y_l & 1<<7)>>(7-BIT_Y_L) | (Gyro_data.out_y_h & 1<<7)>>(7-BIT_Y_H)
				  | (Gyro_data.out_z_l & 1<<7)>>(7-BIT_Z_L) | (Gyro_data.out_z_h & 1<<7)>>(7-BIT_Z_H);

		  uart_data[2] = Gyro_data.out_x_l & 0x7F;
		  uart_data[3] = Gyro_data.out_x_h & 0x7F;
		  uart_data[4] = Gyro_data.out_y_l & 0x7F;
		  uart_data[5] = Gyro_data.out_y_h & 0x7F;
		  uart_data[6] = Gyro_data.out_z_l & 0x7F;
		  uart_data[7] = Gyro_data.out_z_h & 0x7F;

		  HAL_UART_Transmit(&huart5,uart_data,8,500);

		  to.x_axis = Gyro_data.out_x_l + (Gyro_data.out_x_h<<8);
		  to.y_axis = Gyro_data.out_y_l + (Gyro_data.out_y_h<<8);
		  to.z_axis = Gyro_data.out_z_l + (Gyro_data.out_z_h<<8);


	  }
	  if(flaga & 1<<conf_int_flag)
	  {
		  flaga ^= 1<<conf_int_flag;

		  if(L3GD20_Init(&Gyro_Init))
		  {
			  uart_data[1] = conf;
		  }
		  else
		  {
			  conf = uart_data[1];
		  }

		  if(~flaga & 1<<BUTTON_int_flag)
		  {
			  uart_data[0] = (1<<START_BIT);
			  uart_data[2] = 0;
			  uart_data[3] = 0;
			  uart_data[4] = 0;
			  uart_data[5] = 0;
			  uart_data[6] = 0;
			  uart_data[7] = 0;
			  HAL_UART_Transmit(&huart5,uart_data,8,500);
		  }
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 19;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0xffff;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PF0   ------> FMC_A0
     PF1   ------> FMC_A1
     PF2   ------> FMC_A2
     PF3   ------> FMC_A3
     PF4   ------> FMC_A4
     PF5   ------> FMC_A5
     PF10   ------> LTDC_DE
     PC0   ------> FMC_SDNWE
     PA3   ------> LTDC_B5
     PA4   ------> LTDC_VSYNC
     PA6   ------> LTDC_G2
     PB0   ------> LTDC_R3
     PB1   ------> LTDC_R6
     PF11   ------> FMC_SDNRAS
     PF12   ------> FMC_A6
     PF13   ------> FMC_A7
     PF14   ------> FMC_A8
     PF15   ------> FMC_A9
     PG0   ------> FMC_A10
     PG1   ------> FMC_A11
     PE7   ------> FMC_D4_DA4
     PE8   ------> FMC_D5_DA5
     PE9   ------> FMC_D6_DA6
     PE10   ------> FMC_D7_DA7
     PE11   ------> FMC_D8_DA8
     PE12   ------> FMC_D9_DA9
     PE13   ------> FMC_D10_DA10
     PE14   ------> FMC_D11_DA11
     PE15   ------> FMC_D12_DA12
     PB10   ------> LTDC_G4
     PB11   ------> LTDC_G5
     PB12   ------> USB_OTG_HS_ID
     PB13   ------> USB_OTG_HS_VBUS
     PB14   ------> USB_OTG_HS_DM
     PB15   ------> USB_OTG_HS_DP
     PD8   ------> FMC_D13_DA13
     PD9   ------> FMC_D14_DA14
     PD10   ------> FMC_D15_DA15
     PD14   ------> FMC_D0_DA0
     PD15   ------> FMC_D1_DA1
     PG4   ------> FMC_A14_BA0
     PG5   ------> FMC_A15_BA1
     PG6   ------> LTDC_R7
     PG7   ------> LTDC_CLK
     PG8   ------> FMC_SDCLK
     PC6   ------> LTDC_HSYNC
     PC7   ------> LTDC_G6
     PC9   ------> I2C3_SDA
     PA8   ------> I2C3_SCL
     PA9   ------> USART1_TX
     PA10   ------> USART1_RX
     PA11   ------> LTDC_R4
     PA12   ------> LTDC_R5
     PC10   ------> LTDC_R2
     PD0   ------> FMC_D2_DA2
     PD1   ------> FMC_D3_DA3
     PD3   ------> LTDC_G7
     PD6   ------> LTDC_B2
     PG10   ------> LTDC_G3
     PG11   ------> LTDC_B3
     PG12   ------> LTDC_B4
     PG15   ------> FMC_SDNCAS
     PB5   ------> FMC_SDCKE1
     PB6   ------> FMC_SDNE1
     PB8   ------> LTDC_B6
     PB9   ------> LTDC_B7
     PE0   ------> FMC_NBL0
     PE1   ------> FMC_NBL1
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin 
                           A4_Pin A5_Pin SDNRAS_Pin A6_Pin 
                           A7_Pin A8_Pin A9_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin 
                          |A4_Pin|A5_Pin|SDNRAS_Pin|A6_Pin 
                          |A7_Pin|A8_Pin|A9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDNWE_Pin */
  GPIO_InitStruct.Pin = SDNWE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(SDNWE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GYRO_CS_Pin */
  GPIO_InitStruct.Pin = GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B5_Pin VSYNC_Pin G2_Pin R4_Pin 
                           R5_Pin */
  GPIO_InitStruct.Pin = B5_Pin|VSYNC_Pin|G2_Pin|R4_Pin 
                          |R5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin R6_Pin */
  GPIO_InitStruct.Pin = R3_Pin|R6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A10_Pin A11_Pin BA0_Pin BA1_Pin 
                           SDCLK_Pin SDNCAS_Pin */
  GPIO_InitStruct.Pin = A10_Pin|A11_Pin|BA0_Pin|BA1_Pin 
                          |SDCLK_Pin|SDNCAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin 
                           D8_Pin D9_Pin D10_Pin D11_Pin 
                           D12_Pin NBL0_Pin NBL1_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin 
                          |D12_Pin|NBL0_Pin|NBL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : G4_Pin G5_Pin B6_Pin B7_Pin */
  GPIO_InitStruct.Pin = G4_Pin|G5_Pin|B6_Pin|B7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D13_Pin D14_Pin D15_Pin D0_Pin 
                           D1_Pin D2_Pin D3_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|D0_Pin 
                          |D1_Pin|D2_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : R7_Pin DOTCLK_Pin B3_Pin */
  GPIO_InitStruct.Pin = R7_Pin|DOTCLK_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : HSYNC_Pin G6_Pin R2_Pin */
  GPIO_InitStruct.Pin = HSYNC_Pin|G6_Pin|R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : G7_Pin B2_Pin */
  GPIO_InitStruct.Pin = G7_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : G3_Pin B4_Pin */
  GPIO_InitStruct.Pin = G3_Pin|B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCKE1_Pin SDNE1_Pin */
  GPIO_InitStruct.Pin = SDCKE1_Pin|SDNE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

void MAIN_GYRO_INIT(void)
{
	Gyro_Init.spi = &hspi5;
	Gyro_Init.CS_Port = GYRO_CS_GPIO_Port;
	Gyro_Init.CS_Pin = GYRO_CS_Pin;
	Gyro_Init.power_mode = L3GD20_MODE_ACTIVE;
	Gyro_Init.axes = L3GD20_AXES_ENABLE;
	Gyro_Init.endian_data = L3GD20_BLE_LSB;
	Gyro_Init.full_scale = L3GD20_FULLSCALE_2000;
	Gyro_Init.output_datarate = L3GD20_OUTPUT_DATARATE_760;
	Gyro_Init.high_pass_filter_status = L3GD20_HIGHPASSFILTER_ENABLE;
	Gyro_Init.int2_status = L3GD20_INT2INTERRUPT_ENABLE;

	if(L3GD20_Init(&Gyro_Init))
		  HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,GPIO_PIN_SET);
	 else
	  {
		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);
		  switch(Gyro_Init.full_scale)
		  {
		  	  case L3GD20_FULLSCALE_250:
		  	  {
		  		  conf |= FULLSCALE_BIT_250;
		  		  break;
		  	  }
		  	  case L3GD20_FULLSCALE_500:
		  	  {
		  		  conf |= FULLSCALE_BIT_500;
		  		  break;
		  	  }
		  	  case L3GD20_FULLSCALE_2000:
		  	  {
		  		  conf |= FULLSCALE_BIT_2000;
		  		  break;
		  	  }
		  }
		  switch(Gyro_Init.output_datarate)
		  {
		  	  case L3GD20_OUTPUT_DATARATE_95:
		  	  {
		  		  conf |= DATARATE_BIT_95;
		  		  break;
		  	  }
		  	  case L3GD20_OUTPUT_DATARATE_190:
		  	  {
		  		  conf |= DATARATE_BIT_190;
		  		  break;
		  	  }
		  	  case L3GD20_OUTPUT_DATARATE_380:
		  	  {
		  		  conf |= DATARATE_BIT_380;
		  		  break;
		  	  }
		  	  case L3GD20_OUTPUT_DATARATE_760:
		  	  {
		  		  conf |= DATARATE_BIT_760;
		  		  break;
		  	  }
		  }

		  uart_data[1] = conf;
	  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{

		flaga ^= 1<<BUTTON_int_flag;
		uart_data[1] ^= 1<<TRANSMITION_BIT;
		conf ^= 1<<TRANSMITION_BIT;

		if(flaga & 1<<BUTTON_int_flag)
			HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,GPIO_PIN_RESET);



	}
	else if(GPIO_Pin == GPIO_PIN_2)
	{
		flaga |= 1<<gyro_int_flag;						//set flag

		time = __HAL_TIM_GET_COUNTER(&htim7) + 1;		//
		__HAL_TIM_SET_COUNTER(&htim7,0);				//calculate freq of data ready

		freq = 45000000/10./time;

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(data & 1<<TRANSMITION_BIT)
	{
		uart_data[1] |= 1<<TRANSMITION_BIT;
		conf |= 1<<TRANSMITION_BIT;
		flaga |= 1<<BUTTON_int_flag;
		HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,GPIO_PIN_SET);
	}
	else
	{
		uart_data[1] &= (0xFF^(1<<TRANSMITION_BIT));
		conf &= (0xFF^(1<<TRANSMITION_BIT));
		flaga &= (0xFF^(1<<BUTTON_int_flag));
		HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,GPIO_PIN_RESET);
	}
	if(data != uart_data[1] || !(data & 1<<TRANSMITION_BIT))
	{
		if(data != uart_data[1])
		{
			uart_data[1] = data;
			switch(data & FULLSCALE_MASK)
			{
				case FULLSCALE_BIT_250:
				{
					flaga |= 1<<conf_int_flag;
					Gyro_Init.full_scale = L3GD20_FULLSCALE_250;
					break;
				}
				case FULLSCALE_BIT_500:
				{
					flaga |= 1<<conf_int_flag;
					Gyro_Init.full_scale = L3GD20_FULLSCALE_500;
					break;
				}
				case FULLSCALE_BIT_2000:
				{
					flaga |= 1<<conf_int_flag;
					Gyro_Init.full_scale = L3GD20_FULLSCALE_2000;
					break;
				}
			}
			switch(data & DATARATE_MASK)
			{
				case DATARATE_BIT_95:
				{
					//flaga |= 1<<conf_int_flag;
					Gyro_Init.output_datarate = L3GD20_OUTPUT_DATARATE_95;
					break;
				}
				case DATARATE_BIT_190:
				{
					//flaga |= 1<<conf_int_flag;
					Gyro_Init.output_datarate = L3GD20_OUTPUT_DATARATE_190;
					break;
				}
				case DATARATE_BIT_380:
				{
					//flaga |= 1<<conf_int_flag;
					Gyro_Init.output_datarate = L3GD20_OUTPUT_DATARATE_380;
					break;
				}
				case DATARATE_BIT_760:
				{
					//flaga |= 1<<conf_int_flag;
					Gyro_Init.output_datarate = L3GD20_OUTPUT_DATARATE_760;
					break;
				}
			}
		}
		else
			flaga |= 1<<conf_int_flag;
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
