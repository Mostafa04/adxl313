# adxl313
adxl313 interface with stm32f401re
#include "main.h"
#include "stm32f4xx_hal.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

#define adxl_address_R  0XA7
#define adxl_address_W	0XA6

#define I2Cx_SCL_PIN                    GPIO_PIN_8		//PB8
#define I2Cx_SCL_GPIO_PORT              GPIOB			//PB8
#define I2Cx_SCL_AF                     GPIO_AF4_I2C1	//PB8 SCL

#define I2Cx_SDA_PIN                    GPIO_PIN_9		//PB9
#define I2Cx_SDA_GPIO_PORT              GPIOB			//PB9
#define I2Cx_SDA_AF                     GPIO_AF4_I2C1	//PB9 SDA


#define USARTx_TX_PIN                    GPIO_PIN_2    	 //PA2 Trasnmit
#define USARTx_TX_GPIO_PORT              GPIOA			 //PA2
#define USARTx_TX_AF                     GPIO_AF7_USART2 //PA2

#define USARTx_RX_PIN                    GPIO_PIN_3		  //PA3 Recieve
#define USARTx_RX_GPIO_PORT              GPIOA			  //PA3
#define USARTx_RX_AF                     GPIO_AF7_USART2  //PA3

//uint8_t bufftx1[8]= "108 \n\r";
int16_t data_rec[6];
uint8_t chipid=0;
int16_t x,y,z;
uint8_t data_final[3];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_write (uint8_t reg, uint8_t value)
{
	uint8_t data[1];
	data[0] = value;
	HAL_I2C_Mem_Write(&hi2c1, adxl_address_W, reg, 1, (uint8_t *)data, 1, 100);
}

void adxl_read_values (uint8_t reg, uint8_t numbofbytes)
{
	HAL_I2C_Mem_Read (&hi2c1, adxl_address_R, reg, 1, (uint8_t *)data_rec, numbofbytes, 100);
}

void adxl_read_address (uint8_t reg)
{
	HAL_I2C_Mem_Read (&hi2c1, adxl_address_R, reg, 1, &chipid, 1, 100);
}

void adxl_init (void)
{
	adxl_write (0x18, 0x52);  //  soft reset

	adxl_read_address (0x02); // PARTID
	adxl_read_address (0x00); // read the DEVID 0
	adxl_read_address (0x01); // read the DEVID 1

	adxl_write (0x2c, 0x0d);  //  800 HZ
	adxl_write (0x31, 0x03);  //  range (+-2)   0000 0011
	adxl_write (0x2d, 0x08);  //  measure bit 1, wakeup 0, 0 at 8HZ; 0000 1000

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  adxl_init();

  while (1)
  {
    /* USER CODE END WHILE */

	//HAL_UART_Transmit(&huart2, bufftx1, 8, 100);
	//HAL_Delay(1000);

	adxl_read_values (0x32, 6);

	x = ((data_rec[1]<<8)|data_rec[0]);
	y = ((data_rec[3]<<8)|data_rec[2]);
	z = ((data_rec[5]<<8)|data_rec[4]);

	data_final[0]=x;
	data_final[1]=y;
	data_final[2]=z;

	//printf("%d %d %d \r\n", x, y, z);
	HAL_UART_Transmit(&huart2, data_final, 3, 100);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSI Oscillator and activate PLL with HSI as source  -> 84Mhz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK (84Mhz), PCLK1 (42Mhz) and PCLK2 (84 Mhz)
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;


  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;					// PB8 SCL
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;				// PB8
  GPIO_InitStruct.Pull      = GPIO_PULLUP;					// PB8
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;				// PB8
  GPIO_InitStruct.Alternate = I2Cx_SCL_AF;					// PB8 SCL

  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin 		= I2Cx_SDA_PIN;					// PB9 SDA
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;				// PB9
  GPIO_InitStruct.Pull      = GPIO_PULLUP;					// PB9
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;				// PB9
  GPIO_InitStruct.Alternate = I2Cx_SDA_AF;					// PB9 SDA

  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);


  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

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
