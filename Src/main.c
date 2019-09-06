/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "MLX90640_API.h"
#include "network.h"
#include "w5500.h"
#include "socket.h"

#include "stm32f4xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  Rate2HZ   0x02
#define  Rate4HZ   0x03
#define  Rate8HZ   0x04
#define  Rate16HZ  0x05
#define  Rate32HZ  0x06

#define  MLX_I2C_ADDR 0x33
#define	 RefreshRate Rate4HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air
#define USART1_RXBUF_SIZE 40
#define		GET_PREV_INDEX(index)	((index <= 0)? USART1_RXBUF_SIZE-1: index-1)
#define		GET_NEXT_INDEX(index)	((index >= USART1_RXBUF_SIZE-1)? 0: index+1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static uint16_t USART1_RxIndex = 0;
static FlagStatus USART1_RXFLAG = RESET;
uint8_t read_buffer[40];
extern DMA_HandleTypeDef hdma_usart1_rx;
struct ConfigPack config_pack;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t USART1_Read(uint8_t * const buf, uint32_t length);
void writeConfifPackToFlash();
void readConfigPackFromFlash();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct UploadNetwork
{
	unsigned long id;
	float target_temperature;
	float self_temperature;

};


struct UploadUart
{
	unsigned short start;
	unsigned short target_temperature[768];
	unsigned short self_temperature;
};

struct ConfigPack
{
    unsigned char id;
    unsigned char sip[4];
    unsigned char dip[4];
    unsigned short s_port;
    unsigned short d_port;
    unsigned char row_start;
    unsigned char row_end;
    unsigned char column_start;
    unsigned char column_end;
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static uint16_t eeMLX90640[832];
	uint16_t frame[834];
	float Ta,tr;
	float emissivity=0.95;
	static float mlx90640To[768];
	u16 i=0;
	struct UploadUart upload_uart;
	upload_uart.start = 0x697e; //~!
	struct UploadNetwork upload_network;
	upload_network.id = DEVICE_ID;


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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_DMA(&huart1, read_buffer, 40);
	readConfigPackFromFlash();
	network_init();


	Delay_Init(84);
	MLX90640_SetRefreshRate(MLX_I2C_ADDR, RefreshRate);
	MLX90640_SetChessMode(MLX_I2C_ADDR);
	paramsMLX90640 mlx90640;
	MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
	MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

	for(i=0;i<3;i++)//Lose the start frame
	{
			MLX90640_GetFrameData(MLX_I2C_ADDR, frame);
			delay_ms(500);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MLX90640_GetFrameData(MLX_I2C_ADDR, frame);
		Ta = MLX90640_GetTa(frame, &mlx90640);
		tr = Ta - TA_SHIFT;
		MLX90640_CalculateTo(frame, &mlx90640, emissivity, tr, mlx90640To);

		for(i=0;i<768;i++)//hundredfold,first send low 8bit and then high 8bit,DMA circular transfer
		{
			upload_uart.target_temperature[i] = (unsigned short)(mlx90640To[i]*100);
		}

		upload_uart.self_temperature = (unsigned short)(Ta*100);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&upload_uart,  sizeof(struct UploadUart));




		upload_network.target_temperature = 0;
		for(int row=6; row<18;row++)
		{
			for(int column=12; column<20;column++)
			{
				upload_network.target_temperature += mlx90640To[32*row+column];
			}
		}
		upload_network.target_temperature = upload_network.target_temperature / 96;

		upload_network.self_temperature = Ta;


		sendto(0, (uint8_t *)&upload_network, sizeof(struct UploadNetwork), config_pack.dip, config_pack.d_port);

		int read_size = USART1_Read((uint8_t *)&config_pack, sizeof(struct ConfigPack));
		if(read_size > 0)
		{
			writeConfifPackToFlash();
		}


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void UART1_IDLEHandler(void)
{
  uint32_t tmp_flag = 0, tmp_it_source = 0;

  tmp_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE);

  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		USART1_RXFLAG = SET;
  }
}

uint32_t USART1_Read(uint8_t * const buf, uint32_t length)
{
	uint16_t size = 0;
	uint16_t index  = 0;

	if(USART1_RXFLAG != RESET)
	{
		USART1_RXFLAG = RESET;
		index = USART1_RXBUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

		while((USART1_RxIndex != index) && (size < length))
		{
			buf[size++] = read_buffer[USART1_RxIndex];
			USART1_RxIndex = GET_NEXT_INDEX(USART1_RxIndex);
		}
		return size;

	}
	return 0;

}

void writeConfifPackToFlash(void)
{
	uint32_t startAddress = 0x08020000;
	uint32_t SectorError;

	static FLASH_EraseInitTypeDef EraseInitStruct;
	HAL_FLASH_Unlock();    //解锁,删除和写入必须先解锁,规定的
	EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
	EraseInitStruct .VoltageRange =VOLTAGE_RANGE_3 ;
	EraseInitStruct.Sector = FLASH_SECTOR_5;
	EraseInitStruct.NbSectors = 1;
	//以上,定义删除类型是SECTORS就是块,提供的电压是3.3v的所以选VOLTAGE_RANGE_3
	HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	//清除函数,如果出错则错误码存储到SectorError这个里面

	//设置了准备写入数据的起始地址和结束地址
	uint8_t *ptr = (uint8_t *)&config_pack;
	for(int i=0; i<sizeof(struct ConfigPack);i++)
    {
		if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, startAddress, ptr[i]) == HAL_OK)
		{
			startAddress += 1;
		}
    }
	HAL_FLASH_Lock();
}

void readConfigPackFromFlash()
{
	uint32_t startAddress = 0x08020000;
	uint8_t *ptr = (uint8_t *)&config_pack;
	for(int i=0; i<sizeof(struct ConfigPack);i++)
    {
		ptr[i] = *(__IO uint8_t*)(startAddress + i);
    }

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
