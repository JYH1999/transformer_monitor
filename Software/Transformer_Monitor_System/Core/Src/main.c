/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "lwip.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "sht31.h"
#include "W25Qxx.h"
#include "ethernetif.h"
#include "ade7878.h"
#include "string.h"
#include "udp_client.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#pragma  diag_suppress 870 //消除℃符号警告问问题
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t sysclock = 0;//系统时钟变量

extern float sht31_tmp;//sht31温度变量
extern float sht31_hum;//sht31湿度变量

uint8_t testValue __attribute__((at(EXT_SRAM_ADDR)));//SRAM测试变量，地址定位于SRAM

uint8_t flash_id[]={0xaa,0xaa};//Flash ID buffer
uint8_t flash_test_data[]={0x10,0x11};//Flash测试用数据
uint8_t flash_test_buffer[2];//Flash read buffer

unsigned char chip_checksum[4]={5};//ADE7878校验变量
extern unsigned char CONFIG2dat[1],MMODEdat[1],GAINdat[2];
extern unsigned char AIRMSReg[DataBufferSize*4],IAWVReg[4],VAWVReg[4],
              AVRMSReg[DataBufferSize*4],Status_1[4],Status_0[4],VPEAKReg[DataBufferSize*4],IPEAKReg[DataBufferSize*4],
			        AWATTReg[4],AWATTHRReg[4],MASK0_data[4];  

extern unsigned char page0_logo[10];
extern unsigned char current_page;

extern unsigned char wwan_receive_buffer[200];
extern unsigned char wwan_receive_temp[1];
extern unsigned char wwan_receive_count;

extern uint8_t wwan_ip[4];
extern unsigned int wwan_port;
extern unsigned char wwan_flash_ip[4];
extern unsigned char wwan_flash_port[2];

extern uint8_t server_ip[4];
extern unsigned char start_connection[38];//建立UDP连接
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_FSMC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  //MX_IWDG_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  sht31_init();
  BSP_W25Qx_Init();
	
	//自检
	printf("配电变压器智能监控系统 \nAuthor:金煜航 \n");
	printf("Software version:%.1f\n",SOFTWARE_VERSION);
	printf("Build Time:%02d-%02d-%02d %02d:%02d:%02d CST\n",MDK_YEAR,MDK_MONTH,MDK_DAY,MDK_HOUR,MDK_MIN,MDK_SEC);
	sysclock = HAL_RCC_GetSysClockFreq();
	printf("System clock@%dHz\n",sysclock);
	printf("Start self-test, log encode in UTF-8:\n");
	//复位串口屏
	printf("Reset display content...");
	HAL_UART_Transmit(&huart6,page0_logo,sizeof(page0_logo),100); //显示LOGO界面
	current_page=0;
	printf("Done!\n");
	//复位LTE模组
	printf("Reset WWAN module...");
	HAL_Delay(500);
	HAL_GPIO_WritePin(RESET_4G_GPIO_Port, RESET_4G_Pin, GPIO_PIN_SET);//复位引脚置高
	HAL_Delay(1500);//延时1.5s 使模组复位
	HAL_GPIO_WritePin(RESET_4G_GPIO_Port, RESET_4G_Pin, GPIO_PIN_RESET);
	printf("Done!\n");
	//串口测试
  printf("USART1 Output Test:\n");
  printf("Hello World!\n");
	//蜂鸣器测试
	printf("BEEP Test:\n");
	printf("BEEP ON\n");
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
	printf("BEEP OFF\n");
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
	//SD卡插入监测
	if (HAL_GPIO_ReadPin(TF_IN_GPIO_Port,TF_IN_Pin) == 0)
		printf("Micro SD card detected!\n");
	else
		printf("Micro SD card missing!\n");
	//LED测试
	printf("LED Test:\n");
	printf("LED ON\n");
	HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin|LED2_GREEN_Pin|LED1_RED_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
	printf("LED OFF\n");
  HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin|LED2_GREEN_Pin|LED1_RED_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	//SRAM测试
	printf("Start FSMC SRAM Test:\n");
	if (bsp_TestExtSRAM() == 0)
	{
    printf("SRAM Test success!\n");
	}
	else 
	{
    printf("SRAM Test fail!\n");
		Error_Handler();
  }
  printf("Start SRAM value test, testValue=0x5a:\n");
  testValue = 0x5a;
  printf("testValue from SRAM is %#x\n", testValue);
	//温湿度传感器测试
	printf("Start SHT31 sensor test:\n");
	sht31_sample(&sht31_tmp, &sht31_hum);
  printf("Temp=%.2f℃ \n", sht31_tmp);
  printf("Humid=%.2f%%\n", sht31_hum);
	HAL_Delay(50);
	//SPI Flash测试
	printf("SPI flash test:\n");
	printf("Read SPI flash device ID:\n");
  BSP_W25Qx_Read_ID(flash_id);
  printf("Device ID: %#x %#x\n",flash_id[0],flash_id[1]);
	printf("Start SPI flash write&read test, write data: 0x10 0x11\n");
  BSP_W25Qx_Write(flash_test_data, W25Q128FV_FLASH_SIZE-1000, sizeof(flash_test_data));
	HAL_Delay(50);
  BSP_W25Qx_Read(flash_test_buffer, W25Q128FV_FLASH_SIZE-1000, sizeof(flash_test_data));
  printf("Read data: %#x %#x\n",flash_test_buffer[0],flash_test_buffer[1]);
	HAL_Delay(50);
	//SPI Flash读取并设置服务器IP信息
	printf("Read IP data from SPI flash:\n");
	BSP_W25Qx_Read(wwan_flash_ip, W25Q128FV_FLASH_SIZE-6000, sizeof(wwan_flash_ip));
	HAL_Delay(50);
	BSP_W25Qx_Read(wwan_flash_port, W25Q128FV_FLASH_SIZE-4000, sizeof(wwan_flash_port));
	HAL_Delay(50);
	wwan_port=wwan_flash_port[1];
	wwan_port=wwan_port<<8;
	wwan_port=wwan_flash_port[0]+wwan_port;
	printf("Setting IP:%d.%d.%d.%d Port:%d...",wwan_flash_ip[0],wwan_flash_ip[1],wwan_flash_ip[2],wwan_flash_ip[3],wwan_port);
	sprintf((char*)start_connection,"AT+CIPSTART=\"UDP\",\"%d.%d.%d.%d\",%d\r",wwan_flash_ip[0],wwan_flash_ip[1],wwan_flash_ip[2],wwan_flash_ip[3],wwan_port);
	server_ip[0]=wwan_flash_ip[0];
	server_ip[1]=wwan_flash_ip[1];
	server_ip[2]=wwan_flash_ip[2];
	server_ip[3]=wwan_flash_ip[3];
	server_port=wwan_port;
	memset(wwan_ip,0,sizeof(wwan_ip));
	memset(wwan_flash_ip,0,sizeof(wwan_flash_ip));
	memset(wwan_flash_port,0,sizeof(wwan_flash_port));
	wwan_port=0;
	printf("Done!\n");
	//ADE7878测试
	printf("Initialize ADE7878, read CHECKSUM register\n");
  ADE7878_init();
  HAL_Delay(5);
  Read_ADE7878_SPI(CONFIG2,1,CONFIG2dat);
  Read_ADE7878_SPI(MMODE,1,MMODEdat);
  Read_ADE7878_SPI(GAIN,2,GAINdat);
  Read_ADE7878_SPI(STATUS1,4,Status_1);
  Read_ADE7878_SPI(MASK0,4,MASK0_data);
  Read_ADE7878_SPI(STATUS1,4,Status_0);
	printf("Read modified ADE7878 CHECKSUM register:\n");
	Read_ADE7878_SPI(CHECKSUM, 4,chip_checksum);
	printf("ADE7878 CHECKSUM register[1]: %#x\n",chip_checksum[0]);
	printf("ADE7878 CHECKSUM register[2]: %#x\n",chip_checksum[1]);
	printf("ADE7878 CHECKSUM register[3]: %#x\n",chip_checksum[2]);
	printf("ADE7878 CHECKSUM register[4]: %#x\n",chip_checksum[3]);
	printf("Start FreeRTOS with LWIP&FATFS, keys detecting...\n");
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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

/* USER CODE BEGIN 4 */

int fputc(int c,FILE *stream)//enable printf functions
{
	HAL_UART_Transmit(&huart1,(unsigned char *)&c,1,100); 
	return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		wwan_receive_buffer[wwan_receive_count++] = wwan_receive_temp[0];
	  if(wwan_receive_count<=49)
	    HAL_UART_Receive_IT(&huart3, wwan_receive_temp, 1);
	}
  
}

uint8_t Key_Scan(GPIO_TypeDef *GPIOx,uint16_t GPIO_PIN)
{
    if (HAL_GPIO_ReadPin(GPIOx,GPIO_PIN) == 0)
    {
		osDelay(10);//按键消抖10ms
        while (HAL_GPIO_ReadPin(GPIOx,GPIO_PIN) == 0);
		osDelay(10);//按键消抖10ms
        return 1;
    }
    else
    {
        return 0;
    }   
}

uint32_t bsp_TestExtSRAM(void)
{
	uint32_t i;
	uint32_t *pSRAM;
	uint8_t *pBytes;
	uint32_t err;
	const uint8_t ByteBuf[4] = {0x55, 0xA5, 0x5A, 0xAA};
	
	/* 写SRAM */
	pSRAM = (uint32_t *)EXT_SRAM_ADDR;
	for (i = 0; i < EXT_SRAM_SIZE / 4; i++)
	{
		*pSRAM++ = i;
	}
	
	/* 读SRAM */
	err = 0;
	pSRAM = (uint32_t *)EXT_SRAM_ADDR;
	for (i = 0; i < EXT_SRAM_SIZE / 4; i++)
	{
		if (*pSRAM++ != i)
		{
			err++;
		}
	}
	printf("SRAM check round 1 error = %d\n", err);
	if (err > 0)
	{
		return (4 * err);
	}

	#if 1
	/* 对SRAM数据求反并写入 */
	pSRAM = (uint32_t *)EXT_SRAM_ADDR;
	for (i = 0; i < EXT_SRAM_SIZE/4; i++)
	{
		*pSRAM = ~*pSRAM;
		pSRAM++;
	}

	/* 再次比较SRAM的数据 */
	err = 0;
	pSRAM = (uint32_t *)EXT_SRAM_ADDR;
	for (i = 0; i<EXT_SRAM_SIZE/4;i++)
	{
		if (*pSRAM++ != (~i))
		{
			err++;
		}
	}

	printf("SRAM check round 2 error = %d\n", err);
	if (err>0)
	{
	return (4 * err);
	}
	#endif

	/* 测试按字节方式访问，目的是测试FSMC_NBL0 FSMC_NBL1 口线 */
	pBytes = (uint8_t *)EXT_SRAM_ADDR;
	for (i = 0; i < sizeof(ByteBuf); i++)
	{
		*pBytes++ = ByteBuf[i];
	}

	/* 比较SRAM的数据 */
	err = 0;
	pBytes = (uint8_t *)EXT_SRAM_ADDR;
	for (i = 0; i < sizeof(ByteBuf); i++)
	{
		if (*pBytes++ != ByteBuf[i])
		{
			err++;
		}
	}
	printf("SRAM check round 3 error = %d\n", err);
	if (err > 0)
	{
		return err;
	}
	return 0;
}

//Keil编译月份获取
uint8_t get_month(void)
{
  uint8_t month_val;
  switch(__DATE__[2])
  {
    case 'n':
	  if(__DATE__[1] == 'a')
        month_val = 1;
      else
        month_val = 6;
    break;
    case 'b':
      month_val = 2;
    break;
    case 'r':
      if(__DATE__[1] == 'a')
        month_val = 3;
      else
        month_val = 4;
    break;
    case 'y':
      month_val = 5;
    break;
    case 'l':
      month_val = 7;
    break;
    case 'g':
      month_val = 8;
    break;
    case 'p':
      month_val = 9;
    break;
    case 't':
      month_val = 10;
    break;
    case 'v':
      month_val = 11;
    break; 
    case 'c':
      month_val = 12;
    break; 
    default:
      month_val = 0;
      break;
  }
  return month_val;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
	HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin|LED2_GREEN_Pin|LED1_RED_Pin, GPIO_PIN_RESET);
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOA,LED1_RED_Pin);
	  HAL_UART_Transmit(&huart1,(unsigned char *)"System in ERROR!\n",17,100); 
      for(uint16_t time=500; time>0; time--)
      {
         uint16_t i=12000;
         while(i--) ;    
      }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
