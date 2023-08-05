/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "arm_math.h"
#include "sht31.h"
#include "W25Qxx.h"
#include "ethernetif.h"
#include "ade7878.h"
#include "sdio.h"
#include "fatfs.h"
#include "sd_log.h"
#include "usart.h"
#include "string.h"
#include "iwdg.h"
#include "rtc.h"
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
#pragma  diag_suppress 870 //消除中文编译警告问题
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//系统变量
extern uint32_t sysclock;//系统时钟变量

//LAN8720A 以太网变量
extern ETH_HandleTypeDef heth;
uint32_t lan_BSR;//以太网BSRR寄存器变量

//SHT31 变量
float sht31_tmp=0.0f;//sht31温度变量
float sht31_hum=0.0f;//sht31湿度变量

//串口屏相关变量
unsigned char page0_logo[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x00};//LOGO页命令
unsigned char page1_vainfo[]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x01};//电压电流页命令
unsigned char page2_wattinfo[]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x02};//功率页命令
unsigned char lte_on[]={0x5A,0xA5,0x05,0x82,0x20,0x00,0x00,0x15};//LTE状态显示为开启
unsigned char lte_off[]={0x5A,0xA5,0x05,0x82,0x20,0x00,0x00,0x13};//LTE状态显示为关闭
unsigned char eth_on[]={0x5A,0xA5,0x05,0x82,0x21,0x00,0x00,0x15};//以太网状态显示为开启
unsigned char eth_off[]={0x5A,0xA5,0x05,0x82,0x21,0x00,0x00,0x13};//以太网状态显示为关闭
unsigned char server_on[]={0x5A,0xA5,0x05,0x82,0x22,0x00,0x00,0x15};//服务器连接状态显示为开启
unsigned char server_off[]={0x5A,0xA5,0x05,0x82,0x22,0x00,0x00,0x13};//服务器连接状态显示为关闭
unsigned char writereg[8]={0x5A,0xA5,0x05,0x82,0x81,0x00,0x00,0xEF};//串口屏寄存器写入模板 示例向地址0x8100写入0x00EF
unsigned char current_page=1;//当前页面
unsigned char lte_status=0;//LTE状态
unsigned char eth_status=0;//ETH状态
unsigned char server_status=0;//SERVER状态

//LTE模组相关变量(YED-C724)
unsigned char wwan_receive_buffer[200];
unsigned char wwan_receive_temp[1];
unsigned char wwan_receive_count=0;
unsigned char at_command[]="AT\r";//AT指令 同时用于模组匹配波特率
unsigned char manufacturer_info[]="AT+CGMI\r";//模块厂商信息
unsigned char firmware_version[]="AT+CGMR\r";//模块固件版本信息
unsigned char sim_status[]="AT+CPIN?\r";//查询卡是否插好
unsigned char signal_quality[]="AT+CSQ\r";//查询设置信号质量
unsigned char network_regstate[]="AT+CREG?\r";//查询网络注册状态
unsigned char gprs_attachstate[]="AT+CGATT?\r";//查询附着GPRS网络
unsigned char transparent_mode[]="AT+CIPMODE=1\r";//设置透传模式
unsigned char single_connection[]="AT+CIPMUX=0\r";//设置IP为单连接
unsigned char cmcc_apn[]="AT+CSTT=\"CMNET\",\"\",\"\" \r";//设置中国移动APN
unsigned char activate_mobile[]="AT+CIICR\r";//激活移动场景，获取IP地址
unsigned char get_ip[]="AT+CIFSR\r";//查询IP地址
unsigned char save_param[]="AT+CIPSCONT\r";//保存TCPIP参数
unsigned char start_connection[38]="AT+CIPSTART=\"UDP\",\"8.136.87.152\",4950\r";//建立UDP连接
unsigned char stop_transparent[]="+++";//关闭透传模式
unsigned char close_connection[]="AT+CIPCLOSE=0\r";//关闭连接
unsigned char time_sync[]="AT+CNTP\r";//同步网络时间
unsigned char get_time[]="AT+CCLK?\r";//获取时间
uint8_t wwan_ip[4]={0};
unsigned int wwan_port=0;
//SPI Flash相关变量
unsigned char wwan_flash_ip[4]={0};
unsigned char wwan_flash_port[2]={0};
unsigned char flash_writeflag=0;
//RTC相关变量
unsigned char yy=22;
unsigned char MM=1;
unsigned char dd=1;
unsigned char hh=0;
unsigned char mm=0;
unsigned char ss=0;
RTC_TimeTypeDef time_set = {0};
RTC_DateTypeDef date_set = {0};
RTC_TimeTypeDef time_get = {0};
RTC_DateTypeDef date_get = {0};
unsigned char rtc_ready=0;

//ADE7878 变量
unsigned char CONFIG2dat[1]={0x00},MMODEdat[1]={0x04},GAINdat[2];
unsigned char AIRMSReg[DataBufferSize*4]={0},//A相
							IAWVReg[4],
							VAWVReg[4],
              AVRMSReg[DataBufferSize*4]={0},
							AWATTReg[4]={0},
							AWATTHRReg[4],
							AVARReg[4]={0},
							AVARHRReg[4],
							BIRMSReg[DataBufferSize*4]={0},//B相
							IBWVReg[4],
							VBWVReg[4],
              BVRMSReg[DataBufferSize*4]={0},
							BWATTReg[4]={0},
							BWATTHRReg[4],
							BVARReg[4]={0},
							BVARHRReg[4],
							CIRMSReg[DataBufferSize*4]={0},//C相
							ICWVReg[4],
							VCWVReg[4],
              CVRMSReg[DataBufferSize*4]={0},
							CWATTReg[4]={0},
							CWATTHRReg[4],
							CVARReg[4]={0},
							CVARHRReg[4],
							NIRMSReg[DataBufferSize*4]={0},//中性线N
							INWVReg[4],
							Status_1[4],
							Status_0[4],
							VPEAKReg[DataBufferSize*4],
							IPEAKReg[DataBufferSize*4],
							MASK0_data[4];
s32 iwave_a=0,vwave_a=0;
s32 iwave_b=0,vwave_b=0;
s32 iwave_c=0,vwave_c=0;
s32 iwave_n=0;
float irms_a=0,vrms_a=0,i_RMS_a=0,i_WAVE_a=0,v_RMS_a=0,v_WAVE_a=0,awatt=0,A_WATT=0,awatthr=0,A_WATTHR=0,avar=0,A_VAR=0,avarhr=0,A_VARHR=0;
float irms_b=0,vrms_b=0,i_RMS_b=0,i_WAVE_b=0,v_RMS_b=0,v_WAVE_b=0,bwatt=0,B_WATT=0,bwatthr=0,B_WATTHR=0,bvar=0,B_VAR=0,bvarhr=0,B_VARHR=0;
float irms_c=0,vrms_c=0,i_RMS_c=0,i_WAVE_c=0,v_RMS_c=0,v_WAVE_c=0,cwatt=0,C_WATT=0,cwatthr=0,C_WATTHR=0,cvar=0,C_VAR=0,cvarhr=0,C_VARHR=0;
float irms_n=0,i_RMS_n=0,i_WAVE_n=0;
float A_WATTHR_Acc=0,B_WATTHR_Acc=0,C_WATTHR_Acc=0,A_VARHR_Acc=0,B_VARHR_Acc=0,C_VARHR_Acc=0;//功率累计值变量，用于ADE7878 xWATTHR寄存器溢出时数据存储
							
/* USER CODE END Variables */
osThreadId MAIN_TaskHandle;
osThreadId LED_TaskHandle;
osThreadId Flash_TaskHandle;
osThreadId BEEP_TaskHandle;
osThreadId ADE7878_TaskHandle;
osThreadId SCREEN_TaskHandle;
osThreadId WWAN_TaskHandle;
osThreadId SHT31_TaskHandle;
osThreadId ETH_TaskHandle;
osThreadId KEY_TaskHandle;
osThreadId SD_TaskHandle;
osThreadId DATA_TaskHandle;
osThreadId IWDG_TaskHandle;
osMutexId ADE7878_MutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void main_task(void const * argument);
void led_task(void const * argument);
void flash_task(void const * argument);
void beep_task(void const * argument);
void ade7878_task(void const * argument);
void screen_task(void const * argument);
void wwan_task(void const * argument);
void sht31_task(void const * argument);
void eth_task(void const * argument);
void key_task(void const * argument);
void sd_task(void const * argument);
void data_task(void const * argument);
void iwdg_task(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of ADE7878_Mutex */
  osMutexDef(ADE7878_Mutex);
  ADE7878_MutexHandle = osMutexCreate(osMutex(ADE7878_Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MAIN_Task */
  osThreadDef(MAIN_Task, main_task, osPriorityNormal, 0, 128);
  MAIN_TaskHandle = osThreadCreate(osThread(MAIN_Task), NULL);

  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, led_task, osPriorityIdle, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of Flash_Task */
  osThreadDef(Flash_Task, flash_task, osPriorityIdle, 0, 128);
  Flash_TaskHandle = osThreadCreate(osThread(Flash_Task), NULL);

  /* definition and creation of BEEP_Task */
  osThreadDef(BEEP_Task, beep_task, osPriorityIdle, 0, 128);
  BEEP_TaskHandle = osThreadCreate(osThread(BEEP_Task), NULL);

  /* definition and creation of ADE7878_Task */
  osThreadDef(ADE7878_Task, ade7878_task, osPriorityHigh, 0, 128);
  ADE7878_TaskHandle = osThreadCreate(osThread(ADE7878_Task), NULL);

  /* definition and creation of SCREEN_Task */
  osThreadDef(SCREEN_Task, screen_task, osPriorityIdle, 0, 128);
  SCREEN_TaskHandle = osThreadCreate(osThread(SCREEN_Task), NULL);

  /* definition and creation of WWAN_Task */
  osThreadDef(WWAN_Task, wwan_task, osPriorityIdle, 0, 128);
  WWAN_TaskHandle = osThreadCreate(osThread(WWAN_Task), NULL);

  /* definition and creation of SHT31_Task */
  osThreadDef(SHT31_Task, sht31_task, osPriorityIdle, 0, 128);
  SHT31_TaskHandle = osThreadCreate(osThread(SHT31_Task), NULL);

  /* definition and creation of ETH_Task */
  osThreadDef(ETH_Task, eth_task, osPriorityIdle, 0, 128);
  ETH_TaskHandle = osThreadCreate(osThread(ETH_Task), NULL);

  /* definition and creation of KEY_Task */
  osThreadDef(KEY_Task, key_task, osPriorityIdle, 0, 128);
  KEY_TaskHandle = osThreadCreate(osThread(KEY_Task), NULL);

  /* definition and creation of SD_Task */
  osThreadDef(SD_Task, sd_task, osPriorityIdle, 0, 512);
  SD_TaskHandle = osThreadCreate(osThread(SD_Task), NULL);

  /* definition and creation of DATA_Task */
  osThreadDef(DATA_Task, data_task, osPriorityAboveNormal, 0, 256);
  DATA_TaskHandle = osThreadCreate(osThread(DATA_Task), NULL);

  /* definition and creation of IWDG_Task */
  osThreadDef(IWDG_Task, iwdg_task, osPriorityIdle, 0, 128);
  IWDG_TaskHandle = osThreadCreate(osThread(IWDG_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_main_task */
/**
  * @brief  Function implementing the MAIN_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_main_task */
void main_task(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN main_task */
  /* Infinite loop */
  for(;;)
  {
		if(rtc_ready==1)
		{
			HAL_RTC_GetTime(&hrtc, &time_get, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &date_get, RTC_FORMAT_BIN);//GetDate必须在GetTime之后
			//printf("\nTime:%02d-%02d-%02d %02d:%02d:%02d\n",date_get.Year,date_get.Month,date_get.Date,time_get.Hours,time_get.Minutes,time_get.Seconds);
		}
		
//		printf("A相电流瞬时值:%.2fA\n",i_WAVE_a);
//		printf("A相电压瞬时值:%.2fV\n",v_WAVE_a);
//	  printf("A相电流瞬时有效值:%.2fA\n",i_RMS_a);
//    printf("A相电压瞬时有效值:%.2fV\n",v_RMS_a);
//    printf("A相瞬时有功功率:%.2fW\n",A_WATT);
//    printf("A相有功功率电能累计:%.2fWh\n",A_WATTHR);
//		printf("A相瞬时无功功率:%.2fVAR\n",A_VAR);
//    printf("A相无功功率累计:%.2fVARh\n",A_VARHR);
//		
//		printf("\nB相电流瞬时值:%.2fA\n",i_WAVE_b);
//		printf("B相电压瞬时值:%.2fV\n",v_WAVE_b);
//	  printf("B相电流瞬时有效值:%.2fA\n",i_RMS_b);
//    printf("B相电压瞬时有效值:%.2fV\n",v_RMS_b);
//    printf("B相瞬时有功功率:%.2fW\n",B_WATT);
//    printf("B相有功功率电能累计:%.2fWh\n",B_WATTHR);
//		printf("B相瞬时无功功率:%.2fVAR\n",B_VAR);
//    printf("B相无功功率累计:%.2fVARh\n",B_VARHR);
//		
//		printf("\nC相电流瞬时值:%.2fA\n",i_WAVE_c);
//		printf("C相电压瞬时值:%.2fV\n",v_WAVE_c);
//	  printf("C相电流瞬时有效值:%.2fA\n",i_RMS_c);
//    printf("C相电压瞬时有效值:%.2fV\n",v_RMS_c);
//    printf("C相瞬时有功功率:%.2fW\n",C_WATT);
//    printf("C相有功功率电能累计:%.2fWh\n",C_WATTHR);
//		printf("C相瞬时无功功率:%.2fVAR\n",C_VAR);
//    printf("C相无功功率累计:%.2fVARh\n",C_VARHR);
//		
//		printf("\nN相(中性线)电流瞬时值:%.2fA\n",i_WAVE_n);
//	  printf("N相(中性线)电流瞬时有效值:%.2fA\n",i_RMS_n);
		
    osDelay(1000);
  }
  /* USER CODE END main_task */
}

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
void led_task(void const * argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(GPIOA, LED2_GREEN_Pin, GPIO_PIN_SET);
  osDelay(1000);
  HAL_GPIO_WritePin(GPIOA, LED2_GREEN_Pin, GPIO_PIN_RESET);
	osDelay(1000);
  }
  /* USER CODE END led_task */
}

/* USER CODE BEGIN Header_flash_task */
/**
* @brief Function implementing the Flash_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flash_task */
void flash_task(void const * argument)
{
  /* USER CODE BEGIN flash_task */
  /* Infinite loop */
  for(;;)
  {
		if(flash_writeflag==1)
		{
			printf("Write IP data from microSD to flash...");
			wwan_flash_ip[0]=wwan_ip[0];
			wwan_flash_ip[1]=wwan_ip[1];
			wwan_flash_ip[2]=wwan_ip[2];
			wwan_flash_ip[3]=wwan_ip[3];
			wwan_flash_port[0]=wwan_port&0xFF;
			wwan_flash_port[1]=(wwan_port>>8)&0xFF;
			BSP_W25Qx_Write(wwan_flash_ip, W25Q128FV_FLASH_SIZE-6000, sizeof(wwan_flash_ip));
			osDelay(100);
			BSP_W25Qx_Write(wwan_flash_port, W25Q128FV_FLASH_SIZE-4000, sizeof(wwan_flash_port));
			osDelay(100);
			flash_writeflag=0;
			printf("Done!\n");
		}
    osDelay(2000);
  }
  /* USER CODE END flash_task */
}

/* USER CODE BEGIN Header_beep_task */
/**
* @brief Function implementing the BEEP_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_beep_task */
void beep_task(void const * argument)
{
  /* USER CODE BEGIN beep_task */
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(200);
  }
  /* USER CODE END beep_task */
}

/* USER CODE BEGIN Header_ade7878_task */
/**
* @brief Function implementing the ADE7878_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ade7878_task */
void ade7878_task(void const * argument)
{
  /* USER CODE BEGIN ade7878_task */
	//u16 tcount=0;
	//u8 icount=0;
  
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(ADE7878_MutexHandle, portMAX_DELAY);
		for(int s=0;s<DataBufferSize;s++)
		{
			Read_ADE7878_SPI(AIRMS,4,AIRMSReg+4*(s%DataBufferSize));
			Read_ADE7878_SPI(AVRMS,4,AVRMSReg+4*(s%DataBufferSize));			 
			Read_ADE7878_SPI(AWATT,4,AWATTReg);
			Read_ADE7878_SPI(AVAR,4,AVARReg);
			Read_ADE7878_SPI(STATUS1,4,Status_1);
	//		Read_ADE7878_SPI(STATUS0,4,Status_0);
			Read_ADE7878_SPI(IAWV,4,IAWVReg);
			Read_ADE7878_SPI(VAWV,4,VAWVReg);
			Read_ADE7878_SPI(AWATTHR,4,AWATTHRReg);
			Read_ADE7878_SPI(AVARHR,4,AVARHRReg);
			
			Read_ADE7878_SPI(BIRMS,4,BIRMSReg+4*(s%DataBufferSize));
			Read_ADE7878_SPI(BVRMS,4,BVRMSReg+4*(s%DataBufferSize));			 
			Read_ADE7878_SPI(BWATT,4,BWATTReg);
			Read_ADE7878_SPI(BVAR,4,BVARReg);
			Read_ADE7878_SPI(STATUS1,4,Status_1);
			Read_ADE7878_SPI(IBWV,4,IBWVReg);
			Read_ADE7878_SPI(VBWV,4,VBWVReg);
			Read_ADE7878_SPI(BWATTHR,4,BWATTHRReg);
			Read_ADE7878_SPI(BVARHR,4,BVARHRReg);
			
			Read_ADE7878_SPI(CIRMS,4,CIRMSReg+4*(s%DataBufferSize));
			Read_ADE7878_SPI(CVRMS,4,CVRMSReg+4*(s%DataBufferSize));			 
			Read_ADE7878_SPI(CWATT,4,CWATTReg);
			Read_ADE7878_SPI(CVAR,4,CVARReg);
			Read_ADE7878_SPI(STATUS1,4,Status_1);
			Read_ADE7878_SPI(ICWV,4,ICWVReg);
			Read_ADE7878_SPI(VCWV,4,VCWVReg);
			Read_ADE7878_SPI(CWATTHR,4,CWATTHRReg);
			Read_ADE7878_SPI(CVARHR,4,CVARHRReg);
			
			Read_ADE7878_SPI(NIRMS,4,NIRMSReg+4*(s%DataBufferSize));
			Read_ADE7878_SPI(INWV,4,INWVReg);
			Read_ADE7878_SPI(STATUS1,4,Status_1);
			
			Read_ADE7878_SPI(VPEAK,4,VPEAKReg+4*(s%DataBufferSize));
			Read_ADE7878_SPI(IPEAK,4,IPEAKReg+4*(s%DataBufferSize)); 
			osDelay(5);
		}
		xSemaphoreGive(ADE7878_MutexHandle);

    osDelay(50);
  }
  /* USER CODE END ade7878_task */
}

/* USER CODE BEGIN Header_screen_task */
/**
* @brief Function implementing the SCREEN_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_screen_task */
void screen_task(void const * argument)
{
  /* USER CODE BEGIN screen_task */
	int data_temp=0;//数据发送临时变量
	//复位串口屏
	HAL_UART_Transmit(&huart6,page0_logo,sizeof(page0_logo),1000); //显示LOGO界面
	current_page=0;
	HAL_UART_Transmit(&huart6,lte_off,sizeof(lte_off),1000); //取消LTE显示
	HAL_UART_Transmit(&huart6,eth_off,sizeof(eth_off),1000); //取消ETH显示
	HAL_UART_Transmit(&huart6,server_off,sizeof(server_off),1000); //取消SERVER连接显示
	osDelay(500);
	HAL_UART_Transmit(&huart6,page1_vainfo,sizeof(page1_vainfo),1000); //显示电压电流界面
	current_page=1;
  /* Infinite loop */
  for(;;)
  {
		//发送各项参数到串口屏
		//发送温度
		writereg[4]=0x81;//Temp变量地址
		data_temp=(int)roundf(sht31_tmp*10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送湿度
		writereg[4]=0x80;//Humid变量地址
		data_temp=(int)roundf(sht31_hum*10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送AVRMS
		writereg[4]=0x40;//AVRMS变量地址
		data_temp=(int)roundf(v_RMS_a*10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送BVRMS
		writereg[4]=0x41;//BVRMS变量地址
		data_temp=(int)roundf(v_RMS_b *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送CVRMS
		writereg[4]=0x42;//CVRMS变量地址
		data_temp=(int)roundf(v_RMS_c *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送AIRMS
		writereg[4]=0x43;//AIRMS变量地址
		data_temp=(int)roundf(i_RMS_a *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送BIRMS
		writereg[4]=0x44;//BIRMS变量地址
		data_temp=(int)roundf(i_RMS_b *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送CIRMS
		writereg[4]=0x45;//CIRMS变量地址
		data_temp=(int)roundf(i_RMS_c *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送NIRMS
		writereg[4]=0x46;//NIRMS变量地址
		data_temp=(int)roundf(i_RMS_n *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送VA
		writereg[4]=0x47;//VA变量地址
		data_temp=(int)roundf(v_WAVE_a *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送VB
		writereg[4]=0x48;//VB变量地址
		data_temp=(int)roundf(v_WAVE_b *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送VC
		writereg[4]=0x49;//VC变量地址
		data_temp=(int)roundf(v_WAVE_c *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送IA
		writereg[4]=0x4A;//IA变量地址
		data_temp=(int)roundf(i_WAVE_a *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送IB
		writereg[4]=0x4B;//IB变量地址
		data_temp=(int)roundf(i_WAVE_b *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送IC
		writereg[4]=0x4C;//IC变量地址
		data_temp=(int)roundf(i_WAVE_c *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送IN
		writereg[4]=0x4D;//IN变量地址
		data_temp=(int)roundf(i_WAVE_n *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送AWATT
		writereg[4]=0x60;//AWATT变量地址
		data_temp=(int)roundf(A_WATT *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送BWATT
		writereg[4]=0x61;//BWATT变量地址
		data_temp=(int)roundf(B_WATT *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送CWATT
		writereg[4]=0x62;//CWATT变量地址
		data_temp=(int)roundf(C_WATT *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送AVAR
		writereg[4]=0x63;//AVAR变量地址
		data_temp=(int)roundf(A_VAR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送BVAR
		writereg[4]=0x64;//BVAR变量地址
		data_temp=(int)roundf(B_VAR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送CVAR
		writereg[4]=0x65;//CVAR变量地址
		data_temp=(int)roundf(C_VAR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送AWATTHR
		writereg[4]=0x67;//AWATTHR变量地址
		data_temp=(int)roundf(A_WATTHR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送BWATTHR
		writereg[4]=0x68;//BWATTHR变量地址
		data_temp=(int)roundf(B_WATTHR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送CWATTHR
		writereg[4]=0x69;//CWATTHR变量地址
		data_temp=(int)roundf(C_WATTHR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送AVARHR
		writereg[4]=0x6A;//AVARHR变量地址
		data_temp=(int)roundf(A_VARHR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送BVARHR
		writereg[4]=0x6B;//BVARHR变量地址
		data_temp=(int)roundf(B_VARHR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//发送CVARHR
		writereg[4]=0x6C;//CVARHR变量地址
		data_temp=(int)roundf(C_VARHR *10);
		writereg[7]=data_temp;//取临时变量低8位至数据低8位
		writereg[6]=data_temp>>8;//取临时变量高8位至数据高8位
		HAL_UART_Transmit(&huart6,writereg,sizeof(writereg),1000); //传送数据至串口屏
		//调整LTE图标
		if(lte_status==1)
			HAL_UART_Transmit(&huart6,lte_on,sizeof(lte_on),1000); //激活LTE显示
		else
			HAL_UART_Transmit(&huart6,lte_off,sizeof(lte_off),1000); //取消LTE显示
		//调整ETH图标
		if(eth_status==1)
			HAL_UART_Transmit(&huart6,eth_on,sizeof(eth_on),1000); //激活ETH显示
		else
			HAL_UART_Transmit(&huart6,eth_off,sizeof(eth_off),1000); //取消ETH显示
		//调整SERVER图标
		if(server_status==1)
			HAL_UART_Transmit(&huart6,server_on,sizeof(server_on),1000); //激活SERVER连接显示
		else
			HAL_UART_Transmit(&huart6,server_off,sizeof(server_off),1000); //取消SERVER连接显示
		
    osDelay(500);
  }
  /* USER CODE END screen_task */
}

/* USER CODE BEGIN Header_wwan_task */
/**
* @brief Function implementing the WWAN_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_wwan_task */
void wwan_task(void const * argument)
{
  /* USER CODE BEGIN wwan_task */
	char wwan_buffer[30];
	//WWAN初始化
	printf("WWAN module initialization start...\n");
	HAL_UART_Transmit(&huart3,at_command,sizeof(at_command),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,at_command,sizeof(at_command),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,at_command,sizeof(at_command),1000);
	osDelay(500);
	HAL_UART_Transmit(&huart3,sim_status,sizeof(sim_status),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,signal_quality,sizeof(signal_quality),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,network_regstate,sizeof(network_regstate),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,gprs_attachstate,sizeof(gprs_attachstate),1000);
	osDelay(1000);
	printf("Sync&get network time...\n");
	HAL_UART_Transmit(&huart3,time_sync,sizeof(time_sync),1000);
	osDelay(1000);
	HAL_UART_Receive_IT(&huart3,wwan_receive_temp,1);
	HAL_UART_Transmit(&huart3,get_time,sizeof(get_time),1000);
	osDelay(1000);
	yy=(wwan_receive_buffer[21]-'0')*10+wwan_receive_buffer[22]-'0';
	MM=(wwan_receive_buffer[24]-'0')*10+wwan_receive_buffer[25]-'0';
	dd=(wwan_receive_buffer[27]-'0')*10+wwan_receive_buffer[28]-'0';
	hh=(wwan_receive_buffer[30]-'0')*10+wwan_receive_buffer[31]-'0';
	mm=(wwan_receive_buffer[33]-'0')*10+wwan_receive_buffer[34]-'0';
	ss=(wwan_receive_buffer[36]-'0')*10+wwan_receive_buffer[37]-'0';
	time_set.Hours = hh;
  time_set.Minutes = mm;
  time_set.Seconds = ss;
  time_set.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time_set.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &time_set, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  date_set.WeekDay = RTC_WEEKDAY_SATURDAY;
  date_set.Month = MM;
  date_set.Date = dd;
  date_set.Year = yy;
  if (HAL_RTC_SetDate(&hrtc, &date_set, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
	rtc_ready=1;
	printf("Done! Time:%02d-%02d-%02d %02d:%02d:%02d\n",yy,MM,dd,hh,mm,ss);
	HAL_UART_Transmit(&huart3,transparent_mode,sizeof(transparent_mode),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,single_connection,sizeof(single_connection),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,cmcc_apn,sizeof(cmcc_apn),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,activate_mobile,sizeof(activate_mobile),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,get_ip,sizeof(get_ip),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,save_param,sizeof(save_param),1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart3,start_connection,sizeof(start_connection),1000);
	osDelay(1000);
	printf("WWAN module initialization finished!\n");
	lte_status=1;
  /* Infinite loop */
  for(;;)
  {
		server_status=1;
		sprintf(wwan_buffer,"Temp=%.2f℃ ",sht31_tmp);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
		osDelay(500);
		sprintf(wwan_buffer,"Humid=%.2f%%\n",sht31_hum);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AIWAVE=%.2fA\n",i_WAVE_a);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BIWAVE=%.2fA\n",i_WAVE_b);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CIWAVE=%.2fA\n",i_WAVE_c);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"NIWAVE=%.2fA\n",i_WAVE_n);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AVWAVE=%.2fV\n",v_WAVE_a);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BVWAVE=%.2fV\n",v_WAVE_b);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CVWAVE=%.2fV\n",v_WAVE_c);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AIRMS=%.2fA\n",i_RMS_a);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BIRMS=%.2fA\n",i_RMS_b);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CIRMS=%.2fA\n",i_RMS_c);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"NIRMS=%.2fA\n",i_RMS_n);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AVRMS=%.2fV\n",v_RMS_a);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BVRMS=%.2fV\n",v_RMS_b);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CVRMS=%.2fV\n",v_RMS_c);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AWATT=%.2fW\n",A_WATT);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BWATT=%.2fW\n",B_WATT);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CWATT=%.2fW\n",C_WATT);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AWAHR=%.2fWh\n",A_WATTHR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BWAHR=%.2fWh\n",B_WATTHR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CWAHR=%.2fWh\n",C_WATTHR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AVAR=%.2fVAR\n",A_VAR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BVAR=%.2fVAR\n",B_VAR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CVAR=%.2fVAR\n",C_VAR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"AVAHR=%.2fVARh\n",A_VARHR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"BVAHR=%.2fVARh\n",B_VARHR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
    osDelay(500);
		sprintf(wwan_buffer,"CVAHR=%.2fVARh\n",C_VARHR);
		HAL_UART_Transmit(&huart3,(unsigned char*)wwan_buffer,strlen(wwan_buffer),1000);
		
		server_status=0;
    osDelay(3000);
  }
  /* USER CODE END wwan_task */
}

/* USER CODE BEGIN Header_sht31_task */
/**
* @brief Function implementing the SHT31_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sht31_task */
void sht31_task(void const * argument)
{
  /* USER CODE BEGIN sht31_task */
  /* Infinite loop */
  for(;;)
  {
    sht31_sample(&sht31_tmp, &sht31_hum);
    //printf("Temp=%.2f℃\n", sht31_tmp);
    //printf("Humid=%.2f%%\n", sht31_hum);
    osDelay(500);
  }
  /* USER CODE END sht31_task */
}

/* USER CODE BEGIN Header_eth_task */
/**
* @brief Function implementing the ETH_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_eth_task */
void eth_task(void const * argument)
{
  /* USER CODE BEGIN eth_task */
	osDelay(10000);
	char eth_buffer[30];
	HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &lan_BSR);
	printf("PHY_BSR=%#x\n",lan_BSR);
	if(lan_BSR!=0x7809)
	{
		printf("Start UDP Connection...\n");
		udp_client_init();
	}
	else
	{
		printf("No ETH physical connect, skip establishing UDP connection!\n");
	}
  /* Infinite loop */
  for(;;)
  {
		HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &lan_BSR);
		if(lan_BSR!=0x7809)
		{
			server_status=1;
			sprintf(eth_buffer,"Temp=%.2f℃ ",sht31_tmp);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"Humid=%.2f%%\n",sht31_hum);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AIWAVE=%.2fA\n",i_WAVE_a);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BIWAVE=%.2fA\n",i_WAVE_b);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CIWAVE=%.2fA\n",i_WAVE_c);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"NIWAVE=%.2fA\n",i_WAVE_n);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AVWAVE=%.2fV\n",v_WAVE_a);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BVWAVE=%.2fV\n",v_WAVE_b);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CVWAVE=%.2fV\n",v_WAVE_c);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AIRMS=%.2fA\n",i_RMS_a);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BIRMS=%.2fA\n",i_RMS_b);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CIRMS=%.2fA\n",i_RMS_c);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"NIRMS=%.2fA\n",i_RMS_n);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AVRMS=%.2fV\n",v_RMS_a);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BVRMS=%.2fV\n",v_RMS_b);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CVRMS=%.2fV\n",v_RMS_c);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AWATT=%.2fW\n",A_WATT);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BWATT=%.2fW\n",B_WATT);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CWATT=%.2fW\n",C_WATT);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AWAHR=%.2fWh\n",A_WATTHR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BWAHR=%.2fWh\n",B_WATTHR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CWAHR=%.2fWh\n",C_WATTHR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AVAR=%.2fVAR\n",A_VAR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BVAR=%.2fVAR\n",B_VAR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CVAR=%.2fVAR\n",C_VAR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"AVAHR=%.2fVARh\n",A_VARHR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"BVAHR=%.2fVARh\n",B_VARHR);
			udp_client_send(eth_buffer);
			osDelay(500);
			sprintf(eth_buffer,"CVAHR=%.2fVARh\n",C_VARHR);
			udp_client_send(eth_buffer);	
			server_status=0;
		}
    osDelay(500);
  }
  /* USER CODE END eth_task */
}

/* USER CODE BEGIN Header_key_task */
/**
* @brief Function implementing the KEY_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_key_task */
void key_task(void const * argument)
{
  /* USER CODE BEGIN key_task */
  /* Infinite loop */
  for(;;)
  {
	  if (Key_Scan(GPIOE, KEY_UP_Pin) == 1)
        {
            printf("KEY_UP triggered!\n");
					  HAL_UART_Transmit(&huart6,page1_vainfo,sizeof(page1_vainfo),100); //显示电压电流界面
	          current_page=1;
        }
    if (Key_Scan(GPIOE, KEY_DOWN_Pin) == 1)
        {
            printf("KEY_DOWN triggered!\n");
					  HAL_UART_Transmit(&huart6,page2_wattinfo,sizeof(page2_wattinfo),100); //显示功率能量界面
	          current_page=2;
        }
	  if (Key_Scan(GPIOE, KEY_LEFT_Pin) == 1)
        {
            printf("KEY_LEFT triggered!\n");
					  HAL_UART_Transmit(&huart6,page1_vainfo,sizeof(page1_vainfo),100); //显示电压电流界面
	          current_page=1;
        }
	  if (Key_Scan(GPIOE, KEY_RIGHT_Pin) == 1)
        {
            printf("KEY_RIGHT triggered!\n");
					  HAL_UART_Transmit(&huart6,page2_wattinfo,sizeof(page2_wattinfo),100); //显示功率能量界面
	          current_page=2;
        }
	  if (Key_Scan(GPIOE, KEY_IN_Pin) == 1)
        {
            printf("KEY_IN triggered!\n");
					  HAL_UART_Transmit(&huart6,page2_wattinfo,sizeof(page2_wattinfo),100); //显示功率能量界面
	          current_page=2;
        }
	  if (Key_Scan(KEY_BACK_GPIO_Port, KEY_BACK_Pin) == 1)
        {
            printf("KEY_BACK triggered!\n");
					  HAL_UART_Transmit(&huart6,page1_vainfo,sizeof(page1_vainfo),100); //显示电压电流界面
	          current_page=1;
        }
    osDelay(50);
  }
  /* USER CODE END key_task */
}

/* USER CODE BEGIN Header_sd_task */
/**
* @brief Function implementing the SD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sd_task */
void sd_task(void const * argument)
{
  /* USER CODE BEGIN sd_task */
	uint32_t byteswritten;                /* File write counts */
  uint32_t bytesread;                   /* File read counts */
  uint8_t wtext[] = "FATFS@FreeRTOS TEST\n"; /* File write buffer */
  uint8_t rtext[150];                     /* File read buffers */
  char test_filename[] = "FATFS_Test.txt";
	char config_filename[] = "config.txt";
	char sprintf_buffer[100];
	//FATFS测试
	if (HAL_GPIO_ReadPin(TF_IN_GPIO_Port,TF_IN_Pin) == 0)
	{
		printf("Test FATFS:\n");
		HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin, GPIO_PIN_SET);
		/*##-1- Register the file system object to the FatFs module ##############*/
		retSD = f_mount(&SDFatFS, "", 0);
		if(retSD)
		{
				printf("FATFS mount error : %d \n",retSD);
				Error_Handler();
		}
		else
				printf("FATFS mount sucess! \n");     
		/*##-2- Create and Open new text file objects with write access ######*/
		retSD = f_open(&SDFile, test_filename, FA_CREATE_ALWAYS | FA_WRITE);
		if(retSD)
				printf("FATFS open file error : %d\n",retSD);
		else
				printf("FATFS open file sucess! \n");    
		/*##-3- Write data to the text files ###############################*/
		retSD = f_write(&SDFile, wtext, sizeof(wtext), (void *)&byteswritten);
		if(retSD)
				printf("FATFS write file error : %d\n",retSD);
		else
		{
				printf("FATFS write file sucess!\n");
				printf("FATFS write Data : %s\n",wtext);
		}    
		/*##-4- Close the open text files ################################*/
		retSD = f_close(&SDFile);
		if(retSD)
				printf("FATFS close error : %d\n",retSD);
		else
				printf("FATFS close sucess! \n");     
		/*##-5- Open the text files object with read access ##############*/
		retSD = f_open(&SDFile, test_filename, FA_READ);
		if(retSD)
				printf("FATFS open file error : %d\n",retSD);
		else
				printf("FATFS open file sucess!\n");    
		/*##-6- Read data from the text files ##########################*/
		retSD = f_read(&SDFile, rtext, sizeof(rtext), (UINT*)&bytesread);
		if(retSD)
				printf("FATFS read error!%d\n",retSD);
		else
		{
				printf("FATFS read sucess!\n");
				printf("FATFS read Data : %s\n",rtext);
		}    
		/*##-7- Close the open text files ############################*/
		retSD = f_close(&SDFile);
		if(retSD)  
				printf("FATFS close error! %d\n",retSD);
		else
				printf("FATFS close sucess! \n");   
		/*##-8- Compare read data with the expected data ############*/
		if(bytesread == byteswritten)
		{ 
				printf("FATFS TEST PASS!\n");
		}
		//查看SDMMC卡信息
		printf("Scan SDMMC Info:\n");
		Show_SDMMC_Info();
		//读取config文件信息
		printf("Reading config.txt...\n");
		memset(rtext,'\0',sizeof(rtext));
		retSD = f_mount(&SDFatFS, "", 0);
		retSD = f_open(&SDFile, config_filename, FA_READ);
		retSD = f_read(&SDFile, rtext, sizeof(rtext), (UINT*)&bytesread);
		retSD = f_close(&SDFile);
		printf("config.txt:%s\n",rtext);
		unsigned char ip_status=0;
		unsigned char ip_pointer=0;
		for(ip_pointer=0;ip_pointer<sizeof(rtext);ip_pointer++)
		{
			if(rtext[ip_pointer]=='\0'||rtext[ip_pointer]=='\n')
				break;
			else if(rtext[ip_pointer]=='.')
				ip_status++;
			else
				wwan_ip[ip_status]=wwan_ip[ip_status]*10+rtext[ip_pointer]-'0';
		}
		ip_pointer++;
		for(;ip_pointer<sizeof(rtext);ip_pointer++)
		{
			if(rtext[ip_pointer]=='\0'||rtext[ip_pointer]=='\n')
				break;
			wwan_port=wwan_port*10+(rtext[ip_pointer]-'0');
		}
		printf("Read IP=%d.%d.%d.%d, port=%d\n",wwan_ip[0],wwan_ip[1],wwan_ip[2],wwan_ip[3],wwan_port);
		printf("New IP takes effect after reboot!\n");
		flash_writeflag=1;
		//开始记录log
		printf("Log start!\n");
		Write_TFCard("log.txt","***********LOG START***********\n");
		sprintf(sprintf_buffer,"Software version:%.1f System clock@%dHz\n",SOFTWARE_VERSION,sysclock);
		Write_TFCard("log.txt",sprintf_buffer);
		sprintf(sprintf_buffer,"Build Time:%02d-%02d-%02d %02d:%02d:%02d CST\n",MDK_YEAR,MDK_MONTH,MDK_DAY,MDK_HOUR,MDK_MIN,MDK_SEC);
		Write_TFCard("log.txt",sprintf_buffer);
		HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin, GPIO_PIN_RESET);
	}
	else
	{
		printf("No SD card detected, skip testing!\n");
		HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin, GPIO_PIN_RESET);
	}
  /* Infinite loop */
  for(;;)
  {
		if (HAL_GPIO_ReadPin(TF_IN_GPIO_Port,TF_IN_Pin) == 0)
		{
			HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin, GPIO_PIN_SET);
			if(rtc_ready==1)
		  {
			  HAL_RTC_GetTime(&hrtc, &time_get, RTC_FORMAT_BIN);
			  HAL_RTC_GetDate(&hrtc, &date_get, RTC_FORMAT_BIN);//GetDate必须在GetTime之后
			  sprintf(sprintf_buffer,"LOG TIME:%02d-%02d-%02d %02d:%02d:%02d\n",date_get.Year,date_get.Month,date_get.Date,time_get.Hours,time_get.Minutes,time_get.Seconds);
				Write_TFCard("log.txt",sprintf_buffer);
		  }
			sprintf(sprintf_buffer,"SHT31 Sensor: Temp=%.2f℃ Humid=%.2f%%\n",sht31_tmp, sht31_hum);
			Write_TFCard("log.txt",sprintf_buffer);
			sprintf(sprintf_buffer,"LAN8720A: PHY_BSR=%#x\n",lan_BSR);
			Write_TFCard("log.txt",sprintf_buffer);
			sprintf(sprintf_buffer,"AIWAVE=%.2fA AVWAVE=%.2fV AIRMS=%.2fA AVRMS=%.2fV AWATT=%.2fW AWATTHR=%.2fWh AVAR=%.2fVAR AVARHR=%.2fVARh\n",i_WAVE_a,v_WAVE_a,i_RMS_a,v_RMS_a,A_WATT,A_WATTHR,A_VAR,A_VARHR);
			Write_TFCard("log.txt",sprintf_buffer);
			sprintf(sprintf_buffer,"BIWAVE=%.2fA BVWAVE=%.2fV BIRMS=%.2fA BVRMS=%.2fV BWATT=%.2fW BWATTHR=%.2fWh BVAR=%.2fVAR BVARHR=%.2fVARh\n",i_WAVE_b,v_WAVE_b,i_RMS_b,v_RMS_b,B_WATT,B_WATTHR,B_VAR,B_VARHR);
			Write_TFCard("log.txt",sprintf_buffer);
			sprintf(sprintf_buffer,"CIWAVE=%.2fA CVWAVE=%.2fV CIRMS=%.2fA CVRMS=%.2fV CWATT=%.2fW CWATTHR=%.2fWh CVAR=%.2fVAR CVARHR=%.2fVARh\n",i_WAVE_c,v_WAVE_c,i_RMS_c,v_RMS_c,C_WATT,C_WATTHR,C_VAR,C_VARHR);
			Write_TFCard("log.txt",sprintf_buffer);
			sprintf(sprintf_buffer,"NIWAVE=%.2fA NIRMS=%.2fA\n",i_WAVE_n,i_RMS_n);
			Write_TFCard("log.txt",sprintf_buffer);
			HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin, GPIO_PIN_RESET);
		}
		else
		{
			printf("No SD card detected, skip logging!\n");
			HAL_GPIO_WritePin(GPIOA, LED3_BLUE_Pin, GPIO_PIN_RESET);
		}
		
		osDelay(1000);//每1s记录一次log
  }
  /* USER CODE END sd_task */
}

/* USER CODE BEGIN Header_data_task */
/**
* @brief Function implementing the DATA_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_data_task */
void data_task(void const * argument)
  {
  /* USER CODE BEGIN data_task */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(ADE7878_MutexHandle, portMAX_DELAY);//获取ADE7878 Mutex
		//A相有功功率的瞬时值
		if(AWATTReg[1]>=0X80)
		{			
			awatt = -(16777216*1 - (AWATTReg[1])*65536 - (AWATTReg[2])*256 -(AWATTReg[3]));				
		}
		if(AWATTReg[1]<0X80) 
		{
			awatt = AWATTReg[3] +  AWATTReg[2]*256 +  AWATTReg[1]*65536;
		}
		
		//A相无功功率的瞬时值
		if(AVARReg[1]>=0X80)
		{			
			avar = -(16777216*1 - (AVARReg[1])*65536 - (AVARReg[2])*256 -(AVARReg[3]));				
		}
		if(AVARReg[1]<0X80) 
		{
			avar = AVARReg[3] +  AVARReg[2]*256 +  AVARReg[1]*65536;
		}
      
		//A相电流的瞬时值
		if(IAWVReg[1]>=0X80)
		{			
			iwave_a = -(16777216*1 - (IAWVReg[1])*65536 - (IAWVReg[2])*256 -(IAWVReg[3]));				
		}
		if(IAWVReg[1]<0X80) 
		{
			iwave_a = IAWVReg[3] + IAWVReg[2]*256 + IAWVReg[1]*65536;
		}
	     
		//A相电压的瞬时值
		if(VAWVReg[1]>=0X80)
		{			
			vwave_a = -(16777216*1 - (VAWVReg[1])*65536 - (VAWVReg[2])*256 -(VAWVReg[3]));				
		}
		if(VAWVReg[1]<0X80) 
		{
			vwave_a = VAWVReg[3] + VAWVReg[2]*256 + VAWVReg[1]*65536;
		}
		
		//B相有功功率的瞬时值
		if(BWATTReg[1]>=0X80)
		{			
			bwatt = -(16777216*1 - (BWATTReg[1])*65536 - (BWATTReg[2])*256 -(BWATTReg[3]));				
		}
		if(BWATTReg[1]<0X80) 
		{
			bwatt = BWATTReg[3] +  BWATTReg[2]*256 +  BWATTReg[1]*65536;
		}
      
		//B相无功功率的瞬时值
		if(BVARReg[1]>=0X80)
		{			
			bvar = -(16777216*1 - (BVARReg[1])*65536 - (BVARReg[2])*256 -(BVARReg[3]));				
		}
		if(BVARReg[1]<0X80) 
		{
			bvar = BVARReg[3] +  BVARReg[2]*256 +  BVARReg[1]*65536;
		}
		
		//B相电流的瞬时值
		if(IBWVReg[1]>=0X80)
		{			
			iwave_b = -(16777216*1 - (IBWVReg[1])*65536 - (IBWVReg[2])*256 -(IBWVReg[3]));				
		}
		if(IBWVReg[1]<0X80) 
		{
			iwave_b = IBWVReg[3] + IBWVReg[2]*256 + IBWVReg[1]*65536;
		}
	     
		//B相电压的瞬时值
		if(VBWVReg[1]>=0X80)
		{			
			vwave_b = -(16777216*1 - (VBWVReg[1])*65536 - (VBWVReg[2])*256 -(VBWVReg[3]));				
		}
		if(VBWVReg[1]<0X80) 
		{
			vwave_b = VBWVReg[3] + VBWVReg[2]*256 + VBWVReg[1]*65536;
		}
		
		//C相有功功率的瞬时值
		if(CWATTReg[1]>=0X80)
		{			
			cwatt = -(16777216*1 - (CWATTReg[1])*65536 - (CWATTReg[2])*256 -(CWATTReg[3]));				
		}
		if(CWATTReg[1]<0X80) 
		{
			cwatt = CWATTReg[3] +  CWATTReg[2]*256 +  CWATTReg[1]*65536;
		}
      
		//C相无功功率的瞬时值
		if(CVARReg[1]>=0X80)
		{			
			cvar = -(16777216*1 - (CVARReg[1])*65536 - (CVARReg[2])*256 -(CVARReg[3]));				
		}
		if(CVARReg[1]<0X80) 
		{
			cvar = CVARReg[3] +  CVARReg[2]*256 +  CVARReg[1]*65536;
		}
		
		//C相电流的瞬时值
		if(ICWVReg[1]>=0X80)
		{			
			iwave_c = -(16777216*1 - (ICWVReg[1])*65536 - (ICWVReg[2])*256 -(ICWVReg[3]));				
		}
		if(ICWVReg[1]<0X80) 
		{
			iwave_c = ICWVReg[3] + ICWVReg[2]*256 + ICWVReg[1]*65536;
		}
	     
		//C相电压的瞬时值
		if(VCWVReg[1]>=0X80)
		{			
			vwave_c = -(16777216*1 - (VCWVReg[1])*65536 - (VCWVReg[2])*256 -(VCWVReg[3]));				
		}
		if(VCWVReg[1]<0X80) 
		{
			vwave_c = VCWVReg[3] + VCWVReg[2]*256 + VCWVReg[1]*65536;
		}
		
		//(中性线)N相电流的瞬时值
		if(INWVReg[1]>=0X80)
		{			
			iwave_n = -(16777216*1 - (INWVReg[1])*65536 - (INWVReg[2])*256 -(INWVReg[3]));				
		}
		if(INWVReg[1]<0X80) 
		{
			iwave_n = INWVReg[3] + INWVReg[2]*256 + INWVReg[1]*65536;
		}
		
		for(unsigned char i=0; i<DataBufferSize;i++)
		{
			irms_a = irms_a + (AIRMSReg[4*i+3] + AIRMSReg[4*i+2]*256 + AIRMSReg[4*i+1]*65536)*1.0f/DataBufferSize;//对AIRMS寄存器中A相电流瞬时有效值进行均值滤波(20次)
      vrms_a = vrms_a + (AVRMSReg[4*i+3] + AVRMSReg[4*i+2]*256 + AVRMSReg[4*i+1]*65536)*1.0f/DataBufferSize;//对AVRMS寄存器中A相电压瞬时有效值进行均值滤波(20次)
			
			irms_b = irms_b + (BIRMSReg[4*i+3] + BIRMSReg[4*i+2]*256 + BIRMSReg[4*i+1]*65536)*1.0f/DataBufferSize;//对BIRMS寄存器中B相电流瞬时有效值进行均值滤波(20次)
      vrms_b = vrms_b + (BVRMSReg[4*i+3] + BVRMSReg[4*i+2]*256 + BVRMSReg[4*i+1]*65536)*1.0f/DataBufferSize;//对BVRMS寄存器中B相电压瞬时有效值进行均值滤波(20次)
			
			irms_c = irms_c + (CIRMSReg[4*i+3] + CIRMSReg[4*i+2]*256 + CIRMSReg[4*i+1]*65536)*1.0f/DataBufferSize;//对CIRMS寄存器中C相电流瞬时有效值进行均值滤波(20次)
      vrms_c = vrms_c + (CVRMSReg[4*i+3] + CVRMSReg[4*i+2]*256 + CVRMSReg[4*i+1]*65536)*1.0f/DataBufferSize;//对CVRMS寄存器中C相电压瞬时有效值进行均值滤波(20次)
			
			irms_n = irms_n + (NIRMSReg[4*i+3] + NIRMSReg[4*i+2]*256 + NIRMSReg[4*i+1]*65536)*1.0f/DataBufferSize;//对NIRMS寄存器中N相(中性线)电流瞬时有效值进行均值滤波(20次)
		}
		
		i_RMS_a=(irms_a/4191910.0f) *50.0f/sqrtf(2.0f);	 //将irms_x中的数值转换为实际电流有效值(A)
		A_WATT = (awatt/33516139.0f)*62500.0f/3.0f*16;//将xwatt中的数值转换为实际有功功率值(W)
		A_VAR = (avar/33516139.0f)*62500.0f/3.0f*16;//将xvar中的数值转换为实际无功功率值(VAR)
		v_RMS_a=(vrms_a/4191910.0f)*0.5f/sqrtf(2.0f)*200000.0f/120.0f;//将vrms_x中的数值转换为实际电压有效值(V)
		i_WAVE_a=(iwave_a/5928256.0f) *50;//将iwave_x中的数值转换为实际电流(A)
		v_WAVE_a=(vwave_a/5928256.0f) *0.5f*200000.0f/120.0f;//将vwave_x中的数值转换为实际电压(V)
		awatthr = (AWATTHRReg[3] +  AWATTHRReg[2]*256 +  AWATTHRReg[1]*65536 + AWATTHRReg[0]*16777216)*1.0f;
		A_WATTHR=awatthr*0.01f+A_WATTHR_Acc;
		avarhr = (AVARHRReg[3] +  AVARHRReg[2]*256 +  AVARHRReg[1]*65536 + AVARHRReg[0]*16777216)*1.0f;
		A_VARHR=avarhr*0.01f+A_VARHR_Acc;
		if(MEASURE_SERIAL_OUTPUT)
		{
			printf("\nA相电流瞬时值:%.2fA\n",i_WAVE_a);
			printf("A相电压瞬时值:%.2fV\n",v_WAVE_a);
//		printf("A相电流瞬时有效数值:%.2f\n",irms_a);
			printf("A相电流瞬时有效值:%.2fA\n",i_RMS_a);
//		printf("A相电压瞬时有效数值:%.2f\n",vrms_a);
			printf("A相电压瞬时有效值:%.2fV\n",v_RMS_a);
			printf("A相瞬时有功功率:%.2fW\n",A_WATT);
			printf("A相有功功率电能累计:%.2fWh\n",A_WATTHR);
			printf("A相瞬时无功功率:%.2fVAR\n",A_VAR);
			printf("A相无功功率累计:%.2fVARh\n",A_VARHR);
		}
				
		
		i_RMS_b=(irms_b/4191910.0f) *50.0f/sqrtf(2.0f);	 //将irms_x中的数值转换为实际电流有效值(A)
		B_WATT = (bwatt/33516139.0f)*62500.0f/3.0f*16;//将xwatt中的数值转换为实际有功功率值(W)
		B_VAR = (bvar/33516139.0f)*62500.0f/3.0f*16;//将xvar中的数值转换为实际无功功率值(VAR)
		v_RMS_b=(vrms_b/4191910.0f)*0.5f/sqrtf(2.0f)*200000.0f/120.0f;//将vrms_x中的数值转换为实际电压有效值(V)
		i_WAVE_b=(iwave_b/5928256.0f) *50;//将iwave_x中的数值转换为实际电流(A)
		v_WAVE_b=(vwave_b/5928256.0f) *0.5f*200000.0f/120.0f;//将vwave_x中的数值转换为实际电压(V)
		bwatthr = (BWATTHRReg[3] +  BWATTHRReg[2]*256 +  BWATTHRReg[1]*65536 + BWATTHRReg[0]*16777216)*1.0f;
		B_WATTHR=bwatthr*0.01f+B_WATTHR_Acc;
		bvarhr = (BVARHRReg[3] +  BVARHRReg[2]*256 +  BVARHRReg[1]*65536 + BVARHRReg[0]*16777216)*1.0f;
		B_VARHR=bvarhr*0.01f+B_VARHR_Acc;
		if(MEASURE_SERIAL_OUTPUT)
		{
			printf("\nB相电流瞬时值:%.2fA\n",i_WAVE_b);
			printf("B相电压瞬时值:%.2fV\n",v_WAVE_b);
//		printf("B相电流瞬时有效数值:%.2f\n",irms_b);
			printf("B相电流瞬时有效值:%.2fA\n",i_RMS_b);
//		printf("B相电压瞬时有效数值:%.2f\n",vrms_b);
			printf("B相电压瞬时有效值:%.2fV\n",v_RMS_b);
			printf("B相瞬时有功功率:%.2fW\n",B_WATT);
			printf("B相有功功率电能累计:%.2fWh\n",B_WATTHR);
			printf("B相瞬时无功功率:%.2fVAR\n",B_VAR);
			printf("B相无功功率累计:%.2fVARh\n",B_VARHR);
		}
			
		
		i_RMS_c=(irms_c/4191910.0f) *50.0f/sqrtf(2.0f);	 //将irms_x中的数值转换为实际电流有效值(A)
		C_WATT = (cwatt/33516139.0f)*62500.0f/3.0f*16;//将xwatt中的数值转换为实际有功功率值(W)
		C_VAR = (cvar/33516139.0f)*62500.0f/3.0f*16;//将xvar中的数值转换为实际无功功率值(VAR)
		v_RMS_c=(vrms_c/4191910.0f)*0.5f/sqrtf(2.0f)*200000.0f/120.0f;//将vrms_x中的数值转换为实际电压有效值(V)
		i_WAVE_c=(iwave_c/5928256.0f) *50;//将iwave_x中的数值转换为实际电流(A)
		v_WAVE_c=(vwave_c/5928256.0f) *0.5f*200000.0f/120.0f;//将vwave_x中的数值转换为实际电压(V)
		cwatthr = (CWATTHRReg[3] +  CWATTHRReg[2]*256 +  CWATTHRReg[1]*65536 + CWATTHRReg[0]*16777216)*1.0f;
		C_WATTHR=cwatthr*0.01f+C_WATTHR_Acc;
		cvarhr = (CVARHRReg[3] +  CVARHRReg[2]*256 +  CVARHRReg[1]*65536 + CVARHRReg[0]*16777216)*1.0f;
		C_VARHR=cvarhr*0.01f+C_VARHR_Acc;
		if(MEASURE_SERIAL_OUTPUT)	
		{
			printf("\nC相电流瞬时值:%.2fA\n",i_WAVE_c);
			printf("C相电压瞬时值:%.2fV\n",v_WAVE_c);
//		printf("C相电流瞬时有效数值:%.2f\n",irms_c);
			printf("C相电流瞬时有效值:%.2fA\n",i_RMS_c);
//		printf("C相电压瞬时有效数值:%.2f\n",vrms_c);
			printf("C相电压瞬时有效值:%.2fV\n",v_RMS_c);
			printf("C相瞬时有功功率:%.2fW\n",C_WATT);
			printf("C相有功功率电能累计:%.2fWh\n",C_WATTHR);
			printf("C相瞬时无功功率:%.2fVAR\n",C_VAR);
			printf("C相无功功率累计:%.2fVARh\n",C_VARHR);
		}			
				
		
		i_RMS_n=(irms_n/4191910.0f) *50.0f/sqrtf(2.0f);	 //将irms_x中的数值转换为实际电流有效值(A)
		i_WAVE_n=(iwave_n/5928256.0f) *50;//将iwave_x中的数值转换为实际电流(A)
		if(MEASURE_SERIAL_OUTPUT)	
		{
			printf("\nN相(中性线)电流瞬时值:%.2fA\n",i_WAVE_n);
//		printf("N相(中性线)电流瞬时有效数值:%.2f\n",irms_n);
			printf("N相(中性线)电流瞬时有效值:%.2fA\n",i_RMS_n);
		}

		
		//ADE7878寄存器溢出保护
		if(awatthr>42900000||bwatthr>42900000||cwatthr>42900000||avarhr>42900000||bvarhr>42900000||cvarhr>42900000)
		{
			A_WATTHR_Acc=A_WATTHR;
			B_WATTHR_Acc=B_WATTHR;
			C_WATTHR_Acc=C_WATTHR;
			A_VARHR_Acc=A_VARHR;
			B_VARHR_Acc=B_VARHR;
			C_VARHR_Acc=C_VARHR;
			
			//打开读取复位设定
			//禁用DSP RAM写保护
			unsigned char RAM[3] = {0xAD, 0X00, 0x80};   //DSP RAM保护寄存器值
			Write_ADE7878_SPI(0xE7FE, 1, RAM);     //向0xE7FE写入0xAD
			Write_ADE7878_SPI(0xE7E3, 1, RAM + 1); //向0xE7E3写入0x00
			unsigned char LCYCMODE_cof[1] = {0x40};       // 不设置任何线周期累计模式，打开xWATTHR读取复位设定
      Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器
			Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器
			Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器
			Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器//队列最后一个寄存器写3次，保证写入
			//DSP RAM写保护
			Write_ADE7878_SPI(0xE7FE, 1, RAM);     //向0xE7FE写入0xAD
			Write_ADE7878_SPI(0xE7E3, 1, RAM + 2); //向0xE7E3写入0x80
			osDelay(50);
			
			//读取以清空WATTHR和VARHR寄存器
			Read_ADE7878_SPI(AWATTHR,4,AWATTHRReg);
			Read_ADE7878_SPI(AVARHR,4,AVARHRReg);
			Read_ADE7878_SPI(BWATTHR,4,BWATTHRReg);
			Read_ADE7878_SPI(BVARHR,4,BVARHRReg);
			Read_ADE7878_SPI(CWATTHR,4,CWATTHRReg);
			Read_ADE7878_SPI(CVARHR,4,CVARHRReg);
			osDelay(50);
			
			//重新关闭读取复位设定
			//禁用DSP RAM写保护
			Write_ADE7878_SPI(0xE7FE, 1, RAM);     //向0xE7FE写入0xAD
			Write_ADE7878_SPI(0xE7E3, 1, RAM + 1); //向0xE7E3写入0x00
			LCYCMODE_cof[0] = 0x00;       // 不设置任何线周期累计模式，关闭xWATTHR读取复位设定
      Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器
			Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器
			Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器
			Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器//队列最后一个寄存器写3次，保证写入
			//DSP RAM写保护
			Write_ADE7878_SPI(0xE7FE, 1, RAM);     //向0xE7FE写入0xAD
			Write_ADE7878_SPI(0xE7E3, 1, RAM + 2); //向0xE7E3写入0x80
			osDelay(50);
			
		}
		
		
		irms_a=0;
		awatt = 0;
		vrms_a=0;
		awatthr=0;
		avar=0;
		avarhr=0;
		
		irms_b=0;
		bwatt = 0;		
		vrms_b=0;
		bwatthr=0;
		bvar=0;
		bvarhr=0;
		
		irms_c=0;		
		cwatt = 0;	  
		vrms_c=0;		
	  cwatthr=0;
		cvar=0;
		cvarhr=0;
		
		irms_n=0;	
		
		xSemaphoreGive(ADE7878_MutexHandle);//释放ADE7878 Mutex
    osDelay(50);
  }
  /* USER CODE END data_task */
}

/* USER CODE BEGIN Header_iwdg_task */
/**
* @brief Function implementing the IWDG_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_iwdg_task */
void iwdg_task(void const * argument)
{
  /* USER CODE BEGIN iwdg_task */
	MX_IWDG_Init();//启动IWDG
  /* Infinite loop */
  for(;;)
  {
		HAL_IWDG_Refresh(&hiwdg);//喂狗
    osDelay(500);
  }
  /* USER CODE END iwdg_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
