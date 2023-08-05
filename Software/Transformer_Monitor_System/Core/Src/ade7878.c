#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "ade7878.h"
#include "stm32f407xx.h"

void ade7878_delay(u32 nCount)
{
  HAL_Delay(nCount);
}

void ADE7878_PSM0_Enable(void)
{
  ADE_RESET_0;
  ade7878_delay(500);
  ADE_RESET_1;
  ade7878_delay(500);
}

unsigned char SPI2_ReadWrite(unsigned char SPI_Send_byte)
{
  unsigned char return_temp[1] = {0};
  HAL_SPI_TransmitReceive(&hspi2, &SPI_Send_byte, return_temp, 1, 1000);
  return return_temp[0];
}

void Write_ADE7878_SPI(unsigned int ADE_Addr, unsigned char Nr_Bytes, unsigned char *pReg_Data)
{
  unsigned char i;
  unsigned char MS_Addr;
  unsigned char LS_Addr;
  LS_Addr = (unsigned char)ADE_Addr;
  MS_Addr = (unsigned char)(ADE_Addr >> 8);
  ADE_CS_0;
  SPI2_ReadWrite(0x00); // 0x00表示接下来要对所发送地址对应的寄存器进行写操作
  SPI2_ReadWrite(MS_Addr);
  SPI2_ReadWrite(LS_Addr);
  for (i = 0; i < Nr_Bytes; i++)
  {
    SPI2_ReadWrite(*pReg_Data);
    pReg_Data++;
  }
  ADE_CS_1;
}

void Read_ADE7878_SPI(unsigned int ADE_Addr, unsigned char Nr_Bytes, unsigned char *pReg_Data)
{
  unsigned char i;
  unsigned char MS_Addr;
  unsigned char LS_Addr;
  LS_Addr = (unsigned char)ADE_Addr;
  MS_Addr = (unsigned char)(ADE_Addr >> 8);
  ADE_CS_0;
  SPI2_ReadWrite(0x01); //	0x01表示接下来要对所发送地址对应的寄存器进行读操作
  SPI2_ReadWrite(MS_Addr);
  SPI2_ReadWrite(LS_Addr);
  for (i = 0; i < Nr_Bytes; i++)
  {
    *pReg_Data = SPI2_ReadWrite(0xff); //读取SPI时写入dummy data 0xff
    pReg_Data++;
  }
  ADE_CS_1;
}

void ADE7878_SPI_Enable(void)
{
  unsigned char CONFIG2Reg[1] = {0x02};
  ADE_CS_1;
  ade7878_delay(50);
  ADE_CS_0;
  ade7878_delay(50);
  ADE_CS_1;
  ade7878_delay(50);
  ADE_CS_0;
  ade7878_delay(50);
  ADE_CS_1;
  ade7878_delay(50);
  ADE_CS_0;
  ade7878_delay(50);
  ADE_CS_1;
  Write_ADE7878_SPI(CONFIG2, 1, CONFIG2Reg); //对CONFIG2寄存器执行任意写操作选取通信方式（spi）维持不变
}

void ADE7878_BASICCONFIG(void) // ADE7878基本配置
{
  //禁用DSP RAM写保护
  unsigned char RAM[3] = {0xAD, 0X00, 0x80};   //DSP RAM保护寄存器值
  Write_ADE7878_SPI(0xE7FE, 1, RAM);     //向0xE7FE写入0xAD
  Write_ADE7878_SPI(0xE7E3, 1, RAM + 1); //向0xE7E3写入0x00

  //设置PGA增益
  unsigned char GAIN_cof[2] = {0x00, 0x00}; // 增益为1，不放大
  Write_ADE7878_SPI(GAIN, 2, GAIN_cof);     //向GAIN寄存器写入设置内容

  //禁用积分器，设置电压电流相序对应关系
  unsigned char CONFIG_cof[2] = {0x00, 0x00}; // CONFIG寄存器内容
  Write_ADE7878_SPI(CONFIG, 2, CONFIG_cof);   //向CONFIG寄存器中写入设置内容

  //设置电能累计模式、Y形连接方式与相序检测
  unsigned char ACCMODE_cof[1] = {0x00};      // ACCMODE寄存器内容
  Write_ADE7878_SPI(ACCMODE, 1, ACCMODE_cof); //向ACCMODE寄存器中写入设置内容

  //设置相位间隔、数字频率转换CFx
  unsigned char Compmode_cof[2] = {0x00, 0x01}; // CF1引脚，转换A相功率至数字频率
  // unsigned char Compmode_cof[2]={0x00,0x04};  //CF1引脚，转换C相功率至数字频率
  Write_ADE7878_SPI(COMPMODE, 2, Compmode_cof); //测角度，CF,视在功率等的设置

  //设置电压线路周期测量的相来源、需要进行电压电流峰值检测的相
  // unsigned char MMODE_cof[1]={0x12};//C相作为周期测量源，C相峰值检测
  unsigned char MMODE_cof[1] = {0x1c}; // A相作为周期测量源，ABC相峰值检测
  // unsigned char MMODE_cof[1]={0x04};//A相作为周期测量源，A相峰值检测
  Write_ADE7878_SPI(MMODE, 1, MMODE_cof); //写入MMODE寄存器，线周期以A相为基准以及开启峰值检测

  //设置线路周期有功功率累计模式（简化校准、提高短时间电能累计精度）
  // unsigned char LCYCMODE_cof[1]={0x21};	//C相计入线周期过零计数，WATT置于线周期累计模式
  // unsigned char LCYCMODE_cof[1]={0x49};	//YK修改 2016.4.5
  //unsigned char LCYCMODE_cof[1] = {0x09};       // A相计入线周期过零计数，WATT置于线周期累计模式
	unsigned char LCYCMODE_cof[1] = {0x00};       // 不设置任何线周期累计模式，取消xWATTHR读取复位设定
  Write_ADE7878_SPI(LCYCMODE, 1, LCYCMODE_cof); //写入LCYCMODE寄存器

  //设置线周期过零计数数量（每n个半波周期的电能累计写入瓦时累计寄存器）
  // u8 LINECYC_cof[2]={0x00,0x64};   //100个半波周期
  // u8 LINECYC_cof[2]={0x03,0xe8};	 //1000个半波周期
  // u8 LINECYC_cof[2]={0x01,0x90};	 //400个半波周期
  //u8 LINECYC_cof[2] = {0x75, 0x30};           // 30000个半波周期
  //Write_ADE7878_SPI(LINECYC, 2, LINECYC_cof); //写入LINECYC寄存器，设置线周期累计模式计数

  //设置电压电流峰值检测的半波周期数(检测n个半波周期内的电压电流峰值)
  //Write_ADE7878_SPI(PEAKCYC,1,MMODE_cof);	//峰值检测半波周期数
  //Write_ADE7878_SPI(PEAKCYC, 1, LINECYC_cof); //写入PEAKCYC寄存器，设置半波周期数与LINECYC周期数相同

  //设置中断屏蔽寄存器MASK0和MASK1
  //unsigned char MASK_cof[8] = {0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00}; // LENERGY位置1
  //Write_ADE7878_SPI(MASK0, 4, MASK_cof);                                        // MASK0使能线路累计模式下半波周期积分结束中断，IRQ0引脚中断时置低
  //Write_ADE7878_SPI(MASK1, 4, MASK_cof + 4);                                    // MASK1不使能任何中断

  //设置WTHR寄存器，用于计算有功功率
  //u8 WTHR_cof[8] = {0x00, 0xce, 0xe7, 0xc1, 0x00, 0x00, 0x00, 0x0d};//VFS=76.71A IFS=589.25V 0.01WH/LSB PMAX=33516139
	u8 WTHR_cof[8] = {0x00, 0x9C, 0xD3, 0x96, 0x00, 0x00, 0x00, 0x1B};//VFS=35.36A IFS=589.25V 0.01WH/LSB PMAX=33516139
  // u8 WTHR_cof[8]={0X00,0X00,0XB6,0x64,0x00,0x00,0x00,0x00};
  // u8 WTHR_cof[8]={0X00,0Xe0,0X70,0x4c,0x00,0x00,0x00,0x02};//YK修改 2016.4.5 0.01WH/LSB
  //u8 WTHR_cof[8] = {0x00, 0xce, 0x23, 0x7b, 0x00, 0x00, 0x00, 0x04}; // YK修改 2016.4.5 0.01WH/LSB	2016.7.31
  Write_ADE7878_SPI(WTHR0, 4, WTHR_cof);//向WTHR0寄存器写入低24位数值  PMAX = 33,516,139	  0.01WH/LSB
  Write_ADE7878_SPI(WTHR1, 4, WTHR_cof + 4);//向WTHR1寄存器写入高24位数值

  //设置VATHR寄存器，用于计算视在功率
  //VATHR寄存器计算公式与WTHR寄存器计算公式相同，使用WTHR_cof中的值
  Write_ADE7878_SPI(VATHR0, 4, WTHR_cof); // PMAX = 33,516,139
  Write_ADE7878_SPI(VATHR1, 4, WTHR_cof + 4);
  
  //设置VARTHR寄存器，用于计算无功功率
  //VARTHR寄存器计算公式与WTHR寄存器计算公式相同，使用WTHR_cof中的值
  Write_ADE7878_SPI(VARTHR0, 4, WTHR_cof); // PMAX = 33,516,139
  Write_ADE7878_SPI(VARTHR1, 4, WTHR_cof + 4);
  
  //设置空载阈值寄存器，配置空载监测功能
  //u8 APNOLOAD_cof[4] = {0x00, 0x00, 0x45, 0x21};//Vn=220 Inoload=0.1 PMAX=33516139
  //u8 APNOLOAD_cof[4]={0x00,0x00,0x00,0x00};
  //u8 APNOLOAD_cof[4] = {0x00, 0x02, 0x20, 0x60}; // YK修改 2016.4.5
  //u8 APNOLOAD_cof[3]={0x00,0x39,0x1c};	 //YK修改 2016.4.5
  //Write_ADE7878_SPI(APNOLOAD, 4, APNOLOAD_cof);//写入有功功率空载阈值寄存器APNOLOAD
  //Write_ADE7878_SPI(VANOLOAD, 4, APNOLOAD_cof);//写入视在功率空载阈值寄存器VANOLOAD
  //Write_ADE7878_SPI(VARNOLOAD, 4, APNOLOAD_cof);//写入无功功率空载阈值寄存器VARNOLOAD
  
  //配置CF引脚控制寄存器CFMODE
  //u8 CFMODE_cof[2]={0x10,0x00};
  //u8 CFMODE_cof[2] = {0x08, 0xc8}; // YK修改 2016.4.5 CF1总有功 CF2总无功 CF3禁能
  u8 CFMODE_cof[2] = {0x08, 0x88};//CF1总有功 CF2总无功 CF3总视在（但禁能输出）
  //u8 CFMODE_cof[2]={0x0e,0x88};  //YK修改 2016.4.6 CF1/CF2/CF3禁能
  Write_ADE7878_SPI(CFMODE, 2, CFMODE_cof);
  
  //根据电表常数配置CF引脚输出频率
  //u8 CF1DEN_cof[2] = {0x00, 0x53};//MC=1200impulses/kWh
  //u8 CF1DEN_cof[2]={0x3D,0x09};
  u8 CF1DEN_cof[2] = {0x00, 0x0a}; //MC=10000impulses/kWh
  Write_ADE7878_SPI(CF1DEN, 2, CF1DEN_cof);
  Write_ADE7878_SPI(CF2DEN, 2, CF1DEN_cof);
  Write_ADE7878_SPI(CF3DEN, 2, CF1DEN_cof);
  
  //设置CFCYC寄存器为默认值0x01
  unsigned char RUN_cof[2] = {0x00, 0x01};
  Write_ADE7878_SPI(CFCYC, 1, RUN_cof + 1);

  //设置高通滤波器以消除通道上的ADC直流失调
  unsigned char HPFDIS_cof[4] = {0x00, 0x00, 0x00, 0x00};//HPFDIS寄存器清除至全0，使能全部通道的HPF
  //向队列最后一个寄存器写入3次以确保写入RAM(Rev.H Page42)
  Write_ADE7878_SPI(HPFDIS, 4, HPFDIS_cof); // enabled high-pass filters
  Write_ADE7878_SPI(HPFDIS, 4, HPFDIS_cof); // enabled high-pass filters
  Write_ADE7878_SPI(HPFDIS, 4, HPFDIS_cof); // enabled high-pass filters
  
  //DSP RAM写保护
  Write_ADE7878_SPI(0xE7FE, 1, RAM);     //向0xE7FE写入0xAD
  Write_ADE7878_SPI(0xE7E3, 1, RAM + 2); //向0xE7E3写入0x80
  
  //向RUN寄存器写入0x0001，启动DSP
  Write_ADE7878_SPI(RUN, 2, RUN_cof);       //启动DSP
}

void ADE7878_CORRECTCONFIG(void)
{
  // 电流校准
  //    unsigned char CorrectReg_CIGAIN[4]={0x00,0x03,0x91,0x92};//YK修改 2016.3.22
  //	unsigned char CorrectReg_CIGAIN[4]={0x00,0x17,0x81,0x65};//YK修改 2016.5.29
  unsigned char CorrectReg_CIGAIN[4] = {0x00, 0x03, 0xa7, 0x32}; // YK修改 2016.6.15

  //	unsigned char CorrectReg_CVGAIN[4]={0x00,0x06,0x56,0xac};//YK修改 2016.6.17
  //	unsigned char CorrectReg_CVGAIN[4]={0x00,0x04,0x28,0x10};//YK修改 2016.6.17
  //	unsigned char CorrectReg_CVGAIN[4]={0x00,0x05,0xeb,0xbb};//YK修改 2016.8.4 2号板子
  //	unsigned char CorrectReg_CVGAIN[4]={0x00,0x08,0x6b,0xb6};//YK修改 2016.8.4 1号板子
  //	unsigned char CorrectReg_CVGAIN[4]={0x00,0x05,0x5c,0x86};//YK修改 2016.8.4 3号板子
  //  unsigned char CorrectReg_CVGAIN[4]={0x00,0x05,0x68,0x32};//YK修改 2016.8.4 5号板子
  //	unsigned char CorrectReg_CVGAIN[4]={0x00,0x07,0x85,0xa2};//YK修改 2016.8.4 6号板子
  //  unsigned char CorrectReg_CVGAIN[4]={0x00,0x04,0xe7,0x62};//YK修改 2016.8.4 7号板子
  unsigned char CorrectReg_CVGAIN[4] = {0x00, 0x05, 0xb5, 0x2d}; // YK修改 2016.8.4 4号板子
  //	unsigned char CorrectReg_CVGAIN[4]={0x00,0x04,0xd5,0x12};//YK修改 2016.8.4 8号板子

  unsigned char CorrectReg_CWGAIN[4] = {0xff, 0xff, 0x29, 0xb2}; // YK修改 2016.6.17
  //	unsigned char CorrectReg_APHCAL[2]={0x01,0x7f};//YK修改 2016.4.13
  //	unsigned char CorrectReg_CPHCAL[2]={0x00,0xbf};//YK修改 2016.4.13
  Write_ADE7878_SPI(CIGAIN, 4, CorrectReg_CIGAIN); //校准电流有效值
  Write_ADE7878_SPI(CVGAIN, 4, CorrectReg_CVGAIN); //校准电压有效值
  Write_ADE7878_SPI(CWGAIN, 4, CorrectReg_CWGAIN); //校准有功功率值
                                                   // Write_ADE7878_SPI(AVGAIN,4,CorrectReg_cof);
  //  Write_ADE7878_SPI(BIGAIN,4,CorrectReg_cof);
  //  Write_ADE7878_SPI(BVGAIN,4,CorrectReg_cof);
  //  Write_ADE7878_SPI(CIGAIN,4,CorrectReg_cof);
  //  Write_ADE7878_SPI(CVGAIN,4,CorrectReg_cof);
  //  Write_ADE7878_SPI(NIGAIN,4,CorrectReg_cof);
  //功率校准
  /* Write_ADE7878_SPI(AIRMSOS,4,CorrectReg_cof);		   //电压电流有效值校准
   Write_ADE7878_SPI(AVRMSOS,4,CorrectReg_cof);
   Write_ADE7878_SPI(BIRMSOS,4,CorrectReg_cof);
   Write_ADE7878_SPI(BVRMSOS,4,CorrectReg_cof);
   Write_ADE7878_SPI(CIRMSOS,4,CorrectReg_cof);
   Write_ADE7878_SPI(CVRMSOS,4,CorrectReg_cof);
   Write_ADE7878_SPI(NIRMSOS,4,CorrectReg_cof);

   Write_ADE7878_SPI(AWGAIN,4,CorrectReg_cof);	   //总有功功率输出波形调整
   Write_ADE7878_SPI(AWATTOS,4,CorrectReg_cof);	   //有功功率失调校准
   Write_ADE7878_SPI(BWGAIN,4,CorrectReg_cof);
   Write_ADE7878_SPI(BWATTOS,4,CorrectReg_cof);
   Write_ADE7878_SPI(CWGAIN,4,CorrectReg_cof);
   Write_ADE7878_SPI(CWATTOS,4,CorrectReg_cof);*/

  //  Write_ADE7878_SPI(CPHCAL,2,CorrectReg_CPHCAL);		//相位校准
  //  Write_ADE7878_SPI(BPHCAL,2,CorrectReg_cof);
  //  Write_ADE7878_SPI(CPHCAL,2,CorrectReg_cof);
}

void ADE7878_init(void)
{
  // SPI2_Init();
  // ADE7878_GPIOCONFIG();
  extern unsigned char chip_checksum[4];
  ADE7878_PSM0_Enable();
  HAL_Delay(40);
  ADE7878_SPI_Enable();
  printf("ADE7878 CHECKSUM register should be 0x33666787\n");
  Read_ADE7878_SPI(CHECKSUM, 4,chip_checksum);
  printf("ADE7878 CHECKSUM register[1]: %#x\n",chip_checksum[0]);
  printf("ADE7878 CHECKSUM register[2]: %#x\n",chip_checksum[1]);
  printf("ADE7878 CHECKSUM register[3]: %#x\n",chip_checksum[2]);
  printf("ADE7878 CHECKSUM register[4]: %#x\n",chip_checksum[3]);
  printf("Config ADE7878...");
  //ADE7878_CORRECTCONFIG();
  ADE7878_BASICCONFIG();
  printf("OK!\n");
}
