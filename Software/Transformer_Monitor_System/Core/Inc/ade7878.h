																	   
#ifndef __ADE7878_H
#define __ADE7878_H

#include "stm32f4xx_hal.h"
#include "main.h"
//the addresses of all ADE78xx registers
#define   AIGAIN            0x4380
#define   AVGAIN            0x4381
#define   BIGAIN            0x4382
#define   BVGAIN            0x4383
#define   CIGAIN            0x4384
#define   CVGAIN            0x4385
#define   NIGAIN            0x4386	   //零线电流增益调整
#define   AIRMSOS           0x4387
#define   AVRMSOS           0x4388
#define   BIRMSOS           0x4389
#define   BVRMSOS           0x438A
#define   CIRMSOS           0x438B
#define   CVRMSOS           0x438C
#define   NIRMSOS           0x438D	   //零线电流有效值偏移失调
#define   AVAGAIN           0X438E	   //XXX视在功率增益调整
#define   BVAGAIN           0X438F
#define   CVAGAIN           0X4390
#define   AWGAIN            0X4391	   //XXX总有功功率增益调整
#define   AWATTOS           0x4392	   //XXX总有功功率偏移失调补偿
#define   BWGAIN            0X4393
#define   BWATTOS           0x4394
#define   CWGAIN            0X4395
#define   CWATTOS           0x4396
#define   AVARGAIN          0X4397
#define   AVAROS            0x4398
#define   BVARGAIN  		0X4399
#define   BVAROS            0x439A
#define   CVARGAIN          0X439B
#define   CVAROS            0x439C
#define   AFWGAIN	        0X439D	      //XXX基波有功功率增益调整  
#define   AFWATTOS          0x439E		  //XXX基波有功功率偏移失调补偿
#define   BFWGAIN           0X439F
#define   BFWATTOS          0x43A0
#define   CFWGAIN           0X43A1
#define   CFWATTOS          0x43A2
#define   AFVARGAIN			0X43A3
#define   AFVAROS           0x43A4
#define   BFVARGAIN         0X43A5
#define   BFVAROS           0x43A6
#define   CFVARGAIN			0X43A7
#define   CFVAROS           0x43A8
#define   VATHR1            0x43A9			//相位视在功率数据路径中所用VATHR[47:0]阈值的24个最高有效位
#define   VATHR0            0x43AA			//相位视在功率数据路径中所用VATHR[47:0]阈值的24个最低有效位
#define   WTHR1             0x43AB			//相位总/基波有功功率数据路径中所用WTHR[47:0]阈值的24个最高有效位
#define   WTHR0             0x43AC			//相位总/基波有功功率数据路径中所用WTHR[47:0]阈值的24个最低有效位
#define   VARTHR1           0x43AD			//相位总/基波无功功率数据路径中所用VARTHR[47:0]阈值的24个最高有效位
#define   VARTHR0           0x43AE			//相位总/基波无功功率数据路径中所用VARTHR[47:0]阈值的24个最低有效位

#define   VANOLOAD          0x43B0			//视在功率数据路径中的空载阈值
#define   APNOLOAD          0x43B1			//总/基波有功功率数据路径中的空载阈值
#define   VARNOLOAD         0x43B2			//总/基波无功功率数据路径中的空载阈值
#define   VLEVEL            0x43B3			//基波有功/无功功率算法中使用的寄存器
#define   DICOEFF           0x43B5   		//数字积分器算法中使用的寄存器。如果积器处于开启状态，则必须设为0xFF8000
#define   HPFDIS		    0X43B6		    //禁用/使能电流数据路径中的HPF
#define   ISUMLVL           0x43B8			//比较相电流之和和零线电流时使用的阈值
#define   ISUM              0x43BF
#define   AIRMS             0x43C0			  //XX电流有效值
#define   AVRMS             0x43C1
#define   BIRMS             0x43C2
#define   BVRMS             0x43C3			   //XX电压有效值
#define   CIRMS             0x43C4
#define   CVRMS             0x43C5
#define   NIRMS             0x43C6

#define   RUN               0xE228			   //Run寄存器负责启停DSP
//Billable Registers
#define   AWATTHR           0xE400			   
#define   BWATTHR           0xE401			   //XX总有功电能累计
#define   CWATTHR           0xE402
#define   AFWATTHR          0xE403
#define   BFWATTHR          0xE404			   //XX基波有功电能累计
#define   CFWATTHR          0xE405
#define   AVARHR            0xE406
#define   BVARHR            0xE407				//XX总无功电能累计
#define   CVARHR            0xE408
#define   AFVARHR           0xE409
#define   BFVARHR           0xE40A				//XX基波无功电能累计
#define   CFVARHR           0xE40B
#define   AVAHR             0xE40C			    //视在电能累计
#define   BVAHR             0xE40D
#define   CVAHR             0xE40E

//Configuration and Power Quality Resisiters
#define   IPEAK             0xE500				 //Current peak寄存器
#define   VPEAK             0xE501				 //Voltage peak寄存器
#define   STATUS0           0xE502				 //interrupt status寄存器0
#define   STATUS1           0xE503				 //interrupt status寄存器1
#define   AIMAV             0xE504				 
#define   BIMAV             0xE505				 //PSM0和PSM1模式下计算出的XX电流平均绝对值
#define   CIMAV             0xE506
#define   OILVL             0xE507				 //过流阈值
#define   OVLVL             0xE508				 //过压阈值
#define   SAGLVL            0xE509				 //电压SAG电平阈值
#define   MASK0             0xE50A				 //interrupt enable寄存器0
#define   MASK1             0xE50B				 //interrupt enable寄存器1
#define   IAWV              0xE50C
#define   IBWV              0xE50D				 //XX电流的瞬时值
#define   ICWV              0xE50E
#define   INWV              0xE50F				 //零线电流的瞬时值
#define   VAWV              0xE510
#define   VBWV              0xE511				  //XX电压的瞬时值
#define   VCWV              0xE512
#define   AWATT             0xE513
#define   BWATT             0xE514				   //XX总有功功率的瞬时值
#define   CWATT             0xE515
#define   AVAR              0xE516
#define   BVAR              0xE517				   //XX总无功功率的瞬时值
#define   CVAR              0xE518
#define   AVA               0xE519
#define   BVA               0xE51A				   //XX视在功率的瞬时值
#define   CVA               0xE51B
#define   CHECKSUM          0xE51F				   //检验和验证
#define   VNOM              0xE520				   //视在功率替代计算中使用的标称相电压有效值；这些地址不应写入数据，以保正常工作

//16bits resisiter
#define   PHSTATUS          0xE600				   //phase peak寄存器
#define   ANGLE0            0xE601
#define   ANGLE1            0xE602				   //时间延迟x
#define   ANGLE2            0xE603
#define   PERIOD            0xE607				   //网络线周期
#define   PHNOLOAD          0xE608				   //Phase Noload寄存器
#define   LINECYC           0xE60C				   //线周期累计模式技术
#define   ZXTOUT            0xE60D				   //过零超时计数
#define   COMPMODE          0xE60E				   //Computation-mode寄存器
#define   GAIN              0xE60F				   //增益，ADC输入端的PGA增益
#define   CFMODE            0xE610				   //CFx conguration寄存器
#define   CF1DEN            0xE611
#define   CF2DEN            0xE612				   //CFx分母
#define   CF3DEN            0xE613
#define   APHCAL            0xE614
#define   BPHCAL            0xE615				   //xx相位校准
#define   CPHCAL            0xE616
#define   PHSIGN            0xE617				   //Power sign寄存器
#define   CONFIG            0xE618				   //ADE7878 configuration寄存器

//8bits reister
#define   MMODE             0xE700				   //Measurement mode寄存器
#define   ACCMODE           0xE701				   //Accumulation mode寄存器
#define   LCYCMODE          0xE702				   //线路累计模式状态
#define   PEAKCYC           0xE703				   //峰值检测半波周期数
#define   SAGCYC            0xE704				   //SAG检测半波周期数
#define   CFCYC             0xE705				   //两次连续电能锁存之间的CF脉冲数
#define   HSDC_CFG          0xE706				   //HSDC conguration寄存器
#define   VERSION           0xE707                 //ADE7878芯片版本
#define   LPOILVL           0xEC00				   //PSM2模式下使用的过流阈值
#define   CONFIG2           0xEC01				   //PSM1模式下使用的configuration寄存器

#define DataBufferSize    20
#define ADE_PM0_0    GPIO_ResetBits(GPIOD,GPIO_Pin_15)    
#define ADE_PM0_1    GPIO_SetBits(GPIOD,GPIO_Pin_15)
#define ADE_PM1_0  	 GPIO_ResetBits(GPIOD,GPIO_Pin_14)	  
#define ADE_PM1_1    GPIO_SetBits(GPIOD,GPIO_Pin_14)
#define ADE_RESET_0  HAL_GPIO_WritePin(GPIOF, RESET_7878_Pin, GPIO_PIN_RESET)	  
#define ADE_RESET_1  HAL_GPIO_WritePin(GPIOF, RESET_7878_Pin, GPIO_PIN_SET)
#define ADE_CS_0	 HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET)	  
#define ADE_CS_1	 HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET)

#define AlarmV 0x30
#define AlarmI 0X31
#define AlarmS 0x32 

void ade7878_delay (u32 nCount);
void ADE7878_init(void);
void Write_ADE7878_SPI(unsigned int ADE_Addr,unsigned char Nr_Bytes,unsigned char *pReg_Data);
void Read_ADE7878_SPI(unsigned int ADE_Addr, unsigned char Nr_Bytes,unsigned char *pReg_Data);

#endif


















