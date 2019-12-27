//==========================================================================================================
//                     F28335系统主程序及相关功能---三相查表法双极性SPWM方波并网程序

//----------------------------------------------------------------------------------------------------------
// 作者：孔杰 QQ:190459556
// 日期：2018.12
// 功能： 1、三相查表法双极性SPWM方波并网程序

// F28335片内RAM34KB，FLASH是256KB
//==========================================================================================================

/***********************************************************************************************************
 *                                          如何设置一个新项目CCSV4.x
 * 1、搭建此项目的目录结构，将需要的文件复制到相应目录
 * 
 * 2、设置include路径Include Options:
 * "${PROJECT_ROOT}/include"
 * 
 * 3、basic Options:
 * 0x200   --针对F28027
 * 0x380   --针对F28035
 * 0x380   --针对F28335
***********************************************************************************************************/

#include "DSP28x_Project.h"
#include "Spwm.h"
#include "SysInit.h"
#include "LED.h"
#include "AT24C02.h"
#include "SCI_Uart.h"
#include <string.h>

//----------------------------------------------------------------------------------------------------------
//ADC定义
#define ADC_CKPS   0x1   												// ADC模块时钟 = HSPCLK/2*ADC_CKPS  = 30.0MHz/(1*2) = 15MHz; = 20.0MHz/(1*2) = 10MHz
#define ADC_SHCLK  0xf   												// S/H width in ADC module periods = 16 ADC clocks
//分别是：输入电流，输入电压，BUS电压，逆变输出电流，逆变输出电压
Uint16 Id_REG[128], Ud_REG[128], Uf_REG[128], Uoa_REG[128],Uob_REG[128], Uoc_REG[128];
Uint16 Ioa_REG[128],Iob_REG[128],Ioc_REG[128],Vr1_REG[128],Ub_REG[128],A32_REG[128],A33_REG[128];
Uint16 Uw_REG[128],Uv_REG[128], Uu_REG[128];
//Uint32 Id,Ud,Uf,Uoa,Uob,Uoc;											//对应上面参数
float32 Id,Ud,Uf,Uoa,Uob,Uoc;
float32 Ioa,Iob,Ioc,Vr1,Ub,ADC;
//Uint32 Ioa,Iob,Ioc,Vr1,Ub,ADC;
Uint32 Sum,Max,Min;														//ADC采样求和值，采样最大值，采样最小值
Uint16 ConversionCount;													//ADC采样计数器
Uint16 ADC_Flag,i,j;													//ADC采样标志,i,j循环变量
Uint16 MPPT_Flag;														//DCDC MPPT标志
void ADCInit(void);
interrupt void adc_isr(void);

//----------------------------------------------------------------------------------------------------------
//DCDC
Uint16 DCDC_N;															//DCDC占空比调节变量
Uint16 DCDC_T;															//DCDC周期变量
Uint16 DCDC_LED;														//DCDC占空比左死区变量
Uint16 DCDC_RED;														//DCDC占空比右死区变量
Uint16 DC_PWM,pwm_step;													//MPPT后的占空比,pwm步进值
void InitSPwm4(void);
interrupt void epwm4_isr(void);
//DCDC PID
//float32 startpid,pmin,setmid,pmax,fastv;								//DCDC PID参数分别是：启动PID的闸阀值，稳压范围的最小值，稳压值，稳压范围的最大值，快速调节值
//Uint16 Is_UbPID = 0;													//是否开启DCDC闭环PID 1-开启
//Uint16 PWMD;															//PWM调节幅度
//void Init_Ub_PID(void);
//void PWM_Ub_PID(void);

//----------------------------------------------------------------------------------------------------------
//DCAC
void InitEPwmTZ(void);
void InitSPwm1(void);
void InitSPwm2(void);
void InitSPwm3(void);
interrupt void epwm1_isr(void);
interrupt void epwm2_isr(void);
interrupt void epwm3_isr(void);
//DCAC PID
//增量式PID控制DCAC,PID调节变量,并对变量进行初始化
float32 startpids,pmins,setmids,pmaxs,fastvs;							//SPWM1 PID参数分别是：启动PID的闸阀值，稳压范围的最小值，稳压值，稳压范围的最大值，快速调节值
Uint16 PWMDs;															//SPWM调节幅度
Uint16 Is_UoPID = 0;													//是否开启DCAC闭环PID 1-开启
void Init_Uo_PID(void);
void PWM_Uo_PID(void);

//----------------------------------------------------------------------------------------------------------
//ECAP
Uint32 nCAP1,nCAP2,nCAP3;												//DSP捕获
float32 Freq,Phase;														//频率,相位
Uint16 Freq_Lock=0;														//是否锁频 1-锁相
Uint16 Phase_Lock=0;													//是否锁相 1-锁相
Uint16 Freq_Flag1,Freq_Flag2,Freq_Flag3;								//频率标志
void Lock_Freq(void);
void InitECapture1(void);
void InitECapture2(void);
void InitECapture3(void);
interrupt void ecap1_isr(void);
interrupt void ecap2_isr(void);
interrupt void ecap3_isr(void);

//----------------------------------------------------------------------------------------------------------
//SCI
//#define COMM_ID		0xA2					//串口命令识别地址
#define COMM_ID			0xEE					//串口命令识别地址
#define ID_I			0						//串口命令第0位
#define CMD_I			1						//串口命令第1位
#define ADDR_I  		2						//串口命令第2位
#define DATA_I  		3						//串口命令第3位
#define DATB_I  		4						//串口命令第4位
Uint16 send_timer=0;							//LCD数据发送时间
Uint16 timerNum=100;							//发送间隔变量
Uart_Msg SCI_Msg={0, {0},0,0, 0};
unsigned char HandleCommAG(void);
void sendTFT(float32 U,Uint16 SID,Uint16 ID);
void getTFT(Uint16 SID,Uint16 ID);

//----------------------------------------------------------------------------------------------------------
//SYS
Uint16 DisType=0;								//显示设置，0-LCD12864 1-TFT 2-NoDisply
Uint16 DisTypeData=0;							//显示数据类型，0-原始采样值  1-乘以变比值

Uint16 IsProt=0;								//保护状态 1-使能保护
Uint16 START_Flag=1;
void Delay(Uint16 i);
Uint16 KeyScan(void);
void itoa(int n,char s[]);
void float_TO_ascii(float a, char dat[8]);
void StopRun(void);

//----------------------------------------------------------------------------------------------------------
//SPI数码管显示参数
Uint16 Show_Flag,Show_Type,Show_Count;
Uint16 LedBuffer[4];													//数码管显示的位
Uint16 showdata;														//数码管显示的值
Uint16 sdata;  															//向数码管发送的数据
void spi_xmit(Uint16 a);
void spi_fifo_init(void);
void spi_init(void);

//----------------------------------------------------------------------------------------------------------
//timer0
void timer0_init(void);
interrupt void cpu_timer0_isr(void);

//----------------------------------------------------------------------------------------------------------
//AT2408
Uint16 addr = 0;
Uint16 RecvBuf[16]={0};
Uint16 TranBuf[16]={0,1,2,3,4,5,6,7,8,9,0xA,0xB,0xC,0xD,0xE,0xF};
void ReadEEPROM(void);
void WriteEEPROM(void);

void main(void)
{
	//1、初始化系统
	InitSysCtrl();
	GPIOInit();									//初始化IO口

	//2、初始化GPIO
	InitEPwm1Gpio();							//PWM1 IO初始化，DCAC使用
	InitEPwm2Gpio();							//PWM2 IO初始化，DCAC使用
	InitEPwm3Gpio();							//PWM3 IO初始化，DCAC使用
	InitEPwm4Gpio();							//PWM4 IO初始化，DCDC使用
	InitECap1Gpio();							//Cap1 IO初始化，CAP1使用
	InitECap2Gpio();							//Cap2 IO初始化，CAP2使用
	InitECap3Gpio();							//Cap3 IO初始化，CAP3使用
	InitSpiaGpio();  							//SPIA初始化，数码管使用

	//3、清除所有中断，并禁用CPU中断
	DINT;

	//PIE初始化
	InitPieCtrl();

	//禁用CPU中断和清除所有CPU中断
	IER = 0x0000;
	IFR = 0x0000;

	//PIE向量表初始化
	InitPieVectTable();

///*
//-------------------------------------------------------------------------------------------------------- 
// 烧写FLASH更换为F28335.cmd文件，添加下面2行代码,并添加DSP2833x_MeMCopy.c,DSP280x_CSMPasswords.asm,F28335.cmd文件，重新编译
//--------------------------------------------------------------------------------------------------------  
MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
//上述两句话添加在InitPieVectTable();这句的下面的一行。 
//-------------------------------------------------------------------------------------------------------- 
//*/

	//申明中断
	EALLOW;

	PieVectTable.EPWM1_INT = &epwm1_isr;		//PWM1中断
	PieVectTable.EPWM2_INT = &epwm2_isr;		//PWM2中断
	PieVectTable.EPWM3_INT = &epwm3_isr;		//PWM3中断
	PieVectTable.EPWM4_INT = &epwm4_isr;		//PWM4中断
	PieVectTable.ECAP1_INT = &ecap1_isr;		//CAP1中断
	PieVectTable.ECAP2_INT = &ecap2_isr;		//CAP2中断
	PieVectTable.ECAP3_INT = &ecap3_isr;		//CAP3中断
	PieVectTable.ADCINT = &adc_isr;				//ADC中断

	EDIS;

	//4、初始化所有的外设
	InitAdc();									//初始化ADC

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;		//外设时钟控制寄存器0
	EDIS;

	DCDC_N=95;									//=5%,将占空比分成100份，每份21，DCDC_N越大占空比越小
	DCDC_T=2130;								//35.2KHz 30MHz=2130 T=28.4us
	DCDC_LED=60;								//初始死区=60时3.4us
	DCDC_RED=60;								//初始死区
	DC_PWM=21*DCDC_N;
	InitSPwm4();								//PWM4功能初始化

	M=0.95;										//调制比
	N=400;										//载波比400
	fs=50;										//50Hz
	Calc_Spwm();								//计算PWM正弦表

	SPWMCntA=0;  
	SPWMCntB=133;  
	SPWMCntC=266;  
	//驱动频率=F/(CarrVal/2)/1000 kHz
	PWMHz=(F/2000)/CarrVal;


	InitEPwmTZ();
	InitSPwm1();								//PWM1功能初始化
	InitSPwm2();								//PWM2功能初始化
	InitSPwm3();								//PWM3功能初始化
	InitECapture1();							//Cap1功能初始化
	InitECapture2();							//Cap2功能初始化
	InitECapture3();							//Cap3功能初始化

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	//5、用户特定的代码，允许中断

	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;			//使能ADC中断
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;			//使能PWM1中断
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;			//使能PWM2中断
	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;			//使能PWM3中断
	PieCtrlRegs.PIEIER3.bit.INTx4 = 1;			//使能PWM4中断
	PieCtrlRegs.PIEIER4.bit.INTx1 = 1;			//使能CAP1中断
	PieCtrlRegs.PIEIER4.bit.INTx2 = 1;			//使能CAP2中断
	PieCtrlRegs.PIEIER4.bit.INTx3 = 1;			//使能CAP3中断

	IER |= M_INT1+M_INT3+M_INT4;				//使能CPU中断

	timer0_init();

	//6、初始化用户参数
	Freq_Flag1=0;
	Freq_Flag2=0;
	Freq_Flag3=0;

	Freq=0;										//频率

	nCAP1=0;
	nCAP2=0;
	nCAP3=0;

	Id=0;Ud=0;Uf=0;Uoa=0;Uob=0;Uoc=0;
	Ioa=0;Iob=0;Ioc=0;Vr1=0;Ub=0;ADC=0;

	ADC_Flag=0;
	ConversionCount = 0;
	MPPT_Flag=1;
	ADCInit();									//ADC功能初始化

	timerNum=100;

	//BOOST PWM PID初始化
	//Init_Ub_PID();
	//DCAC PWM PID初始化
	Init_Uo_PID();

	Eerom_Gpio_Init();							// Initalize Eeprom 

	scic_init();  								// Initalize SCI for echoback

	Show_Flag=1;
	Show_Type=0;
	Show_Count=0;

	spi_fifo_init();							//初始化Spi的FIFO
	spi_init();		  							//初始化SPI
	//关数码管
	spi_xmit(0x0000);

	mRun_OFF();
	mErr_OFF();

	mPWM_OFF();									//禁止PWM3 DCDC
	mSPWM_OFF();								//禁止PWM1 SPWM
	//全能全局中断
	EINT;										// 全能全局中断 INTM
	ERTM;										// 全能全局实时中断 DBGM

	//7、 IDLE 循环
	for(;;)
	{
		asm(" NOP");

		if(ADC_Flag==1)
		{
			//---------------------------------ADC采样完毕，随时可以处理----------------------------------
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Ioa_REG[i];
	
				if(Ioa_REG[i]<Min) 
					Min=Ioa_REG[i];
				else if(Ioa_REG[i]>Max) 
					Max=Ioa_REG[i];
			}
			Ioa=Max;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Iob_REG[i];
	
				if(Iob_REG[i]<Min) 
					Min=Iob_REG[i];
				else if(Iob_REG[i]>Max) 
					Max=Iob_REG[i];
			}
			Iob=Max;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Ioc_REG[i];
	
				if(Ioc_REG[i]<Min) 
					Min=Ioc_REG[i];
				else if(Ioc_REG[i]>Max) 
					Max=Ioc_REG[i];
			}
			Ioc=Max;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Vr1_REG[i];
	
				if(Vr1_REG[i]<Min) 
					Min=Vr1_REG[i];
				else if(Vr1_REG[i]>Max) 
					Max=Vr1_REG[i];
			}
			Vr1=(Sum-Max-Min)/126.0;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Uf_REG[i];
	
				if(Uf_REG[i]<Min) 
					Min=Uf_REG[i];
				else if(Uf_REG[i]>Max) 
					Max=Uf_REG[i];
			}
			Uf=(Sum-Max-Min)/126.0;						
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Ub_REG[i];
	
				if(Ub_REG[i]<Min) 
					Min=Ub_REG[i];
				else if(Ub_REG[i]>Max) 
					Max=Ub_REG[i];
			}
			Ub=(Sum-Max-Min)/126.0/4096.0*3.0;
			Ub=300.48*Ub;

			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Id_REG[i];
	
				if(Id_REG[i]<Min) 
					Min=Id_REG[i];
				else if(Id_REG[i]>Max) 
					Max=Id_REG[i];
			}
			Id=(Sum-Max-Min)/126.0/4096.0*3.0*500;								//Id输入电流采样,去掉最大值和最小值
			Id=Id/97.85;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Ud_REG[i];
	
				if(Ud_REG[i]<Min) 
					Min=Ud_REG[i];
				else if(Ud_REG[i]>Max) 
					Max=Ud_REG[i];
			}
			Ud=(Sum-Max-Min)/126.0/4096.0;								//Ud输入电压采样
			Ud=Ud*200.6*3.0;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Uoa_REG[i];
	
				if(Uoa_REG[i]<Min) 
					Min=Uoa_REG[i];
				else if(Uoa_REG[i]>Max) 
					Max=Uoa_REG[i];
			}
			Uoa=Max/4096.0*3;
			Uoa=(Uoa-1.5)*384.61;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Uob_REG[i];
	
				if(Uob_REG[i]<Min) 
					Min=Uob_REG[i];
				else if(Uob_REG[i]>Max) 
					Max=Uob_REG[i];
			}
			Uob=Max;
	
			Sum=0;Max=0;Min=0xffff;
			for(i=0;i<128;i++)
			{
				Sum+=Uoc_REG[i];
	
				if(Uoc_REG[i]<Min) 
					Min=Uoc_REG[i];
				else if(Uoc_REG[i]>Max) 
					Max=Uoc_REG[i];
			}
			Uoc=Max;
	
			ConversionCount = 0;
			ADC_Flag=0;
		}

		//===================================ADC采样数据处理 end=======================================
		if(START_Flag==1)
		{
			START_Flag=0;

			//开启PWM4 DCDC
			Delay(500);
			mPWM_ON();
			DCDC_N=95;

			//开启PWM1 SPWM
			Delay(500);
			mSPWM_ON();

			IsProt=0;
			DisType=1;
			Is_UoPID=0;
			send_timer=0;
			timerNum=50;

			//当系统保护后执行一下复位保护
			mOC_RESET_PRO_L();

			//读取设置数据
			ReadEEPROM();
			Delay(500);
			if(RecvBuf[2]!=0xFF)
			{
				fs=RecvBuf[2];
				//Calc_Spwm();								//fs发生变化就重新计算SPWM
				//EPwm1Regs.TBPRD = CarrVal;				//修改fs正弦波输出频率
				//EPwm2Regs.TBPRD = CarrVal;
				//EPwm3Regs.TBPRD = CarrVal;

				sendTFT(fs,0x02,0x07);
			}

			JDQ1_ON();
		}

		if(KEY1==1)
		{
			mSPWM_ON();
		}
		else
		{
			mSPWMA_ERR();
		}


		//PWM_Ub_PID();										//DCDC PID
		if(Is_UoPID==1) PWM_Uo_PID();						//使用DCAC PID时，开发板上的继电器应该处于吸合状态，但不能接电网电压
/*
		//KEY1按下手动保护使能
		if(KeyScan()==1)
		{
			IsProt=2;										//保护开启成功
			//保护使能
			mRun_ON();
			mOC_RESET_PRO_L();
			Delay(5000);
			mOC_RESET_PRO_H();
			mErr_OFF();
		}
*/
		if(IsProt==1)
		{
			IsProt=2;										//保护开启成功
			//保护复位使能
			mRun_ON();
			mOC_RESET_PRO_L();
			Delay(5000);
			mOC_RESET_PRO_H();
			mErr_OFF();
		}

		if(IsProt==2)
		{
			//PWM1 IGBT故障
			if(mERR==0)
			{
				mErr_ON();
				//TZ处理代码
			}
		}

/*		if(DisType==0)
		{
			if(send_timer>timerNum)
			{
				//Modbus协议发送数据
				//注：串口助手要设置成以16进制显示
				//send_data[1]        = BusV()>>8;             	//BUS.V H8高8位数
				//send_data[2]        = BusV()&0xFF;           	//BUS.V L8低8位数，还原方法：(高8位<<8)|低8位
				send_data[1]=Ud>>8;
				send_data[2]=Ud&0xFF;
				send_data[3]=Id>>8;
				send_data[4]=Id&0xFF;
				send_data[5]=Uoa>>8;
				send_data[6]=Uoa&0xFF;
				send_data[7]=Uob>>8;
				send_data[8]=Uob&0xFF;
				send_data[9]=Uoc>>8;
				send_data[10]=Uoc&0xFF;
				send_data[11]=Ioa>>8;
				send_data[12]=Ioa&0xFF;
				send_data[13]=0;
				send_data[14]=0;
				send_msg(send_data);
				send_timer=0;
			}
			else
			{
				send_timer++;
			}
		}


		*/
		if(DisType==1)
		{
			if(send_timer==timerNum)
			{
				sendTFT(Ud,0x01,0x01);
			}
			if(send_timer==timerNum*2)
			{
				sendTFT(Id,0x01,0x02);
			}
			if(send_timer==timerNum*3)
			{
				sendTFT(Ub,0x01,0x03);
			}
	
			if(send_timer==timerNum*4)
			{
				sendTFT(Uoa,0x01,0x04);
			}
			if(send_timer==timerNum*5)
			{
				sendTFT(Ioa,0x01,0x05);
			}
			if(send_timer==timerNum*6)
			{
				sendTFT(0.0,0x01,0x06);
			}
	
			if(send_timer==timerNum*7)
			{
				sendTFT(Uf,0x01,0x07);
			}
			if(send_timer==timerNum*8)
			{
				sendTFT(Vr1,0x01,0x08);
			}
			if(send_timer==timerNum*9)
			{
				sendTFT(PWMHz,0x01,0x09);
			}
	
			if(send_timer==timerNum*10)
			{
				sendTFT(Freq,0x01,0x0A);
				send_timer=0;
			}
			else
			{
				send_timer++;
			}
		}
/*
		//显示数据
		Show_Count++;										//用于调整SPI数码管显示参数类型的变换时间
		if(Show_Count==15)Show_Type++;						//变换时间大概6s
		if(Show_Flag==1)
		{
		    if(Show_Type==1) showdata=Ud;
		    //if(Show_Type==2) showdata=Id;
		    //if(Show_Type==3) showdata=Uoa;
			//if(Show_Type==4) showdata=Uob;
			//if(Show_Type==5) showdata=Uoc;
		    //if(Show_Type==6) showdata=Io;

			//将需要显示的数值送入A ==> 查表求得显示段码==>将段码逐位移入164==>8位移完后点亮数码管==>延时==>返回第一步执行
			//数值必须是16位的数值送入
			CharDisplay(showdata,LedBuffer);
			//先放最高16位数值，由于本开发板是三位数码管，所以最高位是一位数值
			//sdata=LedBuffer[2];
			//先放最高16位数值，这是四位数码管
			//sdata=(LedBuffer[3]<<8)+0x80;					//带小数点
			sdata=(LedBuffer[3]<<8)+LedBuffer[2];
			spi_xmit(sdata);
			//再放低16位数值
			sdata=(LedBuffer[1]<<8)+LedBuffer[0];
			spi_xmit(sdata);
			Delay(50000);
		}
		if(Show_Type>6)Show_Type=0;							//显示6个参数
		if(Show_Count>60)Show_Count=1;
*/
		//等待串口调试助手发送 操作命令
		//SCI_Msg.timerOut在定时器timer0 中
 		if(SCI_Msg.timerOut >= RTU_TIMEROUT)
		{
			SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag = 0;
			SCI_Msg.timerOut = 0;
			SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 1;

			//串口命令
			HandleCommAG();

			SCI_Msg.Mark_Para.Status_Bits.DISRevflag = 0;
		}
	}
} 

void ReadEEPROM(void)
{
	//读Eeprom
	for(addr = 0;addr<=0xf;addr++)
	{
		RecvBuf[addr] = readbyte(addr);	
		delay(500);
	}
}

void WriteEEPROM(void)
{
	//AT24C08 写EEPROM操作
	for(addr = 0;addr<=0xf;addr++)
	{
		writebyte(addr,TranBuf[addr]);
		delay(50000); 				// 间隔时间一定要够长，不然写EEPROM不会成功
	}
}

//---------------------------------------------------------------------
//操作串口命令协议，SCI串口命令参数BaudRate=19200
//#define COMM_ID		0xA2		//串口命令识别地址
//#define ID_I			0			//串口命令第0位
//#define CMD_I			1			//串口命令第1位
//#define ADDR_I  		2			//串口命令第2位
//#define DATA_I  		3			//串口命令第3位
//#define DATB_I  		4			//串口命令第4位
//---------------------------------------------------------------------
unsigned char HandleCommAG()
{
	char str[10];										//char str[10];定义不能放到main()前的全局定义上，否则串口命令不能正常运行，原因不明
	char s1[]="Enter the command is not correct!\r\n";
	char s2[]="The command format is:A2 00 00\r\n";
	char s3[]="ReStart Power!\r\n";
	char s4[]="Setup OK!\r\n";
	char pa1[]="->Ud=";
	char pa2[]="->Id=";
	char pa3[]="->Ub=";
	char pa4[]="->Uoa=";
	char pa5[]="->Vr=";
	char pa6[]="->Ioa=";
	int len;
	unsigned char addr,getdata,getdatb,getdatc;
	//s1字符串输出到串口调试助手上，如果以16进制查看都是ascii码
	if(SCI_Msg.rxData[ID_I] != COMM_ID)					//ID_I=0   COMM_ID=0xEE
	{
		SCI_Msg.rxWriteIndex = 0;
		SCI_Msg.rxReadIndex = 0;
		//如果命令不正确就直接返回也可在此写命令不正确要执行的代码
		len=strlen(s1);									//strlen #include <string.h>
		send_msgc(s1,len);
		return 1;
	}
	//解析操作
	switch(SCI_Msg.rxData[CMD_I])						//CMD_I=1表示串口接收到的数据rxData中的数组指针位置
	{
		//获得帮助，命令示例：
		//EE  功能    地址  数据
		//EE  00   00
		case 0: 
			addr =  SCI_Msg.rxData[ADDR_I];				//ADDR_I=2 串口命令第2位
			//getdata = SCI_Msg.rxData[DATA_I];			//DATA_I=3 得到调试模式值，必须是0x00-0xFF
			if(addr==0)
			{
				len=strlen(s2);
				send_msgc(s2,len);
			}
			break;
		//系统重新启动，和系统设置
		//EE  01   00
		case 1: 
			addr =  SCI_Msg.rxData[ADDR_I];				//ADDR_I=2 串口命令第2位
			if(addr==0)
			{
				//系统重启
				START_Flag=1;
				len=strlen(s3);
				send_msgc(s3,len);
			}
			if(addr==1)
			{
				//LCD12864显示
				DisType=0;
				len=strlen(s4);
				send_msgc(s4,len);
			}
			if(addr==2)
			{
				//TFTLCD显示
				DisType=1;
				len=strlen(s4);
				send_msgc(s4,len);
			}
			if(addr==3)
			{
				//取消所有显示
				DisType=2;
				len=strlen(s4);
				send_msgc(s4,len);
			}
			if(addr==4)
			{
				//显示原始采样值
				DisTypeData=0;
				len=strlen(s4);
				send_msgc(s4,len);
			}
			if(addr==5)
			{
				//显示乘以变比值
				DisTypeData=1;
				len=strlen(s4);
				send_msgc(s4,len);
			}
			break;
		//手动保护使能 
		//EE  02   00
		case 2: 
			addr =  SCI_Msg.rxData[ADDR_I];				//ADDR_I=2 串口命令第2位
			if(addr==0)
			{
				//保护使能
				IsProt=1;
			}
			if(addr==1)
			{
				//并网拉载电源使用，增加占空比
				DC_PWM=DC_PWM-75;
				//限幅，限制占空比
			   	if(DC_PWM<95) DC_PWM=95;
			   	if(DC_PWM>1995) DC_PWM=1995;
				sendTFT(DC_PWM,0x02,0x07);
			}
			if(addr==2)
			{
				//并网拉载电源使用，减小占空比
				DC_PWM=DC_PWM+75;
				//限幅，限制占空比
			   	if(DC_PWM<95) DC_PWM=95;
			   	if(DC_PWM>1995) DC_PWM=1995;
				sendTFT(DC_PWM,0x02,0x07);
			}
			if(addr==3)
			{
				//停止系统运行
				StopRun();
			}
			if(addr==4)
			{
				//开始锁相
				Phase_Lock=1;
			}
			len=strlen(s4);
			send_msgc(s4,len);
			break;
		//获得TFT数据操作
		//ASCII(., 0, 1, 2, 3, 4, 5, 6, 7, 8,9)
		//=HEX(2E,30,31,32,33,34,35,36,37,38,39)
		//=10D(46,48,49,50,51,52,53,54,55,56,57)
		//三种情况TFT的文本控件07的数为0 10 100时
		//EE B1 11 00 02 00 07 11 [30 00] FF FC FF FF 
		//EE B1 11 00 02 00 07 11 [31 30 00] FF FC FF FF 
		//EE B1 11 00 02 00 07 11 [31 30 30 00] FF FC FF FF 
		case 0xB1: 
			addr =  SCI_Msg.rxData[6];
			if(addr==7)
			{
				if(SCI_Msg.rxData[9]==0)
				{
					//由于rxData是16进制数-0x30变成10进制数
					getdata = SCI_Msg.rxData[8]-0x30;
					TranBuf[2]=getdata;
				}
				else if(SCI_Msg.rxData[10]==0)
				{
					//合并[31 32为一个10进制数
					getdata = SCI_Msg.rxData[8]-0x30;
					getdatb = SCI_Msg.rxData[9]-0x30;
					TranBuf[2]=getdata*10+getdatb;
				}
				else if(SCI_Msg.rxData[11]==0)
				{
					getdata = SCI_Msg.rxData[8]-0x30;
					getdatb = SCI_Msg.rxData[9]-0x30;
					getdatc = SCI_Msg.rxData[10]-0x30;
					TranBuf[2]=getdata*100+getdatb*10+getdatc;
				}
		    	WriteEEPROM();
			}
			len=strlen(s4);
			send_msgc(s4,len);
			break;
		//配置数据操作
		//EE  03   00
		case 3: 
			addr =  SCI_Msg.rxData[ADDR_I];				//ADDR_I=2 串口命令第2位
			if(addr==0)
			{
				//EE B1 11 00 02 00 07 FF FC FF FF ;
				getTFT(0x2,0x7);
			}
			if(addr==1)
			{
				//读取数据
				ReadEEPROM();
				sendTFT(RecvBuf[2],0x02,0x07);
			}
			if(addr==2)
			{
				//修改相位
				phaseV=phaseV+1;
				//限幅，限制占空比
			   	if(phaseV<1) phaseV=1;
			   	if(phaseV>N) phaseV=N;
				sendTFT(phaseV,0x02,0x08);
			}
			if(addr==3)
			{
				//修改相位
				phaseV=phaseV-1;
				//限幅，限制占空比
			   	if(phaseV<1) phaseV=1;
			   	if(phaseV>N) phaseV=N;
				sendTFT(phaseV,0x02,0x08);
			}
			if(addr==4)
			{
				M=M+0.1;
				//限幅，限制占空比
			   	if(M<0.1) M=0.1;
			   	if(M>0.95) M=0.95;
				sendTFT(M,0x02,0x09);
			}
			if(addr==5)
			{
				M=M-0.1;
				//限幅，限制占空比
			   	if(M<0.1) M=0.1;
			   	if(M>0.95) M=0.95;
				sendTFT(M,0x02,0x09);
			}
			len=strlen(s4);
			send_msgc(s4,len);
			break;

		//命令示例：EE 04 00

		//Modbus协议发送ASCII数据到蓝牙
		//命令示例：EE 05 00
		//注：串口助手要设置成不以16进制显示
		case 5:
			addr = SCI_Msg.rxData[ADDR_I];
			if(addr==0)
			{
				itoa((int)Ud, str);
				strcat(pa1,str);
				strcat(pa1,"\r\n");
				len=strlen(pa1);
				send_msgc(pa1,len);

				itoa((int)Id, str);
				strcat(pa2,str);
				strcat(pa2,"\r\n");
				len=strlen(pa2);
				send_msgc(pa2,len);

				itoa((int)Uoa, str);
				strcat(pa3,str);
				strcat(pa3,"\r\n");
				len=strlen(pa3);
				send_msgc(pa3,len);

				itoa((int)Uob, str);
				strcat(pa4,str);
				strcat(pa4,"\r\n");
				len=strlen(pa4);
				send_msgc(pa4,len);

				itoa((int)Uoc, str);
				strcat(pa5,str);
				strcat(pa5,"\r\n");
				len=strlen(pa5);
				send_msgc(pa5,len);

				itoa((int)Ioa, str);
				strcat(pa6,str);
				strcat(pa6,"\r\n");
				len=strlen(pa6);
				send_msgc(pa6,len);
			}
			break;

		default:break;
	}

	SCI_Msg.rxWriteIndex = 0;
	SCI_Msg.rxReadIndex = 0;

	return 0;
}

//---------------------------------------------------------------------
//定时器0初始化1ms 中断配置
//---------------------------------------------------------------------
void timer0_init()
{
	//CpuTimer0初始化
    InitCpuTimers();

	//中断配置步骤-----1,开启模块中断使能，位于 Timer->RegsAddr->TCR.bit.TIE = 1;
	//ConfigCpuTimer(&CpuTimer0, 150, 1000);		//定时时间1000=1ms
	ConfigCpuTimer(&CpuTimer0, 150, 500);			//定时时间500=0.5ms
    CpuTimer0Regs.TCR.all = 0x4001;             	//Use write-only instruction to set TSS bit = 0

	//中断配置步骤-----2，重映射中断服务函数
	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;
	//中断配置步骤-----3，连接CPU中断Y
	IER |= M_INT1;
	//中断配置步骤-----4，连接Y中断里的第几位
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
}

interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;

	//AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;						//timer0软件触发启动ADC SEQ1，定时时间1000=1ms AD采样也是1ms

	//串口通讯处理
	if(SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag == 1)
	{
	   SCI_Msg.timerOut++;
	}

	if(CpuTimer0.InterruptCount>1000)
	{
		CpuTimer0.InterruptCount=0;
	}
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

Uint16 KeyScan(void)
{
	static Uint16 key_up=1;									//按键按松开标志
	if(key_up&&(KEY1==0))
	//if(key_up&&(KEY1==0||KEY2==0||KEY3==0))					//有键按下
	{
		Delay(10000);										//去抖动
		key_up=0;											//表示按键没松开
		if(KEY1==0)											//按下KEY1
		{
  			return 1;
		}
	}
	else if(KEY1==1)
	{
		key_up=1;
	}
	return 0;

/*		else if(KEY2==0)
		{
			return 2;
		}
		else if(KEY3==0)
		{
			return 3;
		}
	}
	else if(KEY1==1||KEY2==1||KEY3==1)
	{
		key_up=1;
	}
	return 0;			*/									//无按键按下
}

//将整数n这个数字转换为对应的字符串，保存到s中
void itoa(int n,char s[])  
{
     int i = 0;  
     int left = 0;  
     int right = 0;  
     while( n )       			//将数字转换为字符  
     {  
        s[i] = n % 10 + '0';  
        n /= 10;  
        i++;         			//left已经向后移位  
     }  
     right = i - 1;  
     s[i] = '\0';        		//添加字符串结束标志  
     while(left < right)    	//将数组中的元素逆置  
     {  
         char tmp = s[left];  
         s[left] = s[right];  
         s[right] = tmp;  
         left++;  
         right--;  
      }  
}  

//将浮点数处理为ASCII码
//6位有效数字，这里为了整齐 如：0.01234当作6位有效数字
void float_TO_ascii(float a, char dat[8])
{
        if(1000<=a&&a<10000)
        {
                dat[0] = (int)a%10000/1000 + 0x30;
                dat[1] = (int)a%1000/100 + 0x30;
                dat[2] = (int)a%100/10 + 0x30;
                dat[3] = (int)a%10 + 0x30;
                dat[4] = 0x2e;
                dat[5] = (int)(a*10)%10 + 0x30;
                dat[6] = (int)(a*100)%10 + 0x30;
                dat[7] = 0;
        }
        if(100<=a&&a<1000)
        {
                dat[0] = (int)a%1000/100 + 0x30;
                dat[1] = (int)a%100/10 + 0x30;
                dat[2] = (int)a%10 + 0x30;
                dat[3] = 0x2e;
                dat[4] = (int)(a*10)%10 + 0x30;
                dat[5] = (int)(a*100)%10 + 0x30;
                dat[6] = (int)(a*1000)%10 + 0x30;
                dat[7] = (int)(a*10000)%10 + 0x30;
        }
        if(10<=a&&a<100)
        {
                dat[0] = (int)a%100/10 + 0x30;
                dat[1] = (int)a%10 + 0x30;
                dat[2] = 0x2e;
                dat[3] = (int)(a*10)%10 + 0x30;
                dat[4] = (int)(a*100)%10 + 0x30;
                dat[5] = (int)(a*1000)%10 + 0x30;
                dat[6] = (int)(a*10000)%10 + 0x30;
                dat[7] = (int)(a*100000)%10 + 0x30;
        }
        if(1<=a&&a<10)
        {
                dat[0] = (int)a%10 + 0x30;
                dat[1] = 0x2e;
                dat[2] = (int)(a*10)%10 + 0x30;
                dat[3] = (int)(a*100)%10 + 0x30;
                dat[4] = (int)(a*1000)%10 + 0x30;
                dat[5] = (int)(a*10000)%10 + 0x30;
                dat[6] = (int)(a*100000)%10 + 0x30;
                dat[7] = 0;
        }
        if(0<=a&&a<1)
        {
                dat[0] = 0x30;
                dat[1] = 0x2e;
                dat[2] = (int)(a*10)%10 + 0x30;
                dat[3] = (int)(a*100)%10 + 0x30;
                dat[4] = (int)(a*1000)%10 + 0x30;
                dat[5] = (int)(a*10000)%10 + 0x30;
                dat[6] = (int)(a*100000)%10 + 0x30;
                dat[7] = 0;
        }
        if(-1<a&&a<0)
        {
                dat[0] = 0x2d;
                dat[1] = 0x30;
                dat[2] = 0x2e ;
                dat[3] = (int)(-a*10)%10 + 0x30;
                dat[4] = (int)(-a*100)%10 + 0x30;
                dat[5] = (int)(-a*1000)%10 + 0x30;
                dat[6] = (int)(-a*10000)%10 + 0x30;
                dat[7] = (int)(-a*100000)%10 + 0x30;
        }
        if(-10<a&&a<=-1)
        {
                dat[0] = 0x2d;
                dat[1] = (int)(-a)%10 + 0x30;
                dat[2] = 0x2e ;
                dat[3] = (int)(-a*10)%10 + 0x30;
                dat[4] = (int)(-a*100)%10 + 0x30;
                dat[5] = (int)(-a*1000)%10 + 0x30;
                dat[6] = (int)(-a*10000)%10 + 0x30;
                dat[7] = (int)(-a*100000)%10 + 0x30;
        }
        if(-100<a&&a<=-10)
        {
                dat[0] = 0x2d;
                dat[1] = (int)(-a)%100/10 + 0x30;
                dat[2] = (int)(-a)%10 + 0x30;
                dat[3] = 0x2e ;
                dat[4] = (int)(-a*10)%10 + 0x30;
                dat[5] = (int)(-a*100)%10 + 0x30;
                dat[6] = (int)(-a*1000)%10 + 0x30;
                dat[7] = (int)(-a*10000)%10 + 0x30;
        }
        if(-1000<a&&a<=-100)
        {
                dat[0] = 0x2d;
                dat[1] = (int)(-a)%1000/100 + 0x30;
                dat[2] = (int)(-a)%100/10 + 0x30;
                dat[3] = (int)(-a)%10 + 0x30;
                dat[4] = 0x2e ;
                dat[5] = (int)(-a*10)%10 + 0x30;
                dat[6] = (int)(-a*100)%10 + 0x30;
                dat[7] = (int)(-a*1000)%10 + 0x30;
        }
        if(-10000<a&&a<=-1000)
        {
                dat[0] = 0x2d;
                dat[1] = (int)(-a)%10000/1000 + 0x30;
                dat[2] = (int)(-a)%1000/100 + 0x30;
                dat[3] = (int)(-a)%100/10 + 0x30;
                dat[4] = (int)(-a)%10 + 0x30;
                dat[5] = 0x2e ;
                dat[6] = (int)(-a*10)%10 + 0x30;
                dat[7] = (int)(-a*100)%10 + 0x30;
        }
}

void StopRun()
{
	//停止PWM4 DCDC
	DCDC_N=95;
	mPWM_OFF();

	//停止PWM1 SPWM
	Delay(500);
	mSPWM_OFF();

	IsProt=0;
	DisType=0;

	mRun_OFF();
	JDQ1_OFF();
}

void sendTFT(float32 U,Uint16 SID,Uint16 ID)
{
	//广州大彩设备出厂前均默认 RS232 电平，若用户需要 TTL 通讯方式，只需将 J5 焊盘短路
	//短路后就不能用RS232进行串口测试，但可以用VisualTFT下载工程
	//在工程配置中可以设置与F28035的传输波特率 
	//TTL 电平模式下兼容 3.3V 和 5V IO 输入输出
	//SCI发送数据，波特率19200
	//ASCII(., 0, 1, 2, 3, 4, 5, 6, 7,  8, 9)
	//=HEX(2E,30,31,32, 33,34,35,36,37,38,39)
	//EE B1 10 Screen_id Control_id  	Strings    		FF FC FF FF
	//EE B1 10   00 01     00 01     32 33 2E 30 35     FF FC FF FF =23.05
	// 0  1  2    3  4     5   6     7   8  9  10  11   12 13 14 15
	char str[10];										//char str[10];定义不能放到main()前的全局定义上，否则串口命令不能正常运行，原因不明
	//ftoa()使用时系统不能正常运行，现象是：如果1和2路拉载开启，串口发送A2 03 00命令时第1路拉载的PWM方波会停止
	//如果不使用ftoa()系统能正常运行，也不会出现上面现象，应该找一个更好的函数
	float_TO_ascii(U,str);

	send_Tdata[4] = SID;					//页面控件ID

	send_Tdata[6] = ID;						//控件ID

	send_Tdata[7] = str[0];					//send_Tdata[7-11]为ASCII数值
	send_Tdata[8] = str[1];
	send_Tdata[9] = str[2];
	send_Tdata[10]= str[3];
	send_Tdata[11]= str[4];

	send_msg(send_Tdata);					//发送数据
}

//得到TFT的控件数据
void getTFT(Uint16 SID,Uint16 ID)
{
	send_Rdata[4] = SID;					//页面控件ID
	send_Rdata[6] = ID;						//控件ID
	send_msg(send_Rdata);					//发送数据
}

/*
//---------------------------------------------------------------------
//DCDC PID初始化
//---------------------------------------------------------------------
void Init_Ub_PID(void)
{
	//输入160VDC输出400VDC的PID参数
    startpid=0.8;
    pmin=2.17;						//setmid-pmin=0.04<fastv
    setmid=2.21;
    pmax=2.25;						//pmax-setmid=0.04<fastv
    fastv=0.05;	
    PWMD=10;						//PWM调节幅度
}

//---------------------------------------------------------------------
//DCDC PID
//---------------------------------------------------------------------
void PWM_Ub_PID(void)
{
	//----------------------------PID----------------------------------
	//输出偏大，减小占空比
	if(Ub>pmax)
	{
		DC_PWM+=1;					//细调PWM
		if((Ub-setmid)>fastv)
		{
			DC_PWM+=PWMD;			//快速调节PWM幅度
		}
	}
	//输出稳定，保持占空比
	if((Ub<=pmax)&&(Ub>=pmin)){DC_PWM+=0;}
	//输出减小，增大占空比
	if((Ub<pmin)&&(Ub>startpid))
	{
		DC_PWM-=1;					//细调PWM
		if((setmid-Ub)>fastv)
		{
			DC_PWM-=PWMD;			//快速调节PWM幅度
		}
	}
	//无输出电压，保持占空比最小，输出最小
	if(Ub<startpid){DC_PWM=1995;}
	//限幅，限制占空比
   	if(DC_PWM<1155) DC_PWM=1155;
   	if(DC_PWM>1995) DC_PWM=1995;
}
*/
//---------------------------------------------------------------------
//DCAC PID初始化
//---------------------------------------------------------------------
void Init_Uo_PID(void)
{
	//输入400VDC输出225VAC的PID参数
    startpids=1.5;
    pmins=2.97;								//220VAC setmid-pmin=0.04<fastv
    setmids=3.01;							//230VAC
    pmaxs=3.05;								//235VAC pmax-setmid=0.04<fastv
    fastvs=0.05;	
    PWMDs=0.05;
}

//---------------------------------------------------------------------
//DCAC增量式PID算法
//---------------------------------------------------------------------
void PWM_Uo_PID(void)
{
	//----------------------------PID----------------------------------
	///*
	//无输出电压，保持占空比最小，输出最小
	if(Ud>startpids)
	{
		//输出偏大，减小占空比
		if(Uoa>pmaxs)
		{
			M-=0.01;					//细调SPWM
			if((Uoa-setmids)>fastvs)
			{
				M-=PWMDs;				//快速调节SPWM幅度
			}
		}
		//输出稳定，保持占空比
		if((Uoa<=pmaxs)&&(Uoa>=pmins)){M+=0;}
		//输出减小，增大占空比
		if((Uoa<pmins)&&(Uoa>startpids))
		{
			M+=0.01;					//细调SPWM
			if((setmids-Uoa)>fastvs)
			{
				M+=PWMDs;				//快速调节SPWM幅度
			}
		}
		//限幅，限制占空比
	   	if(M<0.1) M=0.1;
	   	if(M>0.95) M=0.95;
   		Calc_Spwm();
	}
}

//ADC中断
interrupt void  adc_isr(void)
{
	Ub_REG[ConversionCount]  = AdcRegs.ADCRESULT0 >>4;
	Id_REG[ConversionCount]  = AdcRegs.ADCRESULT1 >>4;
	Uu_REG[ConversionCount]  = AdcRegs.ADCRESULT2 >>4;
	Vr1_REG[ConversionCount] = AdcRegs.ADCRESULT3 >>4;

	Uoa_REG[ConversionCount] = AdcRegs.ADCRESULT4 >>4;
	Iob_REG[ConversionCount] = AdcRegs.ADCRESULT5 >>4;
	Ioc_REG[ConversionCount] = AdcRegs.ADCRESULT6 >>4;
	Ud_REG[ConversionCount]  = AdcRegs.ADCRESULT7 >>4;

	Ioa_REG[ConversionCount] = AdcRegs.ADCRESULT8 >>4;
	Uob_REG[ConversionCount] = AdcRegs.ADCRESULT9 >>4;
	Uoc_REG[ConversionCount] = AdcRegs.ADCRESULT10>>4;
	Uf_REG[ConversionCount]  = AdcRegs.ADCRESULT11>>4;

	Uv_REG[ConversionCount]  = AdcRegs.ADCRESULT12>>4;//...
	A32_REG[ConversionCount] = AdcRegs.ADCRESULT13>>4;
	A33_REG[ConversionCount] = AdcRegs.ADCRESULT14>>4;
	Uw_REG[ConversionCount]  = AdcRegs.ADCRESULT15>>4;
	
	//如果128个采样点完成了就进行下一轮ADC采样
	if(ConversionCount == 127)
	{
		ADC_Flag=1;
	 	ConversionCount = 0;
	}
	else
	{
		ConversionCount++;
	}

	//重新初始化为下ADC序列
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; 			// 重置 SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; 		// 清除INT SEQ1位
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; 	// 确认中断到PIE

	return;
}

void Lock_Freq(void)
{
	//------------------------------------捕获频率 begin------------------------------------------ 
	//本系统采用DSP的捕获模块通过对电网参考过零方波信号上升沿的捕获来获取频率,通过DSP内部的计数器得到过零方波信号前后两个上升沿的时间差，
	//即为过零方波信号的周期，从而得到正弦波的频率；而过零方波信号的每个上升沿都对应着电网参考电压正弦波的过零点，
	//-------------------------------------------------------------------------------------------
	if(Freq_Flag1)
	{
		Freq=150000000/nCAP1;

		//频率调整，频率在44-56Hz内，调整频率
		//频率跟踪测试方法：先开启逆变，然后通过k1,k2按键调整频率，如调到输出53Hz(44-56Hz)
		//然后接通电网，此时输出正弦波频率马上就会与电网频率一致，表示测试成功，注此过程中千万不要合并网继电器，因为没做电网并网条件测试
		//开环情况下进行频率跟踪
		if(Is_UoPID==0)
		{
			if((Freq>=44)&&(Freq<=56))
			{
				if(((fs-Freq)>0.3)||((Freq-fs)>0.3))			//fs初始值是50Hz标准频率
				{
					fs=Freq;
					Calc_Spwm();								//fs发生变化就重新计算SPWM
					EPwm1Regs.TBPRD = CarrVal;					//修改fs正弦波输出频率
					EPwm2Regs.TBPRD = CarrVal;					//修改fs正弦波输出频率
					EPwm3Regs.TBPRD = CarrVal;					//修改fs正弦波输出频率
				}
			}
			Freq_Flag1 = 0;
			Freq_Lock = 1;										//锁频完毕
		}
	}
	//-----------------------------频率和相位捕获计算 end-------------------------------------------- 
}

//SPWM 中断
//ePWM模块能够在保证系统开销最小的前提下可提供0%~100%占空比，有三种工作模式：加法计数模式、可逆计数模式和减法计数模式。
//本系统采用可逆（Up-Down）计数模式（PWM波形对称)，当加法计数值达到与CMPA值匹配，置位ePWM1A输出；
//当减法计数值达到与CMPA值匹配,ePWM1A输出复位；如果CMPA值与计数器的值不匹配，则调用ISR并加载阴影寄存器。

//利用DSP的ePWM，由开关频率计算出ePWM的周期，在每一次ePWM中断到来时，由规则采样法计算本周期PWM波的占空比，即可得到相应的SPWM波。
//当频率发生改变，则只需改变ePWM的周期。若需要调整输出波形的相位，则移动正弦波表的指针即可。
interrupt void epwm1_isr(void)
{
	SPWMCntA++;

	if(SPWMCntA>=N)
	{
		SPWMCntA=0;
	}

	SinCntA=(Uint16)(SPWMCntA*128/N);
	EPwm1Regs.CMPA.half.CMPA = CompVal[SinCntA];		//CMPA 计数器比较A 寄存器集,CompVal[]在calc_spwm()方法中,SinCnt=0-128

	EPwm1Regs.ETCLR.bit.INT=1;
	PieCtrlRegs.PIEACK.all =PIEACK_GROUP3;
}

interrupt void epwm2_isr(void)
{
	SPWMCntB++;

	if(SPWMCntB>=N)
	{
		SPWMCntB=0;
	}

	SinCntB=(Uint16)(SPWMCntB*128/N);
	EPwm2Regs.CMPA.half.CMPA = CompVal[SinCntB];		//CMPA 计数器比较A 寄存器集,CompVal[]在calc_spwm()方法中,SinCnt=0-128

   	EPwm2Regs.ETCLR.bit.INT=1;
   	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void epwm3_isr(void)
{
	SPWMCntC++;

	if(SPWMCntC>=N)
	{
		SPWMCntC=0;
	}

	SinCntC=(Uint16)(SPWMCntC*128/N);
	EPwm3Regs.CMPA.half.CMPA = CompVal[SinCntC];		//CMPA 计数器比较A 寄存器集,CompVal[]在calc_spwm()方法中,SinCnt=0-128

	EPwm3Regs.ETCLR.bit.INT=1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//DCDC的PWM
interrupt void epwm4_isr(void)
{
	//关于PWM的频率，占空比CMPA值和TBPRD周期的关系：
	//1、TBPRD周期一定则PWM频率一定
	//2、TBPRD值越大PWM频率越小
	//3、PWM频率越小，占空比CMPA值可以取得越大，反之
	//所以TBPRD值和CMPA值取多大很有讲究。
	EPwm4Regs.CMPA.half.CMPA=DC_PWM;

	EPwm4Regs.DBRED = DCDC_LED;
	EPwm4Regs.DBFED = DCDC_RED;

	EPwm4Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//捕获中断
interrupt void ecap1_isr(void)
{
	nCAP1=ECap1Regs.CAP1;					//捕捉电网

	Freq_Flag1=1;
	Lock_Freq();
	if(Phase_Lock==1)
	{
		if(Freq_Lock) 
		{
			//SPWMCntA是调整与电网的相位差的，相位差越大，并网冲击电流越大甚至跳闸或烧毁功率管
			SPWMCntA=phaseV;
			Freq_Lock = 0;						//开启一下次锁频
		}
	}

	ECap1Regs.ECCLR.bit.CEVT1 = 1;			//ECCLR 捕捉中断清除寄存器,CEVT1在DSP2833x_ECap.h文件中定义
	ECap1Regs.ECCLR.bit.INT = 1;
	ECap1Regs.ECCTL2.bit.REARM = 1;			//ECCTL2 捕捉控制寄存器2

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

interrupt void ecap2_isr(void)
{
	nCAP2=ECap1Regs.TSCTR;					//TSCTR 时间戳计数器
	Freq_Flag2=1;

	if(Phase_Lock==1)
	{
		SPWMCntB=phaseV;
	}

	ECap2Regs.ECCLR.bit.CEVT1 = 1;
	ECap2Regs.ECCLR.bit.INT = 1;
	ECap2Regs.ECCTL2.bit.REARM = 1;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

interrupt void ecap3_isr(void)
{
	nCAP3=ECap1Regs.TSCTR;					//TSCTR 时间戳计数器
	Freq_Flag3=1;

	if(Phase_Lock==1)
	{
		SPWMCntC=phaseV;
	}

	ECap3Regs.ECCLR.bit.CEVT1 = 1;
	ECap3Regs.ECCLR.bit.INT = 1;
	ECap3Regs.ECCTL2.bit.REARM = 1;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

//初始化捕获模块Uref
void InitECapture1()
{
	ECap1Regs.ECEINT.all = 0x0000; 			// 禁用所有捕捉中断
	ECap1Regs.ECCLR.all = 0xFFFF;  			// 清除所有CAP中断标志
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0; 		// 禁止CAP1-CAP4寄存器加载
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;  	// 确保计数器停止

	//配置外设寄存器
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;	// 单次
	ECap1Regs.ECCTL2.bit.STOP_WRAP = 0;  	// 停在4事件
	ECap1Regs.ECCTL1.bit.CAP1POL = 0; 		// 上升沿
	ECap1Regs.ECCTL1.bit.CTRRST1 = 1; 		// 差异操作
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;		// 在启用同步
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;  	// 通过
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1; 		// 启用捕获单元

	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;  	// 启动计数器
	ECap1Regs.ECCTL2.bit.REARM= 1;			// arm one-shot
	ECap1Regs.ECEINT.bit.CEVT1 = 1;			// 4 events = interrupt
}

void InitECapture2()
{
	ECap2Regs.ECEINT.all = 0x0000; 			// Disable all capture interrupts
	ECap2Regs.ECCLR.all = 0xFFFF;  			// Clear all CAP interrupt flags
	ECap2Regs.ECCTL1.bit.CAPLDEN = 0; 		// Disable CAP1-CAP4 register loads
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;  	// Make sure the counter is stopped

	// Configure peripheral registers
	ECap2Regs.ECCTL2.bit.CONT_ONESHT = 1;	// One-shot
	ECap2Regs.ECCTL2.bit.STOP_WRAP = 0;  	// Stop at 4 events
	ECap2Regs.ECCTL1.bit.CAP1POL = 0; 		// Rising edge
	ECap2Regs.ECCTL1.bit.CTRRST1 = 1; 		// Difference operation
	ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;		// Enable sync in
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;  	// Pass through
	ECap2Regs.ECCTL1.bit.CAPLDEN = 1; 		// Enable capture units

	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;  	// Start Counter
	ECap2Regs.ECCTL2.bit.REARM= 1;			// arm one-shot
	ECap2Regs.ECEINT.bit.CEVT1 = 1;			// 4 events = interrupt
}

void InitECapture3()
{
	ECap3Regs.ECEINT.all = 0x0000; 			// Disable all capture interrupts
	ECap3Regs.ECCLR.all = 0xFFFF;  			// Clear all CAP interrupt flags
	ECap3Regs.ECCTL1.bit.CAPLDEN = 0; 		// Disable CAP1-CAP4 register loads
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;  	// Make sure the counter is stopped

	// Configure peripheral registers
	ECap3Regs.ECCTL2.bit.CONT_ONESHT = 1;	// One-shot
	ECap3Regs.ECCTL2.bit.STOP_WRAP = 0;  	// Stop at 4 events
	ECap3Regs.ECCTL1.bit.CAP1POL = 0; 		// Rising edge
	ECap3Regs.ECCTL1.bit.CTRRST1 = 1; 		// Difference operation
	ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;		// Enable sync in
	ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;  	// Pass through
	ECap3Regs.ECCTL1.bit.CAPLDEN = 1; 		// Enable capture units

	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;  	// Start Counter
	ECap3Regs.ECCTL2.bit.REARM= 1;			// arm one-shot
	ECap3Regs.ECEINT.bit.CEVT1 = 1;			// 4 events = interrupt
}


void InitEPwmTZ()
{
   // Enable TZ1 and TZ2 as one shot trip sources
   EALLOW;

   // What do we want the TZ1 and TZ2 to do?
//  EPwm1Regs.TZCTL.bit.TZA = 0x3;
//  EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm2Regs.TZCTL.bit.TZA = 0x3;
   EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
  EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
   EPwm3Regs.TZCTL.bit.TZB = 0x3;

   EDIS;
}


//初始化的ePWM模块SPWM
//ePWM模块由以下几个子模块构成：时基(TB)子模块、计数器-比较器(CC)子模块、动作限定(AQ)子模块、死区(DB)发生器子模块、PWM斩波器(PC)子模块、
//故障断路器(Trip Zone)子模块、事件触发器(ET)子模块。配置ePWM模块时需要对上述子模块中的寄存器进行初始化。
void InitSPwm1()
{
	EPwm1Regs.TBPRD = CarrVal;						// Set timer period
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
	EPwm1Regs.TBCTR = 0x0000;                       // Clear counter

	EPwm1Regs.CMPA.half.CMPA = 0;					//系统上电时占空比为0

	// Setup TBCLK
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Count up
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        	// Disable phase loading
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       	// Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;          	// Slow so we can observe on the scope

	// Setup shadowing CMPCTL计数器比较控制寄存器
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  	// Load on Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on CAU
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;            // Clear PWM1A on CAD

	EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Clear PWM3B on CAU
	EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;              // Set PWM1B on CAD

	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;

	EPwm1Regs.DBRED = 145;							//1.5us死区
	EPwm1Regs.DBFED = 145;
   
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
	EPwm1Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             // Generate INT on 1st event   
}

void InitSPwm2()
{
	EPwm2Regs.TBPRD = CarrVal;						// Set timer period
	EPwm2Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
	EPwm2Regs.TBCTR = 0x0000;                       // Clear counter

	EPwm2Regs.CMPA.half.CMPA = 0;					//系统上电时占空比为0

	// Setup TBCLK
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Count up
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        	// Disable phase loading
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       	// Clock ratio to SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;          	// Slow so we can observe on the scope

	// Setup shadowing CMPCTL计数器比较控制寄存器
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  	// Load on Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on CAU
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;            // Clear PWM1A on CAD

	EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Clear PWM3B on CAU
	EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;              // Set PWM1B on CAD

	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;

	EPwm2Regs.DBRED = 145;
	EPwm2Regs.DBFED = 145;

	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
	EPwm2Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
	EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;             // Generate INT on 1st event   
}

void InitSPwm3()
{
	EPwm3Regs.TBPRD = CarrVal;						// Set timer period
	EPwm3Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
	EPwm3Regs.TBCTR = 0x0000;                       // Clear counter

	EPwm3Regs.CMPA.half.CMPA = 0;					//系统上电时占空比为0

	// Setup TBCLK
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Count up
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        	// Disable phase loading
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       	// Clock ratio to SYSCLKOUT
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;          	// Slow so we can observe on the scope

	// Setup shadowing CMPCTL计数器比较控制寄存器
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  	// Load on Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on CAU
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;            // Clear PWM1A on CAD

	EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Clear PWM3B on CAU
	EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;              // Set PWM1B on CAD

	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;

	EPwm3Regs.DBRED = 145;
	EPwm3Regs.DBFED = 145;
   
	EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
	EPwm3Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
	EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;             // Generate INT on 1st event   
}

//DCDC推挽用的PWM
void InitSPwm4()
{
   //ePWM的时钟TBCLK=SYSCLKOUT/(HSPCLKDIV×CLKDIV)
   //PWM信号周期与频率的计算如下
   //Tpwm=(TBPRD+1)*TBCLK
   //Fpwm=1/Tpwm
   EPwm4Regs.TBPRD = DCDC_T;                        // 35.2KHz,DCDC推挽用的PWM的频率,30MHz=2130,值越大，频率越小
   EPwm4Regs.TBPHS.half.TBPHS = 0x0000;            	// Phase is 0
   EPwm4Regs.TBCTR = 0x0000;                       	// Clear counter

   // Setup compare
   EPwm4Regs.CMPA.half.CMPA = 1995;					//初始占空比5%

   // Setup TBCLK
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Count up
   EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;       	// Disable phase loading
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       	// =0,Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;          	// =0,Slow so we can observe on the scope

   // Setup shadowing CMPCTL计数器比较控制寄存器
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  	// Load on Zero
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;              	// Set PWM3A on Zero
   EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;

   EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;            	// Set PWM3A on Zero
   EPwm4Regs.AQCTLB.bit.CAD = AQ_SET;

   // Active high complementary PWMs - Setup the deadband
   EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   //DB_ACTV_HIC PWM输出为高有效(PWM死区同时为低电平)
   //DB_ACTV_LOC PWM输出为低有效(PWM死区同时为高电平)
   EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;		//=DB_ACTV_HIC(DB_ACTV_HI,DB_ACTV_LOC,DB_ACTV_HIC,DB_ACTV_LO)
   //DBA_ALL PWM互补模式
   //DBB_RED_DBA_FED非互补模式，电平同高同低
   EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;			//=DBA_ALL(DBA_ALL,DBB_RED_DBA_FED,DBA_RED_DBB_FED,DBB_ALL)

   //DBRED PWM左边死区值
   //DBFED PWM右边死区值
   EPwm4Regs.DBRED = DCDC_LED;
   EPwm4Regs.DBFED = DCDC_RED;

   // Interrupt where we will change the deadband
   EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       	// Select INT on Zero event
   EPwm4Regs.ETSEL.bit.INTEN = 1;                  	// Enable INT
   EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;             	// Generate INT on 3rd event
}

//初始化ADC模块
void ADCInit(void)
{
	// 配置ADC
	AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;			// ADCTRL1 ADC 控制寄存器1
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        		// 级联模式
	AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;		// ADCTRL3 ADC 控制寄存器3
	AdcRegs.ADCTRL3.bit.SMODE_SEL= 0;				// 设置顺序采样模式
	AdcRegs.ADCMAXCONV.all = 0x000F;       			// ADCMAXCONV ADC 最大转换信道数寄存器,设置为16信道

	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; 			// Setup ADCINA0 as 1st SEQ1 conv.ADCCHSELSEQ1  ADC 信道选择定序控制寄存器1
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; 			// Setup ADCINA1 as 2nd SEQ1 conv.
	AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; 			// Setup ADCINA2 as 3nd SEQ1 conv.
	AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; 			// Setup ADCINA3 as 4nd SEQ1 conv.

	AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; 			// Setup ADCINA4 as 1st SEQ2 conv.ADCCHSELSEQ2  ADC 信道选择定序控制寄存器2
	AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5; 			// Setup ADCINA5 as 2nd SEQ2 conv.
	AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6; 			// Setup ADCINA6 as 3nd SEQ2 conv.
	AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7; 			// Setup ADCINA7 as 4nd SEQ2 conv.

	AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8; 			// Setup ADCINB0 as 1nd SEQ3 conv.ADCCHSELSEQ3  ADC 信道选择定序控制寄存器3
	AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x9; 			// Setup ADCINB1 as 2nd SEQ3 conv.
	AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0xA; 			// Setup ADCINB2 as 3nd SEQ3 conv.
	AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0xB; 			// Setup ADCINB3 as 4nd SEQ3 conv.

	AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0xC; 			// Setup ADCINB4 as 1nd SEQ4 conv.ADCCHSELSEQ4  ADC 信道选择定序控制寄存器3
	AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xD; 			// Setup ADCINB5 as 2nd SEQ4 conv.
	AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xE; 			// Setup ADCINB6 as 3nd SEQ4 conv.
	AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0xF; 			// Setup ADCINB7 as 4nd SEQ4 conv.

	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;			// Enable SOCA from ePWM to start SEQ1 使用ePWM源就用这句
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  			// Enable SEQ1 interrupt (every EOS)

///*
	//使用ePWM源就用上下面的语句
	//Assumes(假设) ePWM1 clock is already enabled in InitSysCtrl();
	//片上的ADC采用EPWM模块驱动，通过调整EPWM模块的周期来改变ADC采样率，ADC采样率为6.4kHz。
	//和F28035不一样，F28335如下直接指定一路PWM作触发器，这路PWM就不能用作驱动了
	EPwm5Regs.ETSEL.bit.SOCAEN = 1;        			// ETSEL事件触发器选择寄存器,Enable SOC on A group
	EPwm5Regs.ETSEL.bit.SOCASEL = 4;       			// Select SOC from from CPMA on upcount
	EPwm5Regs.ETPS.bit.SOCAPRD = 1;        			// ETPS事件触发器预分频寄存器
	EPwm5Regs.CMPA.half.CMPA = 0x0080;	  			// CMPA计数器比较A 寄存器,Set compare A value
	EPwm5Regs.TBPRD = 0x3A98;              			// Set period for ePWM2
	EPwm5Regs.TBCTL.bit.CTRMODE = 0;		  		// TBCTL时基控制寄存器,count up and start
//*/
}

//==========================================================================================================
//初始化SPI函数
void spi_init()
{    
	SpiaRegs.SPICCR.all =0x004F;		// SPI软件复位, 极性位为1（下降沿发送数据）, 每次移进和移出16位字长度；禁止SPI内部回送（LOOKBACK）功能；
	SpiaRegs.SPICTL.all =0x0006; 		// 使能主机模式，正常相位，使能主机发送，禁止接收溢出中断，禁止SPI中断；

	SpiaRegs.SPIBRR =0x007F;			// SPI波特率=25M/128	=195.3KHZ；							
    SpiaRegs.SPICCR.all =0x00CF;		// 停止SPI软件复位准备接收或发送；禁止回送模式； 
    SpiaRegs.SPIPRI.bit.FREE = 1;  		// 自由运行     
}

//初始化SPI FIFO
void spi_fifo_init()										
{
    SpiaRegs.SPIFFTX.all=0xE040;		//使能FIFO;清除发送中断标志位；禁止FIFO发送中断；发送中断级别定义为0；
    SpiaRegs.SPIFFRX.all=0x204f;		//清除FF溢出标志位；清除溢出接受中断标志位；禁止FF接受中断；接受中断级别为16；
    SpiaRegs.SPIFFCT.all=0x0;			//SPITXBUF到移位寄存器传送不延迟；
}

//发送SPI数据
void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF=a;
}

//==========================================================================================================
//SYS
//延时程序
void Delay(Uint16 i)
{
	while(i--);
}

//==========================================================================================================
//                                              系统主程序 end
//==========================================================================================================
