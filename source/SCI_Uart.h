//====================================================================
//SCI串口操作函数
//====================================================================

#include "DSP28x_Project.h"

#define SYSCLK 150000000L							//30MHz
#define SCIBaudRate 9600L
#define TIMEROUTSCI (Uint32)5*(SYSCLK/SCIBaudRate)	//估算的等待超时时间，请根据实际修改

#define SCI_FIFO_LEN  1 	//定义DSP串口FIFO深度
#define UartRxLEN 20  		//接收缓存长度
#define UartTxLEN 20  		//发送缓存长度

#define RTU_TIMEROUT 5 		//ms 用于等待串口调试助手发送操作命令的超时时间

typedef struct Uart_Type{
	union
	{
		Uint16 All;
	    struct{
			Uint16  UartRevFlag		:1;		//接收到数据标志
			Uint16  HWOVFlag		:1;		//DSP硬件接收数据溢出标志

			Uint16  rFifoDataflag	:1;		//接收内存非空
			Uint16  rFifoFullflag	:1;		//接收内存溢出

			Uint16  DISRevflag		:1;		//接收关闭

		}Status_Bits;
	}Mark_Para;

	char rxData[UartRxLEN];					//接收缓存
	Uint16 rxReadIndex;						//接收FIFO写入索引
	Uint16 rxWriteIndex;					//接收FIFO读出索引

	Uint16 timerOut;						//超时判断
}Uart_Msg; 

Uart_Msg SCI_Msg;

//Modbus协议方式,发送协议定义
char send_data[16]={0x55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x0d};			//发送的数据
//发送的数据更新TFT LCD文本数据
//EE B1 10 Screen_id Control_id  	Strings    		FF FC FF FF
//EE B1 10   00 01     00 01     32 33 2E 30 35     FF FC FF FF =23.05
char send_Tdata[16]={0xEE,0xB1,0x10,0x00,0x01,0x00,0,0,0,0,0,0,0xFF,0xFC,0xFF,0xFF};

//获取数据命令
char send_Rdata[11]={0xEE,0xB1,0x11,0x00,0x02,0x00,0x07,0xFF,0xFC,0xFF,0xFF};

//--------------------------------------------------------------------
void scic_init(void);
void scic_xmit(int a);
void send_msg(char s[]);											//传送数据值
void send_msgc(char *str,int len);									//传送字符值

//---------------------------------------------------------------------
//串口接收中断函数
//采用FIFO机制（缓存）
//SCI_FIFO_LEN 定义为 1，最大为4
//---------------------------------------------------------------------
interrupt void uartRx_isr(void)
{
	if(ScicRegs.SCIFFRX.bit.RXFFOVF == 0)			//接收FIFO未溢出
	{
		SCI_Msg.Mark_Para.Status_Bits.rFifoDataflag = 1;

		if((SCI_Msg.rxWriteIndex + SCI_FIFO_LEN) != SCI_Msg.rxReadIndex )
		{
			//接收数据
			while(ScicRegs.SCIFFRX.bit.RXFFST)
			{
				SCI_Msg.rxData[SCI_Msg.rxWriteIndex] = ScicRegs.SCIRXBUF.all;
				SCI_Msg.rxWriteIndex=(++SCI_Msg.rxWriteIndex)%(UartRxLEN);
			}
		}
		else
		{
			//用户这里做缓存满的处理,
			ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;  	//Write 0 to reset the FIFO pointer to zero, and hold in reset.
			ScicRegs.SCIFFRX.bit.RXFIFORESET = 1 ; 	//Re-enable receive FIFO operation

			SCI_Msg.Mark_Para.Status_Bits.rFifoFullflag = 1;
		}
	}
	else
	{
		//用户这里做串口硬件溢出的处理,可以完全读取出FIFO里的数据或者清空FIFO
		//这里清空FIFO操作
		ScicRegs.SCIFFRX.bit.RXFFOVRCLR=1;   		// Clear HW Overflow flag
		ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;  		// Write 0 to reset the FIFO pointer to zero, and hold in reset.
		ScicRegs.SCIFFRX.bit.RXFIFORESET = 1 ; 		// Re-enable receive FIFO operation
		SCI_Msg.Mark_Para.Status_Bits.HWOVFlag = 1;
	}
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;   			// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

//---------------------------------------------------------------------
//串口初始化
//9600  8N1
//---------------------------------------------------------------------
void scic_init()
{
	InitScicGpio();

    ScicRegs.SCICTL1.bit.SWRESET =0;
 	ScicRegs.SCICCR.all =0x0007;   					// 1 stop bit,  No loopback,No parity,8 char bits, async mode, idle-line protocol

	//本系统定义的CPU_FRQ_150MHZ=1
	//BaudRate=LSPCLK/[(BRR+1)*8]
	#if (CPU_FRQ_150MHZ)							// 150 MHz CPU Freq (30 MHz input freq) by DEFAULT
		ScicRegs.SCIHBAUD    =0x0001;  				// BaudRate=9600 baud @LSPCLK = 37.5MHz,BRR=0x1E7
		ScicRegs.SCILBAUD    =0x00E7;

		//ScicRegs.SCIHBAUD    =0x0000;  			// 19200 baud @LSPCLK = 37.5MHz.
		//ScicRegs.SCILBAUD    =0x00F3;

    	//ScicRegs.SCIHBAUD    =0x0000;  			// 115200 baud @LSPCLK = 37.5MHz.
        //ScicRegs.SCILBAUD    =0x0028;
	#endif
	#if (CPU_FRQ_100MHZ)							// 100 Mhz CPU Freq (20 MHz input freq)
		ScicRegs.SCIHBAUD    =0x0001;  				// 9600 baud @LSPCLK = 25MHz.
		ScicRegs.SCILBAUD    =0x0044;
	#endif

    ScicRegs.SCICTL1.bit.SWRESET = 1;				// Relinquish SCI from Reset
    ScicRegs.SCIFFTX.bit.SCIRST=1;

	ScicRegs.SCIFFRX.bit.RXFFIL  = SCI_FIFO_LEN;  	//设置FIFO深度
	ScicRegs.SCICTL1.bit.TXENA = 1;       			//使能发送
	ScicRegs.SCICTL1.bit.RXENA = 1;       			//使能接收
	// 中断配置步骤-----1
	ScicRegs.SCIFFTX.bit.SCIFFENA = 1; 				//使能FIFO中断
	ScicRegs.SCIFFRX.bit.RXFFIENA=1;
	EALLOW;
	// 中断配置步骤-----2
	PieVectTable.SCIRXINTC = &uartRx_isr;
	EDIS;
	// 中断配置步骤-----3
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;				//SCIC接收中断
	// 中断配置步骤-----4   			
	IER |= M_INT8;						  			

	ScicRegs.SCIFFCT.all=0x00;

	ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;
}

//---------------------------------------------------------------------
//发送一个字节,超时机制,SYSCLK = 60MHz
//---------------------------------------------------------------------
void scic_xmit(int a)
{
	Uint32 WaitTimer = 0;

	//while(ScicRegs.SCICTL2.bit.TXEMPTY != 1)
	while (ScicRegs.SCIFFTX.bit.TXFFST != 0)
	{
		WaitTimer++;
		if(WaitTimer > TIMEROUTSCI)break;
	}
	if(WaitTimer <= TIMEROUTSCI)
		ScicRegs.SCITXBUF=a;
}

//传送数据值都是16进制数据
void send_msg(char s[])
{
    int i;

	//TX_EN;
    for(i=0;i<16;i++)
    {
        scic_xmit(s[i]);
    }
	//Delay(3000);
	//RX_EN;
}

//传送字符串值都是ascii码数据
void send_msgc(char *str,int len)
{
    int i;

	//TX_EN;
    for(i=0;i<len;i++)
    {
        scic_xmit(str[i]);
    }
	//Delay(3000);
	//RX_EN;
}

//---------------------------------------------------------------------
//sci_uart end
//---------------------------------------------------------------------
