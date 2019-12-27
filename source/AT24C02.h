//============================================================================================
//程序功能：IO口模拟I2C读写Eeprom程序
//程序使用：	1.调用Eerom_Gpio_Init函数，初始化与Eeprom相关的IO
//			2.调用 	writebyte(Uint16 addr,Uint16 data);  	//写Eeprom
//					readbyte(Uint16 addr);				 	//读Eeprom
//			3.查看读取的内容与写入内容是否一致
//kypine  2015.02.16
//============================================================================================

#include "DSP2833x_Device.h"     					// DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   					// DSP2833x Examples Include File

#define SDA_R   GpioDataRegs.GPBDAT.bit.GPIO32;     //SDA 读状态
#define SDA_W0  GpioDataRegs.GPBCLEAR.bit.GPIO32=1; //SDA 输出0 写状态
#define SDA_W1  GpioDataRegs.GPBSET.bit.GPIO32=1;   //SDA 输出1 写状态
#define SCL_0   GpioDataRegs.GPBCLEAR.bit.GPIO33=1; //SCL 输出0
#define SCL_1   GpioDataRegs.GPBSET.bit.GPIO33=1;   //SCL 输出1
#define DELAY_UNIT	10								//宏定义延时时间常数
Uint16 eromrw_err;									//Eeprom读写错误指示

void writebyte(Uint16 addr,Uint16 data);
Uint16 readbyte(Uint16 addr);
void Eerom_Gpio_Init(void);
void delay(Uint16 time);
void begintrans();
void stoptrans();
void ack();
void bytein(Uint16 ch);
Uint16 byteout(void);

void Eerom_Gpio_Init(void)
{
	EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;	  	//上拉
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;   	// 输出端口
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;  	// IO口
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3; 	// 不同步

    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;	  	//上拉
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;   	// 输出端口
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;  	// IO口
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;   // 不同步
    EDIS;
}

__inline void SDA_READ(void)
{
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO32=0;       //Input, SDA
    EDIS;
}

__inline void SDA_WRITE(void)
{
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO32=1;       //Output. SDA
    EDIS;
}

//延时函数
void delay(Uint16 time) 					
{
    for(; time>0 ; time--)
    {
        asm(" nop");
        asm(" nop");
        asm(" nop");
        asm(" nop");
		asm(" nop");
        asm(" nop");
        asm(" nop");
        asm(" nop");
    }
}

//发送START 信号
void begintrans(void)       				
{
    SDA_W1;         						//SDA=1
    delay(DELAY_UNIT * 10);         		//延时
    SDA_WRITE();            				//SDA 方向为输出到EEPROM
    delay(DELAY_UNIT * 10);         		//延时
    SCL_1;          						//SCL=1
    delay(DELAY_UNIT * 10);         		//延时
    SDA_W0;         						//SDA=0
    delay(DELAY_UNIT * 10);        			//延时
}

//发送STOP 信号
void stoptrans(void)        				
{
    SDA_WRITE();            				//SDA方向为输出到EEPROM
    delay(DELAY_UNIT * 10);        			//延时
    SDA_W0;         						//SDA=0
    delay(DELAY_UNIT * 10);         		//延时
    SCL_1;          						//SCL=1
    delay(DELAY_UNIT * 10);         		//延时
    SDA_W1;         						//SDA=1
    delay(DELAY_UNIT * 10);	
}

//等待EEPROM 的ACK 信号
void ack(void)              				
{
    Uint16 d;
    Uint16  i;     
    SDA_READ();             				//SDA方向为从EEPROM 输入
    delay(DELAY_UNIT * 2);          		//延时
    SCL_1;          						//SCL=1
    delay(DELAY_UNIT * 2);         			//延时
    i = 0;              
    do
    {
        d = SDA_R;
        i++;
        delay(DELAY_UNIT);	
    }
    while((d == 1) && (i <= 500));      	//等待EEPROM 输出低电平,4ms后退出循环

    if (i >= 499)
    {
        eromrw_err = 0xff;
    }
    
    i = 0;           
    SCL_0;          						//SCL=0
    delay(DELAY_UNIT * 2);          		//延时
}

//向EEPROM 写入一个字节 
void bytein(Uint16 ch)  					
{
    Uint16 i;     
    SCL_0;          						//SCL=0
    delay(DELAY_UNIT * 2);					//延时
    SDA_WRITE();            				//SDA方向为输出到EEPROM
    delay(DELAY_UNIT);         				//延时
    for(i=8;i>0;i--)
    {	
        if ((ch & 0x80)== 0) 
    	{
            SDA_W0;     					//数据通过SDA 串行移入EEPROM
            delay(DELAY_UNIT);				//延时
    	}
        else 
    	{
            SDA_W1;
            delay(DELAY_UNIT);				//延时
    	}
        SCL_1;      						//SCL=1 
        delay(DELAY_UNIT * 2);      		//延时
        ch <<= 1;
        SCL_0;      						//SCL=0 
        delay(DELAY_UNIT);      			//延时
    } 
    ack();
}

//从EEPROM 输出一个字节
Uint16 byteout(void)        				
{
    unsigned char i;
    Uint16 ch;
    ch = 0;

    SDA_READ();             				//SDA 的方向为从EEPROM 输出
    delay(DELAY_UNIT * 2);         			//延时
    for(i=8;i>0;i--)
    {
        ch <<= 1;
        SCL_1;      						//SCL=1
        delay(DELAY_UNIT);      			//延时
        ch |= SDA_R;    					//数据通过SDA 串行移出EEPROM
        delay(DELAY_UNIT);     				//延时
        SCL_0;      						//SCL=0
        delay(DELAY_UNIT * 2);      		//延时
    }
    return(ch);
}

//向EEPROM 指定地址写入一个字节的数据
void writebyte(Uint16 addr,Uint16 data) 	
{
    begintrans();							//开始
    bytein(0xA0 + ((addr & 0x0300) >> 7));  //写入写控制字0xA0
    bytein(addr);       					//写入指定地址
    bytein(data);      						//写入待写入EEPROM 的数据
    stoptrans();							//停止
    delay(8000);
}

//从EEPROM 指定地址读取一个字节的数据
Uint16 readbyte(Uint16 addr) 				
{
    Uint16 c;    
    begintrans();       					//开始
    bytein(0xA0 + ((addr & 0x0300) >> 7));  //写入写控制字0xA0
    bytein(addr);       					//写入指定地址
    begintrans();       					//开始
    bytein(0xA1);       					//写入读控制字0xA1
    c = byteout();      					//读出EEPROM 输出的数据
    stoptrans();        					//停止
    delay(2000);        					//延时
    return(c);
}

//===========================================================================
// No more.
//===========================================================================
