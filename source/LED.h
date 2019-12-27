//============================================================================================
//LED.h
//SPI LED Driver 3 LED
//kypine QQ:190459556
//2015.10.16
//============================================================================================
#include "DSP2833x_Device.h"		// DSP2803x Headerfile Include File
#include "DSP2833x_Examples.h"		// DSP2803x Examples Include File 

//LED Num Define
#define LEDNo1		0xf7
#define LEDNo2		0xef
#define LEDNo3		0xdf
#define LEDNo4		0xbf
#define LEDNo5		0x7f

//LED Data Define
#define LEDNULL 	0
#define LED0 		0x3f		//0xfc
#define LED1 		0x06		//0x60
#define LED2		0x5b		//0xda
#define LED3   		0x4f		//0xf2
#define LED4 		0x66
#define LED5 		0x6d		//0xb6
#define LED6 		0x7d		//be
#define LED7 		0x07		//e0
#define LED8 		0x7f		//fe
#define LED9 		0x6f		//f6
#define LEDA 		0x77		//ee
#define LEDB 		0x7c		//3e
#define LEDC 		0x58		//1a
#define LEDD 		0x5e		//7a
#define LEDE 		0x79		//9e
#define LEDF 		0x71		//8e
#define LEDG		0x3d		//bc
#define LEDH		0x76		//6e
#define LEDJ		0x0e		//70
#define LEDL		0x38
#define LEDN		0x54		//2a
#define LEDO		0x5c		//3a
#define LEDP		0x73		//ce
#define LEDQ		0x67		//E6
#define LEDR		0x50		//0a
#define LEDT		0x78		//1e
#define LEDU		0x3e		//7c
#define LEDY		0x6e
#define LEDDOT			0x80	//1
#define LEDMINUS		0x40	//2
#define LEDUNDERLINE 	0x08	//10
#define LEDUPLINE		0x01	//80
#define LEDBRACEL		0x20	//4
#define LEDBRACER  		0x02	//40
#define LEDSBRACKETL 	0x10	//8
#define LEDSBRACKETR 	0x04	//20
#define LEDSPACE		0x00	//0

Uint16 CharToLed(Uint16 s)			//Uint16定义在DSP2803x_Device.h
{
	switch(s)
	{
		case 0:
			return LED0;			//#define LED0 0x3f=111111
		case 1:
			return LED1;
		case 2:
			return LED2;
		case 3:
			return LED3;
		case 4:
			return LED4;
		case 5:
			return LED5;
		case 6:
			return LED6;
		case 7:
			return LED7;
		case 8:
			return LED8;
		case 9:
			return LED9;
		default:
			return LEDNULL;
	}
}
/*
//三位数码管程序 
void CharDisplay(Uint16 s, Uint16 LedBuffer[])		
{
	if(s<=99)
	{
		LedBuffer[2]=CharToLed(0);
		LedBuffer[1]=CharToLed(s/10);
		LedBuffer[0]=CharToLed(s%10);
    }
	else if(99<s<=990)
	{
        LedBuffer[2]=CharToLed(s/100);
        LedBuffer[1]=CharToLed((s%100)/10);
		LedBuffer[0]=CharToLed((s%100)%10);
	}
	else
	{
	    LedBuffer[2]=CharToLed(9);
	    LedBuffer[1]=CharToLed(9);
		LedBuffer[0]=CharToLed(9);
	}
}
*/
///*
//四位数码管程序 
void CharDisplay(Uint16 s, Uint16 LedBuffer[])		
{
	if(s<=99)
	{
		LedBuffer[3]=CharToLed(0);
		LedBuffer[2]=CharToLed(0);
		LedBuffer[1]=CharToLed(s/10);
		LedBuffer[0]=CharToLed(s%10);
    }
	else if(99<s<=9990)
	{
        LedBuffer[3]=CharToLed(s/1000);
        LedBuffer[2]=CharToLed((s%1000)/100);
        LedBuffer[1]=CharToLed(((s%1000)%100)/10);
		LedBuffer[0]=CharToLed(((s%1000)%100)%10);
	}
	else
	{
	    LedBuffer[3]=CharToLed(9);
	    LedBuffer[2]=CharToLed(9);
	    LedBuffer[1]=CharToLed(9);
		LedBuffer[0]=CharToLed(9);
	}
}
//*/
//============================================================================================
// No more.
//============================================================================================
