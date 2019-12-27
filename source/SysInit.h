//==========================================================================================================
//                                          IO口及功能定义程序
//==========================================================================================================
#define mERR GpioDataRegs.GPADAT.bit.GPIO12								//PWM1 IGBT故障

//PWM1,2,3的保护复位控制
#define mOC_RESET_PRO_H() {GpioDataRegs.GPASET.bit.GPIO10 = 1;}			//过流复位输出，正常=H，输出信号
#define mOC_RESET_PRO_L() {GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;}		//过流复位输出，过流复位=L，输出信号

#define		JDQ1_ON()	{GpioDataRegs.GPASET.bit.GPIO15 = 1;}
#define		JDQ1_OFF()	{GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;}

#define mRun_ON()	{GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;}				//GPIO13 置0
#define mRun_OFF()	{GpioDataRegs.GPASET.bit.GPIO13 = 1;}				//GPIO13 置1

#define mErr_ON()	{GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;}				//GPIO23 置0
#define mErr_OFF()	{GpioDataRegs.GPASET.bit.GPIO23 = 1;}				//GPIO23 置1

#define	mPWM_ON()	{EALLOW; \
						EPwm4Regs.TZCLR.bit.OST = 1; \
						EDIS;}
								 
#define	mPWM_OFF()	{EALLOW; \
						EPwm4Regs.TZFRC.bit.OST = 1; \
						EDIS;}

#define	mSPWM_ON()	{EALLOW; \
						EPwm1Regs.TZCLR.bit.OST = 1; \
						EPwm2Regs.TZCLR.bit.OST = 1; \
						EPwm3Regs.TZCLR.bit.OST = 1; \
						EPwm1Regs.TZCLR.bit.INT = 1; \
						EPwm2Regs.TZCLR.bit.INT = 1; \
						EPwm3Regs.TZCLR.bit.INT = 1; \
						EDIS;}
								 
#define	mSPWM_OFF()	{EALLOW; \
						EPwm1Regs.TZFRC.bit.OST = 1; \
						EPwm2Regs.TZFRC.bit.OST = 1; \
						EPwm3Regs.TZFRC.bit.OST = 1; \
						EDIS;}

#define	mSPWMA_ERR()	{EALLOW; \
							EPwm1Regs.TZCLR.bit.OST = 1; \
							EPwm2Regs.TZFRC.bit.OST = 1; \
							EPwm3Regs.TZFRC.bit.OST = 1; \
							EDIS;}






#define KEY1 GpioDataRegs.GPBDAT.bit.GPIO58
#define KEY2 GpioDataRegs.GPBDAT.bit.GPIO59
#define KEY3 GpioDataRegs.GPBDAT.bit.GPIO48

void GPIOInit(void)
{
	EALLOW;

	//mOC_RESET_PRO GPIO10 配置
	GpioCtrlRegs.GPAPUD.bit.GPIO10=0;			//启用 GPIO10 的上拉
	GpioCtrlRegs.GPAMUX1.bit.GPIO10=0;			//GPIO10 定义为IO口
	GpioCtrlRegs.GPADIR.bit.GPIO10=1;			//GPIO10 定义为输出
	GpioDataRegs.GPASET.bit.GPIO10 = 1;			//GPIO10 置1

	//mPWM GPIO15 配置
	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;			//启用 GPIO15 的上拉
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;		//GPIO15 定义为IO口
	GpioCtrlRegs.GPADIR.bit.GPIO15=1;			//GPIO15 定义为输出
	GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;		//GPIO15 置0

	//mRun GPIO60 配置
	GpioCtrlRegs.GPAPUD.bit.GPIO13=0;			//启用 GPIO13 的上拉
	GpioCtrlRegs.GPAMUX1.bit.GPIO13=0;			//GPIO13 定义为IO口
	GpioCtrlRegs.GPADIR.bit.GPIO13=1;			//GPIO13 定义为输出
	GpioDataRegs.GPASET.bit.GPIO13 = 1;			//GPIO13 置1

	//mErr GPIO23 配置
	GpioCtrlRegs.GPAPUD.bit.GPIO23=0;			//启用 GPIO23 的上拉
	GpioCtrlRegs.GPAMUX2.bit.GPIO23=0;			//GPIO23 定义为IO口
	GpioCtrlRegs.GPADIR.bit.GPIO23=1;			//GPIO23 定义为输出
	GpioDataRegs.GPASET.bit.GPIO23 = 1;			//GPIO23 置1

	//KEY1 IO配置
	GpioCtrlRegs.GPBPUD.bit.GPIO58=0;			//启用 GPIO49 的上拉
	GpioCtrlRegs.GPBMUX2.bit.GPIO58=0;			//GPIO49 定义为IO口
	GpioCtrlRegs.GPBDIR.bit.GPIO58=0;			//GPIO49 定义为输入
	//KEY2 IO配置
	GpioCtrlRegs.GPBPUD.bit.GPIO59=0;			//启用 GPIO59 的上拉
	GpioCtrlRegs.GPBMUX2.bit.GPIO59=0;			//GPIO59 定义为IO口
	GpioCtrlRegs.GPBDIR.bit.GPIO59=0;			//GPIO59 定义为输入
	//KEY3 IO配置
	GpioCtrlRegs.GPBPUD.bit.GPIO48=0;			//启用 GPIO48 的上拉
	GpioCtrlRegs.GPBMUX2.bit.GPIO48=0;			//GPIO48 定义为IO口
	GpioCtrlRegs.GPBDIR.bit.GPIO48=0;			//GPIO48 定义为输入

	//mERR IO配置
	GpioCtrlRegs.GPAPUD.bit.GPIO12=0;			//启用 GPIO12 的上拉
	GpioCtrlRegs.GPAMUX1.bit.GPIO12=0;			//GPIO12 定义为IO口
	GpioCtrlRegs.GPADIR.bit.GPIO12=0;			//GPIO12 定义为输入

	EDIS;
}
//==========================================================================================================
//                                          IO口及LCD功能定义程序 END
//==========================================================================================================
