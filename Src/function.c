#include "function.h"
#include "i2c.h"

extern	uint8_t chrTemp[25];
extern	float a[3],w[3],h[3],Angle[3];
char Rx_data[8]={0};
extern uint8_t rx_data[8];
int i=0;
float error_roll,error_yaw;
extern float Startstate_roll,Startstate_yaw;


short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}	
	
	
/**********************
	陀螺仪数据读取及分析9250,JY901
***********************/
void Gyro_Analyse(void)
{
	//0x34，x轴加速度地址，到z轴角度差24
		HAL_I2C_Mem_Read(&hi2c2, (0x50<<1)|1, 0x34, I2C_MEMADD_SIZE_8BIT, chrTemp, 24, 100);//0x51二进制1010001，0xa1二进制10100001，（http://dl.wit-motion.com:2103/#/markdown/preview?document=37ec3ecfdc804a79a1fb99ea215b3913）7.3.2节
		a[0] = (float)CharToShort(&chrTemp[0])/32768*16;
		a[1] = (float)CharToShort(&chrTemp[2])/32768*16;
		a[2] = (float)CharToShort(&chrTemp[4])/32768*16;
		w[0] = (float)CharToShort(&chrTemp[6])/32768*2000;
		w[1] = (float)CharToShort(&chrTemp[8])/32768*2000;
		w[2] = (float)CharToShort(&chrTemp[10])/32768*2000;
		h[0] = CharToShort(&chrTemp[12]);
		h[1] = CharToShort(&chrTemp[14]);
		h[2] = CharToShort(&chrTemp[16]);
		Angle[0] = (float)CharToShort(&chrTemp[18])/32768*180;//x轴roll轴
		Angle[1] = (float)CharToShort(&chrTemp[20])/32768*180;//y轴pitch轴
		Angle[2] = (float)CharToShort(&chrTemp[22])/32768*180;//z轴yaw轴
		HAL_Delay(100);
}


//函数默认	：左右翼->	中间为舵机转角中点	；向前转为CCR减	；向后转为CCR加（翼片降低，该侧翼升高）	；即舵机从前向后转
//							尾翼->	中间为舵机转角中点	；向左转为CCR减	；向右转为CCR加（翼片向左，左拐）	；即舵机从左向右转	
//							CCR1->左翼舵机		；CCR2->右翼舵机			；CCR3->尾翼舵机
//						陀螺仪->	yaw轴	：左负右正（相对于开始储存值）即保证陀螺仪顺时针旋转时yaw轴数值递增

/*****************************
	接收到视觉反馈数据前，舵机保持平稳飞行
	控制roll轴与yaw轴
	避免高低翼及朝向产生较大偏差
	需要测试确定的数值：容差->5	;	角度与ccr倍数关系->10	;	三个舵机中点对应ccr值ccr_middle
*****************************/
void Fly_Steady(ccr_system*CCR)
{
	CCR->ccr1.middle=1300;
	CCR->ccr2.middle=1300;
	CCR->ccr3.middle=1300;
	
	error_roll=Angle[0]-Startstate_roll;
	error_yaw=Angle[2]-Startstate_yaw;
	
	error_roll=(error_roll>0)?(error_roll-5):(error_roll+5);
	error_yaw=(error_yaw>0)?(error_yaw-5):(error_yaw+5);	//一定的容差，数据根据测试而定
	
	CCR->ccr1.angle =error_roll*10+CCR->ccr1.middle;
	CCR->ccr2.angle=-error_roll*10+CCR->ccr2.middle;	//采用高低翼只调节一边的策略，实际数据由测试决定
	CCR->ccr3.angle=error_yaw*10+CCR->ccr3.middle;
	
	if (error_roll>0)
	{
		TIM1->CCR1=CCR->ccr1.angle;
	}			//左翼偏低
	else if (error_roll<0)
	{
		TIM1->CCR2=CCR->ccr2.angle;
	}			//右翼偏低
											//出现高低翼情况，往高翼调
	if (error_yaw!=0)
	{
		TIM1->CCR3=CCR->ccr3.angle;
	}
	
}


/****************************
	接收到视觉反馈数据后，减少上下左右偏差
	通过在当前翼形基础上舵机增减偏转角度来更快响应
	需要测试确定的数值：	舵机每次递增递减ccr值ccr_angle->200
******************************/
void Fly_Change(ccr_system*CCR)
{
	CCR->ccr1.angle=200;
	CCR->ccr2.angle=200;
	CCR->ccr3.angle=200;
	if (Rx_data[0]=='2')	//当前在目标左边
	{
		if (TIM1->CCR3 > CCR->ccr1.max)
		{
			TIM1->CCR3 = CCR->ccr1.max;
		}
		else 
		{
			TIM1->CCR3 += CCR->ccr1.angle;
		}
	}												//舵机左转，尾翼向右，右拐
	else if (Rx_data[0]=='4')
	{
		if (TIM1->CCR3 < CCR->ccr3.min)
		{
			TIM1->CCR3 = CCR->ccr3.min;
		}
		else
		{
			TIM1->CCR3 -= CCR->ccr3.angle;
		}
	}
	if (Rx_data[1]=='2')	//当前在目标上面
	{
		if (TIM1->CCR1 > CCR->ccr1.max || TIM1->CCR2 > CCR->ccr2.max)
		{
			TIM1->CCR1 = CCR->ccr1.max;
			TIM1->CCR2 = CCR->ccr2.max;
		}
		else
		{
			TIM1->CCR1+=CCR->ccr1.angle;
			TIM1->CCR2+=CCR->ccr2.angle;
		}
	}												//舵机前转,抬升两侧翼
	else if (Rx_data[1]=='4')
	{
		if (TIM1->CCR1 < CCR->ccr1.min || TIM1->CCR2 < CCR->ccr2.min)
		{
			TIM1->CCR1 = CCR->ccr1.min;
			TIM1->CCR2 = CCR->ccr2.min;
		}
		else
		{
			TIM1->CCR1-=CCR->ccr1.angle;
			TIM1->CCR2-=CCR->ccr2.angle;
		}
	}
}


