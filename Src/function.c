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
	���������ݶ�ȡ������9250,JY901
***********************/
void Gyro_Analyse(void)
{
	//0x34��x����ٶȵ�ַ����z��ǶȲ�24
		HAL_I2C_Mem_Read(&hi2c2, (0x50<<1)|1, 0x34, I2C_MEMADD_SIZE_8BIT, chrTemp, 24, 100);//0x51������1010001��0xa1������10100001����http://dl.wit-motion.com:2103/#/markdown/preview?document=37ec3ecfdc804a79a1fb99ea215b3913��7.3.2��
		a[0] = (float)CharToShort(&chrTemp[0])/32768*16;
		a[1] = (float)CharToShort(&chrTemp[2])/32768*16;
		a[2] = (float)CharToShort(&chrTemp[4])/32768*16;
		w[0] = (float)CharToShort(&chrTemp[6])/32768*2000;
		w[1] = (float)CharToShort(&chrTemp[8])/32768*2000;
		w[2] = (float)CharToShort(&chrTemp[10])/32768*2000;
		h[0] = CharToShort(&chrTemp[12]);
		h[1] = CharToShort(&chrTemp[14]);
		h[2] = CharToShort(&chrTemp[16]);
		Angle[0] = (float)CharToShort(&chrTemp[18])/32768*180;//x��roll��
		Angle[1] = (float)CharToShort(&chrTemp[20])/32768*180;//y��pitch��
		Angle[2] = (float)CharToShort(&chrTemp[22])/32768*180;//z��yaw��
		HAL_Delay(100);
}


//����Ĭ��	��������->	�м�Ϊ���ת���е�	����ǰתΪCCR��	�����תΪCCR�ӣ���Ƭ���ͣ��ò������ߣ�	���������ǰ���ת
//							β��->	�м�Ϊ���ת���е�	������תΪCCR��	������תΪCCR�ӣ���Ƭ������գ�	���������������ת	
//							CCR1->������		��CCR2->������			��CCR3->β����
//						������->	yaw��	��������������ڿ�ʼ����ֵ������֤������˳ʱ����תʱyaw����ֵ����

/*****************************
	���յ��Ӿ���������ǰ���������ƽ�ȷ���
	����roll����yaw��
	����ߵ�����������ϴ�ƫ��
	��Ҫ����ȷ������ֵ���ݲ�->5	;	�Ƕ���ccr������ϵ->10	;	��������е��Ӧccrֵccr_middle
*****************************/
void Fly_Steady(ccr_system*CCR)
{
	CCR->ccr1.middle=1300;
	CCR->ccr2.middle=1300;
	CCR->ccr3.middle=1300;
	
	error_roll=Angle[0]-Startstate_roll;
	error_yaw=Angle[2]-Startstate_yaw;
	
	error_roll=(error_roll>0)?(error_roll-5):(error_roll+5);
	error_yaw=(error_yaw>0)?(error_yaw-5):(error_yaw+5);	//һ�����ݲ���ݸ��ݲ��Զ���
	
	CCR->ccr1.angle =error_roll*10+CCR->ccr1.middle;
	CCR->ccr2.angle=-error_roll*10+CCR->ccr2.middle;	//���øߵ���ֻ����һ�ߵĲ��ԣ�ʵ�������ɲ��Ծ���
	CCR->ccr3.angle=error_yaw*10+CCR->ccr3.middle;
	
	if (error_roll>0)
	{
		TIM1->CCR1=CCR->ccr1.angle;
	}			//����ƫ��
	else if (error_roll<0)
	{
		TIM1->CCR2=CCR->ccr2.angle;
	}			//����ƫ��
											//���ָߵ���������������
	if (error_yaw!=0)
	{
		TIM1->CCR3=CCR->ccr3.angle;
	}
	
}


/****************************
	���յ��Ӿ��������ݺ󣬼�����������ƫ��
	ͨ���ڵ�ǰ���λ����϶������ƫת�Ƕ���������Ӧ
	��Ҫ����ȷ������ֵ��	���ÿ�ε����ݼ�ccrֵccr_angle->200
******************************/
void Fly_Change(ccr_system*CCR)
{
	CCR->ccr1.angle=200;
	CCR->ccr2.angle=200;
	CCR->ccr3.angle=200;
	if (Rx_data[0]=='2')	//��ǰ��Ŀ�����
	{
		if (TIM1->CCR3 > CCR->ccr1.max)
		{
			TIM1->CCR3 = CCR->ccr1.max;
		}
		else 
		{
			TIM1->CCR3 += CCR->ccr1.angle;
		}
	}												//�����ת��β�����ң��ҹ�
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
	if (Rx_data[1]=='2')	//��ǰ��Ŀ������
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
	}												//���ǰת,̧��������
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


