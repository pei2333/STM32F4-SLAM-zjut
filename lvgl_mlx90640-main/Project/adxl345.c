#include "adxl345.h"
#include "delay.h"
#include  <math.h>  
#include "main.h"

/*************************************************************************
�������ƣ����Ͷ˳�ʼ������
�����������
�����������
�������أ���
//ADXL345Ӳ����Դ���Ŷ���
//ADXL345_CS(PF14)     OUT
//ADXL345_CLK(PF11)    OUT
//ADXL345_DIN(PF12)    OUT
//ADXL345_DO(PF13)     IN
*************************************************************************/
void ADXL345_ISP_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    //ADXL345_CS(PF14)��ADXL345_SCK(PF11)��//ADXL345_DO(PF12)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF , ENABLE);            //
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    //ADXL345_DIN(PF13)    IN
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;           //����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    ADXL345_CS_H;
    ADXL345_CLK_H;
    ADXL345_DIN_H;

}

/*************************************************************************
�������ƣ�SPI����ʱ��
�����������
�����������
�������أ���
*************************************************************************/
void spi_clk(void)
{
    ADXL345_CLK_H;
    delay_us(20);
    
	ADXL345_CLK_L;
    delay_us(20);

}
/*************************************************************************
�������ƣ�SPI��д���ݺ���
���������д�������
�����������
�������أ���ȡ��������
*************************************************************************/
unsigned char SPI_RW_Byte(unsigned char dat)
{
    u8 i;
	u8 temp = 0;
    
    for(i = 0; i < 8; i++)
    {   
        if(dat & 0x80)//λ���㣬�ж����λ�Ƿ�Ϊ1
        {
            ADXL345_DIN_H;
        }
        else
        {
            ADXL345_DIN_L;
        }
        
        // ���������ƣ�Ϊ��һ��λ׼��
        dat <<= 1;
              
        
        ADXL345_CLK_H; // ʱ�Ӹ�
        
        temp <<= 1;
        
        if (ADXL345_DO) 
        {
            temp |= 0x01; // ¶Áȡµ½1£¬·ŵ½սȷλփ
        }
 
        delay_us(20);
		
        ADXL345_CLK_L;
        delay_us(20);        
    } 
    
    delay_us(20); 
        
    ADXL345_DIN_L;
    
    return temp;
}


/***************************************************************************
�������ƣ����Ͷ�д���ݺ���
����������Ĵ�����ַ������
�����������
�������أ��Ĵ���״̬
***************************************************************************/
void ADXL345WriteReg(unsigned char addr, unsigned char val) 
{    
    // �õ�CSN��ʹ��SPI����
    ADXL345_CS_L;
    delay_us(20);
    
	spi_clk();
    
    // д�Ĵ���
    addr &= 0x3F;
    SPI_RW_Byte(addr);
    
    delay_us(20);

    // ��Ĵ���д������
//    dat1 = val & 0x7F;  
    SPI_RW_Byte(val);

    ADXL345_CLK_H; // ʱ�Ӹ�
    
    // CSN���ߣ����
    ADXL345_CS_H;
    
    delay_us(20);
}

/***************************************************************************
�������ƣ����Ͷ˶�ȡ1�ֽ����ݺ���
����������Ĵ�����ַ
�����������
�������أ���ȡ��������
***************************************************************************/
unsigned char ADXL345ReadReg(unsigned char addr) 
{
    u8 reg_val;
	 
    //�õ�CSN��ʹ��SPI����
    ADXL345_CS_L;
    delay_us(20);
    
	spi_clk();
     
    addr |= 0x80;
    
    SPI_RW_Byte(addr);
        
    //��ȡ�Ĵ�����ֵ
    reg_val = SPI_RW_Byte(NOP);

    ADXL345_CLK_H; // ʱ�Ӹ�
    
    //CSN���ߣ����
    ADXL345_CS_H;
    delay_us(20);

    return reg_val;
}

/**********************************************************************************************************
�������ƣ�ADXL345��ʼ��
�����������
�����������
�������أ���
**********************************************************************************************************/
unsigned char ADXL345_Init(void)
{
    u8 temp = 0;
    	
    temp = ADXL345ReadReg(DEVICE_ID);     //  ��ȡ����ID   
    if(temp == 0xE5)                        //  ����ID=0xE5
	{  
		ADXL345WriteReg(DATA_FORMAT, 0x2B); //  �͵�ƽ�ж����,13λȫ�ֱ���,��������Ҷ���,16g���� 
		ADXL345WriteReg(BW_RATE, 0x0A);     //  ��������ٶ�Ϊ100Hz
		ADXL345WriteReg(POWER_CTL, 0x28);   //  ����ʹ��,����ģʽ
		ADXL345WriteReg(INT_ENABLE, 0x00);  //  ��ʹ���ж�		 
	 	
        ADXL345WriteReg(OFSX, 0x00);        //  X��ƫ��
		ADXL345WriteReg(OFSY, 0x00);        //  Y��ƫ��
		ADXL345WriteReg(OFSZ, 0x00);	    //  Z��ƫ��
		
        return 0;
	}
    
	return 1;	
}

/**********************************************************************************************************
�������ƣ���ȡ���ٶ�����
������������ݻ�����
�����������
�������أ���
**********************************************************************************************************/
//��ȡADXL��ƽ��ֵ
//x,y,z:��ȡ10�κ�ȡƽ��ֵ
void ADXL345ReadAvval(short *x, short *y, short *z)
{
	short tx = 0, ty = 0, tz = 0;	   
	u8 i;  
    
	for(i = 0; i < 10; i++)
	{
		ADXL345Read_XYZ(x, y, z);
        
		delay_ms(10);
		
        tx += (short)*x;
		ty += (short)*y;
		tz += (short)*z;	   
	}
    
	*x = tx/10;
	*y = ty/10;
	*z = tz/10;
} 


/**********************************************************************************************************
�������ƣ��Զ�У׼
�����������
�����������
�������أ���
**********************************************************************************************************/ 
void ADXL345_AUTO_Adjust(char *xval, char *yval, char *zval)
{
	short tx,ty,tz;
	u8 i;
	short offx = 0, offy = 0, offz = 0;
    
	ADXL345WriteReg(POWER_CTL, 0x00);	   	                            //  �Ƚ�������ģʽ.
	delay_ms(100);
	
    ADXL345WriteReg(DATA_FORMAT, 0x2B);	                                //  �͵�ƽ�ж����,13λȫ�ֱ���,��������Ҷ���,16g���� 
	ADXL345WriteReg(BW_RATE, 0x0A);		                                //  ��������ٶ�Ϊ100Hz
	ADXL345WriteReg(POWER_CTL, 0x28);	   	                            //  ����ʹ��,����ģʽ
	ADXL345WriteReg(INT_ENABLE, 0x00);	                                //  ��ʹ���ж�		 

	ADXL345WriteReg(OFSX, 0x00);
	ADXL345WriteReg(OFSY, 0x00);
	ADXL345WriteReg(OFSZ, 0x00);
    
	delay_ms(12);
    
	for(i = 0; i < 10; i++)
	{
		ADXL345ReadAvval(&tx, &ty, &tz);
        
		offx += tx;
		offy += ty;
		offz += tz;
	}	 		
    
	offx /= 10;
	offy /= 10;
	offz /= 10;
    
	*xval = -offx/4;
	*yval = -offy/4;
	*zval = -(offz - 256)/4;	  
    
 	ADXL345WriteReg(OFSX, *xval);
	ADXL345WriteReg(OFSY, *yval);
	ADXL345WriteReg(OFSZ, *zval);	
}  


/**********************************************************************************************************
�������ƣ���ȡ�����������
�����������
�����������
�������أ���
**********************************************************************************************************/ 
void ADXL345Read_XYZ(short *x, short *y, short *z)				
{    
    u8 x0, y0, z0;
    u8 x1, y1, z1;
    
    x0 = ADXL345ReadReg(DATA_X0); 
    y0 = ADXL345ReadReg(DATA_Y0); 
    z0 = ADXL345ReadReg(DATA_Z0); 
    
    x1 = ADXL345ReadReg(DATA_X1); 
    y1 = ADXL345ReadReg(DATA_Y1); 
    z1 = ADXL345ReadReg(DATA_Z1); 
    
    *x = (short)(((short)x1 << 8)+ x0);                              // DATA X1Ϊ��λ��Ч�ֽ�
    *y = (short)(((short)y1 << 8)+ y0);                              // DATA Y1Ϊ��λ��Ч�ֽ�
    *z = (short)(((short)z1 << 8)+ z0);                              // DATA Z1Ϊ��λ��Ч�ֽ�
}

/**********************************************************************************************************
�������ƣ�����Ƕ�ֵ
�����������
�����������
�������أ���
����˵����
x,y,z:x,y,z������������ٶȷ���(����Ҫ��λ,ֱ����ֵ����)
dir:Ҫ��õĽǶ�.0,��Z��ĽǶ�;1,��X��ĽǶ�;2,��Y��ĽǶ�.
����ֵ:�Ƕ�ֵ.��λ0.1��.
**********************************************************************************************************/
short ADXL345Get_Angle(float x, float y, float z, unsigned char dir)
{
	float temp;
 	float res = 0;
    
    short value = 0;
    
	switch(dir)
	{
		case    0:                                                      //  ����ȻZ��ĽǶ�
                temp = sqrt((x*x + y*y))/z;
                res  = atan(temp);
                break;
        
        case    1:                                                      //  ����ȻX��ĽǶ�
                temp = x/sqrt((y*y + z*z));
                res = atan(temp);
                break;
        
 		case    2:                                                      //  ����ȻY��ĽǶ�
                temp = y/sqrt((x*x + z*z));
                res = atan(temp);
                break;
 	}
    
    value = res*1800/3.14;
    
	return value;
}

