#include "adxl345.h"
#include "delay.h"
#include  <math.h>  
#include "main.h"

/*************************************************************************
函数名称：发送端初始化函数
输入参数：无
输出参数：无
函数返回：无
//ADXL345硬件资源引脚定义
//ADXL345_CS(PF14)     OUT
//ADXL345_CLK(PF11)    OUT
//ADXL345_DIN(PF12)    OUT
//ADXL345_DO(PF13)     IN
*************************************************************************/
void ADXL345_ISP_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    //ADXL345_CS(PF14)、ADXL345_SCK(PF11)、//ADXL345_DO(PF12)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF , ENABLE);            //
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    //ADXL345_DIN(PF13)    IN
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;           //输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    ADXL345_CS_H;
    ADXL345_CLK_H;
    ADXL345_DIN_H;

}

/*************************************************************************
函数名称：SPI产生时钟
输入参数：无
输出参数：无
函数返回：无
*************************************************************************/
void spi_clk(void)
{
    ADXL345_CLK_H;
    delay_us(20);
    
	ADXL345_CLK_L;
    delay_us(20);

}
/*************************************************************************
函数名称：SPI读写数据函数
输入参数：写入的数据
输出参数：无
函数返回：读取到的数据
*************************************************************************/
unsigned char SPI_RW_Byte(unsigned char dat)
{
    u8 i;
	u8 temp = 0;
    
    for(i = 0; i < 8; i++)
    {   
        if(dat & 0x80)//位运算，判断最高位是否为1
        {
            ADXL345_DIN_H;
        }
        else
        {
            ADXL345_DIN_L;
        }
        
        // 将数据左移，为下一个位准备
        dat <<= 1;
              
        
        ADXL345_CLK_H; // 时钟高
        
        temp <<= 1;
        
        if (ADXL345_DO) 
        {
            temp |= 0x01; // 露脕取碌陆1拢卢路诺陆战确位謨
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
函数名称：发送端写数据函数
输入参数：寄存器地址、数据
输出参数：无
函数返回：寄存器状态
***************************************************************************/
void ADXL345WriteReg(unsigned char addr, unsigned char val) 
{    
    // 置低CSN，使能SPI传输
    ADXL345_CS_L;
    delay_us(20);
    
	spi_clk();
    
    // 写寄存器
    addr &= 0x3F;
    SPI_RW_Byte(addr);
    
    delay_us(20);

    // 向寄存器写入数据
//    dat1 = val & 0x7F;  
    SPI_RW_Byte(val);

    ADXL345_CLK_H; // 时钟高
    
    // CSN拉高，完成
    ADXL345_CS_H;
    
    delay_us(20);
}

/***************************************************************************
函数名称：发送端读取1字节数据函数
输入参数：寄存器地址
输出参数：无
函数返回：读取到的数据
***************************************************************************/
unsigned char ADXL345ReadReg(unsigned char addr) 
{
    u8 reg_val;
	 
    //置低CSN，使能SPI传输
    ADXL345_CS_L;
    delay_us(20);
    
	spi_clk();
     
    addr |= 0x80;
    
    SPI_RW_Byte(addr);
        
    //读取寄存器的值
    reg_val = SPI_RW_Byte(NOP);

    ADXL345_CLK_H; // 时钟高
    
    //CSN拉高，完成
    ADXL345_CS_H;
    delay_us(20);

    return reg_val;
}

/**********************************************************************************************************
函数名称：ADXL345初始化
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
unsigned char ADXL345_Init(void)
{
    u8 temp = 0;
    	
    temp = ADXL345ReadReg(DEVICE_ID);     //  读取器件ID   
    if(temp == 0xE5)                        //  器件ID=0xE5
	{  
		ADXL345WriteReg(DATA_FORMAT, 0x2B); //  低电平中断输出,13位全分辨率,输出数据右对齐,16g量程 
		ADXL345WriteReg(BW_RATE, 0x0A);     //  数据输出速度为100Hz
		ADXL345WriteReg(POWER_CTL, 0x28);   //  链接使能,测量模式
		ADXL345WriteReg(INT_ENABLE, 0x00);  //  不使用中断		 
	 	
        ADXL345WriteReg(OFSX, 0x00);        //  X轴偏移
		ADXL345WriteReg(OFSY, 0x00);        //  Y轴偏移
		ADXL345WriteReg(OFSZ, 0x00);	    //  Z轴偏移
		
        return 0;
	}
    
	return 1;	
}

/**********************************************************************************************************
函数名称：读取加速度数据
输入参数：数据缓冲区
输出参数：无
函数返回：无
**********************************************************************************************************/
//读取ADXL的平均值
//x,y,z:读取10次后取平均值
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
函数名称：自动校准
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/ 
void ADXL345_AUTO_Adjust(char *xval, char *yval, char *zval)
{
	short tx,ty,tz;
	u8 i;
	short offx = 0, offy = 0, offz = 0;
    
	ADXL345WriteReg(POWER_CTL, 0x00);	   	                            //  先进入休眠模式.
	delay_ms(100);
	
    ADXL345WriteReg(DATA_FORMAT, 0x2B);	                                //  低电平中断输出,13位全分辨率,输出数据右对齐,16g量程 
	ADXL345WriteReg(BW_RATE, 0x0A);		                                //  数据输出速度为100Hz
	ADXL345WriteReg(POWER_CTL, 0x28);	   	                            //  链接使能,测量模式
	ADXL345WriteReg(INT_ENABLE, 0x00);	                                //  不使用中断		 

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
函数名称：读取三个轴的数据
输入参数：无
输出参数：无
函数返回：无
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
    
    *x = (short)(((short)x1 << 8)+ x0);                              // DATA X1为高位有效字节
    *y = (short)(((short)y1 << 8)+ y0);                              // DATA Y1为高位有效字节
    *z = (short)(((short)z1 << 8)+ z0);                              // DATA Z1为高位有效字节
}

/**********************************************************************************************************
函数名称：计算角度值
输入参数：无
输出参数：无
函数返回：无
函数说明：
x,y,z:x,y,z方向的重力加速度分量(不需要单位,直接数值即可)
dir:要获得的角度.0,与Z轴的角度;1,与X轴的角度;2,与Y轴的角度.
返回值:角度值.单位0.1°.
**********************************************************************************************************/
short ADXL345Get_Angle(float x, float y, float z, unsigned char dir)
{
	float temp;
 	float res = 0;
    
    short value = 0;
    
	switch(dir)
	{
		case    0:                                                      //  与自然Z轴的角度
                temp = sqrt((x*x + y*y))/z;
                res  = atan(temp);
                break;
        
        case    1:                                                      //  与自然X轴的角度
                temp = x/sqrt((y*y + z*z));
                res = atan(temp);
                break;
        
 		case    2:                                                      //  与自然Y轴的角度
                temp = y/sqrt((x*x + z*z));
                res = atan(temp);
                break;
 	}
    
    value = res*1800/3.14;
    
	return value;
}

