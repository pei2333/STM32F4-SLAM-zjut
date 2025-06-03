#include "stm32f4xx.h"
#include "MLX90640_I2C_Driver.h"
#include "delay.h"
#include "led.h"
/*********************************************************************************************************
//I2C引脚定义
//SCL	GPIO_Pin_2
//SDA	GPIO_Pin_4
*********************************************************************************************************/
//SCL		PG2
#define  	SCL_L      		GPIO_ResetBits(GPIOG, GPIO_Pin_2)
#define  	SCL_H      		GPIO_SetBits(GPIOG, GPIO_Pin_2)

//SDA		PG4
#define  	SDA_L      		GPIO_ResetBits(GPIOG, GPIO_Pin_4)
#define  	SDA_H      		GPIO_SetBits(GPIOG, GPIO_Pin_4)

#define  	SDA_READ()     	GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_4)


static unsigned int I2C_delayTime = 10;
static unsigned int I2C_Freq = 800; // 400kHz
// static unsigned int I2C_Freq = 2000;

static uint8_t i2cData[1664] = {0}; // 832 * 2，这里的1664是根据MLX90640_PIXEL_NUM * 2得到的


void I2C_GPIO_Configuration(void);
void I2C_SDA_Input(void);
void I2C_SDA_Output(void);
void I2C_Wait(void);
void I2C_Delay(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_RepeatedStart(void);
void I2C_SendACK(unsigned char ack);
int I2C_ReceiveACK(void);
int I2C_SendByte(unsigned char dat);
unsigned char I2C_ReceiveByte(void);
void I2C_ReceiveBytes(int nBytes, uint8_t *dataP);

void MLX90640_I2CInit(void){
    I2C_GPIO_Configuration();
}

int MLX90640_I2CGeneralReset(void){
    return 0;
}
int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data){
    uint8_t sa; // slave address
    uint8_t cmd[2] = {0}; // command
    int ack = 0; // ACK
    uint16_t cnt = 0;
    uint16_t i = 0;
    uint16_t *p = data; // 指针指向data
    sa = (slaveAddr << 1); // slave address
    cmd[0] = startAddress >> 8; // start address high byte
    cmd[1] = startAddress & 0x00FF; // start address low byte
    
    // 开始I2C通信
    I2C_Stop();
    I2C_Wait();
    I2C_Start();
    I2C_Wait();
    ack = I2C_SendByte(sa); // 发送从机地址
    if(ack != 0){
        return -1;
    }

    ack = I2C_SendByte(cmd[0]); // 发送高位地址
    if(ack != 0){
        return -1;
    }

    ack = I2C_SendByte(cmd[1]); // 发送低位地址
    if(ack != 0){
        return -1;
    }

    // 重新开始I2C通信
    I2C_RepeatedStart();

    sa = sa | 0x01; // 读取模式

    ack = I2C_SendByte(sa); // 发送从机地址
    if(ack != 0){
        return -1;
    }

    LED1_ON;

    I2C_ReceiveBytes(nMemAddressRead << 1, i2cData); // 接收数据，左移一位因为一个数据占用两个字节
    
    LED1_OFF;
    I2C_Stop();

    LED2_ON;
    i = 0;
    for(cnt = 0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1; // i2cData是int8_t类型，占用一个字节，而data是uint16_t类型，占用两个字节
        *p++ = (int)i2cData[i]*256 + (int)i2cData[i+1]; // 将i2cData中的数据赋值给data，int8->uint16
        if(cnt > nMemAddressRead/2){
            LED3_ON;
        }
        if(cnt > nMemAddressRead*2/3){
            LED5_ON;
        }
    }
    LED2_OFF;
    LED3_OFF;
    LED5_OFF;
    return 0;
}
int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data){
    uint8_t sa;
    int ack = 0;
    char cmd[4] = {0,0,0,0};
    static uint16_t dataCheck;
    int i = 0;

    // slave address
    sa = (slaveAddr << 1);
    cmd[0] = writeAddress >> 8; // 高位地址
    cmd[1] = writeAddress & 0x00FF; // 低位地址
    cmd[2] = data >> 8; // 高位数据
    cmd[3] = data & 0x00FF; // 低位数据

    I2C_Stop();
    I2C_Wait();
    I2C_Start();
    // 发送从机地址
    ack = I2C_SendByte(sa);

    // ack != 0，表示发送失败
    if (ack != 0x00)
    {
        return 1;
    }
    
    // 发送高位地址、低位地址、高位数据、低位数据
    for(i = 0; i<4; i++)
    {
        ack = I2C_SendByte(cmd[i]);
    
        if (ack != 0x00)
        {
            return -1;
        }
    }
    I2C_Stop();
    // 读取数据，判断是否写入成功
    MLX90640_I2CRead(slaveAddr,writeAddress,1, &dataCheck);
    
    // 写入的数据与读取的数据不一致
    if ( dataCheck != data)
    {
        return -2;
    }
    
    return 0;
}
void MLX90640_I2CFreqSet(int freq){ // freq in kHz
    I2C_delayTime = 1000/freq; // 1 / (freq * 1000) * 1e6

    I2C_Freq = freq >> 1;
}

void I2C_GPIO_Configuration(void){
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);   //使能GPIOB时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_4;    //LED对应引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //通用输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //输出推挽
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100MHz
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //无上拉或下拉
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    
    I2C_Stop();
}

void I2C_SDA_Input(void){
    GPIO_InitTypeDef GPIO_InitStructure;
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
}
void I2C_SDA_Output(void){
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
}
void I2C_Delay(void)
{
    delay_us(I2C_delayTime);
}
void I2C_Wait(void){
    unsigned int cnt = 0;
    unsigned int freqCnt = (unsigned int)(13000 / I2C_Freq); //8000 / 400 = 20
    while (freqCnt--);

    // delay_us(2);

    // while(I2C_Freq--);// 实现I2C通信频率
    // for(cnt = 0; cnt < 20; cnt++); // 延时20个tick
    // for(cnt = 0; cnt < I2C_Freq + 20; cnt++);
    // while(freqCnt--)
    // for(cnt=20;cnt>0;cnt--);
    
}
void I2C_Start(void){
    SDA_H;             //	拉高数据线
    SCL_H;             //	拉高时钟线
    I2C_Wait();        //	等待
    I2C_Wait();        //	等待
    SDA_L;             //	产生下降沿
    I2C_Wait();        //	等待
    SCL_L;             //	拉低时钟线
    I2C_Wait();        //	等待
}
void I2C_Stop(void)
{
    SCL_L; // 拉低时钟线
    SDA_L; // 拉低数据线
    I2C_Wait();
    SCL_H; // 拉高时钟线
    I2C_Wait();
    SDA_H; // 拉高数据线
    I2C_Wait();
}
void I2C_RepeatedStart(void)
{
    SCL_L;
    I2C_Wait();
    SDA_H;
    I2C_Wait();
    SCL_H;
    I2C_Wait();
    SDA_L;
    I2C_Wait();
    SCL_L;
}
void I2C_SendACK(unsigned char ack){
    // 0: ACK, 1: NACK
    if(ack == 1){
        SDA_H;
    }else{
        SDA_L;
    }
    I2C_Wait();
    SCL_H;
    I2C_Wait();
    I2C_Wait();
    SCL_L;
    I2C_Wait();
    SDA_H;
}
int I2C_ReceiveACK(void){
    unsigned char errcnt = 20;
    
    I2C_SDA_Input(); // SDA input
    
    // I2C_Delay();
    I2C_Wait();
    
    SCL_H; // 在SCL上升沿时，SDA数据有效
    I2C_Wait();
    // I2C_Delay();
    
    while(SDA_READ()){ // wait for ACK(0)
        errcnt--;
        
        if(!errcnt){
            I2C_Stop();
            return -1;
        }
    }
    
    SCL_L;
    I2C_SDA_Output();
    
    return 0;
}

int I2C_SendByte(unsigned char dat){
    unsigned char i;
    int8_t byte = dat;
    for(i = 0; i < 8; i++){
        I2C_Wait();

        if(byte & 0x80){ // MSB first，从高位开始发送，& 0x80 = 1000 0000
            SDA_H; // 1
        }else{
            SDA_L; // 0
        }
        I2C_Wait();
        SCL_H;
        I2C_Wait();
        I2C_Wait();
        SCL_L;
        
        dat <<= 1; // 从高位开始发送，左移一位
        
    }
    I2C_Wait();
    return I2C_ReceiveACK(); // wait for ACK
}
unsigned char I2C_ReceiveByte(void){
    unsigned char i;
    unsigned char dat = 0;
    
    I2C_SDA_Input();
    
    for(i = 0; i < 8; i++){
        dat <<= 1; // 从高位开始接收，左移一位
        
        SCL_H;
        // I2C_Delay(); // 在SCL上升沿时，SDA数据有效
        I2C_Wait();
        if(SDA_READ()){ // 1
            dat |= 0x01; //这里| 0x01 = 0000 0001，即将最低位置1
        }
        
        SCL_L;
        // I2C_Delay();
        I2C_Wait();
    }
    
    I2C_SDA_Output();
    
    return dat;
}

void I2C_ReceiveBytes(int nBytes, uint8_t *dataP){
    char data;
    int i = 0;
    int j = 0;
    
    for(j=0;j<nBytes;j++)
    {
        I2C_Wait();
        SDA_H;
        data = 0;
        // 接收一个字节，8位
        for(i=0;i<8;i++){
            I2C_Wait();
            SCL_H; // 在SCL上升沿时，SDA数据有效
            I2C_Wait();
            data = data<<1;
            if(SDA_READ() == 1){
                data = data+1; // 将最低位置1
            }
            I2C_Wait();
            SCL_L;
            I2C_Wait();
        }
        // 最后一位
        if(j == (nBytes-1))
        {
            // Send NACK
            I2C_SendACK(1);
        }
        else
        {
            // Send ACK
            I2C_SendACK(0);
        }
        
        *(dataP+j) = data; // 将data赋值给dataP
    }
}
