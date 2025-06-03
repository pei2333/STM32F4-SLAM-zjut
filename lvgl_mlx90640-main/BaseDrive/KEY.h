#ifndef _KEY_H_
#define _KEY_H_

/*KEY端口定义
//KEY_S1      PF8
//KEY_S2      PF9
//KEY_S3      PF10
*/
#define  	KEY_S1_READ()  	GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_8)
#define  	KEY_S2_READ()  	GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_9)
#define  	KEY_S3_READ()  	GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10)

// 按键按下状态检测宏（低电平有效）
#define     KEY1_PRESSED()   (KEY_S1_READ() == 0)
#define     KEY2_PRESSED()   (KEY_S2_READ() == 0)
#define     KEY3_PRESSED()   (KEY_S3_READ() == 0)

void KEY_Init(void);
unsigned char KeyScan(void);

#endif
