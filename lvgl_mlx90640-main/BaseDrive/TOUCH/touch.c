#include "touch.h"
#include "lcd.h"
#include "delay.h"
#include "stdlib.h"
#include "math.h"
#include "../24CXX/24cxx.h"

_m_tp_dev tp_dev =
{
    TP_Init,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

//////////////////////////////////////////////////////////////////////////////////
//与LCD部分有关的函数
//画一个触摸点
//用来校准用的
//x,y:坐标
//color:颜色
void TP_Drow_Touch_Point(u16 x, u16 y, u16 color)
{
    POINT_COLOR = color;
    LCD_DrawLine(x - 12, y, x + 13, y); //横线
    LCD_DrawLine(x, y - 12, x, y + 13); //竖线
    LCD_DrawPoint(x + 1, y + 1);
    LCD_DrawPoint(x - 1, y + 1);
    LCD_DrawPoint(x + 1, y - 1);
    LCD_DrawPoint(x - 1, y - 1);
    LCD_Draw_Circle(x, y, 6); //画中心圈
}
//画一个大点(2*2的点)
//x,y:坐标
//color:颜色
void TP_Draw_Big_Point(u16 x, u16 y, u16 color)
{
    POINT_COLOR = color;
    LCD_DrawPoint(x, y); //中心点
    LCD_DrawPoint(x + 1, y);
    LCD_DrawPoint(x, y + 1);
    LCD_DrawPoint(x + 1, y + 1);
}


//触摸屏初始化
//返回值:0,没有进行校准
//       1,进行过校准
u8 TP_Init(void)
{
    if(lcddev.id == 0X5510)		//电容触摸屏
    {
        if(GT9147_Init() == 0)	//是GT9147
        {
            tp_dev.scan = GT9147_Scan;	//扫描函数指向GT9147触摸屏扫描
        }

        tp_dev.touchtype |= 0X80;	//电容屏
        tp_dev.touchtype |= lcddev.dir & 0X01; //横屏还是竖屏
        return 0;
    }

    return 1;
}

