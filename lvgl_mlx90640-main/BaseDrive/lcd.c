// #include "LCD.h"
// #include "Hzk16song.h"
// #include "font.h"

// //LCD?????????
// u16 POINT_COLOR = 0x0000;	                                            // ????
// u16 BACK_COLOR  = 0xFFFF;                                               // ???


// //	LCD?????????
// lcd_dev lcddev;

// /**********************************************************************************************************
// ???????us????
// ???????
// ??????
// **********************************************************************************************************/
// void LCD_delayus(unsigned int time)
// {  
//    unsigned short i=0; 
    
//    while(time--)
//    {
//       i = 8;  
//       while(i--) ;    
//    }
// }

// /**********************************************************************************************************
// ?????LCD???????1ms?
// ???????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_delayms(unsigned int cnt)
// {
// 	volatile unsigned int dl;
    
// 	while(cnt--)
// 	{
// 		for(dl = 0; dl < 20000; dl++);
// 	}
// }


// /**********************************************************************************************************
// ???????????
// ?????????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_WR_REG(u16 regval)
// {
//     LCD->LCD_REG = regval; //??????????
// }

// /**********************************************************************************************************
// ??????????
// ???????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_WR_DATA(u16 data)
// {
//     LCD->LCD_RAM = data;
// }


// /**********************************************************************************************************
// ??????????
// ??????
// ??????
// ?????????
// **********************************************************************************************************/
// u16 LCD_RD_DATA(void)
// {
//     vu16 ram;															//	?????
	
//     ram = LCD->LCD_RAM;
    
// 	return ram;
// }


// /**********************************************************************************************************
// ?????????????
// ????????????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
// {
//     LCD->LCD_REG = LCD_Reg;												//	??????????
//     LCD->LCD_RAM = LCD_RegValue;										//	????
// }

// /**********************************************************************************************************
// ?????????????
// ?????????
// ??????
// ??????????
// **********************************************************************************************************/
// u16 LCD_ReadReg(u16 LCD_Reg)
// {
//     LCD_WR_REG(LCD_Reg);												//	??????????
//     LCD_delayus(5);
	
//     return LCD_RD_DATA();												//	??????
// }

// /**********************************************************************************************************
// ??????GRAM????
// ??????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_WriteRAM_Prepare(void)
// {
//     LCD->LCD_REG = lcddev.wramcmd;
// }


// /**********************************************************************************************************
// ??????RAM????
// ????????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_WriteRAM(u16 RGB_Code)
// {
//     LCD->LCD_RAM = RGB_Code;											//	????GRAM
// }


// ///**********************************************************************************************************
// //?????RGB????
// //?????GBR??????
// //??????
// //?????RGB??????
// //??????ILI93xx??????GBR????????????RGB???
// //**********************************************************************************************************/
// //u16 LCD_BGR2RGB(u16 c)
// //{
// //    u16  r, g, b, rgb;
// //	
// //    b = (c >> 0) & 0x1f;
// //    g = (c >> 5) & 0x3f;
// //    r = (c >> 11) & 0x1f;
// //    
// //	rgb = (b << 11) + (g << 5) + (r << 0);
// //    
// //	return(rgb);
// //}


// ///**********************************************************************************************************
// //?????????
// //???????
// //??????
// //??????
// //**********************************************************************************************************/
// //void opt_delay(u8 i)
// //{
// //    while(i--);
// //}


// ///**********************************************************************************************************
// //??????????????
// //?????x,y:??
// //??????
// //??????????
// //**********************************************************************************************************/
// //u16 LCD_ReadPoint(u16 x, u16 y)
// //{
// //    u16 r = 0;
// //	
// //    if(x >= lcddev.width || y >= lcddev.height)
// //    {
// //        return 0;														//	?????,????
// //    }
// //    
// //    LCD_SetCursor(x, y);
// //    
// //    LCD_WR_REG(0X2E); 												    //	???GRAM??
// //    
// //    r = LCD_RD_DATA();													//	dummy Read
// //    
// //    return r;
// //}


// /**********************************************************************************************************
// ???????????
// ?????Xpos:???
// 		  Ypos:???
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_SetCursor(u16 Xpos, u16 Ypos)
// {
//     if(lcddev.id == 0X5510)
//     {
//         LCD_WR_REG(lcddev.setxcmd);
//         LCD_WR_DATA(Xpos >> 8);
//         LCD_WR_REG(lcddev.setxcmd + 1);
//         LCD_WR_DATA(Xpos & 0XFF);
//         LCD_WR_REG(lcddev.setycmd);
//         LCD_WR_DATA(Ypos >> 8);
//         LCD_WR_REG(lcddev.setycmd + 1);
//         LCD_WR_DATA(Ypos & 0XFF);
//     }
// }

// /**********************************************************************************************************
// ???????LCD???????
// ???????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_Scan_Dir(u8 dir)
// {
//     u16 regval = 0;
//     u16 dirreg = 0;

//     if(lcddev.id == 0X5510)                                             // 5510,????
//     {
//         switch(dir)
//         {
//         case    L2R_U2D://????,????
//                 regval |= (0 << 7) | (0 << 6) | (0 << 5);
//                 break;
        
//         case    L2R_D2U://????,????
//                 regval |= (1 << 7) | (0 << 6) | (0 << 5);
//                 break;
        
//         case    R2L_U2D://????,????
//                 regval |= (0 << 7) | (1 << 6) | (0 << 5);
//                 break;
        
//         case    R2L_D2U://????,????
//                 regval |= (1 << 7) | (1 << 6) | (0 << 5);
//                 break;
        
//         case    U2D_L2R://????,????
//                 regval |= (0 << 7) | (0 << 6) | (1 << 5);
//                 break;
        
//         case    U2D_R2L://????,????
//                 regval |= (0 << 7) | (1 << 6) | (1 << 5);
//                 break;
        
//         case    D2U_L2R://????,????
//                 regval |= (1 << 7) | (0 << 6) | (1 << 5);
//                 break;
        
//         case    D2U_R2L://????,????
//                 regval |= (1 << 7) | (1 << 6) | (1 << 5);
//                 break;
//         }

//         if(lcddev.id == 0X5510)
//         {
//             dirreg = 0X3600;
//         }
//         if(lcddev.id != 0X5510)
//         {
//             regval |= 0X08;                                             // 5510???BGR
//         }

//         LCD_WriteReg(dirreg, regval);

//         if(lcddev.id == 0X5510)
//         {
//             LCD_WR_REG(lcddev.setxcmd);
//             LCD_WR_DATA(0);
//             LCD_WR_REG(lcddev.setxcmd + 1);
//             LCD_WR_DATA(0);
//             LCD_WR_REG(lcddev.setxcmd + 2);
//             LCD_WR_DATA((lcddev.width - 1) >> 8);
//             LCD_WR_REG(lcddev.setxcmd + 3);
//             LCD_WR_DATA((lcddev.width - 1) & 0XFF);
//             LCD_WR_REG(lcddev.setycmd);
//             LCD_WR_DATA(0);
//             LCD_WR_REG(lcddev.setycmd + 1);
//             LCD_WR_DATA(0);
//             LCD_WR_REG(lcddev.setycmd + 2);
//             LCD_WR_DATA((lcddev.height - 1) >> 8);
//             LCD_WR_REG(lcddev.setycmd + 3);
//             LCD_WR_DATA((lcddev.height - 1) & 0XFF);
//         }
//     }
// }

// /**********************************************************************************************************
// ???????
// ?????x,y	??
// 		  color ??
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_DrawPoint(u16 x, u16 y, u16 color)
// {
//     LCD_SetCursor(x, y);												//	??????
//     LCD_WriteRAM_Prepare();												//	????GRAM
//     LCD->LCD_RAM = color;
// }

// /**********************************************************************************************************
// ?????????
// ?????x,y:	??
// 		  color ??
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_Fast_DrawPoint(u16 x, u16 y, u16 color)
// {    
//     if(lcddev.id == 0X5510)
//     {
//         LCD_WR_REG(lcddev.setxcmd);
//         LCD_WR_DATA(x >> 8);
//         LCD_WR_REG(lcddev.setxcmd + 1);
//         LCD_WR_DATA(x & 0XFF);
//         LCD_WR_REG(lcddev.setycmd);
//         LCD_WR_DATA(y >> 8);
//         LCD_WR_REG(lcddev.setycmd + 1);
//         LCD_WR_DATA(y & 0XFF);
//     }
    
//     LCD->LCD_REG = lcddev.wramcmd;
//     LCD->LCD_RAM = color;
// }

// //??LCD????
// //dir:0,???1,??
// /**********************************************************************************************************
// ???????LCD????
// ?????dir:0,???1,??
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_Display_Dir(u8 dir)
// {
//     if(dir == 0)														//	??
//     {
//         lcddev.dir = 0;	                                                // ??
//         lcddev.width = 240;
//         lcddev.height = 320;

//         if(lcddev.id == 0x5510)
//         {
//             lcddev.wramcmd = 0X2C00;
//             lcddev.setxcmd = 0X2A00;
//             lcddev.setycmd = 0X2B00;
//             lcddev.width = 480;
//             lcddev.height = 800;
//         }
//         LCD_Scan_Dir(R2L_D2U);
//     }
//     else 																//	??
//     {
//         lcddev.dir = 1;	                                                // ??
//         lcddev.width = 320;
//         lcddev.height = 240;

//         if(lcddev.id == 0x5510)
//         {
//             lcddev.wramcmd = 0X2C00;
//             lcddev.setxcmd = 0X2A00;
//             lcddev.setycmd = 0X2B00;
//             lcddev.width = 800;
//             lcddev.height = 480;
//         }
//         LCD_Scan_Dir(D2U_L2R);
//     }
    
//     // LCD_Scan_Dir(DFT_SCAN_DIR);											//	??????
// }

// ///**********************************************************************************************************
// //?????????,???????????????(sx,sy).
// //?????sx,sy:??????(???)width,height:???????,????0!!
// //??????
// //??????
// //**********************************************************************************************************/
// //void LCD_Set_Window(u16 sx, u16 sy, u16 width, u16 height)
// //{
// //    u16 twidth, theight;
// //	
// //    twidth = sx + width - 1;
// //    theight = sy + height - 1;
// //	
// //    LCD_WR_REG(lcddev.setxcmd);
// //    LCD_WR_DATA(sx >> 8);
// //    LCD_WR_DATA(sx & 0XFF);
// //    LCD_WR_DATA(twidth >> 8);
// //    LCD_WR_DATA(twidth & 0XFF);
// //    LCD_WR_REG(lcddev.setycmd);
// //    LCD_WR_DATA(sy >> 8);
// //    LCD_WR_DATA(sy & 0XFF);
// //    LCD_WR_DATA(theight >> 8);
// //    LCD_WR_DATA(theight & 0XFF);
// //}

// /**********************************************************************************************************
// ????????lcd
// ??????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_Init(void)
// {
//     vu32 i = 0;

//     GPIO_InitTypeDef  GPIO_InitStructure;
//     FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
//     FSMC_NORSRAMTimingInitTypeDef  readWriteTiming;
//     FSMC_NORSRAMTimingInitTypeDef  writeTiming;
    
//     //	LCD_BL
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);     
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;						//	????
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//	????
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//	100MHz
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;						//	??
//     GPIO_Init(GPIOA, &GPIO_InitStructure);		
    

// 	//	??PD,PE,PF,PG??
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE 
// 						 | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE); 
//     RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 				//	??FSMC??
    
// 	//	PD0,1,4,5,8,9,10,14,15 AF OUT
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 
//                                 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15; 
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//	????
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//	????
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//	100MHz
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//	??
//     GPIO_Init(GPIOD, &GPIO_InitStructure);								//	???

// 	//	PE7~15,AF OUT
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 
//                                 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;						
                                
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//	????
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//	????
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//	100MHz
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//	??
//     GPIO_Init(GPIOE, &GPIO_InitStructure);								//	???

// 	//	LCD_RS PG1,FSMC_A11
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//	????
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//	????
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//	100MHz
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//	??
//     GPIO_Init(GPIOG, &GPIO_InitStructure);								//	???

// 	//	LCD_CS PG12,FSMC_NE4
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//	????
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//	????
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//	100MHz
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//	??
//     GPIO_Init(GPIOG, &GPIO_InitStructure);								//	???

//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC); 			
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC); 
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC); 
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
//     GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC); 

//     GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC); 
//     GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);


//     readWriteTiming.FSMC_AddressSetupTime = 0x0F;	 					//	???????ADDSET??16?HCLK 1/168M=6ns*16=96ns
//     readWriteTiming.FSMC_AddressHoldTime = 0x00;				 		//	???????ADDHLD???A???
//     readWriteTiming.FSMC_DataSetupTime = 60;							//	???????60?HCLK	=6*60=360ns
//     readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
//     readWriteTiming.FSMC_CLKDivision = 0x00;
//     readWriteTiming.FSMC_DataLatency = 0x00;
//     readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 			//	??A


//     writeTiming.FSMC_AddressSetupTime = 9;	     						//	???????ADDSET??9?HCLK =54ns
//     writeTiming.FSMC_AddressHoldTime = 0x00;	 						//	???????A
//     writeTiming.FSMC_DataSetupTime = 8;		 							//	???????6ns*9?HCLK=54ns
//     writeTiming.FSMC_BusTurnAroundDuration = 0x00;
//     writeTiming.FSMC_CLKDivision = 0x00;
//     writeTiming.FSMC_DataLatency = 0x00;
//     writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 				//	??A


//     FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;			//  ??????NE4 ?????BTCR[6],[7]?
//     FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; 	// 	???????
//     FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM; 				// 	FSMC_MemoryType_SRAM;  //SRAM
//     FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;		//	????????16bit
//     FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable; 	// 	FSMC_BurstAccessMode_Disable;
//     FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
//     FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
//     FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
//     FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
//     FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;		//  ??????
//     FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
//     FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; 		// 	?????????
//     FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
//     FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming; 		//	????
//     FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &writeTiming;  				//	???

//     FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  						//	???FSMC??

//     FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);  						// 	??BANK1

//     LCD_delayms(100); 												    // 	delay 50 ms
    
//     LCD_WR_REG(0XDA00);
//     lcddev.id = LCD_RD_DATA();		//??0X00
//     LCD_WR_REG(0XDB00);
//     lcddev.id = LCD_RD_DATA();		//??0X80
//     lcddev.id <<= 8;
//     LCD_WR_REG(0XDC00);
//     lcddev.id |= LCD_RD_DATA();		//??0X00
//     if(lcddev.id == 0x8000)
//     {
//         lcddev.id = 0x5510; //NT35510???ID?8000H,?????,???????5510
//     }
    
//     if(lcddev.id == 0X5510) //??????IC,???WR?????
//     {
//         //???????????????
//         FSMC_Bank1E->BWTR[6] &= ~(0XF << 0); //??????(ADDSET)??
//         FSMC_Bank1E->BWTR[6] &= ~(0XF << 8); //????????
//         FSMC_Bank1E->BWTR[6] |= 3 << 0;		//??????(ADDSET)?3?HCLK =18ns
//         FSMC_Bank1E->BWTR[6] |= 2 << 8; 	//??????(DATAST)?6ns*3?HCLK=18ns
//     }
    
//     if(lcddev.id == 0x5510)
//     {
//         LCD_WriteReg(0xF000, 0x55);
//         LCD_WriteReg(0xF001, 0xAA);
//         LCD_WriteReg(0xF002, 0x52);
//         LCD_WriteReg(0xF003, 0x08);
//         LCD_WriteReg(0xF004, 0x01);
//         //AVDD Set AVDD 5.2V
//         LCD_WriteReg(0xB000, 0x0D);
//         LCD_WriteReg(0xB001, 0x0D);
//         LCD_WriteReg(0xB002, 0x0D);
//         //AVDD ratio
//         LCD_WriteReg(0xB600, 0x34);
//         LCD_WriteReg(0xB601, 0x34);
//         LCD_WriteReg(0xB602, 0x34);
//         //AVEE -5.2V
//         LCD_WriteReg(0xB100, 0x0D);
//         LCD_WriteReg(0xB101, 0x0D);
//         LCD_WriteReg(0xB102, 0x0D);
//         //AVEE ratio
//         LCD_WriteReg(0xB700, 0x34);
//         LCD_WriteReg(0xB701, 0x34);
//         LCD_WriteReg(0xB702, 0x34);
//         //VCL -2.5V
//         LCD_WriteReg(0xB200, 0x00);
//         LCD_WriteReg(0xB201, 0x00);
//         LCD_WriteReg(0xB202, 0x00);
//         //VCL ratio
//         LCD_WriteReg(0xB800, 0x24);
//         LCD_WriteReg(0xB801, 0x24);
//         LCD_WriteReg(0xB802, 0x24);
//         //VGH 15V (Free pump)
//         LCD_WriteReg(0xBF00, 0x01);
//         LCD_WriteReg(0xB300, 0x0F);
//         LCD_WriteReg(0xB301, 0x0F);
//         LCD_WriteReg(0xB302, 0x0F);
//         //VGH ratio
//         LCD_WriteReg(0xB900, 0x34);
//         LCD_WriteReg(0xB901, 0x34);
//         LCD_WriteReg(0xB902, 0x34);
//         //VGL_REG -10V
//         LCD_WriteReg(0xB500, 0x08);
//         LCD_WriteReg(0xB501, 0x08);
//         LCD_WriteReg(0xB502, 0x08);
//         LCD_WriteReg(0xC200, 0x03);
//         //VGLX ratio
//         LCD_WriteReg(0xBA00, 0x24);
//         LCD_WriteReg(0xBA01, 0x24);
//         LCD_WriteReg(0xBA02, 0x24);
//         //VGMP/VGSP 4.5V/0V
//         LCD_WriteReg(0xBC00, 0x00);
//         LCD_WriteReg(0xBC01, 0x78);
//         LCD_WriteReg(0xBC02, 0x00);
//         //VGMN/VGSN -4.5V/0V
//         LCD_WriteReg(0xBD00, 0x00);
//         LCD_WriteReg(0xBD01, 0x78);
//         LCD_WriteReg(0xBD02, 0x00);
//         //VCOM
//         LCD_WriteReg(0xBE00, 0x00);
//         LCD_WriteReg(0xBE01, 0x64);
//         //Gamma Setting
//         LCD_WriteReg(0xD100, 0x00);
//         LCD_WriteReg(0xD101, 0x33);
//         LCD_WriteReg(0xD102, 0x00);
//         LCD_WriteReg(0xD103, 0x34);
//         LCD_WriteReg(0xD104, 0x00);
//         LCD_WriteReg(0xD105, 0x3A);
//         LCD_WriteReg(0xD106, 0x00);
//         LCD_WriteReg(0xD107, 0x4A);
//         LCD_WriteReg(0xD108, 0x00);
//         LCD_WriteReg(0xD109, 0x5C);
//         LCD_WriteReg(0xD10A, 0x00);
//         LCD_WriteReg(0xD10B, 0x81);
//         LCD_WriteReg(0xD10C, 0x00);
//         LCD_WriteReg(0xD10D, 0xA6);
//         LCD_WriteReg(0xD10E, 0x00);
//         LCD_WriteReg(0xD10F, 0xE5);
//         LCD_WriteReg(0xD110, 0x01);
//         LCD_WriteReg(0xD111, 0x13);
//         LCD_WriteReg(0xD112, 0x01);
//         LCD_WriteReg(0xD113, 0x54);
//         LCD_WriteReg(0xD114, 0x01);
//         LCD_WriteReg(0xD115, 0x82);
//         LCD_WriteReg(0xD116, 0x01);
//         LCD_WriteReg(0xD117, 0xCA);
//         LCD_WriteReg(0xD118, 0x02);
//         LCD_WriteReg(0xD119, 0x00);
//         LCD_WriteReg(0xD11A, 0x02);
//         LCD_WriteReg(0xD11B, 0x01);
//         LCD_WriteReg(0xD11C, 0x02);
//         LCD_WriteReg(0xD11D, 0x34);
//         LCD_WriteReg(0xD11E, 0x02);
//         LCD_WriteReg(0xD11F, 0x67);
//         LCD_WriteReg(0xD120, 0x02);
//         LCD_WriteReg(0xD121, 0x84);
//         LCD_WriteReg(0xD122, 0x02);
//         LCD_WriteReg(0xD123, 0xA4);
//         LCD_WriteReg(0xD124, 0x02);
//         LCD_WriteReg(0xD125, 0xB7);
//         LCD_WriteReg(0xD126, 0x02);
//         LCD_WriteReg(0xD127, 0xCF);
//         LCD_WriteReg(0xD128, 0x02);
//         LCD_WriteReg(0xD129, 0xDE);
//         LCD_WriteReg(0xD12A, 0x02);
//         LCD_WriteReg(0xD12B, 0xF2);
//         LCD_WriteReg(0xD12C, 0x02);
//         LCD_WriteReg(0xD12D, 0xFE);
//         LCD_WriteReg(0xD12E, 0x03);
//         LCD_WriteReg(0xD12F, 0x10);
//         LCD_WriteReg(0xD130, 0x03);
//         LCD_WriteReg(0xD131, 0x33);
//         LCD_WriteReg(0xD132, 0x03);
//         LCD_WriteReg(0xD133, 0x6D);
//         LCD_WriteReg(0xD200, 0x00);
//         LCD_WriteReg(0xD201, 0x33);
//         LCD_WriteReg(0xD202, 0x00);
//         LCD_WriteReg(0xD203, 0x34);
//         LCD_WriteReg(0xD204, 0x00);
//         LCD_WriteReg(0xD205, 0x3A);
//         LCD_WriteReg(0xD206, 0x00);
//         LCD_WriteReg(0xD207, 0x4A);
//         LCD_WriteReg(0xD208, 0x00);
//         LCD_WriteReg(0xD209, 0x5C);
//         LCD_WriteReg(0xD20A, 0x00);

//         LCD_WriteReg(0xD20B, 0x81);
//         LCD_WriteReg(0xD20C, 0x00);
//         LCD_WriteReg(0xD20D, 0xA6);
//         LCD_WriteReg(0xD20E, 0x00);
//         LCD_WriteReg(0xD20F, 0xE5);
//         LCD_WriteReg(0xD210, 0x01);
//         LCD_WriteReg(0xD211, 0x13);
//         LCD_WriteReg(0xD212, 0x01);
//         LCD_WriteReg(0xD213, 0x54);
//         LCD_WriteReg(0xD214, 0x01);
//         LCD_WriteReg(0xD215, 0x82);
//         LCD_WriteReg(0xD216, 0x01);
//         LCD_WriteReg(0xD217, 0xCA);
//         LCD_WriteReg(0xD218, 0x02);
//         LCD_WriteReg(0xD219, 0x00);
//         LCD_WriteReg(0xD21A, 0x02);
//         LCD_WriteReg(0xD21B, 0x01);
//         LCD_WriteReg(0xD21C, 0x02);
//         LCD_WriteReg(0xD21D, 0x34);
//         LCD_WriteReg(0xD21E, 0x02);
//         LCD_WriteReg(0xD21F, 0x67);
//         LCD_WriteReg(0xD220, 0x02);
//         LCD_WriteReg(0xD221, 0x84);
//         LCD_WriteReg(0xD222, 0x02);
//         LCD_WriteReg(0xD223, 0xA4);
//         LCD_WriteReg(0xD224, 0x02);
//         LCD_WriteReg(0xD225, 0xB7);
//         LCD_WriteReg(0xD226, 0x02);
//         LCD_WriteReg(0xD227, 0xCF);
//         LCD_WriteReg(0xD228, 0x02);
//         LCD_WriteReg(0xD229, 0xDE);
//         LCD_WriteReg(0xD22A, 0x02);
//         LCD_WriteReg(0xD22B, 0xF2);
//         LCD_WriteReg(0xD22C, 0x02);
//         LCD_WriteReg(0xD22D, 0xFE);
//         LCD_WriteReg(0xD22E, 0x03);
//         LCD_WriteReg(0xD22F, 0x10);
//         LCD_WriteReg(0xD230, 0x03);
//         LCD_WriteReg(0xD231, 0x33);
//         LCD_WriteReg(0xD232, 0x03);
//         LCD_WriteReg(0xD233, 0x6D);
//         LCD_WriteReg(0xD300, 0x00);
//         LCD_WriteReg(0xD301, 0x33);
//         LCD_WriteReg(0xD302, 0x00);
//         LCD_WriteReg(0xD303, 0x34);
//         LCD_WriteReg(0xD304, 0x00);
//         LCD_WriteReg(0xD305, 0x3A);
//         LCD_WriteReg(0xD306, 0x00);
//         LCD_WriteReg(0xD307, 0x4A);
//         LCD_WriteReg(0xD308, 0x00);
//         LCD_WriteReg(0xD309, 0x5C);
//         LCD_WriteReg(0xD30A, 0x00);

//         LCD_WriteReg(0xD30B, 0x81);
//         LCD_WriteReg(0xD30C, 0x00);
//         LCD_WriteReg(0xD30D, 0xA6);
//         LCD_WriteReg(0xD30E, 0x00);
//         LCD_WriteReg(0xD30F, 0xE5);
//         LCD_WriteReg(0xD310, 0x01);
//         LCD_WriteReg(0xD311, 0x13);
//         LCD_WriteReg(0xD312, 0x01);
//         LCD_WriteReg(0xD313, 0x54);
//         LCD_WriteReg(0xD314, 0x01);
//         LCD_WriteReg(0xD315, 0x82);
//         LCD_WriteReg(0xD316, 0x01);
//         LCD_WriteReg(0xD317, 0xCA);
//         LCD_WriteReg(0xD318, 0x02);
//         LCD_WriteReg(0xD319, 0x00);
//         LCD_WriteReg(0xD31A, 0x02);
//         LCD_WriteReg(0xD31B, 0x01);
//         LCD_WriteReg(0xD31C, 0x02);
//         LCD_WriteReg(0xD31D, 0x34);
//         LCD_WriteReg(0xD31E, 0x02);
//         LCD_WriteReg(0xD31F, 0x67);
//         LCD_WriteReg(0xD320, 0x02);
//         LCD_WriteReg(0xD321, 0x84);
//         LCD_WriteReg(0xD322, 0x02);
//         LCD_WriteReg(0xD323, 0xA4);
//         LCD_WriteReg(0xD324, 0x02);
//         LCD_WriteReg(0xD325, 0xB7);
//         LCD_WriteReg(0xD326, 0x02);
//         LCD_WriteReg(0xD327, 0xCF);
//         LCD_WriteReg(0xD328, 0x02);
//         LCD_WriteReg(0xD329, 0xDE);
//         LCD_WriteReg(0xD32A, 0x02);
//         LCD_WriteReg(0xD32B, 0xF2);
//         LCD_WriteReg(0xD32C, 0x02);
//         LCD_WriteReg(0xD32D, 0xFE);
//         LCD_WriteReg(0xD32E, 0x03);
//         LCD_WriteReg(0xD32F, 0x10);
//         LCD_WriteReg(0xD330, 0x03);
//         LCD_WriteReg(0xD331, 0x33);
//         LCD_WriteReg(0xD332, 0x03);
//         LCD_WriteReg(0xD333, 0x6D);
//         LCD_WriteReg(0xD400, 0x00);
//         LCD_WriteReg(0xD401, 0x33);
//         LCD_WriteReg(0xD402, 0x00);
//         LCD_WriteReg(0xD403, 0x34);
//         LCD_WriteReg(0xD404, 0x00);
//         LCD_WriteReg(0xD405, 0x3A);
//         LCD_WriteReg(0xD406, 0x00);
//         LCD_WriteReg(0xD407, 0x4A);
//         LCD_WriteReg(0xD408, 0x00);
//         LCD_WriteReg(0xD409, 0x5C);
//         LCD_WriteReg(0xD40A, 0x00);
//         LCD_WriteReg(0xD40B, 0x81);

//         LCD_WriteReg(0xD40C, 0x00);
//         LCD_WriteReg(0xD40D, 0xA6);
//         LCD_WriteReg(0xD40E, 0x00);
//         LCD_WriteReg(0xD40F, 0xE5);
//         LCD_WriteReg(0xD410, 0x01);
//         LCD_WriteReg(0xD411, 0x13);
//         LCD_WriteReg(0xD412, 0x01);
//         LCD_WriteReg(0xD413, 0x54);
//         LCD_WriteReg(0xD414, 0x01);
//         LCD_WriteReg(0xD415, 0x82);
//         LCD_WriteReg(0xD416, 0x01);
//         LCD_WriteReg(0xD417, 0xCA);
//         LCD_WriteReg(0xD418, 0x02);
//         LCD_WriteReg(0xD419, 0x00);
//         LCD_WriteReg(0xD41A, 0x02);
//         LCD_WriteReg(0xD41B, 0x01);
//         LCD_WriteReg(0xD41C, 0x02);
//         LCD_WriteReg(0xD41D, 0x34);
//         LCD_WriteReg(0xD41E, 0x02);
//         LCD_WriteReg(0xD41F, 0x67);
//         LCD_WriteReg(0xD420, 0x02);
//         LCD_WriteReg(0xD421, 0x84);
//         LCD_WriteReg(0xD422, 0x02);
//         LCD_WriteReg(0xD423, 0xA4);
//         LCD_WriteReg(0xD424, 0x02);
//         LCD_WriteReg(0xD425, 0xB7);
//         LCD_WriteReg(0xD426, 0x02);
//         LCD_WriteReg(0xD427, 0xCF);
//         LCD_WriteReg(0xD428, 0x02);
//         LCD_WriteReg(0xD429, 0xDE);
//         LCD_WriteReg(0xD42A, 0x02);
//         LCD_WriteReg(0xD42B, 0xF2);
//         LCD_WriteReg(0xD42C, 0x02);
//         LCD_WriteReg(0xD42D, 0xFE);
//         LCD_WriteReg(0xD42E, 0x03);
//         LCD_WriteReg(0xD42F, 0x10);
//         LCD_WriteReg(0xD430, 0x03);
//         LCD_WriteReg(0xD431, 0x33);
//         LCD_WriteReg(0xD432, 0x03);
//         LCD_WriteReg(0xD433, 0x6D);
//         LCD_WriteReg(0xD500, 0x00);
//         LCD_WriteReg(0xD501, 0x33);
//         LCD_WriteReg(0xD502, 0x00);
//         LCD_WriteReg(0xD503, 0x34);
//         LCD_WriteReg(0xD504, 0x00);
//         LCD_WriteReg(0xD505, 0x3A);
//         LCD_WriteReg(0xD506, 0x00);
//         LCD_WriteReg(0xD507, 0x4A);
//         LCD_WriteReg(0xD508, 0x00);
//         LCD_WriteReg(0xD509, 0x5C);
//         LCD_WriteReg(0xD50A, 0x00);
//         LCD_WriteReg(0xD50B, 0x81);

//         LCD_WriteReg(0xD50C, 0x00);
//         LCD_WriteReg(0xD50D, 0xA6);
//         LCD_WriteReg(0xD50E, 0x00);
//         LCD_WriteReg(0xD50F, 0xE5);
//         LCD_WriteReg(0xD510, 0x01);
//         LCD_WriteReg(0xD511, 0x13);
//         LCD_WriteReg(0xD512, 0x01);
//         LCD_WriteReg(0xD513, 0x54);
//         LCD_WriteReg(0xD514, 0x01);
//         LCD_WriteReg(0xD515, 0x82);
//         LCD_WriteReg(0xD516, 0x01);
//         LCD_WriteReg(0xD517, 0xCA);
//         LCD_WriteReg(0xD518, 0x02);
//         LCD_WriteReg(0xD519, 0x00);
//         LCD_WriteReg(0xD51A, 0x02);
//         LCD_WriteReg(0xD51B, 0x01);
//         LCD_WriteReg(0xD51C, 0x02);
//         LCD_WriteReg(0xD51D, 0x34);
//         LCD_WriteReg(0xD51E, 0x02);
//         LCD_WriteReg(0xD51F, 0x67);
//         LCD_WriteReg(0xD520, 0x02);
//         LCD_WriteReg(0xD521, 0x84);
//         LCD_WriteReg(0xD522, 0x02);
//         LCD_WriteReg(0xD523, 0xA4);
//         LCD_WriteReg(0xD524, 0x02);
//         LCD_WriteReg(0xD525, 0xB7);
//         LCD_WriteReg(0xD526, 0x02);
//         LCD_WriteReg(0xD527, 0xCF);
//         LCD_WriteReg(0xD528, 0x02);
//         LCD_WriteReg(0xD529, 0xDE);
//         LCD_WriteReg(0xD52A, 0x02);
//         LCD_WriteReg(0xD52B, 0xF2);
//         LCD_WriteReg(0xD52C, 0x02);
//         LCD_WriteReg(0xD52D, 0xFE);
//         LCD_WriteReg(0xD52E, 0x03);
//         LCD_WriteReg(0xD52F, 0x10);
//         LCD_WriteReg(0xD530, 0x03);
//         LCD_WriteReg(0xD531, 0x33);
//         LCD_WriteReg(0xD532, 0x03);
//         LCD_WriteReg(0xD533, 0x6D);
//         LCD_WriteReg(0xD600, 0x00);
//         LCD_WriteReg(0xD601, 0x33);
//         LCD_WriteReg(0xD602, 0x00);
//         LCD_WriteReg(0xD603, 0x34);
//         LCD_WriteReg(0xD604, 0x00);
//         LCD_WriteReg(0xD605, 0x3A);
//         LCD_WriteReg(0xD606, 0x00);
//         LCD_WriteReg(0xD607, 0x4A);
//         LCD_WriteReg(0xD608, 0x00);
//         LCD_WriteReg(0xD609, 0x5C);
//         LCD_WriteReg(0xD60A, 0x00);
//         LCD_WriteReg(0xD60B, 0x81);

//         LCD_WriteReg(0xD60C, 0x00);
//         LCD_WriteReg(0xD60D, 0xA6);
//         LCD_WriteReg(0xD60E, 0x00);
//         LCD_WriteReg(0xD60F, 0xE5);
//         LCD_WriteReg(0xD610, 0x01);
//         LCD_WriteReg(0xD611, 0x13);
//         LCD_WriteReg(0xD612, 0x01);
//         LCD_WriteReg(0xD613, 0x54);
//         LCD_WriteReg(0xD614, 0x01);
//         LCD_WriteReg(0xD615, 0x82);
//         LCD_WriteReg(0xD616, 0x01);
//         LCD_WriteReg(0xD617, 0xCA);
//         LCD_WriteReg(0xD618, 0x02);
//         LCD_WriteReg(0xD619, 0x00);
//         LCD_WriteReg(0xD61A, 0x02);
//         LCD_WriteReg(0xD61B, 0x01);
//         LCD_WriteReg(0xD61C, 0x02);
//         LCD_WriteReg(0xD61D, 0x34);
//         LCD_WriteReg(0xD61E, 0x02);
//         LCD_WriteReg(0xD61F, 0x67);
//         LCD_WriteReg(0xD620, 0x02);
//         LCD_WriteReg(0xD621, 0x84);
//         LCD_WriteReg(0xD622, 0x02);
//         LCD_WriteReg(0xD623, 0xA4);
//         LCD_WriteReg(0xD624, 0x02);
//         LCD_WriteReg(0xD625, 0xB7);
//         LCD_WriteReg(0xD626, 0x02);
//         LCD_WriteReg(0xD627, 0xCF);
//         LCD_WriteReg(0xD628, 0x02);
//         LCD_WriteReg(0xD629, 0xDE);
//         LCD_WriteReg(0xD62A, 0x02);
//         LCD_WriteReg(0xD62B, 0xF2);
//         LCD_WriteReg(0xD62C, 0x02);
//         LCD_WriteReg(0xD62D, 0xFE);
//         LCD_WriteReg(0xD62E, 0x03);
//         LCD_WriteReg(0xD62F, 0x10);
//         LCD_WriteReg(0xD630, 0x03);
//         LCD_WriteReg(0xD631, 0x33);
//         LCD_WriteReg(0xD632, 0x03);
//         LCD_WriteReg(0xD633, 0x6D);
//         //LV2 Page 0 enable
//         LCD_WriteReg(0xF000, 0x55);
//         LCD_WriteReg(0xF001, 0xAA);
//         LCD_WriteReg(0xF002, 0x52);
//         LCD_WriteReg(0xF003, 0x08);
//         LCD_WriteReg(0xF004, 0x00);
//         //Display control
//         LCD_WriteReg(0xB100, 0xCC);
//         LCD_WriteReg(0xB101, 0x00);
//         //Source hold time
//         LCD_WriteReg(0xB600, 0x05);
//         //Gate EQ control
//         LCD_WriteReg(0xB700, 0x70);
//         LCD_WriteReg(0xB701, 0x70);
//         //Source EQ control (Mode 2)
//         LCD_WriteReg(0xB800, 0x01);
//         LCD_WriteReg(0xB801, 0x03);
//         LCD_WriteReg(0xB802, 0x03);
//         LCD_WriteReg(0xB803, 0x03);
//         //Inversion mode (2-dot)
//         LCD_WriteReg(0xBC00, 0x02);
//         LCD_WriteReg(0xBC01, 0x00);
//         LCD_WriteReg(0xBC02, 0x00);
//         //Timing control 4H w/ 4-delay
//         LCD_WriteReg(0xC900, 0xD0);
//         LCD_WriteReg(0xC901, 0x02);
//         LCD_WriteReg(0xC902, 0x50);
//         LCD_WriteReg(0xC903, 0x50);
//         LCD_WriteReg(0xC904, 0x50);
//         LCD_WriteReg(0x3500, 0x00);
//         LCD_WriteReg(0x3A00, 0x55); //16-bit/pixel
//         LCD_WR_REG(0x1100);
//         LCD_delayus(120);
//         LCD_WR_REG(0x2900);
//     }
    
//     LCD_Display_Dir(1);		//?????
    
//     LCD_BL_ON;              // ????

//     LCD_Clear(WHITE);

//     // LCD_WR_REG(0x29);	                                                //  ????
    
//     // LCD_Display_Dir(1);												    //	?????
//     // LCD_BL_ON;
//     // LCD_Clear(WHITE);
// }


// /**********************************************************************************************************
// ?????????
// ?????color:???????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_Clear(u16 color)
// {
//     u32 index = 0;
	
//     u32 totalpoint = lcddev.width;
//     totalpoint *= lcddev.height; 										//	?????
	
//     LCD_SetCursor(0x00, 0x0000);									    //	??????
    
//     LCD_WriteRAM_Prepare();     									    //	????GRAM
    
//     for(index = 0; index < totalpoint; index++)
//     {
//         LCD->LCD_RAM = color;
//     }
// }


// ///**********************************************************************************************************
// //?????????????????
// //?????(sx,sy),(ex,ey):????????,?????:(ex-sx+1)*(ey-sy+1)
// //		  color:??????
// //??????
// //??????
// //**********************************************************************************************************/
// //void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color)
// //{
// //    u16 i, j;
// //    u16 xlen = 0;
// //	
// //    xlen = ex - sx + 1;
// //    
// //    for(i = sy; i <= ey; i++)
// //    {
// //        LCD_SetCursor(sx, i);      									    //	??????
// //        
// //        LCD_WriteRAM_Prepare();     								    //	????GRAM
// //        
// //        for(j = 0; j < xlen; j++)
// //        {
// //            LCD->LCD_RAM = color;									    //	????
// //        }
// //    }
// //}


// ///**********************************************************************************************************
// //??????????????????
// //?????(sx,sy),(ex,ey):????????,?????:(ex-sx+1)*(ey-sy+1)
// //		  color:??????
// //??????
// //??????
// //**********************************************************************************************************/
// void LCD_Color_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 *color)
// {
//    u16 height, width;
//    u16 i, j;
	
//    width = ex - sx + 1; 												//	???????
//    height = ey - sy + 1;												//	??
	
//    for(i = 0; i < height; i++)
//    {
//        LCD_SetCursor(sx, sy + i);   									//	??????
//        LCD_WriteRAM_Prepare();     									//	????GRAM
		
//        for(j = 0; j < width; j++)
//        {
//            LCD->LCD_RAM = color[i * width + j]; 						//	????
//        }
//    }
// }


// /**********************************************************************************************************
// ???????
// ?????x1,y1:????
// 		  x2,y2:????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
// {
//     u16 t;                                                              
//     int xerr = 0, yerr = 0, delta_x, delta_y, distance;                 //   delta_x?delta_y?x?y?????????distance?x?y????????xerr?yerr?????????
//     int incx, incy, uRow, uCol;                                         //  incx?incy???????????????1????0???—1??
    
//     delta_x = x2 - x1; 													//	??????
//     delta_y = y2 - y1;
//     uRow = x1;                                                          //  ??????x??
//     uCol = y1;                                                          //  ??????y??
    
//     if(delta_x > 0)
//     {
//         incx = 1; 														//	??????
//     }
//     else if(delta_x == 0)
//     {
//         incx = 0; 														//	???
//     }
//     else
//     {
//         incx = -1;
//         delta_x = -delta_x;                                             
//     }
    
//     if(delta_y > 0)
//     {
//         incy = 1;
//     }
//     else if(delta_y == 0)
//     {
//         incy = 0; 														//	???
//     }
//     else
//     {
//         incy = -1;
//         delta_y = -delta_y;
//     }
    
//     if( delta_x > delta_y)
//     {
//         distance = delta_x; 											//	?????????
//     }
//     else 
//     {
//         distance = delta_y;
//     }
    
//     for(t = 0; t <= distance + 1; t++ ) 								//	????
//     {
//         LCD_DrawPoint(uRow, uCol, color); 								//	??
//         xerr += delta_x ;
//         yerr += delta_y ;
        
//         if(xerr > distance)
//         {
//             xerr -= distance;
//             uRow += incx;
//         }
//         if(yerr > distance)
//         {
//             yerr -= distance;
//             uCol += incy;
//         }
//     }
// }


// /**********************************************************************************************************
// ????????
// ?????(x1,y1),(x2,y2):???????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
// {
//     LCD_DrawLine(x1, y1, x2, y1, color);
//     LCD_DrawLine(x1, y1, x1, y2, color);
//     LCD_DrawLine(x1, y2, x2, y2, color);
//     LCD_DrawLine(x2, y1, x2, y2, color);
// }


// ///**********************************************************************************************************
// //???????????????????
// //?????(x,y):???
// //		  r    :??
// //??????
// //??????
// //**********************************************************************************************************/
// void LCD_Draw_Circle(u16 x0, u16 y0, u8 r, u16 color)
// {
// 	//????
// }


// /**********************************************************************************************************
// ???????????
// ?????x0?y0    ????
// 		  pcStr     ??
// 		  PenColor  ????
// 		  BackColor ????
// ??????
// ??????
// ???????????????????????	
// **********************************************************************************************************/
// void LCD_ShowHzString(u16 x, u16 y, u8 *c, u16 PenColor, u16 BackColor)
// {
// 	int i, j;
// 	u8 buffer[32];
// 	u16 tmp_char=0;
// 	unsigned int c1, c2;
//     unsigned int c3 = 0;
//     unsigned char *c4;
    
//     c4 = c4;
//     c1 = *c;	                                                        //  ??
//     c2 = *(c + 1);	                                                    //  ??	????
//     c3 = ((c1 - 0xa1)*0x5e + (c2 - 0xa1))*0x20;                         //  ?????????
//     c4 = (unsigned char*)&Hzk16[c3];                                    //  ?????
		  
// 	for(i=0;i<32;i++)
//     {
//         buffer[i] = Hzk16[c3+i];
//     }

// 	for (i = 0; i < 16; i++)		 
// 	{
// 		tmp_char = buffer[i*2];
// 		tmp_char = (tmp_char << 8);
// 		tmp_char |= buffer[2*i + 1];                                    //  ??tmp_char??????????

// 		for (j = 0; j < 16; j++)
// 		{
// 			if((tmp_char >> 15 - j) & 0x01 == 0x01)
// 			{
// 				LCD_DrawPoint(x + j, y + i, PenColor);                  //  ????
// 			}
// 			else
// 			{
// 				LCD_DrawPoint(x + j, y + i, BackColor);                 //  ????
// 			}
// 		}
// 	}

// }


// /**********************************************************************************************************
// ???????ASCII???
// ?????x?y      ?????x:0~234 y:0~308?
// 		  num       ??ASCII??
// 		  size      ?????????8*16
// 		  PenColor  ????
// 		  BackColor ??????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u16 PenColor, u16 BackColor)
// {       
//     u8 temp, t1, t;
// 	u16 y0 = y;
    
// 	u8 csize = (size/8 + ((size%8)? 1 : 0))*(size/2);		            //  ???????????????????	
    
//     //  ????????ASCII?????????????-' '??????????
//  	num = num - ' ';                                                    
	
//     for(t = 0; t < csize; t++)
// 	{   
// 		if (size == 12)
//         {
//             temp = asc2_1206[num][t]; 	 	                            //  ??1206??
//         }
        
// 		else if(size == 16) 
//         {
//             temp = asc2_1608[num][t];	                                //  ??1608??
//         }
        
// 		else if(size == 24)
//         {
//             temp = asc2_2412[num][t];	                                //  ??2412??
//         }
// 		else 
//         {
//             return;								                        //  ?????
//         }
        
// 		for(t1 = 0; t1 < 8; t1++)
// 		{			    
// 			if(temp&0x80)
//             {
//                 LCD_Fast_DrawPoint(x, y, PenColor);
//             }
            
// 			else
//             {
//                 LCD_Fast_DrawPoint(x, y, BackColor);
//             }
            
// 			temp <<= 1;
            
// 			y++;
			
//             if (y >= lcddev.height)
//             {
//                 return;		//????
//             }
            
// 			if((y - y0) == size)
// 			{
// 				y = y0;
				
//                 x++;
				
//                 if(x >= lcddev.width)
//                 {
//                     return;	//????
//                 }
// 				break;
// 			}
// 		}  	 
// 	}  	    	
// }


// /**********************************************************************************************************
// ???????1??????
// ?????x?y      ????
// 	      p         ?????????
// 		  PenColor  ????
// 		  BackColor ????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_ShowCharString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t PenColor, uint16_t BackColor)
// {   
// 	uint8_t size = 16;                                                  //  ??????16*8
	 
//     if(x > lcddev.width)                                                //  ??X??????????
//     {
//         x = 0;
//         y += size;
//     }			         
    
//     if(y > lcddev.height)                                               //  ??Y?????????????????
//     {
//         y = x = 0;
//         LCD_Clear(WHITE);
//     }	 
    
//     LCD_ShowChar(x, y, *p, size, PenColor, BackColor);			   		//	0???????
// }


// /**********************************************************************************************************
// ??????????
// ?????x,y			????
// 		  u8 *pcStr		??
// 		  PenColor  	????
// 		  BackColor 	????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_ShowString(u16 x0, u16 y0, u8 *pcStr, u16 PenColor, u16 BackColor)
// {
//     while(*pcStr)
// 	{
// 		if(*pcStr > 0xa1)                                               //  ????
// 		{
// 			LCD_ShowHzString(x0, y0, pcStr, PenColor, BackColor);
// 			pcStr += 2;
// 			x0 += 16;
// 		}
// 		else                                                            //  ????
// 		{
// 			LCD_ShowCharString(x0, y0, pcStr, PenColor, BackColor);	
// 			pcStr +=1;
// 			x0+= 8;
// 		}
// 	}	
// }

// /**********************************************************************************************************
// ?????LCD_DrawPoint
// ?????x.y:?????
// 		  color???
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_DrawOnrPoint(uint16_t xsta, uint16_t ysta, uint16_t color)
// {
// 	LCD_SetCursor(xsta, ysta);  		                                //  ??????
// 	LCD_WriteRAM_Prepare();          	                                //  ????GRAM
// 	LCD_WriteRAM(color); 

// }


// /**********************************************************************************************************
// ?????????
// ?????StartX     ?????
//           StartY     ?????
//           EndX       ?????
//           EndY       ?????
//           pic        ?????
// ??????
// ??????
// **********************************************************************************************************/
// void LCD_DrawPicture(u16 StartX, u16 StartY, u16 Xend, u16 Yend, u8 *pic)
// {
//     static	u16 i = 0, j = 0;
    
//     u16 *bitmap = (u16 *)pic;
    
//     for(j = 0; j < Yend - StartY; j++)
//     {
//         for(i = 0; i < Xend - StartX; i++)
//         {
//             LCD_DrawOnrPoint(StartX + i, StartY + j, *bitmap++);
//         }
//     }
// }
