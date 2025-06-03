/*
 * @Author: 张皓博 3040692310@qq.com
 * @Date: 2024-12-31 21:02:26
 * @LastEditors: 张皓博 3040692310@qq.com
 * @LastEditTime: 2025-05-27 15:08:45
 * @FilePath: \lvgl_mlx90640-main\BaseDrive\mlx90640\mlx90640.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYMC_MLX90640_H__
#define __SYMC_MLX90640_H__

#include "stm32f4xx.h"
#include "../../lvgl/lvgl.h"

#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned int

#define SRC_WIDTH 32
#define SRC_HEIGHT 24

#define TARGET_WIDTH 320
#define TARGET_HEIGHT 240


extern uint8_t mlx90640_buf[1544];
extern uint8_t state;
extern uint8_t is_update;

uint8_t Check(uint8_t *data);


// 双缓冲区

extern uint8_t disp_finish_flag;
extern uint8_t updating_buffer_flag;
extern uint8_t buffer_a[SRC_WIDTH * SRC_HEIGHT * 2];
extern uint8_t buffer_b[SRC_WIDTH * SRC_HEIGHT * 2];


void swap_buffers(lv_obj_t* img);
void update_buffer(uint8_t* src);
void create_dynamic_image(lv_obj_t* parent);
#endif