/**
 * @file lvgl_stubs.c
 * @brief 提供LVGL缺失函数和符号的实现以解决链接错误
 */

#include "lvgl.h"
#include <stdarg.h>
#include <stddef.h>

// DMA2D函数的空实现（因为我们禁用了DMA2D）
void lv_draw_stm32_dma2d_init(void)
{
    // 空实现 - DMA2D已禁用
}

void lv_draw_stm32_dma2d_ctx_init(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx)
{
    (void)drv;
    (void)draw_ctx;
    // 空实现 - DMA2D已禁用
}

void lv_draw_stm32_dma2d_ctx_deinit(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx)
{
    (void)drv;
    (void)draw_ctx;
    // 空实现 - DMA2D已禁用
}

// 这些函数现在由真正的LVGL实现文件提供：
// lv_draw_sw_gradient.c、lv_draw_sw_img.c、lv_draw_sw_polygon.c、lv_draw_sw_transform.c

// 简化的printf功能存根（LVGL需要printf支持）
int lv_vsnprintf(char* buf, size_t size, const char* fmt, va_list args)
{
    // 简单实现，返回0 - 禁用debug输出以节省内存
    (void)buf; (void)size; (void)fmt; (void)args;
    return 0;
}

// 注意：已删除之前的空实现函数，因为现在项目中包含了真正的实现文件：
// - lv_draw_sw_gradient.c    提供渐变绘制功能
// - lv_draw_sw_img.c         提供图像绘制功能  
// - lv_draw_sw_polygon.c     提供多边形绘制功能
// - lv_draw_sw_transform.c   提供变换绘制功能
// 这些文件现在会提供真正的LVGL绘制功能实现
 