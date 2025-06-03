// #include "lvgl.h"
// #include "linear_interpolation.h"
// #include "stm32f4xx.h"
// #include "led.h"
// uint8_t* input_image;
// uint8_t* output_image;
// lv_obj_t* mlx90640_canvas[TARGET_HEIGHT/LINE_BUFFER_SIZE];
// static lv_color_t line_buffer[TARGET_WIDTH * LINE_BUFFER_SIZE];
// void image_init() {
    // for(uint16_t i = 0; i < TARGET_HEIGHT/LINE_BUFFER_SIZE; i++){
    //     mlx90640_canvas[i] = lv_canvas_create(lv_scr_act());
        // lv_obj_set_size(mlx90640_canvas[i], TARGET_WIDTH, LINE_BUFFER_SIZE);
        // lv_canvas_set_buffer(mlx90640_canvas[i], line_buffer, TARGET_WIDTH, LINE_BUFFER_SIZE, LV_IMG_CF_TRUE_COLOR);
        // lv_obj_align(mlx90640_canvas[i], LV_ALIGN_TOP_LEFT, 0, i*LINE_BUFFER_SIZE);
        // lv_canvas_fill_bg(mlx90640_canvas[i], lv_color_black(), LV_OPA_COVER);
    // }
    // mlx90640_canvas = lv_canvas_create(lv_scr_act());
    // output_image = (uint8_t*)malloc(TARGET_WIDTH * TARGET_HEIGHT * sizeof(uint8_t) * 2);
    // input_image = (uint8_t*)malloc(SRC_WIDTH * SRC_HEIGHT * sizeof(uint8_t) * 2);
    // if(input_image == NULL) {
    //     LED2_ON;
    //     // while(1);
    // }
    // if (output_image == NULL) {
    //     LED3_ON;
    //     // while(1);
    // }
    
    // for (int i = 0; i < SRC_WIDTH * SRC_HEIGHT * 2; i++) {
    //     uint16_t color = (lv_rand(0, 31) << 11) | (lv_rand(0, 63) << 5) | lv_rand(0, 31); // RGB565格式
    //     input_image[i * 2] = color & 0xFF;         // 低8位
    //     input_image[i * 2 + 1] = (color >> 8) & 0xFF; // 高8位
    //     // input_image[i] = (uint8_t)(lv_rand(0, 255));
    // }
    // for (int i = 0; i < TARGET_WIDTH * TARGET_HEIGHT * 2; i++) {
    //     uint16_t color = (lv_rand(0, 31) << 11) | (lv_rand(0, 63) << 5) | lv_rand(0, 31); // RGB565格式
    //     output_image[i * 2] = color & 0xFF;         // 低8位
    //     output_image[i * 2 + 1] = (color >> 8) & 0xFF; // 高8位
    // }
// }
// void image_free() {
//     free(input_image);
//     // free(output_image);
// }

// // 双线性插值函数
// void interpolate_line(uint8_t* src, int y, lv_color_t* dst){
//     // for(int i = 0; i < TARGET_WIDTH; i++){
//     //     dst[i] = lv_color_make(lv_rand(0,255), lv_rand(0,255), lv_rand(0,255));
//     // }
//     // // 原图和目标图的比例
//     // float x_ratio = (float)(SRC_WIDTH - 1) / (TARGET_WIDTH - 1);
//     // float y_ratio = (float)(SRC_HEIGHT - 1) / (TARGET_HEIGHT - 1);

//     // // 在原图中的坐标
//     // float src_y = y * y_ratio;

//     // // 整数部分
//     // int y0 = (int)src_y;

//     // // 浮点部分
//     // float dy = src_y - y0;

//     // // 边界处理
//     // int y1 = (y0 + 1 < SRC_HEIGHT) ? y0 + 1 : y0;

//     // for(int x = 0; x < TARGET_WIDTH; x++){
//     //     // 在原图中的坐标
//     //     float src_x = x * x_ratio;

//     //     // 整数部分
//     //     int x0 = (int)src_x;

//     //     // 浮点部分
//     //     float dx = src_x - x0;

//     //     // 边界处理
//     //     int x1 = (x0 + 1 < SRC_WIDTH) ? x0 + 1 : x0;

//     //     // 获取原图四个相邻像素的RGB565值
//     //     uint16_t Q11 = ((uint16_t*)src)[y0 * SRC_WIDTH + x0];
//     //     uint16_t Q21 = ((uint16_t*)src)[y0 * SRC_WIDTH + x1];
//     //     uint16_t Q12 = ((uint16_t*)src)[y1 * SRC_WIDTH + x0];
//     //     uint16_t Q22 = ((uint16_t*)src)[y1 * SRC_WIDTH + x1];

//     //     // 分离RGB分量
//     //     uint8_t R11 = (Q11 >> 11) & 0x1F; // 红色分量
//     //     uint8_t G11 = (Q11 >> 5) & 0x3F;  // 绿色分量
//     //     uint8_t B11 = Q11 & 0x1F;         // 蓝色分量

//     //     uint8_t R21 = (Q21 >> 11) & 0x1F;
//     //     uint8_t G21 = (Q21 >> 5) & 0x3F;
//     //     uint8_t B21 = Q21 & 0x1F;

//     //     uint8_t R12 = (Q12 >> 11) & 0x1F;
//     //     uint8_t G12 = (Q12 >> 5) & 0x3F;
//     //     uint8_t B12 = Q12 & 0x1F;

//     //     uint8_t R22 = (Q22 >> 11) & 0x1F;
//     //     uint8_t G22 = (Q22 >> 5) & 0x3F;
//     //     uint8_t B22 = Q22 & 0x1F;

//     //     // 插值计算
//     //     uint8_t R = (uint8_t)(
//     //         R11 * (1 - dx) * (1 - dy) +
//     //         R21 * dx * (1 - dy) +
//     //         R12 * (1 - dx) * dy +
//     //         R22 * dx * dy);

//     //     uint8_t G = (uint8_t)(
//     //         G11 * (1 - dx) * (1 - dy) +
//     //         G21 * dx * (1 - dy) +
//     //         G12 * (1 - dx) * dy +
//     //         G22 * dx * dy);

//     //     uint8_t B = (uint8_t)(
//     //         B11 * (1 - dx) * (1 - dy) +
//     //         B21 * dx * (1 - dy) +
//     //         B12 * (1 - dx) * dy +
//     //         B22 * dx * dy);

//     //     // 将RGB值合并为RGB565格式
//     //     uint16_t interpolated_pixel = ((R & 0x1F) << 11) | ((G & 0x3F) << 5) | (B & 0x1F);

//     //     // 写入目标图像
//     //     dst[x] = lv_color_make(R, G, B);

//     // }
// }
// void get_line(uint8_t* src, int y, lv_color_t* dst){
//     uint8_t R, G, B;
//     for(int x = 0; x < SRC_WIDTH; x++){
//         for(int i = 0; i < TARGET_WIDTH / SRC_WIDTH; i++){
//             R = (((uint16_t*)src)[y * SRC_WIDTH + x] >> 11) & 0x1F;
//             G = (((uint16_t*)src)[y * SRC_WIDTH + x] >> 5) & 0x3F;
//             B = ((uint16_t*)src)[y * SRC_WIDTH + x] & 0x1F;
//             // dst[x * TARGET_WIDTH / SRC_WIDTH + i] = src[y * SRC_WIDTH + x]; // 重复复制，实现放大
//             dst[x * TARGET_WIDTH / SRC_WIDTH + i] = lv_color_make(R, G, B);
//         }
//     }
// }
// void get_line_buffer(uint8_t* src, lv_color_t* dst, int id){
//     // for(int i = 0; i < LINE_BUFFER_SIZE; i++){
//     //     interpolate_line(src, i, dst + i * TARGET_WIDTH);
//     // }
//     for(int i = 0; i < LINE_BUFFER_SIZE; i++){
//         get_line(src, i + id * LINE_BUFFER_SIZE, dst + i * TARGET_WIDTH);
//     }
// }

// void mlx90640_draw(){
//     for(int i = 0; i < TARGET_HEIGHT / LINE_BUFFER_SIZE; i++){
//         // lv_obj_t* mlx90640_canvas = lv_canvas_create(lv_scr_act());
//         // lv_obj_set_size(mlx90640_canvas, TARGET_WIDTH, LINE_BUFFER_SIZE);
//         // lv_canvas_set_buffer(mlx90640_canvas, line_buffer, TARGET_WIDTH, LINE_BUFFER_SIZE, LV_IMG_CF_TRUE_COLOR);
//         // lv_obj_align(mlx90640_canvas, LV_ALIGN_TOP_LEFT, 0, 0);
//         // lv_canvas_fill_bg(mlx90640_canvas, lv_color_black(), LV_OPA_COVER);
        
//         lv_obj_set_size(mlx90640_canvas[i], TARGET_WIDTH, LINE_BUFFER_SIZE);
//         get_line_buffer(input_image, line_buffer, i);
//         lv_canvas_set_buffer(mlx90640_canvas[i], line_buffer, TARGET_WIDTH, LINE_BUFFER_SIZE, LV_IMG_CF_TRUE_COLOR);
//         // lv_obj_align(mlx90640_canvas[i], LV_ALIGN_TOP_LEFT, 0, i*LINE_BUFFER_SIZE);
//         // interpolate_line(input_image, i, line_buffer);
        
//         lv_obj_set_pos(mlx90640_canvas[i], 0, i*LINE_BUFFER_SIZE);
//         // 重绘
//         lv_obj_invalidate(mlx90640_canvas[i]);
//     }
// }

// // 双线性插值函数
// void linear_interpolation() {
//     // 原图和目标图的比例
//     float x_ratio = (float)(SRC_WIDTH - 1) / (TARGET_WIDTH - 1);
//     float y_ratio = (float)(SRC_HEIGHT - 1) / (TARGET_HEIGHT - 1);

//     // 遍历目标图像的每个像素
//     for (int y = 0; y < TARGET_HEIGHT; ++y) {
//         for (int x = 0; x < TARGET_WIDTH; ++x) {
//             // 在原图中的坐标
//             float src_x = x * x_ratio;
//             float src_y = y * y_ratio;

//             // 整数部分
//             int x0 = (int)src_x;
//             int y0 = (int)src_y;

//             // 浮点部分
//             float dx = src_x - x0;
//             float dy = src_y - y0;

//             // 边界处理
//             int x1 = (x0 + 1 < SRC_WIDTH) ? x0 + 1 : x0;
//             int y1 = (y0 + 1 < SRC_HEIGHT) ? y0 + 1 : y0;

//             // 获取原图四个相邻像素的RGB565值
//             uint16_t Q11 = ((uint16_t*)input_image)[y0 * SRC_WIDTH + x0];
//             uint16_t Q21 = ((uint16_t*)input_image)[y0 * SRC_WIDTH + x1];
//             uint16_t Q12 = ((uint16_t*)input_image)[y1 * SRC_WIDTH + x0];
//             uint16_t Q22 = ((uint16_t*)input_image)[y1 * SRC_WIDTH + x1];

//             // 分离RGB分量
//             uint8_t R11 = (Q11 >> 11) & 0x1F; // 红色分量
//             uint8_t G11 = (Q11 >> 5) & 0x3F;  // 绿色分量
//             uint8_t B11 = Q11 & 0x1F;         // 蓝色分量

//             uint8_t R21 = (Q21 >> 11) & 0x1F;
//             uint8_t G21 = (Q21 >> 5) & 0x3F;
//             uint8_t B21 = Q21 & 0x1F;

//             uint8_t R12 = (Q12 >> 11) & 0x1F;
//             uint8_t G12 = (Q12 >> 5) & 0x3F;
//             uint8_t B12 = Q12 & 0x1F;

//             uint8_t R22 = (Q22 >> 11) & 0x1F;
//             uint8_t G22 = (Q22 >> 5) & 0x3F;
//             uint8_t B22 = Q22 & 0x1F;

//             // 插值计算
//             uint8_t R = (uint8_t)(
//                 R11 * (1 - dx) * (1 - dy) +
//                 R21 * dx * (1 - dy) +
//                 R12 * (1 - dx) * dy +
//                 R22 * dx * dy);

//             uint8_t G = (uint8_t)(
//                 G11 * (1 - dx) * (1 - dy) +
//                 G21 * dx * (1 - dy) +
//                 G12 * (1 - dx) * dy +
//                 G22 * dx * dy);

//             uint8_t B = (uint8_t)(
//                 B11 * (1 - dx) * (1 - dy) +
//                 B21 * dx * (1 - dy) +
//                 B12 * (1 - dx) * dy +
//                 B22 * dx * dy);

//             // 将RGB值合并为RGB565格式
//             uint16_t interpolated_pixel = ((R & 0x1F) << 11) | ((G & 0x3F) << 5) | (B & 0x1F);

//             // 写入目标图像
//             ((uint16_t*)output_image)[y * TARGET_WIDTH + x] = interpolated_pixel;
//         }
//     }
// }

