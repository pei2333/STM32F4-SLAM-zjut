#include "mlx90640.h"
#include "string.h"
#include "../../lvgl/lvgl.h"

uint8_t mlx90640_buf[1544]= {0};
uint8_t state = 0;
uint8_t is_update = 1;

// 双缓冲区
uint8_t disp_finish_flag = 0;
uint8_t updating_buffer_flag = 0; // 0: buffer_a, 1: buffer_b
uint8_t buffer_a[SRC_WIDTH * SRC_HEIGHT * 2];
uint8_t buffer_b[SRC_WIDTH * SRC_HEIGHT * 2];

static lv_img_dsc_t dynamic_img_dsc = {
    .header.always_zero = 0,
    .header.w = SRC_WIDTH,
    .header.h = SRC_HEIGHT,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = (const uint8_t*)buffer_b,  // 初始使用 buffer_b
    .data_size = SRC_WIDTH * SRC_HEIGHT * 2,
};

void swap_buffers(lv_obj_t* img){
    if(updating_buffer_flag == 0){
        dynamic_img_dsc.data = (const uint8_t*)buffer_b;
    }else{
        dynamic_img_dsc.data = (const uint8_t*)buffer_a;
    }

    // if(dynamic_img_dsc.data == (const uint8_t*)buffer_a){
    //     dynamic_img_dsc.data = (const uint8_t*)buffer_b;
    // }else{
    //     dynamic_img_dsc.data = (const uint8_t*)buffer_a;
    // }
    lv_img_set_src(img, &dynamic_img_dsc);
    // 更新图像源，并刷新
    lv_obj_invalidate(img);

    updating_buffer_flag = !updating_buffer_flag;
}
void update_buffer(uint8_t* src){
    if(updating_buffer_flag == 0){
        memcpy(buffer_a, src, SRC_WIDTH * SRC_HEIGHT * 2);
    }else{
        memcpy(buffer_b, src, SRC_WIDTH * SRC_HEIGHT * 2);
    }
}
void create_dynamic_image(lv_obj_t* parent){
    lv_obj_t* img = lv_img_create(parent);
    lv_img_set_src(img, &dynamic_img_dsc);
    lv_img_set_zoom(img, 256 * 8);
    lv_obj_align(img, LV_ALIGN_TOP_LEFT, 144, 115);
    
    // lv_obj_align(img, LV_ALIGN_TOP_LEFT, 0, 0);
}

uint8_t Check(uint8_t *data)
{
    uint16_t sum=0,length=0,i=0;
    uint16_t temp=0;
    length=((uint16_t )mlx90640_buf[3]<<8)|mlx90640_buf[2]+6;
    if(length>1544)//超过上传数据
        return 0;
    for(i=0; i<length-2; i=i+2)
    {
        temp=((uint16_t )mlx90640_buf[i+1]<<8)|mlx90640_buf[i];
        sum+=temp;
    }
    temp=((uint16_t )mlx90640_buf[i+1]<<8)|mlx90640_buf[i];
    if(sum==temp)
    {
        // memcpy(data,mlx90640_buf,length);
        for(i=0; i<length; i++)
        {
            data[i]=mlx90640_buf[i];
            // printf("%x", data[i]);
        }
        return 1;
    }
    else
        return 0;
}