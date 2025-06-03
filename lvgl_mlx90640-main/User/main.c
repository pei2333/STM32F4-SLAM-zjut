#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <stdint.h>  // 为 uint8_t, uint16_t 等类型
#include <stdlib.h>  // 为 abs() 函数
#include <stdio.h>
#include <string.h>
#include <math.h>

// LVGL相关头文件
#include "lvgl.h"
#include "lvgl/src/core/lv_obj.h"
#include "lvgl/src/widgets/lv_canvas.h"
#include "lvgl/src/widgets/lv_label.h"
#include "lvgl/src/misc/lv_color.h"
#include "lvgl/src/misc/lv_timer.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"

// 项目相关头文件
#include "delay.h"
#include "LED.h"
#include "KEY.h"
#include "lcd.h"
#include "../BaseDrive/TOUCH/touch.h"
#include "../BaseDrive/USART.h"
#include "adxl345.h"

/**********************************************************************************************************
 *  SLAM + 动态障碍检测
**********************************************************************************************************/

// 雷达点数量
#define LASER_POINT_COUNT 32

// 栅格地图参数
#define MAP_SIZE_X       240   // X 方向格子数量
#define MAP_SIZE_Y       240   // Y 方向格子数量
#define MAP_RESOLUTION   20    // 每格代表多少毫米 (20mm)

// 单元格状态
typedef enum {
    CELL_UNKNOWN = 0,  // 未知
    CELL_FREE    = 1,  // 自由
    CELL_OCCUPIED = 2  // 占用
} CellState;

// 机器人位姿
typedef struct {
    float x;      // 单位：格子数（实际距离 / MAP_RESOLUTION）
    float y;      // 单位：格子数
    float theta;  // 朝向，单位：弧度
} RobotPose;

// 雷达点结构
typedef struct {
    float distance;   // 单位：毫米
    float angle;      // 单位：弧度
    uint8_t intensity;
} LaserPoint;

// 栅格地图
typedef struct {
    CellState cells[MAP_SIZE_X][MAP_SIZE_Y];
    int origin_x;   // 地图原点：对应机器人 x=0 时所对应的格子坐标
    int origin_y;   // 地图原点：对应 robot y=0
} GridMap;

// 动态地图数据（用于检测动态障碍）
typedef struct {
    uint16_t hit_count[MAP_SIZE_X][MAP_SIZE_Y];   // 被击中次数
    uint16_t scan_count[MAP_SIZE_X][MAP_SIZE_Y];  // 被扫描次数
    CellState dynamic_cells[MAP_SIZE_X][MAP_SIZE_Y]; // 标记为动态障碍
} DynamicMap;

// 全局变量
static RobotPose  robot_pose = {0};
static LaserPoint laser_points[LASER_POINT_COUNT];
static GridMap    slam_map;
static DynamicMap dynamic_map;
static volatile uint8_t slam_running = 1;  // SLAM 开始/停止标志

// LVGL 对象
static lv_obj_t *map_canvas;
static lv_obj_t *info_label;

// 显示颜色
#define COLOR_UNKNOWN   lv_color_make(100, 100, 100)
#define COLOR_FREE      lv_color_make(255, 255, 255)
#define COLOR_OCCUPIED  lv_color_make(0, 0, 0)
#define COLOR_ROBOT     lv_color_make(255, 0, 0)
#define COLOR_PATH      lv_color_make(0, 255, 0)
#define COLOR_DYNAMIC   lv_color_make(0, 0, 255)

// 地图显示缓冲区 (240×240×2B ≈ 112.5KB，请放到外部 SRAM)
static lv_color_t map_display_buffer[MAP_SIZE_X * MAP_SIZE_Y];

/**
 * @brief  将实际世界坐标转换为地图格子索引
 * @param  map: 指向 GridMap
 * @param  x_world: 机器人/点的世界坐标 x（单位：毫米）
 * @param  y_world: 机器人/点的世界坐标 y（单位：毫米）
 * @param  x_map: 输出：格子坐标 x
 * @param  y_map: 输出：格子坐标 y
 */
static void worldToMap(GridMap *map, float x_world, float y_world, int *x_map, int *y_map)
{
    // 先把毫米换算成"格子数"
    float gx = x_world / MAP_RESOLUTION;
    float gy = y_world / MAP_RESOLUTION;

    *x_map = map->origin_x + (int)gx;
    *y_map = map->origin_y + (int)gy;

    if (*x_map < 0)               *x_map = 0;
    else if (*x_map >= MAP_SIZE_X) *x_map = MAP_SIZE_X - 1;
    if (*y_map < 0)               *y_map = 0;
    else if (*y_map >= MAP_SIZE_Y) *y_map = MAP_SIZE_Y - 1;
}

/**
 * @brief  更新单元格状态
 */
static void updateCell(GridMap *map, int x_map, int y_map, CellState state)
{
    if (x_map >= 0 && x_map < MAP_SIZE_X && y_map >= 0 && y_map < MAP_SIZE_Y) {
        map->cells[x_map][y_map] = state;
    }
}

/**
 * @brief  使用增强版 Bresenham 算法：既更新静态地图，也统计动态障碍
 * @param  map: 栅格地图
 * @param  dmap: 动态地图
 * @param  x0, y0: 起点（机器人所在格子）
 * @param  x1, y1: 终点（激光击中格子）
 */
static void enhancedTraceRay(GridMap *map, DynamicMap *dmap, int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;
    int initial_cells = 2;  // 前 2 格不标记自由，以免把机器人附近也标成 free

    // 第一次过线：只累加 scan_count
    while (1) {
        if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y && initial_cells <= 0) {
            dmap->scan_count[x][y]++;
        }

        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
        initial_cells--;
    }

    // 第二次：标记 free / occupied，并更新 hit_count + dynamic_cells
    x = x0; y = y0;
    err = dx - dy;
    initial_cells = 2;
    while (1) {
        if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y && initial_cells <= 0) {
            if (x == x1 && y == y1) {
                updateCell(map, x, y, CELL_OCCUPIED);
                dmap->hit_count[x][y]++;
                // 当 scan_count 足够大时，判断动态占用
                if (dmap->scan_count[x][y] > 10) {
                    float hit_ratio = (float)dmap->hit_count[x][y] / dmap->scan_count[x][y];
                    if (hit_ratio < 0.4f) {
                        dmap->dynamic_cells[x][y] = CELL_OCCUPIED;
                    } else {
                        dmap->dynamic_cells[x][y] = CELL_UNKNOWN;
                    }
                }
            } else {
                updateCell(map, x, y, CELL_FREE);
            }
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
        initial_cells--;
    }
}

/**
 * @brief  解析一帧雷达数据，并更新栅格地图 + 动态障碍
 *
 * 串口协议：Rxbuf[] 已经装满 108 字节：
 * Byte0~Byte1：A5 5A
 * Byte2：Length
 * Byte3~Byte4：Speed
 * Byte5~Byte6：start_angle_raw  (uint16, 单位 = 0.01°)
 * Byte7~Byte(7+3*31)：32×(Dist_H, Dist_L, Intensity)
 * Byte(103)~Byte(104)：stop_angle_raw (uint16, 单位 = 0.01°)
 * Byte107：校验和
 *
 */
static void enhancedUpdateMapWithLaserData(GridMap *map,
                                           DynamicMap *dmap,
                                           RobotPose *pose,
                                           LaserPoint *laser_points,
                                           int num_points)
{
    // 先把机器人当前格子坐标算出来
    int robot_x_map, robot_y_map;
    worldToMap(map, pose->x * MAP_RESOLUTION, pose->y * MAP_RESOLUTION,
               &robot_x_map, &robot_y_map);

    // 获取UART5接收缓冲区
    uint8_t *uart_buffer = UART5_GetReceiveBuffer();
    if(uart_buffer == NULL) return;

    // 解析起始/终止角度
    uint16_t start_raw = (uint16_t)(uart_buffer[5] << 8) | uart_buffer[6];
    uint16_t stop_raw  = (uint16_t)(uart_buffer[103] << 8) | uart_buffer[104];
    float start_deg = start_raw * 0.01f;
    float stop_deg  = stop_raw  * 0.01f;
    float diff_deg = stop_deg - start_deg;
    if (diff_deg < 0) diff_deg += 360.0f;
    float step_deg = diff_deg / (num_points - 1);

    for (int i = 0; i < num_points; i++) {
        uint16_t dist = (uint16_t)(uart_buffer[7 + i*3] << 8)
                      | uart_buffer[8 + i*3];
        uint8_t intensity = uart_buffer[9 + i*3];

        float angle_deg = start_deg + step_deg * i;
        if (angle_deg >= 360.0f) angle_deg -= 360.0f;
        float angle_rad = angle_deg * (3.14159265f / 180.0f);

        laser_points[i].distance  = (float)dist;
        laser_points[i].angle     = angle_rad + pose->theta;
        laser_points[i].intensity = intensity;
    }

    // 调用增强版 raycasting
    for (int i = 0; i < num_points; i++) {
        if (laser_points[i].distance > 0 && laser_points[i].distance < 5000) {
            float px = pose->x * MAP_RESOLUTION + laser_points[i].distance * cosf(laser_points[i].angle);
            float py = pose->y * MAP_RESOLUTION + laser_points[i].distance * sinf(laser_points[i].angle);
            int px_map, py_map;
            worldToMap(map, px, py, &px_map, &py_map);
            enhancedTraceRay(map, dmap, robot_x_map, robot_y_map, px_map, py_map);
        }
    }
}

/**
 * @brief 将地图数据画到 LVGL Canvas 上
 */
static void drawMap(void)
{
    for (int y = 0; y < MAP_SIZE_Y; y++) {
        for (int x = 0; x < MAP_SIZE_X; x++) {
            lv_color_t color;
            if (dynamic_map.dynamic_cells[x][y] == CELL_OCCUPIED) {
                color = COLOR_DYNAMIC;
            } else {
                switch (slam_map.cells[x][y]) {
                    case CELL_UNKNOWN:   color = COLOR_UNKNOWN;   break;
                    case CELL_FREE:      color = COLOR_FREE;      break;
                    case CELL_OCCUPIED:  color = COLOR_OCCUPIED;  break;
                    default:             color = COLOR_UNKNOWN;   break;
                }
            }
            map_display_buffer[y * MAP_SIZE_X + x] = color;
        }
    }

    // 机器人在格子中心画红点 (3×3)
    int robot_x_map, robot_y_map;
    worldToMap(&slam_map,
               robot_pose.x * MAP_RESOLUTION,
               robot_pose.y * MAP_RESOLUTION,
               &robot_x_map, &robot_y_map);

    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int xx = robot_x_map + dx;
            int yy = robot_y_map + dy;
            if (xx >= 0 && xx < MAP_SIZE_X && yy >= 0 && yy < MAP_SIZE_Y) {
                map_display_buffer[yy * MAP_SIZE_X + xx] = COLOR_ROBOT;
            }
        }
    }

    // 绘制机器人朝向线 (长度 10 格)
    int len = 10;
    int end_x = robot_x_map + (int)(cosf(robot_pose.theta) * len);
    int end_y = robot_y_map + (int)(sinf(robot_pose.theta) * len);
    int dx = abs(end_x - robot_x_map);
    int dy = abs(end_y - robot_y_map);
    int sx = (robot_x_map < end_x) ? 1 : -1;
    int sy = (robot_y_map < end_y) ? 1 : -1;
    int err = dx - dy;
    int x = robot_x_map;
    int y = robot_y_map;
    while (1) {
        if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
            map_display_buffer[y * MAP_SIZE_X + x] = COLOR_PATH;
        }
        if (x == end_x && y == end_y) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }

    // 将缓冲区写到 LVGL Canvas
    lv_canvas_set_buffer(map_canvas,
                         map_display_buffer,
                         MAP_SIZE_X,
                         MAP_SIZE_Y,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_invalidate(map_canvas);
}

/**
 * @brief LVGL 定时器回调：每 100ms 刷新一次地图与信息标签
 */
static void update_map_display(lv_timer_t *timer)
{
    drawMap();
    lv_label_set_text_fmt(info_label,
                          "X: %.2f\nY: %.2f\nθ: %.1f°",
                          robot_pose.x,
                          robot_pose.y,
                          robot_pose.theta * (180.0f / 3.14159265f));
}

/**
 * @brief "开始/暂停"按钮回调
 */
static void start_stop_event_cb(lv_event_t *e)
{
    lv_obj_t *btn   = lv_event_get_target(e);
    lv_obj_t *label = lv_obj_get_child(btn, NULL);

    slam_running = !slam_running;
    if (slam_running) {
        lv_label_set_text(label, "暂停");
    } else {
        lv_label_set_text(label, "开始");
    }
}

/**
 * @brief "清空地图"按钮回调
 */
static void clear_map_event_cb(lv_event_t *e)
{
    initMap(&slam_map);
    initDynamicMap(&dynamic_map);
    robot_pose.x = 0.0f;
    robot_pose.y = 0.0f;
    robot_pose.theta = 0.0f;
}

/**
 * @brief 按键处理：KEY1=开始/暂停，KEY2=清空地图，KEY3=设置当前位置为 (0,0)
 */
static void processKeys(void)
{
    static uint32_t last_key_time = 0;
    uint32_t current_time = lv_tick_get();
    if (current_time - last_key_time < 300) return;

    if (KEY1_PRESSED()) {
        slam_running = !slam_running;
        last_key_time = current_time;
    }
    if (KEY2_PRESSED()) {
        initMap(&slam_map);
        initDynamicMap(&dynamic_map);
        robot_pose.x = 0;
        robot_pose.y = 0;
        robot_pose.theta = 0;
        last_key_time = current_time;
    }
    if (KEY3_PRESSED()) {
        robot_pose.x = 0;
        robot_pose.y = 0;
        robot_pose.theta = 0;
        last_key_time = current_time;
    }
}

/**
 * @brief 触摸平移：将 canvas 左上角位置偏移而不影响 robot_pose
 * @note 这里示例把偏移直接应用到 robot_pose，推荐改为单独 view_offset
 */
static void processTouchPan(void)
{
    static uint8_t touching = 0;
    static int16_t last_x = 0, last_y = 0;
    if (tp_dev.sta & TP_PRES_DOWN) {
        if (!touching) {
            touching = 1;
            last_x = tp_dev.x[0];
            last_y = tp_dev.y[0];
        } else {
            int16_t dx = tp_dev.x[0] - last_x;
            int16_t dy = tp_dev.y[0] - last_y;
            if (dx || dy) {
                // 建议：改为视图偏移，而不是修改 robot_pose
                // robot_pose.x -= dx * (float)MAP_RESOLUTION / 10.0f;
                // robot_pose.y -= dy * (float)MAP_RESOLUTION / 10.0f;
                lv_obj_set_x(map_canvas, lv_obj_get_x(map_canvas) + dx);
                lv_obj_set_y(map_canvas, lv_obj_get_y(map_canvas) + dy);
                last_x = tp_dev.x[0];
                last_y = tp_dev.y[0];
            }
        }
    } else {
        touching = 0;
    }
}

/**
 * @brief 创建 SLAM 界面：Canvas + 控制按钮 + 信息标签
 */
static void createSlamUI(void)
{
    // 填充初始地图缓冲区为 UNKNOWN
    for (int i = 0; i < MAP_SIZE_X * MAP_SIZE_Y; i++) {
        map_display_buffer[i] = COLOR_UNKNOWN;
    }
    // 创建 Canvas
    map_canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(map_canvas,
                         map_display_buffer,
                         MAP_SIZE_X,
                         MAP_SIZE_Y,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(map_canvas, 0, 0);

    // "开始/暂停"按钮
    lv_obj_t *btn_start = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn_start, MAP_SIZE_X + 10, 10);
    lv_obj_set_size(btn_start, 70, 40);
    lv_obj_t *lbl_start = lv_label_create(btn_start);
    lv_label_set_text(lbl_start, "暂停");
    lv_obj_center(lbl_start);
    lv_obj_add_event_cb(btn_start, start_stop_event_cb, LV_EVENT_CLICKED, NULL);

    // "清空地图"按钮
    lv_obj_t *btn_clear = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn_clear, MAP_SIZE_X + 10, 60);
    lv_obj_set_size(btn_clear, 70, 40);
    lv_obj_t *lbl_clear = lv_label_create(btn_clear);
    lv_label_set_text(lbl_clear, "清空");
    lv_obj_center(lbl_clear);
    lv_obj_add_event_cb(btn_clear, clear_map_event_cb, LV_EVENT_CLICKED, NULL);

    // 信息标签
    info_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(info_label, MAP_SIZE_X + 10, 110);
    lv_label_set_text(info_label, "X: 0\nY: 0\nθ: 0°");

    // 提示文字
    lv_obj_t *lbl_hint = lv_label_create(lv_scr_act());
    lv_obj_set_pos(lbl_hint, MAP_SIZE_X + 10, 160);
    lv_label_set_text(lbl_hint,
                      "KEY1: 开始/暂停\n"
                      "KEY2: 清空地图\n"
                      "KEY3: 重置位置\n"
                      "触摸: 平移视图");

    // 定时刷新地图
    lv_timer_create(update_map_display, 100, NULL);
}

/**
 * @brief SLAM处理函数：读取传感器数据，更新机器人姿态和地图
 */
void SLAM_Process(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = lv_tick_get();
    float dt = (current_time - last_time) / 1000.0f;  // 转换为秒
    
    if(dt < 0.001f) return;  // 防止时间间隔过小
    
    last_time = current_time;

    // 读取ADXL345数据（使用新的接口）
    if(ADXL345_ReadData() == ADXL345_SUCCESS) {
        // 使用滤波后的加速度数据
        static float velocity_x = 0, velocity_y = 0;
        static float position_x = 0, position_y = 0;
        
        // 将加速度从g转换为m/s²
        float acc_x = adxl345_data.x * 9.8f;
        float acc_y = adxl345_data.y * 9.8f;
        
        // 简单的加速度积分计算速度和位置
        velocity_x += acc_x * dt;
        velocity_y += acc_y * dt;
        
        position_x += velocity_x * dt;
        position_y += velocity_y * dt;
        
        // 更新机器人位置（转换为地图坐标）
        robot_pose.x = (int)(position_x * 100);  // 转换为cm
        robot_pose.y = (int)(position_y * 100);
        
        // 限制在地图范围内
        if(robot_pose.x < 0) robot_pose.x = 0;
        if(robot_pose.x >= MAP_SIZE_X) robot_pose.x = MAP_SIZE_X - 1;
        if(robot_pose.y < 0) robot_pose.y = 0;
        if(robot_pose.y >= MAP_SIZE_Y) robot_pose.y = MAP_SIZE_Y - 1;
    } else {
        // ADXL345读取失败，可以在这里处理错误
        LED2_ON;  // 点亮LED2表示传感器错误
    }

    // 读取激光雷达数据
    if(UART5_IsReceiveFinished()) {
        enhancedUpdateMapWithLaserData(&slam_map,
                                       &dynamic_map,
                                       &robot_pose,
                                       laser_points,
                                       LASER_POINT_COUNT);
        UART5_ClearReceiveFlag();  // 清除接收标志
    }

    // 更新地图显示
    drawMap();
}

/**
 * @brief 系统全局初始化：延时、LED、KEY、UART、ADXL345、TIM3、地图、LVGL
 */
void SystemInit(void)
{
    delay_init();
    LED_Init();
    KEY_Init();

    // 初始化调试串口
    UART3_Init();

    // USART5 用于雷达
    UART5_Init(460800);

    // 初始化ADXL345加速度计（使用新的I2C接口）
    if(ADXL345_Init() != ADXL345_SUCCESS) {
        // ADXL345初始化失败，可以在这里处理错误
        // 例如闪烁LED或发送错误信息
        for(int i = 0; i < 5; i++) {
            LED1_ON;
            delay_ms(200);
            LED1_OFF;
            delay_ms(200);
        }
    }

    // 不再初始化MLX90640热成像模块 - 用户不需要
    // MLX90640_Init();

    // 定时器配置
    TIM3_Init();

    // 初始化地图
    initMap(&slam_map);
    initDynamicMap(&dynamic_map);

    // 初始化LVGL
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    // 创建UI界面
    createSlamUI();
}

/**
 * @brief 主函数
 */
int main(void)
{
    SystemInit();

    while (1) {
        processKeys();
		tp_dev.scan(0);
        processTouchPan();
        SLAM_Process();
        lv_task_handler();
        delay_ms(5);
    }
    // return 0; // 虽然不会走到这里
}

/**
 * @brief 初始化栅格地图
 */
static void initMap(GridMap *map)
{
    if(map == NULL) return;
    
    // 清空地图
    for(int x = 0; x < MAP_SIZE_X; x++) {
        for(int y = 0; y < MAP_SIZE_Y; y++) {
            map->cells[x][y] = CELL_UNKNOWN;
        }
    }
    
    // 设置地图原点为中心
    map->origin_x = MAP_SIZE_X / 2;
    map->origin_y = MAP_SIZE_Y / 2;
}

/**
 * @brief 初始化动态地图
 */
static void initDynamicMap(DynamicMap *dmap)
{
    if(dmap == NULL) return;
    
    // 清空动态地图
    for(int x = 0; x < MAP_SIZE_X; x++) {
        for(int y = 0; y < MAP_SIZE_Y; y++) {
            dmap->hit_count[x][y] = 0;
            dmap->scan_count[x][y] = 0;
            dmap->dynamic_cells[x][y] = CELL_UNKNOWN;
        }
    }
}
