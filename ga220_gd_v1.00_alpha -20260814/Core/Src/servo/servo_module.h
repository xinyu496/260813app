/**
 * @file    servo_module.h
 * @brief   重构对象后的处理（对外接口）
 *
 *          把 SF_20260303.c 中散落的全局变量与裸函数调用，封装为
 *          dev_servo_t 对象 + 函数指针的方式后，对外暴露的薄封装层。
 *
 *          典型用法：
 *            ① 上电时调用一次 servo_module_init()
 *            ② 1ms 中断/定时器里调用 servo_module_handler()
 * 
 * @author  LinHui
 * @version 1.00
 * @date    2026-06-01
 */

#ifndef _SERVO_MODULE_H_
#define _SERVO_MODULE_H_

#include <stdbool.h>
#include <stdint.h>
#include "servo_test.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------
 *   仿真/在线调试用：可实时修改的激励测试参数
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t axis;            /* 0=FW(方位)  1=GD(俯仰)              */
    uint8_t loop;            /* 1..7，见 servo_test_loop_e          */
    uint8_t mode;            /* 0=开环  1=闭环                      */
    uint8_t ctrl;            /* 0=空闲  1=请求启动  2=请求停止（处理后自动清 0）*/
    float amp;               /* 激励幅值（单位随所测环路而定）        */
    float freq;              /* 正弦频率(Hz)，0=阶跃                 */
    float time_s;            /* 持续时间(s)                         */
    servo_test_wave_e wave;  /**< 激励信号类型（新增） */
    uint32_t step_delay_ms;  /**< 阶跃起跳延迟(ms)，仅 STEP 用，0=立即 */

    float step_min;          /**< 周期阶跃下限（PULSE 用） */
    float step_max;          /**< 周期阶跃上限（PULSE 用） */
    uint32_t step_period_ms; /**< 周期阶跃整周期(ms)，PULSE 用，50% 占空比 */
} servo_test_dbg_t;

extern volatile servo_test_dbg_t g_test;
extern dev_servo_t g_servo;

/* ========================================================================
 *                              公开 API
 * ====================================================================== */

/**
 * @brief 伺服模块初始化
 *
 *        装配 FW / GD 两轴的 IO 钩子（电机驱动器、编码器、陀螺），
 *        并调用 dev_servo_init() 初始化顶层伺服对象。上电后调用一次。
 */
void servo_module_init(void);

/**
 * @brief 伺服模块周期入口
 *
 *        在 1ms 中断或定时器中调用一次，等价于原 SF_20260303.c 中的
 *        alldeal + commandprocess + allprocess 一整套流程。
 */
void servo_module_handler(void);

/** @brief 外部控制指令码（来源通信协议） */
typedef enum {
    SERVO_EXT_CMD_BRAKE = 0x01,    /**< 刹车：无参数；待机/软刹由模块内部设置决定 */
    SERVO_EXT_CMD_VELOCITY = 0x02, /**< 速度控制：velocity.fw/gd（°/s） */
    SERVO_EXT_CMD_POSITION = 0x03, /**< 位置运动：position.fw/gd（°） */
    SERVO_EXT_CMD_GUIDE = 0x04,    /**< 引导：guide.经/纬/高 + flag（当前仅缓存，GEO 未接） */
    SERVO_EXT_CMD_TRACK = 0x05,    /**< 点选跟踪：track.fw_miss/gd_miss（°） */
    SERVO_EXT_CMD_STOW = 0x06,     /**< 收藏 */
    SERVO_EXT_CMD_STEP = 0x0A      /**< 步进运动：step.方向 + 步长（°） */
} servo_ext_cmd_e;

/**
 * @brief 外部控制指令包（已解析好的指令码 + 参数）
 */
typedef struct {
    servo_ext_cmd_e cmd; /**< 指令码 */
    union {
        // clang-format off
        /* 0x01 刹车：无参数（待机/软刹由模块内部设置决定） */
        
        /** 0x02 速度控制（°/s） */
        struct {
            float fw;       /**< 方位速度 */
            float gd;       /**< 俯仰速度 */
        } velocity;
        /** 0x03 位置运动（°） */
        struct {
            float fw;       /**< 方位角度 */
            float gd;       /**< 俯仰角度 */
        } position;
        /** 0x04 引导模式 */
        struct {
            float lon;      /**< 经度(°)，无效值 400 */
            float lat;      /**< 纬度(°)，无效值 400 */
            float alt;      /**< 高度(m) */
            uint8_t flag;   /**< 0=停止跟踪 1=开始跟踪 */
        } guide;
        /** 0x05 点选跟踪（°） */
        struct {
            float fw_miss;      /**< 方位脱靶量 */
            float gd_miss;      /**< 俯仰脱靶量 */
            uint8_t is_track;   /**< 0=停止跟踪 1=开始跟踪 */
        } track;
        /** 0x0A 步进运动（步长 °） */
        struct {
            uint8_t fw_dir; /**< 方位方向：0x01 左 / 0x02 右 */
            float fw_step;  /**< 方位步长(°) */
            uint8_t gd_dir; /**< 俯仰方向：0x01 上 / 0x02 下 */
            float gd_step;  /**< 俯仰步长(°) */
        } step;
        // clang-format on
    } p;
} servo_ext_cmd_t;

/**
 * @brief 外部控制指令入口（模式切换状态机）
 *        通信层填好 servo_ext_cmd_t 后调用一次。本模块只做「指令 → 内部模式/目标」的翻译；实际生效在下一拍 servo_module_handler() 内完成。
 *
 * @param c 已解析好的指令包（不能为 NULL）
 * @return true=已接受并执行；false=指令码非法或 c 为 NULL
 */
bool servo_module_on_command(servo_ext_cmd_t *c);  //const

#ifdef __cplusplus
}
#endif

#endif /* _SERVO_MODULE_H_ */
