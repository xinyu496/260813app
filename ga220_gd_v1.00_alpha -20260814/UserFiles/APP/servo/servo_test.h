/**
 * @file    servo_test.h
 * @brief   伺服系统激励测试对象（重构自 SF_20260303.c::Test）
 * 
 * @note    激励期间不周期清除。本对象只在 start() 与超时/ stop() 时清环，tick() 内只注入+级联控制+使能。
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-06-01
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version <th>Author  <th>Description
 * <tr><td>2026-06-01   <td>1.00    <td>LinHui  <td>由 SF_20260303.c::Test 重构而来
 * </table>
 */

#ifndef _SERVO_TEST_H_
#define _SERVO_TEST_H_

#include "servo_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 *                              枚举定义
 * ====================================================================== */

/** @brief 受测控制环 */
typedef enum {
    SERVO_TEST_LOOP_CURRENT = 1, /**< 电流环（开环=电压开环 / 闭环=电流闭环） */
    SERVO_TEST_LOOP_ACC = 2,     /**< 角加速度环 */
    SERVO_TEST_LOOP_VEL = 3,     /**< 速度环（陀螺反馈） */
    SERVO_TEST_LOOP_EV = 4,      /**< 编码器速度环 */
    SERVO_TEST_LOOP_POSITION = 5, /**< 位置环 */
    SERVO_TEST_LOOP_TRACK = 6,   /**< 测速 / 脱靶量跟踪环（用 rt.T_fb，忽略 amp/freq） */
    SERVO_TEST_LOOP_GEO = 7,     /**< GEO 地理跟踪环 */
} servo_test_loop_e;


/** @brief 开/闭环（对应原 Test() 的 O_or_C 参数） */
typedef enum {
    SERVO_TEST_OPEN_LOOP = 0,  /**< 开环 */
    SERVO_TEST_CLOSED_LOOP = 1 /**< 闭环 */
} servo_test_loop_mode_e;

/** @brief 激励信号类型 */
typedef enum {
    SERVO_TEST_WAVE_FIXED = 0, /**< 固定值：恒为 amp */
    SERVO_TEST_WAVE_STEP = 1,  /**< 阶跃：tick<step_delay_ms 输出 0，之后跳到 amp */
    SERVO_TEST_WAVE_SINE = 2,  /**< 正弦：amp*sin(ω·freq·t) */
    SERVO_TEST_WAVE_PULSE = 3  /**< 周期阶跃(方波)：在 step_min/step_max 间按 step_period_ms 周期来回跳 */
} servo_test_wave_e;

/* ========================================================================
 *                              结构体定义
 * ====================================================================== */

/**
 * @brief 一次激励测试的参数包（替代原 Test() 的 6 个裸参数）
 */
typedef struct {
    servo_axis_id_e axis;        /**< 受测轴：SERVO_AXIS_FW / SERVO_AXIS_GD */
    servo_test_loop_e loop;      /**< 受测环：servo_test_loop_e */
    servo_test_loop_mode_e mode; /**< 开/闭环：servo_test_loop_mode_e */
    servo_test_wave_e wave;      /**< 激励信号类型 */
    float amp;                   /**< 激励幅值 */
    float freq;                  /**< 激励频率(Hz)，0=阶跃 */
    uint32_t step_delay_ms;      /**< 阶跃起跳延迟(ms)，STEP/PULSE 用，0=立即 */

    float step_min;              /**< 周期阶跃下限（PULSE 用） */
    float step_max;              /**< 周期阶跃上限（PULSE 用） */
    uint32_t step_period_ms;     /**< 周期阶跃整周期(ms)，PULSE 用，50% 占空比 */

    float time_s;                /**< 持续时间(s) */
} servo_test_spec_t;             /*  specification，规格说明（参数包） */

/* 前置声明：用于函数指针中引用自身 */
struct dev_servo_test;

/**
 * @brief 伺服激励测试对象
 *
 *        关联一个 dev_servo_t（顶层伺服），按 1ms 周期 tick() 推进一次激励。
 *        所有动作通过 axis 对象的方法指针完成，不再裸调 gd*loop / fw*loop。
 */
typedef struct dev_servo_test {
    /* ---- 关联与状态 ---- */
    dev_servo_t *servo;     /**< 关联的顶层伺服对象 */
    servo_test_spec_t spec; /**< 当前测试参数 */
    uint32_t tick_count;    /**< 1ms 计数（等价原 Test_Count） */
    bool running;           /**< 是否处于测试中 */
    uint8_t out_flag;       /**< 输出模式标志（等价原 Test_flag）：
                                  1=GD 电流输出, 2=FW 电流输出, 3=电压输出 */

    /* ====================================================================
     *                          方法（函数指针）
     * ================================================================== */

    /**
     * @brief 启动一次测试：保存参数、清零计数、清一次环（清掉历史 PI 残留）。
     * @param spec  测试参数包
     */
    void (*start)(struct dev_servo_test *pobj, const servo_test_spec_t *spec);

    /**
     * @brief 立即停止测试：双轴清环 + 刹车。
     */
    void (*stop)(struct dev_servo_test *pobj);

    /**
     * @brief 推进一个 1ms 测试周期（等价原 Test() 的一次调用）。
     *        注入激励 → 级联控制环 → 写电机；计数到时自动清环+刹车并结束。
     */
    bool (*tick)(struct dev_servo_test *pobj);

    /** @brief 计算当前周期的激励值（固定/阶跃/正弦），便于观测。 */
    float (*excitation)(struct dev_servo_test *pobj);

    /** @brief 查询是否在测试中。 */
    bool (*is_running)(struct dev_servo_test *pobj);
} dev_servo_test_t;

/* ========================================================================
 *                              公开 API
 * ====================================================================== */

/**
 * @brief 初始化测试对象：装配方法表、绑定伺服对象、清零状态。
 * @param pobj   待初始化的测试对象
 * @param servo  已 dev_servo_init() 过的顶层伺服对象（不可为 NULL）
 */
void dev_servo_test_init(dev_servo_test_t *pobj, dev_servo_t *servo);

#ifdef __cplusplus
}
#endif

#endif /* _SERVO_TEST_H_ */
