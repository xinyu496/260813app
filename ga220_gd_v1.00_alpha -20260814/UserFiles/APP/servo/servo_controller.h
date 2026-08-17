/**
 * @file    servo_controller.h
 * @brief   伺服顶层对象（双轴 + 模式机 + 上位机指令翻译）
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-05-29
 */

#ifndef _SERVO_CONTROLLER_H_
#define _SERVO_CONTROLLER_H_

#include "servo_axis.h"

/* ========================================================================
 *                              枚举定义
 * ====================================================================== */

/**
 * @brief 伺服工作模式
 */
typedef enum {
    SERVO_MODE_WITHDRAW = 0x01,   /**< 收藏：俯仰回 -90°，方位回零 */
    SERVO_MODE_BRAKE = 0x02,      /**< 停车：清环、断电机 */
    SERVO_MODE_AFOLLOW = 0x03,    /**< 程控/记忆跟踪：位置环 */
    SERVO_MODE_TRACK = 0x04,      /**< 跟踪：脱靶量环 */
    SERVO_MODE_CIR_SCAN = 0x05,   /**< 周扫 */
    SERVO_MODE_VMOTION = 0x06,    /**< 手控速度运动 */
    SERVO_MODE_ENBRAKE = 0x07,    /**< 软刹（速度环目标置 0）*/
    SERVO_MODE_ZDLC = 0x08,       /**< 自动流程 */
    SERVO_MODE_GEO = 0x09,        /**< 地理引导：经纬高 → 目标角，跑 GEO 环 */
    SERVO_MODE_ZERO = 0x55,       /**< 回零 */
    SERVO_MODE_VOLTAGE = 0x0A,    /**< 开环 PWM 直驱（调试/特殊运行） */
    SERVO_MODE_CURRENT = 0X0B,    /** 单独电流环模式，子节点使用  */
    SERVO_MODE_SELFCHECK = 0x0C,  /**< 自检：方位保持零位，照准架依次转上限位→下限位→零位 */
} servo_mode_e;

/** @brief 自检流程步骤（仅 SERVO_MODE_SELFCHECK 下有意义） */
typedef enum {
    SERVO_SELFCHECK_STEP_TO_UP = 0,   /**< 照准架转向上限位 */
    SERVO_SELFCHECK_STEP_TO_DN = 1,   /**< 照准架转向下限位 */
    SERVO_SELFCHECK_STEP_TO_ZERO = 2, /**< 照准架回零位 */
    SERVO_SELFCHECK_STEP_DONE = 3,    /**< 自检完成：方位、照准架均驻留零位 */
} servo_selfcheck_step_e;

/** @brief 上位机指令包（聚合所有用户输入参数，替代散落的全局变量） */
typedef struct {
    uint8_t cmd; /**< 期望工作模式（servo_mode_e） */
    /* AFOLLOW / 程控 */
    float hand_search_fw_pos; /**< 方位位置目标（°） */
    float hand_search_gd_pos; /**< 俯仰位置目标（°） */
    /* VMOTION / 手控 */
    float hand_search_fw_v; /**< 方位速度目标（°/s） */
    float hand_search_gd_v; /**< 俯仰速度目标（°/s） */
    /* TRACK */
    float fw_miss_deg;  /**< 方位脱靶量（°） */
    float fy_miss_deg;  /**< 俯仰脱靶量（°） */
    uint8_t track_flag; /**< 0=不跟踪, 1=跟踪进行中, 3=稳定跟踪 */
    /* CIR_SCAN / 周扫参数 */
    float scan_speed_dps;
    float scan_amp_deg;
    /* GEO / 引导 */
    float geo_lat, geo_lon, geo_hgh; /**< 目标经度/纬度(°)、高度(m) */
    uint8_t geo_track_flag;          /**< 0=停止引导, 1=开始引导 */
} servo_cmd_t;


/** @brief 伺服上报状态（用于上位机回传 / 调试观察） */
typedef struct {
    uint16_t mode;               /**< 当前模式（servo_mode_e） */
    float fw_angle, gd_angle;    /**< 当前角度（°） */
    float fw_vel, gd_vel;        /**< 当前角速度（°/s） */
    float fw_cur, gd_cur;        /**< 电机电流（A） */
    uint16_t fw_fault, gd_fault; /**< 16 位故障位域（servo_axis_fault_t.all） */
    bool reach;                  /**< 位置到位标志（两轴均驻留到位，Dawei_PD 聚合） */
    uint8_t motion_state;        /**< 运动/静止判定（evmeancal）：1=静止, 0=运动 */
    uint8_t selfcheck_step;      /**< 自检步骤（servo_selfcheck_step_e），非自检模式下值无意义 */
} servo_state_t;

/* 前置声明 */
struct dev_servo;

/**
 * @brief 顶层伺服对象
 */
typedef struct dev_servo {
    /* ---- 内部状态 ---- */
    dev_servo_axis_t fw_axis;  /**< 方位轴对象 */
    dev_servo_axis_t gd_axis;  /**< 俯仰轴对象 */
    servo_mode_e mode;         /**< 当前工作模式 */
    servo_mode_e mode_pending; /**< 下一周期生效的模式 */
    servo_cmd_t cmd;           /**< 上位机指令缓存 */
    servo_state_t state;       /**< 上报状态缓存 */

    servo_mode_e mode_last;    /**< 上周期实际生效模式：用于"仅切入时清环" */
    uint8_t sub_last;          /**< 上周期子状态快照：VMOTION 的 ev_mode / TRACK 的 track_flag */

    servo_selfcheck_step_e selfcheck_step; /**< 自检流程步骤，仅 SERVO_MODE_SELFCHECK 下使用 */

    bool manual_brake;         /**< 手动刹车 GPIO 状态（PB14） */
    bool testing;              /**< 是否在测试模式 */
    uint8_t motion_state;      /**< 运动/静止判定结果（evmeancal）：1=静止, 0=运动 */

    /* —— 子节点电流环注入缓存（set_current 写，mode_current_handler 读） —— */
    servo_axis_id_e cur_axis; /**< 子节点驱动的轴 */
    float cur_give;           /**< 主节点下发的电流给定（A） */
    bool cur_enable;          /**< 主节点下发的使能 */

    /* ====================================================================
     *                          方法（函数指针）
     * ================================================================== */

    /** @brief 1ms 周期主调用：读传感器 → 翻译指令 → 跑控制环 → 写电机 */
    void (*update)(struct dev_servo *pobj);

    /** @brief 设置工作模式（下个周期生效） */
    void (*set_cmd)(struct dev_servo *pobj, const servo_cmd_t *cmd);

    /** @brief 仅切换模式 */
    void (*set_mode)(struct dev_servo *pobj, servo_mode_e mode);

    /** @brief 同时设置位置目标（用于 AFOLLOW） */
    void (*set_position)(struct dev_servo *pobj, float fw_deg, float gd_deg);

    /** @brief 同时设置速度目标（用于 VMOTION） */
    void (*set_velocity)(struct dev_servo *pobj, float fw_dps, float gd_dps);

    /** @brief 子节点电流环给定：外部下发 enable + I_give */
    void (*set_current)(struct dev_servo *pobj, servo_axis_id_e axis, float i_give, bool enable);

    /** @brief 设置脱靶量（用于 TRACK） */
    void (*set_miss)(struct dev_servo *pobj, float fw_miss, float fy_miss);

    /** @brief 设置引导目标（经/纬/高 + 是否引导标志，用于 GEO） */
    void (*set_geo)(struct dev_servo *pobj, float lon, float lat, float alt, uint8_t flag);

    /** @brief 紧急刹车 */
    void (*emergency_brake)(struct dev_servo *pobj);

    /** @brief 获取上报状态 */
    void (*get_state)(struct dev_servo *pobj, servo_state_t *out);

    /** @brief 单独访问轴对象（高级调用，调试/标定用） */
    dev_servo_axis_t *(*get_axis)(struct dev_servo *pobj, servo_axis_id_e id);

} dev_servo_t;

/* ========================================================================
 *                              公开 API
 * ====================================================================== */

/**
 * @brief 初始化伺服对象
 * @param pobj    待初始化的对象
 * @param fw_io   方位轴 IO 钩子
 * @param gd_io   俯仰轴 IO 钩子
 *
 * @note  两套 IO 钩子分别指向 FW / GD 的电机驱动器、编码器、陀螺。
 *        典型实现：把 SF.h 中的 userF*RxMotorDrvPar / FYBMQ / FWBMQ / fygyro / fwgyro
 *        全局封装成 getter 函数，再把指针塞进 io 结构。
 */
void dev_servo_init(dev_servo_t *pobj, const servo_axis_io_t *fw_io, const servo_axis_io_t *gd_io);

#endif /* _SERVO_CONTROLLER_H_ */
