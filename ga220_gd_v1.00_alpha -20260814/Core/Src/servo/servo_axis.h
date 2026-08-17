/**
 * @file    servo_axis.h
 * @brief   伺服单轴对象抽象    
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-05-29
 *
 * Copyright (c) 2026
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version  <th>Author     <th>Description
 * <tr><td>2026-05-29     <td>1.00     <td>LinHui     <td>由 SF.h/SF_20260303.c 重构而来
 * </table>
 */

#ifndef _SERVO_AXIS_H_
#define _SERVO_AXIS_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief FW（方位）轴是否为有刷直驱
 *        0 → 无刷伺服驱动器
 *        1 → 有刷电机控制
 */
#ifndef SERVO_FW_DRIVE_BRUSHED
#define SERVO_FW_DRIVE_BRUSHED 0
#endif

/**
 * @brief GD（俯仰）轴是否为有刷直驱
 *        0 → 无刷伺服驱动器
 *        1 → 有刷电机控制
 */
#ifndef SERVO_GD_DRIVE_BRUSHED
#define SERVO_GD_DRIVE_BRUSHED 1
#endif
/* ========================================================================
 *                              枚举定义
 * ====================================================================== */

/** @brief 轴 ID：用于区分 FW（方位）/ GD（俯仰） */
typedef enum {
    SERVO_AXIS_FW = 0, /**< 方位轴（航向 / azimuth） */
    SERVO_AXIS_GD = 1, /**< 俯仰轴（高低 / pitch）   */
    SERVO_AXIS_MAX
} servo_axis_id_e;

/** @brief 旋转方向（部分应用需要正反转切换） */
typedef enum {
    SERVO_AXIS_DIR_CW = 0, /**< 顺时针（默认正向） */
    SERVO_AXIS_DIR_CCW = 1 /**< 逆时针（反向）     */
} servo_axis_dir_e;

/**
 * @brief 电机驱动方式（区分"无刷电机驱动器"与"有刷电机"两种硬件配置）
 *
 *        根本差异，决定电流环在哪里闭合、执行器输出是电流给定还是 PWM 电压：
 *
 *        - CURRENT_CMD（无刷伺服驱动器）：
 *            MCU 只把电流给定 I_give 通过总线发给驱动器（set_motor_current_mA），
 *            电流环 PI 在驱动器内部闭合。控制链路最内层止于 I_give。
 
 *        - BRUSHED_PWM（有刷 H 桥）：
 *            再闭一层电流环：U_give = fw/gdiloop(Kp_I,Ki_I,SamT_I,Umax,I_give,I_fb)，
 *            然后把电压 U_give 通过 set_motor_pwm 钩子直接驱动 H 桥。电流反馈来自 ADC 采样。
 */
typedef enum {
    SERVO_DRIVE_CURRENT_CMD = 0, /**< 无刷电机驱动器：只发电流给定，驱动器内部闭电流环 */
    SERVO_DRIVE_BRUSHED_PWM = 1  /**< 有刷直驱：闭电流环(iloop)→U_give→PWM 输出 */
} servo_drive_type_e;

/**
 * @brief 控制环工作层级（决定从哪一级环路开始向下嵌套）
 *
 *        级联关系（外环→内环）:
 *        POSITION → EV_SPEED → V_SPEED → ACC → CURRENT → VOLTAGE
 *                                ↑         
 *                              TRACK     
 *                                ↑  
 *                               GEO        
 */

typedef enum {
    SERVO_LOOP_VOLTAGE = 0,  /**< 电压环（开环 PWM 测试用） */
    SERVO_LOOP_CURRENT = 1,  /**< 电流环 */
    SERVO_LOOP_ACC = 2,      /**< 角加速度环 */
    SERVO_LOOP_V_SPEED = 3,  /**< 速度环（陀螺反馈） */
    SERVO_LOOP_EV_SPEED = 4, /**< 惯性角速度环（编码器微分） */
    SERVO_LOOP_POSITION = 5, /**< 位置环 */
    SERVO_LOOP_TRACK = 6,    /**< 脱靶量跟踪环 */
    SERVO_LOOP_GEO = 7       /**< 地理跟踪环 */
} servo_loop_level_e;

/**
 * @brief 单轴故障码（16 位位域）。
 *        - 用 .bit.xxx 读写单个故障位；用 .all 整字读取/清零/上报。
 *        - .all 的 bit0..bit15 与下方字段声明顺序一致
 */
typedef union {
    uint16_t all;                  /**< 整字：0=无故障 */
    struct {
        uint16_t overcurrent  : 1; /**< bit0 过流：|I_fb| 长时间 ≥ 阈值 */
        uint16_t overspeed    : 1; /**< bit1 超速：|gyro| 或 |ev| 长时间 ≥ 阈值 */
        uint16_t limit_hit    : 1; /**< bit2 触发机械限位 */
        uint16_t encoder      : 1; /**< bit3 编码器异常（跳变/失通信） */
        uint16_t driver_comm  : 1; /**< bit4 驱动器通讯异常（预留） */
        uint16_t overtemp     : 1; /**< bit5 过温（预留） */
        uint16_t undervoltage : 1; /**< bit6 欠压（预留） */
        uint16_t overvoltage  : 1; /**< bit7 过压（预留） */
        uint16_t pos_error    : 1; /**< bit8 位置超差（预留） */
        uint16_t stall        : 1; /**< bit9 堵转（预留） */
        uint16_t reserved     : 6; /**< bit10..15 预留 */
    } bit;
} servo_axis_fault_t;

/* ========================================================================
 *                              结构体定义
 * ====================================================================== */

/**
 * @brief 轴的"静态配置"参数
 */
// clang-format off
typedef struct {
    /* 编码器（BMQ） */
    int32_t bmq_zero;              /**< 编码器机械零点原始值 */
    float bmq_coef_deg;            /**< 编码器一码对应角度（°/code） */
    uint32_t bmq_max_code;         /**< 编码器单圈最大码值（用于翻圈判断） */

    /* 陀螺仪 */
    float gyro_factor;             /**< 陀螺原始值→°/s 的换算系数 */
    bool has_gyro;                 /**< true=本轴装有陀螺仪；false=仅编码器，速度/加速度反馈走编码器微分 */

    /* 物理限位（°） */
    float angle_up_limit;          /**< 上限/右限 */
    float angle_dn_limit;          /**< 下限/左限 */
    bool is_continuous;            /**< true=无限转（FW 轴）, false=有限位（GD 轴） */
    bool soft_limit_enable;        /**< true=启用软限位检查(ReadXW)；默认 false，避免限位=home 时误报 */

    /* 保护阈值 */
    float overcur_thresh_a;        /**< 过流阈值（A） */
    float overspd_thresh_dps;      /**< 超速阈值（°/s） */
    uint16_t fault_window_ms;      /**< 故障判定时间窗（ms） */
    uint16_t fault_count_th;       /**< 故障窗口内允许触发次数 */

    /* 驱动方式（有刷直驱 vs 无刷伺服驱动器）—— 见 servo_drive_type_e */
    servo_drive_type_e drive_type; /**< 默认 CURRENT_CMD；有刷轴置 BRUSHED_PWM */
    float current_sign;            /**< 电流反馈符号：无刷伺服驱动器 +1；有刷 ADC 采样常为 -1 */
    bool current_use_butter;       /**< 有刷：ADC 原始电流是否过 FwCurrentButter 滤波 */
    uint16_t cur_calib_samples;    /**< 电流零偏标定样本数（CurrentInit 用，默认 3000） */
} servo_axis_config_t;


/**
 * @brief 单轴控制环参数（PI 系数 + 限幅）
 */
typedef struct {
    /* 电压/电流环 */
    float Kp_I, Ki_I;
    float SamT_I;           /**< 电流环采样周期(s) */
    float Umax;             /**< 电压输出限幅 */
    float Imax;             /**< 电流给定限幅(A) */

    /* 加速度环（FW 用增益表[6]） */
    float Kp_A[6], Ki_A[6];
    float SamT;             /**< 主控周期(s) */
    float Kg_A;             /**< 加速度环 Kg */
    float BW_A[6];          /**< 加速度环带宽 */
    float bound_A[5];       /**< 增益调度切换边界 */

    /* 速度环 */
    float Kp_V[6], Ki_V[6];
    float Amax;             /**< 加速度给定限幅 */
    float bound_V[5];

    /* 惯性速度环 */
    float Kp_Ev, Ki_Ev;
    float Evmax;
    float Ev_Set;           /**< 速度饱和门限 */

    /* 位置环 */
    float Kp_P, Ki_P;
    float bound_P;

    /* 脱靶量环 T1/T2/T3（对应可见光1/2、红外） */
    float Kp_1_T1, Ki_1_T1, bound_T1, Kp_2_T1, Ki_2_T1;
    float Kp_1_T2, Ki_1_T2, bound_T2, Kp_2_T2, Ki_2_T2;
    float Kp_1_T3, Ki_1_T3, bound_T3, Kp_2_T3, Ki_2_T3;
    float Vmax;             /**< 脱靶量环速度限幅 */

    /* GEO 环 */
    float Kp_G, Ki_G;
    float bound_G;
    float V_Set;            /**< GEO 环饱和速度 */
} servo_axis_ctrl_param_t;
// clang-format on

/**
 * @brief 单轴实时给定与反馈（"运行期变量"）
 *
 *        每级环路有一对 (xxx_give, xxx_fb)：
 *          - give: 上一级环计算后送入本级的给定
 *          - fb:   本级反馈量（来自传感器或解算）
 */
typedef struct {

    float U_give;
    float I_give, I_fb;     /* 电流 */
    float A_give, A_fb;     /* 加速度 */
    float V_give, V_fb;     /* 速度 */
    float Ev_give, Ev_fb;   /* 惯性速度 */
    float P_give, P_fb;     /* 位置 */
    float T_fb;             /* 脱靶量（T_fb 是当前脱靶量反馈，由跟踪器送入） */
    float GEO_give, GEO_fb; /* GEO */
    float p_error;          /**< 位置误差缓存（调试可视） */
} servo_axis_rt_t;

/**
 * @brief 单轴传感器层状态（编码器 + 陀螺 + 电流）
 */
typedef struct {
    /* 编码器 */
    uint32_t bmq_raw;     /**< 编码器原始值（含 BMQDATA_zero 偏置） */
    uint32_t bmq_raw_old;
    int32_t bmq_data;     /**< 去零点后的编码器值 */
    int32_t lap;          /**< 翻圈计数（仅 continuous 轴有效） */
    double angle_vel_raw; /**< 累计角度（用于求微分得到 ev） */

    /* 角度（°） */
    float angle;     /**< 机械角度（已去零点，主反馈） */
    float angle_360; /**< 机械角度（0~360°） */
    float Cos, Sec;  /**< cos(angle), 1/cos(angle)：用于 FW 解耦 */

    /* 陀螺速度（°/s） */
    float gyro_raw;  /**< 陀螺原始（去 bias 后） */
    float gyro_bias; /**< 静态零漂 */
    float gyro;      /**< 滤波后陀螺速度，主反馈 */
    float acc;       /**< 角加速度（°/s²） */

    /* 由编码器微分得来的角速度 / 加速度 */
    float ev; /**< encoder-velocity */
    float Ev_acc;

    /* 电流（A） */
    float current;      /**< 电机电流（已乘 current_sign、减去 current_bias） */
    float current_bias; /**< 电流零偏（CurrentInit 标定结果，有刷 ADC 用） */
} servo_axis_sensor_t;

/**
 * @brief 单轴保护/状态（运行时观测量）
 */
typedef struct {
    uint16_t fault_timer;     /**< 故障判定窗口计时 */
    uint16_t danger_count;    /**< 窗口内故障次数 */
    uint16_t overspeed_count; /**< 连续超速计数 */
    servo_axis_fault_t fault; /**< 16 位故障位域（.all 整字 / .bit.xxx 单位） */
    bool enabled;             /**< 电机是否使能 */
    bool brake_locked;        /**< 是否进入硬刹（不能再使能） */

    /* 限位标志（ReadXW 的等价物，仅有限位轴有效） */
    bool xw_up; /**< 触发上限/右限 */
    bool xw_dn; /**< 触发下限/左限 */

    /* 到位驻留（Dawei_PD 的等价物） */
    bool in_place;          /**< 位置已稳定到位 */
    uint16_t inplace_dwell; /**< |P_give-P_fb| 连续小于阈值的拍数 */
} servo_axis_status_t;

/**
 * @brief I/O 钩子：把"读电流""使能 GPIO"等硬件相关动作注入进来，
 *        使本模块不直接依赖 HAL/Driver——便于单元测试与多板移植。
 */
typedef struct {
    /** 读电机反馈电流 mA：返回 mA，由本模块除以 1000 */
    int32_t (*get_motor_current_mA)(void);
    /** 读编码器原始值 */
    uint32_t (*get_encoder_raw)(void);
    /** 读陀螺原始值 */
    int32_t (*get_gyro_raw)(void);
    /** 给电机驱动器写电流给定 mA（CURRENT_CMD 驱动用） */
    void (*set_motor_current_mA)(int32_t mA, bool enable);
    /** 电机硬使能（GPIO 控制 FW_DJ_EN/DIS） */
    void (*motor_hard_enable)(bool en);
    /**
     * @brief 有刷 H 桥 PWM 输出（BRUSHED_PWM 驱动用，等价 SF_20240116::FW_PWM）。
     *        实现里需：按 en 控制使能脚、按 u_give 符号设方向脚、|u_give| 限幅后写 PWM 占空比。
     *        CURRENT_CMD 驱动可置 NULL。
     */
    void (*set_motor_pwm)(float u_give, bool en);
} servo_axis_io_t;

/* 前置声明：用于函数指针中引用 */
struct dev_servo_axis;

/**
 * @brief 单轴对象总成
 *
 *        对外仅暴露这一个结构 + 下面的 init 函数。
 *        所有方法调用一律通过 `pobj->func(pobj, ...)` 完成，
 */
typedef struct dev_servo_axis {
    /* ---- 身份/配置 ---- */
    servo_axis_id_e id;
    servo_axis_config_t config; /**< 静态配置 */
    servo_axis_io_t io;         /**< 硬件钩子 */

    /* ---- 参数与状态 ---- */
    servo_axis_ctrl_param_t param; /**< PI 参数集 */
    servo_axis_rt_t rt;            /**< 给定/反馈 */
    servo_axis_sensor_t sensor;    /**< 传感器观测 */
    servo_axis_status_t status;    /**< 故障/使能/刹车 */

    /* ---- 运动学/几何 ---- */
    servo_axis_dir_e dir; /**< 方向 */
    float offset_deg;     /**< 零点偏移（用户调零） */
    float sec_couple;     /**< FW 受 GD 解耦因子（1/cos(GD)） */

    /* ====================================================================
     *                          方法（函数指针）
     * ================================================================== */

    /**
     * @brief 周期性刷新传感器（编码器/陀螺/电流），1ms 调用一次。
     *        内部会调用 io.get_* 钩子，再把原始值滤波得到 sensor.angle / gyro / acc / ev。
     */
    void (*update_sensor)(struct dev_servo_axis *pobj);

    /**
     * @brief 把 sensor 中的反馈量写到 rt 中（fbdispose 的等价物）。
     */
    void (*sync_feedback)(struct dev_servo_axis *pobj);

    /**
     * @brief 运行指定层级的级联控制环组合。
     */
    float (*run_loop)(struct dev_servo_axis *pobj, servo_loop_level_e level, uint8_t dz_flag);

    /** 清除指定层级及外层的所有 PI 内部寄存器（替代 clr_gd_x / clr_fw_x） */
    void (*clear_loops)(struct dev_servo_axis *pobj, servo_loop_level_e from_level);

    /** 清除所有层级的 PI 寄存器 */
    void (*clear_all)(struct dev_servo_axis *pobj);

    /** 电机使能 + 过流/超速保护检查（等价于 gd_en/fw_en） */
    void (*enable)(struct dev_servo_axis *pobj);

    /** 电机刹车（等价于 gd_brake/fw_brake） */
    void (*brake)(struct dev_servo_axis *pobj);

    /** 设置位置给定值（°） */
    void (*set_position_target)(struct dev_servo_axis *pobj, float deg);

    /** 设置速度给定值（°/s） */
    void (*set_velocity_target)(struct dev_servo_axis *pobj, float dps);

    /** 设置当前角度为新零点（参考 dev_mt6701 风格） */
    bool (*set_zero_angle)(struct dev_servo_axis *pobj, float angle_deg);

    /** 读当前机械角度（°） */
    float (*get_angle)(struct dev_servo_axis *pobj);

    /** 读当前角速度（°/s） */
    float (*get_velocity)(struct dev_servo_axis *pobj);

    /** 读故障码（16 位位域整字 servo_axis_fault_t.all） */
    uint16_t (*get_fault)(struct dev_servo_axis *pobj);

    /** 清故障码（解除自锁刹车） */
    void (*clear_fault)(struct dev_servo_axis *pobj);

    /**
     * @brief 电流零偏标定（CurrentInit 等价物）。
     */
    void (*calibrate_current_bias)(struct dev_servo_axis *pobj);

    /**
     * @brief 软限位检查（ReadXW 等价物）。
     */
    void (*check_soft_limit)(struct dev_servo_axis *pobj);

    /**
     * @brief 到位驻留判定（Dawei_PD 等价物）。
     */
    void (*update_in_place)(struct dev_servo_axis *pobj);

    /** 设置 FW 轴对 GD 轴的解耦因子（仅 FW 用，1/cos(gd_angle)） */
    void (*set_couple)(struct dev_servo_axis *pobj, float sec);

    /** 设置/读方向 */
    void (*set_dir)(struct dev_servo_axis *pobj, servo_axis_dir_e dir);
    servo_axis_dir_e (*get_dir)(struct dev_servo_axis *pobj);
} dev_servo_axis_t;

/* ========================================================================
 *                              公开 API
 * ====================================================================== */

/**
 * @brief 初始化单轴对象
 * @param pobj   待初始化的对象指针
 * @param id     轴 ID（SERVO_AXIS_FW / SERVO_AXIS_GD）
 * @param io     硬件钩子（必须提供，NULL 字段会被报错）
 */
void dev_servo_axis_init(dev_servo_axis_t *pobj, servo_axis_id_e id, const servo_axis_io_t *io);

/**
 * @brief 加载默认控制参数（来源：SF_20260303.c::initcontrolpar）
 *        本函数对 FW/GD 分别写入实测调好的 PI 系数与限幅。
 */
void dev_servo_axis_load_default_param(dev_servo_axis_t *pobj);

/* ========================================================================
 *                       调参（在线整定）接口
 * ====================================================================== */

/** @brief 可在线整定的 PI 参数 ID（覆盖各级环路） */
typedef enum {
    SERVO_TUNE_KP_I = 0,
    SERVO_TUNE_KI_I,  /* 电流环 */
    SERVO_TUNE_KP_A,
    SERVO_TUNE_KI_A,  /* 加速度环（带 idx，FW 0~5 / GD 0） */
    SERVO_TUNE_KP_V,
    SERVO_TUNE_KI_V,  /* 速度环  （带 idx，FW 0~5 / GD 0） */
    SERVO_TUNE_KP_EV,
    SERVO_TUNE_KI_EV, /* 惯性速度环 */
    SERVO_TUNE_KP_P,
    SERVO_TUNE_KI_P,  /* 位置环 */
    SERVO_TUNE_KP_G,
    SERVO_TUNE_KI_G,  /* GEO 环 */
    SERVO_TUNE_IMAX,
    SERVO_TUNE_AMAX,  /* 限幅 */
    SERVO_TUNE_PARAM_MAX
} servo_tune_param_e;

/**
 * @brief 在线写单个 PI 参数（运行/仿真时安全调用）
 * @param id   参数 ID（servo_tune_param_e）
 * @param idx  增益调度档位下标（仅 Kp_A/Ki_A/Kp_V/Ki_V 有效；其余传 0）
 * @param val  新值
 * @return true=写入成功；false=参数非法
 */
bool dev_servo_axis_set_param(dev_servo_axis_t *pobj, servo_tune_param_e id, uint8_t idx, float val);

/** @brief 在线读单个 PI 参数（读不到返回 0） */
float dev_servo_axis_get_param(dev_servo_axis_t *pobj, servo_tune_param_e id, uint8_t idx);

#endif /* _SERVO_AXIS_H_ */
