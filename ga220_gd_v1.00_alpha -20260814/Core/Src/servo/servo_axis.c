/**
 * @file    servo_axis.c
 * @brief   单轴对象实现
 *
 *          本文件只做控制算法（PI/陷波/滤波）的编排管理
 * 
 * @author  LinHui
 * @version 1.00
 * @date    2026-05-29
 */

#include "servo_axis.h"
#include "SFlibhead.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ========================================================================
 *                          内部辅助宏 / 常量
 * ====================================================================== */

#define SF_DEG2RAD          0.017453293f /**< 度转弧度 */
#define SF_OVERSPEED_THRESH 100.0f       /**< 超速阈值（°/s），与原代码 gd_en/fw_en 一致 */
#define SF_OVERCUR_THRESH   5.0f         /**< 过流阈值（A） */
#define SF_FAULT_WINDOW_MS  1000         /**< 故障窗口（ms） */
#define SF_FAULT_DANGER_TH  800          /**< 窗口内允许过流次数 */
#define SF_OVERSPEED_TH     1000         /**< 连续超速触发次数 */

#define SF_INPLACE_ERR_DEG   0.1f        /**< 到位误差阈值（°） */
#define SF_INPLACE_DWELL_TH  500         /**< 到位驻留拍数门限（>20 判到位） */
#define SF_CUR_CALIB_SAMPLES 3000        /**< 电流零偏标定默认样本数，与 CurrentInit 一致 */

#define SF_BMQ_COF_FW 0.0001716613769f   /**< FW 每1个码对应的角度 */
#define SF_BMQ_COF_GD 0.0001716613769f   /**< GD 每1个码对应的角度 */
// 21bit:0.0001716613769f; 20bit:0.0003433227539f

/* ========================================================================
 *                          前置静态方法声明
 * ====================================================================== */

static void axis_update_sensor(dev_servo_axis_t *pobj);
static void axis_sync_feedback(dev_servo_axis_t *pobj);
static float axis_run_loop(dev_servo_axis_t *pobj, servo_loop_level_e level, uint8_t dz_flag);
static void axis_clear_loops(dev_servo_axis_t *pobj, servo_loop_level_e from_level);
static void axis_clear_all(dev_servo_axis_t *pobj);
static void axis_enable(dev_servo_axis_t *pobj);
static void axis_brake(dev_servo_axis_t *pobj);
static void axis_set_pos_tgt(dev_servo_axis_t *pobj, float deg);
static void axis_set_vel_tgt(dev_servo_axis_t *pobj, float dps);
static bool axis_set_zero(dev_servo_axis_t *pobj, float angle_deg);
static float axis_get_angle(dev_servo_axis_t *pobj);
static float axis_get_vel(dev_servo_axis_t *pobj);
static uint16_t axis_get_fault(dev_servo_axis_t *pobj);
static void axis_clear_fault(dev_servo_axis_t *pobj);
static void axis_set_couple(dev_servo_axis_t *pobj, float sec);
static void axis_set_dir(dev_servo_axis_t *pobj, servo_axis_dir_e dir);
static servo_axis_dir_e axis_get_dir(dev_servo_axis_t *pobj);
static void axis_calibrate_current_bias(dev_servo_axis_t *pobj);
static void axis_check_soft_limit(dev_servo_axis_t *pobj);
static void axis_update_in_place(dev_servo_axis_t *pobj);

/* 内部工具函数 */
static void axis_load_default_config(dev_servo_axis_t *pobj);
static void axis_process_encoder(dev_servo_axis_t *pobj);
static void axis_process_gyro(dev_servo_axis_t *pobj);
static void axis_check_protect(dev_servo_axis_t *pobj);
static void axis_close_current_loop(dev_servo_axis_t *pobj);

/* ========================================================================
 *                          对外初始化 API
 * ====================================================================== */

/**
 * @brief 初始化单轴对象：装配函数指针 + 加载默认配置 + 清状态
 */
void dev_servo_axis_init(dev_servo_axis_t *pobj, servo_axis_id_e id, const servo_axis_io_t *io)
{
    if (pobj == NULL || io == NULL) {
        return;
    }

    /* 1) 全清，避免野值 */
    memset(pobj, 0, sizeof(dev_servo_axis_t));

    /* 2) 身份 / IO 钩子 */
    pobj->id = id;
    pobj->io = *io;
    pobj->dir = SERVO_AXIS_DIR_CCW;
    if (pobj->id == SERVO_AXIS_GD) {
        pobj->dir = SERVO_AXIS_DIR_CW;
    }
    pobj->sec_couple = 1.0f;

    /* 3) 装配方法表（函数指针） */
    pobj->update_sensor = axis_update_sensor;
    pobj->sync_feedback = axis_sync_feedback;
    pobj->run_loop = axis_run_loop;
    pobj->clear_loops = axis_clear_loops;
    pobj->clear_all = axis_clear_all;
    pobj->enable = axis_enable;
    pobj->brake = axis_brake;
    pobj->set_position_target = axis_set_pos_tgt;
    pobj->set_velocity_target = axis_set_vel_tgt;
    pobj->set_zero_angle = axis_set_zero;
    pobj->get_angle = axis_get_angle;
    pobj->get_velocity = axis_get_vel;
    pobj->get_fault = axis_get_fault;
    pobj->clear_fault = axis_clear_fault;
    pobj->set_couple = axis_set_couple;
    pobj->set_dir = axis_set_dir;
    pobj->get_dir = axis_get_dir;
    pobj->calibrate_current_bias = axis_calibrate_current_bias;
    pobj->check_soft_limit = axis_check_soft_limit;
    pobj->update_in_place = axis_update_in_place;

    /* 4) 默认 config / param */
    axis_load_default_config(pobj);
    dev_servo_axis_load_default_param(pobj);

    /* 5) 陀螺有无：未提供 get_gyro_raw 钩子即判为"仅编码器"。
     *    如需强制指定，可在 init 之后写 pobj->config.has_gyro = true/false; 覆盖。 */
    pobj->config.has_gyro = (io->get_gyro_raw != NULL);
}

/**
 * @brief 加载默认 PI 控制参数（来源：SF_20260303.c::initcontrolpar）
 *        FW / GD 系数不同，按 id 分流。
 */
void dev_servo_axis_load_default_param(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;
    servo_axis_ctrl_param_t *p = &pobj->param;

    p->SamT_I = 0.001f;
    p->SamT = 0.001f;

    if (pobj->id == SERVO_AXIS_GD) {
        /* ---------- 俯仰（GD）----------- */

        /* 电流环参数 */
        p->Kp_I = 750.0f;
        p->Ki_I = 300000.0f;
        p->Umax = 4000.0f;
        p->Imax = 4.0f;  // 电流环最大输出电流

        /* 陀螺加速度环参数 */
        p->Kp_A[0] = 0.0f;
        p->Ki_A[0] = 0.0f;
        p->Kg_A = 321.69f;   // 加速度传递函数增益
        p->BW_A[0] = 10.0f;  // 补偿带宽

        /* 陀螺速度环参数 */
        p->Kp_V[0] = 0.0f;
        p->Ki_V[0] = 0.0f;
        p->Amax = 500.0f;  // 陀螺速度环控制器输出的最大值（给内环加速度环使用）

        /* 轴角速度参数 */
        p->Kp_Ev = 0.38f;
        p->Ki_Ev = 6.5f;

        /* 轴位置参数 */
        p->Kp_P = 8.0f;
        p->Ki_P = 0.0f;
        p->bound_P = 6.0f;  // bound 是脱靶量误差从“固定逼近速度”切换到“闭环控制”的角度阈值
        p->Ev_Set = 35.0f;  // 当位置误差大于 bound_P 时，采用固定逼近速度
        p->Evmax = 40.0f;   // gdploop 位置环控制器输出的最大速度值（作为内环轴角速度环的期望）

        /* 脱靶量环 1 (TV1) */
        p->Kp_1_T1 = 1.5f;
        p->Ki_1_T1 = 0.0f;
        p->bound_T1 = 3.0f;  // bound 按误差大小做增益调度的分段 PI 跟踪环。
        p->Kp_2_T1 = 1.5f;
        p->Ki_2_T1 = 0.0f;
        p->Vmax = 35.0f;      // 脱靶量最大允许跟踪速度

        /* TV2 */
        p->Kp_1_T2 = 0.0f;
        p->Ki_1_T2 = 0.0f;
        p->bound_T2 = 0.0f;
        p->Kp_2_T2 = 0.0f;
        p->Ki_2_T2 = 0.0f;

#if SERVO_GD_DRIVE_BRUSHED
        /* 无刷伺服驱动器把电流环放在驱动器内部，故上面的 Kp_I/Ki_I/Umax 实际未使用；
         * 有刷电机由需要跑 gdiloop，电流环系数单独整定。 */
        p->Kp_I = 750.0f;
        p->Ki_I = 300000.0f;
        p->SamT_I = 0.001f;
        p->Umax = 4000.0f;  // 有刷电机的PWM占空比输出最大值
#endif

    } else {
        /* ---------- 方位（FW）----------- */
        p->Kp_I = 700.0f;
        p->Ki_I = 200000.0f;
        p->Umax = 4000.0f;
        p->Imax = 4.5f;

        /* 陀螺加速度环参数 */
        /* FW 有 6 档增益调度（按 |GD 角度| 切换） */
        const float kA_base = 0.00042f * 1.60f * 0.8f * 0.38f;
        const float scl_A[6] = {0.30f, 0.50f, 0.50f, 0.70f, 1.0f, 1.0f};
        for (int i = 0; i < 6; i++) {
            p->Kp_A[i] = kA_base * scl_A[i];
            p->Ki_A[i] = 1.60f * scl_A[i] * 0.8f * 0.38f;
            p->BW_A[i] = 10.0f;
        }
        p->Kg_A = 254.37f;  // 加速度传递函数增益
        p->bound_A[0] = 75.0f;
        p->bound_A[1] = 70.0f;
        p->bound_A[2] = 60.0f;
        p->bound_A[3] = 40.0f;
        p->bound_A[4] = 30.0f;

        /* 陀螺速度环参数 */
        const float scl_V[6] = {1.0f, 1.0f, 1.2f, 1.4f, 1.6f, 1.6f};
        const float scl_KV[6] = {0.30f, 0.50f, 0.70f, 0.70f, 1.0f, 1.0f};
        for (int i = 0; i < 6; i++) {
            p->Kp_V[i] = 50.0f * 0.80f * scl_V[i];
            p->Ki_V[i] = 1100.0f * 0.80f * scl_KV[i];
        }
        p->Amax = 1500.0f;  // 陀螺速度环控制器输出的最大值（给内环加速度环使用）
        p->bound_V[0] = 75.0f;
        p->bound_V[1] = 70.0f;
        p->bound_V[2] = 60.0f;
        p->bound_V[3] = 40.0f;
        p->bound_V[4] = 30.0f;

        /* 轴角速度参数 */
        p->Kp_Ev = 0.5035f;  // 0.5035f
        p->Ki_Ev = 9.5f;     // 9.5f

        /* 轴位置参数 */
        p->Kp_P = 15.0f;
        p->Ki_P = 0.0f;
        p->bound_P = 6.0f;  // bound 是位置误差从“固定逼近速度”切换到“闭环控制”的角度阈值
        p->Ev_Set = 35.0f;  // 当位置误差大于 bound_P 时，采用固定逼近速度
        p->Evmax = 40.0f;   // fwploop 位置环控制器输出的最大速度值（作为内环轴角速度环的期望）

        /* 脱靶量环 1 (TV1) */
        p->Kp_1_T1 = 2.0f;
        p->Ki_1_T1 = 0.0f;
        p->bound_T1 = 3.0f;  // bound 按误差大小做增益调度的分段 PI 跟踪环。
        p->Kp_2_T1 = 2.0f;
        p->Ki_2_T1 = 0.0f;
        p->Vmax = 35.0f;      // 脱靶量最大允许跟踪速度

        /* TV2 */
        p->Kp_1_T2 = 0.0f;
        p->Ki_1_T2 = 0.0f;
        p->bound_T2 = 0.0f;
        p->Kp_2_T2 = 0.0f;
        p->Ki_2_T2 = 0.0f;

#if SERVO_FW_DRIVE_BRUSHED
        /* 无刷伺服驱动器把电流环放在驱动器内部（通过电机参数整定的 PI），故上面的 Kp_I/Ki_I/Umax 实际未使用；
         * 有刷电机需要跑电流闭环 fwiloop，参数需要单独整定。 */
        p->Kp_I = 700.0f;
        p->Ki_I = 200000.0f;
        p->SamT_I = 0.001f;
        p->Umax = 4000.0f;  // 有刷电机的PWM占空比输出最大值
#endif
    }
}

/* ========================================================================
 *                       在线整定（调参）接口实现
 * ====================================================================== */

bool dev_servo_axis_set_param(dev_servo_axis_t *pobj, servo_tune_param_e id, uint8_t idx, float val)
{
    if (pobj == NULL || id >= SERVO_TUNE_PARAM_MAX) {
        return false;
    }
    servo_axis_ctrl_param_t *p = &pobj->param;

    /* 带增益调度的环路用 idx，越界保护：GD 只有 [0]，FW 0~5 */
    if (idx >= 6u) {
        idx = 5u;
    }

    switch (id) {
        case SERVO_TUNE_KP_I: p->Kp_I = val; break;
        case SERVO_TUNE_KI_I: p->Ki_I = val; break;
        case SERVO_TUNE_KP_A: p->Kp_A[idx] = val; break;
        case SERVO_TUNE_KI_A: p->Ki_A[idx] = val; break;
        case SERVO_TUNE_KP_V: p->Kp_V[idx] = val; break;
        case SERVO_TUNE_KI_V: p->Ki_V[idx] = val; break;
        case SERVO_TUNE_KP_EV: p->Kp_Ev = val; break;
        case SERVO_TUNE_KI_EV: p->Ki_Ev = val; break;
        case SERVO_TUNE_KP_P: p->Kp_P = val; break;
        case SERVO_TUNE_KI_P: p->Ki_P = val; break;
        case SERVO_TUNE_KP_G: p->Kp_G = val; break;
        case SERVO_TUNE_KI_G: p->Ki_G = val; break;
        case SERVO_TUNE_IMAX: p->Imax = val; break;
        case SERVO_TUNE_AMAX: p->Amax = val; break;
        default: return false;
    }
    return true;
}

float dev_servo_axis_get_param(dev_servo_axis_t *pobj, servo_tune_param_e id, uint8_t idx)
{
    if (pobj == NULL || id >= SERVO_TUNE_PARAM_MAX) {
        return 0.0f;
    }
    servo_axis_ctrl_param_t *p = &pobj->param;
    if (idx >= 6u) {
        idx = 5u;
    }

    switch (id) {
        case SERVO_TUNE_KP_I: return p->Kp_I;
        case SERVO_TUNE_KI_I: return p->Ki_I;
        case SERVO_TUNE_KP_A: return p->Kp_A[idx];
        case SERVO_TUNE_KI_A: return p->Ki_A[idx];
        case SERVO_TUNE_KP_V: return p->Kp_V[idx];
        case SERVO_TUNE_KI_V: return p->Ki_V[idx];
        case SERVO_TUNE_KP_EV: return p->Kp_Ev;
        case SERVO_TUNE_KI_EV: return p->Ki_Ev;
        case SERVO_TUNE_KP_P: return p->Kp_P;
        case SERVO_TUNE_KI_P: return p->Ki_P;
        case SERVO_TUNE_KP_G: return p->Kp_G;
        case SERVO_TUNE_KI_G: return p->Ki_G;
        case SERVO_TUNE_IMAX: return p->Imax;
        case SERVO_TUNE_AMAX: return p->Amax;
        default: return 0.0f;
    }
}

/* ========================================================================
 *                          内部工具：加载默认硬件配置
 * ====================================================================== */

static void axis_load_default_config(dev_servo_axis_t *pobj)
{
    servo_axis_config_t *c = &pobj->config;

    /* 手持：20位 1048576 ；GA220：21位 2097152 */
    c->bmq_max_code = 2097152u; /* 20 位绝对值编码器 的单圈分辨率 */
    c->fault_window_ms = SF_FAULT_WINDOW_MS;
    c->fault_count_th = SF_FAULT_DANGER_TH;
    c->overcur_thresh_a = SF_OVERCUR_THRESH;
    c->overspd_thresh_dps = SF_OVERSPEED_THRESH;

    c->has_gyro = true; /* 设置陀螺的有无，默认存在；但 dev_servo_axis_init 会按 IO 钩子自动改写 */

    /* —— 驱动方式选择：有刷电机 / 无刷伺服驱动 */
    c->drive_type = SERVO_DRIVE_CURRENT_CMD;
    c->current_sign = 1.0f;
    c->current_use_butter = false;
    c->cur_calib_samples = SF_CUR_CALIB_SAMPLES;

    if (pobj->id == SERVO_AXIS_GD) {
        c->bmq_zero = 1036242;                         /* 零偏 188256 */
        c->bmq_coef_deg = SF_BMQ_COF_GD;               /* Bmq_cof_gd，每1个码对应的角度 */
        c->gyro_factor = 0.0000292035869364061814765f; /* Gyro_FYFactor */
        c->angle_up_limit = 82.0f;                     /* 俯仰轴角度限位 */
        c->angle_dn_limit = -2.0f;
        c->is_continuous = false; /* 表示它是有限角度轴，有机械上下限，默认目标范围是 0° ~ 75° */

                                  /* —— GD 有刷电机配置 ——
         * 仅当 SERVO_GD_DRIVE_BRUSHED=1 时生效；默认 0 → 维持无刷伺服驱动器。
         *   → 电流符号 -1、过 250Hz 二阶 Butterworth。
         * 闭电流环走 gdiloop，U_give 经 FY_PWM(TIMx->CCR1)输出。 */
#if SERVO_GD_DRIVE_BRUSHED
        c->drive_type = SERVO_DRIVE_BRUSHED_PWM;
        c->current_sign = 1.0f;
        c->current_use_butter = true;
#endif
    } else {
        c->bmq_zero = 715299;                         /* 零偏 */
        c->bmq_coef_deg = SF_BMQ_COF_FW;              /* Bmq_cof_fw，每1个码对应的角度 */
        c->gyro_factor = 0.000029119908707921404318f; /* Gyro_FWFactor */
        c->angle_up_limit = 0.0f;
        c->angle_dn_limit = 0.0f;
        c->is_continuous = true; /* 表示它按连续旋转轴处理，可以跨过编码器 0 点，不用普通上下限裁剪。 */

                                 /* —— 有刷电机配置 ——
         * 仅当编译开关 SERVO_FW_DRIVE_BRUSHED=1 时生效；
         * 对应原代码：
         *   FWControl.I_fb = -1 * FwCurrentButter(2,250,0.001,FW_Current);   → 符号 -1 + 滤波
         *   FWControl.U_give = fwiloop(...);  FW_PWM(FWControl.U_give);      → 闭电流环 + PWM */
#if SERVO_FW_DRIVE_BRUSHED
        c->drive_type = SERVO_DRIVE_BRUSHED_PWM;
        c->current_sign = -1.0f;
        c->current_use_butter = true;
#endif
    }
}

/* ========================================================================
 *                          传感器更新
 * ====================================================================== */

/**
 * @brief 编码器处理（替代 gdbmqdispose / fwbmqdispose）
 */
static void axis_process_encoder(dev_servo_axis_t *pobj)
{
    servo_axis_sensor_t *s = &pobj->sensor;
    servo_axis_config_t *c = &pobj->config;

    const int32_t full = (int32_t)c->bmq_max_code;
    const int32_t half = full / 2;

    /* 方向符号：CW(默认)=+1 顺时针角度增大；CCW=-1 反向 */
    const int32_t sgn = (pobj->dir == SERVO_AXIS_DIR_CCW) ? -1 : +1;

    /* 1. 取原始 */
    s->bmq_raw_old = s->bmq_raw;
    if (pobj->io.get_encoder_raw) s->bmq_raw = pobj->io.get_encoder_raw();

    /* 2. 去零点 + 方向 → 本拍单圈位置折算到 [-half, half) */
    int32_t pos = sgn * ((int32_t)s->bmq_raw - c->bmq_zero);
    pos %= full;
    if (pos < 0) pos += full;
    if (pos >= half) pos -= full;

    /* 上一拍单圈位置（同样带方向） */
    int32_t pos_old = sgn * ((int32_t)s->bmq_raw_old - c->bmq_zero);
    pos_old %= full;
    if (pos_old < 0) pos_old += full;
    if (pos_old >= half) pos_old -= full;

    s->bmq_data = pos;

    /* 3. 机械角 */
    s->angle = c->bmq_coef_deg * (float)s->bmq_data;
    /*  机械角：0~360  */
    s->angle_360 = (s->angle < 0.0f) ? (s->angle + 360.0f) : s->angle;

    /* 4+5. 本拍真实增量 + 翻圈：对位置差做 ±half 回绕
            （前提：单拍位移 < 半圈，1ms@任何现实转速都成立） */
    int32_t delta_raw = pos - pos_old; /* (-full, full) */
    if (delta_raw >= half) {
        delta_raw -= full;
        s->lap--;
    } /* 反向越过 -180 → 减一圈 */
    else if (delta_raw < -half) {
        delta_raw += full;
        s->lap++;
    } /* 正向越过 +180 → 加一圈 */

    /* 6. cos / sec —— FW 受 GD 解耦时用 */
    s->Cos = cosf(s->angle * SF_DEG2RAD);
    if (fabsf(s->Cos) > 1e-6f) s->Sec = 1.0f / s->Cos;

    /* 7. 连续累计角 → 微分滤波得到无跳变速度 */
    s->angle_vel_raw = ((double)s->lap * (double)full + (double)s->bmq_data) * (double)c->bmq_coef_deg;
    if (pobj->id == SERVO_AXIS_GD) {
//        s->ev = GdAxisEvDis(2, 30, 0.001f, (float)s->angle_vel_raw);
        s->Ev_acc = GdeAccDis(70.0f, 0.001f, s->ev);
    } else {
//        s->ev = (float)FwAxisEvDis(2, 30, 0.001f, s->angle_vel_raw);
    }
}

/**
 * @brief 速度/加速度/电流反馈处理（替代 gyrodispose 的轴对应部分）
 *
 *        有陀螺：速度=陀螺滤波、加速度=陀螺微分；
 *        无陀螺：速度=编码器微分 ev、加速度=编码器微分 Ev_acc（GD）。
 */
static void axis_process_gyro(dev_servo_axis_t *pobj)
{
    servo_axis_sensor_t *s = &pobj->sensor;
    servo_axis_config_t *c = &pobj->config;

    /* 电流（A）—— 钩子返回 mA，电流采集与陀螺有无无关，先统一处理。
     *
     * 无刷伺服驱动器（CURRENT_CMD）：驱动器回传的电流已是真实值，sign=+1、不滤波、bias=0，
     *     等价 fbdispose() 里 GDControl.I_fb = userFYRxMotorDrvPar.Current*0.001。
     * 有刷电机（BRUSHED_PWM）：ADC 采样需按需 current_sign 取符号 + Butterworth 滤波 + 减零偏，
     *     等价 SF_20240116::fbdispose() 里 FWControl.I_fb = -1*FwCurrentButter(2,250,0.001,FW_Current)。 */
    if (pobj->io.get_motor_current_mA) {
        float raw = (float)pobj->io.get_motor_current_mA() * 0.001f;
        if (c->drive_type == SERVO_DRIVE_BRUSHED_PWM && c->current_use_butter) {

            /* 2 阶 250Hz；FW/GD 各用独立的滤波器 */
            raw = (pobj->id == SERVO_AXIS_GD) ? GdCurrentButter(2, 400, pobj->param.SamT_I, raw)   // 250Hz
                                              : FwCurrentButter(2, 400, pobj->param.SamT_I, raw);  // 250Hz
        }
        s->current = c->current_sign * raw - s->current_bias;
    }

    /* —— 无陀螺：速度/加速度反馈全部来自编码器微分，跳过陀螺滤波 —— */
    if (!c->has_gyro) {
        s->gyro_raw = 0.0f;
        s->gyro = s->ev; /* 速度反馈 = 编码器微分速度 */

        /* FW 无编码器加速度，无陀螺时不走加速度环 */
        s->acc = (pobj->id == SERVO_AXIS_GD) ? s->Ev_acc : 0.0f;
        return;
    }

    /* —— 有陀螺：原有处理 —— */
    int32_t raw = 0;
    if (pobj->io.get_gyro_raw) {
        raw = pobj->io.get_gyro_raw();
    }

    /* 与原代码符号约定保持一致：取负 + 去 bias */
    s->gyro_raw = -1.0f * ((float)raw * c->gyro_factor - s->gyro_bias);

    if (pobj->id == SERVO_AXIS_GD) {
        s->gyro = GdGyroButter(1, 70, 0.001f, s->gyro_raw);
        if (s->gyro > 300.0f) s->gyro = 300.0f;
        if (s->gyro < -300.0f) s->gyro = -300.0f;
        s->acc = GdAccDis(100, 0.001f, s->gyro_raw);
    } else {
        s->gyro = FwGyroButter(1, 70, 0.001f, s->gyro_raw, pobj->sec_couple);
        if (s->gyro > 500.0f) s->gyro = 500.0f;
        if (s->gyro < -500.0f) s->gyro = -500.0f;
        s->acc = FwAccDis(100, 0.001f, s->gyro_raw, pobj->sec_couple);
    }
    if (s->acc > 3000.0f) s->acc = 3000.0f;
    if (s->acc < -3000.0f) s->acc = -3000.0f;
}

/**
 * @brief 周期性传感器刷新（外部 1ms 调用）
 */
static void axis_update_sensor(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;
//    axis_process_encoder(pobj);
    axis_process_gyro(pobj);
    axis_check_soft_limit(pobj); /* 角度更新后顺带刷新软限位标志（ReadXW 等价物） */
}

/**
 * @brief 把传感器观测同步到控制环反馈字段（fbdispose 等价物）
 *        无陀螺时速度/加速度反馈取编码器微分量。
 */
static void axis_sync_feedback(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;
    pobj->rt.I_fb = pobj->sensor.current;
    pobj->rt.P_fb = pobj->sensor.angle;
    pobj->rt.Ev_fb = pobj->sensor.ev;

    if (pobj->config.has_gyro) {
        pobj->rt.V_fb = pobj->sensor.gyro;
        pobj->rt.A_fb = pobj->sensor.acc;
    } else {
        pobj->rt.V_fb = pobj->sensor.ev; /* 编码器微分速度 */

        /* 无陀螺时，FW 不走加速度环 */
        pobj->rt.A_fb = (pobj->id == SERVO_AXIS_GD) ? pobj->sensor.Ev_acc : 0.0f;
    }
}
/* 开环 PWM 测试给定：仅在 run_loop(level=SERVO_LOOP_VOLTAGE 电压开环) 下生效。
 * 符号=方向，幅值自动限到 ±Umax(默认4000)。调试器 Watch 里直接改 g_olp_u_give[轴]。
 *   [0]=方位FW   [1]=俯仰GD */
volatile float g_olp_u_give[SERVO_AXIS_MAX] = {0};

/**
 * @brief 开环 PWM：把调试给定限幅后作为 U_give，旁路全部控制环。
 *        下游 axis_enable() 的 set_motor_pwm(U_give,true) 据此输出占空比+方向。
 */
static inline void axis_voltage_open_loop(dev_servo_axis_t *pobj)
{
    float u = g_olp_u_give[pobj->id];
    const float umax = pobj->param.Umax; /* 限幅，避免超过 PWM 周期 ±4200 */
    if (u > umax) u = umax;
    if (u < -umax) u = -umax;
    pobj->rt.U_give = u;
    pobj->rt.I_give = 0.0f; /* 开环不走电流环，给定清零便于观测 */
}

/* ========================================================================
 *                          级联控制环编排
 * ====================================================================== */

/**
 * @brief 有刷电机的内层电流环：I_give → U_give
 *        有刷电机则再闭一层电流环 PI（xxiloop），输出电压给定 U_give。
 */
static void axis_close_current_loop(dev_servo_axis_t *pobj)
{
    servo_axis_ctrl_param_t *p = &pobj->param;
    servo_axis_rt_t *r = &pobj->rt;
    if (pobj->id == SERVO_AXIS_GD) {
        r->U_give = gdiloop(p->Kp_I, p->Ki_I, p->SamT_I, p->Umax, r->I_give, r->I_fb);
    } else {
        r->U_give = fwiloop(p->Kp_I, p->Ki_I, p->SamT_I, p->Umax, r->I_give, r->I_fb);
    }
}

static float axis_run_loop(dev_servo_axis_t *pobj, servo_loop_level_e level, uint8_t dz_flag)
{
    if (pobj == NULL) return 0.0f;

    servo_axis_ctrl_param_t *p = &pobj->param;
    servo_axis_rt_t *r = &pobj->rt;
    const bool use_gyro = pobj->config.has_gyro;

    const float gd_abs_angle = (pobj->id == SERVO_AXIS_FW) ? fabsf(pobj->sensor.angle) : pobj->sensor.angle;

    if (pobj->id == SERVO_AXIS_GD) {
        /* ===================== GD 轴级联 ===================== */
        switch (level) {
            case SERVO_LOOP_GEO:
                r->V_give =
                    gdgeo(p->Kp_G, p->Ki_G, p->SamT, p->bound_G, p->V_Set, p->Vmax, r->GEO_give, r->GEO_fb);

            case SERVO_LOOP_TRACK:
                if (level == SERVO_LOOP_TRACK) {
                    r->V_give = gdtloop_TV1(p->Kp_1_T1, p->Ki_1_T1, p->bound_T1, p->Kp_2_T1, p->Ki_2_T1,
                                            p->SamT, p->Vmax, r->T_fb);
                }
            case SERVO_LOOP_V_SPEED:
                if (use_gyro) {
                    /* 有陀螺：陀螺速度环 → 加速度环 → 电流 */
                    r->A_give = gdvloop(p->Kp_V[0], p->Ki_V[0], p->SamT, p->Amax, pobj->config.angle_up_limit,
                                        pobj->config.angle_dn_limit, r->V_give, r->V_fb, pobj->sensor.angle);
                    r->I_give = gdaloop(p->Kp_A[0], p->Ki_A[0], p->SamT, p->Kg_A, p->BW_A[0], p->Imax,
                                        r->A_give, r->A_fb);
                } else {
                    /* 无陀螺：速度指令 → 编码器惯性速度环 → 电流（跳过加速度环） */
                    r->Ev_give = r->V_give;
                    r->I_give = gdevloop(p->Kp_Ev, p->Ki_Ev, p->SamT, p->Imax, pobj->config.angle_up_limit,
                                         pobj->config.angle_dn_limit, r->Ev_give, r->Ev_fb,
                                         pobj->sensor.angle, dz_flag);  // true
                }
                break;

            case SERVO_LOOP_ACC:
                r->I_give = gdaloop(p->Kp_A[0], p->Ki_A[0], p->SamT, p->Kg_A, p->BW_A[0], p->Imax, r->A_give,
                                    r->A_fb);
                break;

            case SERVO_LOOP_POSITION:
                r->Ev_give = gdploop(p->Kp_P, p->Ki_P, p->SamT, pobj->config.angle_up_limit,
                                     pobj->config.angle_dn_limit, p->bound_P, p->Ev_Set, p->Evmax, r->P_give,
                                     r->P_fb, dz_flag);
            case SERVO_LOOP_EV_SPEED:
                r->I_give =
                    gdevloop(p->Kp_Ev, p->Ki_Ev, p->SamT, p->Imax, pobj->config.angle_up_limit,
                             pobj->config.angle_dn_limit, r->Ev_give, r->Ev_fb, pobj->sensor.angle, dz_flag);
                break;

            case SERVO_LOOP_CURRENT:
                /* 由外部直接给 I_give，本函数透传 */
                break;
            case SERVO_LOOP_VOLTAGE:
                axis_voltage_open_loop(pobj); /* 开环 PWM 测试 */
                break;
        }
        r->p_error = r->P_give - r->P_fb;
    } else {
        /* ===================== FW 轴级联 ===================== */
        switch (level) {
            case SERVO_LOOP_GEO:
                r->V_give = fwgeo(p->Kp_G, p->Ki_G, p->SamT, p->bound_G, p->V_Set, p->Vmax, r->GEO_give,
                                  r->GEO_fb, pobj->sec_couple);
            case SERVO_LOOP_TRACK:
                if (level == SERVO_LOOP_TRACK) {
                    r->V_give = fwtloop_TV1(p->Kp_1_T1, p->Ki_1_T1, p->bound_T1, p->Kp_2_T1, p->Ki_2_T1,
                                            p->SamT, p->Vmax, r->T_fb, pobj->sec_couple);
                }
            case SERVO_LOOP_V_SPEED:
                if (use_gyro) {
                    /* 有陀螺：陀螺速度环 → 加速度环 → 电流 */
                    r->A_give = fwvloop(p->Kp_V, p->Ki_V, p->SamT, p->Amax, pobj->config.angle_dn_limit,
                                        pobj->config.angle_up_limit, r->V_give, r->V_fb, pobj->sensor.angle,
                                        gd_abs_angle, p->bound_V);
                    r->I_give = fwaloop(p->Kp_A, p->Ki_A, p->SamT, p->Kg_A, p->BW_A, p->Imax, r->A_give,
                                        r->A_fb, gd_abs_angle, p->bound_A);
                } else {
                    /* 无陀螺：速度指令 → 编码器惯性速度环 → 电流（跳过加速度环） */
                    r->Ev_give = r->V_give;
                    r->I_give = fwevloop(p->Kp_Ev, p->Ki_Ev, p->SamT, p->Imax, pobj->config.angle_dn_limit,
                                         pobj->config.angle_up_limit, r->Ev_give, r->Ev_fb,
                                         pobj->sensor.angle, dz_flag);
                }
                break;

            case SERVO_LOOP_ACC:
                r->I_give = fwaloop(p->Kp_A, p->Ki_A, p->SamT, p->Kg_A, p->BW_A, p->Imax, r->A_give, r->A_fb,
                                    gd_abs_angle, p->bound_A);
                break;

            case SERVO_LOOP_POSITION:
                r->Ev_give = fwploop(p->Kp_P, p->Ki_P, p->SamT, pobj->config.angle_dn_limit,
                                     pobj->config.angle_up_limit, p->bound_P, p->Ev_Set, p->Evmax, r->P_give,
                                     r->P_fb, dz_flag);
            case SERVO_LOOP_EV_SPEED:
                r->I_give =
                    fwevloop(p->Kp_Ev, p->Ki_Ev, p->SamT, p->Imax, pobj->config.angle_dn_limit,
                             pobj->config.angle_up_limit, r->Ev_give, r->Ev_fb, pobj->sensor.angle, dz_flag);
                break;

            case SERVO_LOOP_CURRENT: break;
            case SERVO_LOOP_VOLTAGE:
                axis_voltage_open_loop(pobj); /* 开环 PWM 测试 */
                break;
        }
        r->p_error = r->P_give - r->P_fb;
    }

    /* —— 有刷电机：在最内层电流给定 I_give 之上再闭一层电流环得到 U_give ，然后输出 PWM 给 H 桥驱动器 —— */
    if (pobj->config.drive_type == SERVO_DRIVE_BRUSHED_PWM && level != SERVO_LOOP_VOLTAGE) {
        axis_close_current_loop(pobj);
    }

    return r->I_give;
}

/* ========================================================================
 *                          PI 寄存器清除
 * ====================================================================== */

static void axis_clear_loops(dev_servo_axis_t *pobj, servo_loop_level_e from_level)
{
    if (pobj == NULL) return;

    /* 调用 Servolib.lib 中预定义的清除函数。
     * 注意 clr_xxx_all 会一次清完，按需粒度可拆分调用。 */
    if (pobj->id == SERVO_AXIS_GD) {
        if (from_level >= SERVO_LOOP_GEO) clr_gd_g();
        if (from_level >= SERVO_LOOP_TRACK) {
            clr_gd_t_1();
            clr_gd_t_2();
            clr_gd_t_3();
        }
        if (from_level >= SERVO_LOOP_POSITION) clr_gd_p();
        if (from_level >= SERVO_LOOP_EV_SPEED) clr_gd_ev();
        if (from_level >= SERVO_LOOP_V_SPEED) clr_gd_v();
        if (from_level >= SERVO_LOOP_ACC) clr_gd_a();
        if (from_level >= SERVO_LOOP_CURRENT) clr_gd_i();
        clr_gddg();
    } else {
        if (from_level >= SERVO_LOOP_GEO) clr_fw_g();
        if (from_level >= SERVO_LOOP_TRACK) {
            clr_fw_t_1();
            clr_fw_t_2();
            clr_fw_t_3();
        }
        if (from_level >= SERVO_LOOP_POSITION) clr_fw_p();
        if (from_level >= SERVO_LOOP_EV_SPEED) clr_fw_ev();
        if (from_level >= SERVO_LOOP_V_SPEED) clr_fw_v();
        if (from_level >= SERVO_LOOP_ACC) clr_fw_a();
        if (from_level >= SERVO_LOOP_CURRENT) clr_fw_i();
        clr_fwdg();
    }
}

static void axis_clear_all(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;
    if (pobj->id == SERVO_AXIS_GD)
        clr_gd_all();
    else
        clr_fw_all();
}

/* ========================================================================
 *                          使能 / 刹车 / 保护
 * ====================================================================== */

/**
 * @brief 过流/超速保护检查（替代 gd_en/fw_en 中的故障判定段）
 */
static void axis_check_protect(dev_servo_axis_t *pobj)
{
    servo_axis_status_t *st = &pobj->status;
    servo_axis_config_t *cf = &pobj->config;

    /* 1) 过流计数 */
    if (st->fault_timer <= cf->fault_window_ms) {
        st->fault_timer++;
        if (fabsf(pobj->rt.I_fb) >= cf->overcur_thresh_a) {
            st->danger_count++;
        }
        if (st->fault_timer == cf->fault_window_ms) {
            if (st->danger_count >= cf->fault_count_th) {
                st->fault.bit.overcurrent = 1;
                st->brake_locked = true;
            }
            st->fault_timer = 0;
            st->danger_count = 0;
        }
    }

    /* 2) 超速计数 */
    if (fabsf(pobj->sensor.ev) >= cf->overspd_thresh_dps
        /*|| fabsf(pobj->sensor.gyro) >= cf->overspd_thresh_dps*/) {
        st->overspeed_count++;
    } else {
        st->overspeed_count = 0;
    }
    if (st->overspeed_count >= SF_OVERSPEED_TH) {
        st->fault.bit.overspeed = 1;
        st->brake_locked = true;
        st->overspeed_count = 0;
    }
}

static void axis_enable(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;

    axis_check_protect(pobj); /* 保护判断 */

    const bool brushed = (pobj->config.drive_type == SERVO_DRIVE_BRUSHED_PWM);

    if (pobj->status.brake_locked) {
        /* 自锁后强制关电机，等待 clear_fault */
        if (pobj->io.motor_hard_enable) pobj->io.motor_hard_enable(false);
        if (brushed) {
            if (pobj->io.set_motor_pwm)
                pobj->io.set_motor_pwm(0.0f, false); /* 有刷：占空比清零（等价 TIM3->CCR1=0） */
        } else {
            if (pobj->io.set_motor_current_mA) pobj->io.set_motor_current_mA(0, false);
        }
        pobj->status.enabled = false;
        return;
    }

    if (pobj->io.motor_hard_enable) pobj->io.motor_hard_enable(true);

    if (brushed) {
        /* 有刷直驱：输出由内层电流环 iloop 算得的电压 U_give，
         * 方向/限幅/PWM 占空比由 set_motor_pwm 钩子完成（等价 FW_PWM）。 */
        if (pobj->io.set_motor_pwm) pobj->io.set_motor_pwm(pobj->rt.U_give, true);
    } else {
        /* 无刷伺服驱动器：把电流给定发到驱动器，驱动器内部闭电流环。 */
        if (pobj->io.set_motor_current_mA) {
            int32_t mA = (int32_t)(pobj->rt.I_give * 1000.0f);
            pobj->io.set_motor_current_mA(mA, true);
        }
    }
    pobj->status.enabled = true;
}

static void axis_brake(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;
    if (pobj->io.motor_hard_enable) pobj->io.motor_hard_enable(false);

    if (pobj->config.drive_type == SERVO_DRIVE_BRUSHED_PWM) {
        if (pobj->io.set_motor_pwm) pobj->io.set_motor_pwm(0.0f, false);
        pobj->rt.U_give = 0.0f;
    } else {
        if (pobj->io.set_motor_current_mA) pobj->io.set_motor_current_mA(0, false);
    }
    pobj->rt.I_give = 0.0f;
    pobj->status.fault_timer = 0;
    pobj->status.danger_count = 0;
    pobj->status.enabled = false;
}

/* ========================================================================
 *                          给定 / 查询 / 配置
 * ====================================================================== */

static void axis_set_pos_tgt(dev_servo_axis_t *pobj, float deg)
{
    if (pobj == NULL) return;
    /* 限位裁剪（continuous 轴跳过，无限位），目标角度会被限制在机械范围内 */
    if (!pobj->config.is_continuous) {
        if (deg > pobj->config.angle_up_limit) deg = pobj->config.angle_up_limit;
        if (deg < pobj->config.angle_dn_limit) deg = pobj->config.angle_dn_limit;
    }
    pobj->rt.P_give = deg;
}

/* 设定目标角速度，单位 degrees per second */
static void axis_set_vel_tgt(dev_servo_axis_t *pobj, float dps)
{
    if (pobj == NULL) return;
    pobj->rt.V_give = dps;
}

/* 实际未使用 */
static bool axis_set_zero(dev_servo_axis_t *pobj, float angle_deg)
{
    if (pobj == NULL) return false;
    if (angle_deg < -360.0f || angle_deg > 360.0f) return false;

    /* 思路：当前编码角 - 期望显示角 = 偏置 */
//    axis_process_encoder(pobj);
    pobj->offset_deg = fmodf(pobj->sensor.angle - angle_deg, 360.0f);
    if (pobj->offset_deg < 0.0f) pobj->offset_deg += 360.0f;
    return true;
}

static float axis_get_angle(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? 0.0f : pobj->sensor.angle;
}

static float axis_get_vel(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? 0.0f : pobj->sensor.gyro;
}

static uint16_t axis_get_fault(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? 0u : pobj->status.fault.all;
}

static void axis_clear_fault(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;
    pobj->status.fault.all = 0u;
    pobj->status.brake_locked = false;
    pobj->status.fault_timer = 0;
    pobj->status.danger_count = 0;
    pobj->status.overspeed_count = 0;
}

static void axis_set_couple(dev_servo_axis_t *pobj, float sec)
{
    if (pobj == NULL) return;
    pobj->sec_couple = sec;
}

static void axis_set_dir(dev_servo_axis_t *pobj, servo_axis_dir_e dir)
{
    if (pobj == NULL) return;
    pobj->dir = dir;
}

static servo_axis_dir_e axis_get_dir(dev_servo_axis_t *pobj)
{
    return (pobj == NULL) ? SERVO_AXIS_DIR_CW : pobj->dir;
}

/**
 * @brief 电流零偏标定（CurrentInit 等价物）。
 */
static void axis_calibrate_current_bias(dev_servo_axis_t *pobj)
{
    static float sum[SERVO_AXIS_MAX] = {0.0f};
    static uint32_t cnt[SERVO_AXIS_MAX] = {0u};

    if (pobj == NULL) return;

    uint8_t i = (pobj->id < SERVO_AXIS_MAX) ? (uint8_t)pobj->id : 0u;
    uint16_t N = pobj->config.cur_calib_samples ? pobj->config.cur_calib_samples : SF_CUR_CALIB_SAMPLES;

    if (cnt[i] < N) {
        sum[i] += pobj->sensor.current + pobj->sensor.current_bias; /* 还原成测量原值再累加 */
        cnt[i]++;
        if (cnt[i] == N) {
            pobj->sensor.current_bias = sum[i] / (float)N;          /* 均值即直流零偏 */
        }
    }
}

/**
 * @brief 软限位检查。
 *        仅作上报标志，不自动刹车。
 *        注意：阈值来自 config 的控制上下限。默认 soft_limit_enable=false，
 */
static void axis_check_soft_limit(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;

    servo_axis_status_t *st = &pobj->status;
    servo_axis_config_t *c = &pobj->config;

    /* 先判断是否有限位 || 限位是否使能生效 */
    if (c->is_continuous || !c->soft_limit_enable) {
        st->xw_up = false;
        st->xw_dn = false;
        st->fault.bit.limit_hit = 0;
        return;
    }

    const float a = pobj->sensor.angle; /* 已去零点的机械角（°） */
    if (a >= c->angle_up_limit) {
        st->xw_up = true;
        st->xw_dn = false;
    } else if (a <= c->angle_dn_limit) {
        st->xw_up = false;
        st->xw_dn = true;
    } else {
        st->xw_up = false;
        st->xw_dn = false;
    }

    if (st->xw_up || st->xw_dn)
        st->fault.bit.limit_hit = 1;
    else
        st->fault.bit.limit_hit = 0;
}

/**
 * @brief 到位驻留判定
 */
static void axis_update_in_place(dev_servo_axis_t *pobj)
{
    if (pobj == NULL) return;

    servo_axis_status_t *st = &pobj->status;

    if (fabsf(pobj->rt.P_give - pobj->rt.P_fb) < SF_INPLACE_ERR_DEG) {
        if (st->inplace_dwell <= SF_INPLACE_DWELL_TH) st->inplace_dwell++;
    } else {
        st->inplace_dwell = 0;
        st->in_place = false;
    }

    if (st->inplace_dwell > SF_INPLACE_DWELL_TH) {
        st->in_place = true;
        st->inplace_dwell = SF_INPLACE_DWELL_TH + 1; /* 钳住计数 */
    }
}
