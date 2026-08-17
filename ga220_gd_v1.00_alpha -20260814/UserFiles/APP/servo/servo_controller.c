/**
 * @file    servo_controller.c
 * @brief   伺服顶层对象实现
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-05-29
 */

#include "servo_controller.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 *                          前置静态方法声明
 * ====================================================================== */

static void servo_update(dev_servo_t *pobj);
static void servo_set_cmd(dev_servo_t *pobj, const servo_cmd_t *cmd);
static void servo_set_mode(dev_servo_t *pobj, servo_mode_e mode);
static void servo_set_position(dev_servo_t *pobj, float fw_deg, float gd_deg);
static void servo_set_velocity(dev_servo_t *pobj, float fw_dps, float gd_dps);
static void servo_set_current(dev_servo_t *pobj, servo_axis_id_e axis, float i_give, bool enable);

static void servo_set_miss(dev_servo_t *pobj, float fw_miss, float fy_miss);
static void servo_set_geo(dev_servo_t *pobj, float lon, float lat, float alt, uint8_t flag);
static void servo_emerg_brake(dev_servo_t *pobj);
static void servo_get_state(dev_servo_t *pobj, servo_state_t *out);
static dev_servo_axis_t *servo_get_axis(dev_servo_t *pobj, servo_axis_id_e id);

/* 内部模式处理函数（每个模式对应一个） */
static void mode_voltage_handler(dev_servo_t *pobj);
static void mode_brake_handler(dev_servo_t *pobj);
static void mode_enbrake_handler(dev_servo_t *pobj);
static void mode_afollow_handler(dev_servo_t *pobj);
static void mode_vmotion_handler(dev_servo_t *pobj);
static void mode_zero_handler(dev_servo_t *pobj);
static void mode_withdraw_handler(dev_servo_t *pobj);
static void mode_track_handler(dev_servo_t *pobj);
static void mode_geo_handler(dev_servo_t *pobj);
static void mode_current_handler(dev_servo_t *pobj);
static void mode_selfcheck_handler(dev_servo_t *pobj);

static void servo_refresh_couple(dev_servo_t *pobj);
static void servo_update_motion_state(dev_servo_t *pobj);

/* ========================================================================
 *                              内部工具
 * ====================================================================== */
static float servo_clampf(float in, float lo, float hi)
{
    if (lo > hi) {
        float t = lo;
        lo = hi;
        hi = t;
    }
    if (in > hi)
        return hi;
    if (in < lo)
        return lo;
    return in;
}

/* ========================================================================
 *                              初始化
 * ====================================================================== */

void dev_servo_init(dev_servo_t *pobj, const servo_axis_io_t *fw_io, const servo_axis_io_t *gd_io)
{
    if (pobj == NULL || fw_io == NULL || gd_io == NULL)
        return;

    memset(pobj, 0, sizeof(dev_servo_t));

    /* 1) 初始化两根轴 */
    dev_servo_axis_init(&pobj->fw_axis, SERVO_AXIS_FW, fw_io);
    dev_servo_axis_init(&pobj->gd_axis, SERVO_AXIS_GD, gd_io);

    /* 2) 默认模式：停车 */
    pobj->mode = SERVO_MODE_BRAKE;
    pobj->mode_pending = SERVO_MODE_BRAKE;
    pobj->mode_last = (servo_mode_e)0;                /* 0 不是任何合法模式，保证首次进入任意模式都触发清环 */
    pobj->sub_last = 0xFF;
    pobj->selfcheck_step = SERVO_SELFCHECK_STEP_DONE; /* 上电默认视为"未在自检"，避免误屏蔽外部指令 */

    /* 3) 装方法表 */
    pobj->update = servo_update;
    pobj->set_cmd = servo_set_cmd;
    pobj->set_mode = servo_set_mode;
    pobj->set_position = servo_set_position;
    pobj->set_velocity = servo_set_velocity;
    pobj->set_miss = servo_set_miss;
    pobj->set_geo = servo_set_geo;
    pobj->emergency_brake = servo_emerg_brake;
    pobj->get_state = servo_get_state;
    pobj->get_axis = servo_get_axis;

    pobj->set_current = servo_set_current;
}

/* ========================================================================
 *                          公开方法实现
 * ====================================================================== */

static void servo_set_cmd(dev_servo_t *pobj, const servo_cmd_t *cmd)
{
    if (pobj == NULL || cmd == NULL)
        return;
    pobj->cmd = *cmd;
    pobj->mode_pending = (servo_mode_e)cmd->cmd;
}

static void servo_set_mode(dev_servo_t *pobj, servo_mode_e mode)
{
    if (pobj == NULL)
        return;
    pobj->mode_pending = mode;
}

static void servo_set_position(dev_servo_t *pobj, float fw_deg, float gd_deg)
{
    if (pobj == NULL)
        return;
    pobj->cmd.hand_search_fw_pos = fw_deg;
    pobj->cmd.hand_search_gd_pos = gd_deg;
}

static void servo_set_velocity(dev_servo_t *pobj, float fw_dps, float gd_dps)
{
    if (pobj == NULL)
        return;
    pobj->cmd.hand_search_fw_v = fw_dps;
    pobj->cmd.hand_search_gd_v = gd_dps;
}

static void servo_set_current(dev_servo_t *pobj, servo_axis_id_e axis, float i_give, bool enable)
{
    if (pobj == NULL)
        return;
    pobj->cur_axis = axis;
    pobj->cur_give = i_give;
    pobj->cur_enable = enable;
}

static void servo_set_miss(dev_servo_t *pobj, float fw_miss, float fy_miss)
{
    if (pobj == NULL)
        return;

    /* 对输入的脱靶量进行初步限幅 */
    float fw_tbl = servo_clampf(fw_miss, -10.0F, 10.0F);
    float fy_tbl = servo_clampf(fy_miss, -10.0F, 10.0F);

    pobj->cmd.fw_miss_deg = fw_tbl;
    pobj->cmd.fy_miss_deg = fy_tbl;
}

static void servo_set_geo(dev_servo_t *pobj, float lon, float lat, float alt, uint8_t flag)
{
    if (pobj == NULL)
        return;
    pobj->cmd.geo_lon = lon;
    pobj->cmd.geo_lat = lat;
    pobj->cmd.geo_hgh = alt;
    pobj->cmd.geo_track_flag = flag;
}

static void servo_emerg_brake(dev_servo_t *pobj)
{
    if (pobj == NULL)
        return;
    pobj->fw_axis.clear_all(&pobj->fw_axis);
    pobj->gd_axis.clear_all(&pobj->gd_axis);
    pobj->fw_axis.brake(&pobj->fw_axis);
    pobj->gd_axis.brake(&pobj->gd_axis);
    pobj->mode = SERVO_MODE_BRAKE;
    pobj->mode_pending = SERVO_MODE_BRAKE;
}

static void servo_get_state(dev_servo_t *pobj, servo_state_t *out)
{
    if (pobj == NULL || out == NULL)
        return;

    out->mode = (uint16_t)pobj->mode;
    out->fw_angle = pobj->fw_axis.sensor.angle;
    out->gd_angle = pobj->gd_axis.sensor.angle;
    out->fw_vel = pobj->fw_axis.sensor.gyro;
    out->gd_vel = pobj->gd_axis.sensor.gyro;
    out->fw_cur = pobj->fw_axis.sensor.current;
    out->gd_cur = pobj->gd_axis.sensor.current;
    out->fw_fault = pobj->fw_axis.status.fault.all;
    out->gd_fault = pobj->gd_axis.status.fault.all;

    /* 到位状态 */
    out->reach = pobj->fw_axis.status.in_place && pobj->gd_axis.status.in_place;

    /* 运动/静止判定（evmeancal） */
    out->motion_state = pobj->motion_state;

    /* 自检步骤（非自检模式下值无意义，上位机应结合 out->mode 一起判断） */
    out->selfcheck_step = (uint8_t)pobj->selfcheck_step;

    *(&pobj->state) = *out;
}

/**
 * @brief 根据传入的轴 ID，从伺服对象里取出对应的轴对象指针。
 */
static dev_servo_axis_t *servo_get_axis(dev_servo_t *pobj, servo_axis_id_e id)
{
    if (pobj == NULL)
        return NULL;
    if (id == SERVO_AXIS_FW)
        return &pobj->fw_axis;
    if (id == SERVO_AXIS_GD)
        return &pobj->gd_axis;
    return NULL;
}

/* ========================================================================
 *                          周期主调用
 * ====================================================================== */

/**
 * @brief 把 FW 受 GD 解耦因子 (1/cos(GD 角)) 写入 FW 轴
 */
static void servo_refresh_couple(dev_servo_t *pobj)
{
    float gd_angle = pobj->gd_axis.sensor.angle;
    float c = cosf(gd_angle * 0.017453293f);
    float sec = (fabsf(c) > 1e-6f) ? (1.0f / c) : 1.0f;
    pobj->fw_axis.set_couple(&pobj->fw_axis, sec);
}

/**
 * @brief 运动/静止判定（evmeancal 等价物）。
 *
 *        对两轴编码器微分速度 ev 累加 1000 拍（≈1s）求均值，|均值|<0.1°/s 判静止。
 *        结果写 pobj->motion_state（1=静止, 0=运动）。
 */
static void servo_update_motion_state(dev_servo_t *pobj)
{
    static uint16_t ev_sum_time = 0;
    static float gd_ev_sum = 0.0f, fw_ev_sum = 0.0f;

    ev_sum_time++;
    gd_ev_sum += pobj->gd_axis.sensor.ev;
    fw_ev_sum += pobj->fw_axis.sensor.ev;

    if (ev_sum_time >= 1000u) {
        const float gd_mean = gd_ev_sum * 0.001f;
        const float fw_mean = fw_ev_sum * 0.001f;
        pobj->motion_state = (fabsf(gd_mean) < 0.1f && fabsf(fw_mean) < 0.1f) ? 1u : 0u;
        ev_sum_time = 0;
        gd_ev_sum = 0.0f;
        fw_ev_sum = 0.0f;
    }
}

/**
 * @brief 1ms 主循环——固定六步：
 *        ① 传感器更新 → ② 同步反馈 → ③ FW/GD 解耦 → ④ 模式切换 → ⑤ 跑模式处理 → ⑤b 到位/运动判定 → ⑥ 控制电机
 */
static void servo_update(dev_servo_t *pobj)
{
    if (pobj == NULL)
        return;

    /* ① 传感器 */
    pobj->fw_axis.update_sensor(&pobj->fw_axis);
    pobj->gd_axis.update_sensor(&pobj->gd_axis);

    /* ② 同步控制环反馈 */
    pobj->fw_axis.sync_feedback(&pobj->fw_axis);
    pobj->gd_axis.sync_feedback(&pobj->gd_axis);

    /* ③ 解耦因子 */
    servo_refresh_couple(pobj);

    /* ④ 模式生效（commandprocess 的等价物） */
    pobj->mode = pobj->mode_pending;

    /* ⑤ 模式机（allprocess 的等价物） */
    switch (pobj->mode) {
        case SERVO_MODE_VOLTAGE:
            mode_voltage_handler(pobj);
            break;
        case SERVO_MODE_BRAKE:
            mode_brake_handler(pobj);
            break;
        case SERVO_MODE_ENBRAKE:
            mode_enbrake_handler(pobj);
            break;
        case SERVO_MODE_AFOLLOW:
            mode_afollow_handler(pobj);
            break;
        case SERVO_MODE_VMOTION:
            mode_vmotion_handler(pobj);
            break;
        case SERVO_MODE_ZERO:
            mode_zero_handler(pobj);
            break;
        case SERVO_MODE_WITHDRAW:
            mode_withdraw_handler(pobj);
            break;
        case SERVO_MODE_TRACK:
            mode_track_handler(pobj);
            break;
        case SERVO_MODE_GEO:
            mode_geo_handler(pobj);
            break;
        case SERVO_MODE_CURRENT:
            mode_current_handler(pobj);
            break;
        case SERVO_MODE_SELFCHECK:
            mode_selfcheck_handler(pobj);
            break;
        default:
            /* CIR_SCAN / ZDLC 暂未迁移，留作扩展 */
            mode_brake_handler(pobj);
            break;
    }

    /* ⑤ 到位驻留判定 + 运动/静止判定。update_in_place 依赖本拍 P_give/P_fb，故放在模式机之后。 */
    pobj->fw_axis.update_in_place(&pobj->fw_axis);
    pobj->gd_axis.update_in_place(&pobj->gd_axis);
    servo_update_motion_state(pobj);

    /* ⑥ 记录本拍模式，供下一拍判断"是否刚切入" */
    pobj->mode_last = pobj->mode;
}

/* ========================================================================
 *                          模式处理函数
 * ====================================================================== */

/** 停车：所有环清零 + 电机断电 */
static void mode_brake_handler(dev_servo_t *pobj)
{
    pobj->fw_axis.clear_all(&pobj->fw_axis);
    pobj->gd_axis.clear_all(&pobj->gd_axis);
    pobj->fw_axis.brake(&pobj->fw_axis);
    pobj->gd_axis.brake(&pobj->gd_axis);
}
/** 开环 PWM：双轴直接用 g_olp_u_give 作为 U_give，不跑任何闭环 */
static void mode_voltage_handler(dev_servo_t *pobj)
{
    pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_VOLTAGE, 0);
    pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_VOLTAGE, 0);
    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/** 软刹（ENBrake）：速度目标 0，仍跑速度环，
 *  控制链路：
 *  有陀螺时：V_SPEED -> ACC -> CURRENT，
 *  无陀螺时实际是 … → EV_SPEED → CURRENT（跳过加速度环） */
static void mode_enbrake_handler(dev_servo_t *pobj)
{
    /* 仅在“进入软刹”这一拍清外环；之后让速度/加速度环正常积分 */
    if (pobj->mode != pobj->mode_last) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_EV_SPEED);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_EV_SPEED);
    }

    pobj->fw_axis.set_velocity_target(&pobj->fw_axis, 0.0f);
    pobj->gd_axis.set_velocity_target(&pobj->gd_axis, 0.0f);
    pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_V_SPEED, 0);
    pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_V_SPEED, 0);
    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/** AFOLLOW：位置环（程控），控制链路：POSITION -> EV_SPEED -> CURRENT */
static void mode_afollow_handler(dev_servo_t *pobj)
{
    if (pobj->mode != pobj->mode_last) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_POSITION);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_POSITION);
    }

    pobj->fw_axis.set_position_target(&pobj->fw_axis, pobj->cmd.hand_search_fw_pos);
    pobj->gd_axis.set_position_target(&pobj->gd_axis, pobj->cmd.hand_search_gd_pos);

    pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_POSITION, 0);
    pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_POSITION, 0);

    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/** VMOTION：手控速度运动 */
// static void mode_vmotion_handler(dev_servo_t *pobj)
// {
//     /* 万向节锁规避：方位轴架在俯仰上，有效旋转含 1/cos(俯仰角)。
//        俯仰逼近 ±90° 时 1/cos→∞、方位陀螺失稳，故改用编码器惯性速度环 EV。
//        —— 但这只在“有陀螺”时才有意义：无陀螺本就只能用 EV，90° 判据无效且会引发边界抖动误复位。 */
//     bool has_gyro = pobj->gd_axis.config.has_gyro; /* 判据基于 GD 角，取 GD 轴；假设整机陀螺配置一致 */

//     /* 有陀螺：按 |GD|→90° 切换，带 2° 迟滞防边界抖动；无陀螺：ev_mode 恒为真 */
//     float d = fabsf(fabsf(pobj->gd_axis.rt.P_fb) - 90.0f);
//     bool ev_mode_prev = (pobj->sub_last == 1u);

//     /* 无陀螺（has_gyro==false）：!has_gyro 为真 → 短路 → ev_mode = true，永远走 EV。*/
//     /* 有陀螺（has_gyro==true）：!has_gyro 为假 → 结果完全由右边运算决定（进入迟滞逻辑）。 */
//     /* 2° 迟滞死区，防止角度在阈值附近微抖时 ev_mode 来回跳。 */
//     bool ev_mode = (!has_gyro) || (ev_mode_prev ? (d < 17.0f) : (d < 15.0f));

//     /* 仅在“切入本模式”或“速度支路真正翻转(EV↔陀螺)”时复位一次。
//        无陀螺时 ev_mode 恒为真 → sub_last 恒定 → 不会因 90° 边界抖动误复位。 */
//     if (pobj->mode != pobj->mode_last || (uint8_t)ev_mode != pobj->sub_last) {

//         pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_EV_SPEED);
//         pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_V_SPEED);
//         pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_EV_SPEED);
//         pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_V_SPEED);
//     }
//     pobj->sub_last = (uint8_t)ev_mode;

//     if (ev_mode) {
//         /* 编码器速度环 */
//         pobj->fw_axis.rt.Ev_give = pobj->cmd.hand_search_fw_v;
//         pobj->gd_axis.rt.Ev_give = pobj->cmd.hand_search_gd_v;
//         pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_EV_SPEED, 0);
//         pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_EV_SPEED, 0);
//     } else {
//         /* 陀螺速度环 */
//         pobj->fw_axis.set_velocity_target(&pobj->fw_axis, pobj->cmd.hand_search_fw_v);
//         pobj->gd_axis.set_velocity_target(&pobj->gd_axis, pobj->cmd.hand_search_gd_v);
//         pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_V_SPEED, 0);
//         pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_V_SPEED, 0);
//     }
//     pobj->fw_axis.enable(&pobj->fw_axis);
//     pobj->gd_axis.enable(&pobj->gd_axis);
// }

/** VMOTION：手控速度运动 */
static void mode_vmotion_handler(dev_servo_t *pobj)
{
    /*
     * 方位轴安装在俯仰轴上：
     * 当俯仰角接近 ±90° 时，方位轴等效旋转会受到 1/cos(GD) 放大影响，
     * 此时方位陀螺速度容易失稳，所以需要切换到编码器速度环 EV。
     *
     * 有陀螺：
     *   - 远离 ±90°：使用陀螺速度环 SERVO_LOOP_V_SPEED
     *   - 接近 ±90°：使用编码器速度环 SERVO_LOOP_EV_SPEED
     *
     * 无陀螺：
     *   - 没有陀螺速度反馈，只能一直使用编码器速度环 EV
     */
    bool has_gyro = pobj->gd_axis.config.has_gyro;

    /*
     * 迟滞阈值：
     *   dist_to_90_deg < 15°  时，进入 EV 编码器速度环
     *   dist_to_90_deg >= 17° 时，退出 EV，回到陀螺速度环
     *
     * 中间 15°~17° 为迟滞区，保持上一拍状态，防止边界附近来回切换。
     */
    const float EV_MODE_ENTER_DEG = 15.0f;
    const float EV_MODE_EXIT_DEG = 17.0f;

    /*
     * 计算当前俯仰角距离 ±90° 的距离。
     * 例：
     *   GD =  90°，dist_to_90_deg = 0°
     *   GD =  80°，dist_to_90_deg = 10°
     *   GD = -90°，dist_to_90_deg = 0°
     *   GD = -70°，dist_to_90_deg = 20°
     */
    float gd_abs_deg = fabsf(pobj->gd_axis.rt.P_fb);
    float dist_to_90_deg = fabsf(gd_abs_deg - 90.0f);

    /*
     * sub_last 在 VMOTION 中保存上一拍 ev_mode：
     *   0 = 上一拍使用陀螺速度环
     *   1 = 上一拍使用编码器速度环 EV
     *
     * 注意：
     *   刚切入 VMOTION 时，sub_last 可能是其他模式留下来的值，
     *   所以只有当前后两拍都在 VMOTION 内时，才采信 sub_last。
     */
    bool mode_changed = (pobj->mode != pobj->mode_last);
    bool ev_mode_prev = (!mode_changed) && (pobj->sub_last == 1u);
    bool ev_mode = false;

    if (!has_gyro) {
        /*
         * 无陀螺：
         * 只能使用编码器速度环 EV。
         */
        ev_mode = true;
    } else if (ev_mode_prev) {
        /*
         * 有陀螺，且上一拍已经在 EV：
         * 只有距离 ±90° 足够远时才退出 EV。
         */
        ev_mode = (dist_to_90_deg < EV_MODE_EXIT_DEG);
    } else {
        /*
         * 有陀螺，且上一拍不在 EV：
         * 只有距离 ±90° 足够近时才进入 EV。
         */
        ev_mode = (dist_to_90_deg < EV_MODE_ENTER_DEG);
    }

    /*
     * 只在以下两种情况清控制环：
     *   1. 刚切入 VMOTION 模式
     *   2. 速度环类型发生切换：陀螺速度环 <-> 编码器速度环
     */
    if (mode_changed || ((uint8_t)ev_mode != pobj->sub_last)) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_EV_SPEED);
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_V_SPEED);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_EV_SPEED);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_V_SPEED);
    }

    /* 保存本拍速度环选择，供下一拍迟滞判断使用 */
    pobj->sub_last = (uint8_t)ev_mode;

    if (ev_mode) { /* 编码器速度环：使用编码器微分/估算速度作为反馈。*/
        pobj->fw_axis.rt.Ev_give = pobj->cmd.hand_search_fw_v;
        pobj->gd_axis.rt.Ev_give = pobj->cmd.hand_search_gd_v;

        pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_EV_SPEED, 0);
        pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_EV_SPEED, 0);
    } else { /* 陀螺速度环：使用陀螺速度作为反馈。*/
        pobj->fw_axis.set_velocity_target(&pobj->fw_axis, pobj->cmd.hand_search_fw_v);
        pobj->gd_axis.set_velocity_target(&pobj->gd_axis, pobj->cmd.hand_search_gd_v);

        pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_V_SPEED, 0);
        pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_V_SPEED, 0);
    }

    /* 输出到执行机构 */
    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/** 调零，控制链路：POSITION -> EV_SPEED -> CURRENT */
static void mode_zero_handler(dev_servo_t *pobj)
{
    if (pobj->mode != pobj->mode_last) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_POSITION);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_POSITION);
    }

    pobj->fw_axis.set_position_target(&pobj->fw_axis, 0.0f);
    pobj->gd_axis.set_position_target(&pobj->gd_axis, 0.0f);
    pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_POSITION, 0);
    pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_POSITION, 0);
    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/** 收藏：FW 回 0°，GD 回 0° */
static void mode_withdraw_handler(dev_servo_t *pobj)
{
    if (pobj->mode != pobj->mode_last) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_POSITION);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_POSITION);
    }

    pobj->fw_axis.set_position_target(&pobj->fw_axis, 0.0f);
    pobj->gd_axis.set_position_target(&pobj->gd_axis, 0.0f);
    pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_POSITION, 0);
    pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_POSITION, 0);
    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/** TRACK：脱靶量跟踪 */
static void mode_track_handler(dev_servo_t *pobj)
{
    if (pobj->mode != pobj->mode_last) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_TRACK);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_TRACK);
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_V_SPEED);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_V_SPEED);
    }

    /* 0=不跟踪：切到 0 速度环（停在原地） */
    if (pobj->cmd.track_flag == 0) {
        pobj->fw_axis.set_velocity_target(&pobj->fw_axis, 0.0f);
        pobj->gd_axis.set_velocity_target(&pobj->gd_axis, 0.0f);
        pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_V_SPEED, 0);
        pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_V_SPEED, 0);
    } else {
        /* —— 脱靶量限幅：在“用之前”钳位。 —— */
        float fw_miss_deg = servo_clampf(pobj->cmd.fw_miss_deg, -10.0F, 10.0F);
        float fy_miss_deg = servo_clampf(pobj->cmd.fy_miss_deg, -10.0F, 10.0F);

        /*  把脱靶量写入 T_fb，
            有陀螺时控制链路：T1 环跑跟踪 → 速度 → 加速度 → 电流 
            无陀螺时控制链路：T1 环跑跟踪 → EV速度 → 电流 */
        pobj->fw_axis.rt.T_fb = fw_miss_deg;
        pobj->gd_axis.rt.T_fb = fy_miss_deg;

        /* —— FW 万向锁规避：俯仰 |GD|≥75° 时 1/cos 放大、方位陀螺失稳 → 改走 T1→EV速度→电流。 */
        bool fw_gimbal_lock = pobj->fw_axis.config.has_gyro && (fabsf(pobj->gd_axis.rt.P_fb) >= 75.0f);
        bool fw_saved_gyro = pobj->fw_axis.config.has_gyro;
        if (fw_gimbal_lock)
            pobj->fw_axis.config.has_gyro = false;

        /* ⚠ 限速 xx°/s：把限幅值 比如25.0f 写进 p->Vmax，由 gdtloop_TV1 自身钳幅（在 load_default_param 里设）。 */
        pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_TRACK, 0);
        pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_TRACK, 0);

        pobj->fw_axis.config.has_gyro = fw_saved_gyro; /* 还原，避免影响下游/下一拍 */
    }
    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/**
 * @brief GEO：地理引导。
 *
 *        控制链路：GEO 环（fwgeo/gdgeo，输出速度）→ 速度环 → (加速度环) → 电流环。与 TRACK 一样，停止时退回 0 速度环。
 *
 *        ⚠ 经纬高 → 目标方位/俯仰角 的换算需要本机自身经纬高 + 航向 + 姿态，
 *          这些数据源当前不在 servo 模块内，故以 TODO 占位：换算未就绪前目标角恒 0。
 */
static void mode_geo_handler(dev_servo_t *pobj)
{
    if (pobj->mode != pobj->mode_last) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_GEO);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_GEO);
    }

    /* 0=停止引导：原地 0 速（与 TRACK 的不跟踪分支一致） */
    if (pobj->cmd.geo_track_flag == 0) {
        pobj->fw_axis.set_velocity_target(&pobj->fw_axis, 0.0f);
        pobj->gd_axis.set_velocity_target(&pobj->gd_axis, 0.0f);
        pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_V_SPEED, 0);
        pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_V_SPEED, 0);
    } else {
        /* ===== TODO(GEO 换算)：经纬高 → 目标方位/俯仰角 =====================
         *   输入：pobj->cmd.geo_lon / geo_lat / geo_hgh（目标），
         *         本机自身经纬高 + 航向 + 姿态（数据源待接入：GPS/惯导）。
         *   输出：fw_tgt_deg（方位目标角）、gd_tgt_deg（俯仰目标角）。
         *   换算就绪前先恒 0 占位（电机停在零点附近，不产生地理跟踪）。
         *
         *   示例（待实现）：
         *     geo_to_pointing(pobj->cmd.geo_lon, pobj->cmd.geo_lat, pobj->cmd.geo_hgh,
         *                     own_lon, own_lat, own_hgh, own_heading,
         *                     &fw_tgt_deg, &gd_tgt_deg);
         * =================================================================== */
        float fw_tgt_deg = 0.0f;
        float gd_tgt_deg = 0.0f;

        /* GEO 环：目标角写 GEO_give，当前角写 GEO_fb */
        pobj->fw_axis.rt.GEO_give = fw_tgt_deg;
        pobj->fw_axis.rt.GEO_fb = pobj->fw_axis.sensor.angle;
        pobj->gd_axis.rt.GEO_give = gd_tgt_deg;
        pobj->gd_axis.rt.GEO_fb = pobj->gd_axis.sensor.angle;

        pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_GEO, 0);
        pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_GEO, 0);
    }

    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);
}

/**
 * @brief 子节点:只跑电流环。用 set_current 缓存的(给定+使能)→跑电流环→驱动/刹车。
 *        位置/速度闭环在主节点完成,本节点不碰外环。
 */
static void mode_current_handler(dev_servo_t *pobj)
{
    dev_servo_axis_t *ax = pobj->get_axis(pobj, pobj->cur_axis);
    if (ax == NULL)
        return;

    if (pobj->mode != pobj->mode_last) {
        ax->clear_loops(ax, SERVO_LOOP_CURRENT); /* 切入清积分 */
    }

    ax->rt.I_give = pobj->cur_give;
    ax->run_loop(ax, SERVO_LOOP_CURRENT, 0); /* 有刷:内部闭电流环→U_give;无刷:透传 */

    if (pobj->cur_enable)
        ax->enable(ax);
    else
        ax->brake(ax);
}

/**
 * @brief 自检：方位（FW）全程位置环保持 0°；照准架（GD）依次走
 *        "转上限位 → 转下限位 → 回零位" 三步，每步用现有的到位驻留判定
 *        `status.in_place` 触发下一步跳转，跳到零位并到位后进入 DONE 常驻。
 *
 *        控制链路与 AFOLLOW/ZERO/WITHDRAW 一致：POSITION -> EV_SPEED -> CURRENT。
 *        限位角度直接取 gd_axis.config 里已标定好的 angle_up_limit/angle_dn_limit，
 *        不复用 xw_up/xw_dn/fault.bit.limit_hit —— 那套标志的语义是"意外触发限位即故障"，
 *        自检主动走到限位是预期成功路径，不应被上报成故障。
 */
static void mode_selfcheck_handler(dev_servo_t *pobj)
{
    if (pobj->mode != pobj->mode_last) {
        pobj->fw_axis.clear_loops(&pobj->fw_axis, SERVO_LOOP_POSITION);
        pobj->gd_axis.clear_loops(&pobj->gd_axis, SERVO_LOOP_POSITION);
    }

    /* 方位（FW）全程保持零位 */
    pobj->fw_axis.set_position_target(&pobj->fw_axis, 0.0f);

    /* 照准架（GD）目标角按当前自检步骤取值 */
    float gd_target_deg;
    switch (pobj->selfcheck_step) {
        case SERVO_SELFCHECK_STEP_TO_UP:
            gd_target_deg = pobj->gd_axis.config.angle_up_limit;
            break;
        case SERVO_SELFCHECK_STEP_TO_DN:
            gd_target_deg = pobj->gd_axis.config.angle_dn_limit;
            break;
        case SERVO_SELFCHECK_STEP_TO_ZERO:
        case SERVO_SELFCHECK_STEP_DONE:
        default:
            gd_target_deg = 0.0f;
            break;
    }
    pobj->gd_axis.set_position_target(&pobj->gd_axis, gd_target_deg);

    pobj->fw_axis.run_loop(&pobj->fw_axis, SERVO_LOOP_POSITION, 0);
    pobj->gd_axis.run_loop(&pobj->gd_axis, SERVO_LOOP_POSITION, 0);
    pobj->fw_axis.enable(&pobj->fw_axis);
    pobj->gd_axis.enable(&pobj->gd_axis);

    /* 本拍 GD 到位（到位判定用的是上一拍的 P_give/P_fb，故此处天然是"上一拍已到位"）
     * 则推进到下一步；目标角本拍才刚改，update_in_place 会在本拍末尾据新目标重新计数，
     * 不会因为提前跳步而误判"仍到位"。 */
    if (pobj->gd_axis.status.in_place) {
        switch (pobj->selfcheck_step) {
            case SERVO_SELFCHECK_STEP_TO_UP:
                pobj->selfcheck_step = SERVO_SELFCHECK_STEP_TO_DN;
                break;
            case SERVO_SELFCHECK_STEP_TO_DN:
                pobj->selfcheck_step = SERVO_SELFCHECK_STEP_TO_ZERO;
                break;
            case SERVO_SELFCHECK_STEP_TO_ZERO:
                pobj->selfcheck_step = SERVO_SELFCHECK_STEP_DONE;
                break;
            default:
                break; /* DONE：保持 */
        }
    }
}
