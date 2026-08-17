/**
 * @file    servo_test.c
 * @brief   伺服系统激励测试对象实现（重构自 SF_20260303.c::Test）
 *
 * @author  LinHui
 * @version 1.00
 * @date    2026-06-01
 */

#include "servo_test.h"
#include <math.h>
#include <string.h>

/** @brief 2π/1000：tick_count 以 ms 计，故 sin(2π·f·t) = sin(0.00628·f·count) */
#define SF_TEST_OMEGA_PER_MS 0.00628f

/* ========================================================================
 *                          前置静态方法声明
 * ====================================================================== */

static void test_start(dev_servo_test_t *pobj, const servo_test_spec_t *spec);
static void test_stop(dev_servo_test_t *pobj);
static bool test_tick(dev_servo_test_t *pobj);
static float test_excitation(dev_servo_test_t *pobj);
static bool test_is_running(dev_servo_test_t *pobj);

/* 内部工具 */
static void test_brake_all(dev_servo_test_t *pobj);

/* ========================================================================
 *                          对外初始化 API
 * ====================================================================== */

void dev_servo_test_init(dev_servo_test_t *pobj, dev_servo_t *servo)
{
    if (pobj == NULL || servo == NULL) return;

    pobj->servo = servo;
    pobj->tick_count = 0;
    pobj->running = false;
    pobj->out_flag = 0;

    /* 清零参数包 */
    pobj->spec.axis = SERVO_AXIS_FW;
    pobj->spec.loop = SERVO_TEST_LOOP_CURRENT;
    pobj->spec.mode = SERVO_TEST_CLOSED_LOOP;
    pobj->spec.amp = 0.0f;
    pobj->spec.freq = 0.0f;
    pobj->spec.time_s = 0.0f;
    pobj->spec.wave = SERVO_TEST_WAVE_SINE; /* 激励波形类型设置 */
    pobj->spec.step_delay_ms = 0u;          /* 阶跃前的等待时间 */
    pobj->spec.step_min = 0.0f;             /* 周期阶跃下限 */
    pobj->spec.step_max = 0.0f;             /* 周期阶跃上限 */
    pobj->spec.step_period_ms = 0u;         /* 周期阶跃周期 */

    /* 装方法表 */
    pobj->start = test_start;
    pobj->stop = test_stop;
    pobj->tick = test_tick;
    pobj->excitation = test_excitation;
    pobj->is_running = test_is_running;
}

/* ========================================================================
 *                          内部工具
 * ====================================================================== */

/**
 * @brief 双轴清环 + 刹车（等价原 clr_all() + all_brake()）
 */
static void test_brake_all(dev_servo_test_t *pobj)
{
    dev_servo_t *s = pobj->servo;
    s->fw_axis.clear_all(&s->fw_axis);
    s->gd_axis.clear_all(&s->gd_axis);
    s->fw_axis.brake(&s->fw_axis);
    s->gd_axis.brake(&s->gd_axis);
}

/* ========================================================================
 *                          start / stop
 * ====================================================================== */

static void test_start(dev_servo_test_t *pobj, const servo_test_spec_t *spec)
{
    if (pobj == NULL || spec == NULL || pobj->servo == NULL) return;

    pobj->spec = *spec;
    pobj->tick_count = 0;
    pobj->out_flag = 0;
    pobj->running = true;

    /* 进入测试前清一次受测轴的所有环，避免历史 PI 积分残留影响激励响应。
     * （注意：测试期间不再清环，否则会清掉正在累积的 PI 积分器。） */
    dev_servo_axis_t *axis = pobj->servo->get_axis(pobj->servo, spec->axis);
    if (axis != NULL) axis->clear_all(axis);
}

static void test_stop(dev_servo_test_t *pobj)
{
    if (pobj == NULL || pobj->servo == NULL) return;
    pobj->running = false;
    pobj->out_flag = 0;
    test_brake_all(pobj);
}

/* ========================================================================
 *                          激励信号
 * ====================================================================== */

/**
 * @brief 计算当前周期激励值，按 spec.wave 选择信号类型：
 *          FIXED — 恒为 amp
 *          STEP  — 起跳延迟前为 0，之后为 amp
 *          SINE  — amp*sin(ω·freq·tick)
 *          PULSE — 周期方波：延迟前保持 step_min，之后每 step_period_ms 一个整周期，
 *                  前半周期输出 step_max、后半周期输出 step_min（50% 占空比）
 */
static float test_excitation(dev_servo_test_t *pobj)
{
    if (pobj == NULL) return 0.0f;

    const servo_test_spec_t *s = &pobj->spec;

    switch (s->wave) {
        case SERVO_TEST_WAVE_SINE:
            return s->amp * sinf(SF_TEST_OMEGA_PER_MS * s->freq * (float)pobj->tick_count);

        case SERVO_TEST_WAVE_STEP: return (pobj->tick_count < s->step_delay_ms) ? 0.0f : s->amp;

        case SERVO_TEST_WAVE_PULSE: {
            /* 周期为 0 时退化为恒定 step_max，避免取模除零 */
            if (s->step_period_ms == 0u) return s->step_max;

            /* 起跳延迟内保持下限 */
            if (pobj->tick_count < s->step_delay_ms) return s->step_min;

            /* 相对延迟起点的周期内相位 */
            uint32_t phase = (pobj->tick_count - s->step_delay_ms) % s->step_period_ms;

            /* 前半周期 max，后半周期 min */
            return (phase < (s->step_period_ms / 2u)) ? s->step_max : s->step_min;
        }

        case SERVO_TEST_WAVE_FIXED:
        default: return s->amp;
    }
}

static bool test_is_running(dev_servo_test_t *pobj)
{
    return (pobj == NULL) ? false : pobj->running;
}

/* ========================================================================
 *                          周期推进（核心）
 * ====================================================================== */

static bool test_tick(dev_servo_test_t *pobj)
{
    if (pobj == NULL || pobj->servo == NULL || !pobj->running) return false;

    dev_servo_axis_t *axis = pobj->servo->get_axis(pobj->servo, pobj->spec.axis);
    if (axis == NULL) {
        pobj->running = false;
        return false;
    }

    /* —— 计时（先自增，再判定，与原 Test() 一致）—— */
    pobj->tick_count++;
    if (pobj->tick_count > (uint32_t)(pobj->spec.time_s * 1000.0f)) {
        /* 到时：清环 + 双轴刹车（原 else 分支：clr_all + all_brake）*/
        test_brake_all(pobj);
        pobj->running = false;
        pobj->out_flag = 0;
        return false;
    }

    /* —— 激励值 + 输出模式标志 —— */
    const float u = test_excitation(pobj);
    const bool closed = (pobj->spec.mode == SERVO_TEST_CLOSED_LOOP);

    /* out_flag 等价原 Test_flag：电压开环=3；电流输出按轴区分 GD=1 / FW=2 */
    const uint8_t cur_flag = (pobj->spec.axis == SERVO_AXIS_GD) ? 1u : 2u;

    /* —— 按受测环注入激励并级联 —— */
    switch (pobj->spec.loop) {
        /* ---- 电流环：开环=电压开环 / 闭环=电流闭环 ---- */
        case SERVO_TEST_LOOP_CURRENT:
            if (closed) {
                /* 电流闭环：无刷伺服驱动器在驱动器内部闭环，这里只给电流给定；
                 * 有刷直驱则需 MCU 再闭一层 iloop → U_give。run_loop(CURRENT) 对无刷伺服驱动器
                 * 为空操作（行为不变），对有刷会自动算出 U_give 供 enable() 输出 PWM。 */
                axis->rt.I_give = u;
                axis->run_loop(axis, SERVO_LOOP_CURRENT, 0);
                pobj->out_flag = cur_flag;
            } else {
                axis->rt.U_give = u;
                axis->rt.I_give = 0.0f;
                pobj->out_flag = 3u;
            }
            axis->enable(axis);
            break;

        /* ---- 加速度环 ---- */
        case SERVO_TEST_LOOP_ACC:
            if (closed) {
                axis->rt.A_give = u;
                axis->run_loop(axis, SERVO_LOOP_ACC, 0);
            } else {
                axis->rt.I_give = u; /* 开环：直接给电流，跳过加速度环 PI */
            }
            pobj->out_flag = cur_flag;
            axis->enable(axis);
            break;

        /* ---- 速度环（陀螺反馈） ---- */
        case SERVO_TEST_LOOP_VEL:
            if (closed) {
                axis->set_velocity_target(axis, u);          /* 写 V_give */
                axis->run_loop(axis, SERVO_LOOP_V_SPEED, 0); /* V→A→I */
            } else {
                axis->rt.I_give = u;
            }
            pobj->out_flag = cur_flag;
            axis->enable(axis);
            break;

        /* ---- 速度环（编码器） ---- */
        case SERVO_TEST_LOOP_EV:
            if (closed) {
                axis->rt.Ev_give = u;
                axis->run_loop(axis, SERVO_LOOP_EV_SPEED, 0); /* Ev→I */
            } else {
                axis->rt.I_give = u;
            }
            pobj->out_flag = cur_flag;
            axis->enable(axis);
            break;

        /* ---- 位置环（无开环动作）---- */
        case SERVO_TEST_LOOP_POSITION:
            if (closed) {
                axis->set_position_target(axis, u);           /* 写 P_give（含限位）*/
                axis->run_loop(axis, SERVO_LOOP_POSITION, 0); /* P→Ev→I */
                pobj->out_flag = cur_flag;
                axis->enable(axis);
            }
            break;

        /* ---- 测速 / 脱靶量跟踪环（用 rt.T_fb，忽略 amp/freq）---- */
        case SERVO_TEST_LOOP_TRACK:
            if (closed) {
                /* 跟踪环以脱靶量 T_fb 为输入（由调用者预先写入 axis->rt.T_fb），
                 * 故激励 amp/freq 在此被忽略，与原 Test() case 4 行为一致。 */
                axis->run_loop(axis, SERVO_LOOP_TRACK, 0); /* T→V→A→I */
                pobj->out_flag = cur_flag;
                axis->enable(axis);
            }
            break;

        /* ---- GEO 地理跟踪环（无开环动作，与原代码一致）---- */
        case SERVO_TEST_LOOP_GEO:
            if (closed) {
                axis->rt.GEO_give = u;
                axis->run_loop(axis, SERVO_LOOP_GEO, 0); /* GEO→V→A→I */
                pobj->out_flag = cur_flag;
                axis->enable(axis);
            }
            break;

        default:
            /* 非法 Loop：安全起见停机 */
            test_brake_all(pobj);
            pobj->running = false;
            pobj->out_flag = 0;
            return false;
    }

    return true;
}
