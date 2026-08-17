/**
 * @file    servo_module.c
 * @brief   把 SF_20260303.c 中散落的全局变量与裸函数调用替换为对象 + 函数指针的方式。
 *
 *          ① 把硬件 IO 钩子接到 dev_servo_axis_t 上
 *          ② 用一行 servo.update(&servo) 替代 alldeal+commandprocess+allprocess
 */

#include "servo_controller.h"
#include "servo_test.h"
#include "servo_module.h"
#include "main.h"
#include "app.h"
//#include "Bsp/bsp_adc.h"
#include <math.h>
#include "servo_tune.h"
//#include "Bsp/bsp_timer.h"

/* ========================================================================
 *                  手持转台，传感器数据正负实测情况：
 *                  方位：顺时针旋转，angle 增大，ev为 +
 *                        逆时针旋转，angle 减小，ev为 -
 *                  俯仰：低头，angle 减小，ev为 -    
 *                        抬头，angle 增大，ev为 +
 * 
 *                  脱靶量正负：
 * ====================================================================== */

extern volatile uint32_t bmq_orgin_val; /* FW BMQ */
extern uint32_t WHGBMQ_Data;
extern __IO uint16_t ADC1_CovertedValue[];
extern REC_FW_CMD_U rec_fw_fy_buff;
extern SEND_FW_CMD_U send_fy_fw_buff;

static void servo_module_test_spec_set(void);
static void servo_module_test_tick_1ms(void);

static void servo_tune_init(void);
static void servo_tune_apply_period(void);

/* ========================================================================
 *   主/子节点角色配置:全工程唯一改这里(同一套源码,改宏即切角色)
 * ====================================================================== */
#define NODE_ROLE_MASTER 0
#define NODE_ROLE_SUB    1
#define NODE_ROLE        NODE_ROLE_MASTER /* ← 本板烧录角色 */
#define LINK_AXIS        SERVO_AXIS_FW    /* ← 主/子之间走链路的轴(方位=FW/俯仰=GD) */

/* ========================================================================
 *               ①  把现有全局/HAL 接口包装成 IO 钩子
 * ====================================================================== */

/* ===== SC813xFT-20B3：ADC 码 → 电流(mA) =====
 * B3 后缀：0A 输出 = 0.5·VCC；灵敏度 = 66 mV/A
 * 输出公式：VIOUT = 0.5·VCC + 电流 × 66mV/A
 */

#define ADC_TO_mV       0.80586080586f /* adc / 4095 *3300 */
#define SC813_SENS_mV_A 66.0f          /* 灵敏度 66 mV/A */
#define SC813_ZERO_mV   1650.0f        /* 0A 零点 = 0.5·VCC = 1650 mV */

/* ADC 原始码(0..4095) → 电流(mA)，含正负 */
static inline int32_t sc813_adc_to_mA(uint16_t adc_cnt)
{
    float vout_mV = (float)adc_cnt * ADC_TO_mV;            /* ① adc cnt → 电压(mV)  */
    float diff_mV = vout_mV - SC813_ZERO_mV;               /* ② 去 0A 零点(mV) */
    return (int32_t)(diff_mV / SC813_SENS_mV_A * 1000.0f); /* ③ ÷66 → A → mA   */
}

/* FW 电流在 ADC1 扫描序列中的通道序号(0..ADC1_CHANEL_NUM-1) */
#define FW_ADC_CH_IDX 0u

/* 取 ADC1 多次采集的均值作为 FW 通道的原始码(0..4095)。 */
static uint16_t fw_get_adc_raw(void)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < ADC1_COLLECT_NUM; i++) {
        sum += ADC1_CovertedValue[i * ADC1_CHANEL_NUM + FW_ADC_CH_IDX];
    }
    return (uint16_t)(sum / ADC1_COLLECT_NUM);
}

/* 取 ADC1 多次采集的均值作为 GD 通道的原始码(0..4095)。 */
static uint16_t gd_get_adc_raw(void)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < ADC1_COLLECT_NUM; i++) {
        sum += ADC1_CovertedValue[i * ADC1_CHANEL_NUM + FW_ADC_CH_IDX];
    }
    return (uint16_t)(sum / ADC1_COLLECT_NUM);
}

/* —— FW 轴 IO —— */
static int32_t fw_get_cur_mA(void)
{
#if SERVO_FW_DRIVE_BRUSHED

    uint16_t code = fw_get_adc_raw(); /* FW 电流通道读取, 0..4095 */
    return sc813_adc_to_mA(code);     /* 有刷：返回 ADC 换算后的"原始"电流(mA)。*/
#else                                 /* 无刷：驱动器总线回传 */

    int16_t rev_i_mA = rec_fw_fy_buff.rec_cmd_struct.fw_current;
    return rev_i_mA;
#endif
}
static uint32_t fw_get_enc_raw(void)
{
    uint32_t rcv_bmq_raw = rec_fw_fy_buff.rec_cmd_struct.bpm_cal;
    return rcv_bmq_raw;
}
static int32_t fw_get_gyro_raw(void)
{
    //    return fwgyro;
    return 0;
}
static void fw_set_cur_mA(int32_t mA, bool en)
{
    send_fy_fw_buff.send_cmd_struct.current = mA;
    send_fy_fw_buff.send_cmd_struct.motor_enable = en ? 1 : 0;
}
static void fw_hard_en(bool en)
{
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#if SERVO_FW_DRIVE_BRUSHED
/**
 * @brief 有刷 H 桥 PWM 输出钩子（参考实现，等价 SF_20240116::FW_PWM）。
 *
 *        仅在 SERVO_FW_DRIVE_BRUSHED=1 时编译启用；
 *        部署到有刷硬件，请按实际接线核对引脚/定时器通道：
 *          - PC11：H 桥使能脚（en）
 *          - PC12：方向脚（按 u_give 符号；u>0→RESET，u≤0→SET，与原 FW_PWM 一致）
 *          - TIM3->CCR1：PWM 占空比 = |u_give|，限幅 ±4200
 */
static void fw_set_pwm(float u_give, bool en)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, (u_give > 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if (u_give > 4200.0f)
        u_give = 4200.0f;
    else if (u_give < -4200.0f)
        u_give = -4200.0f;

    TIM3->CCR1 = (uint32_t)fabsf(u_give);
}
#endif /* SERVO_FW_DRIVE_BRUSHED */

/* —— GD 轴 IO —— */
static int32_t gd_get_cur_mA(void)
{
#if SERVO_GD_DRIVE_BRUSHED
    uint16_t code = gd_get_adc_raw(); /* ← 换成你的 GD 电流通道读取, 0..4095 */
    return sc813_adc_to_mA(code);     /* 有刷：返回 ADC 换算后的"原始"电流(mA)。*/
#else
    int16_t rev_i_mA = rec_fw_fy_buff.rec_cmd_struct.fw_current;
    return rev_i_mA; /* 无刷：驱动器总线回传 */
#endif
}
static uint32_t gd_get_enc_raw(void)
{
    return WHGBMQ_Data;
}
static int32_t gd_get_gyro_raw(void)
{
    //    return fygyro;
    return 0;
}
static void gd_set_cur_mA(int32_t mA, bool en)
{
    //    userFYTxMotorDrvPar.Current_Par = mA;
    //    userFYTxMotorDrvPar.Enable = en ? 1 : 0;
    //    userFYTxMotorDrvPar.Use_flg = en ? 1 : 0;
}
static void gd_hard_en(bool en)
{
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#if SERVO_GD_DRIVE_BRUSHED
/**
 * @brief 有刷 H 桥 PWM 输出钩子（GD 俯仰轴）。
 */
static void gd_set_pwm(float u_give, bool en)
{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, (u_give > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (u_give > 4200.0f)
        u_give = 4200.0f;
    else if (u_give < -4200.0f)
        u_give = -4200.0f;

    TIM3->CCR1 = (uint32_t)fabsf(u_give);
}
#endif /* SERVO_GD_DRIVE_BRUSHED */

/* ===================== 子节点 UART 链路 ===================== */
#if NODE_ROLE == NODE_ROLE_SUB /* NODE_ROLE_SUB */
/* 子节点:读主节点下行命令(沿用你已有的接收缓冲) */
static inline float link_cmd_current_A(void)
{
    return rec_fw_fy_buff.send_cmd_struct.current * 0.001f;
}
static inline bool link_cmd_enable(void)
{
    return rec_fw_fy_buff.send_cmd_struct.motor_enable != 0;
}

/* 上行:把本地原码回传主节点 */
static void link_send_fb(void)
{
    dev_servo_axis_t *ax = g_servo.get_axis(&g_servo, LINK_AXIS);
    uint32_t enc = ax->sensor.bmq_raw;                      /* 编码器原码 */
    int16_t i_mA = (int16_t)(ax->sensor.current * 1000.0f); /* A -> mA */
    uint8_t enable = ax->status.enabled;                    /* 使能 */

    /* TODO: 组帧把 enc(+陀螺/电流)发 UART 给主节点 */

    send_fy_fw_buff.rec_cmd_struct.bpm_val = (uint32_t)enc;
    send_fy_fw_buff.rec_cmd_struct.fw_current = i_mA;
    send_fy_fw_buff.rec_cmd_struct.motor_state = enable;
}
#endif

/* ========================================================================
 *               ②  顶层对象实例
 * ====================================================================== */

dev_servo_t g_servo;
dev_servo_test_t g_servo_test; /* 激励测试对象，同样开放便于观测 */

void servo_module_init(void)
{
    /* 无陀螺设备应把钩子直接置 NULL，dev_servo_axis_init 会自动识别为"仅编码器" */
    /* 装配 IO 钩子 */
    const servo_axis_io_t fw_io = {
        .get_motor_current_mA = fw_get_cur_mA,  // fw_get_cur_mA,
        .get_encoder_raw = fw_get_enc_raw,
        .get_gyro_raw = NULL,                   /* ← 无陀螺 */
        .set_motor_current_mA = fw_set_cur_mA,  // fw_set_cur_mA, ga220中，方位的电流环给定值通过通信发送
        .motor_hard_enable = NULL,              // fw_hard_en, ga220中，方位的使能通过通信发送，故置NULL
#if SERVO_FW_DRIVE_BRUSHED
        .set_motor_pwm = fw_set_pwm,            /* 有刷：PWM 输出钩子 fw_set_pwm, */
#else
        .set_motor_pwm = NULL, /* 无刷伺服驱动器：不用 PWM 钩子 */
#endif
    };
    const servo_axis_io_t gd_io = {
        .get_motor_current_mA = gd_get_cur_mA,  // gd_get_cur_mA,
        .get_encoder_raw = gd_get_enc_raw,      // gd_get_enc_raw,
        .get_gyro_raw = NULL,                   /* ← 无陀螺 */
        .set_motor_current_mA = NULL,           // 无刷通信方式时才使用：gd_set_cur_mA,
        .motor_hard_enable = gd_hard_en,        // gd_hard_en,
#if SERVO_GD_DRIVE_BRUSHED
        .set_motor_pwm = gd_set_pwm,            /* 有刷：PWM 输出钩子 gd_set_pwm, */
#else
        .set_motor_pwm = NULL, /* 无刷伺服驱动器：不用 PWM 钩子 */
#endif
    };

    /* 初始化伺服 */
    dev_servo_init(&g_servo, &fw_io, &gd_io);

    /* 初始化参数表，当前参数回填到表 */
    servo_tune_init();

    /* 初始化激励测试对象（绑定到上面的伺服对象） */
    dev_servo_test_init(&g_servo_test, &g_servo);
}

static void servo_module_tick_1ms(void)
{
    g_servo.update(&g_servo); /* 一行替代原 alldeal + commandprocess + allprocess */
}

/* ===== 手刹（HANDBREAK）外部按钮 ===== */
#define HANDBRAKE_DEBOUNCE_MS 20u          /* 消抖时间(ms)，按调用周期=1ms 计 */

volatile bool g_handbrake_engaged = false; /* 手刹状态（true=刹车生效） */

/* 读手刹（带消抖）：上电默认放行；如需“上电即安全”可把 engaged 初值改为 true。 */
static bool handbrake_is_engaged(void)
{
    static uint16_t low_cnt = 0, high_cnt = 0;
    static bool engaged = false;

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) {
        high_cnt = 0;
        if (low_cnt < HANDBRAKE_DEBOUNCE_MS) low_cnt++;
        if (low_cnt >= HANDBRAKE_DEBOUNCE_MS) engaged = true; /* 低电平 → 刹车 */
    } else {
        low_cnt = 0;
        if (high_cnt < HANDBRAKE_DEBOUNCE_MS) high_cnt++;
        if (high_cnt >= HANDBRAKE_DEBOUNCE_MS) engaged = false; /* 高电平 → 放行 */
    }

    g_handbrake_engaged = engaged;
    return engaged;
}

/* ========================================================================
 *               ③  1ms 周期入口（替代 SF_20260303.c::alldeal）
 * ====================================================================== */

volatile float i_A_dbg = 0;            /* 用于调试 */
volatile uint8_t eable_dbg = 0;        /* 用于调试 */
servo_ext_cmd_t g_servo_ext_cmd_debug; /* 用于调试 */

void servo_module_handler(void)
{
    /* 调试用：外部命令直接注入，正常控制时需要屏蔽 */
    // servo_module_on_command(&g_servo_ext_cmd_debug);

    /* —— 手刹最高优先级：按下(低电平)→ 强制刹车，并跳过正常流程 —— */
    if (handbrake_is_engaged()) {
        if (g_servo_test.is_running(&g_servo_test)) {
            g_servo_test.stop(&g_servo_test);         /* 同时停掉激励/开环测试 */
        }
        g_servo.set_mode(&g_servo, SERVO_MODE_BRAKE); /* 置 BRAKE：清环 + 断电机 */
        g_servo.update(&g_servo);                     /* 立即执行一次，让 BRAKE 生效 */

        /* —— 同步失能方位(FW)轴：发送"电流=0 + 失能" —— */
        send_fy_fw_buff.send_cmd_struct.current = 0;      /* 给定清零 */
        send_fy_fw_buff.send_cmd_struct.motor_enable = 0; /* 失能 */
        return;
    }

#if NODE_ROLE == NODE_ROLE_SUB
    /* 子节点:取下行(给定+使能)注入,并保持电流环模式 */
    g_servo.set_current(&g_servo, LINK_AXIS, i_A_dbg, eable_dbg);  // dbg调试使用

    // g_servo.set_current(&g_servo, LINK_AXIS, link_cmd_current_A(), link_cmd_enable());
    g_servo.set_mode(&g_servo, SERVO_MODE_CURRENT);
#endif

    servo_module_test_spec_set();

    servo_tune_apply_period();

    if (g_servo_test.is_running(&g_servo_test)) {
        servo_module_test_tick_1ms(); /* 测试入口 */
    } else {
        servo_module_tick_1ms();      /* 默认循环入口 */
    }

#if NODE_ROLE == NODE_ROLE_SUB
    link_send_fb(); /* 子节点:回传本地反馈给主节点 */
#endif
}

/* ========================================================================
 *               ⑤  激励测试入口（原 SF_20260303.c::Test）
 * ====================================================================== */

volatile servo_test_dbg_t g_test = {
    .axis = SERVO_AXIS_FW,
    .loop = SERVO_TEST_LOOP_CURRENT,
    .mode = SERVO_TEST_OPEN_LOOP,
    .ctrl = 0,
    .amp = 0.50f,
    .freq = 1.0f,
    .time_s = 10.0f,
    .wave = SERVO_TEST_WAVE_SINE, /* 激励波形类型设置 */
    .step_delay_ms = 5000u,       /* 阶跃前的等待时间 */
    .step_min = -1.0f,            /* 阶跃最小值 */
    .step_max = 1.0f,             /* 阶跃最大值 */
    .step_period_ms = 4000u,      /* 周期 4000ms：前 2000ms 输出 max，后 2000ms 输出 min */
};

/**
 * @brief 轮询"调试期可改参数"：按 g_test_ctrl 触发启动/停止，并实时同步幅频时长。
 *
 *        本函数由 servo_module_test_handler() 每周期调用
 */
static void servo_module_test_spec_set(void)
{
    /* ① 启动 / 停止：边沿命令（处理后清 0，避免每周期重复触发） */
    if (g_test.ctrl == 1) {
        servo_test_spec_t spec = {
            .axis = (servo_axis_id_e)g_test.axis,
            .loop = (servo_test_loop_e)g_test.loop,
            .mode = (servo_test_loop_mode_e)g_test.mode,
            .amp = g_test.amp,
            .freq = g_test.freq,
            .time_s = g_test.time_s,
            .step_delay_ms = g_test.step_delay_ms,
            .wave = g_test.wave,
            .step_min = g_test.step_min,
            .step_max = g_test.step_max,
            .step_period_ms = g_test.step_period_ms,
        };
        g_servo_test.start(&g_servo_test, &spec); /* 驱动调试 */
        g_test.ctrl = 0;
    } else if (g_test.ctrl == 2) {
        g_servo_test.stop(&g_servo_test);
        g_test.ctrl = 0;
    }

    /* ② 运行中实时同步 幅值/频率/时长（轴/环/开闭环切换需重新启动才安全）*/
    if (g_servo_test.is_running(&g_servo_test)) {
        g_servo_test.spec.amp = g_test.amp;
        g_servo_test.spec.freq = g_test.freq;
        g_servo_test.spec.time_s = g_test.time_s;
        g_servo_test.spec.step_delay_ms = g_test.step_delay_ms;
        g_servo_test.spec.wave = g_test.wave;
        g_servo_test.spec.step_min = g_test.step_min;
        g_servo_test.spec.step_max = g_test.step_max;
        g_servo_test.spec.step_period_ms = g_test.step_period_ms;
    }
}

/**
 * @brief 测试模式 1ms 入口：刷新传感器/反馈/解耦后推进一次激励步。
 *
 *        相当于"测试版"的 servo_module_handler——只做"采集 + 激励"，
 *        不走 servo_update 里的模式机。
 *        ⚠️：与 servo_module_handler 互斥使用。
 */
static void servo_module_test_tick_1ms(void)
{
    /* ① 刷新两轴传感器（编码器/陀螺/电流） */
    g_servo.fw_axis.update_sensor(&g_servo.fw_axis);
    g_servo.gd_axis.update_sensor(&g_servo.gd_axis);

    /* ② 同步控制环反馈 */
    g_servo.fw_axis.sync_feedback(&g_servo.fw_axis);
    g_servo.gd_axis.sync_feedback(&g_servo.gd_axis);

    /* ③ FW 受 GD 解耦因子 sec = 1/cos(GD 角)（与 servo_update 内一致） */
    float c = cosf(g_servo.gd_axis.sensor.angle * 0.017453293f);
    float sec = (fabsf(c) > 1e-6f) ? (1.0f / c) : 1.0f;
    g_servo.fw_axis.set_couple(&g_servo.fw_axis, sec);

    /* ④ 推进一次激励步（到时自动结束并刹车） */
    g_servo_test.tick(&g_servo_test);
}

/* ========================================================================
 *                       在线整定（调参）接口实现
 * ====================================================================== */

/* 每轴一张表；仿真时 Watch 窗口直接改这两个 */
servo_tune_t g_tune[SERVO_AXIS_MAX];

/* ① 开机一次：dev_servo_init() 之后调，用现值回填 */
static void servo_tune_init(void)
{
    servo_tune_sync(&g_tune[SERVO_AXIS_FW], &g_servo.fw_axis);
    servo_tune_sync(&g_tune[SERVO_AXIS_GD], &g_servo.gd_axis);
}

/* ② 1ms 主循环：先下发整定表，再跑控制环 */
static void servo_tune_apply_period(void)
{
    servo_tune_apply(&g_servo.fw_axis, &g_tune[SERVO_AXIS_FW]);
    servo_tune_apply(&g_servo.gd_axis, &g_tune[SERVO_AXIS_GD]);
}

static bool servo_mode_is_supported(servo_mode_e mode)
{
    switch (mode) {
        case SERVO_MODE_VOLTAGE:
        case SERVO_MODE_BRAKE:
        case SERVO_MODE_ENBRAKE:
        case SERVO_MODE_AFOLLOW:
        case SERVO_MODE_TRACK:
        case SERVO_MODE_VMOTION:
        case SERVO_MODE_ZERO:
        case SERVO_MODE_WITHDRAW:
        case SERVO_MODE_GEO:
        case SERVO_MODE_CURRENT: return true;
        default: return false;
    }
}

/* 设置模式，用于模式切换 */
static void servo_module_api_go_mode(servo_mode_e mode)
{
    /* 先判断合法模式 */
    if (!servo_mode_is_supported(mode)) {
        return;
    }
    g_servo.set_mode(&g_servo, mode);
}

/* ========================================================================
 *               ⑥  外部控制指令状态机（指令来源：主控/通信层）
 *
 *   外部指令码 → 内部 servo_mode_e 的翻译（二者数值不一致，必须翻译）：
 *     0x01 刹车  → BRAKE(待机) / ENBRAKE(速度0控制)，由参数[0] 选择
 *     0x02 速度  → VMOTION
 *     0x03 位置  → AFOLLOW
 *     0x04 引导  → GEO 未接：仅缓存经纬高 + 占位 BRAKE
 *     0x05 点跟  → TRACK（track_flag=1）
 *     0x06 收藏  → WITHDRAW（收藏）
 *     0x0A 步进  → AFOLLOW（当前机械角 + 步长）
 * ====================================================================== */

/* 刹车(0x01)具体行为：
 *   false → BRAKE  （待机：清环 + 断电机）
 *   true  → ENBRAKE（速度0控制：速度环目标置 0、电机保持使能/软刹） */
static bool s_brake_zero_speed = false;

/* 步进方向 → 步长符号（基于协议约定） */
static float fw_step_signed(uint8_t dir, float mag)
{
    if (dir == 0x01) return -mag; /* 左 */
    if (dir == 0x02) return +mag; /* 右 */
    return 0.0f;
}
static float gd_step_signed(uint8_t dir, float mag)
{
    if (dir == 0x01) return +mag; /* 上 */
    if (dir == 0x02) return -mag; /* 下 */
    return 0.0f;
}

/* ========================================================================
 *                   对外 API，外部调用接口进行控制
 * ====================================================================== */

bool servo_module_on_command(servo_ext_cmd_t *c)  //const
{
    if (c == NULL) {
        return false;
    }

    switch (c->cmd) {

        /* —— 0x01 刹车：待机/软刹由内部 s_brake_zero_speed 决定，外部指令无参数 —— */
        case SERVO_EXT_CMD_BRAKE:
            servo_module_api_go_mode(s_brake_zero_speed ? SERVO_MODE_ENBRAKE : SERVO_MODE_BRAKE);
            return true;

        /* —— 0x02 速度控制（°/s） → VMOTION —— */
        case SERVO_EXT_CMD_VELOCITY:
            g_servo.set_velocity(&g_servo, c->p.velocity.fw, c->p.velocity.gd); /* 直接写目标，无模式门限 */
            servo_module_api_go_mode(SERVO_MODE_VMOTION);
            return true;

        /* —— 0x03 位置运动（°） → AFOLLOW —— */
        case SERVO_EXT_CMD_POSITION:
            g_servo.set_position(&g_servo, c->p.position.fw, c->p.position.gd); /* 无 AFOLLOW 门限 */
            servo_module_api_go_mode(SERVO_MODE_AFOLLOW);
            return true;

        /* —— 0x04 引导模式：缓存参数 + 下发引导目标 → GEO。
         *      经纬高→目标角换算在 controller::mode_geo_handler 内以 TODO 占位。 —— */
        case SERVO_EXT_CMD_GUIDE:
            g_servo.set_geo(&g_servo, c->p.guide.lon, c->p.guide.lat, c->p.guide.alt, c->p.guide.flag);
            servo_module_api_go_mode(SERVO_MODE_GEO);
            return true;

        /* —— 0x05 点选跟踪（°） → TRACK —— */
        case SERVO_EXT_CMD_TRACK:
            if (c->p.track.is_track == 1U) {
                /* 脱靶量有效：更新脱靶量，进入脱靶量跟踪 */
                g_servo.set_miss(&g_servo, c->p.track.fw_miss, c->p.track.gd_miss); /* 内部已 ±5° 限幅 */
                g_servo.cmd.track_flag = 1u; /* 启动跟踪（无独立 API，直写 cmd） */
            } else {
                /* 脱靶量无效：清脱靶量，TRACK 内部 track_flag=0 时执行零速保持 */
                g_servo.set_miss(&g_servo, 0.0f, 0.0f);
                g_servo.cmd.track_flag = 0u;
            }
            servo_module_api_go_mode(SERVO_MODE_TRACK);
            return true;

        /* —— 0x06 收藏 → WITHDRAW —— */
        case SERVO_EXT_CMD_STOW: servo_module_api_go_mode(SERVO_MODE_WITHDRAW); return true;

        /* —— 0x0A 步进（步长 °）：相对当前机械角叠加增量， → AFOLLOW —— */
        case SERVO_EXT_CMD_STEP: {
            float fw_d = fw_step_signed(c->p.step.fw_dir, c->p.step.fw_step);
            float gd_d = gd_step_signed(c->p.step.gd_dir, c->p.step.gd_step);

            /* 基准 = 当前机械角度 */
            float fw_tgt = g_servo.fw_axis.sensor.angle + fw_d;
            float gd_tgt = g_servo.gd_axis.sensor.angle + gd_d;

            g_servo.set_position(&g_servo, fw_tgt, gd_tgt);
            servo_module_api_go_mode(SERVO_MODE_AFOLLOW);
            return true;
        }

        default: return false; /* 非法指令码 */
    }
}
