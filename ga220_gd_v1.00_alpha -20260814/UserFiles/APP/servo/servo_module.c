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
#include "app_ctrl.h"
#include <math.h>
#include "servo_tune.h"

/* ========================================================================
 *                  手持转台，传感器数据正负实测情况：
 *                  方位：顺时针旋转，angle 增大，ev为 +
 *                        逆时针旋转，angle 减小，ev为 -
 *                  俯仰：低头，angle 减小，ev为 -    
 *                        抬头，angle 增大，ev为 +
 * 
 *                  脱靶量正负：
 * ====================================================================== */

// extern volatile uint32_t bmq_orgin_val; /* FW BMQ */
// extern __IO uint16_t ADC1_CovertedValue[];
// extern MOTOR_TO_FW_U rec_motor_fw_buff; /**< 驱动→方位接收缓冲 */

static void servo_module_test_spec_set(void);
static void servo_module_test_tick_1ms(void);

static void servo_tune_init(void);
static void servo_tune_apply_period(void);

static void servo_lock(bool lock)
{
    if (lock) {
        TIM1->CCR3 = 1000;
        TIM1->CCR4 = 0;
    } else {
        TIM1->CCR3 = 0;
        TIM1->CCR4 = 1000;
    }
}

/* ========================================================================
 *   电动推杆锁：外部通信指令控制开/关，动作持续时间可配（非阻塞状态机）
 * ====================================================================== */

/* 通信指令码（按需与协议对齐） */
#define LOCK_CMD_LOCK   0x01u        /* 上锁：推杆伸出 */
#define LOCK_CMD_UNLOCK 0x02u        /* 开锁：推杆缩回 */

/* 推杆动作默认持续时间(ms)：驱动到位所需时间，按调用周期=1ms 计 */
#define LOCK_ACTION_MS_DEFAULT 2000u

typedef enum {
    LOCK_STATE_IDLE = 0,             /* 空闲：推杆已停止 */
    LOCK_STATE_LOCKING,              /* 上锁动作进行中 */
    LOCK_STATE_UNLOCKING,            /* 开锁动作进行中 */
} lock_state_e;

static struct {
    lock_state_e state;              /* 当前动作状态 */
    uint16_t     timer_ms;           /* 剩余动作时间(ms)，递减到 0 即停 */
} s_lock = { LOCK_STATE_IDLE, 0u };

/* 停止推杆：两路 PWM 都清零，撤除供电 */
static void servo_lock_stop(void)
{
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
}

/**
 * @brief 外部通信指令控制推杆锁开/关。
 * @param cmd        通信指令码：LOCK_CMD_LOCK=上锁 / LOCK_CMD_UNLOCK=开锁
 * @param action_ms  本次开关动作持续时间(ms)；传 0 则用 LOCK_ACTION_MS_DEFAULT
 * @return true=指令已受理并启动；false=非法指令码
 *
 * 非阻塞：仅给出方向输出并设定计时，推杆到时停止由 servo_lock_tick_1ms() 完成。
 * 重复下发会以最新指令为准（重设方向与计时）。
 */
bool servo_lock_on_command(uint8_t cmd, uint16_t action_ms)
{
    bool lock;

    switch (cmd) {
        case LOCK_CMD_LOCK:   lock = true;  break;
        case LOCK_CMD_UNLOCK: lock = false; break;
        default:              return false; /* 非法指令码 */
    }

    s_lock.timer_ms = (action_ms != 0u) ? action_ms : LOCK_ACTION_MS_DEFAULT;
    s_lock.state    = lock ? LOCK_STATE_LOCKING : LOCK_STATE_UNLOCKING;
    servo_lock(lock);                /* 立即给出方向输出 */
    return true;
}

/* 1ms 周期推进：动作计时到点后自动停止推杆 */
static void servo_lock_tick_1ms(void)
{
    if (s_lock.state == LOCK_STATE_IDLE) {
        return;
    }

    if (s_lock.timer_ms > 0u) {
        s_lock.timer_ms--;
    }

    if (s_lock.timer_ms == 0u) {
        servo_lock_stop();
        s_lock.state = LOCK_STATE_IDLE;
    }
}

// 俯仰电机：电阻 2.83~2.84 Ω （2835）， 电感 3.857~3.943 mH （3900），极对数 10
// 方位电机：电阻 2.48~2.49 Ω （2485）， 电感 4.091~4.121 mH （4100），极对数 17

/*
	0x00：电机关
	0x01：电机开，此时1、2字节为电流环输入值。只要该字节不为0x01，驱动器应关闭电机（寻零命令除外）
	0xF0：电机寻零，电机和编码器初次上电调试时需要发送寻零命令，寻零成功并将寻零结果写入EEPROM后。（寻零成功后，发送0xF1，将结果存储到EEPROM中）
	0xF1：将所有参数写入EEPROM（每次只发送一次命令，之后一直发送停机命令，写入EEPROM需要100ms，不要反复重复发送该命令）
	0xF2：设置电感：“字节7和字节8”*0.000001为电机线电感感值，单位为H，感值的范围为0~+0.065535。
	0xF3：读取：电机线电感感值。
	0XF4：设置电阻：“字节7和字节8”*0.001为电机线电阻阻值，单位为Ω，阻值的范围为0~+65.535。
	0xF5：读取：电机线电阻阻值。
	0xF6：设置电机极对数：将“字节7”*1设置为电机极对数。
	0xF7：读取：电机极对数设置值。
	0xF8：查询寻零结果。
	0xF9：设置饱和输出：“字节7和字节8”*0.001为饱和输出，数据范围为0~0.9。
	0xFA：读取饱和输出
	0xFB：设置对齐电压，“字节7和字节8”*0.001为对齐电压，数据范围为0~0.9。（一般不超过0.4，设置时应结合输入电机，驱动板过流能力，电机内阻大小决定，避免损伤驱动器和电机）
	0xFC：读取对齐电压
	0xFD：读取存储结果
 */

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
#if SERVO_FW_DRIVE_BRUSHED

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

#endif /* SERVO_FW_DRIVE_BRUSHED */

/* FW 电流在 ADC1 扫描序列中的通道序号(0..ADC1_CHANEL_NUM-1) */
#define FW_ADC_CH_IDX 0u

#if SERVO_FW_DRIVE_BRUSHED
/* 取 ADC1 多次采集的均值作为 FW 通道的原始码(0..4095)。 */
static uint16_t fw_get_adc_raw(void)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < ADC1_COLLECT_NUM; i++) {
        sum += ADC1_CovertedValue[i * ADC1_CHANEL_NUM + FW_ADC_CH_IDX];
    }
    return (uint16_t)(sum / ADC1_COLLECT_NUM);
}
#endif /* SERVO_FW_DRIVE_BRUSHED */

#if SERVO_GD_DRIVE_BRUSHED
/* 取 ADC1 多次采集的均值作为 GD 通道的原始码(0..4095)。 */
static uint16_t gd_get_adc_raw(void)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < ADC1_COLLECT_NUM; i++) {
        sum += ADC1_CovertedValue[i * ADC1_CHANEL_NUM + FW_ADC_CH_IDX];
    }
    return (uint16_t)(sum / ADC1_COLLECT_NUM);
}
#endif /* SERVO_GD_DRIVE_BRUSHED */

/* —— FW 轴 IO —— */
static int32_t fw_get_cur_mA(void)
{
#if SERVO_FW_DRIVE_BRUSHED

    uint16_t code = fw_get_adc_raw(); /* FW 电流通道读取, 0..4095 */
    return sc813_adc_to_mA(code);     /* 有刷：返回 ADC 换算后的"原始"电流(mA)。*/
#else                                 /* 无刷：驱动器总线回传 */
    /* 方位电流来源通信：方位->俯仰 UART 传入 */
    int16_t rev_i_mA = FwToGdCmd.field.fw_current;
    return rev_i_mA;
#endif
}

static uint32_t fw_get_enc_raw(void)
{
#if NODE_ROLE == NODE_ROLE_MASTER /* 俯仰作为主机，方位编码器数据来源通信 */

    uint32_t rcv_bmq_raw = FwToGdCmd.field.encoder_val;
    return rcv_bmq_raw;

#else /* 方位作为从机，直接读取本地编码器 */
    return (FWBMQ_Data & FW_BMQ_MASK) >> 1U;
#endif
}

static int32_t fw_get_gyro_raw(void)
{
#if NODE_ROLE == NODE_ROLE_MASTER /* 俯仰作为主机，方位陀螺仪数据来源通信 */
    return 0;
#else                             /* 方位作为从机，直接读取本地陀螺仪 */
    //    return fwgyro;
    return 0;
#endif
}

static void fw_set_cur_mA(int32_t mA, bool en)
{
    /* 方位作为从机，通过 UART 发至 电机驱动板 */
    // send_fw_motor_buff.field.ctrl_current = mA;
    // send_fw_motor_buff.field.cmd = en ? 1 : 0;

    USER_Ctrl_FwSetRunCmd(mA, en);
    //    GdToFwCmd.field.ctrl_current = mA;
    //    GdToFwCmd.field.motor_enable = en ? 1 : 0;
}
static void fw_hard_en(bool en)
{
#if NODE_ROLE == NODE_ROLE_SUB /* 俯仰作为主机，方位陀螺仪数据来源通信 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
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

#else                                 /* 无刷：驱动器总线回传 */
    /* TODO : 实现无刷电流读取，来源驱动板上传 */
    int16_t rev_i_mA = MdrvToGdCmd.field.current;
    return rev_i_mA;
#endif
}
static uint32_t gd_get_enc_raw(void)
{
    return (WHGBMQ_Data >> 1); /* ← 换成你的 GD 编码器数据读取 */
}
static int32_t gd_get_gyro_raw(void)
{
    //    return fygyro;
    return 0; /* ← 换成你的 GD 陀螺仪数据读取 */
}
static void gd_set_cur_mA(int32_t mA, bool en)
{
    /* TODO : 俯仰主控通过 UART 向驱动板发送 使能 + 电流命令 */
    USER_Ctrl_MdrvSetRunCmd(mA, en);
    // GdToMdrvCmd.field.ctrl_current = mA;
    // GdToMdrvCmd.field.motor_cmd = en ? 1 : 0;
}
static void gd_hard_en(bool en)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

/* 子节点:读主节点下行命令 */
static inline float link_cmd_current_A(void)
{
    return rec_fw_fy_buff.field.ctrl_current * 0.001f;
}
static inline bool link_cmd_enable(void)
{
    return rec_fw_fy_buff.field.motor_enable != 0 ? true : false;
}

/* 上行:把本地原码回传主节点 */
static void link_send_fb(void)
{
    dev_servo_axis_t *ax = g_servo.get_axis(&g_servo, LINK_AXIS);
    uint32_t enc = ax->sensor.bmq_raw;                      /* 编码器原码 */
    int16_t i_mA = (int16_t)(ax->sensor.current * 1000.0f); /* A -> mA */
    uint8_t enable = ax->status.enabled;                    /* 使能 */

    /* TODO: 组帧把 enc(+陀螺/电流)发 UART 给主节点 */

    send_fy_fw_buff.field.encoder_val = (uint32_t)enc;
    send_fy_fw_buff.field.fw_current = i_mA;
    send_fy_fw_buff.field.motor_sta = enable;
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
        .get_motor_current_mA = fw_get_cur_mA,  // 来源方位通信：UART 传入
        .get_encoder_raw = fw_get_enc_raw,      // 编码器原始值
        .get_gyro_raw = NULL,                   // 方位陀螺仪原始值，来源方位通信上传
        .set_motor_current_mA = fw_set_cur_mA,  // FW 电流环给定值通过通信发送
        .motor_hard_enable = fw_hard_en,        // 方位的使能：DJ_EN + HandBrake 组合
#if SERVO_FW_DRIVE_BRUSHED
        .set_motor_pwm = fw_set_pwm,            /* 有刷：PWM 输出钩子 fw_set_pwm, */
#else
        .set_motor_pwm = NULL, /* 无刷伺服驱动器：不用 PWM 钩子 */
#endif
    };
    /*  */
    const servo_axis_io_t gd_io = {
        .get_motor_current_mA = gd_get_cur_mA,  // 电流信号来源驱动板通信传入
        .get_encoder_raw = gd_get_enc_raw,      // 来源自身采集的编码器原始值
        .get_gyro_raw = NULL,                   // GD 陀螺仪原始值
        .set_motor_current_mA = gd_set_cur_mA,  // 无刷通信方式时才使用
        .motor_hard_enable = gd_hard_en,        // GD 电机硬使能 GPIO 钩子
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

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
        high_cnt = 0;
        if (low_cnt < HANDBRAKE_DEBOUNCE_MS)
            low_cnt++;
        if (low_cnt >= HANDBRAKE_DEBOUNCE_MS)
            engaged = true; /* 低电平 → 刹车 */
    } else {
        low_cnt = 0;
        if (high_cnt < HANDBRAKE_DEBOUNCE_MS)
            high_cnt++;
        if (high_cnt >= HANDBRAKE_DEBOUNCE_MS)
            engaged = false; /* 高电平 → 放行 */
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
    /* 推杆锁 1ms 推进：与伺服/手刹解耦，动作到时自动停止（放最前，避免被提前 return 跳过） */
    servo_lock_tick_1ms();

    /* 调试用：外部命令直接注入，正常控制时需外部指令触发，此处则需要屏蔽 */
    servo_module_on_command(&g_servo_ext_cmd_debug);

    /* —— 手刹最高优先级：按下(低电平)→ 强制刹车，并跳过正常流程 —— */
    if (handbrake_is_engaged()) {
        if (g_servo_test.is_running(&g_servo_test)) {
            g_servo_test.stop(&g_servo_test);         /* 同时停掉激励/开环测试 */
        }
        g_servo.set_mode(&g_servo, SERVO_MODE_BRAKE); /* 置 BRAKE：清环 + 断电机 */
        g_servo.update(&g_servo);                     /* 立即执行一次，让 BRAKE 生效 */

#if NODE_ROLE == NODE_ROLE_MASTER                     /* 俯仰作为主机，通过通信控制方位 */
        /* —— 同步失能方位(FW)轴：发送"电流=0 + 失能" —— */
        GdToFwCmd.field.ctrl_current = 0; /* 给定清零 */
        GdToFwCmd.field.motor_enable = 0; /* 失能 */
#endif
        return;
    }

#if NODE_ROLE == NODE_ROLE_SUB
#if 0
    /* 子节点:取下行(给定+使能)注入,并保持电流环模式 */
    //    g_servo.set_current(&g_servo, LINK_AXIS, i_A_dbg, eable_dbg);  // dbg调试使用
    g_servo.set_current(&g_servo, LINK_AXIS, link_cmd_current_A(), link_cmd_enable());

    g_servo.set_mode(&g_servo, SERVO_MODE_CURRENT);
#endif
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
        case SERVO_MODE_CURRENT:
        case SERVO_MODE_SELFCHECK:
            return true;
        default:
            return false;
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

/**
 * @brief 自检是否正在运行（用于屏蔽会改变运动目标的外部指令）。
 *        运行中定义为：当前待生效模式是 SELFCHECK，且自检步骤还没走到 DONE。
 */
static bool servo_module_selfcheck_is_active(void)
{
    return (g_servo.mode_pending == SERVO_MODE_SELFCHECK)
           && (g_servo.selfcheck_step != SERVO_SELFCHECK_STEP_DONE);
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
    if (dir == 0x01)
        return -mag; /* 左 */
    if (dir == 0x02)
        return +mag; /* 右 */
    return 0.0f;
}
static float gd_step_signed(uint8_t dir, float mag)
{
    if (dir == 0x01)
        return +mag; /* 上 */
    if (dir == 0x02)
        return -mag; /* 下 */
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

    /* —— 自检运行期间：屏蔽会改变运动目标的指令，仅刹车/自检指令可以通行。
     *    物理手刹优先级更高，走 servo_module_handler 里独立的 handbrake_is_engaged() 分支，不受此处影响。 —— */
    if (servo_module_selfcheck_is_active() && c->cmd != SERVO_EXT_CMD_BRAKE
        && c->cmd != SERVO_EXT_CMD_SELFCHECK) {
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
        case SERVO_EXT_CMD_STOW:
            servo_module_api_go_mode(SERVO_MODE_WITHDRAW);
            return true;

        /* —— 0x0B 自检 → SELFCHECK：每次收到都重新从"转上限位"开始 —— */
        case SERVO_EXT_CMD_SELFCHECK:
            g_servo.selfcheck_step = SERVO_SELFCHECK_STEP_TO_UP;
            servo_module_api_go_mode(SERVO_MODE_SELFCHECK);
            return true;

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

        default:
            return false; /* 非法指令码 */
    }
}
