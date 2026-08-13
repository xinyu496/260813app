/**
 * @file    app.c
 * @brief   俯仰控制板应用层：图像板/方位板串口通信、伺服指令分发、
 *          光电校正 Flash 存取、编码器 SPI 采集及 1ms 周期任务调度。
 *
 * 串口分配：
 *   - USART1 (1843200)：与图像板通信，帧头 0xEB，累加和校验
 *   - USART2 (921600) ：与方位板通信，帧头 0xEB，累加和校验
 *
 * 控制链路说明：
 *   图像板指令经 CMD_Dataanalysis 同时触发
 *   ① commandprocess()（SF 旧链路，切换 PTstate）
 *   ② REC_CMD_FUN_Handle[]（servo_module 新链路）
 *
 * 图像板→俯仰指令码（rec_addr=0x10 时有效，见 SF.h）：
 *   0x01 刹车      0x02 速度运动    0x03 位置运动    0x04 地理引导
 *   0x05 点选跟踪  0x06 收藏        0x07 上锁        0x08 解锁
 *   0x09 光电校正  0x0A 步进运动
 *
 * 俯仰→图像板 error_state 故障位（本文件维护）：
 *   bit2：方位编码器故障（来自方位板 fw_bmp_state）
 *   bit3：SPI 俯仰编码器通信故障
 *
 * 1ms 周期任务（TIM6→ms→APP_Ctrl_System_Handle）：
 *   alldeal() → SEND_IMAGE_CMD_Handle() → SEND_CONTROL_CMD_Handle() → SPI 编码器读取
 */

#include "app.h"
#include "string.h"
#include "servo_module.h"   /* 新版伺服模块：servo_ext_cmd_t / servo_module_on_command */
#include "sf.h"             /* 旧版 SF 控制环：FWControl、PTstate、alldeal、commandprocess */
#include "GEO_Track_C.h"    /* 地理引导：GeoLos_CalcAbsAngle */

#include "Bsp/bsp_flash.h"  /* 光电校正参数 Sector10 持久化 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * 串口帧通用格式（图像板/方位板）：
 *   byte[0] = 0xEB 帧头
 *   byte[1] = len   帧长（含校验字节）
 *   byte[2..len-2]  载荷
 *   byte[len-1]     校验 = sum(byte[0..len-2]) & 0xFF
 *
 * 图像板→俯仰 CMD_220_T（31B）：
 *   rec_addr=0x10 伺服指令；cmd=0x01~0x0A；data[20] 为参数区
 *
 * 俯仰→图像板 CMD_FY_IMAGE_T（56B）：
 *   含方位/俯仰角速度、故障/告警、载荷状态等
 *
 * 俯仰→方位板 CMD_FY_FW_T（14B）：
 *   motor_enable + 俯仰电流给定(mA) + 偏差角等
 */

/* -------------------------------------------------------------------------- */
/*                              全局缓冲与标志位                                */
/* -------------------------------------------------------------------------- */

/** USART1 接收 DMA 缓冲（图像板下行） */
uint8_t u1_rx_buff[256];
/** USART2 接收 DMA 缓冲（方位板上送） */
uint8_t u2_rx_buff[256];

/** 图像板→俯仰 解析后指令 */
REC_IMAGE_CMD_U rec_cmd_buffer;
/** 俯仰→图像板 发送帧 */
SEND_IMAGE_CMD_U send_image_cmd_buffer;
/** 方位板→俯仰 解析后数据 */
REC_FW_CMD_U rec_fw_fy_buff;
/** 俯仰→方位板 控制帧 */
SEND_FW_CMD_U send_fy_fw_buff;

/** SPI1 编码器原始接收缓冲（6 字节） */
__IO uint8_t SPI1_rxbuf[6];

/** TIM6 1ms 节拍计数，满 1 触发周期任务 */
uint8_t ms;
/** 串口帧解析完成标志：1=有新帧待处理 */
uint8_t u1_rec_flag, u2_rec_flag;
/** ADC DMA 采集完成标志 */
uint8_t dma_get_flag;

/** 俯仰/方位偏差角缓存（0.01°），当前文件内未使用，保留 */
static int16_t s_fw_dev_angle_cdeg;
static int16_t s_gd_dev_angle_cdeg;

/** 引导指令接收计数（调试） */
uint32_t fh_debug;

/** ADC1 DMA 采样缓冲：1 通道 × 10 次 */
__IO uint16_t ADC1_CovertedValue[ADC1_CHANEL_NUM * ADC1_COLLECT_NUM] = {0};

/** 编码器解析值及角度（调试/显示用） */
uint32_t WHGBMQ_Data, WHGBMQ_Data1, WHGBMQ_Data2;
float angle_real;

/* -------------------------------------------------------------------------- */
/*                         光电校正 Flash 持久化结构                            */
/* -------------------------------------------------------------------------- */

#pragma pack(push, 1)
/** Sector 10 光电校正参数，magic='COLB' + CRC16-CCITT */
typedef struct {
    uint32_t magic;
    int32_t  lon;      /**< 经度，LSB=0.00001° */
    int32_t  lat;      /**< 纬度，LSB=0.00001° */
    int16_t  alt;      /**< 海拔，m */
    int16_t  roll;     /**< 横滚补偿，0.01° */
    int32_t  fw_comp;  /**< 方位补偿，0.01° */
    int32_t  gd_comp;  /**< 俯仰补偿，0.01° */
    uint16_t crc;
} USER_Ctrl_OptCalibFlash_t;
#pragma pack(pop)

static USER_Ctrl_OptCalibFlash_t s_opt_calib_flash;

/* -------------------------------------------------------------------------- */
/*                              内部函数声明                                    */
/* -------------------------------------------------------------------------- */

static void CMD_Dataanalysis(CMD_220_T *cmd_struct);

/** 图像板伺服指令处理，cmd 码见 SF.h（0x01~0x0A） */
static void CMD_REC_Brake(uint8_t *data);
static void CMD_REC_Speed_Move(uint8_t *data);
static void CMD_REC_PLACE_Move(uint8_t *data);
static void CMD_REC_Guide(uint8_t *data);
static void CMD_REC_Trace(uint8_t *data);
static void CMD_REC_Collect(uint8_t *data);
static void CMD_REC_Lock(uint8_t *data);
static void CMD_REC_UNLock(uint8_t *data);
static void CMD_REC_Revise(uint8_t *data);
static void CMD_REC_Stepmove(uint8_t *data);

/**
 * @brief 图像板伺服指令分发表
 * @note  下标与 cmd 字段对应：0=无效，1=刹车 … 10=步进
 */
REC_CMD_Fun REC_CMD_FUN_Handle[11] = {
    NULL,               /* 0x00 无效 */
    CMD_REC_Brake,        /* 0x01 刹车 */
    CMD_REC_Speed_Move,   /* 0x02 速度运动 */
    CMD_REC_PLACE_Move,   /* 0x03 位置运动 */
    CMD_REC_Guide,        /* 0x04 地理引导 */
    CMD_REC_Trace,        /* 0x05 点选跟踪 */
    CMD_REC_Collect,      /* 0x06 收藏 */
    CMD_REC_Lock,         /* 0x07 上锁 */
    CMD_REC_UNLock,       /* 0x08 解锁 */
    CMD_REC_Revise,       /* 0x09 光电校正 */
    CMD_REC_Stepmove,     /* 0x0A 步进运动 */
};

/* -------------------------------------------------------------------------- */
/*                    图像板伺服指令处理（servo_module 链路）                  */
/* -------------------------------------------------------------------------- */

/** @brief 0x01 刹车：下发 SERVO_EXT_CMD_BRAKE */
static void CMD_REC_Brake(uint8_t *data)
{
    (void)data;
    servo_ext_cmd_t cmd = {0};
    cmd.cmd = SERVO_EXT_CMD_BRAKE;  /* 立即停止方位/俯仰运动 */
    servo_module_on_command(&cmd);

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive brake cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/**
 * @brief 0x02 速度运动
 * @param data [0:1] 方位速度 Int16，[2:3] 俯仰速度 Int16（小端，单位见协议）
 */
static void CMD_REC_Speed_Move(uint8_t *data)
{
    /* 小端 Int16 解析方位/俯仰速度给定 */
    int16_t fw_speed = (int16_t)((data[1] << 8) | data[0]);
    int16_t fy_speed = (int16_t)((data[3] << 8) | data[2]);

    /* 封装为 servo_module 外部速度指令 */
    servo_ext_cmd_t cmd = {0};
    cmd.cmd = SERVO_EXT_CMD_VELOCITY;
    cmd.p.velocity.fw = fw_speed;
    cmd.p.velocity.gd = fy_speed;
    servo_module_on_command(&cmd);

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive speed move g_servo_ext_cmd_debug\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/**
 * @brief 0x03 位置运动
 * @param data [0:1] 方位角，[2:3] 俯仰角（Int16 小端）
 * @note  servo_module 下发已注释；位置环仍由 commandprocess→alldeal 旧链路执行
 */
static void CMD_REC_PLACE_Move(uint8_t *data)
{
    /* 小端 Int16 解析方位/俯仰角度给定（单位见协议） */
    int16_t fw_angle = (int16_t)((data[1] << 8) | data[0]);
    int16_t fy_angle = (int16_t)((data[3] << 8) | data[2]);

    /* 组装位置指令；实际执行走 SF 旧链路（PTstate=P_ALead） */
    servo_ext_cmd_t cmd = {0};
    cmd.cmd = SERVO_EXT_CMD_POSITION;
    cmd.p.position.fw = fw_angle;
    cmd.p.position.gd = fy_angle;

//    servo_module_on_command(&cmd);

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive place move cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/* -------------------------------------------------------------------------- */
/*                         地理引导（GEO）相关变量                              */
/* -------------------------------------------------------------------------- */

/** 本机/目标地理坐标及视线角输出 */
GeoPoint_t lxy_self, lxy_target;
GeoLosAngle_t lxy_out;

/** 计算本机→目标的绝对方位/俯仰视线角，结果写入 lxy_out */
void GEO(void)
{
    GeoLos_CalcAbsAngle(&lxy_self, &lxy_target, &lxy_out);
}

/** 遗留 SF 结构体，当前文件未直接使用 */
INSTypeDef INS_ = {0};
PTangleTypeDef PT_ = {0};
OrientLoadTypeDef OrientLoad_ = {0};

/** 默认 INS 坐标（调试/仿真用） */
double INS_longitude = 103.908940683563, INS_lattitude = 30.823423234, Targetheight = 1000;

/** 角度制 → 弧度 */
double deg__(double deg)
{
    return deg * M_PI / 180.0;
}

double jing, wei;
uint32_t time_flag = 0;
/** 引导时方位脱靶补偿角（°） */
float yaw_lxy_miss = 0;

/**
 * @brief 0x04 地理引导
 * @param data [0:3]  目标经度 Int32，LSB=0.00001°
 *             [4:7]  目标纬度 Int32，LSB=0.00001°
 *             [8:11] 目标高度 Int32，m
 *             [12]   引导标志（当前未参与控制）
 * @note  更新 lxy_target 后调用 GEO()；PTstate 由 commandprocess 切至 P_GEOLead
 */
static void CMD_REC_Guide(uint8_t *data)
{
    /* 从 param 区解析目标经纬高（大端 Int32 / Int32 / Int32） */
    double longtitude = (double)(((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]) * 0.00001f);
    double latitude   = (double)(((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]) * 0.00001f);
    float  high       = (float)((data[11] << 24) | (data[10] << 16) | (data[9] << 8) | data[8]);
    uint8_t flag      = data[12];  /* 引导类型标志，暂未使用 */

    (void)flag;
    fh_debug++;
    jing = longtitude;  /* 调试变量：最新目标经度 */
    wei  = latitude;    /* 调试变量：最新目标纬度 */

    /* 写入 GEO 解算目标点 */
    lxy_target.alt_m   = high;
    lxy_target.lon_deg = (float)longtitude;
    lxy_target.lat_deg = (float)latitude;

    /* 方位反馈角归一化到 [0, 360) 供引导解算参考 */
    if (FWControl.P_fb < 0) {
        yaw_lxy_miss = FWControl.P_fb + 360;
    } else {
        yaw_lxy_miss = FWControl.P_fb;
    }

    GEO();

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive guide cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/** @brief 0x05 点选跟踪：脱靶量来自 rec_cmd_buffer 帧内字段 */
static void CMD_REC_Trace(uint8_t *data)
{
    (void)data;
    /* 脱靶量在 CMD_220_T 帧尾字段，单位 0.01° */
    int16_t fw_miss = rec_cmd_buffer.rec_cmd_struct.fw_miss_distance;
    int16_t fy_miss = rec_cmd_buffer.rec_cmd_struct.fy_miss_distance;
    uint8_t is_track = rec_cmd_buffer.rec_cmd_struct.miss_distance_enable;

    /* 有效时 is_track=1，脱靶量参与跟踪环 */
    servo_ext_cmd_t cmd = {0};
    cmd.cmd = SERVO_EXT_CMD_TRACK;
    cmd.p.track.fw_miss = fw_miss * 0.01f;
    cmd.p.track.gd_miss = fy_miss * 0.01f;
    cmd.p.track.is_track = is_track;
    servo_module_on_command(&cmd);

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive trace cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/** @brief 0x06 收藏（收拢）：转台收至收藏位 */
static void CMD_REC_Collect(uint8_t *data)
{
    (void)data;
    servo_ext_cmd_t cmd = {0};
    cmd.cmd = SERVO_EXT_CMD_STOW;
    servo_module_on_command(&cmd);

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive collect cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/**
 * @brief 0x07 上锁
 * @note  此处无具体动作；PTstate 由 commandprocess 切至 P_LOCK
 */
static void CMD_REC_Lock(uint8_t *data)
{
    (void)data;
#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive lock cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/**
 * @brief 0x08 解锁
 * @note  此处无具体动作；PTstate 由 commandprocess 切至 P_DELOCK
 */
static void CMD_REC_UNLock(uint8_t *data)
{
    (void)data;
#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive unlock cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/** @brief 将偏差角限幅到协议范围 ±1000（±10.00°） */
static int16_t USER_Ctrl_ClampDevAngleCdeg(int32_t cdeg)
{
    if (cdeg < GD_TO_FW_DEV_ANGLE_MIN) {
        return (int16_t)GD_TO_FW_DEV_ANGLE_MIN;
    }
    if (cdeg > GD_TO_FW_DEV_ANGLE_MAX) {
        return (int16_t)GD_TO_FW_DEV_ANGLE_MAX;
    }
    return (int16_t)cdeg;
}

/** @brief 将 s_opt_calib_flash 写入 Flash Sector 10 */
static void USER_Ctrl_OptCalibFlashSave(void)
{
    uint8_t *buf = (uint8_t *)&s_opt_calib_flash;

    /* 写入前刷新 magic 与 CRC，再擦除扇区并编程 */
    s_opt_calib_flash.magic = USER_CTRL_OPT_CALIB_FLASH_MAGIC;
    s_opt_calib_flash.crc   = UTL_CRC16_CCITT(buf, USER_CTRL_OPT_CALIB_FLASH_CRC_LEN);
    Flash_Erase_Sector(USER_CTRL_OPT_CALIB_FLASH_ADDR);
    Flash_WriteNoErase(USER_CTRL_OPT_CALIB_FLASH_ADDR, buf, USER_CTRL_OPT_CALIB_FLASH_SIZE);
}

/**
 * @brief 0x09 光电校正
 * @param data  param1~4  经度 Int32，LSB=0.00001°，无效值 40000000
 *              param5~8  纬度 Int32，无效值 40000000
 *              param9~10 海拔 Int16，m
 *              param11~12 横滚 0.01°
 *              param13~16 方位补偿 0.01°
 *              param17~20 俯仰补偿 0.01°
 * @note  校验通过后更新 INS 与 Flash；PC11 拉低关闭 H 桥使能
 */
static void CMD_REC_Revise(uint8_t *data)
{
    /* 校正期间关闭驱动 H 桥（PC11=低） */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

    int32_t lon;
    int32_t lat;
    int16_t alt;
    float   roll;
    int16_t roll16;
    int32_t fw_comp;
    int32_t gd_comp;
    uint8_t updated = 0U;

    /* 按 param 顺序解包各字段（小端） */
    lon     = (int32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
    lat     = (int32_t)((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);
    roll16  = (int16_t)((data[11] << 8) | data[10]);
    alt     = (int32_t)((data[9] << 8) | data[8]);
    roll    = (float)(roll16 * 0.01);
    fw_comp = (float)(((data[15] << 24) | (data[14] << 16) | (data[13] << 8) | data[12]) * 0.01);
    gd_comp = (float)(((data[19] << 24) | (data[18] << 16) | (data[17] << 8) | data[16]) * 0.01);

    /* 逐项校验，任一失败则放弃整帧校正 */
    if (lon != 40000000 && lon >= -18000000 && lon <= 18000000) {
        s_opt_calib_flash.lon = lon;
        updated = 1U;
    } else {
        return;
    }
    /* 纬度有效性：非 40000000 且在 ±90° 内 */
    if (lat != 40000000 && lat >= -9000000 && lat <= 9000000) {
        s_opt_calib_flash.lat = lat;
        updated = 1U;
    } else {
        return;
    }
    if (alt >= -1000 && alt <= 30000) {
        s_opt_calib_flash.alt  = alt;
        s_opt_calib_flash.roll = roll16;
        updated = 1U;
    } else {
        return;
    }
    /* 横滚/俯仰补偿范围检查（原逻辑保留） */
    if ((roll > 180) && (roll < -180)) {
        return;
    }
    if ((gd_comp > 180) && (gd_comp < -180)) {
        return;
    }

    (void)updated;

    /* 经纬度 LSB=0.00001° → 弧度制 INS；补偿角直接写入 INS 姿态 */
    INS.height    = alt;
    INS.lattitude = deg__((float)lat / 100000);
    INS.longitude = deg__((float)lon / 100000);
    INS.Yaw       = (float)fw_comp;
    INS.Roll      = (float)roll;
    INS.Pitch     = (float)gd_comp;

    s_opt_calib_flash.roll    = roll16;
    s_opt_calib_flash.fw_comp = fw_comp;
    s_opt_calib_flash.gd_comp = gd_comp;

    /* 方位补偿角归一化到 [0, 360) */
    if (INS.Yaw >= 360) {
        INS.Yaw -= 360;
    }

    /* 持久化到 Flash，下次上电由 OptCalibFlashLoad 恢复 */
    USER_Ctrl_OptCalibFlashSave();

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive revise cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/**
 * @brief 0x0A 步进运动
 * @param data [0] 方位方向 0x01=左/0x02=右
 *             [1:2] 方位步长，0.01°
 *             [3] 俯仰方向 0x01=上/0x02=下
 *             [4:5] 俯仰步长，0.01°（注：当前实现低字节取自 data[3]）
 */
static void CMD_REC_Stepmove(uint8_t *data)
{
    /* 方位：方向 + 步长（0.01°） */
    uint8_t  fw_dir      = data[0];
    uint16_t fw_step_len = (uint16_t)((data[2] << 8) | data[1]);
    /* 俯仰：方向 + 步长；注意 fy_step_len 低字节与 fy_dir 共用 data[3] */
    uint8_t  fy_dir      = data[3];
    uint16_t fy_step_len = (uint16_t)((data[4] << 8) | data[3]);

    servo_ext_cmd_t cmd = {0};
    cmd.cmd = SERVO_EXT_CMD_STEP;
    cmd.p.step.fw_dir  = fw_dir;
    cmd.p.step.fw_step = fw_step_len * 0.01f;   /* 换算为 ° */
    cmd.p.step.gd_dir  = fy_dir;
    cmd.p.step.gd_step = fy_step_len * 0.01f;
    servo_module_on_command(&cmd);

#if DEBUG_UART_DRIVER
    SEGGER_RTT_SetTerminal(2);
    SEGGER_RTT_printf(0, "receive step cmd\n");
    SEGGER_RTT_SetTerminal(0);
#endif
}

/* -------------------------------------------------------------------------- */
/*                         图像板指令解析与收发                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief 解析图像板伺服指令并分发
 * @note  仅处理 rec_addr==REC_ADDR_SF(0x10)；
 *        先 commandprocess 切 SF 工作模式，再调 REC_CMD_FUN_Handle
 */
static void CMD_Dataanalysis(CMD_220_T *cmd_struct)
{
    CMD_220_T *rec_cmd = cmd_struct;
    if (rec_cmd == NULL) {
        return;
    }

    /* 仅伺服地址 0x10 的指令才进入本板处理 */
    if (rec_cmd->rec_addr != REC_ADDR_SF) {
        return;
    }
    /* cmd 有效范围 1~10 */
    if ((rec_cmd->cmd > 10) || (rec_cmd->cmd == 0)) {
        return;
    }

    /* SF 旧链路：切换 PTstate，供 alldeal 控制环使用 */
    commandprocess();
    /* servo_module 新链路：按 cmd 码调用具体处理函数 */
    REC_CMD_FUN_Handle[rec_cmd->cmd](rec_cmd->data);
}

/** @brief 字节累加和校验（取低 8/16 位由调用方决定） */
static uint16_t UTL_ADD_CHECK(uint8_t *data, uint16_t length)
{
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

/** @brief 帧累加和校验：sum(byte[0..len-2]) == byte[len-1] */
static uint8_t UartFrameChecksumOk(const uint8_t *buf, uint8_t len)
{
    if (len < 2U) {
        return 0U;
    }
    return ((uint8_t)UTL_ADD_CHECK((uint8_t *)buf, (uint16_t)(len - 1U)) == buf[len - 1U]) ? 1U : 0U;
}
uint16_t u1_cnt0, u1_cnt1;  /* 图像板收帧统计（调试） */
uint8_t  update_u1, update_u2, error_u1;
/** @brief 处理 USART1 新到图像板指令 */
static void RECEIVE_IMAGE_CMD_Handle(void)
{
    if (u1_rec_flag == 1) {
        u1_rec_flag = 0;
        uint8_t flen = rec_cmd_buffer.rec_cmd_struct.len;
        if (UartFrameChecksumOk(rec_cmd_buffer.cmd, flen)) {
            CMD_Dataanalysis(&rec_cmd_buffer.rec_cmd_struct);
            u1_cnt0++;
            if (u1_cnt0 == 200U) {
                u1_cnt0 = 0U;
                u1_cnt1++;
            }
        } else {
            error_u1++;
        }
    }
}

/**
 * @brief 处理 USART2 方位板上送帧
 * @note  根据 fw_bmp_state 更新发往图像板的 error_state bit2（方位编码器故障）
 */
static void RECEIVE_FW_CMD_Handle(void)
{
    if (u2_rec_flag == 1) {
        u2_rec_flag = 0;
        uint8_t flen = rec_fw_fy_buff.rec_cmd_struct.len;
        if (!UartFrameChecksumOk(rec_fw_fy_buff.cmd, flen)) {
            return;
        }
        /* fw_bmp_state=1 表示方位编码器异常，置位上报图像板的故障 bit2 */
        if (rec_fw_fy_buff.rec_cmd_struct.fw_bmp_state == 1) {
            send_image_cmd_buffer.send_cmd_struct.error_state |= 1U << 2;
        } else {
            send_image_cmd_buffer.send_cmd_struct.error_state &= ~(1U << 2);
        }
    }
}

#define APP_UART_TX_RECOVER_MS  5U

void APP_Uart_TxCompletePoll(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance == NULL)) {
        return;
    }
    if (huart->gState != HAL_UART_STATE_BUSY_TX) {
        return;
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) &&
        (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC) != RESET)) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
        huart->TxXferCount = 0U;
        huart->gState = HAL_UART_STATE_READY;
    }
}

void APP_Uart_ClearHwErrors(UART_HandleTypeDef *huart)
{
    uint32_t sr;

    if ((huart == NULL) || (huart->Instance == NULL)) {
        return;
    }

    /* 仅在有错误位时清标志；无条件读 DR 会丢弃正在 DMA 接收的字节 */
    sr = READ_REG(huart->Instance->SR);
    if ((sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) == 0U) {
        return;
    }

    if ((sr & USART_SR_PE) != 0U) {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    if ((sr & USART_SR_FE) != 0U) {
        __HAL_UART_CLEAR_FEFLAG(huart);
    }
    if ((sr & USART_SR_NE) != 0U) {
        __HAL_UART_CLEAR_NEFLAG(huart);
    }
    if ((sr & USART_SR_ORE) != 0U) {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
}

static void APP_Uart_TxHardRecover(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance == NULL)) {
        return;
    }

    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
    __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

    if (huart->hdmatx != NULL) {
        if (huart->hdmatx->State == HAL_DMA_STATE_BUSY) {
            (void)HAL_DMA_Abort(huart->hdmatx);
        }
        if (huart->hdmatx->Lock == HAL_LOCKED) {
            __HAL_UNLOCK(huart->hdmatx);
        }
        huart->hdmatx->State = HAL_DMA_STATE_READY;
    }

    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
    huart->TxXferCount = 0U;
    huart->gState = HAL_UART_STATE_READY;
}

static void APP_Uart_TxRecoverIfStuck(UART_HandleTypeDef *huart, uint32_t *busy_tick)
{
    APP_Uart_TxCompletePoll(huart);
    if (huart->gState != HAL_UART_STATE_BUSY_TX) {
        *busy_tick = 0U;
        return;
    }

    if (*busy_tick == 0U) {
        *busy_tick = HAL_GetTick();
        return;
    }

    if ((HAL_GetTick() - *busy_tick) >= APP_UART_TX_RECOVER_MS) {
        APP_Uart_TxHardRecover(huart);
        *busy_tick = 0U;
    }
}

HAL_StatusTypeDef APP_Uart_RxDmaRestart(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
    if ((huart == NULL) || (buf == NULL) || (len == 0U)) {
        return HAL_ERROR;
    }

    huart->RxState = HAL_UART_STATE_READY;
    APP_Uart_ClearHwErrors(huart);

    if (HAL_UART_Receive_DMA(huart, buf, len) != HAL_OK) {
        return HAL_ERROR;
    }

    SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
    return HAL_OK;
}

/**
 * @brief 组帧并通过 USART1 DMA 发送俯仰→图像板状态（56 字节）
 * @note  1ms 任务中周期调用；方位角叠加 INS.Yaw 北向补偿
 */
static void SEND_IMAGE_CMD_Handle(void)
{
#if 1
    /*
     * 填充俯仰→图像板实时状态：
     *   fw_angle / fy_angle  单位 0.01°
     *   fw_speed / fy_speed  单位 0.01°/s（与 SF 反馈一致 ×100）
     * 方位角需叠加 INS.Yaw（北向/光电校正补偿）并归一化到 36000
     */
    if (FWControl.P_fb < 0) {
        /* 负反馈角先转到 [0,360) 再加补偿 */
        send_image_cmd_buffer.send_cmd_struct.fw_angle =
            (int16_t)((FWControl.P_fb + 360) * 100);
        send_image_cmd_buffer.send_cmd_struct.fw_angle += (int16_t)(INS.Yaw * 100);
        if (send_image_cmd_buffer.send_cmd_struct.fw_angle > 36000) {
            send_image_cmd_buffer.send_cmd_struct.fw_angle -= 36000;
        }
    } else {
        send_image_cmd_buffer.send_cmd_struct.fw_angle =
            (int16_t)(FWControl.P_fb * 100);
        send_image_cmd_buffer.send_cmd_struct.fw_angle += (int16_t)(INS.Yaw * 100);
        if (send_image_cmd_buffer.send_cmd_struct.fw_angle > 36000) {
            send_image_cmd_buffer.send_cmd_struct.fw_angle -= 36000;
        }
    }

    /* 俯仰角/角速度直接取 GD 轴反馈 */
    send_image_cmd_buffer.send_cmd_struct.fy_angle = (int16_t)(GDControl.P_fb * 100);
    send_image_cmd_buffer.send_cmd_struct.fw_speed = FWControl.Ev_fb * 100;
    send_image_cmd_buffer.send_cmd_struct.fy_speed = GDControl.Ev_fb * 100;

    /* warning_state：bit0=俯仰到位，bit1=方位到位 */
    uint8_t warning_state =
        (g_servo.fw_axis.status.in_place << 1) | g_servo.gd_axis.status.in_place;
    send_image_cmd_buffer.send_cmd_struct.warning_state = warning_state;

    /* 目标经纬高字段当前填 0，由图像板侧自行维护时可扩展 */
    send_image_cmd_buffer.send_cmd_struct.target_longitude = 0x00;
    send_image_cmd_buffer.send_cmd_struct.target_latitude  = 0x00;
    send_image_cmd_buffer.send_cmd_struct.target_high      = 0x00;
#endif

    /* 固定帧头/帧长，累加和覆盖前 len-1 字节 */
    send_image_cmd_buffer.send_cmd_struct.head = 0xeb;
    send_image_cmd_buffer.send_cmd_struct.len  = 56;

    static uint8_t cnt = 0;
    cnt++;
    (void)cnt;

    uint8_t val = 0;
    for (int i = 0; i < send_image_cmd_buffer.send_cmd_struct.len - 1; i++) {
        val += send_image_cmd_buffer.cmd[i];
    }

    send_image_cmd_buffer.send_cmd_struct.check = val;

    static uint32_t u1_tx_busy_tick;
    APP_Uart_TxRecoverIfStuck(&huart1, &u1_tx_busy_tick);
    if (huart1.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit_DMA(&huart1, send_image_cmd_buffer.cmd,
                              send_image_cmd_buffer.send_cmd_struct.len);
    }
}

extern uint8_t FW_EN;

/**
 * @brief 组帧并通过 USART2 DMA 发送俯仰→方位控制（14 字节）
 * @note  含电机使能、俯仰电流给定（mA）
 */
static void SEND_CONTROL_CMD_Handle(void)
{
    /* 帧头/长度固定为 0xEB / 14 */
    send_fy_fw_buff.send_cmd_struct.head         = 0xeb;
    send_fy_fw_buff.send_cmd_struct.len          = 14;
    send_fy_fw_buff.send_cmd_struct.motor_enable  = FW_EN;
    /* 俯仰电流给定：A → mA 上报给方位驱动板 */
    send_fy_fw_buff.send_cmd_struct.current      = FWControl.I_give * 1000;

    /* 前 len-1 字节累加和作为校验字节 */
    uint8_t check_val = (uint8_t)UTL_ADD_CHECK(send_fy_fw_buff.cmd,
                                               send_fy_fw_buff.send_cmd_struct.len - 1);
    send_fy_fw_buff.send_cmd_struct.check = check_val;

    static uint32_t u2_tx_busy_tick;
    APP_Uart_TxRecoverIfStuck(&huart2, &u2_tx_busy_tick);
    if (huart2.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit_DMA(&huart2, send_fy_fw_buff.cmd,
                              send_fy_fw_buff.send_cmd_struct.len);
    }
}

/* -------------------------------------------------------------------------- */
/*                              LED / Flash / ADC                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief 运行指示灯 PC5 闪烁
 * @note  上电 5s 内 100ms 快闪，之后 500ms 慢闪
 */
void USER_Ctrl_LedFun(void)
{
    static uint32_t led_tick;

    /* 上电 5s 后切换为慢闪，便于区分启动阶段与正常运行 */
    if (HAL_GetTick() >= 5000) {
        if (HAL_GetTick() >= led_tick + 500) {
            led_tick = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
        }
    } else {
        if (HAL_GetTick() >= led_tick + 100) {
            led_tick = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
        }
    }
}

/** @brief 从 Flash 加载校正参数并应用到 INS / lxy_self */
static void USER_Ctrl_OptCalibFlashApply(void)
{
    /* 地理引导本机点：度制经纬 + 海拔米 */
    lxy_self.alt_m   = (float)s_opt_calib_flash.alt;
    lxy_self.lat_deg = (float)s_opt_calib_flash.lat / 100000;
    lxy_self.lon_deg = (float)s_opt_calib_flash.lon / 100000;

    /* SF 全局 INS：经纬转弧度，姿态/comp 直接赋值 */
    INS.height    = s_opt_calib_flash.alt;
    INS.lattitude = deg__((float)s_opt_calib_flash.lat / 100000);
    INS.longitude = deg__((float)s_opt_calib_flash.lon / 100000);
    INS.Yaw       = (float)s_opt_calib_flash.fw_comp;
    INS.Roll      = (float)s_opt_calib_flash.roll;
    INS.Pitch     = (float)s_opt_calib_flash.gd_comp;
}

/** @brief 上电从 Flash Sector 10 读取光电校正参数 */
static void USER_Ctrl_OptCalibFlashLoad(void)
{
    USER_Ctrl_OptCalibFlash_t tmp;
    uint16_t crc;

    Flash_Read(USER_CTRL_OPT_CALIB_FLASH_ADDR, (uint8_t *)&tmp,
               USER_CTRL_OPT_CALIB_FLASH_SIZE);
    /* magic 不匹配视为未校正过 */
    if (tmp.magic != USER_CTRL_OPT_CALIB_FLASH_MAGIC) {
        memset(&s_opt_calib_flash, 0, sizeof(s_opt_calib_flash));
        return;
    }

    /* CRC 校验失败则丢弃，防止 Flash 损坏数据 */
    crc = UTL_CRC16_CCITT((uint8_t *)&tmp, USER_CTRL_OPT_CALIB_FLASH_CRC_LEN);
    if (crc != tmp.crc) {
        memset(&s_opt_calib_flash, 0, sizeof(s_opt_calib_flash));
        return;
    }

    s_opt_calib_flash = tmp;
    USER_Ctrl_OptCalibFlashApply();
}

/* -------------------------------------------------------------------------- */
/*                           系统初始化与主循环                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief 系统外设与应用模块初始化
 * @note  串口 DMA+IDLE 接收、TIM6 1ms、SPI 编码器、ADC、伺服、Flash 校正加载
 */
void APP_Ctrl_System_Init(void)
{
    /* USART1：图像板，DMA 环形接收 + IDLE 帧边界检测 */
    HAL_UART_Receive_DMA(&huart1, u1_rx_buff, sizeof(u1_rx_buff));
    SET_BIT((&huart1)->Instance->CR1, USART_CR1_IDLEIE);

    /* USART2：方位板，同上 */
    HAL_UART_Receive_DMA(&huart2, u2_rx_buff, sizeof(u2_rx_buff));
    SET_BIT((&huart2)->Instance->CR1, USART_CR1_IDLEIE);

    /* TIM6 提供 1ms 时基，驱动 ms 计数与周期任务 */
    HAL_TIM_Base_Start_IT(&htim6);

    /* SPI1 编码器接口使能 */
    __HAL_SPI_ENABLE(&hspi1);

    /* ADC1 DMA 连续采集（电流/电压等） */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_CovertedValue,
                      ADC1_CHANEL_NUM * ADC1_COLLECT_NUM);

    servo_module_init();
    USER_Ctrl_OptCalibFlashLoad();  /* 恢复上次光电校正参数 */
}

/** @brief 查询 ADC DMA 是否采集完成 */
HAL_StatusTypeDef GET_ADC_STATE_Flag(void)
{
    if (dma_get_flag == 1) {
        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

/** @brief 置位 ADC 采集完成标志（DMA 回调中调用） */
void SET_ADC_STATE_Flag(void)
{
    dma_get_flag = 1;
}

/** @brief 清除 ADC 采集完成标志 */
void CLEAR_ADC_STATE_Flag(void)
{
    dma_get_flag = 0;
}

/** @brief 重启 ADC DMA 连续采集 */
void ADC_restart(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_CovertedValue,
                      ADC1_CHANEL_NUM * ADC1_COLLECT_NUM);
}

/** @brief ADC 采集完成处理：重启 DMA 并翻转 PC0 调试脚 */
static void ADC_GET_Handle(void)
{
    if (GET_ADC_STATE_Flag() == HAL_OK) {
        CLEAR_ADC_STATE_Flag();
        ADC_restart();                          /* 立即重启下一轮 DMA 采集 */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);  /* PC0 翻转，示波器观测采样节拍 */
    }
}

/** 调试开关：置 1 触发一次 CMD_REC_Guide */
uint8_t debug_, debug_1;
/** SF 伺服工作模式状态字，定义见 SF.h P_xxx */
extern uint8_t PTstate;

/**
 * @brief 主循环周期任务（由 main while(1) 调用）
 * @note  含 LED、ADC、串口收帧、1ms 节拍下的 SF 控制环与编码器 SPI 采集
 */
void APP_Ctrl_System_Handle(void)
{
    USER_Ctrl_LedFun();              /* 运行指示灯 */
    ADC_GET_Handle();                /* ADC 采集完成处理 */
    RECEIVE_IMAGE_CMD_Handle();      /* 图像板指令解析（事件驱动） */
    RECEIVE_FW_CMD_Handle();         /* 方位板状态更新（事件驱动） */

    /* 调试：手动触发引导 */
    if (debug_) {
        debug_ = 0;
        CMD_REC_Guide(0);
    }
    /* 调试：直接进入地理引导工作模式 */
    if (debug_1) {
        debug_1 = 0;
        PTstate = P_GEOLead;
    }

    /* 1ms 节拍任务：SF 控制环 + 双串口发送 + 编码器 SPI 读取 */
    if (ms >= 1) {
        alldeal();                   /* SF 伺服控制主循环（SF_20240116.c） */
        ms = 0;
        SEND_IMAGE_CMD_Handle();     /* 俯仰→图像板 56B 状态帧 */
        SEND_CONTROL_CMD_Handle();   /* 俯仰→方位板 14B 电流/使能帧 */

        /* SPI 编码器 DMA 读取，失败置 error_state bit3 */
        if (hspi1.State == HAL_SPI_STATE_READY) {
            HAL_StatusTypeDef FY_State = HAL_SPI_Receive_DMA(&hspi1, SPI1_rxbuf, 6);
            if (FY_State != HAL_OK) {
                send_image_cmd_buffer.send_cmd_struct.error_state |= 1U << 3;
            } else {
                send_image_cmd_buffer.send_cmd_struct.error_state &= ~(1U << 3);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                         编码器 SPI 数据处理                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 解析 SPI 编码器 21bit 绝对值
 * @note  取前 3 字节高 21 位（>>2），量程 0~0x1FFFFF
 */
void BmqData_Dispose(void)
{
    static uint32_t WHGBMQ_Data_temp;
    /* 21bit 绝对式编码器：3 字节有效数据，低 2 bit 为状态位需右移丢弃 */
    WHGBMQ_Data_temp = ((uint32_t)((SPI1_rxbuf[0] << 16) | (SPI1_rxbuf[1] << 8) |
                                   SPI1_rxbuf[2]) >> 2) &
                       0x1fffff;
    WHGBMQ_Data = WHGBMQ_Data_temp;
}

/** @brief SPI1 接收完成回调，解析俯仰编码器数据 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        BmqData_Dispose();
    }
}

/* -------------------------------------------------------------------------- */
/*                         串口 IDLE+DMA 接收回调                               */
/* -------------------------------------------------------------------------- */




/**
 * @brief UART 空闲中断/DMA 收完回调
 * @note  帧头 0xEB，长度 byte[1]，末字节为累加和；
 *        USART1→图像指令，USART2→方位数据
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* ---------- USART1：图像板下行帧（中断内只做拷贝，校验放主循环） ---------- */
    if (huart == &huart1) {
        uint8_t flen = u1_rx_buff[1];
        if ((u1_rx_buff[0] == 0xeb) && (flen >= 2U) && (flen <= sizeof(u1_rx_buff))) {
            memcpy(rec_cmd_buffer.cmd, u1_rx_buff, flen);
            u1_rec_flag = 1;
            update_u1++;
        }
        if (APP_Uart_RxDmaRestart(&huart1, u1_rx_buff, (uint16_t)sizeof(u1_rx_buff)) != HAL_OK) {
            (void)APP_Uart_RxDmaRestart(&huart1, u1_rx_buff, (uint16_t)sizeof(u1_rx_buff));
        }
        return;
    }

    /* ---------- USART2：方位板上送帧 ---------- */
    if (huart == &huart2) {
        uint8_t flen = u2_rx_buff[1];
        if ((u2_rx_buff[0] == 0xeb) && (flen >= 2U) && (flen <= sizeof(u2_rx_buff))) {
            memcpy(rec_fw_fy_buff.cmd, u2_rx_buff, flen);
            u2_rec_flag = 1;
            update_u2++;
        }
        if (APP_Uart_RxDmaRestart(&huart2, u2_rx_buff, (uint16_t)sizeof(u2_rx_buff)) != HAL_OK) {
            (void)APP_Uart_RxDmaRestart(&huart2, u2_rx_buff, (uint16_t)sizeof(u2_rx_buff));
        }
    }
}

/** @brief TIM6 溢出回调，1ms 递增 ms 计数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) {
        ms++;
    }
}
