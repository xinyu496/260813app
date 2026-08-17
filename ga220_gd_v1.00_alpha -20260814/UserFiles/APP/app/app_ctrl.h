/**
 * @file    app_ctrl.h
 * @brief   俯仰板（GD）应用层控制：内部通信协议帧定义与收发接口
 *
 * 协议依据：
 *   - AOES-AB220/GA270-2601 内部通信协议（图像/方位链路）
 *   - 陀螺协议.xlsx（陀螺串口上报）
 *   - 无刷直流力矩电机驱动器通讯协议_中科友成自研版_20240613.docx
 *
 * 通信链路：
 *   - 俯仰 ↔ 图像板（IMG）：USART，921600bps，8N1，累加和校验，小端
 *   - 俯仰 ↔ 方位板（FW） ：USART，921600bps，8N1，累加和校验，小端
 *   - 俯仰 ↔ 驱动板（MDRV）：RS422，460800bps，8N1，累加和校验，小端，1ms 周期
 *   - 陀螺 → 俯仰（GYRO）  ：USART6，定长 13 字节，异或校验，角速度大端 24bit
 *
 * 联合体命名：类型名全大写；成员名全小写；通过 .buf 访问原始字节，.field 访问字段。
 * 多字节整数端序：图像/方位链路为小端；陀螺角速度为按字节高→中→低的大端 24bit。
 */
#ifndef __APP_CTRL_H
#define __APP_CTRL_H

#include <stdint.h>
#include "Common/utl_check.h"
#include "Bsp/bsp_uart.h"

/* -------------------------------------------------------------------------- */
/*                         小端序读写（协议 wire 格式）                          */
/* -------------------------------------------------------------------------- */
#define USER_CTRL_LE_GET_U16(p) \
    ((uint16_t)((uint16_t)(p)[0] | ((uint16_t)(p)[1] << 8)))
#define USER_CTRL_LE_GET_S16(p) \
    ((int16_t)USER_CTRL_LE_GET_U16(p))
#define USER_CTRL_LE_GET_U32(p) \
    ((uint32_t)((uint32_t)(p)[0] | ((uint32_t)(p)[1] << 8) | \
                ((uint32_t)(p)[2] << 16) | ((uint32_t)(p)[3] << 24)))
#define USER_CTRL_LE_GET_S32(p) \
    ((int32_t)USER_CTRL_LE_GET_U32(p))

#define USER_CTRL_LE_PUT_U16(p, v)                         \
    do {                                                   \
        uint16_t _le_v = (uint16_t)(v);                    \
        (p)[0] = (uint8_t)(_le_v);                         \
        (p)[1] = (uint8_t)(_le_v >> 8);                    \
    } while (0)
#define USER_CTRL_LE_PUT_S16(p, v) \
    USER_CTRL_LE_PUT_U16(p, (uint16_t)(int16_t)(v))
#define USER_CTRL_LE_PUT_U32(p, v)                         \
    do {                                                   \
        uint32_t _le_v = (uint32_t)(v);                    \
        (p)[0] = (uint8_t)(_le_v);                         \
        (p)[1] = (uint8_t)(_le_v >> 8);                    \
        (p)[2] = (uint8_t)(_le_v >> 16);                   \
        (p)[3] = (uint8_t)(_le_v >> 24);                   \
    } while (0)
#define USER_CTRL_LE_PUT_S32(p, v) \
    USER_CTRL_LE_PUT_U32(p, (uint32_t)(int32_t)(v))

/* -------------------------------------------------------------------------- */
/*                              接收端地址定义                                  */
/* -------------------------------------------------------------------------- */
#define REC_ADDR_NULL    0x00  /**< 空指令 / 广播 */
#define REC_ADDR_SF      0x10  /**< 伺服板 */
#define REC_ADDR_IR      0x20  /**< 红外 */
#define REC_ADDR_VISIBLE 0x30  /**< 可见光 */

/* -------------------------------------------------------------------------- */
/*                              帧头 / 帧长定义                                 */
/* -------------------------------------------------------------------------- */
#define GD_IMG_FRAME_HEAD   0xEB  /**< 图像链路帧头 */
#define GD_FW_FRAME_HEAD    0xEB  /**< 方位链路帧头 */

#define GD_TO_IMG_FRAME_LEN 56U   /**< 俯仰→图像，整帧字节数 */
#define IMG_TO_GD_FRAME_LEN 31U   /**< 图像→俯仰，整帧字节数（含故障码） */
#define GD_TO_FW_FRAME_LEN  14U   /**< 俯仰→方位，整帧字节数（含故障码） */
#define FW_TO_GD_FRAME_LEN  54U   /**< 方位→俯仰，整帧字节数 */

/** @brief 协议帧内长度字段取值（byte[1]） */
#define GD_TO_IMG_LEN_VAL   0x38U /**< 俯仰→图像：长度字段 0x38（56 字节） */
#define GD_TO_FW_LEN_VAL    0x0EU /**< 俯仰→方位：长度字段 0x0E */
#define IMG_TO_GD_LEN_VAL   0x1FU /**< 图像→俯仰：长度字段 0x1F */

/** @brief 参与累加和校验的字节数（不含 checksum 本身） */
#define GD_TO_IMG_CHECKSUM_LEN  (GD_TO_IMG_FRAME_LEN - 1U) /**< byte[0]~byte[53] */
#define GD_TO_FW_CHECKSUM_LEN   (GD_TO_FW_FRAME_LEN - 1U)  /**< byte[0]~byte[12] */
#define IMG_TO_GD_CHECKSUM_LEN  (IMG_TO_GD_FRAME_LEN - 1U) /**< byte[0]~byte[29] */

/** @brief 俯仰编码器 SPI1 DMA 接收字节数 */
#define SPI1_BMQ_RX_LEN           10U

/** @brief 俯仰 SPI 绝对编码器有效位数（单圈分辨率 2^25） */
#define SPI_BMQ_BIT_WIDTH         25U
#define SPI_BMQ_RAW_MASK          0x1FFFFFFU
#define SPI_BMQ_MAX_CODE          (1UL << SPI_BMQ_BIT_WIDTH)
#define SPI_BMQ_STATUS_SHIFT      2U   /**< 与 21 位帧相同：32bit 组装后右移 2 位取数 */
#define SPI_BMQ_ANGLE_PER_CODE    (360.0f / (float)SPI_BMQ_MAX_CODE)
/** @brief 25bit 编码器值缩放到驱动板 24bit  wire 格式（360°=2^24） */
#define SPI_BMQ_TO_MDRV_ENC24(raw) \
    (((uint32_t)(raw) & SPI_BMQ_RAW_MASK) >> (SPI_BMQ_BIT_WIDTH - 24U))

/* -------------------------------------------------------------------------- */
/*                              串口映射                                        */
/* -------------------------------------------------------------------------- */
#define COM_GD_IMG  COM_USART_5//COM_USART_5  /**< 与图像板通信（USART1） */
#define COM_GD_FW   COM_USART_2  /**< 与方位板通信（USART2）   */

/* -------------------------------------------------------------------------- */
/*           图像→俯仰：伺服控制指令码（recv_addr = REC_ADDR_SF）               */
/* -------------------------------------------------------------------------- */
#define IMG_SF_CMD_BRAKE       0x01U /**< 刹车 */
#define IMG_SF_CMD_SPEED_MOVE  0x02U /**< 速度运动：param1/2 方位速度，param3/4 俯仰速度（Int16） */
#define IMG_SF_CMD_POS_MOVE    0x03U /**< 位置运动：param1/2 方位角度，param3/4 俯仰角度 */
#define IMG_SF_CMD_GUIDE       0x04U /**< 引导：param1~12 经纬高，param13 引导标志 */
#define IMG_SF_CMD_POINT_TRACK 0x05U /**< 点选跟踪 */
#define IMG_SF_CMD_STOW        0x06U /**< 收藏 */
#define IMG_SF_CMD_LOCK        0x07U /**< 上锁 */
#define IMG_SF_CMD_UNLOCK      0x08U /**< 解锁 */
#define IMG_SF_CMD_OPT_CALIB   0x09U /**< 光电校正：param1~12 经纬高，param13~20 方位/俯仰补偿 */
#define IMG_SF_CMD_STEP_MOVE   0x0AU /**< 步进运动：param1 方位方向，param2/3 方位步长，param4/5 俯仰方向/步长 */

/* ========================================================================== */
/*  图像 → 俯仰（下行）                                                        */
/*  周期 1Hz；图像板下发伺服 / 可见光 / 红外控制及脱靶量                          */
/* ========================================================================== */

/**
 * @brief 图像→俯仰下行帧
 *
 * ctrl_cmd 为 REC_ADDR_SF(0x10) 时为伺服控制指令（0x01~0x0A 等）；
 * 为 REC_ADDR_VISIBLE / REC_ADDR_IR 时为载荷控制指令，param1~param20 含义见协议。
 * 脱靶量仅在 miss_valid=1 时有效；byte[29] 为故障码。
 */
#pragma pack(push, 1)
typedef union {
    uint8_t buf[IMG_TO_GD_FRAME_LEN]; /**< 原始字节缓冲，用于串口收发 */
    struct {
        uint8_t  head;       /**< [0]    帧头，固定 0xEB */
        uint8_t  len;        /**< [1]    帧长，固定 0x1F（31 字节） */
        uint8_t  recv_addr;  /**< [2]    接收端：0x00空 / 0x10伺服 / 0x20红外 / 0x30可见 */
        uint8_t  ctrl_cmd;   /**< [3]    控制指令码 */
        uint8_t  param1;     /**< [4]    参数 1 */
        uint8_t  param2;     /**< [5]    参数 2 */
        uint8_t  param3;     /**< [6]    参数 3 */
        uint8_t  param4;     /**< [7]    参数 4 */
        uint8_t  param5;     /**< [8]    参数 5 */
        uint8_t  param6;     /**< [9]    参数 6 */
        uint8_t  param7;     /**< [10]   参数 7 */
        uint8_t  param8;     /**< [11]   参数 8 */
        uint8_t  param9;     /**< [12]   参数 9 */
        uint8_t  param10;    /**< [13]   参数 10 */
        uint8_t  param11;    /**< [14]   参数 11 */
        uint8_t  param12;    /**< [15]   参数 12 */
        uint8_t  param13;    /**< [16]   参数 13 */
        uint8_t  param14;    /**< [17]   参数 14 */
        uint8_t  param15;    /**< [18]   参数 15 */
        uint8_t  param16;    /**< [19]   参数 16 */
        uint8_t  param17;    /**< [20]   参数 17 */
        uint8_t  param18;    /**< [21]   参数 18 */
        uint8_t  param19;    /**< [22]   参数 19 */
        uint8_t  param20;    /**< [23]   参数 20 */
        int16_t  fw_miss;    /**< [24-25] 方位脱靶量，小端 */
        int16_t  gd_miss;    /**< [26-27] 俯仰脱靶量，小端 */
        uint8_t  miss_valid; /**< [28]   脱靶量有效：0 无效，1 有效 */
        uint8_t  fault_code; /**< [29]   故障码 */
        uint8_t  checksum;   /**< [30]   校验和：byte[0]~byte[29] 累加和低 8 位 */
    } field;
} IMG_TO_GD_U;
#pragma pack(pop)

/**
 * @brief 图像板下发的伺服控制指令处理函数类型
 * @param cmd  指向最新接收帧 ImgToGdCmd，只读；param1~param20 含义随 ctrl_cmd 变化
 */
typedef void (*USER_Ctrl_ImgSfCmdHandler_t)(const IMG_TO_GD_U *cmd);

/**
 * @brief 伺服控制指令分发表条目：指令码 + 处理函数指针
 */
typedef struct {
    uint8_t                     cmd;     /**< 控制指令码，见 IMG_SF_CMD_xxx */
    USER_Ctrl_ImgSfCmdHandler_t handler; /**< 对应处理函数，NULL 表示不处理 */
} USER_Ctrl_ImgSfCmdEntry_t;

/* ========================================================================== */
/*  俯仰 → 图像（上行）                                                        */
/*  周期 1kHz；俯仰板上报伺服角、目标经纬高、载荷状态等                          */
/* ========================================================================== */

/** @brief 俯仰→图像：故障状态字节（bit=1 表示故障） */
#pragma pack(push, 1)
typedef struct {
    uint8_t ir_fault         : 1; /**< bit0 红外故障 */
    uint8_t vis_fault        : 1; /**< bit1 可见光故障 */
    uint8_t fw_encoder_fault : 1; /**< bit2 方位编码器故障 */
    uint8_t gd_encoder_fault : 1; /**< bit3 俯仰编码器故障 */
    uint8_t gyro_fault       : 1; /**< bit4 陀螺故障（270） */
    uint8_t servo_comm_fault : 1; /**< bit5 与伺服通信故障（270） */
    uint8_t vis_video_fault  : 1; /**< bit6 可见光视频故障 */
    uint8_t ir_video_fault   : 1; /**< bit7 红外视频故障 */
} GD_TO_IMG_FAULT_T;
#pragma pack(pop)

/** @brief 俯仰→图像：告警状态字节（bit=1 表示故障） */
#pragma pack(push, 1)
typedef struct {
    uint8_t gd_motor_fault : 1; /**< bit0 俯仰电机故障 */
    uint8_t fw_motor_fault : 1; /**< bit1 方位电机故障 */
    uint8_t reserved       : 6; /**< bit2~7 保留 */
} GD_TO_IMG_ALARM_T;
#pragma pack(pop)

/** @brief 俯仰→图像：可见光状态字节，见协议表 1 */
#pragma pack(push, 1)
typedef struct {
    uint8_t comm_sta           : 1; /**< bit0 通信：1 正常，0 异常 */
    uint8_t focus_mode         : 1; /**< bit1 调焦：1 手动，0 自动 */
    uint8_t night_mode_setting : 1; /**< bit2 夜间模式设置：1 手动，0 自动 */
    uint8_t night_mode         : 1; /**< bit3 昼夜：1 夜间，0 白天 */
    uint8_t defog              : 1; /**< bit4 透雾：1 开，0 关 */
    uint8_t reserved           : 2; /**< bit5~6 保留 */
    uint8_t digital_zoom       : 1; /**< bit7 数字变倍：1 开，0 关 */
} GD_TO_IMG_VIS_STA_T;
#pragma pack(pop)

/** @brief 俯仰→图像：红外状态字节，见协议表 2 */
#pragma pack(push, 1)
typedef struct {
    uint8_t comm_sta       : 1; /**< bit0 通信：1 正常，0 异常 */
    uint8_t focus_mode     : 1; /**< bit1 调焦：1 手动，0 自动 */
    uint8_t image_polarity : 1; /**< bit2 极性：1 黑热，0 白热 */
    uint8_t ele_zoom       : 3; /**< bit3~5 电子变倍档位 */
    uint8_t crosshair      : 1; /**< bit6 十字光标：1 开，0 关 */
    uint8_t image_enhance  : 1; /**< bit7 图像增强：1 开，0 关（270） */
} GD_TO_IMG_IR_STA_T;
#pragma pack(pop)

/**
 * @brief 俯仰→图像上行帧
 *
 * 发送前填写 payload 字段，调用 USER_Ctrl_ImgCmdSend() 自动补帧头/长度/校验并发出。
 */
#pragma pack(push, 1)
typedef union {
    uint8_t buf[GD_TO_IMG_FRAME_LEN];
    struct {
        uint8_t             head;             /**< [0]     帧头，固定 0xEB */
        uint8_t             len;              /**< [1]     帧长，固定 0x38 */
        uint16_t            fw_angle;         /**< [2-3]   方位角，LSB=0.01°，小端 */
        int16_t             gd_angle;         /**< [4-5]   俯仰角，LSB=0.01°，小端 */
        int16_t             fw_speed;         /**< [6-7]   方位角速度，LSB=0.01°/s，小端 */
        int16_t             gd_speed;         /**< [8-9]   俯仰角速度，LSB=0.01°/s，小端 */
        int32_t             target_lon;       /**< [10-13] 目标经度，LSB=0.00001°，小端 */
        int32_t             target_lat;       /**< [14-17] 目标纬度，LSB=0.00001°，小端 */
        int32_t             target_alt;       /**< [18-21] 目标高度，m，小端 */
        uint16_t            time_year;        /**< [22-23] 年，小端 */
        uint8_t             time_month;       /**< [24]    月，1~12，无效 0xFF */
        uint8_t             time_day;         /**< [25]    日，1~31，无效 0xFF */
        uint8_t             time_hour;        /**< [26]    时，0~23，无效 0xFF */
        uint8_t             time_min;         /**< [27]    分，0~59，无效 0xFF */
        uint8_t             time_sec;         /**< [28]    秒，0~59，无效 0xFF */
        uint16_t            time_ms;          /**< [29-30] 毫秒，0~999，无效 0xFFFF */
        uint32_t            track_id;         /**< [31-34] 航迹号 */
        GD_TO_IMG_FAULT_T   fault_sta;        /**< [35]    故障状态 */
        GD_TO_IMG_ALARM_T   alarm_sta;        /**< [36]    告警状态 */
        uint16_t            vis_focal;        /**< [37-38] 可见光焦距，LSB=0.1mm（270） */
        GD_TO_IMG_VIS_STA_T vis_sta;          /**< [39]    可见光状态 */
        uint8_t             brightness;       /**< [40]    亮度 */
        uint8_t             contrast;         /**< [41]    对比度 */
        uint8_t             saturation;       /**< [42]    饱和度 */
        uint8_t             ele_zoom;         /**< [43]    电子变倍倍数 */
        uint16_t            ir_focal;         /**< [44-45] 红外焦距，LSB=0.1mm */
        GD_TO_IMG_IR_STA_T  ir_sta;           /**< [46]    红外状态 */
        uint8_t             ir_contrast;      /**< [47]    红外对比度 */
        uint8_t             ir_brightness;    /**< [48]    红外亮度 */
        uint8_t             integration_time; /**< [49]    积分时间 ms（270） */
        uint8_t             noise_filter;     /**< [50]    噪点滤波：1 开，0 关（270） */
        int16_t             north_fw;         /**< [51-52] 正北方位，LSB=0.01° */
        int16_t             center_gd;        /**< [53-54] 中心俯仰，LSB=0.01° */
        uint8_t             checksum;         /**< [55]    校验和：byte[0]~byte[53] 累加和低 8 位 */
    } field;
} GD_TO_IMG_U;
#pragma pack(pop)

/* ========================================================================== */
/*  俯仰 → 方位（1kHz）                                                        */
/*  俯仰板向方位板发送电流、偏差角、工作状态等                                    */
/* ========================================================================== */

/** @brief 俯仰→方位：伺服工作状态（byte10） */
#define GD_TO_FW_SERVO_STOP   0U /**< 停止 */
#define GD_TO_FW_SERVO_GUIDE  1U /**< 引导 */
#define GD_TO_FW_SERVO_LOCK   2U /**< 锁定（点选跟踪） */

/** @brief 俯仰→方位：偏差角 LSB（0.01°），协议有效范围 ±1000（±10°） */
#define GD_TO_FW_DEV_ANGLE_MIN  (-1000)
#define GD_TO_FW_DEV_ANGLE_MAX   1000

/** @brief 俯仰→方位：故障状态字节（byte11，0=正常，1=故障） */
#pragma pack(push, 1)
typedef struct {
    uint8_t ir_fault         : 1; /**< bit0 红外故障状态 */
    uint8_t vis_fault        : 1; /**< bit1 可见光故障状态 */
    uint8_t fw_encoder_fault : 1; /**< bit2 方位编码器故障状态 */
    uint8_t gd_encoder_fault : 1; /**< bit3 俯仰编码器故障状态 */
    uint8_t gyro_fault       : 1; /**< bit4 陀螺故障状态（仅A项目） */
    uint8_t servo_comm_fault : 1; /**< bit5 与伺服通信状态（仅A项目） */
    uint8_t vis_video_fault  : 1; /**< bit6 可见光视频状态 */
    uint8_t ir_video_fault   : 1; /**< bit7 红外视频状态 */
} GD_TO_FW_FAULT_T;
#pragma pack(pop)

/**
 * @brief 俯仰→方位上行帧
 */
#pragma pack(push, 1)
typedef union {
    uint8_t buf[GD_TO_FW_FRAME_LEN];
    struct {
        uint8_t            head;           /**< [0]    帧头，固定 0xEB */
        uint8_t            len;            /**< [1]    帧长，固定 0x0E（14 字节） */
        uint8_t            motor_enable;   /**< [2]    电机使能：0 关闭，1 使能 */
        int16_t            ctrl_current;   /**< [3-4]  控制电流，LSB=0.001 */
        uint8_t            lock_enable;    /**< [5]    电磁锁：0 关闭，1 使能（270） */
        int16_t            fw_dev_angle;   /**< [6-7]  方位偏差角，LSB=0.01°，±1000 */
        int16_t            gd_dev_angle;   /**< [8-9]  俯仰偏差角，LSB=0.01°，±1000 */
        uint8_t            servo_work_sta; /**< [10]   伺服状态：0 停止，1 引导，2 锁定 */
        GD_TO_FW_FAULT_T  fault_sta;       /**< [11]   故障状态：0 正常，1 故障 */
        uint8_t           fault_code;      /**< [12]   故障码 */
        uint8_t           checksum;        /**< [13]   校验和：byte[0]~byte[12] 累加和低 8 位 */
    } field;
} GD_TO_FW_U;
#pragma pack(pop)

/*  发送流程（1kHz）：                                                          */
/*    USER_Ctrl_FwSetRunCmd(mA, enable) / FwSetLockEnable / FwSetDevAngle 等     */
/*    → 1ms 任务 USER_Ctrl_FwCmdSend(NULL) 刷新 payload 并发出 14 字节          */

/* ========================================================================== */
/*  方位 → 俯仰（1kHz）                                                        */
/*  方位板向俯仰板发送编码器、引导信息、惯导/转台数据等                          */
/* ========================================================================== */

/**
 * @brief 方位→俯仰上行帧
 */
#pragma pack(push, 1)
typedef union {
    uint8_t buf[FW_TO_GD_FRAME_LEN];
    struct {
        uint8_t  head;               /**< [0]     帧头，固定 0xEB */
        uint8_t  len;                /**< [1]     帧长（协议字段，以实际报文为准） */
        uint32_t encoder_val;        /**< [2-5]   方位编码器值 */
        int16_t  fw_current;         /**< [6-7]   方位电流，LSB=0.001 */
        uint8_t  motor_sta;          /**< [8]     电机状态：0 关闭，1 使能 */
        uint8_t  fw_encoder_sta;     /**< [9]     方位编码器：0 正常，1 故障 */
        uint8_t  ext_servo_comm_sta; /**< [10]    外部伺服通信：0 正常，1 故障（270） */
        uint16_t vehicle_heading;    /**< [11-12] 车航向 H，LSB=0.01°，0~36000 */
        int16_t  vehicle_pitch;      /**< [13-14] 车俯仰 P，LSB=0.01°，-9000~9000 */
        uint16_t vehicle_roll;       /**< [15-16] 车横滚 R，LSB=0.01°，-9000~9000 */
        int32_t  longitude;          /**< [17-20] 经度，LSB=0.00001° */
        int32_t  latitude;           /**< [21-24] 纬度，LSB=0.00001° */
        int32_t  altitude;           /**< [25-28] 海拔，m */
        uint16_t turntable_fw;       /**< [29-30] 转台方位，0~36000 */
        int16_t  turntable_gd;       /**< [31-32] 转台俯仰，-9000~9000 */
        uint8_t  guide_type;         /**< [33]    引导：0 停止跟踪，1 启动跟踪 */
        int16_t  axis_x;             /**< [34-35] 目标 X，东北天坐标，±30000 */
        int16_t  axis_y;             /**< [36-37] 目标 Y */
        int16_t  axis_z;             /**< [38-39] 目标 Z */
        uint16_t time_year;          /**< [40-41] 年 */
        uint8_t  time_month;         /**< [42]    月 */
        uint8_t  time_day;           /**< [43]    日 */
        uint8_t  time_hour;          /**< [44]    时 */
        uint8_t  time_min;           /**< [45]    分 */
        uint8_t  time_sec;           /**< [46]    秒 */
        uint16_t time_ms;            /**< [47-48] 毫秒 */
        uint32_t track_id;           /**< [49-52] 航迹号 */
        uint8_t  checksum;           /**< [53]    校验和：byte[0]~byte[52] 累加和低 8 位 */
    } field;
} FW_TO_GD_U;
#pragma pack(pop)

/** @brief 方位→俯仰接收帧别名（与 servo 模块命名一致） */
typedef FW_TO_GD_U REC_FW_FY_CMD_U;
/** @brief 俯仰→方位发送帧别名（与 servo 模块命名一致） */
typedef GD_TO_FW_U SEND_FW_FY_CMD_U;

/* ========================================================================== */
/*  俯仰 ↔ 无刷直流力矩电机驱动板（MDRV）                                       */
/*  协议：无刷直流力矩电机驱动器通讯协议_中科友成自研版_20240613                  */
/*  物理接口：RS422；460800bps，8N1；全双工；1ms 周期；接收应答模式               */
/*  驱动板「接收」= 俯仰「发送」；驱动板「发送」= 俯仰「接收」                     */
/* ========================================================================== */

/** @brief 驱动板串口映射（与 bsp_uart.h COM_MOTOR 同口） */
#define COM_GD_MDRV          COM_MOTOR

#define MDRV_FRAME_HEAD      0x55U  /**< 帧头固定值 */
#define GD_TO_MDRV_FRAME_LEN 10U   /**< 俯仰→驱动板（驱动模块接收帧）整帧字节数 */
#define MDRV_TO_GD_FRAME_LEN 8U    /**< 驱动板→俯仰（驱动模块发送帧）整帧字节数 */

/** @brief 累加和参与字节数（从 byte[1] 起算，不含帧头与 checksum 本身） */
#define GD_TO_MDRV_CHECKSUM_PAYLOAD_LEN  8U /**< byte[1]~byte[8] */
#define MDRV_TO_GD_CHECKSUM_PAYLOAD_LEN  6U /**< byte[1]~byte[6] */

/**
 * @brief COM_REC_DataAnalysis_nocheck 定长参数
 * @note  8 字节帧：帧头 0x55 + 7 字节载荷；包尾校验字节 index=7（与 motor_ctrl 一致）
 */
#define MDRV_NOCHECK_FRAME_LEN  (MDRV_TO_GD_FRAME_LEN - 1U)

/** @brief 俯仰→驱动板：电流环输入 LSB（A） */
#define GD_TO_MDRV_CURRENT_LSB_A     0.001f
/** @brief 驱动板→俯仰：反馈电流 LSB（A，1mA） */
#define MDRV_TO_GD_CURRENT_LSB_A     0.001f
/** @brief 俯仰→驱动板：编码器 LSB（°），360° 对应 24bit */
#define GD_TO_MDRV_ENCODER_LSB_DEG     (360.0f / 16777216.0f)

/* ---------- 俯仰→驱动板：byte[6] 电机命令 / 使能 ---------- */
#define MDRV_CMD_MOTOR_OFF       0x00U /**< 电机关，byte[1-2] 电流强制为 0 */
#define MDRV_CMD_MOTOR_ON        0x01U /**< 电机开，byte[1-2] 为电流环输入 */
#define MDRV_CMD_FIND_ZERO       0xF0U /**< 寻零（只发一次，之后发停机） */
#define MDRV_CMD_WRITE_EEPROM    0xF1U /**< 参数写入 EEPROM（只发一次） */
#define MDRV_CMD_SET_INDUCTANCE  0xF2U /**< 设置线电感：param*1e-6 H */
#define MDRV_CMD_READ_INDUCTANCE 0xF3U /**< 读取线电感 */
#define MDRV_CMD_SET_RESISTANCE  0xF4U /**< 设置线电阻：param*0.001 Ω */
#define MDRV_CMD_READ_RESISTANCE 0xF5U /**< 读取线电阻 */
#define MDRV_CMD_SET_POLE_PAIRS  0xF6U /**< 设置极对数：byte[7] 为极对数 */
#define MDRV_CMD_READ_POLE_PAIRS 0xF7U /**< 读取极对数 */
#define MDRV_CMD_QUERY_ZERO      0xF8U /**< 查询寻零结果 */
#define MDRV_CMD_SET_SATURATION  0xF9U /**< 设置饱和输出：param*0.001，0~0.9 */
#define MDRV_CMD_READ_SATURATION 0xFAU /**< 读取饱和输出 */
#define MDRV_CMD_SET_ALIGN_VOLT  0xFBU /**< 设置对齐电压：param*0.001，0~0.9 */
#define MDRV_CMD_READ_ALIGN_VOLT 0xFCU /**< 读取对齐电压 */
#define MDRV_CMD_READ_STORE      0xFDU /**< 读取存储结果 */

/* ---------- 驱动板→俯仰：byte[4] 参数标识（应答 cmd 为 0xF3~0xFD 等） ---------- */
#define MDRV_RSP_PARAM_TBD       0x00U /**< 正常运控应答，byte[5-6] 待定 */
#define MDRV_RSP_PARAM_RUN       0x01U /**< 正常运控应答，byte[5-6] 待定 */

/* ---------- 寻零 / 存储结果（byte[5-6] 低 16 位） ---------- */
#define MDRV_ZERO_NOT_DONE       0U    /**< 本次上电未寻零 / 对齐电压不在 0~1 */
#define MDRV_ZERO_SUCCESS        1U    /**< 寻零成功 */
#define MDRV_ZERO_DIR_MISMATCH   128U  /**< 编码器与电机方向不一致，需换线 */
#define MDRV_STORE_NOT_DONE      0U    /**< 本次上电未存储 */
#define MDRV_STORE_SUCCESS       1U    /**< 存储成功 */
#define MDRV_STORE_FAIL          128U  /**< 存储失败 */

/**
 * @brief 从 encoder_l/m/h 组装 24bit 无符号编码器原始值（小端 wire 顺序）
 */
#define USER_CTRL_MDRV_GET_U24(l, m, h) \
    ((uint32_t)(((uint32_t)(l)) | ((uint32_t)(m) << 8) | ((uint32_t)(h) << 16)))

/**
 * @brief 俯仰→驱动板发送帧（驱动模块接收帧，表 1）
 *
 * ctrl_current：Int16 小端，bit15 为方向（1 负 / 0 正），LSB=0.001A。
 * encoder_*    ：24bit 编码器值，小端；360° 对应 2^24。
 * motor_cmd    ：0x00 关 / 0x01 开 / 0xF0~0xFD 配置命令，见 MDRV_CMD_xxx。
 * param        ：Uint16 小端；含义由 motor_cmd 决定（如 0xF2 线电感等）。
 * checksum     ：byte[1]~byte[8] 累加和低 8 位。
 */
#pragma pack(push, 1)
typedef union {
    uint8_t buf[GD_TO_MDRV_FRAME_LEN];
    struct {
        uint8_t  head;         /**< [0]    帧头，固定 0x55 */
        int16_t  ctrl_current; /**< [1-2]  电流环输入，小端，LSB=0.001A */
        uint8_t  encoder_l;    /**< [3]    编码器低字节（24bit 小端） */
        uint8_t  encoder_m;    /**< [4]    编码器中字节 */
        uint8_t  encoder_h;    /**< [5]    编码器高字节 */
        uint8_t  motor_cmd;    /**< [6]    电机使能/命令，见 MDRV_CMD_xxx */
        uint16_t param;        /**< [7-8]  配置参数，小端；含义随 motor_cmd */
        uint8_t  checksum;     /**< [9]    校验和：byte[1]~byte[8] 累加和低 8 位 */
    } field;
} GD_TO_MDRV_U;
#pragma pack(pop)

/**
 * @brief 驱动板→俯仰接收帧（驱动模块发送帧，表 2）
 *
 * current   ：Int16 小端，bit15 为方向，范围 -25000~+25000，LSB=1mA。
 * drv_sta   ：驱动状态（协议待定）。
 * param_id  ：参数标识；0x00/0x01 时 byte[5-6] 待定；0xF3~0xFD 为配置应答。
 * param     ：Uint16 小端；含义由 param_id 决定。
 * checksum  ：byte[1]~byte[6] 累加和低 8 位。
 */
#pragma pack(push, 1)
typedef union {
    uint8_t buf[MDRV_TO_GD_FRAME_LEN];
    struct {
        uint8_t  head;      /**< [0]    帧头，固定 0x55 */
        int16_t  current;   /**< [1-2]  反馈电流，小端，LSB=1mA（-25A~+25A） */
        uint8_t  drv_sta;   /**< [3]    驱动状态，定义待定 */
        uint8_t  param_id;  /**< [4]    参数标识 / 应答命令码 */
        uint16_t param;     /**< [5-6]  参数值，小端 */
        uint8_t  checksum;  /**< [7]    校验和：byte[1]~byte[6] 累加和低 8 位 */
    } field;
} MDRV_TO_GD_U;
#pragma pack(pop)

/** @brief 俯仰→驱动板发送帧别名 */
typedef GD_TO_MDRV_U SEND_MDRV_CMD_U;
/** @brief 驱动板→俯仰接收帧别名 */
typedef MDRV_TO_GD_U REC_MDRV_CMD_U;

/*  接收流程（app_ctrl.c）：                                                    */
/*    COM_Rcv_SerialPort_Init(COM_GD_MDRV, 0x55, 0, MDRV_NOCHECK_FRAME_LEN)   */
/*    → COM_REC_DataAnalysis_nocheck() 取帧到 MdrvToGdCmd.buf                   */
/*    → 应用层校验 byte[1]~byte[6] 累加和 = byte[7]                              */
/*  发送流程（1kHz）：                                                          */
/*    USER_Ctrl_MdrvSetRunCmd(mA, enable) 更新给定                              */
/*    → 1ms 任务 USER_Ctrl_MdrvCmdSend(NULL) 刷新编码器并发出 10 字节             */

/* ========================================================================== */
/*  陀螺串口上报帧（协议：陀螺协议.xlsx）                                        */
/*                                                                            */
/*  物理接口：USART6（COM_GD_GYRO）                                            */
/*  帧格式  ：定长 13 字节；帧头 0xA2；仅含 X/Z 两轴，无 Y 轴                    */
/*  校验方式：byte[12] = byte[1] ^ byte[2] ^ … ^ byte[11]（异或，非累加和）     */
/*  角速度  ：24bit 有符号，wire 顺序为高/中/低字节（大端），16384 LSB/(°/s)     */
/*                                                                            */
/*  接收流程（app_ctrl.c）：                                                    */
/*    COM_Rcv_SerialPort_Init(COM_GD_GYRO, 0xA2, 0, 13)                       */
/*    → COM_REC_DataAnalysis_1head_xor() 取帧到 GyroRxFrame.buf               */
/*    → USER_Ctrl_GyroBeUnpack() 大端解析到 GyroRxData                         */
/*                                                                            */
/*  帧布局：                                                                    */
/*    [0]head [1]res1 [2]res2 [3]res3                                         */
/*    [4]gyro_x_h [5]gyro_x_m [6]gyro_x_l                                     */
/*    [7]gyro_z_h [8]gyro_z_m [9]gyro_z_l                                     */
/*    [10]res4 [11]frame_cnt [12]checksum                                     */
/* ========================================================================== */

/** @brief 陀螺串口映射：USART6（与 bsp_uart.h 中 COM_GD_IN 同口，勿重复初始化） */
#define COM_GD_GYRO          COM_USART_6

#define GYRO_FRAME_LEN       13U   /**< 整帧字节数：byte[0]~byte[12] */
#define GYRO_FRAME_HEAD      0xA2U /**< 帧头固定值 */
#define GYRO_CHECKSUM_LEN    11U   /**< 异或参与字节数：byte[1]~byte[11] */
#define GYRO_LSB_PER_DPS     16384.0f /**< 角速度量化：16384 个 LSB 对应 1°/s */
#define GYRO_DPS_FACTOR      (1.0f / GYRO_LSB_PER_DPS) /**< raw → °/s 换算系数 */

/**
 * @brief 陀螺帧大端解析后的物理量
 *
 * 由 USER_Ctrl_GyroCmdRecvTask() 在收帧成功后写入；
 * 业务层优先读 GyroRxData（已换算），调试时可读 GyroRxFrame（原始字节）。
 */
typedef struct {
    int32_t gyro_x_raw; /**< X 轴 24bit 有符号原始值（未缩放） */
    int32_t gyro_z_raw; /**< Z 轴 24bit 有符号原始值（未缩放） */
    float   gyro_x_dps; /**< X 轴角速度，单位 °/s */
    float   gyro_z_dps; /**< Z 轴角速度，单位 °/s */
} GYRO_RX_DATA_T;

/**
 * @brief 从 gyro_x_h/m/l 或 gyro_z_h/m/l 组装 24bit 有符号原始值（大端）
 *
 * @param h  高 8 位（含符号位，byte[4] 或 byte[7]）
 * @param m  中 8 位
 * @param l  低 8 位
 * @return   符号扩展后的 int32 原始值；除以 GYRO_LSB_PER_DPS 得 °/s
 */
#define USER_CTRL_GYRO_GET_S24(h, m, l) \
    ((((int32_t)(int8_t)(h) << 16) | ((int32_t)(m) << 8) | (int32_t)(l)))

/**
 * @brief 陀螺上报帧联合体
 *
 * 校验：checksum = byte[1] ^ … ^ byte[11]。
 * 角速度(°/s) = USER_CTRL_GYRO_GET_S24(h,m,l) * GYRO_DPS_FACTOR。
 * 接收校验由 COM_REC_DataAnalysis_1head_xor() 完成，无需手动验 checksum。
 */
#pragma pack(push, 1)
typedef union {
    uint8_t buf[GYRO_FRAME_LEN]; /**< 原始字节缓冲，串口 DMA 写入 / 驱动读出 */
    struct {
        uint8_t head;      /**< [0]  帧头，固定 0xA2 */
        uint8_t res1;      /**< [1]  预留，协议规定固定 0x00，参与异或校验 */
        uint8_t res2;      /**< [2]  预留，参与异或校验 */
        uint8_t res3;      /**< [3]  预留，参与异或校验 */
        uint8_t gyro_x_h;  /**< [4]  X 轴角速度高字节（MSB，含符号位） */
        uint8_t gyro_x_m;  /**< [5]  X 轴角速度中字节 */
        uint8_t gyro_x_l;  /**< [6]  X 轴角速度低字节（LSB） */
        uint8_t gyro_z_h;  /**< [7]  Z 轴角速度高字节（MSB，含符号位） */
        uint8_t gyro_z_m;  /**< [8]  Z 轴角速度中字节 */
        uint8_t gyro_z_l;  /**< [9]  Z 轴角速度低字节（LSB） */
        uint8_t res4;      /**< [10] 预留，协议规定固定 0x00，参与异或校验 */
        uint8_t frame_cnt; /**< [11] 帧计数，0~255 循环，参与异或校验 */
        uint8_t checksum;  /**< [12] 异或校验字节，不参与自身异或 */
    } field;
} GYRO_RX_U;
#pragma pack(pop)

/* -------------------------------------------------------------------------- */
/*                              接收数据缓冲（最新一帧）                         */
/* -------------------------------------------------------------------------- */
extern IMG_TO_GD_U ImgToGdCmd; /**< 图像→俯仰接收缓冲，USER_Ctrl_ImgCmdRecvTask 更新 */
extern FW_TO_GD_U  FwToGdCmd;  /**< 方位→俯仰接收缓冲，USER_Ctrl_FwCmdRecvTask 更新 */
extern GD_TO_IMG_U GdToImgCmd; /**< 俯仰→图像发送缓冲，USER_Ctrl_ImgCmdSend 发出 */
extern GD_TO_FW_U  GdToFwCmd;  /**< 俯仰→方位发送缓冲，USER_Ctrl_FwCmdSend 发出 */
extern GD_TO_MDRV_U GdToMdrvCmd;   /**< 俯仰→驱动板发送缓冲（驱动模块接收帧） */
extern MDRV_TO_GD_U MdrvToGdCmd;   /**< 驱动板→俯仰接收缓冲（驱动模块发送帧） */
extern GYRO_RX_U      GyroRxFrame;  /**< 陀螺最新一帧原始数据（COM_REC_DataAnalysis_1head_xor 写入） */
extern GYRO_RX_DATA_T GyroRxData;   /**< 陀螺最新一帧解析结果（大端 24bit → °/s） */

/* -------------------------------------------------------------------------- */
/*                              系统入口                                        */
/* -------------------------------------------------------------------------- */
void APP_Ctrl_System_Init(void);   /**< 系统初始化：串口、定时器、图像/方位/陀螺接收等 */
void APP_Ctrl_System_Handle(void); /**< 主循环：心跳灯、ADC、图像/方位/陀螺接收任务 */

/* -------------------------------------------------------------------------- */
/*                              通信辅助接口                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  计算累加和校验（低 8 位）
 * @param  buf  待校验数据首地址
 * @param  len  参与累加的字节数（不含 checksum 本身）
 * @return 校验字节，可直接赋给 frame.checksum
 */
uint8_t USER_Ctrl_CalcChecksum(const uint8_t *buf, uint16_t len);

/**
 * @brief  计算陀螺帧异或校验（组帧 / 自测用；正常接收由驱动内部校验）
 * @param  buf  待校验数据首地址，通常为 &GyroRxFrame.buf[1]
 * @param  len  参与异或的字节数，固定传 GYRO_CHECKSUM_LEN（11）
 * @return 校验字节，应等于 GyroRxFrame.field.checksum
 * @note   校验范围 byte[1]~byte[11]；byte[0] 帧头与 byte[12] 校验位均不参与
 */
uint8_t USER_Ctrl_GyroCalcXorChecksum(const uint8_t *buf, uint16_t len);

/** @brief 图像指令是否收到新帧（1=有新数据，读后建议 USER_Ctrl_ImgCmdRecvClearFresh） */
uint8_t USER_Ctrl_ImgCmdRecvIsFresh(void);
/** @brief 方位指令是否收到新帧 */
uint8_t USER_Ctrl_FwCmdRecvIsFresh(void);
/** @brief 清除图像指令新帧标志 */
void USER_Ctrl_ImgCmdRecvClearFresh(void);
/** @brief 清除方位指令新帧标志 */
void USER_Ctrl_FwCmdRecvClearFresh(void);

/**
 * @brief  查询陀螺是否收到新帧
 * @return 1=有新数据，0=无新数据
 * @note   读 GyroRxFrame / GyroRxData 后应调用 USER_Ctrl_GyroCmdRecvClearFresh()
 */
uint8_t USER_Ctrl_GyroCmdRecvIsFresh(void);

/** @brief  清除陀螺新帧标志 */
void USER_Ctrl_GyroCmdRecvClearFresh(void);

/** @brief 驱动板应答是否收到新帧（1=有新数据，读后建议 USER_Ctrl_MdrvCmdRecvClearFresh） */
uint8_t USER_Ctrl_MdrvCmdRecvIsFresh(void);
/** @brief 清除驱动板应答新帧标志 */
void USER_Ctrl_MdrvCmdRecvClearFresh(void);

/**
 * @brief  更新驱动板运控给定（在 1kHz 发送前写入，由电流环/伺服层调用）
 * @param  current_mA  电流给定，单位 mA（协议 LSB=0.001A，数值与 mA 一致）
 * @param  enable      1=电机开（0x01），0=电机关（0x00，电流强制为 0）
 */
void USER_Ctrl_MdrvSetRunCmd(int16_t current_mA, uint8_t enable);

/**
 * @brief  更新方位轴运控给定（1kHz 发送前写入，由电流环/伺服层调用）
 * @param  current_mA  控制电流，单位 mA（协议 LSB=0.001A，数值与 mA 一致）
 * @param  enable      1=电机使能，0=电机关（电流强制为 0）
 */
void USER_Ctrl_FwSetRunCmd(int16_t current_mA, uint8_t enable);

/** @brief  设置电磁锁使能（协议 byte5：0 关，1 开，270 项目） */
void USER_Ctrl_FwSetLockEnable(uint8_t enable);

/**
 * @brief  设置方位/俯仰偏差角
 * @param  fw_cdeg  方位偏差角，LSB=0.01°，范围 ±1000
 * @param  gd_cdeg  俯仰偏差角，LSB=0.01°，范围 ±1000
 */
void USER_Ctrl_FwSetDevAngle(int16_t fw_cdeg, int16_t gd_cdeg);

/** @brief  设置伺服工作状态（GD_TO_FW_SERVO_STOP/GUIDE/LOCK） */
void USER_Ctrl_FwSetServoWorkSta(uint8_t sta);

/** @brief  设置俯仰→方位帧 byte12 故障码 */
void USER_Ctrl_FwSetFaultCode(uint8_t code);

/**
 * @brief  俯仰→图像板发送（1kHz 周期上报）
 * @param  frame  待发送帧；传 NULL 则发送全局缓冲 GdToImgCmd
 * @note   自动写入 head/len/checksum，经 COM_GD_IMG 发出 56 字节
 */
void USER_Ctrl_ImgCmdSend(GD_TO_IMG_U *frame);

/**
 * @brief  俯仰→方位板发送（1kHz 周期上报）
 * @param  frame  待发送帧；传 NULL 则发送全局缓冲 GdToFwCmd（自动刷新 payload）
 * @note   自动写入 head/len/checksum，经 COM_GD_FW 发出 14 字节
 */
void USER_Ctrl_FwCmdSend(GD_TO_FW_U *frame);

/**
 * @brief  俯仰→驱动板发送
 * @param  frame  待发送帧；传 NULL 则发送全局缓冲 GdToMdrvCmd（自动刷新编码器与运控给定）
 * @note   通信口 COM_GD_MDRV，整帧 10 字节；由 1ms 任务以 1kHz 周期调用
 */
void USER_Ctrl_MdrvCmdSend(GD_TO_MDRV_U *frame);

/* -------------------------------------------------------------------------- */
/*                              俯仰编码器 SPI                                   */
/* -------------------------------------------------------------------------- */

extern __IO uint8_t   SPI1_rxbuf[SPI1_BMQ_RX_LEN]; /**< SPI1 DMA 原始接收缓冲 */
extern volatile uint32_t WHGBMQ_Data;            /**< 编码器解析码值（25 位，ISR 更新） */
extern volatile float    angle_real;             /**< 编码器角度（°，ISR 更新） */

/**
 * @brief  1ms 任务：启动 SPI1 DMA 读取编码器到 SPI1_rxbuf
 * @note   传输完成后由 HAL_SPI_RxCpltCallback 调用 BmqData_Dispose 解析；
 *         若 bsp_timer.c 中 TIM6 仍调用 COM_SPI1_Sample1ms，请二选一避免重复启动 DMA
 */
void USER_Ctrl_BmqSpiReadTask(void);

/** @brief 编码器原始数据解析（也可在 SPI 接收完成回调中调用） */
void BmqData_Dispose(void);

/* -------------------------------------------------------------------------- */
/*                              ADC 采集标志                                    */
/* -------------------------------------------------------------------------- */
HAL_StatusTypeDef GET_ADC_STATE_Flag(void); /**< 获取 ADC DMA 采集完成标志 */
void SET_ADC_STATE_Flag(void);              /**< 置位 ADC 采集完成标志 */
void CLEAR_ADC_STATE_Flag(void);            /**< 清除 ADC 采集完成标志 */

#endif
