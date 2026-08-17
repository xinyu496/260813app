/**
 * @file    app.h
 * @brief   俯仰控制板应用层头文件
 *
 * 本文件定义：
 *   1. 图像板 / 方位板串口通信协议结构体
 *   2. 光电校正 Flash、ADC 等常量
 *   3. 系统初始化与主循环对外接口
 *
 * 硬件接口：
 *   USART1  1843200 bps  俯仰 <-> 图像板
 *   USART2   921600 bps  俯仰 <-> 方位板
 *
 * 通用帧格式：
 *   byte[0] = 0xEB（帧头）
 *   byte[1] = len（整帧长度，含校验字节）
 *   byte[len-1] = sum(byte[0..len-2]) & 0xFF（累加和）
 */

#ifndef __APP__
#define __APP__

/* HAL 与外设头文件 */
#include "main.h"
#include "usart.h"   /* huart1 / huart2 图像板、方位板串口 */
#include "tim.h"     /* TIM6 1ms 时基 */
#include "adc.h"     /* ADC1 电流/电压采集 */
#include "spi.h"     /* SPI1 俯仰编码器 */
#include "geo_los.h" /* 地理引导视线角解算 */

/* ========================================================================== */
/*                     图像板协议：帧内接收端地址 rec_addr                       */
/* ========================================================================== */

#define REC_ADDR_NULL     0x00  /* 空地址，不处理 */
#define REC_ADDR_SF       0x10  /* 伺服：本板解析 cmd 0x01~0x0A */
#define REC_ADDR_IR       0x20  /* 红外载荷控制 */
#define REC_ADDR_VISIBLE  0x30  /* 可见光载荷控制 */

/*
 * 仅当 rec_addr == REC_ADDR_SF 时，app.c 中 CMD_Dataanalysis
 * 才会分发 cmd 字段；其余地址由图像板本地处理或透传。
 */

/* ========================================================================== */
/*                        光电校正参数 Flash 存储常量                            */
/* ========================================================================== */

/* 魔数 ASCII 'COLB'，标识 Sector 10 已写入校正数据 */
#define USER_CTRL_OPT_CALIB_FLASH_MAGIC    0x434F4C42U
/* 物理地址：STM32F4 内部 Flash Sector 10 */
#define USER_CTRL_OPT_CALIB_FLASH_ADDR     FLASH_F4ADDR_SECTOR_10
/* CRC16-CCITT 覆盖字节数：magic~gd_comp，不含 crc 字段本身 */
#define USER_CTRL_OPT_CALIB_FLASH_CRC_LEN  24U
/* 结构体总大小（#pragma pack(1) 紧凑排列，无尾填充） */
#define USER_CTRL_OPT_CALIB_FLASH_SIZE     26U

/* ========================================================================== */
/*                              ADC 采样参数                                   */
/* ========================================================================== */

#define ADC1_CHANEL_NUM   1   /* ADC1 使能通道数 */
#define ADC1_COLLECT_NUM  10  /* DMA 每通道采样点数，供均值滤波 */

/* ========================================================================== */
/*              俯仰->方位 偏差角协议限幅（单位 0.01 度，1 LSB = 0.01 度）       */
/* ========================================================================== */

#define GD_TO_FW_DEV_ANGLE_MIN  (-1000)  /* -10.00 度 */
#define GD_TO_FW_DEV_ANGLE_MAX   1000   /* +10.00 度 */

/* ========================================================================== */
/*                                 类型定义                                     */
/* ========================================================================== */

/*
 * 图像板伺服指令回调类型。
 * 参数 data 指向 CMD_220_T.data[20]，具体布局随 cmd 码变化，见 SF.h。
 */
typedef void (*REC_CMD_Fun)(uint8_t *cmd_struct);

/* ========================================================================== */
/*              图像板 -> 俯仰  下行帧 CMD_220_T（31 字节，USART1 RX）           */
/* ========================================================================== */

/*
 * 伺服指令码 cmd（rec_addr=0x10 时）：
 *   0x01 刹车    0x02 速度运动  0x03 位置运动  0x04 地理引导
 *   0x05 点选跟踪 0x06 收藏     0x07 上锁     0x08 解锁
 *   0x09 光电校正 0x0A 步进运动
 *
 * 脱靶量 fw/fy_miss_distance 在 cmd=0x05 且 miss_distance_enable=1 时有效。
 */
#pragma pack(1)
typedef struct {
    uint8_t  head;                 /* [0]  帧头，固定 0xEB */
    uint8_t  len;                  /* [1]  帧长，固定 0x1F（31） */
    uint8_t  rec_addr;             /* [2]  接收端，见 REC_ADDR_xxx */
    uint8_t  cmd;                  /* [3]  控制指令 0x01~0x0A */
    uint8_t  data[20];             /* [4]  参数区，含义随 cmd 变化 */
    int16_t  fw_miss_distance;     /* [24] 方位脱靶量，0.01 度，小端 */
    int16_t  fy_miss_distance;     /* [26] 俯仰脱靶量，0.01 度，小端 */
    uint8_t  miss_distance_enable; /* [28] 脱靶量有效：1=有效 */
    uint8_t  error_state;          /* [29] 图像板故障状态 */
    uint8_t  check_val;            /* [30] 累加和校验字节 */
} CMD_220_T;
#pragma pack()

/*
 * 联合体便于：
 *   rec_cmd_buffer.cmd[]          — DMA/串口整帧 memcpy
 *   rec_cmd_buffer.rec_cmd_struct — 字段级访问
 */
typedef union {
    CMD_220_T rec_cmd_struct;
    uint8_t   cmd[31];
} REC_IMAGE_CMD_U;

/* ========================================================================== */
/*              俯仰 -> 图像板  上行帧 CMD_FY_IMAGE_T（56 字节，USART1 TX）      */
/* ========================================================================== */

/*
 * 1ms 周期由 app.c SEND_IMAGE_CMD_Handle 组帧发送。
 *
 * error_state 本板维护位（app.c 写入）：
 *   bit2 — 方位编码器故障（方位板 fw_bmp_state==1）
 *   bit3 — SPI 俯仰编码器 DMA 读取失败
 *
 * warning_state：
 *   bit0 — 俯仰轴到位（gd_axis.status.in_place）
 *   bit1 — 方位轴到位（fw_axis.status.in_place）
 *
 * 角/速度字段单位均为 0.01 度或 0.01 度/s；方位角含 INS.Yaw 北向补偿。
 */
#pragma pack(1)
typedef struct {
    uint8_t  head;              /* [0]   帧头 0xEB */
    uint8_t  len;               /* [1]   帧长 0x38（56） */
    uint16_t  fw_angle;          /* [2]   方位角，0.01 度 */
    int16_t  fy_angle;          /* [4]   俯仰角，0.01 度 */
    int16_t  fw_speed;          /* [6]   方位角速度，0.01 度/s */
    int16_t  fy_speed;          /* [8]   俯仰角速度，0.01 度/s */
    int      target_longitude;  /* [10]  目标经度（当前实现填 0） */
    int      target_latitude;   /* [14]  目标纬度（当前实现填 0） */
    int      target_high;       /* [18]  目标高度 m（当前实现填 0） */
    uint16_t year;              /* [22]  时间-年 */
    uint8_t  month;             /* [24]  时间-月 */
    uint8_t  day;               /* [25]  时间-日 */
    uint8_t  hour;              /* [26]  时间-时 */
    uint8_t  minute;            /* [27]  时间-分 */
    uint8_t  second;            /* [28]  时间-秒 */
    uint16_t ms;                /* [29]  时间-毫秒 */
    uint32_t num;               /* [31]  航迹号 */
    uint8_t  error_state;       /* [35]  故障状态位图 */
    uint8_t  warning_state;     /* [36]  告警/到位状态位图 */
    uint16_t visible_focus;     /* [37]  可见光焦距 */
    uint8_t  visible_state;     /* [39]  可见光状态 */
    uint8_t  visible_light;     /* [40]  可见光亮度 */
    uint8_t  visible_contrast;  /* [41]  可见光对比度 */
    uint8_t  saturability;      /* [42]  可见光饱和度 */
    uint8_t  visible_multiple;  /* [43]  可见光电子变倍倍数 */
    uint16_t lr_focus;          /* [44]  红外焦距 */
    uint8_t  lr_state;          /* [46]  红外状态 */
    uint8_t  lr_contrast;       /* [47]  红外对比度 */
    uint8_t  lr_light;          /* [48]  红外亮度 */
    uint8_t  time;              /* [49]  红外积分时间 */
    uint8_t  filter;            /* [50]  红外噪点滤波 */
    uint16_t fw_north;          /* [51]  正北方位，0.01 度 */
    uint16_t fy_center;         /* [53]  中心俯仰，0.01 度 */
	int      self_longitude;  
    int      self_latitude;   
    int      self_high;       
	uint16_t self_fwangle;
	int16_t  self_fyangle;
	int16_t  self_hgangle;
    uint8_t  check;             /* []  累加和校验 */
} CMD_FY_IMAGE_T;
#pragma pack()

typedef union {
    CMD_FY_IMAGE_T send_cmd_struct;
    uint8_t          cmd[74];   /* DMA 发送缓冲 */
} SEND_IMAGE_CMD_U;

/* ========================================================================== */
/*              方位板 -> 俯仰  上送帧 CMD_FW_FY_T（54 字节，USART2 RX）         */
/* ========================================================================== */

/*
 * 方位板约 1kHz 上送；app.c 主要使用：
 *   fw_bmp_state — 更新俯仰->图像板 error_state bit2
 *   fw_current / bpm_cal 等 — 供 SF 控制环或扩展使用
 */
#pragma pack(1)
typedef struct {
    uint8_t  head;           /* [0]   帧头 0xEB */
    uint8_t  len;            /* [1]   帧长 */
    uint32_t bpm_cal;        /* [2]   方位编码器计数值 */
    int16_t  fw_current;     /* [6]   方位电流 mA */
    uint8_t  motor_state;    /* [8]   方位电机状态 */
    uint8_t  fw_bmp_state;   /* [9]   方位编码器：0=正常 1=故障 */
    uint8_t  connect_state;  /* [10]  外部伺服通信状态 */
    uint16_t H;              /* [11]  车航向 */
    uint16_t P;              /* [13]  车俯仰 */
    uint16_t R;              /* [15]  车横滚 */
    int      longitude;      /* [17]  经度 */
    int      latitude;       /* [21]  纬度 */
    int      high;           /* [25]  海拔 m */
    int16_t  zhuantai_fw;    /* [29]  转台方位角 */
    int16_t  zhuantai_fy;    /* [31]  转台俯仰角 */
    uint8_t  yindao_kind;    /* [33]  引导类型 */
    int16_t  X;              /* [34]  惯导 X */
    int16_t  Y;              /* [36]  惯导 Y */
    int16_t  Z;              /* [38]  惯导 Z */
    uint16_t year;           /* [40]  时间-年 */
    uint8_t  month;          /* [42]  时间-月 */
    uint8_t  day;            /* [43]  时间-日 */
    uint8_t  hour;           /* [44]  时间-时 */
    uint8_t  minute;         /* [45]  时间-分 */
    uint8_t  second;         /* [46]  时间-秒 */
    uint16_t ms;             /* [47]  时间-毫秒 */
    uint32_t num;            /* [49]  航迹号 */
    uint8_t  check;          /* [53]  累加和校验 */
} CMD_FW_FY_T;
#pragma pack()

typedef union {
    CMD_FW_FY_T rec_cmd_struct;
    uint8_t     cmd[54];
} REC_FW_CMD_U;

/* ========================================================================== */
/*              俯仰 -> 方位板  下行帧 CMD_FY_FW_T（14 字节，USART2 TX）         */
/* ========================================================================== */

/*
 * 1ms 周期由 app.c SEND_CONTROL_CMD_Handle 发送。
 * current 字段 = FWControl.I_give * 1000，单位 mA。
 */
#pragma pack(1)
typedef struct {
    uint8_t  head;           /* [0]  帧头 0xEB */
    uint8_t  len;            /* [1]  帧长 0x0E（14） */
    uint8_t  motor_enable;   /* [2]  俯仰电机使能 FW_EN */
    int16_t  current;        /* [3]  俯仰电流给定 mA */
    uint8_t  lock_enable;    /* [5]  电磁锁使能 */
    int16_t  fw_miss_angle;  /* [6]  方位偏差角 0.01 度 */
    int16_t  fy_miss_enable; /* [8]  俯仰偏差角 0.01 度（协议字段名） */
    uint8_t  sf_work_state;  /* [10] 伺服工作状态 */
    uint8_t  fy_error_code;  /* [11] 俯仰故障码 */
    uint8_t  error;          /* [12] 扩展故障字节 */
    uint8_t  check;          /* [13] 累加和校验 */
} CMD_FY_FW_T;
#pragma pack()

typedef union {
    CMD_FY_FW_T send_cmd_struct;
    uint8_t       cmd[14];     /* DMA 发送缓冲 */
} SEND_FW_CMD_U;



/* ========================================================================== */
/*                                外部变量                                      */
/* ========================================================================== */

/*
 * 地理引导视线角输出，由 app.c GEO() 调用 GeoLos_CalcAbsAngle 写入。
 * SF 控制环在 P_GEOLead 模式下读取 lxy_out 计算引导给定。
 */
extern GeoLosAngle_t lxy_out;

/* ========================================================================== */
/*                           系统对外 API 声明                                  */
/* ========================================================================== */

/* 外设与应用初始化：串口 DMA、TIM6、SPI、ADC、伺服、Flash 校正加载 */
void APP_Ctrl_System_Init(void);

/*
 * 主循环任务（main while(1) 调用）：
 *   LED 指示 / ADC 处理 / 串口收帧 / 1ms SF 控制环 / 双串口发送 / SPI 编码器
 */
void APP_Ctrl_System_Handle(void);

/* 返回 HAL_OK 表示 ADC DMA 缓冲区已更新，可读取 ADC1_CovertedValue */
HAL_StatusTypeDef GET_ADC_STATE_Flag(void);

/* 由 ADC DMA 完成中断置位 dma_get_flag */
void SET_ADC_STATE_Flag(void);

/* 主循环读取 ADC 数据后清除标志 */
void CLEAR_ADC_STATE_Flag(void);

/** USART1/2 接收 DMA 缓冲，供中断里重启 DMA 使用 */
extern uint8_t u1_rx_buff[256];
extern uint8_t u2_rx_buff[256];

/** 补处理遗漏的 TC 完成，避免 gState 永久 BUSY_TX */
void APP_Uart_TxCompletePoll(UART_HandleTypeDef *huart);
/** 仅当 SR 存在 ORE/FE/NE/PE 时清除，避免误读 DR 破坏 DMA 接收 */
void APP_Uart_ClearHwErrors(UART_HandleTypeDef *huart);
/** 强制恢复并重启 UART RX DMA + IDLE 中断 */
HAL_StatusTypeDef APP_Uart_RxDmaRestart(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len);

/*
 * 结构体/联合体长度校验（协议固定，不可随意改动）：
 *   CMD_220_T      = 31 B   REC_IMAGE_CMD_U.cmd[31]
 *   CMD_FY_IMAGE_T = 56 B   SEND_IMAGE_CMD_U.cmd[56]
 *   CMD_FW_FY_T    = 54 B   REC_FW_CMD_U.cmd[54]
 *   CMD_FY_FW_T    = 14 B   SEND_FW_CMD_U.cmd[14]
 */

#endif /* __APP__ */
