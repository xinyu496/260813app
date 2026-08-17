/**
  ******************************************************************************
  * @file    rtk_nmea_rx.h
  * @brief   BT-982G1 RTK 模块 NMEA 变长帧接收（逻辑串口 0 / USART6）
  *
  * 【模块】北天 BT-982G1 双天线 RTK GNSS，默认 115200 8N1，TTL（J1 RX1/TX1）
  * 【物理口】USART6：PC6=TX，PC7=RX（MCU RX ← 模块 TX1，交叉连接）
  *
  * 【协议】变长 ASCII 文本（非定长二进制，不可用 0xEB+len 方式收帧）：
  *   NMEA-0183 : '$' + CSV 字段 + '*' + 2 位 XOR 十六进制 + "\r\n"
  *   Unicore   : '#' + 头字段 + ';' + SOL 段 + '*' + CRC32 十六进制 + "\r\n"
  *
  * 【模块默认 1Hz 输出】GNGGA、GNRMC、GNVTG、GNGSA、GNGSV、HEADINGA、THS
  *   本层只负责“收完整句”；语义解析见 rtk_nmea_parse.h / .c
  *
  * 【调用顺序】
  *   1. 系统初始化 MX_USART6_UART_Init()
  *   2. RtkNmea_Rx_Init()                     — 启动 DMA + IDLE
  *   3. USART6_IRQHandler → RtkNmea_Rx_IdleIrqHandler()
  *   4. 主循环 APP_Ctrl_System_Handle() 内 RtkNmea_Rx_Process()
  *
  * 【与定基座业务关系】
  *   定基座场景下模块仍 1Hz 输出；本层队列缓冲多条语句，
  *   供解析层提取：本机经纬高(GGA)、基座导航系航向/俯仰(HEADINGA) 等。
  ******************************************************************************
  */
#ifndef RTK_NMEA_RX_H
#define RTK_NMEA_RX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*                              配置常量                                       */
/* -------------------------------------------------------------------------- */

/**
  * 逻辑串口编号。
  * 0 = USART6(RTK)；1/2 预留给 USART1 图像板、USART2 方位板，便于日志区分。
  */
#define RTK_NMEA_UART_PORT           0U

/**
  * DMA 接收缓冲字节数。
  * BT-982G1 1Hz burst 连发约 7 条、合计 ~280~800 字节；须 ≥ 单轮总量，
  * 否则末句（#UNIHEADINGA）截断会导致拼帧器挂半句。
  */
#define RTK_NMEA_RX_DMA_BUF_SIZE     512U

/**
  * 单条语句最大字符数（含字符串结束符 '\0'）。
  * HEADINGA 含 CRC 尾缀时可达 ~180 字符，256 留余量防截断。
  */
#define RTK_NMEA_SENTENCE_MAX        256U

/**
  * 已完成语句环形队列槽位数。
  * 1Hz 约 7 句/burst；20Hz×3 句/s 时主循环偶发阻塞需更大余量。
  * 满则丢弃新句，避免在中断里阻塞。
  */
#define RTK_NMEA_SENTENCE_QUEUE      16U

/** 主循环 RX 看门狗：idle_cnt 在此时间内不增则强制重启 DMA */
#define RTK_NMEA_RX_WATCHDOG_MS      500U

/* -------------------------------------------------------------------------- */
/*                         调试统计（Keil Watch 直接观察）                      */
/* -------------------------------------------------------------------------- */

/**
  * @brief RTK 接收链路调试计数，仅观测用，不影响业务逻辑
  *
  * 正常 1Hz / 7 句 burst 运行约 10s 后参考：
  *   idle_cnt ≈ 10
  *   push_ok_cnt ≈ 70，pop_cnt ≈ 70，push_drop_cnt == 0
  *   last_idle_push_cnt ≈ 7（每次 IDLE 后）
  *   push_ok_cnt - pop_cnt ≈ q_now（通常 0~7）
  *   watchdog_recover_cnt == 0（非 0 表示曾触发 RX 看门狗恢复）
  *   restart_fail_cnt == 0
  */
typedef struct
{
  volatile uint32_t idle_cnt;           /**< IDLE 中断次数 */
  volatile uint32_t push_ok_cnt;        /**< 入队成功累计 */
  volatile uint32_t push_drop_cnt;      /**< 队列满丢弃累计 */
  volatile uint32_t push_reject_cnt;    /**< 语句长度非法丢弃累计 */
  volatile uint32_t pop_cnt;            /**< 出队成功累计 */
  volatile uint32_t process_cnt;        /**< RtkNmea_Rx_Process 调用次数 */
  volatile uint32_t builder_reset_cnt;  /**< 半句/超长强制复位次数 */
  volatile uint32_t watchdog_recover_cnt; /**< RX 看门狗强制恢复次数 */
  volatile uint32_t restart_fail_cnt;   /**< DMA 重启连续失败次数 */
  volatile uint8_t  q_max;              /**< 历史最大队列深度 */
  volatile uint8_t  q_now;              /**< 当前队列深度（= s_q_count） */
  volatile uint16_t last_recv_len;      /**< 最近一次 IDLE recv_len */
  volatile uint8_t  last_idle_push_cnt; /**< 最近一次 IDLE 内入队条数 */
} RtkNmea_RxDebug_t;

/** 调试统计快照，RtkNmea_Rx_Init 清零 */
extern RtkNmea_RxDebug_t rtk_nmea_rx_dbg;

/* -------------------------------------------------------------------------- */
/*                              对外 API                                       */
/* -------------------------------------------------------------------------- */

/**
  * @brief  初始化 RTK 接收：清队列/拼帧状态、清 g_rtk_nmea_data、启动 USART6 DMA+IDLE
  * @note   须在 MX_USART6_UART_Init() 之后调用；波特率 115200 在 usart.c 配置
  */
void RtkNmea_Rx_Init(void);

/**
  * @brief  向拼帧状态机喂入 len 字节原始数据
  * @param  data  源缓冲，可为 s_rtk_dma_buf 或测试数据
  * @param  len   字节数；0 或 NULL 直接返回
  * @note   可在 USART6 IDLE 中断上下文调用；O(n) 逐字节，n 通常 ≤512
  */
void RtkNmea_Rx_Feed(const uint8_t *data, uint16_t len);

/**
  * @brief  DMA 收满一整块缓冲时由 HAL_UART_RxCpltCallback 调用
  * @note   连续 burst ≥512B 且无 IDLE 间隔时走此路径；与 IDLE 部分长度路径互补
  */
void RtkNmea_Rx_DmaCpltHandler(void);

/**
  * @brief  从环形队列取出一条以 '\0' 结尾的完整语句（不含 \r\n）
  * @param  buf      输出缓冲
  * @param  buf_size 必须 > 语句长度；建议 ≥ RTK_NMEA_SENTENCE_MAX
  * @retval 1 成功；0 队列空 / 参数非法 / buf 太小
  */
uint8_t RtkNmea_Rx_GetSentence(char *buf, uint16_t buf_size);

/**
  * @brief  主循环轮询： RX 看门狗 → dequeue 全部待处理语句 → RtkNmea_ParseSentence()
  * @note   更新全局 g_rtk_nmea_data；与图像板/方位板收帧同级调用即可
  */
void RtkNmea_Rx_Process(void);

/**
  * @brief  USART6 中断内调用：判 IDLE → 算 recv_len → Feed → 重启 DMA
  * @note   不可在主循环调用；由 stm32f4xx_it.c::USART6_IRQHandler 触发
  */
void RtkNmea_Rx_IdleIrqHandler(void);

/**
  * @brief  获取内部 DMA 缓冲指针（调试或外部重启 DMA 时用）
  * @param  size  非 NULL 时返回 RTK_NMEA_RX_DMA_BUF_SIZE
  */
uint8_t *RtkNmea_Rx_GetDmaBuffer(uint16_t *size);

#ifdef __cplusplus
}
#endif

#endif /* RTK_NMEA_RX_H */
