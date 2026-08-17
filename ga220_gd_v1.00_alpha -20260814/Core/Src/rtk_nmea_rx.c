/**
  ******************************************************************************
  * @file    rtk_nmea_rx.c
  * @brief   BT-982G1 RTK NMEA 变长帧接收实现
  *
  * 【数据流】
  *   BT-982G1 TX1 ──► USART6 RX (PC7)
  *        │ DMA 写入 s_rtk_dma_buf[]
  *        │ UART IDLE（线路空闲）中断
  *        ▼
  *   RtkNmea_Rx_IdleIrqHandler()
  *        │ 计算 recv_len，Feed 增量字节
  *        ▼
  *   rtk_builder_push_byte()  状态机：等 $/#，收至 \n
  *        │ 完整句入队
  *        ▼
  *   RtkNmea_Rx_Process() [主循环]
  *        ▼
  *   RtkNmea_ParseSentence() → g_rtk_nmea_data
  *
  * 【关键设计】
  *   · IDLE ≠ 一条 NMEA：一句可跨多个 DMA 块，必须字节拼帧
  *   · 中断里只 Feed+重启 DMA，不做 strtod/解析（避免阻塞 SF 1ms 环）
  *   · 队列深度 16，容多轮 burst；满则丢句
  *   · DMA 512B；满缓冲由 DMA TC 交付，IDLE 只交付部分长度（去重）
  *   · 主循环看门狗：idle_cnt 超时未增则强制重启 DMA
  *   · 半句遇新 '$'/'#' 强制复位，防跨周期拼帧污染
  *
  * 【线程/上下文】
  *   写：IDLE ISR（Feed → queue_push）
  *   读：主循环（GetSentence → Parse）
  *   无 OS 锁；靠“单写单读环形队列 + 主循环及时 Process”保证一致
  ******************************************************************************
  */

#include "rtk_nmea_rx.h"
#include "rtk_nmea_parse.h"
#include "usart.h"
#include "app.h"
#include "main.h"
#include <string.h>

/* ========================================================================== */
/*                              内部类型与静态变量                              */
/* ========================================================================== */
static uint8_t s_q_count;  /**< 当前排队条数，0..QUEUE-1 */
/** 队列元素：一条完整 ASCII 语句（已剥 \r\n，以 '\0' 结尾） */
typedef struct
{
  char     text[RTK_NMEA_SENTENCE_MAX]; /**< 语句正文 */
  uint16_t len;                         /**< strlen(text) */
} RtkNmea_QueueItem_t;

/** USART6 HAL DMA 接收缓冲；IDLE 中断内读取 [0, recv_len) */
 uint8_t s_rtk_dma_buf[RTK_NMEA_RX_DMA_BUF_SIZE];

/** 调试统计（Keil Watch: rtk_nmea_rx_dbg.xxx） */
RtkNmea_RxDebug_t rtk_nmea_rx_dbg;

/** 更新 q_now / q_max */
static void rtk_dbg_sync_q_depth(void)
{
  rtk_nmea_rx_dbg.q_now = s_q_count;
  if (s_q_count > rtk_nmea_rx_dbg.q_max)
  {
    rtk_nmea_rx_dbg.q_max = s_q_count;
  }
}

/** 已完成语句环形队列：ISR 写 tail，主循环读 head */
static RtkNmea_QueueItem_t s_queue[RTK_NMEA_SENTENCE_QUEUE];
static uint8_t s_q_head;   /**< 读索引，GetSentence 后递增 */
static uint8_t s_q_tail;   /**< 写索引，push 后递增 */


/** 拼帧工作区：正在组装、尚未收到 '\n' 的半条语句 */
static char     s_build_buf[RTK_NMEA_SENTENCE_MAX];
static uint16_t s_build_len;  /**< 已写入 s_build_buf 的字节数 */

/** 512B 满缓冲是否已由 DMA TC 或 IDLE 兜底交付（防双路径重复 Feed） */
static volatile uint8_t s_full_buf_delivered;

/** RX 看门狗：跟踪 idle_cnt 上次变化时刻 */
static uint32_t s_watch_last_idle_cnt;
static uint32_t s_watch_last_activity_tick;

static void rtk_rx_feed_chunk(uint16_t len);
static HAL_StatusTypeDef rtk_rx_dma_restart(void);
static void rtk_rx_watchdog_poll(void);

/* ========================================================================== */
/*                              环形队列（仅 ISR/Feed 写）                     */
/* ========================================================================== */

/**
  * @brief  完整语句压入环形队列
  * @param  sentence  以 '\0' 结尾，须以 '$' 或 '#' 开头
  * @retval 1 成功；0 队列满 / 长度非法 / NULL
  * @note   队列满时静默丢弃，避免在 USART6 中断里死等主循环
  */
static uint8_t rtk_queue_push(const char *sentence)
{
  RtkNmea_QueueItem_t *item;
  uint16_t len;

  if ((sentence == NULL) || (s_q_count >= RTK_NMEA_SENTENCE_QUEUE))
  {
    if (sentence != NULL)
    {
      rtk_nmea_rx_dbg.push_drop_cnt++;
    }
    return 0U;
  }

  len = (uint16_t)strlen(sentence);
  if ((len == 0U) || (len >= RTK_NMEA_SENTENCE_MAX))
  {
    rtk_nmea_rx_dbg.push_reject_cnt++;
    return 0U;
  }

  item = &s_queue[s_q_tail];
  memcpy(item->text, sentence, len + 1U);
  item->len = len;

  s_q_tail = (uint8_t)((s_q_tail + 1U) % RTK_NMEA_SENTENCE_QUEUE);
  s_q_count++;
  rtk_nmea_rx_dbg.push_ok_cnt++;
  rtk_nmea_rx_dbg.last_idle_push_cnt++;
  rtk_dbg_sync_q_depth();
  return 1U;
}

/** 清空拼帧缓冲（正常收完一句 \n 后调用，不计入复位统计） */
static void rtk_clear_builder(void)
{
  s_build_len = 0U;
  s_build_buf[0] = '\0';
}

/** 丢弃半条语句（DMA 截断/超长/新帧头），计入 builder_reset_cnt */
static void rtk_discard_builder(void)
{
  if (s_build_len > 0U)
  {
    rtk_nmea_rx_dbg.builder_reset_cnt++;
  }
  rtk_clear_builder();
}

/**
  * @brief  单字节拼帧状态机（BT-982G1 NMEA 行协议）
  *
  * 合法帧形态：
  *   $GNGGA,...\r\n
  *   #HEADINGA,...\r\n
  *
  * 状态逻辑：
  *   1. 忽略 \r
  *   2. 遇 \n → 若缓冲非空则入队，复位
  *   3. 首字节必须是 '$' 或 '#'，否则丢弃
  *   4. 拼帧中再遇 '$'/'#' → 丢弃半句，从新帧头重收（DMA 截断恢复）
  *   5. 超长 → 复位（防 HEADINGA 异常或噪声）
  */
static void rtk_builder_push_byte(uint8_t byte)
{
  /* 步骤1：CR 为行尾前缀，不存入语句正文 */
  if (byte == '\r')
  {
    return;
  }

  /* 步骤2：LF 标志一条语句结束 */
  if (byte == '\n')
  {
    if (s_build_len > 0U)
    {
      s_build_buf[s_build_len] = '\0';
      (void)rtk_queue_push(s_build_buf);
    }
    rtk_clear_builder();
    return;
  }

  /* 步骤3/4：帧同步；半句期间出现新帧头则丢弃残缺句 */
  if ((byte == '$') || (byte == '#'))
  {
    if (s_build_len > 0U)
    {
      rtk_discard_builder();
    }
  }
  else if (s_build_len == 0U)
  {
    return;
  }

  /* 步骤5：溢出保护 */
  if (s_build_len >= (RTK_NMEA_SENTENCE_MAX - 1U))
  {
    rtk_discard_builder();
    return;
  }

  s_build_buf[s_build_len++] = (char)byte;
  s_build_buf[s_build_len] = '\0';
}

/* ========================================================================== */
/*                              对外接口实现                                   */
/* ========================================================================== */

void RtkNmea_Rx_Feed(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  /* 逐字节驱动拼帧状态机；len 来自 DMA 增量，通常 ≤512 */
  for (i = 0U; i < len; ++i)
  {
    rtk_builder_push_byte(data[i]);
  }
}

uint8_t RtkNmea_Rx_GetSentence(char *buf, uint16_t buf_size)
{
  const RtkNmea_QueueItem_t *item;

  if ((buf == NULL) || (buf_size == 0U) || (s_q_count == 0U))
  {
    return 0U;
  }

  item = &s_queue[s_q_head];

  /* 调用方缓冲必须能容纳 len + '\0' */
  if (item->len >= buf_size)
  {
    return 0U;
  }

  memcpy(buf, item->text, item->len + 1U);

  s_q_head = (uint8_t)((s_q_head + 1U) % RTK_NMEA_SENTENCE_QUEUE);
  s_q_count--;
  rtk_nmea_rx_dbg.pop_cnt++;
  rtk_dbg_sync_q_depth();
  return 1U;
}

void RtkNmea_Rx_Process(void)
{
  /*
   * 每轮主循环最多解析 1 条语句。
   * ParseSentence 含 strtod 等重操作（栈深 ~900B），burst 时若一次 drain
   * 全部 7 条会长时间占用主循环，导致 1ms 伺服环 alldeal() 节拍抖动。
   * 队列深度 16，主循环高频轮询下仍可及时消化 1Hz burst。
   */
  static char s_sentence[RTK_NMEA_SENTENCE_MAX];
  const uint8_t k_budget = 1U;
  uint8_t n = 0U;

  rtk_nmea_rx_dbg.process_cnt++;
  rtk_rx_watchdog_poll();

  while ((n < k_budget) &&
         (RtkNmea_Rx_GetSentence(s_sentence, sizeof(s_sentence)) != 0U))
  {
    /*
     * 先分类后解析：GSA/GSV/VTG/HEADINGA 等非业务句仅 dequeue，
     * 跳过 VerifyChecksum/strcpy/strtod，降低 burst CPU 占用。
     */
    if ((strstr(s_sentence, "GGA") != NULL) ||
        (strstr(s_sentence, "HPR") != NULL) ||
        (strstr(s_sentence, "HDT") != NULL) ||
        (strstr(s_sentence, "RMC") != NULL))
    {
      (void)RtkNmea_ParseSentence(s_sentence, &g_rtk_nmea_data);
    }
    n++;
  }
}

/** 向拼帧器交付一段 DMA 数据；满 512B 时只交付一次 */
static void rtk_rx_feed_chunk(uint16_t len)
{
  if (len == 0U)
  {
    return;
  }

  if (len >= (uint16_t)sizeof(s_rtk_dma_buf))
  {
    len = (uint16_t)sizeof(s_rtk_dma_buf);
    if (s_full_buf_delivered != 0U)
    {
      return;
    }
    s_full_buf_delivered = 1U;
  }

  RtkNmea_Rx_Feed(s_rtk_dma_buf, len);
}

/** DMA 重启，失败重试一次（与 USART1/2 一致） */
static HAL_StatusTypeDef rtk_rx_dma_restart(void)
{
  HAL_StatusTypeDef st;

  s_full_buf_delivered = 0U;

  st = APP_Uart_RxDmaRestart(&huart6, s_rtk_dma_buf, (uint16_t)sizeof(s_rtk_dma_buf));
  if (st != HAL_OK)
  {
    st = APP_Uart_RxDmaRestart(&huart6, s_rtk_dma_buf, (uint16_t)sizeof(s_rtk_dma_buf));
  }
  if (st != HAL_OK)
  {
    rtk_nmea_rx_dbg.restart_fail_cnt++;
  }
  return st;
}

/** 主循环 RX 看门狗：idle_cnt 长时间不增则强制恢复 DMA */
static void rtk_rx_watchdog_poll(void)
{
  uint32_t now;
  uint32_t idle;

  now = HAL_GetTick();
  idle = rtk_nmea_rx_dbg.idle_cnt;

  if (idle != s_watch_last_idle_cnt)
  {
    s_watch_last_idle_cnt = idle;
    s_watch_last_activity_tick = now;
    return;
  }

  if ((now - s_watch_last_activity_tick) < RTK_NMEA_RX_WATCHDOG_MS)
  {
    return;
  }

  (void)HAL_UART_AbortReceive(&huart6);
  s_full_buf_delivered = 0U;
  if (rtk_rx_dma_restart() == HAL_OK)
  {
    rtk_nmea_rx_dbg.watchdog_recover_cnt++;
  }
  s_watch_last_activity_tick = now;
}

void RtkNmea_Rx_DmaCpltHandler(void)
{
  (void)HAL_UART_AbortReceive(&huart6);
  rtk_rx_feed_chunk((uint16_t)sizeof(s_rtk_dma_buf));
  (void)rtk_rx_dma_restart();
}

uint8_t *RtkNmea_Rx_GetDmaBuffer(uint16_t *size)
{
  if (size != NULL)
  {
    *size = (uint16_t)sizeof(s_rtk_dma_buf);
  }
  return s_rtk_dma_buf;
}

/**
  * @brief USART6 UART IDLE 中断处理例程
  *
  * 处理步骤（与 app.c 中 USART1/2 图像板/方位板收帧一致）：
  *   1. 确认 IDLE 标志且 IDLE 中断已使能
  *   2. 清 IDLE 标志
  *   3. recv_len = RxXferSize - DMA剩余计数
  *   4. AbortReceive 停止当前 DMA
  *   5. 部分长度 Feed；满 512B 由 DMA TC 交付（rtk_rx_feed_chunk 去重）
  *   6. rtk_rx_dma_restart 重启下一轮 DMA（失败重试一次）
  */
void RtkNmea_Rx_IdleIrqHandler(void)
{
  uint16_t recv_len;

  if (((READ_REG(huart6.Instance->SR) & UART_IT_IDLE) == RESET) ||
      ((READ_REG(huart6.Instance->CR1) & UART_IT_IDLE) == RESET))
  {
    return;
  }

  __HAL_UART_CLEAR_IDLEFLAG(&huart6);
  recv_len = (uint16_t)(huart6.RxXferSize - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
  (void)HAL_UART_AbortReceive(&huart6);

  rtk_nmea_rx_dbg.idle_cnt++;
  rtk_nmea_rx_dbg.last_idle_push_cnt = 0U;
  rtk_nmea_rx_dbg.last_recv_len = recv_len;

  if (recv_len > 0U)
  {
    rtk_rx_feed_chunk(recv_len);
  }

  (void)rtk_rx_dma_restart();
}

void RtkNmea_Rx_Init(void)
{
  /* ---- 软件状态复位 ---- */
  s_q_head = 0U;
  s_q_tail = 0U;
  s_q_count = 0U;
  rtk_clear_builder();
  s_full_buf_delivered = 0U;
  s_watch_last_idle_cnt = 0U;
  s_watch_last_activity_tick = HAL_GetTick();
  memset(&g_rtk_nmea_data, 0, sizeof(g_rtk_nmea_data));
  memset((void *)&rtk_nmea_rx_dbg, 0, sizeof(rtk_nmea_rx_dbg));

  /*
   * ---- 硬件接收启动 ----
   * 波特率 115200 / 8N1 在 MX_USART6_UART_Init() 配置（BT-982G1 默认）
   * IDLEIE：任意两次字符间隔超过 1 帧时间即进中断，用于切分 DMA 块
   */
  HAL_UART_Receive_DMA(&huart6, s_rtk_dma_buf, (uint16_t)sizeof(s_rtk_dma_buf));
  SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);
}
