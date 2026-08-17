/**
  ******************************************************************************
  * @file    rtk_nmea_parse.c
  * @brief   GNGGA + GNHPR + GPHDT + GNRMC(date) 解析实现
  *
  * 【支持】$GNGGA（位置）、$GNHPR（三姿）、$GPHDT（航向）、$GNRMC（仅 date）；其余 NMEA 忽略。
  ******************************************************************************
  */

#include "rtk_nmea_parse.h"
#include <string.h>
#include <stdlib.h>

/** 全局 RTK 解析快照，RtkNmea_Rx_Process 周期刷新 */
RtkNmea_Data_t g_rtk_nmea_data;

/* ========================================================================== */
/*                    字符串工具（NMEA 空字段常见，须防崩溃）                   */
/* ========================================================================== */

/** 空串/NULL → 0.0f；否则 strtod */
static float rtk_parse_float(const char *s)
{
  if ((s == NULL) || (s[0] == '\0'))
  {
    return 0.0f;
  }
  return (float)strtod(s, NULL);
}

/** 空串/NULL → 0；否则 strtol 十进制 */
static int32_t rtk_parse_int(const char *s)
{
  if ((s == NULL) || (s[0] == '\0'))
  {
    return 0;
  }
  return (int32_t)strtol(s, NULL, 10);
}

/** NMEA 未定位时常见连续逗号 ",,"，字段为空串 */
static uint8_t rtk_field_is_empty(const char *s)
{
  return (s == NULL) || (s[0] == '\0');
}

/**
  * @brief  提取第一个逗号前的语句 ID（仅 '$' 句）
  * @example "$GNGGA,utc,..." → "$GNGGA"
  */
static uint8_t rtk_sentence_id(const char *sentence, char *id, uint16_t id_size)
{
  const char *comma;
  uint16_t len;

  if ((sentence == NULL) || (id == NULL) || (id_size < 2U))
  {
    return 0U;
  }

  if (sentence[0] != '$')
  {
    return 0U;
  }

  comma = strchr(sentence, ',');
  if (comma == NULL)
  {
    return 0U;
  }

  len = (uint16_t)(comma - sentence);
  if ((len == 0U) || (len >= id_size))
  {
    return 0U;
  }

  memcpy(id, sentence, len);
  id[len] = '\0';
  return 1U;
}

/**
  * @brief  原地按逗号切字段（destructive：',' → '\0'）
  * @return 字段个数；fields[0]..fields[count-1] 指向各子串
  * @note   NMEA 不含带逗号的引号字段，BT-982G1 输出满足此假设
  */
static uint8_t rtk_split_fields(char *sentence, char *fields[], uint8_t max_fields)
{
  uint8_t count = 0U;
  char *p = sentence;

  if (sentence == NULL)
  {
    return 0U;
  }

  fields[count++] = p;
  while ((*p != '\0') && (count < max_fields))
  {
    if (*p == ',')
    {
      *p = '\0';
      fields[count++] = p + 1;
    }
    ++p;
  }

  return count;
}

/**
  * @brief  截断末字段中 '*' 及后续校验子串
  * @note   末字段可能是 "102.6347*6F" 或 "0,f3*9e744258"，截断后便于 strtod
  */
static void rtk_strip_checksum_field(char *last_field)
{
  char *star;

  if (last_field == NULL)
  {
    return;
  }

  star = strchr(last_field, '*');
  if (star != NULL)
  {
    *star = '\0';
  }
}

/* ========================================================================== */
/*                         坐标转换与校验                                      */
/* ========================================================================== */

double RtkNmea_DdmToDeg(const char *ddm, char dir)
{
  double val;
  double deg;
  double min;

  if (rtk_field_is_empty(ddm))
  {
    return 0.0;
  }

  /*
   * NMEA 经纬格式：度分合成一个 double，非十进制度
   *   纬度  DDmm.mmmmm   如 2243.08050950 = 22°43.08050950'
   *   经度 DDDmm.mmmmm   如 11401.10579872 = 114°01.10579872'
   * 转换：dec = 整度 + 分/60
   */
  val = strtod(ddm, NULL);
  deg = (double)((int32_t)(val / 100.0));
  min = val - deg * 100.0;
  val = deg + min / 60.0;

  if ((dir == 'S') || (dir == 'W'))
  {
    val = -val;
  }

  return val;
}

/** 单个十六进制字符 → 数值；非法字符返回 -1 */
static int32_t rtk_hex_nibble(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (int32_t)(c - '0');
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (int32_t)(c - 'A' + 10);
  }
  if ((c >= 'a') && (c <= 'f'))
  {
    return (int32_t)(c - 'a' + 10);
  }
  return -1;
}

/**
  * @brief  读取 '*' 后连续十六进制
  * @param  star     指向 '*' 的指针
  * @param  out_val  解析值（NMEA 用低 8 位，HEADINGA CRC32 用全 32 位）
  * @param  out_len  十六进制字符个数（NMEA=2，HEADINGA=8）
  */
static uint8_t rtk_parse_checksum_hex(const char *star, uint32_t *out_val, uint8_t *out_len)
{
  uint32_t val = 0U;
  uint8_t len = 0U;
  const char *p;
  int32_t n;

  if ((star == NULL) || (star[0] != '*'))
  {
    return 0U;
  }

  p = star + 1;
  while ((n = rtk_hex_nibble(*p)) >= 0)
  {
    val = (val << 4) | (uint32_t)n;
    len++;
    p++;
  }

  if (len == 0U)
  {
    return 0U;
  }

  if (out_val != NULL)
  {
    *out_val = val;
  }
  if (out_len != NULL)
  {
    *out_len = len;
  }
  return 1U;
}

uint8_t RtkNmea_VerifyChecksum(const char *sentence)
{
  const char *star;
  const char *p;
  uint8_t sum = 0U;
  uint32_t chk = 0U;
  uint8_t chk_len = 0U;

  if (sentence == NULL)
  {
    return 0U;
  }

  if (sentence[0] != '$')
  {
    return 0U;
  }

  p = sentence + 1;

  star = strchr(sentence, '*');
  if (star == NULL)
  {
    return 1U;  /* 无 * 字段，视为无校验要求 */
  }

  while (p < star)
  {
    sum ^= (uint8_t)(*p);
    ++p;
  }

  if (rtk_parse_checksum_hex(star, &chk, &chk_len) == 0U)
  {
    return 0U;
  }

  return ((uint8_t)chk == sum) ? 1U : 0U;
}

/* ========================================================================== */
/*                    各语句字段解析（字段号见 datasheet 表 7-x）             */
/* ========================================================================== */

/**
  * @brief  GNGGA / GPGGA / BDGGA …（表 7-1，Talker 不同字段布局相同）
  *
  * | idx | 字段    | 映射                |
  * |-----|---------|---------------------|
  * |  1  | utc     | utc_hhmmss          |
  * | 2,3 | lat,NS  | lat_deg             |
  * | 4,5 | lon,EW  | lon_deg             |
  * |  6  | qual    | qual (4=RTK Fixed)  |
  * |  7  | #sats   | num_sats            |
  * |  8  | hdop    | hdop                |
  * |  9  | alt     | alt_m               |
  */
static uint8_t rtk_parse_gga(char *fields[], uint8_t n_fields, RtkNmea_Gga_t *gga)
{
  if ((n_fields < 10U) || (gga == NULL))
  {
    return 0U;
  }

  rtk_strip_checksum_field(fields[n_fields - 1U]);

  /* 冷启动/失锁时 lat/lon 为空：$GNGGA,time,,,,,0,00,... */
  if (rtk_field_is_empty(fields[2]) || rtk_field_is_empty(fields[4]))
  {
    gga->valid = 0U;
    return 0U;
  }

  gga->utc_hhmmss = rtk_parse_float(fields[1]);
  gga->lat_deg = RtkNmea_DdmToDeg(fields[2], fields[3][0]);
  gga->lon_deg = RtkNmea_DdmToDeg(fields[4], fields[5][0]);
  gga->qual = (uint8_t)rtk_parse_int(fields[6]);
  gga->num_sats = (uint8_t)rtk_parse_int(fields[7]);
  gga->hdop = rtk_parse_float(fields[8]);
  gga->alt_m = rtk_parse_float(fields[9]);
  gga->valid = 1U;
  return 1U;
}

/**
  * @brief  GNHPR / GPHPR / BDHPR …（表 7-42 HPR 数据结构）
  *
  * | idx | 字段    | 映射           |
  * |-----|---------|----------------|
  * |  1  | utc     | utc_hhmmss     |
  * |  2  | heading | heading_deg    |
  * |  3  | pitch   | pitch_deg      |
  * |  4  | roll    | roll_deg       |
  * |  5  | QF      | qual / valid   |
  * |  6  | #sats   | num_sats       |
  */
static uint8_t rtk_parse_hpr(char *fields[], uint8_t n_fields, RtkNmea_Tra_t *hpr)
{
  uint8_t qf;

  if ((n_fields < 6U) || (hpr == NULL))
  {
    return 0U;
  }

  rtk_strip_checksum_field(fields[n_fields - 1U]);

  hpr->utc_hhmmss = rtk_parse_float(fields[1]);
  hpr->heading_deg = rtk_parse_float(fields[2]);
  hpr->pitch_deg = rtk_parse_float(fields[3]);
  hpr->roll_deg = rtk_parse_float(fields[4]);
  qf = (uint8_t)rtk_parse_int(fields[5]);
  hpr->qual = qf;
  if (n_fields >= 7U)
  {
    hpr->num_sats = (uint8_t)rtk_parse_int(fields[6]);
  }
  else
  {
    hpr->num_sats = 0U;
  }

  if (qf == (uint8_t)RTK_NMEA_QUAL_RTK_FIX)
  {
    hpr->valid = 1U;
    return 1U;
  }

  hpr->valid = 0U;
  return 0U;
}

/**
  * @brief  GNRMC / GPRMC …（表 7-12，仅解析 date 字段）
  *
  * | idx | 字段       | 映射              |
  * |-----|------------|-------------------|
  * |  2  | pos status | 须 'A' 才 valid   |
  * |  9  | date       | date_ddmmyy       |
  */
static uint8_t rtk_parse_rmc(char *fields[], uint8_t n_fields, RtkNmea_Rmc_t *rmc)
{
  if ((n_fields < 10U) || (rmc == NULL))
  {
    return 0U;
  }

  rtk_strip_checksum_field(fields[n_fields - 1U]);

  if (fields[2][0] != 'A')
  {
    rmc->valid = 0U;
    return 0U;
  }

  if (rtk_field_is_empty(fields[9]))
  {
    rmc->valid = 0U;
    return 0U;
  }

  rmc->date_ddmmyy = (uint32_t)rtk_parse_int(fields[9]);
  if (rmc->date_ddmmyy == 0U)
  {
    rmc->valid = 0U;
    return 0U;
  }

  rmc->valid = 1U;
  return 1U;
}

/**
  * @brief  GPHDT / GNHDT …（表 7-6 GPHDT 数据结构）
  *
  * | idx | 字段    | 映射           |
  * |-----|---------|----------------|
  * |  1  | heading | heading_deg    |
  * |  2  | T       | 须 'T' 才 valid |
  */
static uint8_t rtk_parse_hdt(char *fields[], uint8_t n_fields, RtkNmea_Hdt_t *hdt)
{
  if ((n_fields < 3U) || (hdt == NULL))
  {
    return 0U;
  }

  rtk_strip_checksum_field(fields[n_fields - 1U]);

  if (rtk_field_is_empty(fields[1]))
  {
    hdt->valid = 0U;
    return 0U;
  }

  if (fields[2][0] != 'T')
  {
    hdt->valid = 0U;
    return 0U;
  }

  hdt->heading_deg = rtk_parse_float(fields[1]);
  hdt->valid = 1U;
  return 1U;
}

/* ========================================================================== */
/*              语句分发：GNGGA + GNHPR + GPHDT + GNRMC(date)                  */
/* ========================================================================== */

uint8_t RtkNmea_ParseSentence(const char *sentence, RtkNmea_Data_t *out)
{
  char work[RTK_NMEA_SENTENCE_MAX];
  char id[16];
  char *fields[24];
  uint8_t n_fields;

  if ((sentence == NULL) || (out == NULL))
  {
    return 0U;
  }

  if (sentence[0] != '$')
  {
    return 0U;
  }

  if (strlen(sentence) >= sizeof(work))
  {
    return 0U;
  }

  if (RtkNmea_VerifyChecksum(sentence) == 0U)
  {
    return 0U;
  }

  strcpy(work, sentence);

  if (rtk_sentence_id(work, id, sizeof(id)) == 0U)
  {
    return 0U;
  }

  n_fields = rtk_split_fields(work, fields, 24U);

  if (strstr(id, "GGA") != NULL)
  {
    if (rtk_parse_gga(fields, n_fields, &out->gga) != 0U)
    {
      out->frame_count++;
      return 1U;
    }
    return 0U;
  }

  if (strstr(id, "HPR") != NULL)
  {
    if (rtk_parse_hpr(fields, n_fields, &out->tra) != 0U)
    {
      out->frame_count++;
      return 1U;
    }
    return 0U;
  }

  if (strstr(id, "HDT") != NULL)
  {
    if (rtk_parse_hdt(fields, n_fields, &out->hdt) != 0U)
    {
      out->frame_count++;
      return 1U;
    }
    return 0U;
  }

  if (strstr(id, "RMC") != NULL)
  {
    if (rtk_parse_rmc(fields, n_fields, &out->rmc) != 0U)
    {
      out->frame_count++;
      return 1U;
    }
    return 0U;
  }

  return 0U;
}
