/**

  ******************************************************************************

  * @file    rtk_nmea_parse.h

  * @brief   RTK NMEA 解析：GNGGA（位置）+ GNHPR（三姿）+ GPHDT（航向）

  *

  * 【已支持语句】

  *   GNGGA / GPGGA / BDGGA …   本机 WGS84 经纬高 + qual

  *   GNHPR / GPHPR / BDHPR …   航向+俯仰+横滚 + QF 解状态

  *   GPHDT / GNHDT …           真北航向（HEADING 模式，表 7-6）

  *   GNRMC / GPRMC …           仅 date 字段（ddmmyy）

  *

  * 【故意忽略】GSA、GSV、VTG、HEADINGA、GPTRA 等，返回 0 不报错。

  *

  * 【定基座 / 地理引导】

  *   位置  gga (qual==4)  → lxy_self

  *   三姿  hpr (QF==4)    → INS.Yaw / Pitch / Roll（存于 tra 字段）

  *   航向  hdt (T)        → GNHPR 无效时 fallback 更新 INS.Yaw

  *

  * 【坐标】GGA 原始为度分(DDM)，解析后十进制度，对接 geo_los。

  ******************************************************************************

  */

#ifndef RTK_NMEA_PARSE_H

#define RTK_NMEA_PARSE_H



#ifdef __cplusplus

extern "C" {

#endif



#include <stdint.h>



/** 与 rtk_nmea_rx.h 一致，单句最大长度 */

#define RTK_NMEA_SENTENCE_MAX        256U



/* -------------------------------------------------------------------------- */

/*                         GNGGA qual / GNHPR QF                               */

/* -------------------------------------------------------------------------- */



/**

  * @brief GNGGA qual / GNHPR QF（datasheet 表 7-1 / 7-42，数值一致）

  * @note  业务层建议：位置与三姿均须 qual/QF==RTK_NMEA_QUAL_RTK_FIX

  */

typedef enum

{

  RTK_NMEA_QUAL_INVALID   = 0,  /**< 定位/姿态不可用 */

  RTK_NMEA_QUAL_SPP       = 1,  /**< 单点定位，米级 */

  RTK_NMEA_QUAL_DGPS      = 2,  /**< 伪距差分 / SBAS，亚米级 */

  RTK_NMEA_QUAL_RTK_FIX   = 4,  /**< RTK 固定解，厘米级 */

  RTK_NMEA_QUAL_RTK_FLOAT = 5,  /**< RTK 浮点解，分米级 */

  RTK_NMEA_QUAL_INS       = 6,  /**< 惯导定位 */

  RTK_NMEA_QUAL_FIXED_POS = 7   /**< 用户设定固定位置模式 */

} RtkNmea_Qual_e;



/* -------------------------------------------------------------------------- */

/*                         解析结果结构体                                      */

/* -------------------------------------------------------------------------- */



/**

  * @brief GNGGA 解析结果

  *

  * 示例：$GNGGA,073551.00,2243.08...,N,11401.10...,E,4,28,0.6,102.6,M,,*6F

  */

typedef struct

{

  uint8_t  valid;       /**< 1=经纬有效且 qual 可用 */

  float    utc_hhmmss;

  double   lat_deg;

  double   lon_deg;

  float    alt_m;

  uint8_t  qual;

  uint8_t  num_sats;

  float    hdop;

} RtkNmea_Gga_t;



/**

  * @brief GNHPR 三姿解析结果（亦兼容旧字段名 tra）

  *

  * 示例：$GNHPR,081212.00,341.48,-00.64,000.00,4,46,0.00,0999*4F

  *        UTC        航向   俯仰    横滚   QF

  */

typedef struct

{

  uint8_t  valid;         /**< 1=QF==RTK Fixed */

  float    utc_hhmmss;

  float    heading_deg;   /**< 航向 Yaw，0~360° */

  float    pitch_deg;     /**< 俯仰 Pitch，-90~90° */

  float    roll_deg;      /**< 横滚 Roll，-90~90° */

  uint8_t  qual;          /**< QF 解状态，同 RtkNmea_Qual_e */

  uint8_t  num_sats;

} RtkNmea_Tra_t;



/**

  * @brief GNRMC 解析结果（仅 date）

  *

  * 示例：$GNRMC,...,301221,... → date_ddmmyy = 301221（30 日 12 月 21 年）

  */

typedef struct

{

  uint8_t  valid;           /**< 1=pos status=='A' 且 date 非空 */

  uint32_t date_ddmmyy;     /**< 日期 ddmmyy，如 301221 */

} RtkNmea_Rmc_t;



/**

  * @brief GPHDT 航向解析结果（表 7-6）

  *

  * 示例：$GPHDT,178.7236,T*15

  *        航向角(°)  真北指示

  */

typedef struct

{

  uint8_t  valid;           /**< 1=heading 非空且指示符为 T */

  float    heading_deg;     /**< 真北航向，0~360° */

} RtkNmea_Hdt_t;



/** @deprecated 保留占位，不再解析 */

typedef struct

{

  uint8_t  valid;

  uint8_t  sol_computed;

  float    heading_deg;

  float    pitch_deg;

  float    baseline_m;

} RtkNmea_HeadingA_t;



typedef struct

{

  RtkNmea_Gga_t      gga;

  RtkNmea_Rmc_t      rmc;

  RtkNmea_Tra_t      tra;       /**< GNHPR 三姿（Yaw/Pitch/Roll） */

  RtkNmea_Hdt_t      hdt;       /**< GPHDT 真北航向 */

  RtkNmea_HeadingA_t heading;

  uint32_t           frame_count;

} RtkNmea_Data_t;



extern RtkNmea_Data_t g_rtk_nmea_data;



double RtkNmea_DdmToDeg(const char *ddm, char dir);

uint8_t RtkNmea_VerifyChecksum(const char *sentence);

uint8_t RtkNmea_ParseSentence(const char *sentence, RtkNmea_Data_t *out);



#ifdef __cplusplus

}

#endif



#endif /* RTK_NMEA_PARSE_H */


