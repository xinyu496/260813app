/**
  ******************************************************************************
  * @file    geo_los.h
  * @brief   地理坐标视线角计算模块（绝对方位角 / 俯仰角）
  *
  * 适用场景：
  *   - 平台保持水平，输出导航系下的绝对 yaw / pitch
  *   - 输入为 WGS84 经纬度（度）与地图海拔（米，同基准）
  *   - 典型作用距离 0~50 km，点动触发计算
  ******************************************************************************
  */
#ifndef GEO_LOS_H
#define GEO_LOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/** @brief 地理点：纬度/经度（度），海拔（米，地图 AMSL） */
typedef struct
{
  float lat_deg;  /**< 纬度，范围 [-90, 90] */
  float lon_deg;  /**< 经度，范围 [-180, 180] */
  float alt_m;    /**< 海拔高度（米），自身与目标须同基准 */
} GeoPoint_t;

/** @brief 绝对视线角输出 */
typedef struct
{
  float yaw_deg;    /**< 偏航角：真北顺时针，[0, 360) */
  float pitch_deg;  /**< 俯仰角：相对水平面，目标在上方为正，[-90, 90] */
} GeoLosAngle_t;

/* Exported constants --------------------------------------------------------*/

#define GEO_LOS_OK            0   /**< 计算成功 */
#define GEO_LOS_ERR_NULL     -1   /**< 空指针参数 */
#define GEO_LOS_ERR_LAT      -2   /**< 纬度非法 */
#define GEO_LOS_ERR_LON      -3   /**< 经度非法 */
#define GEO_LOS_ERR_COINCIDE -4   /**< 两点重合或距离过近 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  根据自身与目标经纬高，计算绝对偏航角与俯仰角
  * @param  self   自身位置（不可为 NULL）
  * @param  target 目标位置（不可为 NULL）
  * @param  out    输出角度（不可为 NULL）
  * @retval GEO_LOS_OK 成功；其他值为错误码
  *
  * @note   yaw  = atan2(E, N)，真北为 0°，顺时针增加
  * @note   pitch = atan2(U, 水平距离)，向上为正
  */
int32_t GeoLos_CalcAbsAngle(const GeoPoint_t *self,
                            const GeoPoint_t *target,
                            GeoLosAngle_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GEO_LOS_H */
