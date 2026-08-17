/**
  ******************************************************************************
  * @file    geo_los.c
  * @brief   地理坐标视线角计算实现
  *
  * 算法流程：
  *   1. 地图海拔 + 轻量 geoid 近似 -> WGS84 椭球高
  *   2. 经纬高 -> ECEF 地心地固坐标
  *   3. 目标相对向量 -> 以自身为原点的 ENU（东-北-天）
  *   4. ENU 分量求 yaw / pitch
  *
  * 内存占用：无查表，仅运行时浮点运算，适合 STM32
  ******************************************************************************
  */

#include "geo_los.h"
#include "string.h"
#include <math.h>

/* Private constants ---------------------------------------------------------*/

#ifndef GEO_LOS_PI
#define GEO_LOS_PI 3.14159265358979323846f
#endif

#define GEO_LOS_DEG2RAD (GEO_LOS_PI / 180.0f)
#define GEO_LOS_RAD2DEG (180.0f / GEO_LOS_PI)

/** WGS84 椭球长半轴（米） */
#define WGS84_A   6378137.0f
/** WGS84 扁率 */
#define WGS84_F   (1.0f / 298.257223563f)
/** WGS84 第一偏心率平方 */
#define WGS84_E2  (WGS84_F * (2.0f - WGS84_F))

/** 判定两点重合的最小距离（米） */
#define GEO_LOS_MIN_COINCIDE_M 0.01f

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  角度转弧度
  */
static float geo_deg_to_rad(float deg)
{
  return deg * GEO_LOS_DEG2RAD;
}

/**
  * @brief  将偏航角归一化到 [0, 360)
  */
static float geo_normalize_yaw_deg(float yaw_deg)
{
  yaw_deg = fmodf(yaw_deg, 360.0f);
  if (yaw_deg < 0.0f)
  {
    yaw_deg += 360.0f;
  }
  return yaw_deg;
}

/**
  * @brief  大地水准面起伏 N（米），轻量公式近似
  *
  * 用于地图海拔 -> 椭球高：h = alt_amsl + N
  * 不占用 Flash 查表；同基准海拔下，50 km 内残差通常为数米级
  */
static float geo_geoid_undulation_m(float lat_deg, float lon_deg)
{
  const float lat = geo_deg_to_rad(lat_deg);
  const float lon = geo_deg_to_rad(lon_deg);

  return 9.0f * sinf(lat)
         + 3.0f * cosf(lat) * cosf(lon)
         + 2.0f * cosf(2.0f * lat) * sinf(lon);
}

/**
  * @brief  大地坐标 (lat, lon, 海拔) 转 ECEF (X, Y, Z)
  *
  * @param  lat_deg     纬度（度）
  * @param  lon_deg     经度（度）
  * @param  alt_amsl_m  地图海拔（米）
  * @param  x,y,z       输出 ECEF 坐标（米）
  */
static void geo_llh_to_ecef(float lat_deg, float lon_deg, float alt_amsl_m,
                            float *x, float *y, float *z)
{
  const float lat = geo_deg_to_rad(lat_deg);
  const float lon = geo_deg_to_rad(lon_deg);
  const float sin_lat = sinf(lat);
  const float cos_lat = cosf(lat);
  const float sin_lon = sinf(lon);
  const float cos_lon = cosf(lon);

  /* 正高转椭球高 */
  const float h = alt_amsl_m + geo_geoid_undulation_m(lat_deg, lon_deg);
  /* 卯酉圈曲率半径 */
  const float n = WGS84_A / sqrtf(1.0f - WGS84_E2 * sin_lat * sin_lat);

  *x = (n + h) * cos_lat * cos_lon;
  *y = (n + h) * cos_lat * sin_lon;
  *z = (n * (1.0f - WGS84_E2) + h) * sin_lat;
}

/**
  * @brief  ECEF 差分向量转以自身为原点的 ENU
  *
  * @param  dx,dy,dz  目标相对自身的 ECEF 向量（米）
  * @param  lat_deg   自身纬度，用于建立本地水平系
  * @param  lon_deg   自身经度
  * @param  e,n,u     东、北、天方向分量（米）
  */
static void geo_ecef_delta_to_enu(float dx, float dy, float dz,
                                  float lat_deg, float lon_deg,
                                  float *e, float *n, float *u)
{
  const float lat = geo_deg_to_rad(lat_deg);
  const float lon = geo_deg_to_rad(lon_deg);
  const float sin_lat = sinf(lat);
  const float cos_lat = cosf(lat);
  const float sin_lon = sinf(lon);
  const float cos_lon = cosf(lon);

  /* 旋转矩阵：ECEF -> ENU */
  *e = (-sin_lon * dx) + (cos_lon * dy);
  *n = (-sin_lat * cos_lon * dx) - (sin_lat * sin_lon * dy) + (cos_lat * dz);
  *u = (cos_lat * cos_lon * dx) + (cos_lat * sin_lon * dy) + (sin_lat * dz);
}

/**
  * @brief  校验输入地理点范围
  */
static int32_t geo_validate_point(const GeoPoint_t *point)
{
  if (point->lat_deg < -90.0f || point->lat_deg > 90.0f)
  {
    return GEO_LOS_ERR_LAT;
  }

  if (point->lon_deg < -180.0f || point->lon_deg > 180.0f)
  {
    return GEO_LOS_ERR_LON;
  }

  return GEO_LOS_OK;
}

/* Exported functions --------------------------------------------------------*/

int32_t GeoLos_CalcAbsAngle(const GeoPoint_t *self,
                            const GeoPoint_t *target,
                            GeoLosAngle_t *out)
{
  float sx;
  float sy;
  float sz;
  float tx;
  float ty;
  float tz;
  float dx;
  float dy;
  float dz;
  float e;
  float n;
  float u;
  float horiz;
  int32_t status;

  /* 参数检查 */
  if (self == NULL || target == NULL || out == NULL)
  {
    return GEO_LOS_ERR_NULL;
  }

  status = geo_validate_point(self);
  if (status != GEO_LOS_OK)
  {
    return status;
  }

  status = geo_validate_point(target);
  if (status != GEO_LOS_OK)
  {
    return status;
  }

  /* 两步：经纬高 -> ECEF */
  geo_llh_to_ecef(self->lat_deg, self->lon_deg, self->alt_m, &sx, &sy, &sz);
  geo_llh_to_ecef(target->lat_deg, target->lon_deg, target->alt_m, &tx, &ty, &tz);

  /* 目标相对向量（ECEF） */
  dx = tx - sx;
  dy = ty - sy;
  dz = tz - sz;

  /* 距离过近无法定义方位 */
  if ((dx * dx + dy * dy + dz * dz) < (GEO_LOS_MIN_COINCIDE_M * GEO_LOS_MIN_COINCIDE_M))
  {
    return GEO_LOS_ERR_COINCIDE;
  }

  /* ECEF 差分 -> 本地 ENU */
  geo_ecef_delta_to_enu(dx, dy, dz, self->lat_deg, self->lon_deg, &e, &n, &u);

  /* 水平距离与视线角 */
  horiz = sqrtf((e * e) + (n * n));
  out->yaw_deg = geo_normalize_yaw_deg(atan2f(e, n) * GEO_LOS_RAD2DEG);
  out->pitch_deg = atan2f(u, horiz) * GEO_LOS_RAD2DEG;

  return GEO_LOS_OK;
}
