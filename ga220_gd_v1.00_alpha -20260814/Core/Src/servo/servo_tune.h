#ifndef _SERVO_TUNE_H_
#define _SERVO_TUNE_H_

#include "servo_controller.h" /* dev_servo_t / servo_axis_id_e / SERVO_AXIS_MAX */
#include "servo_axis.h"       /* servo_tune_param_e / set_param / get_param */

/** @brief 每轴一张在线整定表：仿真与上位机都写它，主循环统一下发 */
typedef struct {
    uint8_t enable;                  /**< 0=不干预(用现值) 1=按本表下发 */
    uint8_t idx;                     /**< 增益调度档位(FW 0~5, GD 恒 0) */
    uint8_t reserved[2];             /* 保证 val 4 字节对齐，sizeof 为 60 */
    float val[SERVO_TUNE_PARAM_MAX]; /**< 以 param_id 为下标的目标值 */
} servo_tune_t;

/** @brief 上位机调参帧(预留)：一帧 = 改一个参数 */
#pragma pack(1)
typedef struct {
    uint8_t head;     /**< 帧头 0xB5(预留) */
    uint8_t axis;     /**< servo_axis_id_e */
    uint8_t param_id; /**< servo_tune_param_e */
    uint8_t idx;      /**< 增益档位 */
    float val;        /**< 新值 */
    uint8_t sum;      /**< head..val 累加校验 */
} servo_tune_frame_t;
#pragma pack()        /* 必须加：恢复默认对齐 */

/* —— 统一入口：仿真/上位机两条路都走这几个 —— */
void servo_tune_sync(servo_tune_t *t, dev_servo_axis_t *ax);        /**< 用轴现值回填(开机调) */
void servo_tune_write(servo_tune_t *t, servo_tune_param_e id, uint8_t idx, float val);
void servo_tune_apply(dev_servo_axis_t *ax, const servo_tune_t *t); /**< 主循环每周期调 */
bool servo_tune_on_frame(servo_tune_t tbl[SERVO_AXIS_MAX], const uint8_t *buf, uint16_t len);

#endif                                                              /* _SERVO_TUNE_H_ */
