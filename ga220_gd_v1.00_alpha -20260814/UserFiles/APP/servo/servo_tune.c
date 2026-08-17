#include "servo_tune.h"
#include <string.h>

/* 用轴当前参数回填整定表，避免 enable 后用 0 覆盖现值 */
void servo_tune_sync(servo_tune_t *t, dev_servo_axis_t *ax)
{
    if (!t || !ax) return;
    for (uint8_t id = 0; id < SERVO_TUNE_PARAM_MAX; id++)
        t->val[id] = dev_servo_axis_get_param(ax, (servo_tune_param_e)id, 0);
    t->idx = 0u;
    t->enable = 0u;
}

/* 写一条到整定表（仿真/上位机共用） */
void servo_tune_write(servo_tune_t *t, servo_tune_param_e id, uint8_t idx, float val)
{
    if (!t || id >= SERVO_TUNE_PARAM_MAX) return;
    t->val[id] = val;
    t->idx = idx;
    t->enable = 1u;
}

/* 整表下发到轴（enable 时生效）——唯一落地点 */
void servo_tune_apply(dev_servo_axis_t *ax, const servo_tune_t *t)
{
    if (!ax || !t || !t->enable) return;
    for (uint8_t id = 0; id < SERVO_TUNE_PARAM_MAX; id++)
        dev_servo_axis_set_param(ax, (servo_tune_param_e)id, t->idx, t->val[id]);
}

/* 上位机调参帧解析(预留)：校验 → 写表 */
bool servo_tune_on_frame(servo_tune_t tbl[SERVO_AXIS_MAX], const uint8_t *buf, uint16_t len)
{
    if (!tbl || !buf || len < sizeof(servo_tune_frame_t)) return false;

    servo_tune_frame_t f;
    memcpy(&f, buf, sizeof(f));
    if (f.head != 0xB5u || f.axis >= SERVO_AXIS_MAX || f.param_id >= SERVO_TUNE_PARAM_MAX) return false;

    uint8_t sum = 0;
    for (uint16_t i = 0; i < sizeof(f) - 1; i++)
        sum += buf[i];
    if (sum != f.sum) return false;

    servo_tune_write(&tbl[f.axis], (servo_tune_param_e)f.param_id, f.idx, f.val);
    return true;
}
