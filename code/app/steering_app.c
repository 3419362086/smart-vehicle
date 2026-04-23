/*
 * steering_app.c
 *
 *  Created on: 2026年4月23日
 *      Author: Codex
 *  Description: 转向外环延迟与软启动接入逻辑，中环和内环先稳定，再平滑接入转向外环
 */

#include "steering_app.h"

static volatile uint32_t steering_outer_loop_reset_tick = 0U;        // 最近一次复位时刻
static volatile uint32_t steering_outer_loop_soft_start_tick = 0U;   // 软启动开始时刻
static volatile uint8_t steering_outer_loop_soft_start_active = 0U;  // 软启动是否已经开始
static volatile uint8_t steering_outer_loop_enabled = 0U;            // 外环是否已经开始参与控制

/*
 * @brief 初始化转向外环接入控制状态
 * @note 上电后默认从“延迟等待”状态开始
 */
void steering_init(void)
{
    steering_reset_outer_loop_delay();
}

/*
 * @brief 重置转向外环延迟与软启动状态
 * @note 侧倒保护恢复或 PID 整体复位后，会重新经历“先等待、再缓慢接入”的过程
 */
void steering_reset_outer_loop_delay(void)
{
    steering_outer_loop_reset_tick = uwtick;
    steering_outer_loop_soft_start_tick = 0U;
    steering_outer_loop_soft_start_active = 0U;
    steering_outer_loop_enabled = 0U;
}

/*
 * @brief 获取当前转向外环软启动缩放系数
 * @return 0.0f 表示外环尚未接入，1.0f 表示外环已全量接入
 * @note 缩放系数会在延迟结束后，从 0 平滑增加到 1，避免 target_angle 阶跃变化
 */
float steering_get_outer_loop_scale(void)
{
    uint32_t delay_elapsed_ms = (uint32_t)(uwtick - steering_outer_loop_reset_tick);
    uint32_t soft_start_elapsed_ms = 0U;

    // 延迟阶段内完全屏蔽外环，让中环和内环先稳定下来。
    if (delay_elapsed_ms < STEERING_OUTER_LOOP_DELAY_MS)
    {
        return 0.0f;
    }

    // 第一次越过延迟门槛时，记录软启动起点。
    if (!steering_outer_loop_soft_start_active)
    {
        steering_outer_loop_soft_start_active = 1U;
        steering_outer_loop_soft_start_tick = uwtick;
        steering_outer_loop_enabled = 1U;
    }

    // 软启动时长配置为 0 时，直接按全量接入处理，避免除零。
    if (STEERING_OUTER_LOOP_SOFT_START_MS == 0U)
    {
        return 1.0f;
    }

    soft_start_elapsed_ms = (uint32_t)(uwtick - steering_outer_loop_soft_start_tick);
    if (soft_start_elapsed_ms >= STEERING_OUTER_LOOP_SOFT_START_MS)
    {
        return 1.0f;
    }

    return ((float)soft_start_elapsed_ms) / ((float)STEERING_OUTER_LOOP_SOFT_START_MS);
}

/*
 * @brief 查询转向外环是否已经开始参与控制
 * @return 1 表示已进入软启动或已全量接入，0 表示仍处于纯延迟阶段
 */
uint8_t steering_outer_loop_is_enabled(void)
{
    (void)steering_get_outer_loop_scale();
    return steering_outer_loop_enabled;
}

/*
 * @brief 按延迟 + 软启动策略运行转向外环
 * @param steering_error 外环输入误差
 * @note 先按原始外环 PID 计算目标倾角，再用软启动系数缩放输出，避免上层目标一步到位
 */
void steering_run_outer_loop(float steering_error)
{
    float outer_loop_scale = steering_get_outer_loop_scale();

    if (outer_loop_scale <= 0.0f)
    {
        return;
    }

    // 先保持原有外环 PID 算法不变，再对输出结果做软启动缩放。
    balance_steering_loop(steering_error);
    target_angle *= outer_loop_scale;
}
