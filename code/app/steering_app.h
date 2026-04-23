/*
 * steering_app.h
 *
 *  Created on: 2026年4月23日
 *      Author: Codex
 *  Description: 转向外环延迟与软启动接入控制接口
 */

#ifndef CODE_APP_STEERING_APP_H_
#define CODE_APP_STEERING_APP_H_

#include "zf_common_headfile.h"

// 上电或复位后，先让中环和内环单独运行的时间，单位：毫秒
#define STEERING_OUTER_LOOP_DELAY_MS         (500U)
// 外环开始接入后，从 0% 输出平滑拉升到 100% 输出的时间，单位：毫秒
#define STEERING_OUTER_LOOP_SOFT_START_MS    (500U)

void steering_init(void);                             // 初始化转向外环接入控制状态
void steering_reset_outer_loop_delay(void);          // 重置转向外环延迟与软启动状态
uint8_t steering_outer_loop_is_enabled(void);        // 查询转向外环是否已经开始参与控制
float steering_get_outer_loop_scale(void);           // 获取当前转向外环软启动缩放系数（0.0f ~ 1.0f）
void steering_run_outer_loop(float steering_error);  // 按延迟/软启动策略运行转向外环

#endif /* CODE_APP_STEERING_APP_H_ */
