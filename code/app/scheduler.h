/*
 * scheduler.h
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 *  Description: 简易时间片调度器接口，基于毫秒节拍轮询执行应用任务
 */

#ifndef CODE_APP_SCHEDULER_H_
#define CODE_APP_SCHEDULER_H_

#include "zf_common_headfile.h"

extern volatile uint32_t uwtick;   // 系统毫秒节拍，由中断或定时基准更新
void scheduler_init(void);     // 初始化任务数量统计
void scheduler_run(void);      // 轮询任务表，执行到期任务

#endif /* CODE_APP_SCHEDULER_H_ */
