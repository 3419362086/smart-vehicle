/*
 * motor_app.h
 *
 *  Created on: 2025年11月12日
 *      Author: suiyungui
 *  Description: 电机应用层接口，负责速度/占空比指令下发与测速量更新
 */

#ifndef CODE_APP_MOTOR_APP_H_
#define CODE_APP_MOTOR_APP_H_

#include "zf_common_headfile.h"

#define WHEEL_DIAMETER              0.064f       // 轮子直径，单位：米

#define RPM_TO_WHEEL                (WHEEL_DIAMETER * PI / 60)      // 电机转速RPM换算为轮速的比例系数

#define MOTOR_ROLL_CUTOFF_DEG   (16.0f)// 横滚角超过此值即进入保护态，单位：度
#define MOTOR_ROLL_RECOVER_DEG  (14.0f)// 横滚角低于此值即恢复输出，单位：度

extern int16_t motor_rpm;       // 当前电机转速反馈，单位：RPM
extern float wheel_speed;       // 当前轮速反馈，单位：m/s

void motor_init(void);               // 初始化电机驱动通信接口

void motor_set_duty(int16_t duty);   // 设置双电机占空比，内部带限幅与横滚保护

void motor_set_speed(int16_t speed);  // 设置双电机目标速度，内部带限幅与横滚保护

void motor_get_speed(void);            // 读取反馈转速并换算轮速

void motor_guard_update(void);         // 刷新横滚保护状态，并在首次触发时主动下发0命令

void motor_test(void);               // 电机正反转测试

#endif /* CODE_APP_MOTOR_APP_H_ */
