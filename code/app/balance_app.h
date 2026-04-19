/*
 * balance_app.h
 *
 *  Created on: 2026年4月5日
 *      Author: suiyungui
 *  Description: 单车平衡串级PID控制（转向环→角度环→角速度环→舵机）
 */

#ifndef CODE_APP_BALANCE_APP_H_
#define CODE_APP_BALANCE_APP_H_

#include "zf_common_headfile.h"


extern PID_T steering_pid;      // 转向环PID
extern PID_T angle_pid;         // 角度环PID
extern PID_T gyro_pid;          // 角速度环PID

extern float target_angle;      // 目标横滚角度（转向环输出，对应变量 pitch）
extern float target_gyro_rate;  // 目标角速度（角度环输出）
extern float servo_output;      // 舵机PWM偏移输出值

void balance_init(void);                              // 初始化串级PID参数
void balance_steering_loop(float steering_error);    // 外环，建议20ms调用一次
void balance_angle_loop(void);                       // 中环，建议10ms调用一次
void balance_gyro_loop(void);                        // 内环，建议2ms调用一次
void balance_reset_all_pid(void);                    // 清空三个PID环的历史状态
void pid_test(void);                                 // 输出PID相关调试量
#endif /* CODE_APP_BALANCE_APP_H_ */
