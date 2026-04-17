/*
 * servo_app.h
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 *  Description: 舵机应用层接口，负责PWM占空比限幅与测试摆动
 */

#ifndef CODE_APP_SERVO_APP_H_
#define CODE_APP_SERVO_APP_H_

#include "zf_common_headfile.h"

/* 舵机PWM边界，需根据机械安装与实际舵机重新标定 */
#define l_max  4530             // 左打角极限占空比
#define mid    6030             // 舵机中值占空比
#define r_max  7530             // 右打角极限占空比

void servo_set(uint32_t duty); // 设置舵机占空比并自动限幅
void servo_test(void);         // 测试舵机往返摆动，便于标定极限和中值

#endif /* CODE_APP_SERVO_APP_H_ */
