/*
 * imu_app.h
 *
 *  Created on: 2025年11月14日
 *      Author: suiyungui
 *  Description: IMU应用层接口，负责姿态角与角速度数据的初始化、更新和调试输出
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_//
#include "zf_common_headfile.h"

extern float roll;          // 俯仰角，单位：度
extern float pitch;         // 横滚角，单位：度
extern float yaw;           // 航向角，单位：度，已做零点偏移
extern float gyro_x_rate;   // X轴角速度，单位：度/秒
extern float gyro_y_rate;   // Y轴角速度，单位：度/秒
extern float gyro_z_rate;   // Z轴角速度，单位：度/秒

void imu_proc(void);        // 更新姿态角与角速度全局量
void imu_all_init(void);    // 初始化IMU并完成零偏标定
void imu_test(void);        // 输出姿态数据，供上位机调试

#endif /* CODE_IMU_H_ */
