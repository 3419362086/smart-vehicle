/*
 * balance_app.c
 *
 *  Created on: 2026年4月5日
 *      Author: suiyungui
 *  Description: 单车平衡串级PID控制
 *      转向环(20ms) → 角度环(10ms) → 角速度环(2ms) → 舵机PWM
 *      反馈量：pitch（横滚角，左倾为正）、gyro_y_rate
 */

#include "balance_app.h"

/* 三个PID实例 */
PID_T steering_pid;         // 转向环：转向误差 -> 目标角度
PID_T angle_pid;            // 角度环：目标横滚角 - pitch(横滚角) -> 目标角速度
PID_T gyro_pid;             // 角速度环：目标角速度 - gyro_y_rate -> 舵机PWM偏移

/* 级联中间变量 */
float target_angle = 0;         // 转向环输出，作为横滚角控制目标值
float target_gyro_rate = 0;     // 角度环输出，作为角速度环目标值
float servo_output = 0;         // 角速度环输出的舵机PWM偏移量

/*
 * @brief 串级PID初始化
 * @note  初始化 PID 参数后，同时复位转向外环的延迟/软启动接入状态
 */
void balance_init(void)
{
    //                          kp    ki    kd    target  limit
    pid_init(&steering_pid,     1.0f, 0.0f, 0.0f, 0.0f,  30.0f);      // 输出限幅: ±30度
    pid_init(&angle_pid,        13.0f, 0.0f, 0.6f, 0.0f,  500.0f);     // 输出限幅: ±200度/秒
    pid_init(&gyro_pid,         8.0f, 0.0f, 0.0f, 0.0f,  15000.0f);    // 只用P，机械阻尼足够

    // 角度环积分限幅：防止积分饱和
    // 限幅 = 输出限幅/Ki ≈ 200/0.8 = 250，保守取±200
    pid_app_limit_integral(&angle_pid, -200.0f, 200.0f);

    steering_init();   // 转向外环默认先延迟，再按软启动斜坡慢慢接入
}

/*
 * @brief 外环 - 转向环 (20ms调用)
 * @param steering_error 转向误差目标航向
 * @note  输出为目标横滚角度，存入 target_angle
 */
void balance_steering_loop(float steering_error)
{
    // 将航向或转向误差转换为目标倾角
    target_angle = pid_calculate_positional(&steering_pid, steering_error);
}

/*
 * @brief 中环 - 角度环 (10ms调用)
 * @note  目标值来自转向环输出(target_angle)，反馈值为当前pitch（此工程中 pitch 表示横滚角）
 *        输出为目标角速度，存入 target_gyro_rate
 */
void balance_angle_loop(void)
{
    // 根据当前横滚角误差计算期望角速度
    pid_set_target(&angle_pid, target_angle);
    target_gyro_rate = pid_calculate_positional(&angle_pid, pitch);
}

/*
 * @brief 内环 - 角速度环 (2ms调用)
 * @note  目标值来自角度环输出(target_gyro_rate)，反馈值为当前gyro_y_rate
 *        输出为舵机PWM偏移量，叠加到舵机中值后输出
 */
void balance_gyro_loop(void)
{
    // 根据角速度误差输出最终舵机控制量
    pid_set_target(&gyro_pid, target_gyro_rate);
    servo_output = pid_calculate_positional(&gyro_pid, gyro_y_rate);
    servo_set((uint32_t)(mid + (int32_t)servo_output));
}

/*
 * @brief 清空三个串级PID的积分项与历史状态
 * @note  PID 复位后重新开始转向外环延迟与软启动，避免保护恢复时上层目标突然跳变
 */
void balance_reset_all_pid(void)
{
    pid_reset(&steering_pid);
    pid_reset(&angle_pid);
    pid_reset(&gyro_pid);
    target_angle = 0.0f;
    target_gyro_rate = 0.0f;
    servo_output = 0.0f;
    steering_reset_outer_loop_delay();
}

/*
 * @brief 输出内环调参需要观察的关键量
 * @note  通道顺序固定为：
 *        1. target_gyro_rate：内环目标角速度
 *        2. gyro_y_rate：当前角速度反馈
 *        3. gyro_pid.error：内环角速度误差
 *        4. servo_output：内环输出控制量
 *        5. pitch：当前横滚角，辅助判断舵机是否已把车体带偏
 */
void pid_test(void)
{
    // 侧倒保护期间暂停 5 路波形连续输出，退出保护后自动恢复。
    if (motor_guard_is_active())
    {
        return;
    }
    //调内环
    // wireless_uart_printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                        // target_gyro_rate, gyro_y_rate, gyro_pid.error, servo_output, pitch);
    wireless_uart_printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                        wheel_speed, yaw, gyro_z_rate, pitch, servo_output);
}
