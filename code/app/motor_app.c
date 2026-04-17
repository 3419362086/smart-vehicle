/*
 * motor_app.c
 *
 *  Created on: 2025年11月12日
 *      Author: suiyungui
 *  Modified: 改为无刷FOC驱动接口
 */

#include "motor_app.h"

int16_t motor_rpm = 0;   // 电机反馈转速，单位：RPM
float wheel_speed = 0;   // 由转速换算得到的轮速，单位：m/s

/*
 * @brief 初始化电机驱动通信接口
 */
void motor_init(void)
{
    small_driver_uart_init();
}

/*
 * @brief 设置电机占空比
 * @param duty 双电机统一使用的占空比命令
 */
void motor_set_duty(int16_t duty)
{
    // 限幅 duty最大为-10000 ~ 10000 自行判断限幅
    if(duty > 5000) duty = 5000;
    if(duty < -5000) duty = -5000;

    small_driver_set_duty((int16)duty, (int16)duty);
}

/*
 * @brief 设置电机目标速度
 * @param speed 双电机统一使用的速度命令
 */
void motor_set_speed(int16_t speed)
{
    // 限幅保护
    if(speed > 3000) speed = 3000;
    if(speed < -3000) speed = -3000;

    small_driver_set_speed((int16)speed, (int16)speed);
}

/*
 * @brief 获取当前电机速度反馈
 */
void motor_get_speed(void)
{
    // 根据当前接线方向对左轮反馈取反，统一正方向定义
    motor_rpm =  -motor_value.receive_left_speed_data;
    wheel_speed = motor_rpm * RPM_TO_WHEEL;
//    printf("rpm:%d,speed:%f\r\n", motor_rpm, wheel_speed);
}

/*
 * @brief 电机占空比往返测试
 */
void motor_test(void)
{
    static int16_t duty = 0;      // 当前测试占空比
    static uint8_t duty_dir = 1;  // 占空比变化方向，1递增，0递减
    if(duty_dir)
    {
        duty += 100;
        if(duty >= 2000)
            duty_dir ^= 1;
    }
    else
    {
        duty -= 100;
        if(duty <= -2000)
            duty_dir ^= 1;
    }
    motor_set_duty(duty);
//    printf("%d\r\n",duty);
}
