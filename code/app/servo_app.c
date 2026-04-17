/*
 * servo_app.c
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 */

#include "servo_app.h"

/*
 * @brief 设置舵机占空比
 * @param duty 目标占空比
 */
void servo_set(uint32_t duty)
{
    // 对输出占空比做机械极限保护，避免舵机撞限位
    if(duty <= l_max)
        duty = l_max;
    if(duty >= r_max)
        duty = r_max;
    pwm_set_duty(ATOM1_CH1_P33_9, duty); // ATOM1_CH1_P33_9 非固定引脚
}

/*
 * @brief 舵机往返测试
 * @note 用于观察舵机左右极限并辅助标定中值
 */
void servo_test(void)
{
    static uint16_t servo_motor_duty = mid; // 当前测试占空比
    static uint8_t servo_motor_dir = 0;     // 摆动方向标志，0向左，1向右
    if (servo_motor_dir)
    {
        servo_motor_duty += 10;
        if (servo_motor_duty >= r_max)
        {
            servo_motor_dir = 0x00;
        }
    }
    else
    {
        servo_motor_duty -= 10;
        if (servo_motor_duty <= l_max)
        {
            servo_motor_dir = 0x01;
        }
    }
    servo_set(servo_motor_duty);
//    printf("duty:%d\r\n",servo_motor_duty);
}
