/*
 * imu.c
 *
 *  Created on: 2025年4月22日
 *      Author: suiyungui
 */
#include "imu_app.h"

float roll = 0;              // 当前俯仰角，单位：度
float pitch = 0;             // 当前横滚角，单位：度
float yaw = 0;               // 当前航向角，单位：度，已减去上电标定零点
float gyro_x_rate = 0;       // 当前X轴角速度，单位：度/秒
float gyro_y_rate = 0;       // 当前Y轴角速度，单位：度/秒
float gyro_z_rate = 0;       // 当前Z轴角速度，单位：度/秒
static float yaw_offset = 0; // 航向零点偏移量，在初始化标定完成后记录

/*
 * @brief 发送姿态数据到上位机
 * @param roll 俯仰角，单位：度
 * @param pitch 横滚角，单位：度
 * @param yaw 航向角，单位：度
 * @param fusion_sta 姿态融合状态标志
 */
void usart_send(float roll, float pitch, float yaw, uint8_t fusion_sta)
{
    uint8_t buffer[15];   // 匿名飞控上位机协议数据帧缓冲区
    uint8_t sumcheck = 0; // 一次累加校验
    uint8_t addcheck = 0; // 二次累加校验
    uint8_t index = 0;    // 当前写入位置

    // 将欧拉角转换为 int16，并放大100倍
    int16_t roll_int = (int16_t)(roll * 100.0f);
    int16_t pitch_int = (int16_t)(pitch * 100.0f);
    int16_t yaw_int = (int16_t)(yaw * 100.0f);

    // 帧头 (0xAB)
    buffer[index++] = 0xAB;
    // 源地址 (假设为 0xDC, 匿名飞控的默认地址)
    buffer[index++] = 0xDC;
    // 目标地址 (0xFE, 上位机地址)
    buffer[index++] = 0xFE;
    // 功能码 (ID: 0x03 表示飞控姿态：欧拉角格式)
    buffer[index++] = 0x03;
    // 数据长度 (7字节数据)
    buffer[index++] = 7;
    buffer[index++] = 0;  // 数据长度高字节为0

    // 欧拉角数据 (int16, 角度扩大100倍)
    buffer[index++] = (uint8_t)(roll_int & 0xFF);
    buffer[index++] = (uint8_t)((roll_int >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(pitch_int & 0xFF);
    buffer[index++] = (uint8_t)((pitch_int >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(yaw_int & 0xFF);
    buffer[index++] = (uint8_t)((yaw_int >> 8) & 0xFF);

    // 融合状态 (uint8)
    buffer[index++] = fusion_sta;

    // 计算校验和和附加校验 (从帧头开始到DATA区结束)
    for (int i = 0; i < index; i++)
    {
        sumcheck += buffer[i];
        addcheck += sumcheck;
    }

    // 添加校验和和附加校验值
    buffer[index++] = sumcheck;
    buffer[index++] = addcheck;

    // 发送数据帧
    for (int i = 0; i < index; i++)
    {
        printf("%c", buffer[i]);
    }
}

/*
 * @brief 初始化IMU并完成零偏标定
 */
void imu_all_init(void)
{
    while(imu660rc_init(IMU660RC_QUARTERNION_480HZ))
    {
        // 初始化失败重试
        system_delay_ms(100);
    }

    // 校准期间关闭INT2中断，防止ISR与主程序同时操作SPI导致死锁
    exti_disable(IMU660RC_INT2_PIN);
    imu660rc_bias_calibrate(2000);
    exti_enable(IMU660RC_INT2_PIN);

    system_delay_ms(200);
    yaw_offset = imu660rc_yaw;  // 记录校准结束后的yaw，用于归零
    imu_proc();                 // 处理一次数据，更新全局变量

}

/*
 * @brief 读取并更新IMU姿态与角速度结果
 */
void imu_proc(void)
{
    // 注意：当前工程中变量命名与物理姿态含义相反，pitch 表示横滚角，roll 表示俯仰角
    pitch = imu660rc_pitch;
    roll  = imu660rc_roll;
    yaw   = imu660rc_yaw - yaw_offset;
    gyro_x_rate  = imu660rc_gyro_transition(imu660rc_gyro_x);
    gyro_y_rate  = imu660rc_gyro_transition(imu660rc_gyro_y);
    gyro_z_rate  = imu660rc_gyro_transition(imu660rc_gyro_z);

    if (yaw > 180)  yaw -= 360;
    if (yaw < -180) yaw += 360;
}

/*
 * @brief 输出姿态数据，供串口上位机调试
 */
void imu_test(void)
{
//    usart_send(roll, pitch, yaw, 1);
    //   printf("%f,%f\r\n", yaw, gyro_z_rate);
    //    printf("%f,%f,%f\r\n", roll, pitch, yaw);
       wireless_uart_printf("%f,%f,%f\r\n", roll, pitch, yaw);

    // printf("%f,%f,%f\r\n", gyro_x_rate, gyro_y_rate, gyro_z_rate);
//    JustFloat_Test_two(yaw,gyro_z_rate);
}
