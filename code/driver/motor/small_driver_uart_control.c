#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 电机通讯参数结构体

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback();
// 备注信息     用于接收驱动返回的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // 临时接收变量

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前缓冲区首位是否已正确存入帧头
        {
            motor_value.receive_data_count = 0;                                             // 未收到帧头或者未正确存入帧头则重新计数
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // 存储串口数据

        if(motor_value.receive_data_count >= 7)                                             // 判断是否已收到指定数量的数据
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {

                motor_value.sum_check_data = 0;                                             // 清空校验位数据

                for(int i = 0; i < 6; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[6])        // 校验数据准确性
                {

                    if(motor_value.receive_data_buffer[1] == 0x02)                          // 判断是否正确接收到 速度数据 功能号
                    {
                        motor_value.receive_left_speed_data  = (((int)motor_value.receive_data_buffer[2] << 8) | (int)motor_value.receive_data_buffer[3]);  // 读取左侧转速数据

                        motor_value.receive_right_speed_data = (((int)motor_value.receive_data_buffer[4] << 8) | (int)motor_value.receive_data_buffer[5]);  // 读取右侧转速数据
                    }

                    motor_value.receive_data_count = 0;                                     // 清空接收计数数值

                    memset(motor_value.receive_data_buffer, 0, 7);                          // 清空接收缓冲数组
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // 清空接收计数数值

                    memset(motor_value.receive_data_buffer, 0, 7);                          // 清空接收缓冲数组
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // 清空接收计数数值

                memset(motor_value.receive_data_buffer, 0, 7);                              // 清空接收缓冲数组
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置占空比
// 参数说明     left_duty       左侧占空比  范围 -10000 ~ 10000  正数为正转
// 参数说明     right_duty      右侧占空比  范围 -10000 ~ 10000  正数为正转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 设置帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 设置功能号

    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 填入 左占空比 的高八位

    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 填入 左占空比 的低八位

    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 填入 右侧占空比 的高八位

    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 填入 右侧占空比 的低八位

    motor_value.send_data_buffer[6] = 0;                                            // 清空校验数据

    for(int i = 0; i < 6; i ++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i];         // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);                     // 发送设置占空比的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置目标速度
// 参数说明     left_speed      左侧目标速度  单位：RPM
// 参数说明     right_speed     右侧目标速度  单位：RPM
// 返回参数     void
// 使用示例     small_driver_set_speed(1000, -1000);
// 备注信息     功能号 0x04  驱动端收到后切换为速度环模式并设置目标速度
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_speed(int16 left_speed, int16 right_speed)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 设置帧头

    motor_value.send_data_buffer[1] = 0x04;                                         // 设置功能号

    motor_value.send_data_buffer[2] = (uint8)((left_speed & 0xFF00) >> 8);          // 填入 左侧目标速度 的高八位

    motor_value.send_data_buffer[3] = (uint8)(left_speed & 0x00FF);                 // 填入 左侧目标速度 的低八位

    motor_value.send_data_buffer[4] = (uint8)((right_speed & 0xFF00) >> 8);         // 填入 右侧目标速度 的高八位

    motor_value.send_data_buffer[5] = (uint8)(right_speed & 0x00FF);                // 填入 右侧目标速度 的低八位

    motor_value.send_data_buffer[6] = 0;                                            // 清空校验数据

    for(int i = 0; i < 6; i ++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i];         // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);          // 发送设置目标速度的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     需要先发送一次 驱动板才会在返回速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 设置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 设置功能号

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清零

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位清零

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位清零

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位清零

    motor_value.send_data_buffer[6] = 0xA7;                                         // 设置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 7);                             // 清空发送缓冲数组

    memset(motor_value.receive_data_buffer, 0, 7);                          // 清空接收缓冲数组

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;

    motor_value.receive_right_speed_data    = 0;

    motor_value.receive_left_speed_data     = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化

    small_driver_set_duty(0, 0);                                                                // 发送0占空比

    small_driver_get_speed();                                                                   // 获取实时速度数据
}
