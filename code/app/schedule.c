/*
 * schedule.c
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 */

#include "scheduler.h"

uint8_t task_num;      // 当前调度器已注册任务数量
uint32_t uwtick = 0;   // 全局毫秒节拍计数

typedef struct {
    void (*task_func)(void); // 任务入口函数
    uint32_t rate_ms;        // 任务周期，单位：毫秒
    uint32_t last_run;       // 上次运行时刻，单位：毫秒
} task_t;

// 静态任务表，根据调试需求按需启用对应任务
static task_t scheduler_task[] =
{
//        {servo_test,10,0},   // 舵机测试 不需要的时候关掉
    //    {motor_test,100,0},   // 电机测试 不需要的时候关掉
        // {imu_test,10,0},      // imu测试 不需要的时候关掉
    //    {motor_get_speed,10,0},  // 电机速度获取测试 不需要的时候关掉
        {wireless_uart_pid_service, 5, 0},  // 先处理无线PID命令
        {pid_test, 10, 0},                  // 再按状态发送PID测试流
        {motor_guard_update, 10, 0},        // 侧倒保护
};
/**
 * @brief 调度器初始化函数
 * @note 统计任务表中的任务数量，供运行时遍历使用
 */
void scheduler_init(void)
{
    // 计算任务数组的元素个数，并将结果存储在 task_num 中
    task_num = sizeof(scheduler_task) / sizeof(task_t);
}
/**
 * @brief 调度器运行函数
 * @note 轮询所有任务，周期到达后执行对应函数
 */
void scheduler_run(void)
{
    // 遍历任务数组中的所有任务
    for (uint8_t i = 0; i < task_num; i++)
    {
        // 获取当前的系统时间（毫秒）
        //uint32_t now_time = system_getval_ms();
        uint32_t now_time = uwtick;
        // 检查当前时间是否达到任务的执行时间
        if ((uint32_t)(now_time - scheduler_task[i].last_run) >= scheduler_task[i].rate_ms)
        {
            // 更新任务的上次运行时间为当前时间
            scheduler_task[i].last_run = now_time;

            // 执行任务函数
            scheduler_task[i].task_func();
        }
    }
}
