/*
 * motor_app.c
 *
 *  Created on: 2025年11月12日
 *      Author: suiyungui
 *  Modified: 改为无刷FOC驱动接口
 */

#include "motor_app.h"
#include "balance_app.h"


int16_t motor_rpm = 0;   // 电机反馈转速，单位：RPM
float wheel_speed = 0;   // 由转速换算得到的轮速，单位：m/s
static uint8_t motor_roll_protected = 1;   // 上电默认保护，等待横滚角回到安全区并稳定一段时间后再恢复输出
static uint8_t motor_roll_recover_pending = 0; // 1: 当前正在等待“连续回正一段时间后退出保护”
static uint8_t motor_last_command_is_speed = 0;   // 0: 最近一次是占空比命令 1: 最近一次是速度命令
static int16_t motor_last_speed_cmd = 0;          // 最近一次速度模式目标值
static int16_t motor_speed_before_protect = 0;    // 进入保护前缓存的速度目标值
static uint8_t motor_speed_resume_ready = 0;      // 1: 当前存在可在退出保护后恢复的速度命令
static uint32_t motor_roll_recover_start = 0;     // 回到恢复阈值内后的起始时刻，单位：毫秒

/* stop 命令触发的强制锁定期：锁定期内 motor_guard_update 拒绝任何恢复动作 */
#define MOTOR_STOP_LOCK_MS      (4000U)
static uint8_t  motor_stop_locked     = 0;   // 1: 当前处于 stop 锁定期
static uint32_t motor_stop_lock_start = 0;   // 锁定开始时的 uwtick 时间戳 (ms)

/*
 * @brief 保护触发后按最近一次控制模式主动下发0命令
 */
static void motor_force_stop_by_last_mode(void)
{
    // 速度模式在驱动内部会保持目标值，进入保护时必须主动补发一次 0 命令把旧目标冲掉。
    if (motor_last_command_is_speed)
    {
        small_driver_set_speed(0, 0);
    }
    else
    {
        small_driver_set_duty(0, 0);
    }
}

/*
 * @brief 根据当前横滚角更新电机保护状态
 * @return 1 表示处于保护态，0 表示允许输出
 */
static uint8_t motor_roll_guard_active(void)
{
    uint8_t previous_state = motor_roll_protected;
    float roll_abs = pitch;

    if (roll_abs < 0.0f)
    {
        roll_abs = -roll_abs;
    }

    // 注意：当前工程中 pitch 表示物理横滚角，而非常规命名中的俯仰角。
    // 这里使用 cut-off / recover 两个阈值形成迟滞，避免角度在临界值附近抖动时电机反复开关。
    // 同时退出保护还要求“连续回正 MOTOR_ROLL_RECOVER_DELAY_MS 毫秒”，防止刚过阈值就立刻恢复。
    if (motor_roll_protected)
    {
        if (roll_abs <= MOTOR_ROLL_RECOVER_DEG)
        {
            // 第一次进入恢复候选区时只记时，不立刻退出保护。
            if (!motor_roll_recover_pending)
            {
                motor_roll_recover_pending = 1;
                motor_roll_recover_start = uwtick;
            }
            else if ((uint32_t)(uwtick - motor_roll_recover_start) >= MOTOR_ROLL_RECOVER_DELAY_MS)
            {
                motor_roll_protected = 0;
                motor_roll_recover_pending = 0;
            }
        }
        else
        {
            // 只要中途再次超出恢复阈值，就清空已累计的稳定时间，重新计时。
            motor_roll_recover_pending = 0;
        }
    }
    else
    {
        if (roll_abs >= MOTOR_ROLL_CUTOFF_DEG)
        {
            motor_roll_protected = 1;
            motor_roll_recover_pending = 0;
        }
    }

    if ((previous_state != motor_roll_protected) && motor_roll_protected)
    {
        printf("[motor_guard] enter roll protection, pitch=%f\r\n", pitch);
        wireless_uart_printf("[motor_guard] enter roll protection, pitch=%f\r\n", pitch);
    }
    else if ((previous_state != motor_roll_protected) && !motor_roll_protected)
    {
        printf("[motor_guard] exit roll protection, pitch=%f\r\n", pitch);
        wireless_uart_printf("[motor_guard] exit roll protection, pitch=%f\r\n", pitch);
    }

    return motor_roll_protected;
}

/*
 * @brief 刷新横滚保护状态，并在首次进入保护时主动下发0命令
 * @note  退出保护除了满足恢复角度阈值外，还要求连续稳定保持 MOTOR_ROLL_RECOVER_DELAY_MS 毫秒
 */
void motor_guard_update(void)
{
    uint8_t previous_state = motor_roll_protected;
    uint8_t current_state = 0;

    // stop 命令的强制锁定期（见 MOTOR_STOP_LOCK_MS）：忽略一切横滚判断，保持停车态。
    // 无符号溢出下差值运算依然正确，只要锁定时长 << 2^31 ms 就安全。
    if (motor_stop_locked)
    {
        if ((uwtick - motor_stop_lock_start) < MOTOR_STOP_LOCK_MS)
        {
            motor_roll_protected = 1;
            motor_roll_recover_pending = 0;
            balance_hold_servo_center();
            return;
        }
        motor_stop_locked = 0;   // 锁定期已过，恢复常规横滚保护判断
    }

    current_state = motor_roll_guard_active();

    if (current_state)
    {
        balance_hold_servo_center();
    }

    // 只在“正常态 -> 保护态”的边沿主动停机一次，避免每个 2ms 周期都重复打 0 命令。
    if ((!previous_state) && current_state)
    {
        if (motor_last_command_is_speed && motor_speed_resume_ready)
        {
            motor_speed_before_protect = motor_last_speed_cmd;
        }
        motor_force_stop_by_last_mode();
    }
    else if (previous_state && (!current_state))
    {
        balance_reset_all_pid();
        if (motor_last_command_is_speed && motor_speed_resume_ready)
        {
            small_driver_set_speed((int16)motor_speed_before_protect, (int16)motor_speed_before_protect);
        }
    }
}

/*
 * @brief 查询当前横滚保护状态
 * @return 1 表示处于保护态，0 表示允许输出
 */
uint8_t motor_guard_is_active(void)
{
    return motor_roll_protected;
}

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
    // 记录最近一次不是速度模式，供保护触发时决定补发 duty=0。
    motor_last_command_is_speed = 0;
    motor_speed_resume_ready = 0;

    // 限幅 duty最大为-10000 ~ 10000 自行判断限幅
    if(duty > 5000) duty = 5000;
    if(duty < -5000) duty = -5000;

    // 如果此刻已经处于保护态，新的输出命令直接压成 0。
    if (motor_roll_guard_active())
    {
        duty = 0;
    }

    small_driver_set_duty((int16)duty, (int16)duty);
}

/*
 * @brief 设置电机目标速度
 * @param speed 双电机统一使用的速度命令
 */
void motor_set_speed(int16_t speed)
{
    // 记录最近一次是速度模式，供保护触发时决定补发 speed=0。
    motor_last_command_is_speed = 1;

    // 限幅保护
    if(speed > 3000) speed = 3000;
    if(speed < -3000) speed = -3000;

    motor_last_speed_cmd = speed;
    motor_speed_before_protect = speed;
    motor_speed_resume_ready = (speed != 0);

    // 如果此刻已经处于保护态，新的速度命令直接压成 0。
    if (motor_roll_guard_active())
    {
        speed = 0;
    }

    small_driver_set_speed((int16)speed, (int16)speed);
}

/*
 * @brief 立即停车（软暂停 + 强制锁定 MOTOR_STOP_LOCK_MS 毫秒）
 * @note  行为：
 *        1. 缓存当前速度目标为"待恢复点"（motor_last_speed_cmd 故意不清零）
 *        2. 主动进入横滚保护态
 *        3. 启动强制锁定窗口：锁定期内 motor_guard_update 忽略一切恢复
 *        4. 锁定期过后按常规横滚保护机制判断是否恢复：
 *           - |pitch| <= MOTOR_ROLL_RECOVER_DEG (10°) 且连续保持 MOTOR_ROLL_RECOVER_DELAY_MS：自动恢复 stop 前速度
 *           - |pitch| 在 RECOVER 与 CUTOFF 之间：迟滞区，继续停车
 *           - |pitch| >  MOTOR_ROLL_CUTOFF_DEG  (21°)：继续停车，等车体回正
 * @warning 这里故意不清零 motor_last_speed_cmd：
 *          若清零，stop 恢复后再次触发"横滚保护进入边沿"时，
 *          motor_guard_update 会用 motor_last_speed_cmd (=0) 覆盖 motor_speed_before_protect，
 *          导致横滚保护退出后无法恢复到原速度（车不动）。
 */
void motor_stop(void)
{
    // 仅速度模式且当前速度非0时，缓存为待恢复点；否则保留旧的恢复点（可能是 0）
    if (motor_last_command_is_speed && (motor_last_speed_cmd != 0))
    {
        motor_speed_before_protect = motor_last_speed_cmd;
        motor_speed_resume_ready = 1;
    }
    // 注意：不清零 motor_last_speed_cmd，原因见函数顶部 @warning

    // 主动设保护态
    motor_roll_protected = 1;
    motor_roll_recover_pending = 0;
    balance_hold_servo_center();

    // 启动强制锁定窗口
    motor_stop_locked     = 1;
    motor_stop_lock_start = uwtick;

    small_driver_set_speed(0, 0);
    small_driver_set_duty(0, 0);
}

/*
 * @brief 获取当前电机速度反馈
 */
void motor_get_speed(void)
{
    // 根据当前接线方向对左轮反馈取反，统一正方向定义
    motor_rpm =  -motor_value.receive_right_speed_data;
    wheel_speed = motor_rpm * RPM_TO_WHEEL;
   printf("rpm:%d,speed:%f\r\n", motor_rpm, wheel_speed);
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
   printf("%d\r\n",duty);
}
