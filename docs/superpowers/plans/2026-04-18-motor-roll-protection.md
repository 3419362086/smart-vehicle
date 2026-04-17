# Motor Roll Protection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在电机应用层加入基于 IMU 横滚角的迟滞保护，使电机在横滚角超限时强制输出 0，并在回到安全区后恢复输出。

**Architecture:** 保护逻辑集中放在 `code/app/motor_app.c`，通过读取 `imu_app.h` 暴露的 `pitch` 作为物理横滚角进行状态机判断。`motor_set_duty()` 和 `motor_set_speed()` 共用同一套保护状态与日志切换逻辑，保持底层驱动和 IMU 层不变。

**Tech Stack:** TC264 嵌入式 C、现有 IMU 全局量、现有电机应用层与串口日志输出

---

## 文件结构

- 修改: `code/app/motor_app.c`
  - 增加横滚保护阈值宏、保护状态、迟滞判断函数、状态切换日志
  - 在 `motor_set_duty()` / `motor_set_speed()` 中接入统一保护逻辑
- 修改: `code/app/motor_app.h`
  - 补充注释，明确电机输出接口带横滚保护
- 参考: `code/app/imu_app.h`
  - 使用现有 `pitch` 全局量，且必须在注释里明确其物理含义是横滚角
- 文档: `docs/superpowers/specs/2026-04-18-motor-roll-protection-design.md`
  - 作为实现依据，不需要再修改

## 实施约束

- 当前 CLI 环境未发现可直接使用的 C 编译器或现成单元测试框架，无法在本地完成真正的自动化 TDD。
- 本计划因此采用“最小侵入实现 + 串口日志验证 + 台架验证”的路径，但实现步骤仍保持红绿思路：先加最小可验证逻辑，再通过日志与接口行为确认状态切换。
- 不新增底层驱动接口，不修改 IMU 姿态计算，不碰调度结构。

### Task 1: 在电机应用层建立横滚保护状态机

**Files:**
- Modify: `code/app/motor_app.c`
- Reference: `code/app/imu_app.h`

- [ ] **Step 1: 在 `motor_app.c` 引入 IMU 角度依赖并声明保护常量**

在文件头部增加 `imu_app.h` 引用，并定义保护阈值与初始状态常量：

```c
#include "motor_app.h"
#include "imu_app.h"

#define MOTOR_ROLL_CUTOFF_DEG   (22.0f)
#define MOTOR_ROLL_RECOVER_DEG  (18.0f)

static uint8_t motor_roll_protected = 1;
```

- [ ] **Step 2: 写入保护状态更新函数**

在 `motor_app.c` 中新增一个仅文件内可见的静态函数，负责读取 `pitch`、执行迟滞判断并在状态切换时打印一次日志：

```c
static uint8_t motor_roll_guard_active(void)
{
    uint8_t previous_state = motor_roll_protected;
    float roll_abs = pitch;

    if (roll_abs < 0.0f)
    {
        roll_abs = -roll_abs;
    }

    // 当前工程里 pitch 表示物理横滚角，不是常规语义中的俯仰角。
    if (motor_roll_protected)
    {
        if (roll_abs <= MOTOR_ROLL_RECOVER_DEG)
        {
            motor_roll_protected = 0;
        }
    }
    else
    {
        if (roll_abs >= MOTOR_ROLL_CUTOFF_DEG)
        {
            motor_roll_protected = 1;
        }
    }

    if ((previous_state != motor_roll_protected) && motor_roll_protected)
    {
        printf("[motor_guard] enter roll protection, pitch=%f\r\n", pitch);
    }
    else if ((previous_state != motor_roll_protected) && !motor_roll_protected)
    {
        printf("[motor_guard] exit roll protection, pitch=%f\r\n", pitch);
    }

    return motor_roll_protected;
}
```

- [ ] **Step 3: 人工代码检查红态**

检查以下点是否成立：

```text
1. 上电时 motor_roll_protected 初值为 1
2. 保护态只在 abs(pitch) <= 18.0f 时解除
3. 正常态只在 abs(pitch) >= 22.0f 时重新进入保护
4. 日志只在状态切换时打印
```

预期：此时尚未接入输出函数，逻辑存在但不会影响电机输出。

### Task 2: 把保护接入占空比与速度输出路径

**Files:**
- Modify: `code/app/motor_app.c`
- Modify: `code/app/motor_app.h`

- [ ] **Step 1: 在 `motor_set_duty()` 接入保护判断**

将现有函数改为先判断保护状态，再决定是否下发原始占空比：

```c
void motor_set_duty(int16_t duty)
{
    if (duty > 5000) duty = 5000;
    if (duty < -5000) duty = -5000;

    if (motor_roll_guard_active())
    {
        duty = 0;
    }

    small_driver_set_duty((int16)duty, (int16)duty);
}
```

- [ ] **Step 2: 在 `motor_set_speed()` 接入保护判断**

将速度模式纳入同一保护边界，避免未来切换控制方式时失效：

```c
void motor_set_speed(int16_t speed)
{
    if (speed > 3000) speed = 3000;
    if (speed < -3000) speed = -3000;

    if (motor_roll_guard_active())
    {
        speed = 0;
    }

    small_driver_set_speed((int16)speed, (int16)speed);
}
```

- [ ] **Step 3: 更新头文件注释，说明接口自带横滚保护**

把 `motor_app.h` 注释更新为能反映新行为：

```c
void motor_set_duty(int16_t duty);   // 设置双电机占空比，内部带限幅与横滚保护

void motor_set_speed(int16_t speed); // 设置双电机目标速度，内部带限幅与横滚保护
```

- [ ] **Step 4: 人工代码检查绿态**

检查以下点是否成立：

```text
1. motor_test() 继续通过 motor_set_duty() 间接受到保护
2. 保护态下 duty/speed 无论传入值如何都会被压成 0
3. 正常态下仍保留原有限幅行为
4. 保护逻辑没有直接侵入底层 small_driver_* 驱动
```

### Task 3: 做最小行为验证并记录受限项

**Files:**
- Modify: `code/app/motor_app.c`
- Reference: `docs/superpowers/specs/2026-04-18-motor-roll-protection-design.md`

- [ ] **Step 1: 检查日志输出是否满足单次切换要求**

对照代码确认日志触发条件只发生在状态变化边沿：

```text
1. previous_state == motor_roll_protected 时不打印
2. previous_state != motor_roll_protected 且新状态为 1 时打印 enter
3. previous_state != motor_roll_protected 且新状态为 0 时打印 exit
```

- [ ] **Step 2: 按设计文档进行静态场景核对**

逐项核对实现是否覆盖以下场景：

```text
- abs(pitch)=23: 进入或保持保护态
- abs(pitch)=20: 若已保护则继续保持，不退出
- abs(pitch)=17: 退出保护态
- abs(pitch) 在 18~22 间抖动: 不反复切换
- abs(pitch) 为负值大角度: 与正值对称
```

- [ ] **Step 3: 记录当前验证边界**

在实现说明或交付说明中明确以下受限项：

```text
1. 当前 CLI 环境缺少可用 C 编译器，未完成本地自动化编译验证
2. 当前仓库缺少现成单元测试框架，未完成真正的测试先行自动化验证
3. 仍需要在目标板上通过串口日志和台架倾斜动作完成最终验证
```

- [ ] **Step 4: 台架验证建议命令与现象**

交付时明确提醒以下验证动作：

```text
1. 上电时若车体倾斜超过恢复阈值，电机不应动作
2. 缓慢扶正至 abs(pitch) <= 18，应看到退出保护日志
3. 再倾斜到 abs(pitch) >= 22，应看到进入保护日志
4. 在 18~22 度间小幅晃动，不应连续刷 enter/exit 日志
```

## 自检结果

- Spec coverage: 已覆盖切断阈值、恢复阈值、上电默认保护、单次日志、占空比与速度双路径保护、台架验证要求。
- Placeholder scan: 未保留 TBD/TODO/“后续补充”式占位实现步骤。
- Type consistency: 使用的状态变量、阈值宏、函数名在各任务中保持一致，输出接口仍沿用现有 `motor_set_duty()` / `motor_set_speed()`。
