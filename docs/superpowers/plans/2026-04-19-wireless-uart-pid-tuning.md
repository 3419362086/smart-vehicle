# Wireless UART PID Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 `steering_pid`、`angle_pid`、`gyro_pid` 增加无线串口查询/改参协议，并让 `pid_test()` 的无线测试数据流支持 `stop`/`start` 暂停与恢复。

**Architecture:** UART2 RX 中断继续只负责把收到的字节写入逐飞库 FIFO；应用层在 `uart_app.c` 里新增协议状态机、行缓冲和 PID 映射逻辑，由 `schedule.c` 周期调用解析。`balance_app.c` 中的 `pid_test()` 通过协议开关控制是否继续发送 `JustFloat_Test_five(...)`，因此测试数据流与命令会话共享同一条无线串口，但不影响 PIT 中断里的真实控制环。

**Tech Stack:** TC264 嵌入式 C、逐飞 `zf_device_wireless_uart` FIFO 接口、现有 `PID_T` / `pid_reset()`、`JustFloat_Test_five()`、测试调度器 `schedule.c`

---

## File Structure

- Modify: `code/app/uart_app.h`
  - 暴露无线串口 PID 协议服务入口与数据流开关查询接口
- Modify: `code/app/uart_app.c`
  - 新增命令状态机、行缓冲、PID 目标映射、5 浮点解析、查询/改参回包
- Modify: `code/app/balance_app.c`
  - 让 `pid_test()` 在发送 `JustFloat_Test_five(...)` 前先检查无线串口数据流开关
- Modify: `code/app/schedule.c`
  - 注册无线串口命令处理任务，并保证它先于 `pid_test()` 执行
- Reference: `code/app/balance_app.h`
  - 复用现有 `steering_pid` / `angle_pid` / `gyro_pid` 的 `extern` 声明
- Reference: `libraries/zf_device/zf_device_wireless_uart.h`
  - 复用 `wireless_uart_read_buffer()` 与现有 UART2 RX 中断 FIFO
- Reference: `user/isr.c`
  - 确认 UART2 RX ISR 继续只走 `wireless_module_uart_handler()`，不在 ISR 里做协议解析

## Implementation Constraints

- 当前 UART2 RX 中断已经稳定工作，本次**不修改** `user/isr.c`
- `imu_test()` 继续走烧录器虚拟串口，本次**不修改** `code/app/imu_app.c`
- 真正的 2/10/20 ms 控制任务继续跑在 PIT 中断里，本次无线 PID 功能只属于测试链路
- 当前终端环境里未发现 `make` / `mingw32-make`，构建验证默认在 ADS 中完成

### Task 1: 在 `uart_app` 建立无线 PID 协议公共接口与状态骨架

**Files:**
- Modify: `code/app/uart_app.h`
- Modify: `code/app/uart_app.c`

- [ ] **Step 1: 在头文件声明协议服务入口与数据流状态查询接口**

把 `code/app/uart_app.h` 从只保留 `printf` 封装，扩展为下面这组接口：

```c
#ifndef CODE_UART_APP_H_
#define CODE_UART_APP_H_

#include "zf_common_headfile.h"

void wireless_uart_printf(const char *format, ...);
void wireless_uart_pid_service(void);
uint8 wireless_uart_pid_stream_enabled(void);

#endif
```

- [ ] **Step 2: 在 `uart_app.c` 头部补齐协议所需依赖和常量**

在现有 `wireless_uart_printf()` 上方加入 PID 协议需要的头文件、枚举和缓冲长度定义：

```c
#include "uart_app.h"
#include "balance_app.h"
#include <stdlib.h>
#include <string.h>

#define UART_PID_LINE_MAX_LEN    (96U)
#define UART_PID_PARAM_COUNT     (5U)

typedef enum
{
    UART_PID_STATE_STREAMING = 0,
    UART_PID_STATE_PAUSED,
    UART_PID_STATE_WAITING_PARAM,
} uart_pid_state_enum;

typedef enum
{
    UART_PID_TARGET_NONE = 0,
    UART_PID_TARGET_STEERING,
    UART_PID_TARGET_ANGLE,
    UART_PID_TARGET_GYRO,
} uart_pid_target_enum;
```

- [ ] **Step 3: 加入协议静态状态变量，默认开机允许 `pid_test()` 发流**

紧接着上面的枚举，加入协议状态、等待目标和命令行缓冲：

```c
static volatile uint8 uart_pid_stream_enabled = 1U;
static uart_pid_state_enum uart_pid_state = UART_PID_STATE_STREAMING;
static uart_pid_target_enum uart_pid_pending_target = UART_PID_TARGET_NONE;
static char uart_pid_line_buffer[UART_PID_LINE_MAX_LEN];
static uint32 uart_pid_line_length = 0U;
```

- [ ] **Step 4: 补一个最小 getter，供 `pid_test()` 查询是否允许发流**

在 `wireless_uart_printf()` 后面加入这个接口实现：

```c
uint8 wireless_uart_pid_stream_enabled(void)
{
    return uart_pid_stream_enabled;
}
```

- [ ] **Step 5: 做一次接口一致性静态检查并提交**

检查以下点：

```text
1. `uart_app.h` 中新声明的函数名与 `uart_app.c` 后续实现完全一致
2. 默认状态是 STREAMING，`pid_test()` 上电后还能继续发无线测试流
3. 当前还没有接入调度器，代码只是增加协议骨架，不会影响现有行为
```

提交：

```bash
git add code/app/uart_app.h code/app/uart_app.c
git commit -m "feat: add wireless PID protocol skeleton"
```

### Task 2: 在 `uart_app.c` 实现命令解析、PID 映射和改参逻辑

**Files:**
- Modify: `code/app/uart_app.c`
- Reference: `code/app/balance_app.h`

- [ ] **Step 1: 加入 PID 目标选择与名称映射函数**

在 `uart_app.c` 中新增一个按目标枚举返回 `PID_T *` 和名字的帮助函数：

```c
static PID_T *uart_pid_get_target(uart_pid_target_enum target, const char **name)
{
    switch (target)
    {
        case UART_PID_TARGET_STEERING:
            if (NULL != name) { *name = "steering_pid"; }
            return &steering_pid;
        case UART_PID_TARGET_ANGLE:
            if (NULL != name) { *name = "angle_pid"; }
            return &angle_pid;
        case UART_PID_TARGET_GYRO:
            if (NULL != name) { *name = "gyro_pid"; }
            return &gyro_pid;
        default:
            if (NULL != name) { *name = "unknown"; }
            return NULL;
    }
}
```

- [ ] **Step 2: 加入命令名到 PID 目标的映射与字符串清理函数**

补两个帮助函数：一个去掉首尾空白，一个把 `steering_pid` / `angle_pid` / `gyro_pid` 解析成目标枚举：

```c
static char *uart_pid_trim(char *text)
{
    char *end = NULL;

    while ((*text == ' ') || (*text == '\t'))
    {
        text++;
    }

    if ('\0' == *text)
    {
        return text;
    }

    end = text + strlen(text) - 1;
    while ((end > text) && ((*end == ' ') || (*end == '\t')))
    {
        *end-- = '\0';
    }

    return text;
}

static uart_pid_target_enum uart_pid_target_from_name(const char *name)
{
    if (0 == strcmp(name, "steering_pid")) { return UART_PID_TARGET_STEERING; }
    if (0 == strcmp(name, "angle_pid"))    { return UART_PID_TARGET_ANGLE; }
    if (0 == strcmp(name, "gyro_pid"))     { return UART_PID_TARGET_GYRO; }
    return UART_PID_TARGET_NONE;
}
```

- [ ] **Step 3: 实现 5 浮点参数解析，区分数量错误和非法数字**

新增一个解析结果枚举和解析函数，确保协议能返回不同错误：

```c
typedef enum
{
    UART_PID_PARSE_OK = 0,
    UART_PID_PARSE_INVALID_COUNT,
    UART_PID_PARSE_INVALID_NUMBER,
} uart_pid_parse_result_enum;

static uart_pid_parse_result_enum uart_pid_parse_params(char *text, float params[UART_PID_PARAM_COUNT])
{
    char *cursor = text;
    char *end = NULL;
    uint32 index = 0U;

    for (index = 0U; index < UART_PID_PARAM_COUNT; ++index)
    {
        while ((*cursor == ' ') || (*cursor == '\t'))
        {
            cursor++;
        }

        params[index] = strtof(cursor, &end);
        if (end == cursor)
        {
            return UART_PID_PARSE_INVALID_NUMBER;
        }

        while ((*end == ' ') || (*end == '\t'))
        {
            end++;
        }

        if (index < (UART_PID_PARAM_COUNT - 1U))
        {
            if (*end != ',')
            {
                return UART_PID_PARSE_INVALID_COUNT;
            }
            cursor = end + 1;
        }
        else if (*end != '\0')
        {
            return UART_PID_PARSE_INVALID_COUNT;
        }
    }

    return UART_PID_PARSE_OK;
}
```

- [ ] **Step 4: 实现查询回包和参数写入函数**

把“打印当前 PID”和“写新参数 + `pid_reset()`”集中到两个静态函数：

```c
static void uart_pid_send_current(uart_pid_target_enum target)
{
    const char *name = NULL;
    PID_T *pid = uart_pid_get_target(target, &name);

    if (NULL == pid)
    {
        wireless_uart_printf("[ERR] unknown command\r\n");
        return;
    }

    wireless_uart_printf("[OK] %s: %f,%f,%f,%f,%f\r\n",
                         name, pid->kp, pid->ki, pid->kd, pid->target, pid->limit);
}

static void uart_pid_apply_params(uart_pid_target_enum target, const float params[UART_PID_PARAM_COUNT])
{
    PID_T *pid = uart_pid_get_target(target, NULL);

    if (NULL == pid)
    {
        return;
    }

    pid->kp = params[0];
    pid->ki = params[1];
    pid->kd = params[2];
    pid->target = params[3];
    pid->limit = params[4];
    pid_reset(pid);
}
```

- [ ] **Step 5: 实现完整命令状态机分发函数**

新增一个 `uart_pid_handle_line()`，把 `stop/start/check/change/参数输入` 全部统一处理：

```c
static void uart_pid_handle_line(char *line)
{
    float params[UART_PID_PARAM_COUNT];
    char *command = uart_pid_trim(line);
    uart_pid_target_enum target = UART_PID_TARGET_NONE;
    const char *name = NULL;

    if ('\0' == *command)
    {
        return;
    }

    if (0 == strcmp(command, "stop"))
    {
        uart_pid_stream_enabled = 0U;
        uart_pid_state = UART_PID_STATE_PAUSED;
        uart_pid_pending_target = UART_PID_TARGET_NONE;
        wireless_uart_printf("[OK] stream stopped\r\n");
        return;
    }

    if (0 == strcmp(command, "start"))
    {
        uart_pid_stream_enabled = 1U;
        uart_pid_state = UART_PID_STATE_STREAMING;
        uart_pid_pending_target = UART_PID_TARGET_NONE;
        wireless_uart_printf("[OK] stream started\r\n");
        return;
    }

    if (UART_PID_STATE_WAITING_PARAM == uart_pid_state)
    {
        switch (uart_pid_parse_params(command, params))
        {
            case UART_PID_PARSE_OK:
                uart_pid_apply_params(uart_pid_pending_target, params);
                uart_pid_get_target(uart_pid_pending_target, &name);
                wireless_uart_printf("[OK] %s updated\r\n", name);
                uart_pid_pending_target = UART_PID_TARGET_NONE;
                uart_pid_state = UART_PID_STATE_PAUSED;
                break;
            case UART_PID_PARSE_INVALID_COUNT:
                wireless_uart_printf("[ERR] invalid param count\r\n");
                break;
            case UART_PID_PARSE_INVALID_NUMBER:
            default:
                wireless_uart_printf("[ERR] invalid number\r\n");
                break;
        }
        return;
    }

    if (0 == strncmp(command, "check-", 6))
    {
        target = uart_pid_target_from_name(command + 6);
        if (UART_PID_TARGET_NONE == target)
        {
            wireless_uart_printf("[ERR] unknown command\r\n");
            return;
        }

        uart_pid_stream_enabled = 0U;
        uart_pid_state = UART_PID_STATE_PAUSED;
        uart_pid_pending_target = UART_PID_TARGET_NONE;
        uart_pid_send_current(target);
        return;
    }

    if (0 == strncmp(command, "change-", 7))
    {
        target = uart_pid_target_from_name(command + 7);
        if (UART_PID_TARGET_NONE == target)
        {
            wireless_uart_printf("[ERR] unknown command\r\n");
            return;
        }

        uart_pid_stream_enabled = 0U;
        uart_pid_state = UART_PID_STATE_WAITING_PARAM;
        uart_pid_pending_target = target;
        uart_pid_get_target(target, &name);
        wireless_uart_printf("[OK] %s ready\r\n", name);
        return;
    }

    wireless_uart_printf("[ERR] unknown command\r\n");
}
```

- [ ] **Step 6: 实现从 FIFO 拼行并逐行分发的服务函数**

在 `uart_app.c` 末尾加入这个调度任务入口，让它从逐飞库 FIFO 取字节并按换行分帧：

```c
void wireless_uart_pid_service(void)
{
    uint8 rx_buffer[WIRELESS_UART_BUFFER_SIZE];
    uint32 rx_len = wireless_uart_read_buffer(rx_buffer, sizeof(rx_buffer));
    uint32 index = 0U;

    for (index = 0U; index < rx_len; ++index)
    {
        char ch = (char)rx_buffer[index];

        if (('\r' == ch) || ('\n' == ch))
        {
            if (0U == uart_pid_line_length)
            {
                continue;
            }

            uart_pid_line_buffer[uart_pid_line_length] = '\0';
            uart_pid_handle_line(uart_pid_line_buffer);
            uart_pid_line_length = 0U;
            uart_pid_line_buffer[0] = '\0';
            continue;
        }

        if (uart_pid_line_length < (UART_PID_LINE_MAX_LEN - 1U))
        {
            uart_pid_line_buffer[uart_pid_line_length++] = ch;
        }
        else
        {
            uart_pid_stream_enabled = 0U;
            uart_pid_state = UART_PID_STATE_PAUSED;
            uart_pid_pending_target = UART_PID_TARGET_NONE;
            uart_pid_line_length = 0U;
            uart_pid_line_buffer[0] = '\0';
            wireless_uart_printf("[ERR] command too long\r\n");
        }
    }
}
```

- [ ] **Step 7: 做一次解析逻辑静态检查并提交**

检查以下点：

```text
1. 只有收到 `start` 才会把 stream_enabled 重新置 1
2. `check-*` 和 `change-*` 都会先把 stream_enabled 置 0
3. `WAITING_PARAM` 状态下 `start` / `stop` 仍然能优先生效
4. 参数写入顺序固定是 kp, ki, kd, target, limit
5. 改参后一定调用 `pid_reset()`
```

提交：

```bash
git add code/app/uart_app.c
git commit -m "feat: add wireless PID command parser"
```

### Task 3: 让 `pid_test()` 受协议开关控制，并接入测试调度器

**Files:**
- Modify: `code/app/balance_app.c`
- Modify: `code/app/schedule.c`

- [ ] **Step 1: 在 `balance_app.c` 引入 `uart_app.h`，为 `pid_test()` 读取流开关**

在 `balance_app.c` 头部增加包含：

```c
#include "balance_app.h"
#include "uart_app.h"
```

- [ ] **Step 2: 给 `pid_test()` 加无线数据流门控**

把当前 `pid_test()` 改成下面这样，只有 `start` 状态才继续发 `JustFloat`：

```c
void pid_test(void)
{
    if (!wireless_uart_pid_stream_enabled())
    {
        return;
    }

    JustFloat_Test_five(pitch, gyro_y_rate, target_angle, target_gyro_rate, servo_output);
}
```

- [ ] **Step 3: 在 `schedule.c` 注册无线命令处理任务，并把它排在 `pid_test()` 前面**

把调度表改成下面这种顺序，先处理命令，再决定本周期要不要继续发测试流：

```c
static task_t scheduler_task[] =
{
    {wireless_uart_pid_service, 5, 0},  // 无线串口 PID 命令处理
    {pid_test,                  10, 0}, // PID 测试数据流
    {motor_guard_update,        10, 0}, // 侧倒保护
};
```

- [ ] **Step 4: 检查调度边界，确保不把控制主逻辑迁出中断**

人工核对以下点：

```text
1. `imu_proc()` / `balance_gyro_loop()` / `balance_angle_loop()` / `balance_steering_loop()` 仍然只在 `user/isr.c` 里跑
2. 新增到 `schedule.c` 的只有测试相关任务：无线命令解析和 `pid_test`
3. `pid_test` 在同一个调度周期里会晚于 `wireless_uart_pid_service` 执行，因此 `stop` 能先于发流生效
```

- [ ] **Step 5: 提交调度接入改动**

```bash
git add code/app/balance_app.c code/app/schedule.c
git commit -m "feat: gate PID telemetry through wireless protocol"
```

### Task 4: 在板级环境验证协议与测试数据流行为

**Files:**
- Reference: `code/app/uart_app.c`
- Reference: `code/app/balance_app.c`
- Reference: `code/app/schedule.c`

- [ ] **Step 1: 在 ADS 中编译 `Debug` 配置**

当前终端里没有 `make`，所以这一步改为 ADS 手工构建：

```text
1. 打开 ADS 工程 `smart-vehicle`
2. 选择 `Debug` 配置
3. 执行 Build Active Project
4. 预期：无编译错误，生成新的 `Debug/Seekfree_TC264_Opensource_Library.elf`
```

- [ ] **Step 2: 验证默认发流 + `stop/start`**

烧录后连接无线串口助手，波特率 `115200`，按下面顺序操作：

```text
1. 上电后观察 `pid_test()` 的五路 JustFloat 数据持续输出
2. 发送 `stop\r\n`
3. 预期：收到 `[OK] stream stopped`，随后无线 JustFloat 数据停止
4. 再发送 `start\r\n`
5. 预期：收到 `[OK] stream started`，随后无线 JustFloat 数据恢复
```

- [ ] **Step 3: 验证查询命令会先停流再回参数**

在无线串口助手里执行：

```text
1. 发送 `check-steering_pid\r\n`
2. 预期：JustFloat 数据先停，再收到 `[OK] steering_pid: ...`
3. 发送 `check-angle_pid\r\n`
4. 预期：收到 `[OK] angle_pid: ...`
5. 发送 `check-gyro_pid\r\n`
6. 预期：收到 `[OK] gyro_pid: ...`
7. 此时无线数据流保持暂停，直到再次发送 `start`
```

- [ ] **Step 4: 验证改参命令与 5 参数写入顺序**

在无线串口助手里执行下面这组完整流程：

```text
1. 发送 `change-angle_pid\r\n`
2. 预期：收到 `[OK] angle_pid ready`
3. 发送 `1.11, 0.02, 0.33, 0.00, 480.00\r\n`
4. 预期：收到 `[OK] angle_pid updated`
5. 发送 `check-angle_pid\r\n`
6. 预期：回包中的 5 个值顺序正好是 `1.11,0.02,0.33,0.00,480.00`
7. 发送 `start\r\n`
8. 预期：无线 JustFloat 数据重新恢复
```

- [ ] **Step 5: 验证错误路径与范围边界**

继续在无线串口助手中验证：

```text
1. 发送 `change-gyro_pid\r\n`
2. 再发送 `1,2,3,4\r\n`
3. 预期：收到 `[ERR] invalid param count`，仍停留在等待参数状态
4. 再发送 `1,2,x,4,5\r\n`
5. 预期：收到 `[ERR] invalid number`
6. 再发送 `start\r\n`
7. 预期：退出等待状态并恢复数据流
8. 发送 `check-unknown\r\n`
9. 预期：收到 `[ERR] unknown command`
```

- [ ] **Step 6: 做一次跨层回归检查并提交**

确认下面这些行为都成立：

```text
1. `imu_test()` 的烧录器虚拟串口输出完全不受影响
2. `user/isr.c` 没有新增打印、阻塞或协议解析逻辑
3. `schedule.c` 新增的只是测试任务，不影响 PIT 中断里的真实控制链
4. `pid_test()` 之外的无线日志（如未来边沿日志）不会被误拦截
```

提交：

```bash
git add code/app/uart_app.h code/app/uart_app.c code/app/balance_app.c code/app/schedule.c
git commit -m "feat: add wireless UART PID tuning workflow"
```
