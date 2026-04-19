# 无线串口 PID 调参协议设计

## 1. 背景与目标

当前工程已经通过无线串口持续输出实时姿态数据，但还缺少一套稳定的在线 PID 查询与调参协议。若在持续打印期间直接输入文本命令，实时数据流会干扰命令识别和参数录入，导致无线串口调参体验不稳定。

本设计目标：

- 在现有无线串口链路上增加统一的 PID 查询与调参协议
- 支持 `steering_pid`、`angle_pid`、`gyro_pid` 三组 PID 在线查询
- 支持通过串口输入 5 个浮点参数，在线更新对应 PID
- 任何进入查询或调参流程的命令都要先暂停实时数据流
- 仅在收到 `start` 后恢复实时数据流，收到 `stop` 时随时暂停
- 以最小侵入方式接入现有工程，不改底层无线串口驱动

## 2. 已确认事实

以下内容来自当前代码库真实上下文：

- 当前无线串口字符串输出封装位于 `code/app/uart_app.c` 的 `wireless_uart_printf()`
- 当前实时姿态打印位于 `code/app/imu_app.c` 的 `imu_test()`，默认输出 `roll,pitch,yaw`
- 三组 PID 实例定义在 `code/app/balance_app.c`，并在 `code/app/balance_app.h` 中对外暴露：
  - `steering_pid`
  - `angle_pid`
  - `gyro_pid`
- `PID_T` 结构体定义在 `code/driver/pid/pid_driver.h`，其中与本次调参直接相关的 5 个字段顺序为：
  - `kp`
  - `ki`
  - `kd`
  - `target`
  - `limit`
- 当前仓库中未发现现成的无线串口命令解析逻辑，本次需要新增命令状态管理与参数解析能力

## 3. 设计范围

本次改动仅覆盖无线串口应用层协议与 PID 参数映射：

- 修改 `code/app/uart_app.c`
- 视需要修改 `code/app/uart_app.h`
- 修改 `code/app/imu_app.c`，使实时打印受统一开关控制
- 参考 `code/app/balance_app.c` 与 `code/app/balance_app.h`，读写三组 PID 实例
- 视需要补充一个轻量级的命令处理入口，供调度层周期调用

本次不修改：

- `code/driver/pid/*` 的 PID 计算公式
- `libraries/zf_device/*` 或其他底层无线串口驱动
- 平衡控制主算法与 PID 调用链
- 实时打印的数据内容格式，除非为了兼容协议必须增加简单状态回包

## 4. 方案选择

对比过三种实现落点：

1. 把查询和修改逻辑分散到 `balance_app.c`、`imu_app.c` 等业务文件
2. 在 `uart_app` 内建立统一的串口命令状态机，并集中管理“数据流开关 + PID 参数映射 + 命令解析”
3. 直接依赖底层无线串口驱动回调，边接收边立即改全局变量

采用方案 2。

原因：

- 串口命令协议属于应用层边界能力，适合集中放在 `uart_app`
- 实时打印暂停/恢复、本次正在等待哪组 PID 参数、字符串解析等状态应统一管理
- `balance_app` 只暴露 PID 实例，不承担串口协议职责，边界更清晰
- 底层驱动应保持纯粹，避免把命令语义耦合到设备驱动层

## 5. 协议定义

### 5.1 支持的命令

本次协议固定支持以下命令：

- `check-steering_pid`
- `change-steering_pid`
- `check-angle_pid`
- `change-angle_pid`
- `check-gyro_pid`
- `change-gyro_pid`
- `stop`
- `start`

### 5.2 参数输入格式

当系统进入某组 PID 的修改等待状态后，用户需要发送一行包含 5 个浮点数的文本，例如：

```text
1.02, 0.03, 0.04, 0.02, 30.04
```

解析规则：

- 5 个值之间使用英文逗号分隔
- 允许逗号前后存在空格
- 参数顺序固定为：`kp, ki, kd, target, limit`
- 参数个数不是 5 个时，判定为非法输入

### 5.3 建议回包格式

为便于无线串口调试，建议统一输出简短文本回包：

```text
[OK] stream stopped
[OK] stream started
[OK] steering_pid: 1.00,0.00,0.00,0.00,30.00
[OK] angle_pid updated
[ERR] invalid param count
[ERR] invalid number
[ERR] unknown command
```

回包不要求与实时数据同格式，只要求可人工辨认、便于串口助手查看。

## 6. 状态机设计

### 6.1 运行状态

无线串口协议定义 3 个应用层状态：

- `STREAMING`：持续发送实时数据
- `PAUSED`：停止实时数据发送，仅接收命令
- `WAITING_PARAM`：已收到某个 `change-*` 命令，正在等待该组 PID 的 5 个新参数

### 6.2 状态切换规则

默认建议上电进入 `STREAMING`。

状态切换规则如下：

- `STREAMING` 收到 `stop` 后，进入 `PAUSED`
- `STREAMING` 收到任意 `check-*` 后，先停止实时数据流，再返回当前 PID 参数，并进入 `PAUSED`
- `STREAMING` 收到任意 `change-*` 后，先停止实时数据流，再进入 `WAITING_PARAM`
- `PAUSED` 收到 `start` 后，进入 `STREAMING`
- `PAUSED` 收到 `stop` 后，保持 `PAUSED`
- `PAUSED` 收到任意 `check-*`，返回对应 PID 参数并保持 `PAUSED`
- `PAUSED` 收到任意 `change-*`，进入 `WAITING_PARAM`
- `WAITING_PARAM` 收到合法 5 参数输入后，更新对应 PID，随后进入 `PAUSED`
- `WAITING_PARAM` 收到非法参数输入后，返回错误并保持 `WAITING_PARAM` 或退回 `PAUSED`，本次推荐保持 `WAITING_PARAM`，便于用户直接重输

### 6.3 全局优先级

`start` 与 `stop` 在任意时刻都应优先生效：

- 在 `WAITING_PARAM` 中收到 `stop`，应立即停止实时流并保持/回到非发送状态
- 在 `WAITING_PARAM` 中收到 `start`，应清除当前等待修改上下文并恢复实时数据流

本设计采用“全局控制命令优先于参数输入”的规则，避免用户误操作后无法退出等待状态。

## 7. PID 参数映射

### 7.1 目标对象

三组可调对象与代码中的实例一一对应：

- `steering_pid` -> `balance_app.c` 中的 `steering_pid`
- `angle_pid` -> `balance_app.c` 中的 `angle_pid`
- `gyro_pid` -> `balance_app.c` 中的 `gyro_pid`

### 7.2 字段映射

本次查询与修改固定只处理以下 5 个字段：

1. `kp`
2. `ki`
3. `kd`
4. `target`
5. `limit`

查询时输出顺序与修改时写入顺序必须完全一致，避免人机两端理解不一致。

### 7.3 更新行为

当收到合法的 5 个参数后，应执行：

- 将 5 个值写入选中的 `PID_T` 实例对应字段
- 调用 `pid_reset()` 清空该 PID 的历史误差、积分和输出缓存

采用重置策略的原因：

- 修改 `kp/ki/kd/target/limit` 后，旧的积分项和历史误差可能与新参数不匹配
- 在线改参后直接沿用旧状态，容易产生瞬时跳变或难以解释的控制输出

## 8. 实现结构

建议在 `code/app/uart_app.c` 中新增以下内部能力：

- 一个“实时数据流是否允许发送”的开关
- 一个协议状态变量
- 一个“当前等待修改的是哪组 PID”的枚举变量
- 一个命令分发函数
- 一个 PID 选择函数，把命令名映射到具体 `PID_T *`
- 一个 PID 参数格式化输出函数
- 一个 5 浮点参数解析函数

建议结构如下：

1. 定义状态枚举：
   - `UART_STREAMING`
   - `UART_PAUSED`
   - `UART_WAITING_PARAM`
2. 定义 PID 目标枚举：
   - `PID_NONE`
   - `PID_STEERING`
   - `PID_ANGLE`
   - `PID_GYRO`
3. 定义静态状态变量：
   - 当前串口协议状态
   - 当前等待修改的 PID 类型
4. 提供统一命令入口：
   - 读取接收到的一整行文本
   - 先判断 `start/stop`
   - 再判断 `check-*` / `change-*`
   - 若处于 `WAITING_PARAM`，则把非控制命令文本按 5 参数解析
5. 在 `imu_test()` 中通过统一开关判断是否打印实时数据

## 9. 数据流控制策略

### 9.1 实时打印门控

当前 `imu_test()` 直接调用 `wireless_uart_printf("%f,%f,%f\r\n", roll, pitch, yaw);`。

本次需要把实时数据流发送改成“受开关控制”的行为：

- 允许发送时，保持原打印行为
- 暂停发送时，`imu_test()` 不再输出实时数据

### 9.2 查询与调参期间的行为

收到以下命令时必须先关闭数据流：

- `check-steering_pid`
- `change-steering_pid`
- `check-angle_pid`
- `change-angle_pid`
- `check-gyro_pid`
- `change-gyro_pid`

这样可以保证：

- 命令回包不会与实时数据混杂
- 用户输入 5 参数时不会被高频数据流淹没

### 9.3 恢复策略

查询完成和调参完成后都不自动恢复数据流。

只有在收到 `start` 后才恢复实时数据发送。这样更符合调参场景，用户可以连续执行：

1. 查询
2. 修改
3. 再查询
4. 最后手动 `start`

## 10. 错误处理

### 10.1 非法命令

未匹配到任何已知命令时，返回：

```text
[ERR] unknown command
```

并保持当前状态不变。

### 10.2 非法参数

在 `WAITING_PARAM` 状态下，若参数输入存在以下问题，应返回错误：

- 参数个数不是 5 个
- 某个字段无法解析为浮点数

建议回包：

```text
[ERR] invalid param count
[ERR] invalid number
```

### 10.3 边界控制

本次协议设计不额外引入每个参数的业务范围校验，只负责完成文本到 PID 字段的写入。原因是：

- 当前需求只强调在线查询与在线修改
- 每组 PID 的合理范围需要结合实车和控制目标决定，尚未形成统一约束

后续如需要，可再加每组 PID 的范围限制与安全阈值。

## 11. 风险与注意事项

### 11.1 串口接收边界

本设计默认上层能够提供“收到一整行文本命令”的入口。如果当前工程只有逐字节接收而没有行缓冲，还需要补充一个最小文本缓存与换行判帧逻辑。

### 11.2 在线改参后的控制跳变

修改 PID 参数后，即使执行 `pid_reset()`，重新恢复控制时仍可能出现输出变化。这属于在线调参本身的正常风险，台架调试时应从小参数开始。

### 11.3 命令与参数混淆

在 `WAITING_PARAM` 状态下，只有 `start` 和 `stop` 具有全局命令优先级；其他文本默认按参数输入处理。实现时必须明确区分，避免把 `change-*` 或任意随机字符串误当作 5 参数写入。

## 12. 验证方案

### 12.1 协议流程验证

重点验证以下流程：

- 默认实时数据持续打印
- 输入 `stop` 后，实时数据停止打印
- 输入 `start` 后，实时数据恢复打印
- 输入 `check-steering_pid` 后，实时数据停止且返回 `steering_pid` 的 5 参数
- 输入 `change-angle_pid` 后，实时数据停止且进入等待参数状态
- 在等待状态输入 `1,2,3,4,5` 后，对应 PID 被更新并保持暂停
- 更新后再次输入 `check-angle_pid`，能看到新参数

### 12.2 错误路径验证

- 输入未知命令，返回 `unknown command`
- 输入少于 5 个参数，返回 `invalid param count`
- 输入包含非法字符的参数，返回 `invalid number`
- 在 `WAITING_PARAM` 中输入 `start`，应恢复数据流并退出等待状态
- 在 `WAITING_PARAM` 中输入 `stop`，应保持暂停并退出或保留等待状态，推荐退出等待状态，避免旧上下文残留

### 12.3 代码级核对点

- 查询与修改都先关闭实时数据流
- 查询只读 PID，不修改参数
- 修改只影响选中的一组 PID，不串写其他 PID
- PID 输出顺序与输入顺序一致，始终为 `kp,ki,kd,target,limit`
- 调参完成后没有自动恢复实时数据流

## 13. 后续建议

以下建议不纳入本次实现范围，但值得后续考虑：

- 为每组 PID 增加参数上下限约束，防止误输极端值
- 增加 `help` 或 `list` 命令，便于查看可用协议
- 把当前实时数据流的开关状态也作为查询项输出
- 支持将更新后的 PID 参数持久化到 Flash，避免断电丢失
