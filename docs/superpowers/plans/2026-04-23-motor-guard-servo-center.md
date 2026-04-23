# Motor Guard Servo Center Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复侧倒保护的两个缺陷：保护等待区舵机不回中，以及上电首次退出保护后无法恢复之前的速度命令。

**Architecture:** 保持现有 `motor_app.c` 保护状态机为核心，不重构控制链路。通过补充“速度恢复缓存有效性”和“保护期间持续舵机回中”两个最小行为，修复缺陷；回归验证使用主机侧 Python 脚本锁定状态机语义。

**Tech Stack:** TC264 C 固件、Python 3 `unittest`

---

### Task 1: 写入状态机回归测试

**Files:**
- Create: `tests/test_motor_guard_behavior.py`

- [ ] **Step 1: Write the failing test**

```python
def test_initial_protection_can_restore_pending_speed_after_safe_delay():
    guard.command_speed(400)
    guard.tick_to(0, pitch=0.0)
    guard.tick_to(4000, pitch=0.0)
    assert guard.current_speed_command == 400


def test_servo_centers_while_waiting_for_recover_delay():
    guard.trigger_runtime_protection_from_running(speed=400)
    guard.tick_to(1000, pitch=15.0)
    assert guard.servo_is_centered is True
    assert guard.is_protected is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_motor_guard_behavior -v`
Expected: 先因测试文件不存在/行为未实现而失败

- [ ] **Step 3: Write minimal implementation**

```python
class GuardModel:
    ...
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_motor_guard_behavior -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add tests/test_motor_guard_behavior.py docs/superpowers/plans/2026-04-23-motor-guard-servo-center.md
git commit -m "test: add motor guard regression coverage"
```

### Task 2: 修复电机保护恢复缓存

**Files:**
- Modify: `code/app/motor_app.c`

- [ ] **Step 1: Write the failing test**

```python
def test_initial_protection_can_restore_pending_speed_after_safe_delay():
    ...
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_motor_guard_behavior -v`
Expected: `current_speed_command` 仍为 `0`

- [ ] **Step 3: Write minimal implementation**

```c
static uint8_t motor_speed_resume_ready = 0;

void motor_set_speed(int16_t speed)
{
    ...
    motor_speed_before_protect = speed;
    motor_speed_resume_ready = (speed != 0);
    ...
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_motor_guard_behavior -v`
Expected: 初次退出保护后恢复为 400

- [ ] **Step 5: Commit**

```bash
git add code/app/motor_app.c
git commit -m "fix: restore pending speed after initial guard release"
```

### Task 3: 修复保护等待区舵机回中

**Files:**
- Modify: `code/app/balance_app.c`
- Modify: `code/app/balance_app.h`
- Modify: `code/app/motor_app.c`

- [ ] **Step 1: Write the failing test**

```python
def test_servo_centers_while_waiting_for_recover_delay():
    ...
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_motor_guard_behavior -v`
Expected: `servo_is_centered` 为 `False`

- [ ] **Step 3: Write minimal implementation**

```c
void balance_hold_servo_center(void)
{
    target_angle = 0.0f;
    target_gyro_rate = 0.0f;
    servo_output = 0.0f;
    servo_set(mid);
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_motor_guard_behavior -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add code/app/balance_app.c code/app/balance_app.h code/app/motor_app.c
git commit -m "fix: center servo throughout motor guard recovery window"
```
