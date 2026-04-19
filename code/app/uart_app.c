#include "uart_app.h"
#include "balance_app.h"
#include <stdlib.h>
#include <string.h>

/* 单行命令缓冲：仅在收到\r或\n时整行解析 */
static char uart_pid_line_buffer[UART_PID_LINE_MAX_LEN];
static int  uart_pid_line_length = 0;

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

static char *uart_pid_trim(char *text)
{
    char *end = NULL;

    while ((*text == ' ') || (*text == '\t')) { text++; }

    if ('\0' == *text) { return text; }

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

/* 解析5个逗号分隔浮点数；成功返回1，失败返回0 */
static int uart_pid_parse_params(char *text, float params[UART_PID_PARAM_COUNT])
{
    char *cursor = text;
    char *end = NULL;
    int i = 0;

    for (i = 0; i < (int)UART_PID_PARAM_COUNT; i++)
    {
        while ((*cursor == ' ') || (*cursor == '\t')) { cursor++; }

        if ('\0' == *cursor) { return 0; }

        params[i] = strtof(cursor, &end);
        if (end == cursor) { return 0; }

        while ((*end == ' ') || (*end == '\t')) { end++; }

        if (i < (int)(UART_PID_PARAM_COUNT - 1U))
        {
            if (*end != ',') { return 0; }
            cursor = end + 1;
        }
        else if (*end != '\0')
        {
            return 0;
        }
    }

    return 1;
}

static void uart_pid_send_current(uart_pid_target_enum target)
{
    const char *name = NULL;
    PID_T *pid = uart_pid_get_target(target, &name);

    if (NULL == pid) { return; }

    wireless_uart_printf("[OK] %s: %f,%f,%f,%f,%f\r\n",
                         name, pid->kp, pid->ki, pid->kd, pid->target, pid->limit);
}

static void uart_pid_apply_params(uart_pid_target_enum target, const float params[UART_PID_PARAM_COUNT])
{
    PID_T *pid = uart_pid_get_target(target, NULL);

    if (NULL == pid) { return; }

    pid->kp     = params[0];
    pid->ki     = params[1];
    pid->kd     = params[2];
    pid->target = params[3];
    pid->limit  = params[4];
    pid_reset(pid);
}

/*
 * @brief 处理一行命令
 * @note  未知命令、错误格式、错误参数一律静默丢弃，不发回复，避免污染串口。
 *        仅在 stop / check / change 成功时回复 [OK] xxx。
 */
static void uart_pid_handle_line(char *line)
{
    float params[UART_PID_PARAM_COUNT];
    char *command = uart_pid_trim(line);
    uart_pid_target_enum target = UART_PID_TARGET_NONE;
    const char *name = NULL;
    char *open_paren = NULL;
    char *close_paren = NULL;
    char *target_name = NULL;

    if ('\0' == *command) { return; }

    /* stop：紧急停车 */
    if (0 == strcmp(command, "stop"))
    {
        motor_stop();
        wireless_uart_printf("[OK] motor stopped\r\n");
        return;
    }

    /* check-xxx_pid：打印当前PID参数 */
    if (0 == strncmp(command, "check-", 6))
    {
        target = uart_pid_target_from_name(uart_pid_trim(command + 6));
        if (UART_PID_TARGET_NONE == target) { return; }
        uart_pid_send_current(target);
        return;
    }

    /* change-xxx_pid(p1,p2,p3,p4,p5)：单行修改并立即应用 */
    if (0 == strncmp(command, "change-", 7))
    {
        target_name = command + 7;

        open_paren  = strchr(target_name, '(');
        close_paren = strrchr(target_name, ')');
        if ((NULL == open_paren) || (NULL == close_paren) || (close_paren < open_paren))
        {
            return;
        }

        *open_paren  = '\0';   /* 截断目标名 */
        *close_paren = '\0';   /* 截断参数串 */

        target = uart_pid_target_from_name(uart_pid_trim(target_name));
        if (UART_PID_TARGET_NONE == target) { return; }

        if (0 == uart_pid_parse_params(uart_pid_trim(open_paren + 1), params))
        {
            return;
        }

        uart_pid_apply_params(target, params);
        uart_pid_get_target(target, &name);
        wireless_uart_printf("[OK] %s updated\r\n", name);
        return;
    }

    /* 未知命令：静默丢弃 */
}

void wireless_uart_printf(const char *format, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    wireless_uart_send_string(buffer);
}

void wireless_uart_pid_service(void)
{
    unsigned char rx_buffer[WIRELESS_UART_BUFFER_SIZE];
    int rx_len = (int)wireless_uart_read_buffer(rx_buffer, sizeof(rx_buffer));
    int i = 0;

    for (i = 0; i < rx_len; i++)
    {
        char ch = (char)rx_buffer[i];

        /* 收到行结束符：整行解析；空行直接跳过 */
        if (('\r' == ch) || ('\n' == ch))
        {
            if (0 != uart_pid_line_length)
            {
                uart_pid_line_buffer[uart_pid_line_length] = '\0';
                uart_pid_handle_line(uart_pid_line_buffer);
                uart_pid_line_length = 0;
            }
            continue;
        }

        /* 缓冲未满则累积；满了静默丢弃多余字节，等下次\r/\n重新开始 */
        if (uart_pid_line_length < (int)(UART_PID_LINE_MAX_LEN - 1U))
        {
            uart_pid_line_buffer[uart_pid_line_length++] = ch;
        }
    }
}
