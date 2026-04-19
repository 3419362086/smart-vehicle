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

typedef enum
{
    UART_PID_PARSE_OK = 0,
    UART_PID_PARSE_INVALID_COUNT,
    UART_PID_PARSE_INVALID_NUMBER,
} uart_pid_parse_result_enum;

static volatile uint8 uart_pid_stream_enabled = 1U;
static uart_pid_state_enum uart_pid_state = UART_PID_STATE_STREAMING;
static uart_pid_target_enum uart_pid_pending_target = UART_PID_TARGET_NONE;
static char uart_pid_line_buffer[UART_PID_LINE_MAX_LEN];
static uint32 uart_pid_line_length = 0U;

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

void wireless_uart_printf(const char *format, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    wireless_uart_send_string(buffer);
}

uint8 wireless_uart_pid_stream_enabled(void)
{
    return uart_pid_stream_enabled;
}

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
