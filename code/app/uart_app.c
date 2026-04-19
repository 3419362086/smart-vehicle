#include "uart_app.h"

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

static volatile uint8 uart_pid_stream_enabled = 1U;
static uart_pid_state_enum uart_pid_state __attribute__((unused)) = UART_PID_STATE_STREAMING;
static uart_pid_target_enum uart_pid_pending_target __attribute__((unused)) = UART_PID_TARGET_NONE;
static char uart_pid_line_buffer[UART_PID_LINE_MAX_LEN] __attribute__((unused));
static uint32 uart_pid_line_length __attribute__((unused)) = 0U;

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
}
