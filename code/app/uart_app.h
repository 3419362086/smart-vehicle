#ifndef CODE_UART_APP_H_
#define CODE_UART_APP_H_

#include "zf_common_headfile.h"

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

void wireless_uart_printf(const char *format, ...);
void wireless_uart_pid_service(void);
uint8 wireless_uart_pid_stream_enabled(void);

#endif
