#ifndef CODE_UART_APP_H_
#define CODE_UART_APP_H_

#include "zf_common_headfile.h"

#define UART_PID_LINE_MAX_LEN    (128U)
#define UART_PID_PARAM_COUNT     (5U)

typedef enum
{
    UART_PID_TARGET_NONE = 0,
    UART_PID_TARGET_STEERING,
    UART_PID_TARGET_ANGLE,
    UART_PID_TARGET_GYRO,
} uart_pid_target_enum;

void wireless_uart_printf(const char *format, ...);
void wireless_uart_pid_service(void);

#endif
