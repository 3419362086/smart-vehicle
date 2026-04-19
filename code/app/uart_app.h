#ifndef CODE_UART_APP_H_
#define CODE_UART_APP_H_

#include "zf_common_headfile.h"

void wireless_uart_printf(const char *format, ...);
void wireless_uart_pid_service(void);
uint8 wireless_uart_pid_stream_enabled(void);

#endif
