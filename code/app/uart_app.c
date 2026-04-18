#include "uart_app.h"
void wireless_uart_printf(const char *format, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    wireless_uart_send_string(buffer);
}