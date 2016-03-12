
// ANSI Escape Colors
// UART_printf(ANSI_RED"This is Red"ANSI_RESET);
// To see colors using minicom;
// sudo minicom -c on -b 115200 -D /dev/ttyUSB0
#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_BLUE    "\x1b[34m"
#define ANSI_MAGENTA "\x1b[35m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RESET   "\x1b[0m"
#define ANSI_CLEAR   "\x1b[2J"