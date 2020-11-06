
#define YES 1
#define NO 0

#define MODE_ROVER 1
#define MODE_BASE 2
#define MODE_MOVING_BASE 3

//#define RECEIVER_MAX 3
#define RECEIVER_MAX 1

// gps interface  -1:UART CH1  positive: I2C address
#define IF_NONE 0
#define IF_UART -1

//#define UART_BUFF_MAX 4096
#define UART_BUFF_MAX 2048
#define I2C_BUFF_MAX 2048

#define I2C_READ 1
#define I2C_WRITE 2

void dbgPrintf( String msg );
void dbgPrintf( char* format, ... );
