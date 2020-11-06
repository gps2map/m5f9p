#define I2C_READ 1
#define I2C_WRITE 2

struct stI2cCommand {
	int address;
	int registerNum;	// 使わない時は-1
	int command;	// 0:none 1:read 2:write
	byte *buffer;
	int bufferBytes;
	int bytesToReadWrite;
	int result;	// 0:ok -1:timeout -2:error
};

void taskI2c(void* param);
int i2cReadBytes( int i2cAddress, int registerNum, int bytesToRead, byte* buffer );
int i2cWriteBytes( int i2cAddress, int registerNum, int bytesToWrite, byte* buffer );
int i2cWriteBytes2( int i2cAddress, int registerNum, int bytesToWrite, byte* buffer );
int i2cSend( int i2cAddress, int registerNum, int bytesToWrite, byte* buffer );
