#ifndef MODEM_FEP01
#define MODEM_FEP01

#include <Arduino.h>

//#define RECV_BUFF_SIZE 1024

class ModemFep01
{
	
  public:
	ModemFep01();
	int init( int myAddress, int groupAddress = 0xF0 );
	int send( int destAddress, char *buff, int numBytes, bool waitResponse );
	int available();
	int receive( int *senderAddress, char *buff, int buffSize, int msecTimeout );
	int setBaudrate( int baudrate );
	int getBaudrate();
	int baudrate();
	int setAddress( int myAddress, int groupAddress = 0xF0 );
	void clearBuffer();
	void factoryDefault();
	int setSendResponse( int flag );
	
	int writeRegister( int registerNum, int value );
	int readRegister( int registerNum );
	
	
  private:
  	HardwareSerial *mSerial;
  	int mBaudrate;

	int getCommandResponse( int msecTimeout = 1000 );

};

#endif