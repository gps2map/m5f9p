#ifndef RINGBUFF_H
#define RINGBUFF_H

#include <Arduino.h>

class RingBuff {
  public:
	byte* mBuffer;
	int mSize;
	int mWriteIndex;
	int mReadIndex;
	
	RingBuff();
	~RingBuff();
	int setSize( int bytes );
	int write( byte* buffer, int bytes );
	int read( byte* buffer, int maxBytes );
	int available();

};

#endif
