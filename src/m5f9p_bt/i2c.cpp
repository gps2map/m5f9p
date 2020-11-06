/*
          ************************************************************
                            I2Cインターフェース 制御用
          ************************************************************

---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/

#include <M5Stack.h>
#include <Wire.h>

#include "m5f9p.h"
#include "i2c.h"


extern xSemaphoreHandle mMutexI2c;
extern struct stI2cCommand mI2cCommand;


// I2C アクセススレッド
//
// mI2cCommandにセットされたコマンドを実行する。
//
// result= 読み書きしたバイト数
//         -1:タイムアウト
//         -2以下：エラー
//      
void taskI2c(void* param)
{
	int nret;
	Wire.begin();
	Wire.setClock( 400000 );
	while(1)
	{
		vTaskDelay(1);

		int command = mI2cCommand.command;
		if ( command == 0 ) continue;
		
		int address = mI2cCommand.address;
		byte *buffer = mI2cCommand.buffer;
		int result = 0;
		int registerNum = mI2cCommand.registerNum;

		if ( registerNum >= 0 ){
			Wire.beginTransmission( address );
			Wire.write( registerNum );
			Wire.endTransmission(false);	// falseでないと読み出しができない場合がある
		}

		if ( command == I2C_WRITE ){
			int bytesToWrite = mI2cCommand.bytesToReadWrite;
			int residue = bytesToWrite;
			int sentBytesAll = 0;
			int maxBytes = 32;			// I2C用バッファの最大値は32とする
			while ( residue > 0 ){
				Wire.beginTransmission( address );
				int sendBytes;
				if ( residue > maxBytes ) sendBytes = maxBytes;
				else sendBytes = residue;
		
				int sentBytes = Wire.write( buffer +  sentBytesAll, sendBytes );

				if ( residue <= maxBytes ) nret = Wire.endTransmission(true);
				else nret = Wire.endTransmission(false);
				if ( nret != 0 ){
					if ( nret == 17 ) result = -1;
					else result = -2;
					break;
				}
		
				sentBytesAll += sentBytes;
				residue = bytesToWrite - sentBytesAll;
			}
			if ( residue <= 0 ) result = bytesToWrite;
		}
		else if ( command == I2C_READ ){
			int bytesToRead = mI2cCommand.bytesToReadWrite;
			mI2cCommand.bufferBytes = bytesToRead;	// とりあえずバッファ容量チェックは無し
			if ( bytesToRead > mI2cCommand.bufferBytes ) bytesToRead = mI2cCommand.bufferBytes;
			Wire.requestFrom( address, bytesToRead );
			int bytes = Wire.available();
			if ( bytes == 0 ) result = -1;
			else {
				for( int i=0; i < bytes; i++ ){
					buffer[i] = Wire.read();
				}
				result = bytes;
			}
		}
		mI2cCommand.result = result;
		mI2cCommand.command = 0;
	}
}


// I2C 読み出し
//
// i2cAddress: I2Cアドレス
// registerNum : レジスタ番号（使わない時は-1)
//
// 戻り値= 読み出したバイト数
//         負数:エラー（ -1 ～ -4:タイムアウト　-5以下：その他のエラー）
//         
int i2cReadBytes( int i2cAddress, int registerNum, int bytesToRead, byte* buffer )
{
	int nret;
	int msecTimeout = 2000;
	unsigned long msecStart = millis();
	while( mI2cCommand.command ) {
		if ( millis() - msecStart > msecTimeout ) return -1;
		delay(1);
	}
	if ( ! xSemaphoreTake( mMutexI2c, pdMS_TO_TICKS(1000) )) return -2;
	
	mI2cCommand.address = i2cAddress;
	mI2cCommand.registerNum = registerNum;
	mI2cCommand.buffer = buffer;
	mI2cCommand.bytesToReadWrite = bytesToRead;
	mI2cCommand.command = I2C_READ;
	msecStart = millis();
	nret = 0;
	while( mI2cCommand.command ) {
		delay(1);
		if ( millis() - msecStart > msecTimeout ) {
			nret = -3;
			break;
		}
	}
	if ( nret < 0 ) goto ret;
	if ( mI2cCommand.result < 1 ){
		if ( mI2cCommand.result == -1 ) nret = -4;
		else nret = -5;
		goto ret;
	}
	nret = mI2cCommand.result;

ret:
if ( nret != bytesToRead ) dbgPrintf("i2cReadBytes nret=%d\r\n",nret);
	xSemaphoreGive( mMutexI2c );
	return nret;
}

// I2C　書き込み
//
// i2cAddress: I2Cアドレス
// registerNum : レジスタ番号（使わない時は-1)
// 戻り値＝ 書き込んだバイト数
//         -1..-4:タイムアウト
//         -5以下:エラー
//
int i2cWriteBytes( int i2cAddress, int registerNum, int bytesToWrite, byte* buffer )
{
	int nret;
	int msecTimeout = 2000;
	unsigned long msecStart = millis();
	while( mI2cCommand.command ) {
		if ( millis() - msecStart > msecTimeout ) return -1;
		delay(1);
	}

	if ( ! xSemaphoreTake( mMutexI2c, pdMS_TO_TICKS(1000) )) return -2;

	nret = i2cWriteBytes2( i2cAddress, registerNum, bytesToWrite, buffer );
	
	xSemaphoreGive( mMutexI2c );
	return nret;
	
}

int i2cWriteBytes2( int i2cAddress, int registerNum, int bytesToWrite, byte* buffer )
{
	int nret;
	
	nret = i2cSend( i2cAddress, registerNum, bytesToWrite, buffer );
	if ( nret < 0 ){
		if ( nret == -1 || nret == -2 ) nret = -4;
		else nret = -6;
		goto ret;
	}
	else nret = bytesToWrite;

ret:
	return nret;

}	

// 
// 戻り値＝ 0:正常終了
//         -1:タイムアウト
//         -2:タイムアウト
//         -3:エラー
//
int i2cSend( int i2cAddress, int registerNum, int bytesToWrite, byte* buffer )
{
	int msecTimeout = 500;
	mI2cCommand.address = i2cAddress;
	mI2cCommand.registerNum = registerNum;
	mI2cCommand.buffer = buffer;
	mI2cCommand.bytesToReadWrite = bytesToWrite;
	mI2cCommand.command = I2C_WRITE;

	unsigned long msecStart = millis();
	while( mI2cCommand.command ) {
		delay(1);
		if ( millis() - msecStart > msecTimeout ) return -2;
	}
	if ( mI2cCommand.result < 0 ){
		if ( mI2cCommand.result == -1 ) return -1;
		else return -3;
	}

	return 0;
}
	
