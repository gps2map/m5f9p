/*

          ************************************************************
                              リングバッファ（byte配列）
          ************************************************************
          
---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/

#include "RingBuff.h"

RingBuff::RingBuff()
{
	mBuffer = NULL;
	mSize = 0;
	mReadIndex = 0;
	mWriteIndex = 0;
}

RingBuff::~RingBuff()
{
	if ( mBuffer ) free( mBuffer );
	mBuffer = NULL;
}

// 戻り値＝確保したバイト数
//         -1:エラー（メモリが確保できない）
//
int RingBuff::setSize( int bytes )
{
	mBuffer = (byte*) malloc( bytes );
	if ( mBuffer ) {
		mSize = bytes;
		return bytes;
	}
	else return -1;
}

// 戻り値＝リングバッファに書き込んだバイト数
//         負数：エラー
//
int RingBuff::write( byte* buffer, int bytes )
{
	
	if ( ! mBuffer ) return -1;
	
	int residue = bytes;
	while( residue ){
		int copyBytes = residue;
		if ( mWriteIndex + copyBytes > mSize ) copyBytes = mSize - mWriteIndex;
		memcpy( mBuffer + mWriteIndex, buffer, copyBytes );
		mWriteIndex += copyBytes;
		if ( mWriteIndex >= mSize ) mWriteIndex = 0;
		residue -= copyBytes;
	}
	return bytes;
}


// 戻り値＝リングバッファから読み出したバイト数
//         負数：エラー
//
int RingBuff::read( byte* buffer, int maxBytes )
{
	if ( ! mBuffer ) return -1;

	int bytesSaved = 0;	
	int residue = available();
	while( residue ){
		int copyBytes = residue;
		if ( bytesSaved + copyBytes > maxBytes ) copyBytes = maxBytes - bytesSaved;
		if ( mReadIndex + copyBytes > mSize ) copyBytes = mSize - mReadIndex;
		memcpy( buffer + bytesSaved, mBuffer + mReadIndex, copyBytes );
		mReadIndex += copyBytes;
		if ( mReadIndex >= mSize ) mReadIndex = 0;
		residue -= copyBytes;
		bytesSaved += copyBytes;
	}
	return bytesSaved;
	
}


int RingBuff::available()
{
	int bytes = mWriteIndex - mReadIndex;
	if ( bytes < 0 ) bytes += mSize;
	return bytes;
}

