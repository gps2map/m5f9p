/*

          ************************************************************
                            u-blox GNSS受信機制御用 UARTのみ
          ************************************************************

---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/

#include <Arduino.h>

#include "HardwareSerial.h"

#include "m5f9p.h"
#include "gps.h"
//#include "i2c.h"

#define PI 3.141592653589793
#define DEG2RAD 0.01745329251994
#define RAD2DEG 57.295779513082

#define WGS84_A 6378137
#define WGS84_F (1.0/298.257223563)

extern char mUartBuff[];
extern int mUartReadIndex;
extern int mUartWriteIndex;
//extern char mI2cBuff[RECEIVER_MAX][I2C_BUFF_MAX];
//extern int mI2cReadIndex[];
//extern int mI2cWriteIndex[];
extern int mOpeMode;
//extern int mI2cUartSyncMode;
extern HardwareSerial Serial1;

#define UBX_COMMAND_TIMEOUT 1000
#define UBX_SEND_BUFF_MAX 128
byte mUbxSendBuff[ UBX_SEND_BUFF_MAX ];

int byte2int( byte* pdata );

// ************************************************************
//                         測位データ
// ************************************************************

// 測位時刻を通算秒に変換する
//
// 戻り値= 1970/1/1 00:00:00からの秒数
//
double gpsDate2UnixTime( struct stGpsData *gpsData )
{
	struct tm t;
	memset( (void*)&t, 0, sizeof(t) );
	t.tm_year = gpsData->year;
	t.tm_mon = gpsData->month;
	t.tm_mday = gpsData->day;
	t.tm_hour = gpsData->hour;
	t.tm_min = gpsData->minute;
	t.tm_sec = gpsData->second;
	
	double sec = mktime( &t );
	sec += gpsData->msec / 1000.0;
	
	return sec;
}


// ECEF( Earth Centered Earth Fixed ) 座標を緯度経度座標に変換する
//
// x,y,z : ECEF座標（ｍ）
// lat,lon: 緯度経度（度）
// height: 楕円体高（ｍ）
//
// 戻り値＝繰り返し演算回数(４回程度）
//        -1: 極またはパラメータエラー（ x,y=0 )
//
// 出典：u-blox Application Note「Datum Transformations of GPS Positions」
//       逐次近似による解法を実装。高さ方向の誤差が0.1mm以下まで繰り返す。
//
int gpsEcef2Llh( double x, double y, double z, double *lat, double *lon, double *height )
{
	*lat = *lon = *height = 0;
	
	double p = sqrt( x * x + y * y );
	if ( p < 0.0001 ) return -1;
		
	
	double a = WGS84_A;
	double f = WGS84_F;
	double a2 = a * a;
	double b = a * ( 1 - f );
	double b2 = b * b;
	double e = sqrt( ( a2 - b2 ) / a2 );
	double e2 = e * e;

	double lamda = atan2( y, x );
	double h = 0;
	double phi = atan2( z, p * ( 1 - e2 ) );
	int count = 0;
	while(1){
		count++;
		double h0 = h;
		double sinPhi = sin( phi );
		double sinPhi2 = sinPhi * sinPhi;
		double n = a / sqrt( 1 - e2 * sinPhi2 );
		h = p / cos( phi ) - n;
		phi = atan2( z, p * ( 1 - e2 * n / ( n + h ) ) );
		if ( fabs( h - h0 ) < 0.0001 ) break;	// 高さの誤差が0.1mm以下なら終了
		h0 = h;
	}
	
	*lat = phi * RAD2DEG;
	*lon = lamda * RAD2DEG;
	*height = h;
	return count;
}


int gpsLlh2Ecef( double lat, double lon, double height, double *x, double *y, double *z )
{
	double phi = lat * DEG2RAD;
	double lamda = lon * DEG2RAD;
	double a = WGS84_A;
	double f = WGS84_F;
	double a2 = a * a;
	double b = a * ( 1 - f );
	double b2 = b * b;
	double e = sqrt( ( a2 - b2 ) / a2 );
	double e2 = e * e;
	double sinPhi = sin( phi );
	double sinPhi2 = sinPhi * sinPhi;
	double n = a / sqrt( 1 - e2 * sinPhi2 );
	
	double nhcos = ( n + height ) * cos( phi );
	*x = nhcos * cos( lamda );
	*y = nhcos * sin( lamda );
	*z = ( b2 * n / a2 + height ) * sinPhi;
	
	return 0;
} 


// ************************************************************
//                         ZED-F9P
// ************************************************************

// シリアルポート(UART1)のボーレートを設定する。
//
//
int gpsSetBaudrate( int baudrate )
{
	int nret;
	if ( baudrate < 1 ) return -1;
	
	Serial1.begin( 0, SERIAL_8N1, 26, 13);	// 0：自動ボーレート
	if ( Serial1.baudRate() == NULL ) {
		Serial1.begin( 4800, SERIAL_8N1, 26, 13);
		if ( Serial1.baudRate() == NULL )  return -2;	// ボーレートが特定できない
	}
	dbgPrintf("Current Serial1 baudrate=%d\r\n", Serial1.baudRate());

	for( int i=0; i < 3; i++ ){	// 1回ではボーレートが変わらない時がある。
		nret = gpsSetUartPort( IF_UART, 1, baudrate, 1, 1, 1, 1, 1, 0 );
		if ( nret < 0 ) return -3;
	}

	Serial1.begin( baudrate, SERIAL_8N1, 26, 13);
	if ( abs( Serial1.baudRate() - baudrate ) / (float)baudrate > 0.1 ){
		dbgPrintf( "baudrate error : %d != %d\r\n", Serial1.baudRate(), baudrate );
		return -3;
	}
	dbgPrintf("New Serial1 baudrate=%d\r\n", Serial1.baudRate());
	
	return 0;
}

// ZED-F9Pにデータを送る
//
int gpsWrite( int gpsInterface, char *buff, int numBytes )
{
	int nret,bytesWritten;

	if ( gpsInterface == IF_UART ){
		bytesWritten = Serial1.write( (byte *)buff, numBytes );
	}
	else {	// I2C
//		int i2cAddress = gpsInterface;
//		bytesWritten = f9pI2cWriteBytes( i2cAddress, numBytes, (byte *)buff );
	}

	return bytesWritten;
}

// 読み出し可能バイト数
//
/*
int f9pI2cNumBytes( int i2cAddress )
{
	byte buff[2];

	int nret = 	i2cReadBytes( i2cAddress, 0xfd, 2, buff );
	if ( nret != 2 ) return -1;

	int numBytes = buff[0] * 256 + buff[1];

	return numBytes;
}

// 
// ZED-F9Pのデータを読み出す。
//
// bytesToReadまたは32バイト読み出すか、bufferが一杯になった時に戻る。
// 32バイトの制約はBUFFER_LENGTH=32（Wire.h)による.
// Wireのライブラリのバージョンによっては128バイトの場合もあるようだ。
// 
int f9pI2cReadBytes( int i2cAddress, int bytesToRead, byte* buffer, int bufferBytes )
{
	if ( bytesToRead > 32 ) bytesToRead = 32;
	if ( bufferBytes < bytesToRead ) bytesToRead = bufferBytes;

	int nret = i2cReadBytes( i2cAddress, 0xff, bytesToRead, buffer );
	
	return nret;
}

//
// f9pの場合、レジスタは指定しない。
//
int f9pI2cWriteBytes( int i2cAddress, int bytesToWrite, byte* buffer )
{
	int nret = i2cWriteBytes( i2cAddress, -1, bytesToWrite, buffer );
	return nret;
}

*/

// ************************************************************
//                           UBX
// ************************************************************

//
// 戻り値＝0:正常終了
//        -1:I2Cアドレスエラー
//        -2,-3:Checksum エラー
//        -4:バッファオーバーフロー
//
int ubxDecode( int f9pInterface, struct stUbxStatus *ubxStatus )
{
	int nret, numBytesToRead,i2cIndex;
	byte* savePointer;
	
	int retCode = 0;
	if ( f9pInterface == IF_UART ){
		//numBytesToRead = Serial1.available();
		//savePointer = (byte*) mUartBuff;
		savePointer = (byte*) ( mUartBuff + mUartReadIndex );
		numBytesToRead = mUartWriteIndex - mUartReadIndex;
		if ( numBytesToRead < 0 ){
			numBytesToRead = UART_BUFF_MAX - mUartReadIndex;
		}
	}
	else {
		/*
		i2cIndex = f9pInterface - 0x42;
		if ( i2cIndex <= 0 ) return -1;
		
		savePointer = (byte*) ( mI2cBuff[i2cIndex] + mI2cReadIndex[i2cIndex] );
		numBytesToRead = mI2cWriteIndex[i2cIndex] - mI2cReadIndex[i2cIndex];
		if ( numBytesToRead < 0 ){
			numBytesToRead = I2C_BUFF_MAX - mI2cReadIndex[i2cIndex];
		}
		*/
	}
	
	int count = 0;
	retCode = ubxBuffDecode( savePointer, numBytesToRead, ubxStatus, &count );
	if ( f9pInterface == IF_UART ){
		mUartReadIndex += count;
		if ( mUartReadIndex == UART_BUFF_MAX ) mUartReadIndex = 0;
	}
	else {
		/*
		mI2cReadIndex[i2cIndex] += count;
		if ( mI2cReadIndex[i2cIndex] == I2C_BUFF_MAX ) mI2cReadIndex[i2cIndex] = 0;
		*/
	}
	
	return retCode;

}

// 戻り値＝0:正常終了
//        -2,-3:Checksum エラー
//        -4:バッファオーバーフロー
	
int ubxBuffDecode( byte *buffer, int numBytes, struct stUbxStatus *ubxStatus, int *decodedBytes )
{
	int count = 0;
	int retCode = 0;
	if ( numBytes > 0 ){
		for( int i=0; i < numBytes; i++ ){
			byte data8 = buffer[i];
			count++;
			switch( ubxStatus->statusNum ) 
			{
				case 0:	// idle
					if ( data8 != 0xb5 ) continue;
					ubxStatus->statusNum = 1;	// sync1 done
					break;
					
				case 1:	// sync1 done
					if ( data8 != 0x62 )
						ubxStatus->statusNum = 0;	// idle
					else
						ubxStatus->statusNum = 2;	// sync2 done
					ubxStatus->chksumA = 0;
					ubxStatus->chksumB = 0;
					break;					

				case 2:	// sync2 done
					ubxStatus->msgClass = data8;	
					ubxStatus->statusNum = 3;	// class done
					ubxStatus->chksumA += data8;
					ubxStatus->chksumB += ubxStatus->chksumA;
					break;	

				case 3:	// class done
					ubxStatus->msgId = data8;	
					ubxStatus->statusNum = 4;	// id done
					ubxStatus->chksumA += data8;
					ubxStatus->chksumB += ubxStatus->chksumA;
					break;

				case 4:	// id done
					ubxStatus->numbytes = data8;	
					ubxStatus->statusNum = 5;	// low byte done
					ubxStatus->chksumA += data8;
					ubxStatus->chksumB += ubxStatus->chksumA;
					break;

				case 5:	// low byte done
					ubxStatus->numbytes += data8 * 256;	
					ubxStatus->buffIndex = 0;
					ubxStatus->chksumA += data8;
					ubxStatus->chksumB += ubxStatus->chksumA;
					if ( ubxStatus->numbytes == 0 ) ubxStatus->statusNum = 8;	// data done
					else ubxStatus->statusNum = 6;	// high byte done
//Serial.printf("ubx cls=%d id=%d  n=%d\r\n",ubxStatus->msgClass,ubxStatus->msgId,ubxStatus->numbyte//s);
					break;

				case 6:	// hig	h byte done
				case 7:	// receiving payload
					ubxStatus->buff[ ubxStatus->buffIndex++ ] = data8;
					ubxStatus->chksumA += data8;
					ubxStatus->chksumB += ubxStatus->chksumA;
					if ( ubxStatus->numbytes != ubxStatus->buffIndex ) 
					{
						if ( ubxStatus->buffIndex == UBX_BUFF_MAX ){
							ubxStatus->statusNum = 0;	// idle
							retCode = -4; // overflow
							break;
						}
						ubxStatus->statusNum = 7;	// receiving payload
					}
					else ubxStatus->statusNum = 8;	// data done
					break;
				
				case 8:	// data done
					if ( ubxStatus->chksumA == data8 )
						ubxStatus->statusNum = 9;	// chksum A done
					else
					{
						ubxStatus->statusNum = 0;	// idle
						retCode = -2;	// check sum error
					}
					break;

				case 9:	// chksum A done
					if ( ubxStatus->chksumB == data8 )
						ubxStatus->statusNum = 10;	// data ready
					else
					{
						ubxStatus->statusNum = 0;	// idle
						retCode = -3;	// check sum error
					}
					break;
					
			}// switch
			if ( ubxStatus->statusNum == 10 || retCode < 0 ){
				break;
			}
		} // for

	} // if

	*decodedBytes = count;
	return retCode;
}

int ubxDecodeNavPvt( struct stUbxStatus *ubxStatus, struct stGpsData *gpsData)
{
	byte msgClass,msgId;

	memset( gpsData, 0, sizeof( stGpsData ) );
	msgClass = ubxStatus->msgClass;
	msgId = ubxStatus->msgId;
	byte *buff = ubxStatus->buff;

	if ( msgClass == 1 && msgId == 7 )		// NAV-PVT
	{
		u32 iTOW = *(u32 *) buff;
		u16 year = *(u16 *)(buff+4);	// Year(UTC)
		byte month = *(buff+6);			// Month 1..12
		byte day = *(buff+7);			// Day 1..31
		byte hour = *(buff+8);			// Hour 0..23
		byte min = *(buff+9);			// Minute 0..59
		byte sec = *(buff+10);			// Second 0..60
		byte valid = *(buff+11);		// bit 3:validMag 2:fullyResolved 1:validTime 0:validDate
		u32 tAcc = *(u32 *)(buff+12);	// Time accuracy (nsec)
		s32 nano = *(s32 *)(buff+16);	// Fraction of second (nsec)
		byte fixType = *(buff+20);		// fixType 5:time only 4:GNSS+DR 3:3D 2:2D 1:DR 0:no fix
		byte flags = *(buff+21);		// bit 76: 2=RTK fix 1=float 0=none 5:headVehValid 
										//     432:Power save mode 1:diffSoln 0:gnssFixOK
		byte flags2 = *(buff+22);		// bit 7:confirmedTime 6:confirmedDate 5:confirmedAvai
		byte numSV = *(buff+23);		// Number of satellites used in NAV Solution
		s32 lon = *(s32 *)(buff+24);	// Longitude (1E-7degree)
		s32 lat = *(s32 *)(buff+28);	// Latitude (1E-7degree)	
		s32 height = *(s32 *)(buff+32);	// Height above ellipsoid (mm)
		s32 hMSL = *(s32 *)(buff+36);	// Height above mean sea level (mm)
		u32 hAcc = *(u32 *)(buff+40);	// Horizontal accuracy estimate (mm)
		u32 vAcc = *(u32 *)(buff+44);	// Vertical accuracy estimate (mm)
		s32 velN = *(s32 *)(buff+48);	// NED north velocity (mm/s)
		s32 velE = *(s32 *)(buff+52);	// NED east velocity (mm/s)
		s32 velD = *(s32 *)(buff+56);	// NED down velocity (mm/s)
		s32 gSpeed = *(s32 *)(buff+60);	// Ground Speed (2-D) (mm/s)
		s32 headMot = *(s32 *)(buff+64); // Heading of motion (2-D) (1E-5degree)
		u32 sAcc = *(u32 *)(buff+68);	// Speed accuracy estimate (mm/s)
		u32 headAcc = *(u32 *)(buff+72); // Heading accuracy estimate (1E-5degree)
		u16 pDOP = *(u16 *)(buff+76);	// Position DOP (0.01)
		s32 headVeh = *(s32 *)(buff+84); // Heading of vehicle (2-D) (1E-5degree)
		s16 magDec = *(s16 *)(buff+88);	// Magnetic declination (0.01degree)
		u16 magAcc = *(u16 *)(buff+90); // Magnetic declination accuracy (0.01degree)
		
//		gpsData->millis = millis();
		gpsData->micros = micros();
		gpsData->iTOW = iTOW;
		gpsData->year = year;
		gpsData->month = month;
		gpsData->day = day;
		gpsData->hour = hour;
		gpsData->minute = min;
		gpsData->second = sec;
		gpsData->msec = nano * 1E-6;
		gpsData->lat = lat * 1E-7;	// degree
		gpsData->lon = lon * 1E-7;	// degree
		gpsData->height = height * 1E-3;	// m
		gpsData->highPrecisionDone = false;
		gpsData->geoidSep = ( height - hMSL ) * 1E-3; // geoid separation m
		gpsData->velocity = gSpeed * 1E-6;	// km/h
		gpsData->direction = headMot * 1E-5;	// degree

		int q = 0;
		int rtkFix = flags >> 6;
		if ( rtkFix == 2 ) q = 4;
		else if ( rtkFix == 1 ) q = 5;
		else if ( 4 >= fixType && fixType >= 2 ) q = 1;
		else if ( fixType == 1 ) q = 6;
		gpsData->quality = q;	// 0 = No fix, 1 = Autonomous GNSS fix, 
								// 2 = Differential GNSS fix, 4 = RTK fixed, 
								// 5= RTK float, 6 = Estimated/Dead reckoning fix:

		gpsData->numSatelites = numSV;
		gpsData->dop = pDOP * 0.01;
		
		gpsData->ubxDone = true;
	}
	else return -1;
	
	return 0;
}

// 高精度データを取得する
//
// ・この関数を呼ぶ前に、ubxDecodeNavPvt()の結果がgpsDataに格納されていないといけない。
//
int ubxDecodeHPPOSLLH( struct stUbxStatus *ubxStatus, struct stGpsData *gpsData)
{
	byte msgClass,msgId;

	msgClass = ubxStatus->msgClass;
	msgId = ubxStatus->msgId;
	byte *buff = ubxStatus->buff;

	if ( msgClass == 1 && msgId == 0x14 )	// NAV-HPPOSLLH
	{
		byte version = *buff;
		u32 iTOW = *(u32 *)(buff+4);
		s32 lon = *(s32 *)(buff+8);		// Longitude (1E-7degree)
		s32 lat = *(s32 *)(buff+12);	// Latitude (1E-7degree)	
		s32 height = *(s32 *)(buff+16);	// Height above ellipsoid (mm)
		s32 hMSL = *(s32 *)(buff+20);	// Height above mean sea level (mm)
		int lonHp = byte2int(buff+24);		// High precision componet of longitude(1E-9degree)
		int latHp = byte2int(buff+25);		// High precision componet of latitude(1E-9degree)
		int heightHp = byte2int(buff+26);	// High precision componet of hight(0.1mm)
		int hMSLHp = byte2int(buff+27);	// High precision componet of hight abobe ellipsoid(0.1mm)
		u32 hAcc = *(u32 *)(buff+28);	// Horizontal accuracy estimate (0.1mm)
		u32 vAcc = *(u32 *)(buff+32);	// Vertical accuracy estimate (0.1mm)
		if ( gpsData->iTOW == iTOW ){
			gpsData->lat = (double)lat * 1E-7 + (double)latHp * 1E-9;	// degree
			gpsData->lon = (double)lon * 1E-7 + (double)lonHp * 1E-9;	// degree
			gpsData->height = (double)height * 1E-3 + (double)heightHp * 1E-4;	// m
			gpsData->highPrecisionDone = true;
		}
	}
	else return -1;
	
	return 0;
}

// 何故か *(char*)(buff+24) のCASTができないのでこの関数が必要
int byte2int( byte* pdata )
{
	int c = *pdata;
	if ( c > 128 ) c = c - 256;
	return c;
}

// Survey-inの進行状況を取得する
//
// secObservation : 現在までの経過秒数
// mrAccuracy     : 現在の精度(ｍ単位）
//
// 戻り値＝ 1: 終了
//          0: 進行中
//       負数：エラー
//
int ubxDecodeNavSvin( struct stUbxStatus *ubxStatus, int *secObservation,
		float *mrAccuracy, double *x, double *y, double *z )
{
	byte msgClass,msgId;

	msgClass = ubxStatus->msgClass;
	msgId = ubxStatus->msgId;
	byte *buff = ubxStatus->buff;

	if ( msgClass != 1 || msgId != 0x3b ) return -1;
	*secObservation = *(u32 *)(buff + 8);
	*x = *(s32 *)(buff + 12) * 0.01;	// cm to meter
	*y = *(s32 *)(buff + 16) * 0.01;
	*z = *(s32 *)(buff + 20) * 0.01;
	*x += byte2int(buff + 24) * 1E-4;	// high precision, 0.1mm to meter
	*y += byte2int(buff + 25) * 1E-4;
	*z += byte2int(buff + 26) * 1E-4;
	*mrAccuracy = ( *(u32 *)(buff + 28) ) * 1E-4; // 0.1mm to meter
	
	byte valid = *(byte *)(buff + 36);
	byte active = *(byte *)(buff + 37);
	ubxStatus->statusNum = 0;

	if ( active ) return 0;
	if ( valid ) return 1;
	else return -2;
}


int ubxDecodeNavRelPosNed( struct stUbxStatus *ubxStatus, struct stGpsRelPos *relPosNed )
{
	byte msgClass,msgId;

	msgClass = ubxStatus->msgClass;
	msgId = ubxStatus->msgId;
	byte *buff = ubxStatus->buff;

	if ( msgClass != 1 || msgId != 0x3c ) return -1;
	byte version = *buff;
	u32 iTOW = *(u32 *)(buff+4);
	s32 relPosN = *(s32 *)(buff+8);		// North component of relative position vector (cm)
	s32 relPosE = *(s32 *)(buff+12);	// East component of relative position vector (cm)
	s32 relPosD = *(s32 *)(buff+16);	// Down component of relative position vector (cm)
	s32 relPosLength = *(s32 *)(buff+20);	// Length of the relative position vector (cm)
	s32 relPosHeading = *(s32 *)(buff+24);		// Heading of the relative position vector(1E-5 deg)
	relPosN += byte2int(buff+32) * 0.01;	// High-precision North component of relative position vector (0.1mm)
	relPosE += byte2int(buff+33) * 0.01;	// High-precision East component of relative position vector (0.1mm)
	relPosD += byte2int(buff+34) * 0.01;	// High precision componet of hight abobe ellipsoid(0.1mm)
	relPosLength += byte2int(buff+35) * 0.01;	// High-precision component of the length of the relative position vector (0.1mm)
	u32 flags = *(u32 *)(buff+60) & 0x1ff;
	
	
	if ( ! relPosNed ) return -2;
	relPosNed->ubxDone = true;
	relPosNed->iTOW = iTOW;
	relPosNed->micros = micros();
	relPosNed->north = relPosN;
	relPosNed->east = relPosE;
	relPosNed->down = relPosD;
	relPosNed->length = relPosLength;
	relPosNed->heading = relPosHeading * 1E-5;
	relPosNed->flags = flags;
	int quality = 0;
	int fix = (flags >> 3 ) & 0x3;
	if ( fix == 1 ) quality = 5;
	else if ( fix == 2 ) quality = 4;
	else if ( flags & 2 ) quality = 2;
	else if ( flags & 1 ) quality = 1;
	relPosNed->quality = quality;
	return flags;
}


// 衛星データを取得する
//
// satellites[]: 衛星データを格納する配列
// maxSatellites: 配列の要素数
//
// 戻り値＝ 取得した衛星数
//       負数：エラー
//
int ubxDecodeNavSat( struct stUbxStatus *ubxStatus, stGpsSat *satellites, int maxSatellites )
{
	byte msgClass,msgId;

	msgClass = ubxStatus->msgClass;
	msgId = ubxStatus->msgId;
	byte *buff = ubxStatus->buff;

	if ( msgClass != 1 || msgId != 0x35 ) return -1;
	u32 iTOW = *(u32 *)(buff);
	byte version = *(buff + 4);
	byte numSvs = *(buff + 5);
	int numSatellites = 0;
	for( int i=0; i < numSvs; i++ ){
		byte gnssId = *(buff + 8 + 12 * i);
		byte svId = *(buff + 9 + 12 * i);
		byte cno = *(buff + 10 + 12 * i);
		char elev = (char) byte2int(buff + 11 + 12 * i);
		short azim = *(short*)(buff + 12 + 12 * i);
		short prRes = *(short*)(buff + 14 + 12 * i);
		unsigned int flags = *(unsigned int*)(buff + 16 + 12 * i);

		stGpsSat* pSat = &satellites[ numSatellites++ ];
		pSat->iTOW = iTOW;
		pSat->gnssId = gnssId;
		pSat->svId = svId;
		pSat->cno = cno;
		pSat->elev = elev;
		pSat->azim = azim;
		pSat->prRes = prRes;
		pSat->flags = flags;
		
		if ( numSatellites == maxSatellites ) break;
	}
	
	ubxStatus->statusNum = 0;

	return numSatellites;
}

// DOPを取得する
//
//
int ubxDecodeNavDop( struct stUbxStatus *ubxStatus, struct stGpsData *gpsData)
{
	byte msgClass,msgId;

	msgClass = ubxStatus->msgClass;
	msgId = ubxStatus->msgId;
	byte *buff = ubxStatus->buff;

	if ( msgClass == 1 && msgId == 0x04 )	// NAV-DOP
	{
		u32 iTOW = *(u32 *)(buff);
		u16 gDOP = *(u16 *)(buff+4);	// Geometric DOP (0.01)
		u16 pDOP = *(u16 *)(buff+6);	// Position DOP (0.01)
		u16 tDOP = *(u16 *)(buff+8);	// Time DOP (0.01)
		u16 vDOP = *(u16 *)(buff+10);	// Vertical DOP (0.01)
		u16 hDOP = *(u16 *)(buff+12);	// Horizontal DOP (0.01)
		u16 nDOP = *(u16 *)(buff+14);	// Northing DOP (0.01)
		u16 eDOP = *(u16 *)(buff+16);	// Easting DOP (0.01)
		gpsData->dop = pDOP * 0.01;
		gpsData->hDop = hDOP * 0.01;
		gpsData->vDop = vDOP * 0.01;
	}
	else return -1;
	
	return 0;
}

// Ublox GPS
//

// UBXコマンドを設定する。
//
//  SyncChar1   SyncChar2    CLASS  ID   Length  Payload  checkA  checkB
//    
//   1byte        1byte      1byte 1byte  2byte     n      1byte  1byte
//
// 戻り値＝コマンド全体のバイト数
//
int ubxSetCommand( byte msgClass, byte msgId, int numLength, byte *payLoad )
{
	if ( numLength + 8 > UBX_SEND_BUFF_MAX ) return -1;
	
	mUbxSendBuff[0] = 0xb5;		//Sync Char1
	mUbxSendBuff[1] = 0x62;		//Sync Char2
	mUbxSendBuff[2] = msgClass;
	mUbxSendBuff[3] = msgId;
	mUbxSendBuff[4] = numLength & 0xff;
	mUbxSendBuff[5] = numLength >> 8;
	memcpy(mUbxSendBuff+6, payLoad, numLength);

	// チェックサムの計算
	byte ckA,ckB,*p;
	int i,n;
	n = 2 + 2 + numLength;
	p = mUbxSendBuff + 2;
	ckA = 0;
	ckB = 0;
	for( i = 0; i < n; i++ )
	{
		ckA += *p++;
		ckB += ckA;
	}
	*p++ = ckA;
	*p = ckB;
	
	return 2+n+2;
}

int ubxSendCommand( int gpsInterface, byte msgClass, byte msgId, int numLength, byte *payLoad )
{
	int numBytes = ubxSetCommand( msgClass, msgId, numLength, payLoad );
	if ( numBytes < 0 ) return -3;

	int nret = ubxSendCommand( gpsInterface, numBytes, UBX_COMMAND_TIMEOUT );

	return nret;
}

// msecTimeout: Ackを待つ時間  0=Ackを待たない
// 
// 注意：Ackを待つ場合、taskGetSensorData()が走っていないといけない
//
int ubxSendCommand( int gpsInterface, int numBytes, int msecTimeout )
{
	int bytesWritten;
	if ( gpsInterface == 0 ) return -1;
//dbgPrintf("mUbxSendBuff[0]=%x\r\n",mUbxSendBuff[0]);
	byte msgClass = mUbxSendBuff[2];
	byte msgId = mUbxSendBuff[3];
	
	delay(10);

	unsigned long msecStart = millis();
	int retCode = 0;
	int currentOpMode = mOpeMode;
	mOpeMode = MODE_ROVER;
	while(1){
		if ( gpsInterface == IF_UART ){
			bytesWritten = Serial1.write( mUbxSendBuff, numBytes );
		}
		else {	// I2C
		/*
			int i2cAddress = gpsInterface;
			bytesWritten = f9pI2cWriteBytes( i2cAddress, numBytes, mUbxSendBuff );
			*/
		}
		
		if ( numBytes != bytesWritten ) {
			retCode = -2;
			break;
		}
		
		if ( msecTimeout == 0 ) break;
	
		int nret = gpsGetAck( gpsInterface, msgClass, msgId, 100 );
		if ( nret == 0 ) break;
		if ( millis() - msecStart > msecTimeout ){
			retCode = -1;
			break;
		}
		delay(10);
	}
	mOpeMode = currentOpMode;
	
	if ( retCode < 0 ) {
		dbgPrintf( "ubxSendCommand error nret = %d\r\n", retCode );
		return retCode;
	}
	else return bytesWritten;
}



// GPSのメッセージ出力レートを設定する。
//
// rate : 0 = 出力しない　　n = 測位レート / n
//
// 戻り値: 0=正常終了
//         負数=エラー
//
int gpsSetMessageRate( int gpsInterface, byte msgClass, byte msgId, int rate )
{
	byte buff[3];

	buff[0] = msgClass;
	buff[1] = msgId;
	buff[2] = rate;
	
	int numBytes = ubxSetCommand(0x06, 0x01, 3, buff );
	if ( numBytes < 0 ) return -1;

	int sentBytes = ubxSendCommand( gpsInterface, numBytes, UBX_COMMAND_TIMEOUT );
	if ( sentBytes != numBytes ) return -2;
	
	return 0;
}

// GPSの測位レートを設定する。
//
// rate : ms単位の測位間隔
//
// 戻り値: 0=正常終了
//         負数：エラー
//
int gpsSetMeasurementRate( int gpsInterface, int rate )
{
	byte buff[6];

	buff[0] = rate;
	buff[1] = rate >> 8;
	buff[2] = 1;	// 1cycle fixed
	buff[3] = 0;
	buff[4] = 0;	// UTC alignment
	buff[5] = 0;	
	
	int numBytes = ubxSetCommand(0x6, 0x08, 6, buff );
	if ( numBytes < 0 ) return -1;

	int sentBytes = ubxSendCommand( gpsInterface, numBytes, UBX_COMMAND_TIMEOUT );
	if ( sentBytes != numBytes ) return -2;

	return 0;
}

// GPSのUARTポートを設定する。
//
// portId:  1:UART1  2:UART2
// ubxIn:  0=入力UBXメッセージを無効 1=有効
// nmeaIn: 0=入力NMEAメッセージを無効 1=有効
// rtcm3In: 0=入力RTCM3メッセージを無効 1=有効
// ubxOut:  0=UBXメッセージを出力しない 1=出力する
// nmeaOut: 0=NMEAメッセージを出力しない 1=出力する
// rtcm3Out: 0=RTCM3メッセージを出力しない 1=出力する
//
//
// 戻り値: 0=正常終了
//         負数：エラー
//
int gpsSetUartPort( int gpsInterface, int portId, unsigned int baudRate, 
								int ubxIn, int nmeaIn, int rtcm3In,
								int ubxOut, int nmeaOut, int rtcm3Out )
{
	byte buff[20];

	memset( buff, 0, 20 );
	buff[0] = portId;
	buff[4] = 0xC0;	// 8 bit data
	buff[5] = 0x08;	// 1 stop bit, no parity
	*(unsigned int*)(buff + 8) = baudRate;

	int inFlag = 0;
	if ( ubxIn ) inFlag += 1;
	if ( nmeaIn ) inFlag += 2;
	if ( rtcm3In ) inFlag += 0x20;
	int outFlag = 0;
	if ( ubxOut ) outFlag += 1;
	if ( nmeaOut ) outFlag += 2;
	if ( rtcm3Out ) outFlag += 0x20;
	buff[12] = inFlag;
	buff[14] = outFlag;
	
	int numBytes = ubxSetCommand(0x6, 0x00, 20, buff );
	if ( numBytes < 0 ) return -1;

	int sentBytes = ubxSendCommand( gpsInterface, numBytes, 0 );
	if ( sentBytes != numBytes ) return -2;

	return 0;
}

// GPSをリセットする。
//
// 戻り値: 0=正常終了
//         負数：エラー
//
int gpsReset( int gpsInterface )
{
	byte buff[4];

	buff[0] = 0;	// 0-1: 0,0 HotStart
	buff[1] = 0;
	buff[2] = 0;	// 0 Hardware Reset
	buff[3] = 0;
	
	int numBytes = ubxSetCommand(0x6, 0x04, 4, buff );
	if ( numBytes < 0 ) return -1;

	int sentBytes = ubxSendCommand( gpsInterface, numBytes, 0 );
	if ( sentBytes != numBytes ) return -2;
	
	return 0;
}

/*
// GPSのログ用フラッシュにバイトデータを書き込む
//
// numWriteBytes: 100以下
//
// 戻り値: 0=正常終了
//         負数：エラー
//
int ubxLogWriteString( int gpsInterface, char *buff, int numWriteBytes )
{
	int numBytes = ubxSetCommand(0x21, 0x04, numWriteBytes, buff );
	if ( numBytes < 0 ) return -1;

	int sentBytes = ubxSendCommand( gpsInterface, numBytes, UBX_COMMAND_TIMEOUT );
	if ( sentBytes != numBytes ) return -2;

	return 0;
}

// GPSのログ用フラッシュからバイトデータを読み出す
//
//
// 戻り値: 0=正常終了
//         負数：エラー
//
int ubxLogReadString( int gpsInterface, char *buff, int numBuffBytes )
{
	int numBytes = ubxSetCommand(0x21, 0x04, numWriteBytes, buff );
	if ( numBytes < 0 ) return -1;

	int sentBytes = ubxSendCommand( gpsInterface, numBytes, UBX_COMMAND_TIMEOUT );
	if ( sentBytes != numBytes ) return -2;

	return 0;
}
*/


// 基準局アンテナの位置座標を設定する。
//
// mode= 0: 座標の設定を行わない
//       1: 座標を自動設定。degLat,degLon,mHeightは使用しない。
//       2: 座標を数値で指定。secSurveyin,mmSurveyinAccuracyは使用しない。
// degLat,degLon,mHeight: 基準局アンテナの座標（緯度経度は度単位、高さはｍ単位）
// secSurveyin: 座標を自動設定する場合、座標の平均を取る最少の秒数。
// mmSurveyinAccuracy: 座標を自動設定する場合、平均値の標準偏差の最大値(単位ｍ）
//          自動設定は秒数及び標準偏差の両方が満たされた場合に終了する。
//
int gpsSetBaseCoordinate( int gpsInterface, int mode, double degLat, double degLon, double mHeight,
					unsigned int secSurveyin, float mSurveyinAccuracy )
{
	byte buff[40];
	memset( buff, 0, sizeof(buff) );
	buff[2] = mode;
	buff[3] = 1;	// LLA
	
	int degLatE7 = (int)(degLat * 1E7);
	int degLonE7 = (int)(degLon * 1E7);
	int degLatE9 = (int )((degLat * 1E7 - degLatE7) * 100 + 0.5 );
	if ( degLat >= 100 ) degLatE9 = 99;
	if ( degLat <= -100 ) degLatE9 = -99;
	int degLonE9 = (int )((degLon * 1E7 - degLonE7) * 100 );
	if ( degLon >= 100 ) degLonE9 = 99;
	if ( degLon <= -100 ) degLonE9 = -99;
	int cmHeight = (int)( mHeight * 100 );
	int mm01Height = (int)( ( mHeight * 100 - cmHeight ) * 100 + 0.5 );
	
	*(int*)(buff + 4) = degLatE7;
	*(int*)(buff + 8) = degLonE7;
	*(int*)(buff + 12) = cmHeight;
	buff[16] = degLatE9;
	buff[17] = degLonE9;
	buff[18] = mm01Height;
	
	*(unsigned int*)(buff + 24) = secSurveyin;
	*(unsigned int*)(buff + 28) = mSurveyinAccuracy * 1E4;

	int numBytes = ubxSetCommand( 0x06, 0x71, 40, buff );
	if ( numBytes < 0 ) return -1;

	int sentBytes = ubxSendCommand( gpsInterface, numBytes, UBX_COMMAND_TIMEOUT );
	if ( sentBytes != numBytes ) return -2;

	return 0;
}


// Ack,Nakを取得する。
//
// 戻り値＝　0：Ackを受信した
//           1: Nak　　〃
//          -1:タイムアウト
//
int gpsGetAck( int gpsInterface, byte msgClass, byte msgId, int msecTimeout )
{
	struct stUbxStatus ubxStatus;
	memset( (void*) &ubxStatus, 0, sizeof(ubxStatus) );

	unsigned long msecStart = millis();
	int found = 0;
	int nret = -1;
//	mI2cUartSyncMode = 0;
	while(1){
		if ( millis() - msecStart > msecTimeout ) break;
		int nret = ubxDecode( gpsInterface, &ubxStatus );
		if ( nret == 0 && ubxStatus.statusNum == 10 ){
			if ( ubxStatus.msgClass == 0x05 ){
				byte *buff = ubxStatus.buff;
				if ( buff[0] == msgClass && buff[1] == msgId ){
					found = 1;
					break;
				}
			}
			ubxStatus.statusNum = 0;
		}
		delay(1);
	}
	if ( found ){
		if ( ubxStatus.msgId == 1 ) nret = 0;
		else nret = 1;
	}
dbgPrintf("Ack class=%d id=%d nret=%d\r\n", msgClass, msgId, nret );
//	mI2cUartSyncMode = 1;
	return nret;
}

// ************************************************************
//                           RTCM
// ************************************************************

int gpsCheckRtcm( byte *buffer, int numBytes )
{
	byte* p = buffer;
	for( int i=0; i < numBytes; i++ ){
		if ( *p++ != 0xD3 ) continue;
		if ( *p & 0xfc ) continue;
		int length = *p++;
		length = ( length << 8 ) + *p++;
		int msgId = ( ( *p++ << 8 ) + ( *p++ & 0xf0 ) ) >> 4;
		dbgPrintf( "RTCM id=%d  length=%d\r\n", msgId, length );
		p += length - 3;
		i = p - buffer;
		if ( i >= numBytes ) break;
	}
	return 0;
}

