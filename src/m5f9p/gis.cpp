/*

          ************************************************************
                           地理的座標変換、距離計算等
          ************************************************************


---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/



// ************************************************************
//                         座標、投影
// ************************************************************

#include <Arduino.h>
#include <M5Stack.h>

#include "m5f9p.h"
#include "gps.h"
#include "TcpClient.h"

#define PI 3.141592653589793
#define DEG2RAD 0.01745329251994
#define RAD2DEG 57.295779513082

// GRS80楕円体
#define GRS80_A	6378137.0
#define GRS80_E 0.006694380022900788
#define GRS80_F (1.0/298.257222101)



// 緯度経度をXY座標に変換する
//
// phi0,lamda0 : XY座標の原点の緯度経度（度単位）
// phi,lamda   : XY座標を求める点の緯度経度（度単位）
// *x,*y       : XY座標（ｍ単位）

// X軸:北向　Y軸:東向
// ２点の距離は１ｋｍ以下とする

int latlon2xy( double phi0, double lamda0, double phi, double lamda, 
					double *x, double *y )
{
	
	double phi0r = phi0 * DEG2RAD;
	double phir = phi * DEG2RAD;
	
	double s = sin( phi0r );
	double F = sqrt( 1 - GRS80_E * s * s );
	
	*x = GRS80_A * ( 1 - GRS80_E ) * ( phir - phi0r ) / ( F * F * F );
	*y = GRS80_A * cos( phi0r ) * ( lamda - lamda0 ) * DEG2RAD / F;
	
	return 0;
}

// 測位データ間のベクトル（NEU座標系)を求める
//
// gpsData0からgpsDataに向かうベクトル
//
// X(N)軸:北向　Y(E)軸:東向　Z(U)軸：天頂方向　単位ｍ
// ２点の距離は１ｋｍ以下とする
//
int getPosVector( struct stGpsData *gpsData0, struct stGpsData *gpsData, 
					double *x, double *y, double *z )
{
	int nret = latlon2xy( gpsData0->lat, gpsData0->lon, gpsData->lat, gpsData->lon,
							x, y );
	*z = gpsData->height - gpsData0->height;
	
	return 0;
}


// 楕円体（GRS80)上の２点間の距離
//
// phi1,lamda1 : 点１の緯度経度（度）
// phi2,lamda2 : 点２の緯度経度（度）
//
// 戻り値＝ 距離（ｍ）
//
// 出典：「Spheroidal Geodesics,Reference Systems, & Local Geometry」
//        https://apps.dtic.mil/dtic/tr/fulltext/u2/703541.pdf
// 著者： P.D.THOMAS
//
// 精度：北海道（45,145)～沖縄(24,124)間、約3000kmの距離での比較
//       地理院の「距離と方位角の計算」での計算結果より5mm短い程度。
// 
double thomasDistance( double phi1, double lamda1, double phi2, double lamda2 )
{
	if ( abs( phi2 - phi1 ) < 1E-9 && abs( lamda2 - lamda1 ) < 1E-9 ) return 0;
	
	phi1 *= DEG2RAD;
	lamda1 *= DEG2RAD;
	phi2 *= DEG2RAD;
	lamda2 *= DEG2RAD;
	
	double a = GRS80_A;
	double f = GRS80_F;
	double theta1 = atan( (1-f) * tan( phi1 ) );
	double theta2 = atan( (1-f) * tan( phi2 ) );
	double thetaM = ( theta1 + theta2 ) / 2;
	double dThetaM = ( theta2 - theta1 ) / 2;
	double dLamda = lamda2 - lamda1;
	double dLamdaM = dLamda / 2;
	double sinDTM = sin( dThetaM );
	double cosDTM = cos( dThetaM );
	double sinTM = sin( thetaM );
	double cosTM = cos( thetaM );
	double sinDLM = sin( dLamdaM );
	double H = cosDTM * cosDTM - sinTM * sinTM;
	double L = sinDTM * sinDTM + H * sinDLM * sinDLM;
	double d = 2 * asin( sqrt( L ) );
	double U = 2 * sinTM * sinTM * cosDTM * cosDTM / ( 1 - L );
	double V = 2 * sinDTM * sinDTM * cosTM * cosTM / L;
	double X = U + V;
	double Y = U - V;
	double T = d / sin( d );
	double D = 4 * T * T;
	double E = 2 * cos( d );
	double A = D * E;
	double B = 2 * D;
	double C = T - ( A - E ) / 2;
	double n1 = X * ( A + C * X );
	double n2 = Y * ( B + E * Y );
	double n3 = D * X * Y;
	double d1d = f * ( T * X - Y ) / 4;
	double d2d = f * f / 64 * ( n1 - n2 + n3 );
	double S2 = a * sin( d ) * ( T - d1d + d2d );
	
	return S2;
}

// ************************************************************
//                         地図表示
// ************************************************************

// Google MapをダウンロードしSDカードに保存する
//
// client: インターネット接続クライアント(WiFiClientやTinyGsmClient等）
// mapType: "roadmap" , "satellite", "terrain", "hybrid"のいずれかを指定
// lat, lon: 地図の中心の緯度経度（度）
// zoom: 1:世界 5:大陸 10:都市 15:街路 20:ビル　レベル
// savePath: SDカードに保存する絶対パス。/で始まる。
// apiKey: Google API key
//
// ・注意
//  地図データは320x240ピクセルの場合、20KB程度あり、ダウンロードする際、
//  NTRIP等の他のWifiアクセスをストップしておかないと正常に取得できない。
//
// 戻り値＝保存したバイト数
//         負数：エラー
//
byte mGBuff[10000];
int getGoogleMap( Client *client, const char *mapType, double lat, double lon, int zoom,
				int width, int height, const char *savePath, const char *apiKey )
{
	char *host = "maps.googleapis.com";
	char buff[1024];

	TcpClient *tcpClient = new TcpClient( client );
	if ( ! tcpClient ) return -1;
	
	int nret = tcpClient->connect( host, 80 );
	if ( nret < 0 ) {
		dbgPrintf( "http connect error nret=%d\r\n", nret );
		return -2;
	}
	
	int statusCode;
	snprintf( buff, 256, "/maps/api/staticmap?center=%.6f,%.6f&zoom=%d&size=%dx%d&format=jpg-baseline&maptype=%s&key=%s", lat, lon, zoom, width, height, mapType, apiKey );
	int resBytes = tcpClient->httpGet( buff, buff, 1024, &statusCode );
if ( resBytes > 0 ) dbgPrintf("resBytes=%d  str=%s\r\n", resBytes, buff );
	if ( resBytes < 0 ){
		dbgPrintf( "http get error res=%d\r\n", resBytes );
		return -3;
	}
	if ( statusCode != 200 ){
		dbgPrintf( "http get error status=%d\r\n", statusCode );
		return -4;
	}
	
	char* p = strstr( buff, "Content-Length:" );
	int totalBytes = 0;
	if ( p ){
		nret = sscanf( p, "Content-Length: %d", &totalBytes );
	}
	if ( totalBytes < 1 ) return -5;
	
	File fd = SD.open( savePath, FILE_WRITE );
	if (! fd) return -6;

	int retCode = -8;	// timeout error
	unsigned long msecStart = millis();
	int residue = totalBytes;
	while( millis() - msecStart < 5000 ){
		int bytes = tcpClient->read( (byte*) mGBuff, 10000 );
		if ( bytes > 0 ){
			nret = fd.write( (byte*) mGBuff, bytes );
			if ( nret != bytes ) {
				retCode = -7;
				break;
			}
			residue -= bytes;
			if ( residue == 0 ) {
				retCode = totalBytes;
				break;
			}
		}
		else delay(10);
	}
	fd.close();
	
	return retCode;
}

	
	