/*

          ************************************************************
                                TCPクライアント
          ************************************************************
          
・Wifiまたは3Gモデムを使用してネットワークに接続し、HTTPやNTRIP等で
　データを取得する。

---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include <base64.h>

#include "TcpClient.h"


TcpClient::TcpClient( Client *client )
{
	mClient = client;
	strcpy( mAgentName, "" );
}

// TCPで接続する
//
// 戻り値＝ 0:正常終了
//         負数:エラー
//
int TcpClient::connect( const char *host, int port )
{
	if ( !mClient ) return -1;
	
	strcpy( mHost, "" );
	int mPort = -1;
	if ( mClient->connect( host, port ) ) {
		snprintf( mHost, 64, "%s", host );
		strncpy( mHost, host, 63 );
		mHost[63] = '\0';
		mPort = port;
		return 0;
	}
	else return -2;
}

int TcpClient::connected()
{
	if ( ! mClient ) return 0;
	return mClient->connected();
}

void TcpClient::stop()
{
	if ( mClient ) mClient->stop();
}

// 読み出し可能なバイト数を取得する
//
int TcpClient::available()
{
	if ( ! mClient ) return 0;
	return mClient->available();
}

int TcpClient::read()
{
	if ( ! mClient ) return -1;
	return mClient->read();
}


// データがあれば読み出す
// 
// 戻り値＝読み出したバイト数
//         負数：エラー
//
int TcpClient::read( byte *buff, int buffSize )
{
	if ( ! mClient ) return -1;
	int n = available();
	if ( ! n ) return 0;
	if ( n > buffSize ) n = buffSize;
	int nret = mClient->read( buff, n );
	return nret;
}

// 改行コード（\r\n or \n)まで読み込む。
//
// 戻り値＝読み込んだバイト数（buffには改行コードは含まれない）
//         -1:タイムアウト
//         -2:接続がされていない
//         -3:以下：エラー
//
int TcpClient::readLine( char *buff, int buffSize, int msecTimeout )
{
	if ( ! mClient ) return -3;
	if ( buffSize <= 0 ) return -4;

	unsigned long msecStart = millis();
	
	char *p = buff;
	int idx = 0;
	char c = 0;
	while(1){
		if ( millis() - msecStart > msecTimeout ) return -1;	// timeout 
		if ( ! mClient ) return -5;
		int n = mClient->available();
		if ( n == 0 ) {
			if ( ! mClient->connected() ) return -2;
			else continue;
		}
		if ( p + n + 1 >= buff + buffSize ) n = buff + buffSize - p - 1;
		if ( n == 0 ) break;
		for( int i=0; i < n; i++ ){
			c = mClient->read();
			if ( c == '\n' ) {
				if ( p != buff && ( *(p-1) == '\r' ) ) p--;
				break;
			}
			*p++ = c;
		}
		if ( c == '\n' ) break;
	}
	*p = '\0';
	return p - buff;
	
}

int TcpClient::write( byte *buff, int numBytes, int msecTimeout )
{
	if ( ! mClient ) return -2;
	if ( numBytes == 0 ) return 0;
	if ( numBytes < 0 ) return -3;

	byte *p = buff;
	unsigned long msecStart = millis();
	while(1){
		if ( millis() - msecStart > msecTimeout ) return -1;	// timeout 
		int n = numBytes - ( p - buff ) ;
		int nret = mClient->write( p, n );
		if ( nret == n ) break;
		if ( nret < 0 ) return -4;
		p += nret;
	}
	return numBytes;
}

// Agent名を設定する
//
// agentName: 32文字以内
// 
// ・現在、NTRIPでのみ有効
//
int TcpClient::setAgentName( const char *agentName )
{
	int n = strlen( agentName );
	if ( n > 31 ) n = 31;
	strncpy( mAgentName, agentName, n );
	mAgentName[ n ] = '\0';

	return 0;
}

// HTTP GET要求を送る
//
// path: コンテンツのドキュメントルートからの絶対パス。先頭に/が必要
//
// 戻り値＝ サーバより返されたレスポンスのバイト数
//         負数：エラー
//
int TcpClient::httpGet( const char *path, char *response, int responseMax, int *statusCode )
{
	char buff[256];
	int nret;

	if ( ! connected() ) return -1;
	sprintf( buff, "GET %s HTTP/1.1\r\n", path );
	nret = write( (byte*) buff, strlen(buff), 1000 );
	if ( nret < 0 ) return -2;

	sprintf( buff, "Host: %s\r\nConnection: close\r\n\r\n", mHost );
	nret = write( (byte*) buff, strlen(buff), 1000 );
	if ( nret < 0 ) return -3;
	
	if ( ! response || responseMax < 1 ) return -4;
	unsigned long msecStart = millis();
	int i = 0;
	bool done = false;
	while(1){
		if ( millis() - msecStart > 5000 ) return -5;	// timeout 
		int n = mClient->available();
		if ( n < 1 ) continue;
		for( int j=0; j < n; j++ ){
			char c = mClient->read();
			response[i++] = c;
			if ( c == '\n' && i >= 4) {
				if ( response[i-4] == '\r' && response[i-3] == '\n' && response[i-2] == '\r' ) {
					done = true;
					break;
				}
			}
			if ( i == responseMax - 2 ){
				done = true;
				break;
			}
		}
		if (done) break;
	}
	response[i] = '\0';
	
	*statusCode = 0;
	char* p = strstr( response, "HTTP/1.1" );
	if ( p ) nret = sscanf( p, "HTTP/1.1 %d", statusCode );

	return i;
}

// NTRIPキャスターにクライアントとして接続する。
//
// host: NTRIPキャスターのホスト名
// port: ポート番号（-1の時は既定値2101を使う）
// mountPoint: マウントポイント。/は付けない
//             ソーステーブルを取得する時は""を指定
// user,password: Basic認証のみ対応。使わない時は""を指定
// 
//
// 戻り値が0または1の時は、read(),readLine()でデータを読み込む事ができる。
//
// 戻り値＝1:ソーステーブルが返される
//         0:補正データが返される
//        -1:ユーザ認証エラー
//        -2以下：その他のエラー
//

int TcpClient::ntripRequest( const char *mountPoint, char *user, char *password )
{
	return ntripRequest( mountPoint, user, password, NULL );

}

int TcpClient::ntripRequest( const char *mountPoint, char *user, char *password, char *gga )
{
	char buff[512];
	const char *p;
	int nret;

	if ( ! connected() ) return -2;	
	if ( mountPoint == NULL || strlen(mountPoint) == 0 ) p = "";
	else p = mountPoint;
	if ( strlen(p) > 100 ) return -3;
	sprintf( buff, "GET /%s HTTP/1.0\r\n", p );
	sprintf( buff + strlen(buff) , "User-Agent: NTRIP %s\r\n", mAgentName );
	if ( user && strlen(user) ){
		String toEncode = String( user ) + String( ":" ) + String( password );
		String encoded = base64::encode( toEncode );
		if ( encoded.length() > 300 ) return -4;
		strcat( buff, "Authorization: Basic " );
		encoded.toCharArray( buff + strlen(buff), 300 );
		strcat( buff, "\r\n"); 
	}
	
	strcat( buff, "Accept: */*\r\n" );
	strcat( buff, "Connection: close\r\n\r\n" );
	nret = write( (byte*) buff, strlen(buff), 1000 );
	if ( nret < 0 ) return -5;

/*	８月初め、ドコモ用に入れたが、10月になってこのGGAを送ると
　　接続が切られてしまう事が起きたので削除。
	if ( gga ){
		nret = write( (byte*) gga, strlen(gga), 1000 );
		if ( nret < 0 ) return -6;
	}
*/
	
	while(1){
		nret = readLine( buff, 256, 10000 );
		if ( nret < 0 ) return -7;
		if ( strstr( buff, "Unauthorized" ) ) return -1;
		if ( strstr( buff, "ICY 200 OK" ) ) return 0;
		if ( strstr( buff, "SOURCETABLE 200 OK" ) ) return 1;
	}
	
	return -8;
}


int TcpClient::ntripRequest( const char *mountPoint )
{
	return ntripRequest( mountPoint, "", "" );
}


// NTRIPキャスターにサーバとして接続する。
//
// host: NTRIPキャスターのホスト名
// port: ポート番号（-1の時は既定値2101を使う）
// mountPoint: マウントポイント。/は付けない
// password: パスワード（必須）64文字以内
// 
//
// 戻り値が0時は、write()でデータを送る事ができる。
//
// 戻り値＝0:データ転送可能
//        -1: 接続エラー（指定されたhost,portでは接続できない、またはキャスタが応答しない）
//        -2: キャスターとのデータ転送エラー
//        -3: パスワードエラー
//        -4：キャスターからのその他のエラー応答
//        -5以下：その他のエラー
//

int TcpClient::ntripServerConnect( const char *host, int port, 
								const char *mountPoint, const char *password )
{
	char buff[512];

	if ( ! mClient ) return -5;
	if ( ! host || !strlen(host)) return -6;
	if ( ! mountPoint || strlen( mountPoint ) < 1 || strlen( mountPoint ) > 100 ) return -7;
	if ( ! password || strlen( password ) < 1 || strlen( password ) > 64 ) return -8;
	if ( port <= 0 ) port = 2101;
	
	int nret = connect( host, port );
	if ( nret < 0 ) return -1;
	if ( ! connected() ) return -1;
		
	sprintf( buff, "SOURCE %s /%s\r\n", password, mountPoint );
	sprintf( buff + strlen(buff), "Source-Agent: NTRIP %s\r\n", mAgentName );
	strcat( buff, "\r\n" );
	nret = write( (byte*) buff, strlen(buff), 1000 );
	if ( nret < 0 ) return -2;

	while(1){
		nret = readLine( buff, 256, 10000 );
		if ( nret < 0 ) return -2;
		if ( strstr( buff, "ICY 200 OK" ) ) return 0;
		if ( strstr( buff, "Bad Password" ) ) return -3;
		if ( strstr( buff, "ERROR" ) ) return -4;
	}
	
	return -9;
}

