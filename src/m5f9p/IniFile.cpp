/*

          ************************************************************
                                INI ファイル
          ************************************************************


---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/



#include <M5Stack.h>

#include "IniFile.h"

IniFile::IniFile()
{
}

int IniFile::open( const char *path )
{
	File iniFile = SD.open( path, FILE_READ );
	if ( ! iniFile ) {
		mBuffer = NULL;
		return -1;
	}
	unsigned long size = iniFile.size();
	mBuffer = (char*) malloc( size );
	if ( ! mBuffer ) return -2;
	
	int nret = 0;
	unsigned long retSize = iniFile.read( (byte*) mBuffer, size );
	if ( retSize != size ) {
		free( mBuffer );
		mBuffer = NULL;
		nret = -3;
	}
	else {
		mFileSize = retSize;
		mPath = path;
	}
	iniFile.close();
	return nret;
}

int IniFile::close()
{
	if ( mBuffer ) free( mBuffer );
	mBuffer = NULL;
}
	

// 
// INIファイルからkeyに設定された値（文字列）を取得する
//
// section: []で囲まれた文字列
// key    : key=value形式でのkey文字列。NULLの場合はsectionのみ検索。
// 
// 戻り値＝指定されたsection,keyが見つかった行の番号（1から始まる）
//         -1：bufferがNULL
//         -2以下：指定されたsection,key文字列が無い
//
int IniFile::readValue( const char *section, const char *key, char *buffer, int buffMaxBytes )
{
	int nret,retCode;
	char buff[256],buff1[256];

	mReadIndex = 0;
	
	if ( buffer ) *buffer = '\0';
	else return -1;
	
	int lineNum = 0;

	// sectionの検索
	int found = 0;
	if ( section && strlen( section ) > 0 ){
		while(1){
			nret = readLine( buff, 255 );
			lineNum++;
			if ( nret < 0 ) break;
			if ( nret == 0 ) continue;

			nret = getToken( buff, '[', ']', buff1, 256 );
			if ( nret <= 0 ) continue;
			if ( strcmp( buff1, section ) == 0 ){
				found = 1;
				break;
			}
		}
		if ( ! found ) { retCode = -3; goto ret; }
	}
	if ( key == NULL || strlen(key) == 0){
		if ( found ) retCode = lineNum;
		else retCode = -4;
		goto ret;
	}
	
	// keyの検索
	found = 0;
	while(1){
		nret = readLine( buff, 256 );
		lineNum++;
		if ( nret < 0 ) break;
		if ( nret == 0 ) continue;
		
		if ( strchr( buff, '[' ) ) break;
		nret = getToken( buff, 0, '=', buff1, 256 );
		if ( nret <= 0 ) continue;
		if ( strcmp( buff1, key ) == 0 ){
			nret = getToken( buff, '=', 0, buffer, buffMaxBytes );
			found = 1;
			break;
		}
	}
	if ( ! found ) retCode = -5;
	else retCode = lineNum;
ret:
	return retCode;
}

double IniFile::readDouble( const char *section, const char *key, double defaultValue )
{
	char buff[256];
	int nret = readValue( section, key, buff, 255 );
	if ( strlen( buff ) == 0  || nret < 0 ) return defaultValue;

	return atof( buff );
}

int IniFile::readInt( const char *section, const char *key, int defaultValue )
{
	char buff[256];
	int nret = readValue( section, key, buff, 255 );
	if ( strlen( buff ) == 0  || nret < 0 ) return defaultValue;

	return atoi( buff );
}

// INIファイルにkey=valueを書き込む
//
int IniFile::writeValue( const char *section, const char *key, char *value )
{
	char buff[256],lineText[256];
	int nret;
	
	if ( strlen(key) + strlen(value) + 3 > 255 ) return -1;
	sprintf( lineText, "%s=%s\r\n", key, value );

	int lineNum = readValue( section, key, buff, 256 );

	// 指定されたsection,keyが存在する場合
	if ( lineNum >= 0 ){
		nret = fileInsertLine( mPath, lineNum, 0, lineText );
		if ( nret < 0 ) return -3;
		else {
			close();
			open( mPath );
			return 0;
		}
	}
	
	// 指定されたsection,keyが存在しない場合
	// sectionの検索
	int pos = 0;
	if ( section == NULL || strlen(section) == 0 ) {
		lineNum = 1;
		pos = -1;
	}
	else {
		lineNum = readValue( section, NULL, buff, 256 );
		pos = 1;
	}
	if ( lineNum > 0 ) {	// sectionが無しまたは存在する場合
		nret = fileInsertLine( mPath, lineNum, pos, lineText );
		if ( nret < 0 ) return -5;
		else {
			close();
			open( mPath );
			return 0;
		}
	}	
	else {	// sectionが存在しない場合
		if ( strlen(section) + 5 > 256 ) return -6;
		sprintf( buff, "[%s]\r\n", section );
		File iniFile = SD.open( mPath, FILE_APPEND );
		if ( ! iniFile ) return -7;
		nret = iniFile.write( (byte *)"\r\n", 2 );	// 最後に改行コードが無い場合のため
		nret = iniFile.write( (byte *)buff, strlen(buff) );	// section
		nret = iniFile.write( (byte *)lineText, strlen(lineText) );		// key
		iniFile.close();
		if ( nret != strlen(lineText) ) return -8;
		close();
		open( mPath );
	}
	
	return 0;
}

// １行読み込む。コメントは削除する。
//
// 
// 戻り値= 格納したバイト数
//        -1: ファイルの終わり
//        -2: バッファが足りない
//
int IniFile::readLine( char *buff, int maxBytes )
{
	int nret = readBytesUntil( '\n', buff, maxBytes );
	if ( nret < 0 ) return nret;
	buff[ nret ] = '\0';

	char* p = buff;
	while( *p ){
		if ( *p == ';' ){
			*p = '\0';
			break;
		}
		p++;
	}
	return strlen( buff );
}

// cはbuffに含まれない
//
// 戻り値= 格納したバイト数
//        -1: ファイルの終わり
//        -2: バッファが足りない
//
int IniFile::readBytesUntil( char c, char *buff, int maxBytes )
{
	if ( mReadIndex >= mFileSize ) return -1;

	int startIndex = mReadIndex;
	while( mReadIndex < mFileSize ){
		if ( c == mBuffer[ mReadIndex++ ]  ) break;
	}
	int numBytes = mReadIndex - startIndex - 1;
	if ( numBytes > maxBytes ) return -2;
	memcpy( buff, mBuffer + startIndex, numBytes );
	return numBytes;
}


bool IniFile::isComment( char *line )
{
	char* p = line;
	while( *p == ' ' ) p++;
	if ( *p == ';' || *p == '#' ) return true;
	else return false;
}

// 
int IniFile::getToken( char *text, char startChar, char endChar, char *buffer, int bufferLength )
{
	char *p = text;
	if ( startChar ){
		p = strchr( p, startChar );
		if ( ! p ) return -1;
		p++;
	}
	while( *p == ' ' ) p++;
	int i = 0;
	int imax = bufferLength - 1;
	while( *p && i < imax && *p != '\r' && *p != '\n' ){
		if ( endChar && *p == endChar ) break;
		buffer[i++] = *p++;
	}
	buffer[i] = '\0';
	p = buffer + i - 1;
	while( p >= buffer && *p == ' ' ) p--;
	*(p+1) = '\0';

	return i;
}

// ファイルに行テキストを挿入する。
//
// path:    ファイルのパス。存在しない場合は作成される。
// lineNum: 挿入する行位置。行番号は１から始まる。
// pos    : 挿入の仕方。-1:行の前 0:置き換え +1:行の後にtextを挿入する。
// text   : 挿入する文字列。改行コードも含む。
//
// 戻り値＝ 0:正常終了
//         負数:エラー
//
int IniFile::fileInsertLine( const char *path, int lineNum, int pos, char *text )
{
	int retCode,nret;
	char *tmpPath = "/tmptmp.tmp";
	char buff[256];
	
	File tmpFile = SD.open( tmpPath, FILE_WRITE );
	if ( ! tmpPath ) return -1;
	
	File file = SD.open( path, FILE_READ );
	if ( ! file ) {	// ファイルが存在しない場合
		file = SD.open( path, FILE_WRITE );
		if ( ! file ) return -2;
		
		nret = file.write( (byte *)text, strlen(text) );
		file.close();
		if ( nret == strlen(text) ) return 0;
		else return -3;
	}

	
	// 挿入位置までコピー
	int numCopyLines = lineNum;
	if ( pos <= 0 ) numCopyLines--;
	for( int i=0; i < numCopyLines; i++){
		nret = file.readBytesUntil( '\n', buff, 254 );
		if ( nret < 0 ) break;
		buff[ nret ] = '\0';
		strcat( buff, "\n" );
		nret = tmpFile.write( (byte *)buff, strlen(buff) );
		if ( nret != strlen(buff) ) { retCode = -3; goto ret; }
	}
	
	// 行テキスト挿入
	nret = tmpFile.write( (byte *)text, strlen(text) );
	if ( nret != strlen(text) ) { retCode = -4; goto ret; }
	
	// 残りのコピー
	if ( pos == 0 ) nret = file.readBytesUntil( '\n', buff, 254 );
	while(1){
		int numBytes = file.readBytes( buff, 256 );
		if ( numBytes < 0 ) break;
		buff[ numBytes ] = '\0';
		nret = tmpFile.write( (byte *)buff, numBytes );
		if ( nret != numBytes ) { retCode = -5; goto ret; }
	}
	
	file.close();
	tmpFile.close();
	
	// 元ファイルの更新
	file = SD.open( path, FILE_WRITE );
	if ( ! file ) { retCode = -6; goto ret; }
	tmpFile = SD.open( tmpPath, FILE_READ );
	if ( ! tmpFile ) { retCode = -7; goto ret; }
	while(1){
		int numBytes = tmpFile.readBytes( buff, 256 );
		if ( numBytes == 0 ) break;
		nret = file.write( (byte *)buff, numBytes );
		if ( nret != numBytes ) { retCode = -8; goto ret; }
	}
	retCode = 0;
	SD.remove( tmpPath );
	
ret: 
	if ( tmpFile ) tmpFile.close();
	if ( file ) file.close();
	
	return retCode;
}