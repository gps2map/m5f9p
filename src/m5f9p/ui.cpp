/*

          ************************************************************
                           ユーザインターフェース
          ************************************************************


---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/



#include <M5Stack.h>
#include "ui.h"


#define TONE_MAX 1000
byte mToneCurve[TONE_MAX];
int mSampleRate;
int mBeepFreq = 1000;

int mLcdWidth = 320;
int mLcdHeight = 240;
int mLcdRotation = 1;	// 画面の向き （ mLcdRotation x 90度。通常は90度）
int mTextSize = 2;		// 1-5   文字のピクセル数は 8 x mTextSize

// ************************************************************
//                        スピーカー
// ************************************************************


// sineカーブをmToneCurveにセットする
// 
// freauency: Hz   duration: msec
//
int setSineCurve( int frequency, int duration )
{
	int usecSamplePitch = duration * 1000 / TONE_MAX; //usec
	if ( usecSamplePitch == 0 ) return -1;
	mSampleRate = 1000000 / usecSamplePitch;
	if ( mSampleRate == 0 ) return -2;
	float radDelta = 2 * 3.14 * frequency / mSampleRate;
	float rad = 0;

	for( int i=0; i < TONE_MAX; i++ ){
		mToneCurve[i] = 127 + 126 * sin( rad );
		rad += radDelta;
	}
	mToneCurve[ TONE_MAX - 1 ] = 0;		// 0 : array end
	
	return 0;
}

// 音を鳴らす
//
// volume: 0 - 11
//
void playSound( int volume )
{
	M5.Speaker.setVolume( volume );
	M5.Speaker.playMusic( mToneCurve, mSampleRate );
	speakerOff();
}

void speakerOff()
{
	dacWrite(25, 0);	// speaker off
}


// ************************************************************
//                           表示
// ************************************************************

int lcdInit( int setRotation )
{
	return lcdInit( setRotation, "" );
}

// 画面の初期化
//
// setRotation:  0=回転しない  1=180度回転  -1=UIで選択
// message : 画面に表示するメッセージ
//
// 戻り値＝ 0:回転無し 1:180度回転
//
int lcdInit( int setRotation, char *message )
{
	int nret = 0;
	lcdClear();
	lcdTextColor( WHITE );
	lcdTextSize( 2 );
	if ( strlen( message ) ) {
		lcdDispText( 2, "%s", message );
		if ( ! setRotation ) {
			lcdDispText( 10, ">>> Press any button" );
			waitButton();
		}
	}
	
	if ( setRotation ){
		if ( setRotation < 0 ){
			while(1){
				lcdDispButtonText( 2, WHITE, "Rotate", "Ok", "Rotate" );
				lcdDispText( 7, ">>> Is display orientation OK ?" );
				nret = waitButton( 1, 1, 1, 1, 0, 1 );
				if ( nret ){
					lcdClear();
					lcdRotate( 180 );
				}
				else break;
			}
		}
		else {
			if ( setRotation == 1 ){
				lcdRotate( 180 );
				nret = 1;
			}
		}
	}
	lcdClear();
	
	return nret;
}

void lcdDispText( int lineNum, char* format, ... )
{
	char buff[256];
	va_list args;
	va_start( args, format );
	vsprintf( buff, format, args );
	va_end( args );
	if ( mLcdRotation == 3 ) lineNum += 1;
	if ( lineNum >= 0 ) M5.Lcd.setCursor( 0, lineNum * mTextSize * 8 );
	M5.Lcd.printf( buff );
}

//
// greenText = GREEN
// format Text = WHITE
//
void lcdDispText2( int lineNum, char* greenText, char* format, ... )
{
	char buff[256];
	va_list args;
	va_start( args, format );
	vsprintf( buff, format, args );
	va_end( args );
	if ( mLcdRotation == 3 ) lineNum += 1;
	if ( lineNum >= 0 ) M5.Lcd.setCursor( 0, lineNum * mTextSize * 8 );

	lcdTextColor( GREEN );
	M5.Lcd.printf( greenText );
	lcdTextColor( WHITE );
	M5.Lcd.printf( buff );
}

// LCDの文字の大きさを設定する。
// 
// textSize: 1～3　　
// 文字のピクセル数は　textSize * 8 
//
void lcdTextSize( int textSize )
{
	mTextSize = textSize;
	M5.Lcd.setTextSize( textSize );
}

void lcdClear( int color )
{
	M5.Lcd.clear( color );
}

void lcdClear()
{
	lcdClear( BLACK );
}


int lcdGetColor16( int color24 )
{
	int red = (color24 >> 16) & 0xff;
	int green = (color24 >> 8) & 0xff;
	int blue =  color24 & 0xff;
	int color16 = ( (red >> 3) << 11 ) | ( ( green >> 2 ) << 5 ) | ( blue >> 3 );
	return color16;
}

// color: 16bit color
//
void lcdTextColor( int color )
{
	M5.Lcd.setTextColor( (uint16_t) color, BLACK );
}

// color: 24bit color
//
void lcdTextColor24( int color )
{
	lcdTextColor( (color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff );
}

// red:上位5bits  green:6bits  blue:5bitsのみ有効

void lcdTextColor( int red, int green, int blue )
{
	M5.Lcd.setTextColor( ((red>>3)<<11) | ((green>>2)<<5) | (blue>>3) , BLACK );
}

void lcdClearLine( int lineNumber )
{
	char buff[41];
	memset( buff, ' ', 40 );
	buff[40] = '\0';
	if ( mLcdRotation == 3 ) lineNumber += 2;
	M5.Lcd.setCursor( 0, lineNumber * mTextSize * 8 );
	M5.Lcd.printf( buff );
}

// SDカードに保存されたJPEG画像を表示する
//
void lcdDispJpeg( const char *path )
{
	M5.Lcd.drawJpgFile( SD, path );

}

// カーソルを表示する
//
// color: 16bit
//
void lcdDispCursor( int type, int x, int y, int color16 )
{
	int width2 = mLcdWidth / 2;
	int height2 = mLcdHeight / 2;
	
	M5.Lcd.drawLine( x - width2, y, x + width2, y, color16 );
	M5.Lcd.drawLine( x , y - height2, x, y + height2, color16 );
}

// 画面を回転する
//
// degree : 現在の向きから回転する角度。90度単位
//
// 戻り値＝回転後の角度。90度が通常の向き（ボタンが下の状態）
//
int lcdRotate( int degree )
{
	int newAngle = ( mLcdRotation * 90 + degree ) % 360;
	mLcdRotation = newAngle / 90;
	M5.Lcd.setRotation( mLcdRotation );
	return newAngle;
}


// ボタン用メニュを表示する
//
void lcdDispButtonText( int textSize, int color, char *textA, char *textB, char *textC )
{
	lcdDispButtonText( textSize, color, textA, textB, textC, true );
}

void lcdDispButtonText( int textSize, int color, char *textA, char *textB, char *textC, bool clear )

{
	float mmWidth = 42.0;
	float mmLeftStart = 5.0;
	float mmButtonWidth = 7.5;
	float mmPitch = 12;
	float mm2Pixel = mLcdWidth / mmWidth;
	int y = mLcdHeight - textSize * 8;
	if ( mLcdRotation == 3 ) y = 0;
	int pixCenterButton1 = ( mmLeftStart + mmButtonWidth / 2.0 ) * mm2Pixel;
	int pixPitch = mmPitch * mm2Pixel;
	float fontWidth = 8 * textSize * 0.8;

	lcdTextColor( color );
		
	M5.Lcd.setTextSize(textSize);

	int xA,xB,xC;
	if ( mLcdRotation == 1 ) {
		xA = pixCenterButton1 - (strlen(textA) * fontWidth) / 2;
		xB = pixCenterButton1 + pixPitch - (strlen(textB) * fontWidth) / 2;
		xC = pixCenterButton1 + pixPitch * 2 - (strlen(textC) * fontWidth) / 2;
	}
	else {
		int delta = 5;
		xA = pixCenterButton1 + pixPitch * 2 - (strlen(textA) * fontWidth) / 2;
		xB = pixCenterButton1 + pixPitch - (strlen(textB) * fontWidth) / 2 + delta ;
		xC = pixCenterButton1 - (strlen(textC) * fontWidth) / 2 + delta;
	}
	
	// クリア
	if ( clear ){
		char buff[41];
		memset( buff, ' ', 40 );
		buff[40] = '\0';
		M5.Lcd.setCursor( 0, y );
		M5.Lcd.printf( buff );
	}
	
	// 表示
	M5.Lcd.setCursor( xA, y );
	M5.Lcd.printf( textA );

	M5.Lcd.setCursor( xB, y );
	M5.Lcd.printf( textB );

	M5.Lcd.setCursor( xC, y );
	M5.Lcd.printf( textC );

	M5.Lcd.setTextSize(mTextSize);
}

void lcdDispAndWaitButton( int lineNum, char* format, ... )
{
	char buff[256];
	lcdClear();

	va_list args;
	va_start( args, format );
	vsprintf( buff, format, args );
	if ( lineNum > 0 ) M5.Lcd.setCursor( 0, lineNum * mTextSize * 8 );
	M5.Lcd.printf( buff );
	va_end( args );

	lcdTextColor( WHITE );
	lcdDispText( 10, ">>> Press any key" );
	waitButton();
}


// ボタンが押されるまで待つ
//
// useA,useB,useC: ボタンを使用する時1,押されても無視する時0
// numA,numB,numC: ボタンが押された時に返す値（整数）
//
// 戻り値＝押されたボタン番号  1:A  2:B  3:Cボタン
//
int waitButton( int useA, int useB, int useC,  int numA, int numB, int numC )
{
	int num;
	while(1){	
		delay(50);
		M5.update();
		if ( useA &&  M5.BtnA.wasReleased() ) num = numA;
		else if ( useB && M5.BtnB.wasReleased() ) num = numB;
		else if ( useC && M5.BtnC.wasReleased() ) num = numC;
		else continue;
		break;
	}
	return num;
}

int waitButton()
{
	waitButton( 1, 1, 1, 0, 0, 0 );
}

int buttonPressed()
{
	int num = 0;
	M5.update();
	if ( M5.BtnA.wasReleased() ) num = 1;
	else if ( M5.BtnB.wasReleased() ) num = 2;
	else if ( M5.BtnC.wasReleased() ) num = 3;

	return num;
}

