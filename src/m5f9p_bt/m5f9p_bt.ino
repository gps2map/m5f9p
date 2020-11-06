/*

          ************************************************************
              u-blox ZED-F9Pモジュール on M5Stack  制御用プログラム Bluetooth対応版
          ************************************************************
          
          [機能制限]
          １．基準局及びMoving Baseとしては動作しない
          ２．モジュールは１台のみ
          ３．マイクロSDカードは使えない

---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------

*/



#include <M5Stack.h>
#include <WiFi.h>
#include <Preferences.h>
//#include <WiFiAP.h>
//#include <Wire.h>
//#include <WebServer.h>
//#include <SD.h>
#include <time.h>
#include <esp_wifi.h>

#include "BluetoothSerial.h"

#include "m5f9p.h"
#include "RingBuff.h"
//#include "i2c.h"
#include "gis.h"
#include "gps.h"
//#include "IniFile.h"
#include "TcpClient.h"
#include "ModemFep01.h"
#include "ui.h"

//#define TINY_GSM_MODEM_UBLOX
//#include <TinyGsmClient.h>


// ************************************************************
//                        モジュール変数
// ************************************************************

byte mVersionMajor = 1;
byte mVersionMinor = 0;
byte mVersionPatch = 0;

int mCpuFreqMHz;

#define CANCEL 0
#define OK 1
#define NEXT 2

#define OFF 0
#define ON 1


struct stButton {
	bool pressedA;
	bool pressedB;
	bool pressedC;
	bool pressedLongA;
	bool pressedLongB;
	bool pressedLongC;
} mButton;

// Debug device
#define DBG_NONE 0
#define DBG_SERIAL 1
#define DBG_FILE 2

int mDebugDevice = DBG_SERIAL;


#define BT_ENABLE 1
//#define BT_ENABLE 0

char *mBluetoothName = "M5F9P";
BluetoothSerial mBluetooth;

//時刻
time_t mUnixTime;		// 現在の時刻	

struct stDateTime {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int msec;
};

unsigned long mStartMillis;	//測位開始時刻

//

#define USE_WIFI      1
#define USE_MODEM_3G  2
#define USE_MODEM_920 3

int mNetAccess = 0;

char* mSsid;
char* mPassword;
IPAddress mWifiLocalIp;

// スイッチサイエンス製３Gモデム用
//TinyGsm mModem(Serial2); /* 3G board modem */
//TinyGsmClient mGsmClient(mModem);
//int mModemBaudrate = 115200;
//IPAddress mModemLocalIp;

// 双葉電子製920MHzモデム用
ModemFep01 mFep01;
int m920TotalBytes;		// 920MHz modemにより取得したバイト数
int m920LastBytes;
int m920RecvCount;
int m920SendCount;

// パス
String mRootDir = "/m5f9p";
String mGpsLogDir = mRootDir + "/gpslog";
String mHtmlDir = mRootDir + "/html";

// Preferences用
Preferences mPreferences;

// INIファイル用
//String mIniPath = mRootDir + "/m5f9p.ini";
//IniFile *mIniFile;

#define WIFI_MAX 3
struct stWifi {
	char ssid[33];
	char password[65];
	byte ip[4];
	byte dns[4];
} mWifiList[ WIFI_MAX ];
int mNumWifi;

#define BASE_TYPE_NONE   0
#define BASE_TYPE_TCP    1
#define BASE_TYPE_3G     2
#define BASE_TYPE_920    3
#define BASE_TYPE_UART   4

#define PROTO_NONE       0
#define PROTO_NTRIP      1
#define PROTO_NTRIP_GGA  2

//#define BASE_SRC_MAX 10
#define BASE_SRC_MAX 3
struct stBaseSource {
	bool valid;
	int type;
	int protocol;
	char address[64];
	int port;
	char mountPoint[32];
	char user[32];
	char password[32];
} mBaseSrcList[ BASE_SRC_MAX ];
int mNumBaseSrc;
struct stBaseSource mBaseSrc;		// 基準局データ受信先

// データ保存、配信用
#define SAVE_BUFF_MAX 256
char mSaveBuff[ SAVE_BUFF_MAX ];
int mSaveBuffBytes = 0;
//char mSaveFileName[RECEIVER_MAX][64];

int mWifiConnected;
//char mSoftApSsid[16] = "m5f9p";
//char mSoftApPassword[16] = "m5f9p123";
//IPAddress mSoftApIp( 192,168,5,1 );

//WebServer* mWebServer;
//int mHttpPort = 80;

// 基準局データ受信用
bool mNtripSelected = false;

TcpClient *mBaseRecvClient;	// 基準局データを受信するためのクライアント
TcpClient *mBaseSendClient;	// 基準局データを送信するためのクライアント
boolean mBaseRecvReady;
boolean mBaseSendReady;

WiFiClient mWifiClient;

#define BASE_RECV_BUFF_MAX 256
char mBaseRecvBuff[ BASE_RECV_BUFF_MAX ];
int mBaseRecvIdx;

int mBaseRecvCount;
int mNripLastCount;
unsigned long mBaseRecvLastMillis;
unsigned long mNripLastCountMillis;

int mReceiverType[RECEIVER_MAX];	// 0:none 1:F9P/serial 2:F9P/I2C

// UART用
char mUartBuff[ UART_BUFF_MAX ];
int mUartWriteIndex;
int mUartReadIndex;
int mUartSaveIndex;


// 接続されているGPS受信機の台数
int mNumReceivers;

//
unsigned long mGpsLastMillis;
int mSolutionRate;
int mGpsUartBaudrate = 115200;		// 460800,230400,115200,57600,38400,19200,9600
									// BT対応版は115200でないとUBXデコードでchecksum errorが起きる

struct stGpsData mGpsData[ RECEIVER_MAX ];	// 取得された位置データ
struct stGpsData mGpsDataLast;				// メイン受信機の直前のデータ

#define SATELLITES_MAX  60
struct stGpsSat mGpsSatellites[ SATELLITES_MAX ];
int mNumSatellites;

struct stUbxStatus mUbxStatus[ RECEIVER_MAX ];


// 時刻
int mPinGpsPPS = 36;
unsigned long mGpsPPSMicros;

// 
int mSoundOn = 1;

// ３ピンJST-PH コネクタ 主にUART用　TTLレベルのGPIOとしても利用可能
unsigned long mBitTime;
int mPinPhOut = 12;		// 出力
int mPinPhIn = 35;		// 入力

int mPinGpsReset = 15;

int mOpeMode;
char *mOpeModeStr[4] = { "None", "Rover", "Base", "Moving Base" };

// color
int mColor[] = { 0xffdfdf, 0xffffc0, 0x80ffc0, 0xfff7b7,
				 0xff6f6f, 0xffff70, 0x80ff40, 0xffab57 };
int mDisplayType = 0;		// M5Stackのロットにより色が違う機種がある。その補正の番号。

// test
int mPinTest = 16;

// 関数
void btEventCallback( esp_spp_cb_event_t event, esp_spp_cb_param_t *param );

// ************************************************************
//                         Arduino 初期化
// ************************************************************
//

// 関数宣言
int IRAM_ATTR getPvtTime( struct stDateTime *time,  unsigned long *pvtMicros );
time_t IRAM_ATTR dateTime2UnixTime( struct stDateTime *dateTime );

void setup() {
	int nret;
	char buff[256];
	struct stGpsData gpsData;
	
	// put your setup code here, to run once:
	M5.begin( true, false );
dbgPrintf("\r\n");
	// serial port for debug
	Serial.begin(115200);
	//Serial.setDebugOutput(true);

dbgHeapSize("pos1");

	mCpuFreqMHz = ESP.getCpuFreqMHz();
	int taskStackSize = 2048;
	
//	setSineCurve( 1000, 100 );
	speakerOff();

	// 画面の初期化
	lcdInit( true );
	lcdDispText( 3, "Initializing");

	// test
	pinMode( mPinTest, OUTPUT);
	
	// gps enable
	pinMode( mPinGpsReset, OUTPUT);
	digitalWrite( mPinGpsReset, HIGH);

disableCore1WDT();
disableCore0WDT();


	// INIファイル
	nret = readIniFile();
	
    dbgPrintf("CPU clock=%d\r\n",mCpuFreqMHz);
    
	// GPSデータ受信スレッド（core 0)
	mOpeMode = 0;
	xTaskCreatePinnedToCore(taskGetSensorData, "taskGetSensorData", 2048, NULL, 1, NULL, 0);
	delay(10);

	// ネット接続方法の選択。Wifi or Modem ?
	// 動作モード選択の後の方が自然だが、GPS受信機の衛星捕捉の時間を取るために
	// 先に行う。
	// 
	lcdDispText( 8, "Soft AP start" );
	nret = selectNetConnection();
	lcdClear();
	
	lcdDispText( 3, "Starting sensor thread\r\n" );

	// WEBサーバ
//	nret = webServerInit();

	// GPS受信機の初期化、受信機の数え上げ
j1:	gpsInit();	

	// gpsInit()ではSerial1のボーレートの自動設定機能を使っている。
	// その影響かSerial2のボーレートが変わってしまうようだ。
	// そのため、下記設定が必要。
	if ( mNetAccess == USE_MODEM_920 ){		
		Serial2.begin( mFep01.baudrate(), SERIAL_8N1, 16, 17 );
	}
	else if ( mNetAccess == USE_MODEM_3G ){		
//		Serial2.begin( mModemBaudrate, SERIAL_8N1, 16, 17 );
	}

	// 動作モード選択
	lcdClear();
	mOpeMode = MODE_ROVER;
	
	// GPS受信テスト及び日時取得
	lcdClear();
	int count = 0;
	double lat = 100;
	double lon = 400;

	lcdDispButtonText( 2, WHITE, "", "Cancel", "" );
	lcdDispText( 2, ">>> Testing the ZED-F9P receiver" );
	bool canceled = false;
	while(1){
		for( int i=0; i < 50; i++ ){
			M5.update();
			if ( M5.BtnB.wasReleased() ){
				canceled = true;
				break;
			}
			delay( 20 );
		}
		if ( canceled ) break;
		lcdDispText( 5, "count = %d", ++count );
		if ( count > 60 ){
			lcdTextColor( RED );
			lcdDispText( 7, " >>> Check the GNSS antenna." );
			lcdTextColor( WHITE );
			lcdDispButtonText( 2, WHITE, "Retry", "Exit", "", true );
			bool exit = waitButton( ON, ON, OFF, false, true, 0 );
			if ( exit ) break;
			lcdClear();
			count = 0;
			lcdDispButtonText( 2, WHITE, "", "Cancel", "", true );
			lcdDispText( 2, ">>> Testing the ZED-F9P receiver" );
			continue;
		}
		nret = gpsGetPosition( &gpsData, 1000 );
		if ( nret < 0 ){
			dbgPrintf( "gpsGetPosition error nret = %d\r\n", nret );
			continue;
		}
		if ( nret == 0 ){
			if ( gpsData.quality == 0 ){
				continue;
			}
			lcdClear();
			lcdTextColor( GREEN );
			lcdDispText( 3, "> ZED-F9P test Ok. " );
			if ( gpsData.quality > 0 ){
				int lineNum = 5;
				lcdDispText( lineNum++, "%d-%02d-%02d %02d:%02d:%02d\r\n", 
							gpsData.year, gpsData.month, gpsData.day,
							gpsData.hour, gpsData.minute, gpsData.second );
				lat = gpsData.lat;
				lon = gpsData.lon;
				lcdDispText( lineNum++, "LAT=%.8lf\r\n", gpsData.lat);
				lcdDispText( lineNum++, "LON=%.8lf\r\n", gpsData.lon);
				lcdDispText( lineNum++, "ALT=%.3lf\r\n", gpsData.height);
				lineNum += 2;
			}
			lcdTextColor( WHITE );
			lcdDispText( 10, ">>> Press any button" );
			waitButton();
			break;
		}
	}

	// 動作モード毎の初期化
	lcdClear();
	mNtripSelected = false;
	switch( mOpeMode ){
		case MODE_ROVER:
			if ( mNetAccess == USE_MODEM_920 ){	// 920MHzモデム
				nret = mFep01.setAddress( 2 );	// Rover address = 2
				if ( nret < 0 ){
					lcdDispAndWaitButton( 3, "> Modem 920MHz set address error nret=%d\r\n", nret );
				}
			}
			else {
				// 基準局データ取得先の選択と接続。TCP
				mBaseRecvReady = false;
				while(1){
					nret = baseSrcSelect( lat, lon ); 
					if ( nret == 0 ) break;		// cancel
					if ( mBaseSrc.type == BASE_TYPE_TCP ){
						nret = connectBaseSource();	// 無手順もしくはNTRIP経由
						if ( nret == 0 ){
							mBaseRecvReady = true;
							break;
						}
					}
					else if ( mBaseSrc.type == BASE_TYPE_UART ){
						/*
						lcdClear();
						lcdDispText( 3, "Base station: JST-PH Connector" );
						lcdDispAndWaitButton( 4, "  baudrate: %d", mPhUartBaudrate );
						attachInterrupt( digitalPinToInterrupt(mPinPhIn), phUartInIsr, FALLING);
  						mBaseRecvReady = true;
						*/
						break;
					}
				}
			}

			break;
	}
dbgHeapSize("Bt in");

	// Bluetooth
	if ( BT_ENABLE ){
		if ( ! mBluetooth.begin( mBluetoothName ) ) dbgPrintf("Bluetooth init failed\r\n");
		else {
			esp_err_t espret = mBluetooth.register_callback( btEventCallback );

		}
	}
dbgHeapSize("Bt out");
	
	// Server
	/*
	mWifiServer = new WiFiServer( mServerPort );
	if ( ! mWifiServer  ){
		lcdDispAndWaitButton( 3, "Can't use wifi server." );
	}
	else {
		mWifiServer->begin();
		for ( int i=0; i < SERVER_CLIENT_MAX; i++ ) mServerClient[i] = NULL;
		xTaskCreatePinnedToCore(taskWifiServer, "taskWifiServer", taskStackSize, NULL, 1, NULL, 0);
	}
	*/
	
	
	//
//	mFileSaved = 0;
//	mFileSaving = 0;
	mBaseRecvCount = 0;
	mNripLastCount = 0;
	mNripLastCountMillis = millis();
	mBaseRecvIdx = 0;
	mBaseSendReady = false;
	mSolutionRate = 1;	// number of solution per second

	lcdClear();
	
	// 移動局タスクスタート
	if ( true ){
		// 移動局メインタスク（core 1）
		xTaskCreatePinnedToCore( taskRover, "taskRover", 4096, NULL, 1, NULL, 1 );

		// 基準局データ受信スタート（core 0）
		if ( mNetAccess == USE_MODEM_920 ){	// 920MHzモデム経由
			xTaskCreatePinnedToCore(taskModem920Xfer, "taskModem920Xfer", taskStackSize, NULL, 1, NULL, 0);
		}
		else if ( mBaseSrc.type == BASE_TYPE_TCP || mBaseSrc.type == BASE_TYPE_UART ){
			if ( mBaseRecvReady ){
				xTaskCreatePinnedToCore(taskBaseRecv, "taskBaseRecv", 3000, NULL, 1, NULL, 0);
			}
		}
	}
	
	// ボタン監視スレッドスタート
	xTaskCreatePinnedToCore(taskButton, "taskButton", 1000, NULL, 1, NULL, 1);

dbgHeapSize("setup end");
	dbgPrintf("setup() exit\r\n");
	lcdClear();
	mStartMillis = millis();
}

void btEventCallback( esp_spp_cb_event_t event, esp_spp_cb_param_t *param )
{
	char *message = "";
	switch( event ){
		case ESP_SPP_CLOSE_EVT:
			message = "closed";
			 break;

		case ESP_SPP_SRV_OPEN_EVT: message = "opened"; break;
	}
	if ( event != ESP_SPP_WRITE_EVT && event != ESP_SPP_CONG_EVT ){
		dbgPrintf("bt callback event=%d %s\r\n", event, message );
	}
}

// ************************************************************
//                   Arduino ループ（コア1で実行）
// ************************************************************
//

#define PAGE_MAIN 0
#define PAGE_MAP 1
#define PAGE_INFO 2

int mLcdPage;		// 画面に表示するページ番号
int mLcdPageMax = 3;	// 総ページ数

#define MAP_ROAD 0
#define MAP_SAT 1
#define MAP_MAX 2
int mMapType;
unsigned long msecMapDisp;		// 地図が表示された時刻

void loop() 
{
	int nret;
	
	loopRover();

	/*
	if ( mWebServer ) {		// ブラウザからの要求に対しては早く応答しないと正常に表示されない
		mWebServer->handleClient();
		WiFiClient client = mWebServer->client();
		if ( client.connected() ){
			unsigned long msecStart = millis();
			while( millis() - msecStart < 200 ){
				mWebServer->handleClient();
			}
		}
	}
	*/

	if ( buttonCPressed() ){
		nextPage();
		buttonPressedReset();
	}
}

void nextPage()
{
	mLcdPage++;
	if ( mLcdPage == mLcdPageMax ) mLcdPage = 0;
	lcdClear();
}

void loopRover()
{

	switch( mLcdPage ){
		case PAGE_MAIN: 
			dispRoverMainPage();
			break;
		case PAGE_MAP:
//			if ( mFileSaving ) mLcdPage++;	// データ保存時は地図表示できない
//			else dispMapMain();
			break;
		case PAGE_INFO:
			dispInfo();
			break;
	}
	
}

void dispRoverMainPage() 
{
	int nret,numOutBytes,fifoTx,fifoRx;
	char buff[256];
	int f9pInterface;
	double x,y,z;
	

	lcdDispButtonText( 2, WHITE, "", "Rate", "NextPage", false );
	int lineStart = 0;
	int numInfoLines = 4;
	bool updated = false;
	int i = 0;
	while(1){
		if ( i == mNumReceivers ) break;
		int n = i + 1;
		if ( mGpsData[i].ubxDone && 
				( mOpeMode == MODE_ROVER || ( mOpeMode == MODE_MOVING_BASE && i == 0) ) ){
			updated = true;
			int fix = 0;
			if (mGpsData[i].quality == 4 ) fix = 2;
			else if (mGpsData[i].quality == 5 ) fix = 1;
			lineStart = i * 4;
			if ( mNumReceivers > 1 ){
				lcdTextColor24( mColor[ mDisplayType * 4 + i ] );
			}
			lineStart = i * 4;
			lcdDispText( lineStart++, "LAT%d=%.8lf\r\n", n, mGpsData[i].lat);
			lcdDispText( lineStart++, "LON%d=%.8lf\r\n", n, mGpsData[i].lon);
			lcdDispText( lineStart++, "ALT%d=%.3lf\r\n", n, mGpsData[i].height);
			lcdDispText( lineStart++, "FIX%d=%d  SPS%d=%d \r\n", n, fix, n, mSolutionRate);
			lcdTextColor( WHITE );

			if ( fix != 2 ){
				if ( mSoundOn ){
//					playSound(1);
				}
			}
//			mGpsData[i].ubxDone = false;
		}
		
		i++;
	}
	lcdTextColor24( mColor[ mDisplayType * 4 + 3 ] );
	lcdTextColor( WHITE );

	// NTRIP bytes per second
	if ( mBaseRecvReady ){
		int n = mBaseRecvCount - mNripLastCount;
		unsigned long milis = millis();
		unsigned long spanMillis = milis - mNripLastCountMillis;
		if ( spanMillis > 900 ){
			int bps = n * 8 * 1000 / spanMillis;
			mNripLastCountMillis = milis;
			mNripLastCount = mBaseRecvCount;
	
			lcdDispText( mNumReceivers * 4 + 1, "NTRIP=%d bytes\r\n", n );
		}
	}
	else if ( mNetAccess == USE_MODEM_920 ){
		int bytes = m920TotalBytes - m920LastBytes;
		m920LastBytes = m920TotalBytes;
		lcdDispText( mNumReceivers * 4 + 2, "920MHz Modem : %4dKB\r\n", m920TotalBytes / 1000 );
	}
	
			

	// ボタンが押された時の処理
	
	if ( buttonAPressed() ){
	}

	if ( buttonBPressed() ){
		if ( mSolutionRate == 1 ) mSolutionRate = 5;
		else {
			mSolutionRate += 5;
			if ( mSolutionRate > 20 ) mSolutionRate = 1;
		}
		for( int i=0; i < RECEIVER_MAX; i++ ){
			if ( mReceiverType[i] ){
				nret = gpsSetMeasurementRate( mReceiverType[i], 1000 / mSolutionRate );
				if ( nret < 0 ){
					dbgPrintf( "gpsSetMesurementRate error\r\n" );
				}
			}
		}
	}
}

// 情報表示
//
void dispInfo()
{
	String text;
	lcdDispButtonText( 2, WHITE, "", "", "NextPage", false );
	int lineNum = 1;
	lcdDispText( lineNum++, "****** dispInfo *******" );
	lcdDispText( lineNum++, "Version: %d.%d.%d", mVersionMajor, mVersionMinor, mVersionPatch );
	lcdDispText( lineNum++, "OpeMode: %s", mOpeModeStr[ mOpeMode ] );

	byte mac[6];
	esp_read_mac( mac, ESP_MAC_WIFI_STA);
	lcdDispText( lineNum++, "MAC addr:%02X:%02X:%02X:%02X:%02X:%02X", 
							mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	if ( mNetAccess == USE_WIFI )
		lcdDispText( lineNum++, "IP addr:%u.%u.%u.%u", 
					mWifiLocalIp[0], mWifiLocalIp[1],mWifiLocalIp[2], mWifiLocalIp[3] );
	else if ( mNetAccess == USE_MODEM_3G )
		lcdDispText( lineNum++, "IP addr:%u.%u.%u.%u", 
					mWifiLocalIp[0], mWifiLocalIp[1],mWifiLocalIp[2], mWifiLocalIp[3] );
	
//	lcdDispText( lineNum++, "AP IP address:%u.%u.%u.%u", 
//					mSoftApIp[0], mSoftApIp[1], mSoftApIp[2], mSoftApIp[3] );
//	lcdDispText( lineNum++, "Server port:%d", mServerPort );
//	lcdDispText( lineNum++, "AP ssid:%s", mSoftApSsid );
//	lcdDispText( lineNum++, "AP passwd:%s", mSoftApPassword );
	lcdDispText( lineNum++, "Heap: %d KB", esp_get_free_heap_size() / 1000 );

}

// 移動局としてのメインスレッド
//
void taskRover(void* param)
{
	int nret,numOutBytes,fifoTx,fifoRx;
	char buff[256];
	int f9pInterface;
	double x,y,z;

j1:	

	int msecGpsCheck = 20;
	if (millis() - mGpsLastMillis > msecGpsCheck){
		mGpsLastMillis = millis();
		bool updated = false;
		int i = 0;
		while(1){
			if ( i == mNumReceivers ) break;
			
			f9pInterface = mReceiverType[i];
			if ( f9pInterface == 0 ) {
				i++;
				continue;
			}
			
			// UBXメッセージの取得
			if ( mOpeMode == MODE_MOVING_BASE && i == 0){
				// already decoded
			}
			else{
				nret = ubxDecode( f9pInterface, &mUbxStatus[i] );
				if ( nret < 0 ){
					if ( nret == -2 || nret == -3 ) {
						dbgPrintf( "ubx check sum error  nret=%d\r\n", nret );
					}
					else dbgPrintf("ubx decode error nret=%d\r\n", nret);
					i++;
					continue;
				}
			}
			int msgClass = mUbxStatus[i].msgClass;
			int msgId = mUbxStatus[i].msgId;
//if ( msgClass == 0x02 && msgId == 0x15 ) dbgPrintf("RAWX status=%d\r\n",mUbxStatus[i].statusNum);
			if ( mUbxStatus[i].statusNum != 10 ) {
				i++;
				continue;
			}

			// UBX NAV-SATのデコード
			if ( msgClass == 0x01 && msgId == 0x35  ){ // NAV-SAT
				mNumSatellites = ubxDecodeNavSat( &mUbxStatus[i], mGpsSatellites, SATELLITES_MAX );
				mUbxStatus[i].statusNum = 0;
			}

			// UBX NAV-DOPのデコード	NAV-SATの後に送られてくる
			if ( msgClass == 0x01 && msgId == 0x04  ){ // NAV-DOP
				nret = ubxDecodeNavDop( &mUbxStatus[i], &mGpsData[i] );
				mUbxStatus[i].statusNum = 0;
				if ( mNumSatellites > 0 ){
					if ( BT_ENABLE  && mBluetooth.hasClient() ){
						nret = sendNmeaSatData( mGpsSatellites, mNumSatellites, &mGpsData[i], 1 );
					}
				}
			}


			// UBX PVT(Position Velocity Time)のデコード
			if ( msgClass == 0x01 && msgId == 0x07  ){ // NAV-PVT
				
				if ( i == 0 ) {
					memcpy( &mGpsDataLast, &mGpsData[0], sizeof(mGpsDataLast) );
				}
				
				nret = ubxDecodeNavPvt( &mUbxStatus[i], &mGpsData[i] );
				mUbxStatus[i].statusNum = 0;
				
				// 高精度座標値の取得
				unsigned long msecStart = millis();
				while( millis() - msecStart < 40 ){	// 高精度座標はPVTの直後に来る
					nret = ubxDecode( f9pInterface, &mUbxStatus[i] );
					msgClass = mUbxStatus[i].msgClass;
					msgId = mUbxStatus[i].msgId;
					if ( mUbxStatus[i].statusNum == 10 ){
						if ( msgClass == 0x01 && msgId == 0x14 ){ // NAV-HPPOSLLH
							nret = ubxDecodeHPPOSLLH( &mUbxStatus[i], &mGpsData[i] );
							mUbxStatus[i].statusNum = 0;
							 break;
						}
					}
				}
			
				// 受信データの配信、表示
				if ( mGpsData[i].ubxDone ){
					updated = true;
					mGpsData[i].receiverNum = i;
					
					// NMEAデータの作成
					numOutBytes = setNmeaData(i);

					// NMEAデータの配信
					//BT
					if ( BT_ENABLE  ){
						if ( numOutBytes > 0 && mBluetooth.hasClient() ){
							nret = mBluetooth.write( (byte*)mSaveBuff, numOutBytes);
							if ( nret != numOutBytes ){
							dbgPrintf("BT write error nret=%d byteToWrite=%d\r\n", nret, numOutBytes );
							}
						}
					}
				}
			}
			mUbxStatus[i].statusNum = 0;
		}

		mWifiConnected =  (WiFi.status() == WL_CONNECTED) ;
		
	}
	else vTaskDelay(5);
	goto j1;
}

// ボタン監視スレッド
//
void taskButton(void* param)
{
	int nret;
	while(1){
		nret = buttonPressed( &M5.BtnA );
		if ( nret == 1 ) mButton.pressedA = true;
		else if ( nret == 2 ) mButton.pressedLongA = true;
		

		nret = buttonPressed( &M5.BtnB );
		if ( nret == 1 ) mButton.pressedB = true;
		else if ( nret == 2 ) mButton.pressedLongB = true;

		nret = buttonPressed( &M5.BtnC );
		if ( nret == 1 ) mButton.pressedC = true;
		else if ( nret == 2 ) mButton.pressedLongC = true;
	}
	vTaskDelay(10);
}


// ボタンが押されたかどうか判定する
//
// 戻り値＝ 0: 押されていない
//          1: クリックされた
//          2: 長押しされた
//
int buttonPressed( Button *button )
{
	if ( button->read() ){
		vTaskDelay( 20 );
		if ( button->read() ) {
			vTaskDelay( 100 );
			if ( ! button->read() ) return 1;
			vTaskDelay( 400 );
			if ( button->read() ){
				while( button->read() ) vTaskDelay(50);	// 離されるまで待つ
				return 2;
			}
			else return 1;
			
		}
	}
}

// ボタンが押されたかどうか取得する
// 
// 戻り値＝ 0:押されていない
//          1:クリック
//          2:長押し
//
int buttonAPressed()
{
	int ret = 0;
	if ( mButton.pressedLongA ){
		ret = 2;
		mButton.pressedLongA = false;
	}
	else if ( mButton.pressedA ){
		ret = 1;
		mButton.pressedA = false;
	}
	return ret;
}

int buttonBPressed()
{
	int ret = 0;
	if ( mButton.pressedLongB ){
		ret = 2;
		mButton.pressedLongB = false;
	}
	else if ( mButton.pressedB ){
		ret = 1;
		mButton.pressedB = false;
	}
	return ret;
}

int buttonCPressed()
{
	int ret = 0;
	if ( mButton.pressedLongC ){
		ret = 2;
		mButton.pressedLongC = false;
	}
	else if ( mButton.pressedC ){
		ret = 1;
		mButton.pressedC = false;
	}
	return ret;
}

void buttonPressedReset()
{
	memset( (void*)&mButton, 0, sizeof( mButton ) );
}

// ************************************************************
//                         タスク（コア0で実行）
// ************************************************************


// TCPまたはUARTにより、基準局データを受信するスレッド
//
void taskBaseRecv(void* param)
{
	int nret;
	unsigned long msecLastTime = 0;
	vTaskDelay(200);
	while(1)
	{
		vTaskDelay(1);
				
		if (mBaseRecvReady){
			int numRecvBytes = 0;
			if ( mBaseSrc.type == BASE_TYPE_TCP ) {
				numRecvBytes = mBaseRecvClient->available();
				int max = 8;
				if ( numRecvBytes > max ) numRecvBytes = max;
				for( int i=0; i < numRecvBytes; i++ ){
					int ch = mBaseRecvClient->read();
					if ( ch < 0 ) break;
					mBaseRecvBuff[ mBaseRecvIdx++ ] = (char) ch;
					if ( mBaseRecvIdx == BASE_RECV_BUFF_MAX || ch == '\n' ){
						nret = gpsWrite( IF_UART, mBaseRecvBuff, mBaseRecvIdx );
						if ( nret > 0 ) mBaseRecvCount += nret;
						mBaseRecvIdx = 0;
					}
					mBaseRecvLastMillis = millis();
				}
			}
			else continue;
		}
		if ( mBaseSrc.protocol == PROTO_NTRIP_GGA ){	// 1秒毎にGGAをキャスターに送る
			if ( millis() - msecLastTime  >= 1000 ){
				msecLastTime = millis();
				char* p = strstr( mSaveBuff, "$GPGGA" );
				if ( p ) {
					nret = mBaseRecvClient->write( (byte *) p, strlen(p), 5000 );
					if ( nret < 0 ){
						dbgPrintf("GGA send error nret = %d\r\n",nret );
					}
				}
			}
		}

		if ( mBaseRecvClient ){
			if ( millis() - mBaseRecvLastMillis > 5000 ){	// 5sec no source data

				if ( mNetAccess == USE_WIFI ){
					if ( WiFi.status() != WL_CONNECTED) {
						WiFi.begin( mSsid, mPassword );
						vTaskDelay(200);
					}
					if ( WiFi.status() != WL_CONNECTED) continue;
				}

				nret = mBaseRecvClient->connect( mBaseSrc.address, mBaseSrc.port );
				if ( nret == 0 ){
					if ( mBaseSrc.protocol == PROTO_NTRIP || mBaseSrc.protocol == PROTO_NTRIP_GGA ) {
						dbgPrintf("ntrip reconnecting\r\n");
						nret = mBaseRecvClient->ntripRequest( mBaseSrc.mountPoint, 
													mBaseSrc.user, mBaseSrc.password );
						mBaseRecvReady = ( nret == 0 );
						if( ! mBaseRecvReady ){
							dbgPrintf("ntrip reconnct failed\r\n");
						}
					}
					else mBaseRecvReady = true;
					if ( mBaseRecvReady ){
							dbgPrintf("source reconnected\r\n");
					}
				}
				else {
					dbgPrintf("reconnect failed : %s\r\n", mBaseSrc.address);
				}
				mBaseRecvLastMillis = millis();
			}
		}
		
	}
}

// 920MHzモデムにより基準局データを受信するスレッド
//
void taskModem920Xfer(void* param)
{
	int nret,senderAddress;
	char buff[256];
	
	while(1){
		int receivedBytes = mFep01.receive( &senderAddress, buff, 256, 100 );
		if ( receivedBytes ){
			nret = gpsWrite( IF_UART, buff, receivedBytes );
			m920TotalBytes += receivedBytes;
		}
		vTaskDelay(10);
	}
}

// GPS受信機からデータを取得するスレッド
//
// １．メイン受信機からのデータ
//     UART（Serial1)からデータを取得し、
//     移動局の場合は位置データをUART用バッファに格納する。
//
void taskGetSensorData(void* param)
{
	int nret;
	while(1)
	{
		byte *savePointer;
		int numSavedBytes;
		int pvtReceived;

		vTaskDelay(1);

		// メイン受信機からのデータをUARTで取得する
		int numBytesToRead = Serial1.available();
		if ( numBytesToRead > 0 ){
			if (true) {	// 移動局の場合
				unsigned long msecStart = millis();
				int msecTimeout = 100;
				while ( numBytesToRead ){
					savePointer = (byte*) mUartBuff + mUartWriteIndex;
					int buffResidue = UART_BUFF_MAX - mUartWriteIndex;
					if ( buffResidue < numBytesToRead ) numBytesToRead = buffResidue;
					numSavedBytes = Serial1.readBytes( savePointer, numBytesToRead );
					if ( numSavedBytes == numBytesToRead ){
//gpsCheckRtcm( savePointer, numSavedBytes );
						mUartWriteIndex += numSavedBytes;
						if ( mUartWriteIndex == UART_BUFF_MAX ) mUartWriteIndex = 0;
						else if ( mUartWriteIndex > UART_BUFF_MAX ){
							dbgPrintf( "taskGetSensorData() UART error\r\n" );
							mUartWriteIndex = 0;
						}
					}
					else{
						dbgPrintf(" taskGetSensorData() read error toRead=%d saved=%d\r\n", numBytesToRead, numSavedBytes);
					}
					numBytesToRead -= numSavedBytes;
					vTaskDelay(5);
					numBytesToRead = Serial1.available();
				}
			}
		}
	}
}


void ringBuffCopy( byte* source, int numBytes, byte* dest, int *destIndex, int destMaxBytes )
{
	int residue = numBytes;
	byte* pRead = source;
	while( residue > 0 ){
		int n = residue;
		if ( *destIndex + n > destMaxBytes ) n = destMaxBytes - *destIndex;
		memcpy( dest + *destIndex, pRead, n );
		*destIndex += n;
		pRead += n;
		if ( *destIndex >= destMaxBytes ) *destIndex = 0;
		residue -= n;
	}
}

// ************************************************************
//                         初期化関連関数
// ************************************************************

// データの受配信を行うためのネットワーク選択
//
int selectNetConnection()
{
	int nret;
	// 出力制御
	esp_wifi_set_max_tx_power(40);	// 40: 10dBm

	// soft AP は常時使える
	/*
	nret = WiFi.softAP( mSoftApSsid, mSoftApPassword );
	if ( ! nret ){
		dbgPrintf("softAP() failed\r\n");
		lcdDispText( 7, "Can't use soft AP." );
	}
	else dbgPrintf("softAP() ok.\r\n");
	mSoftApIp = WiFi.softAPIP();
	*/
	
	// ネットワーク選択開始
	lcdClear();
	lcdDispButtonText( 2, WHITE, "Wifi", "Modem", "No" );
	lcdDispText( 3, ">>> Do you use Wifi or Modem ?" );
	nret = waitButton( 1, 1, 1, 1, 2, 0 );
	lcdClear();
	if ( nret == 2 ){
		lcdDispButtonText( 2, WHITE, "3G", "920MHz", "Cancel" );
		lcdDispText( 3, ">>> Do you use 3G(Soracom) or 920MHz ?" );
		nret = waitButton( 1, 1, 1, 2, 3, 0 );
		lcdClear();
	}
	if ( nret == 1 ){  // Wifi接続
		WiFi.mode( WIFI_AP_STA );
		if ( mNumWifi == 0 ) {
			lcdDispText( 3, "No wifi AP data in SDcard." );
			lcdDispText( 4, "Can't use wifi." );
		}
		else {
			lcdDispText( 3, "Wifi connecting" );
			mWifiConnected = 0;
			mWifiConnected = wifiConnect();
			if ( mWifiConnected ) {
				lcdDispText( 5, "> Wifi connected" );
				dbgPrintf("WiFi connected\r\n");
				Serial.print("local IP address: ");
				mWifiLocalIp = IPAddress( WiFi.localIP() );
				dbgPrintf( (mWifiLocalIp.toString()).c_str() );
				mNetAccess = USE_WIFI;
			}
			else {
				lcdDispText( 5, "> Wifi not connected" );
				dbgPrintf("WiFi not connected !!!\r\n");
			}
		}
		
		
	}
	else if ( nret == 2 ){ // Modem 3G(Soracom)
	/*
		while(1){
			nret = modemInit();
			delay(1000);	// wait threadModemInit() done

			if ( nret == 0 ){
				lcdDispText( 8, "> Modem 3G connected" );
				mNetAccess = USE_MODEM_3G;
				mModemLocalIp = IPAddress( mModem.localIP() );
				break;
			}
			else {
				lcdClear();
				if ( nret == -1 ) lcdDispText( 3, "> Time out error (%d)", nret );
				else  lcdDispText( 3, "> Connect error(%d)", nret );
				lcdDispText( 4, "> Modem not connected", nret );
				lcdDispText( 6, ">>> Do you retry ?" );
				lcdDispButtonText( 2, WHITE, "exit", "retry", "" );
				nret = waitButton( 1, 1, 0, 1, 2, 0 );
				if ( nret == 1 ) {
					lcdDispText( 4,  "> Modem not connected (%d)", nret );
					break;
				}
				lcdClear();
			}
		}
	*/
	}
	else if ( nret == 3 ){ // Modem 920MHz FEP01
		lcdDispText( 3, "Modem 920MHz initializing" );
		int baudrate = mFep01.init( 0 );
		if ( baudrate > 0 ){
			lcdDispText( 5, "> Modem 920MHz connected" );
			lcdDispText( 6, "> Baudrate : %d", baudrate );
			
			lcdDispText( 8, ">>> Change baudrate ?" );
			lcdDispButtonText( 2, WHITE, "19200", "38400", "No" );
			nret = waitButton( 1, 1, 1, 1, 2, 0 );
			lcdClear();
			int newBaudrate = 19200;
			if ( nret ) {
				if ( nret == 2 ) newBaudrate = 38400;
				nret = mFep01.setBaudrate( newBaudrate );
				if ( nret < 0 ){
					lcdDispText( 3, "> Can't change baudrate to %d", newBaudrate );
				}
				else{
					lcdDispText( 3, "> New baudrate : %d", newBaudrate );
				}
			}
			lcdDispText( 5, "> Modem 920MHz connected" );
			mNetAccess = USE_MODEM_920;
		}
		else {
			lcdDispText( 5, "> Modem 920MHz not connected (%d)", baudrate );
		}
	}
	else return 0;
		
	lcdDispText( 10, ">>> Press any button" );
	waitButton();
	return 0;
}

int readIniFile()
{
	int nret;
	char buff[256];
	
	lcdClear();
	lcdDispText( 3, "Now receiving setting data" );
	lcdDispText( 6, ">>> Press any button to continue" );

	mPreferences.begin( "m5f9p" );
	
	int i = 0;
	while(1){	
		M5.update();
		if ( M5.BtnA.wasReleased() || M5.BtnB.wasReleased() || M5.BtnC.wasReleased() )  break;
		if ( Serial.available() ){
			char c = Serial.read();
Serial.print(c);
			if ( c == 0x08 ){	// BS
				i--;
				if ( i < 0 ) i =0;
			}
			else if ( c >= ' ' && c < 0x7f ){
				buff[i++] = c;
			}
			
			if ( c == '\n' || i > 250 ){
				buff[i] = '\0';
				getSetValue( buff );
				i = 0;
			}
		}
		delay(10);
	}
	

	// Wifi
	mNumWifi = 0;
	memset( mWifiList, 0, sizeof( mWifiList ) );

	size_t numChars = mPreferences.getString( "wifi/ssid", buff, 32 );
	if ( numChars > 0 ){
		strcpy( mWifiList[ mNumWifi ].ssid, buff );
		numChars = mPreferences.getString( "wifi/password", buff, 64 );
		if ( numChars > 0 ){
			strcpy( mWifiList[ mNumWifi ].password, buff );
			mNumWifi++;
		}
	}
	
	// Base source 基準局データ取得先
	memset( mBaseSrcList, 0, sizeof( mBaseSrcList ) );
	mNumBaseSrc = 0;

	numChars = mPreferences.getString( "source/addr", buff, 64 );
	if ( numChars > 0 ){
		strcpy( mBaseSrcList[ mNumBaseSrc ].address, buff );
		numChars = mPreferences.getString( "source/mount", buff, 32 );
		if ( numChars > 0 ){
			strcpy( mBaseSrcList[ mNumBaseSrc ].mountPoint, buff );
			mBaseSrcList[ mNumBaseSrc ].type = BASE_TYPE_TCP;
			mBaseSrcList[ mNumBaseSrc ].protocol = PROTO_NTRIP;
			mBaseSrcList[ mNumBaseSrc ].port = 2101;
			mNumBaseSrc++;
		}
	}

	mPreferences.end();
	
	return 0;
}

int getSetValue( char* buff )
{
	char key[17];
	char value[65];
	
	int n = strlen( buff );
	int i = 0;
	char c;
	char* p = removeSpace( buff );
	if ( strlen( p ) < 3 ) return 0;
	if ( strstr( p, "set" ) != p ) return 0;
	p += 3;
	if ( *p++ != ' ' ) return 0;
	p = removeSpace( p );
	char* p2 = strchr( p, '=' );
	if ( p2 == NULL ) return 0;
	*p2 = '\0';
	p = removeSpace( p );
	if ( strlen( p ) > 16 ) *( p + 16 ) = '\0';
	strcpy( key, p );
	p2 = removeSpace( p2 + 1 );
	if ( strlen( p2 ) > 64 ) *( p + 64 ) = '\0';
	strcpy( value, p2 );

	size_t nret = mPreferences.putString( key, value );
	dbgPrintf("Preferences |%s| = |%s|  nret=%d\r\n", key, value, nret );
	return nret;
}

char* removeSpace( char* buff )
{
	char c;
	char* p = buff;
	while( c = *p ){
		if ( c != ' ' ) break;
		p++;
	}
	char* p2 = buff + strlen( buff ) - 1;
	if ( *p2 == '\r' ) *p2-- = '\0';
	while( *p2  == ' ' ){
		*p2-- = '\0';
		if ( p2 == p ) break;
	}
	return p;
}

// Wifiのアクセスポイントを選択した後、接続する
//
// 戻り値＝ 1: 接続済
//          0: 未接続
//
int wifiConnect()
{
	char buff[256];
	int nret,color;
	
	lcdClear();
	lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
	lcdDispText( 3, ">>> Do you use Wifi ?" );
	int useWifi = waitButton( 1, 1, 0, YES, NO, 0 );
	lcdClear();
	if ( ! useWifi  ) return 0;
	
j1:	lcdClear();
	lcdDispButtonText( 2, WHITE, "Next", "Ok", "Cancel" );
	lcdDispText( 1, ">>> Select Wifi AP" );
	int lineOffset = 3;
	for( int i=0; i < mNumWifi; i++ ){
		sprintf( buff, "%d %s", i+1, mWifiList[i].ssid );
		if ( i == 0 ) color = GREEN;
		else color = WHITE;
		lcdTextColor( color );
		lcdDispText( i + lineOffset, buff );
	}
	int idx = 0;
	int command = 0;
	while(1){
		command = waitButton( 1, 1, 1, 1, 2, 3 );
		if ( command == 1 ){	// next item
			if ( mNumWifi == 1 ) continue;
			sprintf( buff, "%d %s", idx+1, mWifiList[idx].ssid );
			lcdTextColor( WHITE );
			lcdDispText( idx + lineOffset, buff );

			idx++;
			if ( idx == mNumWifi ) idx = 0;
			sprintf( buff, "%d %s", idx+1, mWifiList[idx].ssid );
			lcdTextColor( GREEN );
			lcdDispText( idx + lineOffset, buff );
			continue;
		}
		else {
			break;
		}
	}
	lcdClear();
	lcdTextColor( WHITE );

	int wifiConnected = 0;
	unsigned long msecStart = millis();
	unsigned long msecLastTime = millis();
	int count = 0;
	if ( command == 2 ){	// Ok
		char* ssid = mWifiList[idx].ssid;
		char* password = mWifiList[idx].password;
		byte* dns = mWifiList[idx].dns;
		if ( mWifiList[idx].ip[0] > 0 ){
			byte* p = mWifiList[idx].ip;
			IPAddress ip( p );
			IPAddress gateway( p[0], p[1], p[2], 1 );
			IPAddress subnet( 255, 255, 255, 0 );
			IPAddress dns( mWifiList[idx].dns );
			
			WiFi.config( ip, gateway, subnet, dns );
		}
		WiFi.begin( ssid, password );
		lcdDispText( 3, "Connecting to %s", ssid );
		while (1) {
			if ( WiFi.status() == WL_CONNECTED) {
				wifiConnected = 1;
				mSsid = ssid;
				mPassword = password;
				break;
			}
			if ( millis() - msecLastTime > 1000 ){
				msecLastTime = millis();
				lcdDispText( 5, "count=%d", ++count);
			}
			delay(100);
			Serial.print(".");
			if ( millis() - msecStart > 10000 ) {
				lcdClear();
				lcdTextColor( WHITE );
				lcdDispText( 3, "Continue connecting ?" );
				lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
				int continu = waitButton( 1, 1, 0, YES, NO, 0 );
				if ( ! continu  ) {
					goto j1;
				}
				else {
					lcdClear();
					msecStart = millis();
					lcdDispText( 3, "Connecting to %s", ssid );
				}
			}
		}
	}
	else if ( command == 3 ){	// Cancel
		return false;
	}
	lcdTextColor( WHITE );
	lcdClear();
	
	return wifiConnected;
}


//  3Gモデムの初期化
//
//  戻り値＝ 0:正常終了
//          -1：タイムアウト
//          -2: IPアドレスが取得できない
//          -3: インターネットにアクセスできない
//
//  (注）
//  ネットワークに接続する際や接続後のデータ取得の際、ウォッチドグタイマーに関するエラーが
//  起きる場合がある。この関数を呼び出す前に disableCore0WDT();disableCore1WDT();
//  を実行する。
//  当初、接続ルーチンをコア0で実行していたが、どうしてもWDTのエラーとなる。
//  接続ルーチンをコア１にして、表示ルーチンをコア０で行うようにしたらWDTのエラーは
//  出なくなった。
// 
//
/*
int mModemStatus;
bool mModemTimeout;
int modemInit()
{
	lcdDispText( 2, "Modem connecting" );
	mModemStatus = 0;
	mModemTimeout = false;
	
	// ステータス表示スレッド
	xTaskCreatePinnedToCore( threadModemInit, "Task0_modem", 4096, NULL, 1, NULL, 0);

	int lineNum = 7;
	
	Serial2.begin( mModemBaudrate, SERIAL_8N1, 16, 17);
	mModem.restart();
	mModemStatus = 1;
	vTaskDelay(1);

	for( int i=0; i < 3; i++ ){
		mModem.sendAT(GF("+IPR=460800"));
		mModemBaudrate = 460800;
		vTaskDelay(200);
		Serial2.begin(mModemBaudrate, SERIAL_8N1, 16, 17);
		vTaskDelay(100);
		if ( mModem.testAT(1000) ) {
			dbgPrintf( "Modem 3G baudrate changed to %d\r\n", mModemBaudrate );
			mModemStatus = 2;
			break;
		}
		vTaskDelay(1);
	}
	
	String modemInfo = mModem.getModemInfo();
	dbgPrintf( "%s\r\n", modemInfo.c_str() );
	mModemStatus = 3;
	vTaskDelay(1);
	
	while ( ! mModem.waitForNetwork() ){
		if ( mModemTimeout ) return -1;
		vTaskDelay(1);
	}

	mModemStatus = 4;
	vTaskDelay(1);

	mModem.gprsConnect( "soracom.io", "sora", "sora" );
	while ( ! mModem.isNetworkConnected() ) {
		if ( mModemTimeout ) return -1;
		vTaskDelay(1);
	}

	mModemStatus = 5;
	vTaskDelay(1);

	IPAddress ipaddr = mModem.localIP();
	dbgPrintf( "mModem.localIP() = %s\r\n", ipaddr.toString().c_str() );
	
	if ( ipaddr[3] == 0 ) return -2;
	else {
		//test
int nret=1;
//		int nret = mGsmClient.connect("www.sony.co.jp", 80);
//		dbgPrintf("3G modem inet test nret=%d\r\n", nret );
//		mGsmClient.stop();
		if ( nret == 1 ) return 0;
	}
	
	return -3;
}

void threadModemInit(void* param)
{
	lcdDispText( 2, "Modem connecting" );
	mModemStatus = 0;

	unsigned long msecStart = millis();
	unsigned long msecLastTime = millis();
	int msecTimeout = 60 * 1000;
	int lastStatus = -1;
	
	int count = 0;
	while(1){
		if ( millis() - msecStart > msecTimeout ){
			mModemTimeout = true;
			break;
		}
		if ( millis() - msecLastTime > 1000 ){
			msecLastTime = millis();
			lcdDispText( 4, "count=%d", count++ );
		}
		if ( mModemStatus != lastStatus ){
			int lineNum = 6;
			lcdClearLine( lineNum );
			switch( mModemStatus ){
				case 0: lcdDispText( lineNum, "Modem initializing" ); break;
				case 1: lcdDispText( lineNum, "Modem initialized" ); break;
				case 2: lcdDispText( lineNum, "Baudrate changed" ); break;
				case 3: lcdDispText( lineNum, "Modem info got" ); break;
				case 4: lcdDispText( lineNum, "Network connected" ); break;
				case 5: lcdDispText( lineNum, "Soracom connected" ); break;
			}
			lastStatus = mModemStatus;
		}
		if ( mModemStatus == 5 ) break;
		vTaskDelay(1);
	}
	vTaskDelete( NULL );
}

*/



// Ntrip caster/serverを選択する
//
// 戻り値＝ 1: 選択済
//          0: 未選択
//
//int ntripSelect( int opeMode, double lat, double lon )
int baseSrcSelect( double lat, double lon )
{
	char buff[256];
	int nret,color;
	
	lcdClear();
	lcdDispText( 3, ">>> Do you connect to base station ?" );
	lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
	int yes = waitButton( 1, 1, 0, YES, NO, 0 );
	lcdClear();
	if ( ! yes  ) return 0;

	dbgPrintf( "mNetAccess = %d\r\n", mNetAccess );
	if ( mNetAccess == USE_WIFI ) mBaseRecvClient = new TcpClient( &mWifiClient );
	else if ( mNetAccess == USE_MODEM_3G ) {
//		mBaseRecvClient = new TcpClient( &mGsmClient );
	}
	if ( mNumBaseSrc ) nret = baseSrcSelect_iniFile();
	if ( nret == 1 ) return 1;
/*
	lcdDispText( 3, ">>> Do you use rtk2go.com ?" );
	lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
	nret = waitButton( 1, 1, 0, YES, NO, 0 );
	if ( nret == NO  ) return 0;
	
	lcdClear();
	lcdDispText( 3, "Connecting to rtk2go.com" );
	
	nret = ntripSelect_caster( "rtk2go.com", 2101, lat, lon );
	lcdClear();
	if ( nret < 0 ){
		lcdDispText( 3, "Ntrip not selected : nret = %d", nret );
		waitButton();
		return 0;
	}
*/
	return 1;
}

/*
#define MAX_MOUNT_POINTS 100
struct stMountPoint {
	char mountpoint[32];
	char city[10];
	char format[10];
	char nav[32];
	float lat;
	float lon;
	float distance;
} mMountPoints[ MAX_MOUNT_POINTS ];
int mNumMountPoints;
*/

// Ntripキャスターにアクセスし、マウントポイントを選択する
// 
// 戻り値＝ 0:正常終了
//         負数：エラー
//
/*
int ntripSelect_caster( char* server, int port, double lat, double lon )
{
	char buff[300],*p,item[32];
	int nret,color,command;
	
	// mountpointの取得
	int mNumMountPoint = 0;
	if ( mBaseRecvClient == NULL ) return -1;
	nret =  mBaseRecvClient->connect( server, port );
	if ( nret < 0 ) {
		lcdClear();
		lcdDispText( 3, "> Can't connect to %s\r\n", server );
		lcdDispAndWaitButton( 5, " port: %d\r\n", port );
		return -1;
	}

	if( mBaseRecvClient->ntripRequest( "/" ) ){
		delay(5);
		unsigned long msecStart = millis();
		while(1){
			while( ! mBaseRecvClient->available() ){
				if ( millis() - msecStart > 3000 ) break;
				delay(100);
			}
			mBaseRecvClient->readLine( buff, sizeof(buff), 3000 );
			if ( strstr( buff, "ENDSOURCETABLE" ) ) break;

			// STR チェック
			nret = ntripGetItem( buff, 1, item, 32 );
			if ( nret < 0 || strcmp( item, "STR" ) != 0 ) continue;
			
			// 日本に限定
			nret = ntripGetItem( buff, 9, item, 32 );
			if ( nret < 0 || strcmp( item, "JPN" ) != 0 ) continue;
			
			// 無料のみ
			nret = ntripGetItem( buff, 17, item, 32 );
			if ( nret < 0 || strcmp( item, "N" ) != 0 ) continue;
			
			// mountpoint等の取得
			stMountPoint *mp = & mMountPoints[ mNumMountPoints ];
			nret = ntripGetItem( buff, 2, mp->mountpoint, 32 );
			if ( nret < 0 ) continue;
			nret = ntripGetItem( buff, 3, mp->city, 10 );
			nret = ntripGetItem( buff, 4, mp->format, 10 );
			nret = ntripGetItem( buff, 7, mp->nav, 32 );
			nret = ntripGetItem( buff, 10, item, 32 );
			if ( nret < 0 ) continue;
			mp->lat = atof( item );
			nret = ntripGetItem( buff, 11, item, 32 );
			if ( nret < 0 ) continue;
			mp->lon = atof( item );
			if ( lat <= 90 && mp->lat > 0 ){
				double dist = thomasDistance( mp->lat, mp->lon, lat, lon );
				mp->distance = (float)( dist / 1000.0 );
			}
			else mp->distance = -1;
			mNumMountPoints++;
			if ( mNumMountPoints == MAX_MOUNT_POINTS ) break;
		}
	}
	else return -2;
	if ( mNumMountPoints == 0 ) return -3;
	
	// mountpointの表示と選択
	int lineOffset = 3;

	int numPageLines = 10;
	int startIdx = 0;
	int idx = 0;
	int distance;
	int selected = 0;
	while(1){
		lcdClear();
		lcdDispButtonText( 2, WHITE, "Page", "Next", "Ok" );
		lcdDispText( 1, ">>> Select mount point" );
	
		if ( startIdx >= mNumMountPoints ) {
			startIdx = 0;
			idx = 0;
		}
		for( int i = 0; i < numPageLines; i++ ){
			sprintf( buff, "%d %s", startIdx + i + 1 , mMountPoints[ startIdx + i ].mountpoint );
			distance = (int) (mMountPoints[ startIdx + i ].distance );
			if ( distance > 0 ) sprintf( buff + strlen(buff), " (%dkm)", distance );
			if ( i == idx ) color = GREEN;
			else color = WHITE;
			lcdTextColor( color );
			lcdDispText( i + lineOffset, buff );
		}
	
		int allIdx = startIdx + idx;
		command = 0;
		while(1){
			command = waitButton( 1, 1, 1, 1, 2, 3 );
			if ( command == 1 ){	// next page
				idx = 0;
				startIdx += numPageLines;
				break;
			}
			if ( command == 2 ){	// next item
				if ( mNumMountPoints == 1 ) continue;
				sprintf( buff, "%d %s", allIdx + 1, mMountPoints[allIdx].mountpoint );
				distance = (int) ( mMountPoints[allIdx].distance );
				if ( distance > 0 ) sprintf( buff + strlen(buff), " (%dkm)", distance );
				lcdTextColor( WHITE );
				lcdDispText( idx + lineOffset, buff );

				idx++;
				if ( idx == numPageLines ) {
					idx = 0;
					break;
				}
				allIdx = startIdx + idx;
				sprintf( buff, "%d %s", allIdx + 1, mMountPoints[allIdx].mountpoint );
				distance = (int) ( mMountPoints[allIdx].distance );
				if ( distance > 0 ) sprintf( buff + strlen(buff), " (%dkm)", distance );
				lcdTextColor( GREEN );
				lcdDispText( idx + lineOffset, buff );
				continue;
			}
			else {	// ok
				lcdClear();
				lcdTextColor( WHITE );

				stMountPoint *mp = & mMountPoints[ allIdx ];
				int lineNum = 4;
				lcdDispText( lineNum++, "Mountpoint: %s", mp->mountpoint );
				lcdDispText( lineNum++, "City: %s", mp->city );
				lcdDispText( lineNum++, "Format: %s", mp->format );
				lcdDispText( lineNum++, "Gnss: %s", mp->nav );
				lcdDispText( lineNum++, "Lat: %.2f", mp->lat );
				lcdDispText( lineNum++, "Lon: %.2f", mp->lon );
				if ( mp->distance > 0 ) lcdDispText( lineNum++, "Distance: %dkm", (int)mp->distance );
				
				lcdDispButtonText( 2, WHITE, "Ok", "Back", "Cancel" );
				lcdDispText( 1, ">>> Do you select this mount point ?" );

				command = waitButton( 1, 1, 1, 1, 2, 3 );
				
				if ( command == 1 ){	// ok
					memset( &mBaseSrc, 0, sizeof(mBaseSrc) );
					strncpy( mBaseSrc.address, server, 63 );
					mBaseSrc.port = port;
					strncpy( mBaseSrc.mountPoint, mMountPoints[allIdx].mountpoint, 31 );
					mBaseSrc.protocol = PROTO_NTRIP;
					mBaseSrc.valid = 1;
					selected = 1;
					break;
				}
				else if ( command == 2 ){	// back
					break;
				}
				else {	// cancel
					return -4;
				}
			}
		}
		if ( selected ) break;
	}
	lcdClear();
	lcdTextColor( WHITE );

}

// NTRIP Caster Tableの項目を取得する。
//
// itemNum: 項目番号。1から始まる。
//
// 戻り値＝ 0:正常終了
//          負数：見つからない
//
int ntripGetItem( char* buff, int itemNum, char* item, int itemSize )
{
	char *p1 = buff;
	char *p2 = 0;
	int i = 0;
	while(1){
		p2 =  strchr( p1, ';' );
		if ( ! p2 || i == itemNum - 1 ) break;
		p1 = p2 + 1;
		i++;
	}
	if ( i != itemNum - 1 ) return -1;

	int n;
	if ( p2 ) n = p2 - p1;
	else n = strlen( p1 );
	if ( n > itemSize - 1 ) n = itemSize - 1;
	strncpy( item, p1, n );
	item[n] = '\0';
	return 0;
}
*/


// Iniファイルに記載された基準局データ取得先を選択する
//
// 戻り値＝ 1: 選択済（mBaseSrcにパラメータ設定済）
//          0: 選択無し
//
//int ntripSelect_iniFile()
int baseSrcSelect_iniFile()
{
	char buff[256];
	int color;
	
	lcdClear();
	lcdDispButtonText( 2, WHITE, "Next", "Ok", "Cancel" );
	lcdDispText( 1, ">>> Select base station" );
	int lineOffset = 3;
	for( int i=0; i < mNumBaseSrc; i++ ){
		//if ( i == 0 ) sprintf( buff, "0 UART(PH connector)" );
		//else sprintf( buff, "%d %s %s", i, mBaseSrcList[i].address, mBaseSrcList[i].mountPoint );
		 sprintf( buff, "%d %s %s", i, mBaseSrcList[i].address, mBaseSrcList[i].mountPoint );
		if ( i == 0 ) color = GREEN;
		else color = WHITE;
		lcdTextColor( color );
		lcdDispText( i + lineOffset, buff );
	}
	int idx = 0;
	int command = 0;
	while(1){
		command = waitButton( 1, 1, 1, 1, 2, 3 );
		if ( command == 1 ){	// next item
			if ( mNumBaseSrc == 1 ) continue;
		//	if ( idx == 0 ) sprintf( buff, "0 UART(PH connector)" );
		//	else sprintf( buff, "%d %s %s", idx, mBaseSrcList[idx].address, mBaseSrcList[idx].mountP//oint );
			sprintf( buff, "%d %s %s", idx, mBaseSrcList[idx].address, mBaseSrcList[idx].mountPoint );
			lcdTextColor( WHITE );
			lcdDispText( idx + lineOffset, buff );

			idx++;
			if ( idx == mNumBaseSrc ) idx = 0;
			//if ( idx == 0 ) sprintf( buff, "0 UART(PH connector)" );
			//else sprintf( buff, "%d %s %s", idx, mBaseSrcList[idx].address, mBaseSrcList[idx].mountPoint );
			sprintf( buff, "%d %s %s", idx, mBaseSrcList[idx].address, mBaseSrcList[idx].mountPoint );
			lcdTextColor( GREEN );
			lcdDispText( idx + lineOffset, buff );
			continue;
		}
		else {
			break;
		}
	}
	lcdClear();
	lcdTextColor( WHITE );

	int wifiConnected = 0;
	unsigned long msecStart = millis();
	if ( command == 2 ){
		memcpy( &mBaseSrc, &mBaseSrcList[idx], sizeof(mBaseSrc) );
		mBaseSrc.valid = true;
		char* address = mBaseSrcList[idx].address;
		if ( strstr( address, "rtk2go" ) ) mBaseSrc.protocol = PROTO_NTRIP;
		if ( strstr( address, "ales" ) ) mBaseSrc.protocol = PROTO_NTRIP_GGA;
		
		return 1;
	}
	else mBaseSrc.valid = false;

	return 0;
}

int gpsSetRtcmMessage( int interface )
{
	// RTCM message
	int retCode = 0;
	int nret = gpsSetMessageRate( interface, 0xF5, 0x05, 1 );	// RTCM3.3 1005 
	if ( nret < 0 ) retCode = -1;
	nret = gpsSetMessageRate( interface, 0xF5, 0x4D, 1 );	// RTCM3.3 1077 GPS MSM7
	if ( nret < 0 ) retCode = -2;
	nret = gpsSetMessageRate( interface, 0xF5, 0x57, 1 );	// RTCM3.3 1087 GLONASS MSM7
	if ( nret < 0 ) retCode = -3;
	nret = gpsSetMessageRate( interface, 0xF5, 0x61, 1 );	// RTCM3.3 1097 Galileo MSM7
	if ( nret < 0 ) retCode = -4;
	nret = gpsSetMessageRate( interface, 0xF5, 0x7F, 1 );	// RTCM3.3 1127 BeiDou MSM7
	if ( nret < 0 ) retCode = -5;
	nret = gpsSetMessageRate( interface, 0xF5, 0xE6, 1 );	// RTCM3.3 1230 GLONASS code-phase biases
	if ( nret < 0 ) retCode = -6;
	
	return retCode;
}


// GPS受信機の初期化
//
int gpsInit()
{

	int nret;
	lcdClear();

	// GPS ZED-F9P　シリアルのボーレート設定
	lcdDispText( 3, "Check ZED-F9P UART");
	nret = gpsSetBaudrate( mGpsUartBaudrate );

	// 接続確認
	int i = 0;
	for( ; i < 3; i++ ){
		nret = gpsSetMessageRate( IF_UART, 0x01, 0x07, 1 );	// NAV-PVT
		if ( nret == 0 ) break;
		dbgPrintf("gpsInit() error nret=%d  retrying\r\n", nret);
		delay(200);
	}
	if ( i == 3 ){
		dbgPrintf("No M5F9P module. Stop. \r\n", nret);
		lcdDispText( 5,"M5F9P does not respond." );
		lcdDispText( 8,">>> Press any button. ");
		lcdDispText( 9,"Device will be reset" );
		waitButton();
		ESP.restart();
	}
	
	// 基地局設定の解除
	nret = gpsSetBaseCoordinate( IF_UART, 0, 0, 0, 0, 0, 0 );
	nret = gpsSetMessageRate( IF_UART, 0x01, 0x3b, 0 );	// disable Survey-in data

	mReceiverType[0] = IF_UART;
	
	// 台数確認
	mNumReceivers = 1;
	lcdClear();
	
	// 各受信機の移動局として初期化
	for( int i=0; i < mNumReceivers; i++ )
	{
		int f9pInterface = mReceiverType[i];
		if ( f9pInterface == 0 ) continue;

		// output UBX only
		if ( f9pInterface == IF_UART )
			// input ubx:1 nmea:1 rtcm:1   output ubx:1 nmea:0 rtcm:0
			nret = gpsSetUartPort( f9pInterface, 1, mGpsUartBaudrate, 1, 1, 1, 1, 0, 0 );
		if ( nret < 0 ){
			return -1;
		}
		
		// output message
		nret = gpsSetMessageRate( f9pInterface, 0x01, 0x04, 1 );	// NAV-DOP
		nret = gpsSetMessageRate( f9pInterface, 0x01, 0x07, 1 );	// NAV-PVT
		nret = gpsSetMessageRate( f9pInterface, 0x01, 0x14, 1 );	// NAV-HPPOSLLH
		nret = gpsSetMessageRate( f9pInterface, 0x01, 0x35, 1 );	// NAV-SAT

		// output rate = 1Hz
		nret = gpsSetMeasurementRate( mReceiverType[i], 1000 );
		if ( nret < 0 ) return -2;
	}
}

int gpsGetPosition( struct stGpsData *gpsData, int msecTimeout )
{
	struct stUbxStatus ubxStatus;
	memset( (void*) &ubxStatus, 0, sizeof(ubxStatus) );

	unsigned long msecStart = millis();
	int found = 0;
	int retCode = -1;
	while(1){
		if ( millis() - msecStart > msecTimeout ) break;
		int nret = ubxDecode( IF_UART, &ubxStatus );
		if ( nret == 0 && ubxStatus.statusNum == 10 ){
			int msgClass = ubxStatus.msgClass;
			int msgId = ubxStatus.msgId;
			if ( msgClass == 0x01 && msgId == 0x07 ){ // NAV-PVT
				nret = ubxDecodeNavPvt( &ubxStatus, gpsData );
				if ( gpsData->ubxDone ){
					return 0;
				}
			}
			ubxStatus.statusNum = 0;
		}
		delay(1);
	}
	
	return retCode;
}


// ************************************************************
//                         基準局
// ************************************************************

// 基準局に接続する
//
// ・接続先等は mBaseSrcに設定しておく
//
// 戻り値＝ 0:正常終了(接続完了）
//         負数：エラー
//
int connectBaseSource()
{
	int nret;

	if ( mBaseRecvClient == NULL ) return -1;

	bool connected = false;
	while(1){		
		lcdClear();
		lcdDispText( 3, "Connecting to base station %s ", mBaseSrc.address );

		nret = mBaseRecvClient->connect( mBaseSrc.address, mBaseSrc.port ); 
		if ( nret == 0 ){
			connected = true;
			break;
		}
		else {
			lcdDispText( 6, "> Can't connect. Retry ?" );
			lcdDispButtonText( 2, WHITE, "YES", "NO", "" );
			int yes = waitButton( 1, 1, 0, YES, NO, 0 );
			lcdClear();
			if ( ! yes ) return -2;
		}
	}
	lcdClear();
	if ( ! connected ) return -3;
	if ( mBaseSrc.protocol == PROTO_NONE ) return 0;
	
	// NTRIP
	connected = false;
	dbgPrintf("Requesting MountPoint's Raw data\r\n");
	if ( mNetAccess > 0 && mBaseSrc.valid ){
		while(1){
			lcdClear();
			lcdDispText( 3, "Connecting to mount point %s  ", mBaseSrc.mountPoint);
			for( int i=0; i < 3; i++ ){
				nret = mBaseRecvClient->ntripRequest( mBaseSrc.mountPoint, 
												mBaseSrc.user, mBaseSrc.password );
				if ( nret == 0 ) {
					connected = true;
					break;
				}
				else {
					dbgPrintf("Requesting MountPoint is failed. nret=%d !!!\r\n", nret);
				}
			}
			if ( connected ) break;
			lcdDispText( 5, "> Can't connect." );
			lcdDispButtonText( 2, WHITE, "Retry", "Cancel", "" );
			lcdDispText( 8, ">>> Do you retry ?" );
			int nret = waitButton( 1, 1, 0, 1, 0, 0 );
			if ( ! nret ) break;
		}
		if ( connected ){
			dbgPrintf("Ntrip OK. host=%s  mount point=%s\r\n", mBaseSrc.address, mBaseSrc.mountPoint);
		}
		lcdClear();
	}
	if ( connected ) return 0;
	else return -4;
}


// ************************************************************
//                         時刻
// ************************************************************


// 最新の測位時刻を取得する
//
// 戻り値＝0：正常終了
//        負数：エラー
//
int IRAM_ATTR getPvtTime( struct stDateTime *time,  unsigned long *pvtMicros )
{
	struct stGpsData* pGpsData = &mGpsData[0];
	unsigned long msecNow = millis();
	if ( pGpsData->year == 0 ) return -1;
	time->year = pGpsData->year;
	time->month = pGpsData->month;
	time->day = pGpsData->day;
	time->hour = pGpsData->hour;
	time->minute = pGpsData->minute;
	time->second = pGpsData->second;
	time->msec = pGpsData->msec;
	*pvtMicros = pGpsData->micros;
	return 0;
}

time_t IRAM_ATTR dateTime2UnixTime( struct stDateTime *dateTime )
{
	struct tm tmTime;
	memset( (void*) &tmTime, 0, sizeof(tmTime) );

	int sec = dateTime->second;
	if ( dateTime->msec >= 500 ) sec++;
	tmTime.tm_sec = sec;
	tmTime.tm_min = dateTime->minute;
	tmTime.tm_hour = dateTime->hour;
	tmTime.tm_mday = dateTime->day;
	tmTime.tm_mon = dateTime->month - 1;
	tmTime.tm_year = dateTime->year - 1900;
	return mktime( &tmTime );
}

void unixTime2DateTime( time_t unixTime, struct stDateTime *dateTime )
{
	struct tm *tmTime = localtime( &unixTime );
	
	dateTime->msec = 0;
	dateTime->second = tmTime->tm_sec;
	dateTime->minute = tmTime->tm_min;
	dateTime->hour = tmTime->tm_hour;
	dateTime->day = tmTime->tm_mday;
	dateTime->month = tmTime->tm_mon + 1;
	dateTime->year = tmTime->tm_year + 1900;
}


// ************************************************************
//                           NMEA
// ************************************************************


// NMEA RMC,GGAセンテンスを作成して保存用バッファ(mSaveBuff)に格納
//
// 戻り値＝バッファに保存したデータのバイト数
//
int setNmeaData( int i )
{
	return setNmeaData( &mGpsData[i], mSaveBuff, SAVE_BUFF_MAX );

}

// NMEA RMC,GGAセンテンスを作成してバッファに格納
//
// 戻り値＝バッファに保存したデータのバイト数
//
int setNmeaData( struct stGpsData* pGpsData, char* buff, int buffSize )
{
	if ( buffSize < 150 ) return -1;	// 150：RMC,GGAに必要なバイト数
	
	int year = pGpsData->year;
	int month = pGpsData->month;
	int day = pGpsData->day;
	int hour = pGpsData->hour;
	int minute = pGpsData->minute;
	int sec = pGpsData->second;
	int msec = pGpsData->msec;
	
	int dmy = day * 10000 + month * 100 + ( year - 2000 );
	int hms = hour * 10000 + minute * 100 + sec;
	int csec = msec / 10;
	
	double lat = pGpsData->lat;
	double lon = pGpsData->lon;
	double alt = pGpsData->height;
	double geoid = pGpsData->geoidSep;
	double altMSL = alt - geoid;
	int fixGGA = pGpsData->quality;
	
	double speed = pGpsData->velocity * 0.539957;	// knots
	double heading = pGpsData->direction;
	int numSats = pGpsData->numSatelites;
	double pdop = pGpsData->dop;

	// RMC
	sprintf( buff, "$GPRMC,%06d.%02d,A,%.6lf,N,%.6lf,E,%.3lf,%.1lf,%06d,,",
					hms, csec, deg2degmin(lat), deg2degmin(lon), speed, heading, dmy );
	unsigned char csum = checksumOf( buff + 1, strlen( buff ) - 1 );
	sprintf( buff + strlen( buff ), "*%02x\r\n", csum );

	//GGA
	char* pbuff = buff + strlen( buff );
	sprintf( pbuff, "$GPGGA,%06d.%02d,%.6lf,N,%.6lf,E,%d,%d,%.1lf,%.3lf,M,%.3lf,M,,0000",
			hms, csec, deg2degmin(lat), deg2degmin(lon),  fixGGA, numSats, 
			pdop, altMSL, geoid );
	csum = checksumOf( pbuff + 1, strlen( pbuff ) - 1 );
	sprintf( pbuff + strlen( pbuff ), "*%02x\r\n", csum );
	
	return strlen( buff );
}

// 衛星データをNMEAセンテンスで出力する。
//
// outDevice = 1:bluetooth
//
int sendNmeaSatData( struct stGpsSat* satellites, int numSatellites, struct stGpsData* gpsData, int outDevice )
{
	int nret;
	char charIds[] = { 'P', 'N', 'A', 'B', 'N', 'N', 'L' };	//GPS, SBAS, Galileo, BeiDou, NONE, QZSS, GLONASS
	
	byte gnssIdNow = -1;
	int startIndex, endIndex;
	for( int i=0; i < numSatellites; i++ ){
		stGpsSat* pSat = &satellites[i];
		if ( gnssIdNow < 0 ) {
			gnssIdNow = pSat->gnssId; 
			startIndex = i;
			endIndex = i;
			continue;
		}
		if ( gnssIdNow == pSat->gnssId ){
			endIndex = i;
			if ( i < numSatellites - 1 ) continue;
		}
		else endIndex = i - 1;
		
		char charId = 'N';
		if ( gnssIdNow < 7 ) charId = charIds[ gnssIdNow ];
		nret = sendNmeaSatData2( charId, satellites, startIndex, endIndex, gpsData, outDevice );
		if ( endIndex == numSatellites - 1 ) break;
		if ( i == numSatellites - 1 ){
			startIndex = numSatellites - 1;
			nret = sendNmeaSatData2( charId, satellites, startIndex, startIndex, gpsData, outDevice );
			break;
		}
		gnssIdNow = pSat->gnssId;
		startIndex = i;
		endIndex = i;
	}
	
	return 0;
}

// 同一のgnssIdの衛星についてNMEAで出力する。
//
int sendNmeaSatData2( char charId, struct stGpsSat* satellites, int startIndex, int endIndex, 
								struct stGpsData* gpsData, int outDevice )
{
	int nret;
	
	char* buff = mSaveBuff;
	if ( SAVE_BUFF_MAX < 120 ) return -1; 
	int numSats = endIndex - startIndex + 1;
	int numPages = numSats / 4;
	if ( numSats % 4 ) numPages++;
	
	// GSA
	int n = 12;
	if ( numSats < n ) n = numSats;
	sprintf( buff, "$G%cGSA,A,3", charId );
	for( int i=0; i < n; i++ ){
		sprintf( buff + strlen(buff), ",%d", satellites[i].svId );
	}
	sprintf( buff + strlen(buff), ",%.2lf,%.2lf,%.2lf",gpsData->dop, gpsData->hDop, gpsData->vDop );	if ( outDevice == 1 ){
		nret = sendNmea( buff, outDevice );
	}

	// GSV
	int index = startIndex;
	for( int i=0; i < numPages; i++ ){
		int pageNum = i + 1;
		sprintf( buff, "$G%cGSV,%d,%d,%d", charId, numPages, pageNum, numSats );
		for( int j=0; j < 4; j++ ){
			stGpsSat* pSat = &satellites[index++];
			sprintf( buff + strlen(buff), ",%d,%d,%d,%d", pSat->svId, pSat->elev, pSat->azim, pSat->cno );
			if ( index == endIndex ) break;
		}

		if ( outDevice == 1 ){
			nret = sendNmea( buff, outDevice );
		}
	}

	return 0;
}

int sendNmea( char* buff, int outDevice )
{
	int nret;
	
	unsigned char csum = checksumOf( buff + 1, strlen( buff ) - 1 );
	sprintf( buff + strlen( buff ), "*%02x\r\n", csum );
	int numOutBytes = strlen( buff );

	if ( outDevice == 1 ){		
		nret = mBluetooth.write( (byte*)mSaveBuff, numOutBytes);
		if ( nret != numOutBytes ){
			dbgPrintf("BT write error nret=%d byteToWrite=%d\r\n", nret, numOutBytes );
		}
	}

	return 0;
}

double deg2degmin( double degree )
{
	int deg = (int)degree;
	double min = (degree - deg) * 60;
	double degmin = deg * 100 + min;
	return degmin;
}
	

// Checksum を返す。
//	
unsigned char checksumOf( char* pbuff, int NumOfBytes )
{
	int i;
	unsigned char *buff;
	
	buff = (unsigned char*) pbuff;
	
	unsigned char c = *buff++;
	for ( i = 1; i < NumOfBytes; i++ )  c = c ^ *buff++ ;
	
	return c;
}


// ************************************************************
//                           DEBUG
// ************************************************************

void dbgPrintf( String msg )
{
	dbgPrintf( "%s", msg.c_str() );
}

void dbgPrintf( char* format, ... )
{
	if ( mDebugDevice == DBG_NONE ) return;
	
	char buff[256];
	va_list args;
	va_start( args, format );
	vsnprintf( buff, 256, format, args );
	va_end( args );
	
	switch( mDebugDevice ){
		case DBG_SERIAL:
			Serial.print( buff );
			break;
	}
}

void dbgHeapSize(char *message)
{
	dbgPrintf("Heap Size %s = %d\r\n", message, esp_get_free_heap_size());
}
