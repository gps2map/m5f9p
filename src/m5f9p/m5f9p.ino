/*

          ************************************************************
              u-blox ZED-F9Pモジュール on M5Stack  制御用プログラム
          ************************************************************


---------------- This file is licensed under the MIT License -------------------

Copyright (c) 2020 Geosense Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------

*/

#define M5STACK_MPU6886 

#include <M5Stack.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <Wire.h>
#include <WebServer.h>
#include <SD.h>
#include <time.h>
#include <esp_wifi.h>
#include <rom/rtc.h>

#include "BluetoothSerial.h"

#include "m5f9p.h"
#include "RingBuff.h"
#include "i2c.h"
#include "gis.h"
#include "gps.h"
#include "IniFile.h"
#include "TcpClient.h"
#include "ModemFep01.h"
#include "ui.h"

#define TINY_GSM_MODEM_UBLOX
#include <TinyGsmClient.h>


#define CANCEL 0
#define OK 1
#define NEXT 2

#define OFF 0
#define ON 1

// ************************************************************
//                        モジュール変数
// ************************************************************

byte mVersionMajor = 1;
byte mVersionMinor = 0;
byte mVersionPatch = 17;

int mCpuFreqMHz;

bool mSetupDone = false;

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


//#define BT_ENABLE 1
#define BT_ENABLE 0

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

// 位置
struct stPosition {
	double lat;
	double lon;
	double height;
};

// Runモード
#define RUN_UI 0
#define RUN_NO_UI 1
int mRunMode = RUN_UI;		// リセット後、UIで実行パラメータを選択して実行する(既定値)か、
							// 直前のパラメータで実行するか

// 異常リブート時の動作モード　INIファイルで設定
int mRebootMode = RUN_UI;

// 実行パラメータ
#define RUN_INFO_STR_MAX 10
struct stRunInfo {
	int rebootMode;		// 異常リブートの際、直前の動作モードで実行する時１。UIで選択する時0．
	int lcdRotation;	// 画面の向き 0:回転無 1:180度回転
	int netAccess;		// ネット接続　1:WIFI 2:3G 3:920MHz
	int wifiAp;			// Wifiアクセスポイント　INIファイルのWIFI接続先番号([wifi1]..[wifi3])
	int opeMode;		// 動作モード 1:Rover 2:Base 3:Moving Base
	int baseSrc;		// 基準局データ取得先 -1:UART 1以上：INIファイルの
						//          基準局データ取得先番号（[source1]..[source9])
	struct stPosition basePosition;	// 基準局のアンテナ位置座標
	char basePositionName[RUN_INFO_STR_MAX];		//       〃　　　　　　　　名称
	int saveFormat;		// データ保存形式　0:NMEA 1:RAW 2:RTCM
	int saving;			// SDカード保存 0:保存していない　1:保存中
	int solutionRate;	// 1秒あたりの位置更新レート 
	int baseSend;		// NTRIPキャスタへの送信 0:送信無し 1:送信あり
} mRunInfo;


//

#define USE_WIFI      1
#define USE_MODEM_3G  2
#define USE_MODEM_920 3

int mNetAccess = 0;

char* mSsid;
char* mPassword;
IPAddress mWifiLocalIp;

// メイン受信機から出力されたデータの配信サーバ
// 主に基準局データ（RTCM,RAW）用途だが移動局のデータも利用可能
//
#define SERVER_BUFF_MAX 4096
#define SERVER_CLIENT_MAX 3
int mServerPort = 10000;	// サーバのポート番号　既定値
WiFiServer* mWifiServer;	// 基準局または移動局データ配信用サーバ
WiFiClient *mServerClient[ SERVER_CLIENT_MAX ];	// 配信用サーバに接続されたクライアント
char mServerBuff[ SERVER_BUFF_MAX ];
int mServerWriteIndex;
int mServerReadIndex;

// スイッチサイエンス製３Gモデム用
TinyGsm mModem(Serial2); /* 3G board modem */
TinyGsmClient mGsmClient(mModem);
int mModemBaudrate = 115200;
IPAddress mModemLocalIp;

// 双葉電子製920MHzモデム用
ModemFep01 mFep01;
int m920TotalBytes;		// 920MHz modemにより取得したバイト数
int m920LastBytes;
int m920RecvCount;
int m920SendCount;
int m920SendReady;

// パス
String mRootDir = "/m5f9p";
String mGpsLogDir = mRootDir + "/gpslog";
String mHtmlDir = mRootDir + "/html";

String mRunInfoPath = mRootDir + "/m5f9p.run";	// 実行パラメータファイル
String mBootLogPath = mRootDir + "/m5f9p.log";	// Boot　ログ

// INIファイル用
String mIniPath = mRootDir + "/m5f9p.ini";
IniFile *mIniFile;

#define WIFI_MAX 4
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

#define BASE_SRC_MAX 10
struct stBaseSource {
	bool valid;
	int type;
	int protocol;
	int ggaPeriod;
	char address[64];
	int port;
	char mountPoint[32];
	char user[32];
	char password[32];
} mBaseSrcList[ BASE_SRC_MAX ];
int mNumBaseSrc;
struct stBaseSource mBaseSrc;		// 基準局データ受信先

// 基準局座標
#define BASE_POS_MAX 10
struct stBasePosition {
	bool valid;
	char name[17];
	double lat;
	double lon;
	double height;
} mBasePosList[ BASE_POS_MAX ];
int mNumBasePos;
struct stBasePosition mBasePosition;

int mSurveyinSec = 60;			// Survey in 秒数
float mSurveyinAccuracy = 2;	// Survey in 精度（ｍ単位）


// データ保存、配信用
#define SAVE_BUFF_MAX 256
char mSaveBuff[ SAVE_BUFF_MAX ];
int mSaveBuffBytes = 0;
char mSaveFileName[RECEIVER_MAX][64];

char* mGGAAddress;

#define SAVE_NMEA 0
#define SAVE_RAW 1
#define SAVE_RTCM 2
int mSaveFormat;	// 保存時のフォーマット　NMEA or Raw data

int mFileSaved;		// 保存した回数
int mFileSaving;
QueueHandle_t mQueueFileSave;
int mQueueFileErrorCount;
int mQueueFileErrorCountLast;

int mWifiConnected;
char mSoftApSsid[16];
char mSoftApPassword[16] = "m5f9p123";
IPAddress mSoftApIp( 192,168,5,1 );

WebServer* mWebServer;
int mHttpPort = 80;

// 基準局データ受信用
bool mNtripSelected = false;

TcpClient *mBaseRecvClient;	// 基準局データを受信するためのクライアント
TcpClient *mBaseSendClient;	// 基準局データを送信するためのクライアント
boolean mBaseRecvReady;
boolean mBaseSendReady;
char *mNtripClientName = "M5F9P_Client_1.0";
char *mNtripServerName = "M5F9P_Server_1.0";

WiFiClient mWifiClient;

#define BASE_RECV_BUFF_MAX 256
char mBaseRecvBuff[ BASE_RECV_BUFF_MAX ];
int mBaseRecvIdx;

// 基準局データ送信用
struct stCaster {
	bool valid;
	char address[64];
	int port;
	char mountPoint[32];
	char password[32];
} mCaster;



#define NTRIP_SEND_BUFF_MAX 2048
byte mNtripSendBuff[ NTRIP_SEND_BUFF_MAX ];
int mNtripSendWriteIdx;
int mNtripSendReadIdx;
int mNtripSendCounter;

#define MODEM920_SEND_BUFF_MAX 2048
byte m920SendBuff[ MODEM920_SEND_BUFF_MAX ];
int m920SendWriteIdx;
int m920SendReadIdx;

//#define NTRIP_RTK2GO 1
//#define NTRIP_ALES 2
//int mNtripCaster;

int mBaseRecvCount;
int mNripLastCount;
unsigned long mBaseRecvLastMillis;
unsigned long mNripLastCountMillis;

// 測位データ配信用
char mAgribusIp[18];
int mAgribusPort;
bool mAgribusReady;
WiFiClient *mAgribusClient;

// gps interface  -1:UART CH1  positive: I2C address
//#define IF_NONE 0
//#define IF_UART -1

int mReceiverType[RECEIVER_MAX];	// 0:none 1:F9P/serial 2:F9P/I2C

// UART用
char mUartBuff[ UART_BUFF_MAX ];
int mUartWriteIndex;
int mUartReadIndex;
int mUartSaveIndex;

// I2C用
char mI2cBuff[RECEIVER_MAX][ I2C_BUFF_MAX ];
int mI2cWriteIndex[RECEIVER_MAX];
int mI2cReadIndex[RECEIVER_MAX];

xSemaphoreHandle mMutexI2c;
struct stI2cCommand mI2cCommand;
int mI2cCommandTimeout = 100;		// msec

// 基準局、Moving Base動作時 RTCMデータ配信用
#define RTCM_BUFF_MAX 1024
byte mRtcmBuff[ RTCM_BUFF_MAX ];
int mRtcmIndex;
bool mRtcmXferReady;

int mServerCounter;		// 基準局データ配信回数

// 接続されているGPS受信機の台数
int mNumReceivers;
int mNumI2cReceivers;

//
unsigned long mGpsLastMillis;
int mSolutionRate;
int mGpsUartBaudrate = 230400;		// 460800,230400,115200,57600,38400,19200,9600

int mI2cUartSyncMode;		// I2Cのデータ読み出しをUARTに同期させる場合 true

struct stGpsData mGpsData[ RECEIVER_MAX ];	// 取得された位置データ
struct stGpsData mGpsDataLast;				// メイン受信機の直前のデータ

struct stGpsRelPos mGpsRelPos[ RECEIVER_MAX];
struct stUbxStatus mUbxStatus[ RECEIVER_MAX ];


// 時刻
int mPinGpsPPS = 36;
unsigned long mGpsPPSMicros;

// 軌跡
#define TRACK_MAX 200
struct stGpsTrack {
	time_t time;
	double lat;
	double lon;
} mGpsTrack[ TRACK_MAX ];
int mTrackIndex;

// SDカード保存用
#define SD_BUFF_MAX 2048
char *mSdBuff[RECEIVER_MAX];		// SDカード保存用バッファ
bool mSdSaveReady;					// SDカードで保存可能な時True
unsigned long mSdSavedTime[RECEIVER_MAX];	// SDカードに保存した通算ミリ秒
uint64_t mSdTotalBytes;

// SDカードの地図データ
struct stMapData {
	String path;
	double centerLat;
	double centerLon;
	double leftLon;
	double rightLon;
	double upperLat;
	double bottomLat;
	int mapType;
	int zoomLevel;
	int width;
	int height;
	time_t savedTime;
} mMapData;
struct stMapData *mLcdMap;

// 
int mSoundOn = 1;

// ３ピンJST-PH コネクタ 主にUART用　TTLレベルのGPIOとしても利用可能
unsigned long mBitTime;
int mPinPhOut = 12;		// 出力
int mPinPhIn = 35;		// 入力

#define PH_UART_BUFF_MAX 1024
RingBuff mPhUartBuff;
int mPhUartBaudrate = 115200;

int mPinGpsReset = 15;

int mOpeMode;
char *mOpeModeStr[4] = { "None", "Rover", "Base", "Moving Base" };

// color
int mColor[] = { 0xffdfdf, 0xffffc0, 0x80ffc0, 0xfff7b7,
				 0xff6f6f, 0xffff70, 0x80ff40, 0xffab57 };
int mDisplayType = 0;		// M5Stackのロットにより色が違う機種がある。その補正の番号。

// 受信機名称
char mReceiverName[16] = "m5f9p";		// AP名として使用

// IMU
bool mUseImu = false;
//bool mUseImu = true;
unsigned long mImuLastRead = 0;

float mAccelX;
float mAccelY;
float mAccelZ;

float mGyroX;
float mGyroY;
float mGyroZ;


// 


// デバグ
#define DEBUG_PH 1
int mDebugPort;

// map
char *mGoogleKey;

// test
int mPinTest = 16;


// ************************************************************
//                         Arduino 初期化
// ************************************************************
//

// 関数宣言
void IRAM_ATTR phUartInIsr();
void IRAM_ATTR gpsPPSIsr();
//void gpsPPSIsr();
void IRAM_ATTR gpsPPSIsr2();
int IRAM_ATTR getPvtTime( struct stDateTime *time,  unsigned long *pvtMicros );
time_t IRAM_ATTR dateTime2UnixTime( struct stDateTime *dateTime );


void setup() {
	int nret,year,month,day,hour,minute,second;
	char buff[256];
	struct stGpsData gpsData;
	
	M5.begin();
	buff[0] = '\0';


	mCpuFreqMHz = ESP.getCpuFreqMHz();
	int taskStackSize = 4096;
	
	setSineCurve( 1000, 100 );
	speakerOff();

	// バージョン
	sprintf( buff + strlen(buff), "Version: %d.%d.%d\r\n", mVersionMajor, mVersionMinor, mVersionPatch );


// ＞＞＞　　ブート原因にかかわらず自動実行できるようにする。
// ＞＞＞　　最初の３秒間にボタンを押すとUIモードで実行

	// ブート原因
	int resetReason0 = rtc_get_reset_reason(0);
	int resetReason1 = rtc_get_reset_reason(1);
	sprintf( buff + strlen(buff), "Reset: cpu0=%d cpu1=%d\r\n", resetReason0, resetReason1 );
	dbgPrintf( "!! %s", buff );

	// 異常リセット	
	bool abnormalReset =  ( ( resetReason0 != 1 && resetReason0 != 16 ) || resetReason1 != 14 );
	
	// Runモード
	mRunMode = RUN_UI;
	if ( abnormalReset ){
		nret = readRunInfo( &mRunInfo );
		if ( nret == sizeof( mRunInfo ) ){
			mRunMode = mRunInfo.rebootMode;
			dbgPrintf( "Abnormal reset mRunMode=%d\r\n", mRunMode );
		}
	}
	
	// ZED-F9Pをリセット
	Wire.begin();
	for( int i=0; i < 3; i++ ){
		int i2cAddress = 0x42 + i;
		dbgPrintf( "Resetting ZED-F9P address=%xH\r\n", i2cAddress );
		gpsReset2( i2cAddress );
	}


	// 画面の初期化
	if ( mRunMode == RUN_NO_UI ) {
		dbgPrintf( "Running under the last condition\r\n" );
		nret = lcdInit( mRunInfo.lcdRotation, "" );
	}
	else {
		nret = lcdInit( -1, buff );
	 	lcdDispText( 3, "Initializing");
		mRunInfo.lcdRotation = nret;
	}

	// test
	pinMode( mPinTest, OUTPUT);
	
	// gps enable
	pinMode( mPinGpsReset, OUTPUT);
	digitalWrite( mPinGpsReset, HIGH);

disableCore1WDT();
disableCore0WDT();

	pinMode( mPinGpsPPS, INPUT);
//	attachInterrupt( digitalPinToInterrupt(mPinGpsPPS), gpsPPSIsr, RISING);

	
	// SD
j_sd:
	for( int i=0; i < 5; i++ ){		// １回では正しい容量が得られない場合がある。
		mSdTotalBytes = SD.totalBytes();
		delay(100);
		if ( mSdTotalBytes >0 && mSdTotalBytes == SD.totalBytes() ) break;
		delay(100);
	}
	if ( mSdTotalBytes == 0 ){
		lcdClear();
		lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
		lcdDispText( 3, "SD card : 0 MB" );
		lcdDispText( 5, ">>> Check again ?" );
		lcdDispText( 7, "Yes > Device will be reset." );
		int yes = waitButton( 1, 1, 0, YES, NO, 0 );
		lcdClear();
		if ( yes ) {
			ESP.restart();
			// SD.begin(); ;これでは認識できない
			//delay(1000);
			//goto j_sd;
		}
	}
	
	lcdDispText( 5, "SD card: %lu MB", (int)(mSdTotalBytes / 1E6) );
	dbgPrintf("SD card totalBytes=%lu\r\n", mSdTotalBytes);
//	SdFile::datetimeCallback( sdDateTime );  // not supported

	
	// INIファイル
	lcdDispText( 6, "Reading INI file" );
	if ( mSdTotalBytes > 0 ){
		if ( SD.exists( mIniPath ) ){
			mIniFile = new IniFile();
			nret = mIniFile->open( mIniPath.c_str() );
			if ( nret == 0 ){
				nret = readIniFile( mIniFile );
			}
			else {
				lcdDispText( 7, "INI file not found" );
			}
		}
	}

	// JST-PHコネクタ ( soft serial )
	mBitTime = mCpuFreqMHz * 1000000 / mPhUartBaudrate;
	pinMode(mPinPhOut, OUTPUT);
	pinMode(mPinPhIn, INPUT);
	dbgPrintf("JST-PH uart baudrate=%d\r\n", mPhUartBaudrate );
	
	// serial port for debug
	if ( mDebugPort == DEBUG_PH ) Serial.begin(115200, SERIAL_8N1, mPinPhIn, mPinPhOut );
	else Serial.begin(115200);
	Serial.setDebugOutput(true);

    dbgPrintf("CPU clock=%d\r\n",mCpuFreqMHz);
    
    // メモリ確保
	nret = mPhUartBuff.setSize( PH_UART_BUFF_MAX );
	if ( nret <= 0 ) {
		dbgPrintf("PhUartBuff alloc error");
	}

	// Wire初期化
	// Wire(I2C)を使っているライブラリに関する初期化はここから↓↓↓↓↓↓↓↓↓↓

	// IMU
	Wire.begin();
	Wire.setClock( 400000 );

	if ( mUseImu ){
		M5.IMU.Init();
	}
	
	// ここまでに置く。↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

    // I2Cアクセススレッド(core 0)
	mMutexI2c = xSemaphoreCreateMutex();
    mI2cCommand.command = 0;
	xTaskCreatePinnedToCore(taskI2c, "taskI2c", taskStackSize, NULL, 1, NULL, 0);
	delay(3000);	// Short wait time cause error
	

	// GPSデータ受信スレッド（core 0)
	mOpeMode = 0;
	mI2cUartSyncMode = 1;
	xTaskCreatePinnedToCore(taskGetSensorData, "taskGetSensorData", taskStackSize, NULL, 1, NULL, 0);
	delay(10);


	// 1PPS test
	//pinMode( mPinPhIn, INPUT);
	//attachInterrupt( digitalPinToInterrupt(mPinPhIn), gpsPPSIsr2, RISING);

	// ネット接続方法の選択。Wifi or Modem ?
	// 動作モード選択の後の方が自然だが、GPS受信機の衛星捕捉の時間を取るために
	// 先に行う。
	// 
	lcdDispText( 8, "Soft AP start" );
	nret = selectNetConnection();
	mRunInfo.netAccess = mNetAccess;
	lcdClear();
	
	lcdDispText( 3, "Starting sensor thread\r\n" );

	// WEBサーバ
	nret = webServerInit();

	// GPS受信機の初期化、受信機の数え上げ
j1:	gpsInit();	

	// SDカード保存スレッド（Core 1)
	mSdSaveReady = false;
	if ( mSdTotalBytes > 0 ){
		mQueueFileSave = xQueueCreate( 32, sizeof( stGpsData ) );	//10Hzで1秒毎に保存する場合
																	//は30以上必要
		if ( mQueueFileSave ){
			xTaskCreatePinnedToCore(taskSdSave, "taskSdSave", taskStackSize, NULL, 1, NULL, 1);
			int i;
			for ( i = 0; i < mNumReceivers; i++ ){
				mSdBuff[i] = (char*) malloc( SD_BUFF_MAX );
				if ( mSdBuff[i] == NULL ) break;
				*mSdBuff[i] = '\0';
			}
			if ( i == mNumReceivers ) mSdSaveReady = true;
		}
		else if ( mRunMode == RUN_UI ) {
			lcdClear();
			lcdDispText( 3, "> Can't use SD card " );
			lcdDispText( 10, ">>> Press any button" );
			waitButton();
		}
	}
	
	// gpsInit()ではSerial1のボーレートの自動設定機能を使っている。
	// その影響かSerial2のボーレートが変わってしまうようだ。
	// そのため、下記設定が必要。
	if ( mNetAccess == USE_MODEM_920 ){		
		Serial2.begin( mFep01.baudrate(), SERIAL_8N1, 16, 17 );
	}
	else if ( mNetAccess == USE_MODEM_3G ){		
		Serial2.begin( mModemBaudrate, SERIAL_8N1, 16, 17 );
	}

	// 動作モード選択
	if ( mRunMode == RUN_UI ) {
		lcdClear();
		lcdDispButtonText( 2, WHITE, "Rover", "Base", "MovBase" );
		lcdDispText( 3, ">>> Select operation mode" );
		mOpeMode = waitButton( ON, ON, ON, MODE_ROVER, MODE_BASE, MODE_MOVING_BASE );
		mRunInfo.opeMode = mOpeMode;
	}
	else {
		mOpeMode = mRunInfo.opeMode;
	}
	
	// GPS受信テスト及び日時取得
	lcdClear();
	int count = 0;
	double lat = 100;
	double lon = 400;

	lcdDispButtonText( 2, WHITE, "", "Cancel", "" );
	lcdDispText( 2, ">>> Testing the ZED-F9P receiver" );
	bool canceled = false;
	while(1){
		if ( mRunMode == RUN_UI ) {
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
				setNmeaData( &gpsData, mSaveBuff, SAVE_BUFF_MAX );	// NTRIP接続の際にGGAが必要
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
			if ( mRunMode == RUN_UI ) {
				lcdTextColor( WHITE );
				lcdDispText( 10, ">>> Press any button" );
				waitButton();
			}
			break;
		}
	}
	
	// boot ログ書き込み
	year = month = day = hour = minute = second = 0;
	if ( gpsData.quality > 0 ){
		year = gpsData.year;
		month = gpsData.month;
		day = gpsData.day;
		hour = gpsData.hour;
		minute = gpsData.minute;
		second = gpsData.second;
	}
	sprintf( buff, "<%d-%02d-%02d %02d:%02d:%02d UTC> boot (%d : %d)\r\n", year, month, day, hour, minute, second, resetReason0, resetReason1);
	sdSave( mBootLogPath, buff, strlen( buff ) );

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
					mRunInfo.baseSrc = nret;
					if ( nret == 0 ) break;		// cancel
					if ( mBaseSrc.type == BASE_TYPE_TCP ){
						nret = connectBaseSource();	// 無手順もしくはNTRIP経由
						if ( nret == 0 ){
							mBaseRecvReady = true;
							break;
						}
					}
					else if ( mBaseSrc.type == BASE_TYPE_UART ){
						if ( mRunMode == RUN_UI ) {
							lcdClear();
							lcdDispText( 3, "Base station: JST-PH Connector" );
							lcdDispAndWaitButton( 4, "  baudrate: %d", mPhUartBaudrate );
						}
						attachInterrupt( digitalPinToInterrupt(mPinPhIn), phUartInIsr, FALLING);
  						mBaseRecvReady = true;
						break;
					}
				}
				
				// 保存形式の選択
				if ( mRunMode == RUN_UI ) {
					lcdClear();
					lcdDispButtonText( 2, WHITE, "NMEA", "RAW", "RTCM" );
					lcdDispText( 3, ">>> Select data format for saving" );
					while(1){	
						M5.update();
						if ( M5.BtnA.wasReleased() ) mSaveFormat = SAVE_NMEA;
						else if ( M5.BtnB.wasReleased() ) mSaveFormat = SAVE_RAW;
						else if ( M5.BtnC.wasReleased() ) mSaveFormat = SAVE_RTCM;
						else continue;
						break;
					}
				}
				else {
					mSaveFormat = mRunInfo.saveFormat;
				}
				mRunInfo.saveFormat = mSaveFormat;
				if ( mSaveFormat != SAVE_NMEA ){
					nret = gpsRawInit();
					if ( nret < 0 ){
						dbgPrintf( "gpsRawInit() error nret=%d\r\n", nret );
						lcdDispAndWaitButton( 3, "> RAW data not available nret=%d\r\n", nret );
					}
				}
			}

			// TCP Serverへの接続
			if ( mNetAccess == USE_WIFI ) connectTcpServer();

			break;
		
		case MODE_BASE:
			mRtcmXferReady = 0;
			if ( mNetAccess == USE_MODEM_920 ){
				nret = mFep01.setAddress( 1 );	// Base St. address = 1
				if ( nret < 0 ){
					lcdDispAndWaitButton( 3, "> Modem 920MHz set address error nret=%d\r\n", nret );
				}
			}
			
			if ( mRunMode == RUN_UI ){
				if ( mNumBasePos ){
					nret = basePosSelect();
				}
			
				if ( nret == 1 && mBasePosition.valid ) {
					nret = baseInit( 2, mBasePosition.lat, mBasePosition.lon, mBasePosition.height );
				}
				else nret = baseInit( 1, 0, 0, 0 );
				if ( nret < 0 ){
					if ( nret == -100 ){
						lcdClear();
						lcdDispText( 3, "Now resetting the GPS module." );
						gpsReset( IF_UART );
						delay(5000);
						lcdClear();
						goto j1;
					}
				
					lcdDispText( 0, "Base station init error (%d)", nret );
					dbgPrintf( "Base station init error nret=%d\r\n", nret );
					while(1);
				}
				strncpy( mRunInfo.basePositionName, mBasePosition.name, RUN_INFO_STR_MAX - 1 );
				mRunInfo.basePositionName[ RUN_INFO_STR_MAX - 1 ] = '\0';
				mRunInfo.basePosition.lat = mBasePosition.lat;
				mRunInfo.basePosition.lon = mBasePosition.lon;
				mRunInfo.basePosition.height = mBasePosition.height;
			}
			else{
				struct stPosition *p = & mRunInfo.basePosition;
				int nDest = sizeof( mBasePosition.name ) - 1;
				strncpy( mBasePosition.name, mRunInfo.basePositionName, nDest );
				mBasePosition.name[ nDest ] = '\0';
				mBasePosition.lat = p->lat;
				mBasePosition.lon = p->lon;
				mBasePosition.height = p->height;
				mBasePosition.valid = true;
				nret = baseInit( 2, p->lat, p->lon, p->height );
			}
			
			// Ntrip選択
//			mNtripSelected = ntripSelect( MODE_BASE, 0, 0 );
			
			mRtcmXferReady = 1;
			nret = gpsSetUartPort( IF_UART, 1, mGpsUartBaudrate, 1, 1, 1, 0, 0, 1 );	// output RTCM only
			
			break;
		
		case MODE_MOVING_BASE:
//			mOpeMode = MODE_ROVER;		// Ackを受信するため、暫定的に設定
			nret = movingBaseInit();		
//			mOpeMode = MODE_MOVING_BASE;
			if ( nret < 0 ){
				lcdDispText( 0, "Moving Base station init error (%d)", nret );
				dbgPrintf( "Moving Base station init error nret=%d\r\n", nret );
				while(1);
			}
			
			break;
	}

	// Bluetooth
	if ( BT_ENABLE ){
		if ( ! mBluetooth.begin( "M5Stack" ) ) dbgPrintf("Bluetooth init failed\r\n");
	}
	
	// Server
	mWifiServer = new WiFiServer( mServerPort );
	if ( ! mWifiServer  ){
		lcdDispAndWaitButton( 3, "Can't use wifi server." );
	}
	else {
		mWifiServer->begin();
		for ( int i=0; i < SERVER_CLIENT_MAX; i++ ) mServerClient[i] = NULL;
		xTaskCreatePinnedToCore(taskWifiServer, "taskWifiServer", taskStackSize, NULL, 1, NULL, 0);
	}

	//
	if ( mRunMode == RUN_UI ){
		mFileSaving = 0;
		mSolutionRate = 1;	// number of solution per second
	}
	else {
		mFileSaving = mRunInfo.saving;
		mSolutionRate = mRunInfo.solutionRate;
	}
	
	mFileSaved = 0;
	mBaseRecvCount = 0;
	mNripLastCount = 0;
	mNripLastCountMillis = millis();
	mBaseRecvIdx = 0;
	mBaseSendReady = false;
	mServerCounter = 0;
	mNtripSendCounter = 0;
	
	mRunInfo.rebootMode = mRebootMode;
	mRunInfo.saving = mFileSaving;
	mRunInfo.solutionRate = mSolutionRate;
	
	if ( mSolutionRate > 1 ) gpsSetSolutionRate( mSolutionRate );

	lcdClear();
	
	// 移動局タスクスタート
	if ( mOpeMode == MODE_ROVER || mOpeMode == MODE_MOVING_BASE ){
		// 移動局メインタスク（core 1）
		xTaskCreatePinnedToCore( taskRover, "taskRover", taskStackSize, NULL, 1, NULL, 1 );

		// 基準局データ受信スタート（core 0）
		if ( mNetAccess == USE_MODEM_920 && mOpeMode == MODE_ROVER ){	// 920MHzモデム経由
			xTaskCreatePinnedToCore(taskModem920Recv, "taskModem920Recv", taskStackSize, NULL, 1, NULL, 0);
		}
		else if ( mBaseSrc.type == BASE_TYPE_TCP || mBaseSrc.type == BASE_TYPE_UART ){
			if ( mBaseRecvReady ){
				xTaskCreatePinnedToCore(taskBaseRecv, "taskBaseRecv", taskStackSize, NULL, 1, NULL, 0);
			}
		}
	}
	// 基準局タスクスタート
	else if ( mOpeMode == MODE_BASE && mNetAccess ){
		// 基準局データ送信スタート(core 0)
		nret = connectNtripAsServer();
		if ( nret == 1 ){
			mBaseSendReady = true;
			mNtripSendWriteIdx = 0;
			xTaskCreatePinnedToCore(taskNtripSend, "taskNtripSend", taskStackSize, NULL, 1, NULL, 0);
			mRunInfo.baseSend = 1;
		}
	}

	// 920MHzモデム送信スレッドスタート
	if ( mNetAccess == USE_MODEM_920 && ( mOpeMode == MODE_BASE || mOpeMode == MODE_MOVING_BASE ) ){
		m920SendReady = 1;
		m920SendWriteIdx = 0;
		m920SendReadIdx = 0;
		xTaskCreatePinnedToCore(taskModem920Send, "taskModem920Send", 4096, NULL, 1, NULL, 0);
	}
	
	// ボタン監視スレッドスタート
	xTaskCreatePinnedToCore(taskButton, "taskButton", 1000, NULL, 1, NULL, 1);

	// 動作モード等の実行環境保存
	// 画面の向き、ネット接続方法、動作モード、基準局データ取得先、保存形式
	saveRunInfo( &mRunInfo );
	
	// 初期化終了
	dbgPrintf("setup() exit\r\n");
	lcdClear();
	mStartMillis = millis();
	mSetupDone = true;
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
//	M5.update();
	if ( mOpeMode == MODE_BASE ) loopBase();
	else loopRover();

//if ( millis() - mStartMillis > 10000 ) ESP.restart();

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

	if ( buttonCPressed() ){
		nextPage();
		buttonPressedReset();
		mLcdMap = NULL;
//		mSoundOn = !mSoundOn;
	}
}

void nextPage()
{
	mLcdPage++;
	if ( mLcdPage == mLcdPageMax ) mLcdPage = 0;
	lcdClear();
}



void loopBase(){

	switch( mLcdPage ){
		case PAGE_MAIN: 
			dispBaseMainPage();
			break;
		case PAGE_MAP:
			if ( ( ! mGoogleKey || strlen( mGoogleKey) == 0 ) ) mLcdPage++;
			else dispMapMain();
			break;
		case PAGE_INFO:
			dispInfo();
			break;
	}
	
}

void loopRover()
{

	switch( mLcdPage ){
		case PAGE_MAIN: 
			dispRoverMainPage();
			break;
		case PAGE_MAP:
			if ( mFileSaving ||  ( ! mGoogleKey || strlen( mGoogleKey) == 0 ) ) mLcdPage++;	// データ保存時は地図表示できない
			else dispMapMain();
			break;
		case PAGE_INFO:
			dispInfo();
			break;
	}
	
}


void dispBaseMainPage()
{
	lcdDispButtonText( 2, WHITE, "", "", "NextPage", false );
	lcdDispText( 1, " *** Base station ***" );
	
	if ( mBasePosition.valid ){
		lcdDispText( 3, "name: %s", mBasePosition.name );
		lcdDispText( 4, "lat: %.8f", mBasePosition.lat );
		lcdDispText( 5, "lon: %.8f", mBasePosition.lon );
		lcdDispText( 6, "height: %.3f", mBasePosition.height );
	}
	int colorIndex = mDisplayType * 4;
	lcdTextColor24( mColor[ colorIndex + 0] );
	if ( mNetAccess == USE_MODEM_920 ) lcdDispText( 7, " count = %d", m920SendCount );
	else lcdDispText( 7, "count = %d (%d)", mServerCounter, mNtripSendCounter );

	lcdTextColor24( mColor[ colorIndex + 1 ] );
	lcdDispText( 8, "AP IP address:%u.%u.%u.%u", 
					mSoftApIp[0], mSoftApIp[1], mSoftApIp[2], mSoftApIp[3] );
	lcdDispText( 9, "port:%d", mServerPort );

	if ( mNetAccess == USE_WIFI ){
		lcdTextColor24( mColor[ colorIndex + 2 ] );
		lcdDispText( 10, "IP address:%u.%u.%u.%u", 
					mWifiLocalIp[0], mWifiLocalIp[1],mWifiLocalIp[2], mWifiLocalIp[3] );

		if ( mBaseSendReady ){
			lcdTextColor24( mColor[ colorIndex + 2 ] );
			lcdDispText( 11, "NTRIP: %s", mCaster.address );
			lcdDispText( 12, "mntpnt: %s", mCaster.mountPoint );
		}
	}
	lcdTextColor( WHITE );

	delay(100);
	
}


void dispRoverMainPage() 
{
	int nret,numOutBytes,fifoTx,fifoRx;
	char buff[256];
	int f9pInterface;
	double x,y,z;
	

	lcdDispButtonText( 2, WHITE, "Save", "Rate", "NextPage", false );
	int lineStart = 0;
	int numInfoLines = 4;
	bool updated = false;
	int i = 0;
	while(1){
		if ( i == mNumReceivers ) break;
		int n = i + 1;
		bool isMovingBase = mGpsRelPos[i].ubxDone && mGpsRelPos[i].flags & 0x20;	// 0x20:isMoving flag
		if ( mGpsData[i].ubxDone && ! isMovingBase &&
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
		//if ( mGpsRelPos[i].ubxDone && i > 0 && mOpeMode == MODE_MOVING_BASE ){
		if ( mGpsRelPos[i].ubxDone && mGpsRelPos[i].flags & 0x20 ){	// 0x20:isMoving flag
			lcdTextColor24( mColor[ mDisplayType * 4 + i ] );
			int fix = 0;
			if (mGpsRelPos[i].quality == 4 ) fix = 2;
			else if (mGpsRelPos[i].quality == 5 ) fix = 1;
			lineStart = i * 4;
			double elevation = atan2( -mGpsRelPos[i].down, mGpsRelPos[i].length ) * RAD2DEG;
			lcdDispText( lineStart++, "LENGTH%d=%.3lfm     \r\n", n, mGpsRelPos[i].length );
			lcdDispText( lineStart++, "HEADING%d=%.1lfdeg    \r\n", n, mGpsRelPos[i].heading);
			lcdDispText( lineStart++, "ELEVATION%d=%.1lfdeg   \r\n", n, elevation );
			lcdDispText( lineStart++, "FIX%d=%d  SPS%d=%d \r\n", n, fix, n, mSolutionRate);
			lcdTextColor( WHITE );
		}
		
		i++;
	}
//	lcdDispText( lineStart++, "%.3fm Az=%.2f El=%.2f\r\n", dist, azimuth,elevation);
	lcdTextColor24( mColor[ mDisplayType * 4 + 3 ] );
	lcdDispText( mNumReceivers * 4, "File save:%d  Saved=%d\r\n", mFileSaving, mFileSaved);
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
	
//			lcdDispText( mNumReceivers * 4 + 1, "NTRIP=%d bps\r\n", bps );
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
		if ( mSdSaveReady ){
			mFileSaving = ! mFileSaving;
			dbgPrintf( "File saving=%d\r\n", mFileSaving );
			if ( mFileSaving ) {
				mFileSaved = 0;
				mQueueFileErrorCount = 0;
				mQueueFileErrorCountLast = 0;
				mUartSaveIndex = 0;
			}
			else {
				for( int i=0; i < RECEIVER_MAX; i++ ) strcpy( mSaveFileName[i], "" );
			}
			mRunInfo.saving = mFileSaving;
			saveRunInfo( &mRunInfo );
		}
		else {
			lcdClear();
			lcdDispText( 3, "> Can't save to SD card." );
			lcdDispText( 10, ">>> Press any button" );
			waitButton();
		}
	}

	int buttonB = buttonBPressed();
	if ( buttonB ){
		int pitch = 1;
		if ( buttonB == 2 ) pitch = 5;
		if ( mSolutionRate == 1 && pitch == 5 ) mSolutionRate = pitch;
		else {
			mSolutionRate += pitch;
			if ( mSolutionRate > 20 ) mSolutionRate = 1;
		}
		gpsSetSolutionRate( mSolutionRate );
		mRunInfo.solutionRate = mSolutionRate;
		saveRunInfo( &mRunInfo );
	}
	
//	dbgPrintf("Gyro x=%.3f y=%.3f z=%.3f Accel x=%.3f y=%.3f z=%.3f\r\n",
//				mGyroX,mGyroY,mGyroZ,mAccelX,mAccelY,mAccelZ );

}

int gpsSetSolutionRate( int rate )
{
	int nret;
	if ( rate < 1 ) return -1;
	for( int i=0; i < RECEIVER_MAX; i++ ){
		if ( mReceiverType[i] ){
			nret = gpsSetMeasurementRate( mReceiverType[i], 1000 / rate );
			if ( nret < 0 ){
				dbgPrintf( "gpsSetMesurementRate error\r\n" );
			}
		}
	}

	return 0;
}


int dispMapMain()
{
	int nret;
	bool reload = false;
	bool mapSaved = false;

	String mapDir = mRootDir + "/map";
	if ( ! SD.exists( mapDir ) ){
		if ( ! SD.mkdir( mapDir ) ) {
			dbgPrintf( "mkdir error: %s\r\n", mapDir );
			lcdClear();
			lcdDispAndWaitButton( 3, ">>> Can't create map directory" );
			nextPage();
			return -1;
		}
	}

	if ( ! mLcdMap ){
		double centerLat = mGpsData[0].lat;
		double centerLon = mGpsData[0].lon;
		if ( mOpeMode == MODE_BASE ){
			centerLat = mBasePosition.lat;
			centerLon = mBasePosition.lon;
		}
		reload = true;

		if ( mMapData.path.length() == 0 ) {
			mMapData.path = mapDir + "/map.jpg";
			mMapData.mapType = MAP_ROAD;
			mMapData.zoomLevel = 18;
			mMapData.centerLat = centerLat;
			mMapData.centerLon = centerLon;
			mMapData.width = mLcdWidth;
			mMapData.height = mLcdHeight;
		}
		else {
			double diffLat = mMapData.centerLat - centerLat;
			double diffLon = mMapData.centerLon - centerLon;
			double delta = 0.000001;		// 10cm以内なら保存データを使用
			if ( fabs( diffLat )  < delta && fabs( diffLon ) < delta ) {
				reload = false;
				mapSaved = true;
			}
			else reload = true;
		}
	}		
	if ( reload ){
j1:		nret = getMap( &mMapData );
		if ( nret == 0 ){
			mapSaved = true;
		}
		else if ( nret == -2 ){
			lcdClear();
			lcdDispAndWaitButton( 3, ">>> No google key in SD card" );
			nextPage();
			return -1;
		}
	}
	if ( mapSaved ){
		dispMap( &mMapData );
		mLcdMap = &mMapData;
		msecMapDisp = millis();
	}
	if ( ! mLcdMap ) return -2;

	lcdDispButtonText( 2, WHITE, "ZoomOut", "ZoomIn", "NextPage", false );
	if ( millis() - msecMapDisp > 500 ){
//		if ( M5.BtnA.pressedFor( 1000 ) ){
		int buttonA = buttonAPressed();
		if ( buttonA == 2 ){
			int mapType = mLcdMap->mapType;
			mapType++;
			if ( mapType == MAP_MAX ) mapType = 0;
			mLcdMap->mapType = mapType;
			M5.update();
			goto j1;
		}
		int zoomLevel = mLcdMap->zoomLevel;
//		if ( M5.BtnA.wasReleased() ){
		if ( buttonA == 1 ){
			zoomLevel--;
			if ( zoomLevel < 1 ) zoomLevel = 1;
			else {
				mLcdMap->zoomLevel = zoomLevel;
				goto j1;
			}
		}
//		if ( M5.BtnB.wasReleased() ){
		if ( buttonBPressed() ){
			zoomLevel++;
			if ( zoomLevel > 21 ) zoomLevel = 21;
			else {
				mLcdMap->zoomLevel = zoomLevel;
				goto j1;
			}
		}
	}
	
	return 0;
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
	
	lcdDispText( lineNum++, "AP IP address:%u.%u.%u.%u", 
					mSoftApIp[0], mSoftApIp[1], mSoftApIp[2], mSoftApIp[3] );
	lcdDispText( lineNum++, "Server port:%d", mServerPort );
	lcdDispText( lineNum++, "AP ssid:%s", mSoftApSsid );
	lcdDispText( lineNum++, "AP passwd:%s", mSoftApPassword );
	lcdDispText( lineNum++, "Heap: %d KB", esp_get_free_heap_size() / 1000 );
	lcdDispText( lineNum++, "SD card: %lu MB", (int)(mSdTotalBytes / 1E6) );

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
					dbgPrintf("IF(-1:uart positive:i2c address)=%d  ", f9pInterface);
					if ( nret == -2 || nret == -3 ) {
						dbgPrintf( "ubx check sum error  lastError=%d\r\n", Wire.lastError() );
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

			// UBX PVT(Position Velocity Time)のデコード
			if ( msgClass == 0x01 && msgId == 0x07  ){ // NAV-PVT
				
				if ( i == 0 ) {
					memcpy( &mGpsDataLast, &mGpsData[0], sizeof(mGpsDataLast) );
				}
				
				nret = ubxDecodeNavPvt( &mUbxStatus[i], &mGpsData[i] );
				mUbxStatus[i].statusNum = 0;
				
/*				
int nn = mUartWriteIndex - mUartReadIndex;
	struct stDateTime time;
	getDateTime( &time );

if ( i == 0 ){
	dbgPrintf( "%d %d-%d-%d %d:%d:%d:%d\r\n", nn,time.year,time.month,time.day,time.hour,time.minute,time.second,time.msec);
}
*/
				
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
			
				// 受信データの保存、配信、表示
				if ( mGpsData[i].ubxDone ){
					updated = true;
					mGpsData[i].receiverNum = i;
					
					// Track
					if ( i == 0 ){
						struct stGpsData *pGpsData = &mGpsData[0]; 
						struct stGpsTrack *pGpsTrack = &mGpsTrack[ mTrackIndex ];
						pGpsTrack->time = gpsDate2UnixTime( pGpsData );
						pGpsTrack->lat = pGpsData->lat;
						pGpsTrack->lon = pGpsData->lon;
						mTrackIndex++;
						if ( mTrackIndex == TRACK_MAX ) mTrackIndex = 0;
					}
					
					// NMEAデータの作成
					numOutBytes = setNmeaData(i);

					// NMEAデータの配信

					// シリアルポートからの配信
					if ( i == 0 && mDebugPort != DEBUG_PH ) phUartWriteBytes( (byte *)mSaveBuff, numOutBytes );

					// SDカードへ保存
					if ( mFileSaving && mQueueFileSave && mSaveFormat == SAVE_NMEA){
						BaseType_t qret;
						qret = xQueueSend( mQueueFileSave, &mGpsData[i], 0 );
						if ( qret != pdPASS ){
							mQueueFileErrorCount++;
//							dbgPrintf("File save queue error. num queue=%d\r\n", uxQueueMessagesWait//ing(mQueueFileSave));
						} 
						if ( i == 0 ) {
							if ( mGpsData[i].second == 0 ){
								int cnt = mQueueFileErrorCount - mQueueFileErrorCountLast;
								if ( cnt > 0 ){
									dbgPrintf("File save queue error count=%d/min Total=%d(%dmin)\r\n", cnt, mQueueFileErrorCount, (millis() - mStartMillis) / 60000);
									mQueueFileErrorCountLast = mQueueFileErrorCount;
								}
							}
						}
					//	nret = sdSave( i );
					}
					
					// TCP Serverとしての配信
					if ( mWifiServer && i == 0){
						int residue = numOutBytes;
						char* pRead = mSaveBuff;
						while( residue > 0 ){
							int n = residue;
							if ( mServerWriteIndex + n > SERVER_BUFF_MAX )
								n = SERVER_BUFF_MAX - mServerWriteIndex;
							memcpy( mServerBuff + mServerWriteIndex, pRead, n );
							mServerWriteIndex += n;
							pRead += n;
							if ( mServerWriteIndex >= SERVER_BUFF_MAX ) mServerWriteIndex = 0;
							residue -= n;
						}
					}
					
					// TCP Clientとしての配信
					if ( mAgribusReady && i == 0 ){
						if ( mAgribusClient->connected() ){
							nret = mAgribusClient->write( (byte *)mSaveBuff, numOutBytes );
						}
						else{
							mAgribusClient->stop();
							nret = mAgribusClient->connect( mAgribusIp, mAgribusPort );
							if ( nret < 0 ) dbgPrintf("agribus connect error nret=%d\r\n",nret);
						}
					}
					
					// 相対位置の計算
					if ( ( mOpeMode != MODE_MOVING_BASE) && ( i == 1 ) ){
						struct stGpsData *pd0 = &mGpsData[0];
						struct stGpsData *pd1 = &mGpsData[1];
						float secDiff = ( pd1->hour * 3600 + pd1->minute * 60 + pd1->second + pd1->msec / 1000.0 ) - ( pd0->hour * 3600 + pd0->minute * 60 + pd0->second + pd0->msec / 1000.0 );
						int msecDiff = secDiff * 1000;
//dbgPrintf("msecDiff=%d\r\n",msecDiff);
						bool sameTimes = abs( msecDiff ) < ( 500 / mSolutionRate ) ;
						if ( sameTimes ){
							getPosVector( &mGpsData[0], &mGpsData[1], &x, &y, &z );
							float dist = sqrt( x * x + y * y + z * z );
							float azimuth = -1;
							float elevation = -1;
							if ( dist > 0.02 ){
								azimuth = atan2( y, x ) * RAD2DEG;
								if ( azimuth < 0 ) azimuth += 360;
								elevation = (atan2( z, sqrt(x * x + y * y) )) * RAD2DEG;
							}
//dbgPrintf("dist=%f azimuth=%f elevation=%f\r\n",dist,azimuth,elevation );
						}
					}

					//mGpsData[i].ubxDone = false;
				}
			}
			
			// UBX RELPOSNED(Relative Positioning Information in NED frame)のデコード
			if ( msgClass == 0x01 && msgId == 0x3C  ){ // NAV-RELPOSNED
				memset( &mGpsRelPos[i], 0, sizeof( stGpsRelPos ) );
				nret = ubxDecodeNavRelPosNed( &mUbxStatus[i], &mGpsRelPos[i] );
			}
			
			// UBX RAWX or SFRBX
			if ( msgClass == 0x02 ) {
				if ( msgId == 0x15 || msgId == 0x13  ){ // RAWX or SFRBX
				}
			}


			mUbxStatus[i].statusNum = 0;
		}

		mWifiConnected =  (WiFi.status() == WL_CONNECTED) ;
		
		//BT
		if ( BT_ENABLE ){
			if ( numOutBytes > 0 ){
				mBluetooth.print( mSaveBuff );
			}
		}
	}
	else vTaskDelay(5);
	goto j1;
}


// SDカードにデータを保存するスレッド
//
// Core:0で実行した場合、10Hzで保存しようとすると、GPS受信機からの
// データ取得でエラーが起きる。
// Core:1で実行する
//
void taskSdSave(void* param)
{
	int nret;
	BaseType_t qret;
	struct stGpsData gpsData;

	while(1){
		// NMEAデータはキュー経由
		qret = xQueueReceive( mQueueFileSave, &gpsData, 0);
		if ( qret == pdPASS ){
			nret = sdSave( gpsData.receiverNum, &gpsData );
		}
		
		// NMEA以外はUARTバッファの内容をそのまま保存
		if ( mFileSaving && mSaveFormat != SAVE_NMEA ){
			int numBytes = mUartWriteIndex - mUartSaveIndex;
			if ( numBytes > 0 && numBytes < 500 ) continue;
			if ( numBytes < 0 ) numBytes = UART_BUFF_MAX - mUartSaveIndex;
			nret = sdSave( 0, mUartBuff + mUartSaveIndex, numBytes );
			if ( nret > 0 ) {
				mUartSaveIndex += nret;
				if ( mUartSaveIndex >= UART_BUFF_MAX ) mUartSaveIndex = 0;
			}
		}
		
		vTaskDelay(10);
	}
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
	int reconnectCount = 0;
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
//Serial.printf("ch=%x\r\n",ch);
					if ( ch < 0 ) break;
					mBaseRecvBuff[ mBaseRecvIdx++ ] = (char) ch;
					if ( mBaseRecvIdx == BASE_RECV_BUFF_MAX || ch == '\n' ){
						nret = gpsWrite( IF_UART, mBaseRecvBuff, mBaseRecvIdx );
						if ( nret > 0 ) mBaseRecvCount += nret;
						mBaseRecvIdx = 0;
					}
					mBaseRecvLastMillis = millis();
				}
//if (numRecvBytes) dbgPrintf("    read end\r\n" );
			}
			else if ( mBaseSrc.type == BASE_TYPE_UART ) {
				if ( mPhUartBuff.available() > 64 ){
					numRecvBytes = mPhUartBuff.read( (byte*) mBaseRecvBuff, BASE_RECV_BUFF_MAX );
					if ( numRecvBytes ){
						nret = gpsWrite( IF_UART, mBaseRecvBuff, numRecvBytes );
						if ( nret > 0 ) mBaseRecvCount += nret;
						mBaseRecvLastMillis = millis();
					}
				}
			}
		}
		if ( mBaseSrc.protocol == PROTO_NTRIP_GGA ){	// ggaPeriod毎にGGAをキャスターに送る
			if ( millis() - msecLastTime  >= ( mBaseSrc.ggaPeriod * 1000 ) ){
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
						nret = wifiReconnect( mSsid, mPassword, 10 );
						if ( nret == -1 ){
							dbgPrintf( "Wifi reconnect failed count=%d\r\n", ++reconnectCount );
						}
					}
					if ( WiFi.status() != WL_CONNECTED) continue;
				}

				mBaseRecvClient->stop();
				vTaskDelay(200);
				
				nret = mBaseRecvClient->connect( mBaseSrc.address, mBaseSrc.port );
				if ( nret == 0 ){
					if ( mBaseSrc.protocol == PROTO_NTRIP || mBaseSrc.protocol == PROTO_NTRIP_GGA ) {
						dbgPrintf("ntrip reconnecting\r\n");
						nret = mBaseRecvClient->ntripRequest( mBaseSrc.mountPoint, 
													mBaseSrc.user, mBaseSrc.password, mGGAAddress );
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

// Wifi再接続
//
// 戻り値＝ 0:正常終了
//         -1:タイムアウト
//
int wifiReconnect( char* ssid, char* password, int secTimeout)
{
	WiFi.begin( ssid, password );
	unsigned long msecLastTime = millis();
	int msecTimeout = secTimeout * 1000;
	while (1) {
		if ( WiFi.status() == WL_CONNECTED)  return 0;
		vTaskDelay(200);
		if ( millis() - msecLastTime > msecTimeout ){
			return -1;
		}
	}
}


// NTRIPキャスターへ基準局データを送信するスレッド
//
void taskNtripSend(void* param)
{
	int nret;
	unsigned long msecLastTime = 0;
	unsigned long msecReconnectStop = 0;
	int reconnectCount = 0;
	while(1)
	{
		vTaskDelay(1);
				
		if ( mBaseSendReady && mBaseSendClient ){
			if ( ! mBaseSendClient->connected() ){
dbgPrintf("Ntrip reconnect start\r\n");
				if ( mNetAccess == USE_WIFI ){
					if ( WiFi.status() != WL_CONNECTED) {
						nret = wifiReconnect( mSsid, mPassword, 10 );
						if ( nret == -1 ){
							dbgPrintf( "Wifi reconnect failed count=%d\r\n", reconnectCount );
						}
					}
					if ( WiFi.status() != WL_CONNECTED) continue;
				}

				if ( reconnectCount > 10 ){	// 再接続が何回も続くとキャスタがIPをロックするので
											// 再開まで１時間待つ
					if ( msecReconnectStop ){
						if ( millis() - msecReconnectStop > 3600 * 1000 ){
							reconnectCount = 0;
							msecReconnectStop = 0;
						}
					}
					else msecReconnectStop = millis();
					vTaskDelay(10000);
					continue;
				}
				nret = mBaseSendClient->ntripServerConnect( mCaster.address, mCaster.port, 
											mCaster.mountPoint, mCaster.password );
				if ( nret != 0 ){
					vTaskDelay(5000);
					continue;
				}
				reconnectCount++;
dbgPrintf("reconnect = %d\r\n",reconnectCount);
			}
			
			int numSendBytes = mNtripSendWriteIdx - mNtripSendReadIdx;
			if ( numSendBytes < 0 ) numSendBytes = NTRIP_SEND_BUFF_MAX - mNtripSendReadIdx;
			if ( ! numSendBytes ) continue;
		
			nret = mBaseSendClient->write( mNtripSendBuff + mNtripSendReadIdx, numSendBytes, 1000 );
			if ( nret != numSendBytes ){
				dbgPrintf("Ntrip send error nret=%d\r\n", nret );
				if ( nret == -1 ){	// timeoutの時は再接続
					mBaseSendClient->stop();
					dbgPrintf("mBaseSendClient stopped\r\n");
				}
			}
			else{
				reconnectCount = 0;
				mNtripSendCounter++;
				if ( mNtripSendCounter < 0 ) mNtripSendCounter = 0;
dbgPrintf("Ntrip writed n=%d\r\n",numSendBytes);
			}
			mNtripSendReadIdx += numSendBytes;
			if ( mNtripSendReadIdx >= NTRIP_SEND_BUFF_MAX ) mNtripSendReadIdx = 0;
		}	
		else {
			vTaskDelay(100);
			continue;
		}
	}
}

// 920MHzモデムにより基準局データを受信するスレッド
//
void taskModem920Recv(void* param)
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

// 920MHzモデムにより基準局データを送信するスレッド
//
void taskModem920Send(void* param)
{
	int nret;
	unsigned long msecLastTime = 0;
	while(1)
	{
		vTaskDelay(10);
				
		if ( m920SendReady ){
			int numSendBytes = m920SendWriteIdx - m920SendReadIdx;
			if ( numSendBytes < 0 ) numSendBytes = MODEM920_SEND_BUFF_MAX - m920SendReadIdx;
			if ( ! numSendBytes ) continue;
//dbgPrintf("920 send start\r\n");
//unsigned long msecStart=millis();
			nret = mFep01.send( 2, (char*) m920SendBuff + m920SendReadIdx, numSendBytes, true );
			if ( nret != numSendBytes ){
				dbgPrintf("Modem 920MHz send error in=%d out=%d\r\n", mRtcmIndex, nret);
			}
			m920SendReadIdx += numSendBytes;
			if ( m920SendReadIdx >= MODEM920_SEND_BUFF_MAX ) m920SendReadIdx = 0;
//int msec = millis() - msecStart;
//int bps = numSendBytes * 1000 *8 / msec;
//dbgPrintf("920 send bytes=%d  time=%dmsec speed=%dbps\r\n",numSendBytes,msec,bps);

		}	
		else {
			vTaskDelay(100);
			continue;
		}
	}
}


// GPS受信機からデータを取得するスレッド
//
// １．メイン受信機からのデータ
//     UART（Serial1)からデータを取得し、
//     移動局の場合は位置データをUART用バッファに格納する。
//     基準局（Static or Moving Base)の場合は補正データをサブ受信機にUART（Serial1)で送る
//     基準局の場合、Wifiまたはモデムで宛先に補正データを送る
// ２．サブ受信機からのデータ
//     I2Cからの位置データを取得し、I2C用バッファに格納する
//     位置データを取得するタイミングはメイン受信機からのデータを受信した時に同期させている。
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
			// 基準局の場合
			if ( (mOpeMode == MODE_BASE && mRtcmXferReady) || mOpeMode == MODE_MOVING_BASE ){
				
				// データの取得
				// msecTimeoutの時間内に受信したデータをバッファに格納
				unsigned long msecStart = millis();
				int msecTimeout = 100;	// 100msec程度ないとPH UART送信時にエラーとなる
				if ( mOpeMode == MODE_MOVING_BASE ) msecTimeout = 60;
				mRtcmIndex = 0;
				while( (millis() - msecStart) < msecTimeout ){
					if ( numBytesToRead > 0 ){
						if ( numBytesToRead + mRtcmIndex > RTCM_BUFF_MAX ) 
									numBytesToRead = RTCM_BUFF_MAX - mRtcmIndex;
						numSavedBytes = Serial1.readBytes( mRtcmBuff + mRtcmIndex, numBytesToRead );
						mRtcmIndex += numSavedBytes;
						if ( mRtcmIndex >= RTCM_BUFF_MAX ) break;
					}
					vTaskDelay(1);
					numBytesToRead = Serial1.available();
				}
				
				mServerCounter++;

				// TCP転送用バッファにコピー
				if ( mWifiServer ){
					int residue = mRtcmIndex;
					byte* pRead = mRtcmBuff;
					while( residue > 0 ){
						int n = residue;
						if ( mServerWriteIndex + n > SERVER_BUFF_MAX )
							n = SERVER_BUFF_MAX - mServerWriteIndex;
						memcpy( mServerBuff + mServerWriteIndex, pRead, n );
						mServerWriteIndex += n;
						pRead += n;
						if ( mServerWriteIndex >= SERVER_BUFF_MAX ) mServerWriteIndex = 0;
						residue -= n;
					}
				}
				
				// NTRIP送信用バッファにコピー
				if ( mBaseSendReady ){
					ringBuffCopy( mRtcmBuff, mRtcmIndex, mNtripSendBuff, 
													&mNtripSendWriteIdx, NTRIP_SEND_BUFF_MAX );
				}
				
				// Moving Baseの場合はPVTを取り出しバッファに格納
				if ( mOpeMode == MODE_MOVING_BASE ){
					int decodedBytes;
					nret = ubxBuffDecode( mRtcmBuff, mRtcmIndex, &mUbxStatus[0], &decodedBytes );
				}
				
				// 920MHzモデムが接続されている場合は920MHz送信用バッファにコピー
				if ( mNetAccess == USE_MODEM_920 && m920SendReady ){
					ringBuffCopy( mRtcmBuff, mRtcmIndex, m920SendBuff, 
													&m920SendWriteIdx, MODEM920_SEND_BUFF_MAX );
					m920SendCount++;
dbgPrintf("920 write bytes=%d\r\n",mRtcmIndex);
//					nret = mFep01.send( 2, (char*) mRtcmBuff, mRtcmIndex );
//					m920SendCount++;
//					if ( nret != mRtcmIndex ){
//						dbgPrintf("Modem 920MHz send error in=%d out=%d\r\n", mRtcmIndex, nret);
//					}
				}

				// 補正データをサブ受信機に配信
				if ( mNumI2cReceivers ){
					nret = gpsWrite( IF_UART, (char *)mRtcmBuff, mRtcmIndex );
					if ( nret != mRtcmIndex ){
						dbgPrintf("gpsWrite() error in=%d out=%d\r\n", mRtcmIndex, nret);
					}
				}
				
				// シリアルポートからの配信
				if ( mDebugPort != DEBUG_PH )phUartWriteBytes( mRtcmBuff, mRtcmIndex );

			}
			else {	// 移動局の場合
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
		else {
			if ( mI2cUartSyncMode ) continue;
		}
		

		// I2C
		if ( mNumI2cReceivers == 0 ) continue;

		// サブ受信機からのデータをI2Cで取得する
		bool done[RECEIVER_MAX];
		memset( done, 0, sizeof(done) );
		unsigned long msecStart = millis();
		while(1){
			if ( millis() - msecStart > 50 ) break;
			for( int i=1; i < RECEIVER_MAX; i++ ){
				if ( done[i] ) continue;
				int i2cAddress = mReceiverType[i];
				if ( i2cAddress ){
					numBytesToRead = f9pI2cNumBytes( i2cAddress );
					if ( numBytesToRead == 0 ) continue;
					while ( numBytesToRead > 0 && numBytesToRead != 65535 ){
						savePointer = (byte*) mI2cBuff[i] + mI2cWriteIndex[i];
						numSavedBytes = f9pI2cReadBytes( i2cAddress, numBytesToRead, savePointer,
									 I2C_BUFF_MAX - mI2cWriteIndex[i] );
						if ( numSavedBytes ){
							mI2cWriteIndex[i] += numSavedBytes;
							if ( mI2cWriteIndex[i] == I2C_BUFF_MAX ) mI2cWriteIndex[i] = 0;
							else if ( mI2cWriteIndex[i] > I2C_BUFF_MAX ){
								dbgPrintf( "taskGetSensorData() I2C error\r\n" );
								mI2cWriteIndex[i] = 0;
							}
							numBytesToRead -= numSavedBytes;
						}
					}
					done[i] = true;
				}
				else done[i] = true;
			}
			bool allDone = true;
			for( int i=1; i < RECEIVER_MAX; i++ ){
				if ( ! done[i] ) {
					allDone = false;
					break;
				}
			}
			if ( allDone ) break;
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

// データ配信用TCPサーバとしてクライアントにデータを送信するスレッド
//
void taskWifiServer(void* param)
{
	int nret,senderAddress;
	char buff[256];
	int errorCount[ SERVER_CLIENT_MAX ];
	
	while(1){
		vTaskDelay(50);

		WiFiClient newClient = mWifiServer->available();
		if ( newClient ){
			for ( int i=0; i < SERVER_CLIENT_MAX; i++ ){
				if ( ! mServerClient[i] ) {
					mServerClient[i] = new WiFiClient( newClient );
					errorCount[i] = 0;
					break;
				}
			}
		}
		
		int numSendBytes = mServerWriteIndex - mServerReadIndex;
		if ( numSendBytes < 0 ) numSendBytes = SERVER_BUFF_MAX - mServerReadIndex;
		if ( ! numSendBytes ) continue;
		
		for( int i=0; i < SERVER_CLIENT_MAX; i++ ){
			if ( ! mServerClient[i] ) continue;
			if ( mServerClient[i]->connected() ){
				char* p = mServerBuff + mServerReadIndex;
				nret = mServerClient[i]->write( p, numSendBytes );			
				if ( nret == numSendBytes ){
					errorCount[i] = 0;
				}
				else {
					errorCount[i]++;
					if ( errorCount[i] == 5 ){
						mServerClient[i]->stop();
						mServerClient[i] = NULL;
					}
				}
			}
			else {
				mServerClient[i] = NULL;
			}
		}
		mServerReadIndex += numSendBytes;
		if ( mServerReadIndex >= SERVER_BUFF_MAX ) mServerReadIndex = 0;
	}
}

// スレッドtaskI2c()(i2c.cpp)から呼び出される。
//
// 呼び出しの周期は約1msec毎。但し、ZED-F9Pからのデータ読み出しが
// 入るとその分長くなる。
//
// この関数の実行時間はできるだけ短くしないと、ZED-F9Pのデータ読み出し
// でタイムアウトエラーが起きるので注意する。
//
// Wire(I2C)を使っているライブラリの関数を呼び出す際はこの関数内に
// 記述する。
// そうしないと、taskI2c()のスレッドでもWire関数を呼び出しているので
// I2Cバスへのアクセスが競合し、データ化けする。
//
// ライブラリの関数でなく、このソース内でWire(I2C)を使う場合はi2c.cppにある
// int i2cReadBytes( int i2cAddress, int registerNum, int bytesToRead, byte* buffer )
// int i2cWriteBytes( int i2cAddress, int registerNum, int bytesToWrite, byte* buffer )
// を使えばどこに置いても良い。
//

void executeWireFunction()
{
	if ( ! mSetupDone ) return;
	
	// IMU
	if ( mUseImu ){
		if ( millis() - mImuLastRead > 5 ){
			mImuLastRead = millis();
			M5.IMU.getGyroData( &mGyroX, &mGyroY, &mGyroZ );
			M5.IMU.getAccelData( &mAccelX, &mAccelY, &mAccelZ );
		}
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
	strcpy( mSoftApSsid, mReceiverName );
	nret = WiFi.softAP( mSoftApSsid, mSoftApPassword );
	if ( ! nret ){
		dbgPrintf("softAP() failed\r\n");
		lcdDispText( 7, "Can't use soft AP." );
	}
	else dbgPrintf("softAP() ok.\r\n");
	mSoftApIp = WiFi.softAPIP();

	// ネットワーク選択開始
	if ( mRunMode == RUN_UI ){
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
	}
	else {
		nret = mRunInfo.netAccess;
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
			int wifiApNum = wifiConnect();
			if ( wifiApNum ) {
				mWifiConnected = 1;
				lcdDispText( 5, "> Wifi connected" );
				dbgPrintf("WiFi connected\r\n");
				Serial.print("local IP address: ");
				mWifiLocalIp = IPAddress( WiFi.localIP() );
				dbgPrintf( (mWifiLocalIp.toString()).c_str() );
				mNetAccess = USE_WIFI;
				mRunInfo.wifiAp = wifiApNum;
			}
			else {
				lcdDispText( 5, "> Wifi not connected" );
				dbgPrintf("WiFi not connected !!!\r\n");
			}
		}
		
		
	}
	else if ( nret == 2 ){ // Modem 3G(Soracom)
		while(1){
			nret = modemInit();
			delay(1000);	// wait threadModemInit() done

			if ( nret == 0 ){
				lcdDispText( 8, "> Modem 3G connected" );
				mNetAccess = USE_MODEM_3G;
				mModemLocalIp = IPAddress( mModem.localIP() );
				break;
			}
			else if ( mRunMode == RUN_UI ){
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
	}
	else if ( nret == 3 ){ // Modem 920MHz FEP01
		lcdDispText( 3, "Modem 920MHz initializing" );
		int baudrate = mFep01.init( 0 );
		if ( baudrate > 0 ){
			lcdDispText( 5, "> Modem 920MHz connected" );

			int newBaudrate = 115200;
			nret = mFep01.setBaudrate( newBaudrate );
			if ( nret < 0 ){
				lcdDispText( 4, "> Can't change baudrate to %d", newBaudrate );
			}
			else{
				lcdDispText( 4, "> baudrate : %d", newBaudrate );
			}
			lcdDispText( 5, "> Modem 920MHz connected" );
			mNetAccess = USE_MODEM_920;
		}
		else {
			lcdDispText( 5, "> Modem 920MHz not connected (%d)", baudrate );
		}
	}
	else return 0;
	
	if ( mRunMode == RUN_UI ){
		lcdDispText( 10, ">>> Press any button" );
		waitButton();
	}
	return 0;
}


int readIniFile( IniFile *iniFile )
{
	char section[32],buff[256];
	int nret;
	
	if ( ! iniFile ) return -1;
	
	// 名称
	nret = iniFile->readValue( "receiver", "name", buff, 256 );
	if ( nret > 0 ){
		int n = sizeof( mReceiverName ) - 1;
		strncpy( mReceiverName, buff, n );
		mReceiverName[ n ] = '\0';
	}
	
	// CPU Clock
	int cpuMHz = iniFile->readInt( "receiver", "cpuclock", 0 );
	if ( cpuMHz > 0 ){
		setCpuFrequencyMhz( cpuMHz );
		mCpuFreqMHz = ESP.getCpuFreqMHz();
	}
	
	
	// Wifi
	mNumWifi = 0;
	memset( mWifiList, 0, sizeof( mWifiList ) );
	for( int i=0; i < WIFI_MAX; i++ ){
		if ( i == 0 ) strcpy( section, "wifi" );
		else sprintf( section, "wifi%d", i );
		nret = iniFile->readValue( section, "ssid", buff, 256 );
		if ( nret < 0 || strlen(buff) > 32) continue;
		strcpy( mWifiList[ mNumWifi ].ssid, buff );

		nret = iniFile->readValue( section, "password", buff, 256 );
		if ( nret < 0 || strlen(buff) > 64) continue;
		strcpy( mWifiList[ mNumWifi ].password, buff );
		
		nret = iniFile->readValue( section, "ip", buff, 256 );
		if ( nret >= 0 && strlen(buff) <= 15) iniGetIp( buff, mWifiList[ mNumWifi ].ip );

		nret = iniFile->readValue( section, "dns", buff, 256 );
		if ( nret >= 0 && strlen(buff) <= 15) iniGetIp( buff, mWifiList[ mNumWifi ].dns );
		
		mNumWifi++;
		if ( mNumWifi == WIFI_MAX ) break;
	}
	
	// Base source 基準局データ取得先
	memset( mBaseSrcList, 0, sizeof( mBaseSrcList ) );
	mNumBaseSrc = 0;
	for( int i=0; i < BASE_SRC_MAX; i++ ){
		if ( i == 0 ) {
			mBaseSrcList[0].type = BASE_TYPE_UART;
			mBaseSrcList[0].protocol = PROTO_NONE;
			mNumBaseSrc++;
			continue;
		}
		
		sprintf( section, "source%d", i );
		mBaseSrcList[ mNumBaseSrc ].type = BASE_TYPE_TCP;

		nret = iniFile->readValue( section, "address", buff, 256 );
		if ( nret < 0 || strlen(buff) > 64) continue;
		strcpy( mBaseSrcList[ mNumBaseSrc ].address, buff );
		bool isNtrip = strstr( buff, "rtk2go" ) || strstr( buff, "ales" );

		nret = iniFile->readValue( section, "protocol", buff, 256 );
		if ( strlen(buff) > 16) continue;
		if ( nret < 0 ){
			if ( isNtrip ) mBaseSrcList[ mNumBaseSrc ].protocol = PROTO_NTRIP;
		}
		else {
			strToLower( buff );
			if ( strstr( buff, "ntrip" ) ) mBaseSrcList[ mNumBaseSrc ].protocol = PROTO_NTRIP;
		}

		int gga = iniFile->readInt( section, "gga", 0 );
		mBaseSrcList[ mNumBaseSrc ].ggaPeriod = gga;

		int port = iniFile->readInt( section, "port", 2101 );
		mBaseSrcList[ mNumBaseSrc ].port = port;

		nret = iniFile->readValue( section, "mount", buff, 256 );
		if ( nret > 0 && strlen(buff) < 32) {
			strcpy( mBaseSrcList[ mNumBaseSrc ].mountPoint, buff );
		}
		
		nret = iniFile->readValue( section, "user", buff, 256 );
		if ( nret > 0 && strlen(buff) < 32){
			strcpy( mBaseSrcList[ mNumBaseSrc ].user, buff );
		}

		nret = iniFile->readValue( section, "password", buff, 256 );
		if ( nret > 0 && strlen(buff) < 32) {
			strcpy( mBaseSrcList[ mNumBaseSrc ].password, buff );
		}

		mNumBaseSrc++;
		if ( mNumBaseSrc == BASE_SRC_MAX ) break;
	}
	
	// TCP client
	strcpy( section, "client" );
	nret = iniFile->readValue( section, "ip", buff, 256 );
	if ( nret > 0 && strlen(buff) < 16){
		strcpy( mAgribusIp, buff );
		mAgribusPort = iniFile->readInt( section, "port", 51020 );
	}
	else strcpy( mAgribusIp, "" );

		
	// Server 基準局または移動局データ配信ポート
	mServerPort = iniFile->readInt( "server", "port", mServerPort );

	// NTRIP Caster 基準局データ送信先
	strcpy( section, "caster" );
	mCaster.valid = false;
	for( int i=0; i < 1; i++ ){
		nret = iniFile->readValue( section, "address", buff, 256 );
		if ( nret < 0 || strlen(buff) > 64) continue;
		strcpy( mCaster.address, buff );

		int port = iniFile->readInt( section, "port", 2101 );
		mCaster.port = port;

		nret = iniFile->readValue( section, "mount", buff, 256 );
		if ( nret < 0 || strlen(buff) > 32) continue;
		strcpy( mCaster.mountPoint, buff );
		
		nret = iniFile->readValue( section, "password", buff, 256 );
		if ( strlen(buff) > 32 || nret < 0 ) strcpy( buff, "" );
		strcpy( mCaster.password, buff );
		
		mCaster.valid = true;
	}
	
	// 基準局座標
	strcpy( section, "base" );
	mNumBasePos = 0;
	for( int i=0; i < BASE_POS_MAX; i++ ){
		sprintf( section, "base%d", i );

		nret = iniFile->readValue( section, "name", buff, 256 );
		buff[16] = '\0';
		strcpy( mBasePosList[ mNumBasePos ].name, buff );

		double lat = iniFile->readDouble( section, "lat", 100 );
		if ( lat < -90 || lat > 90 ) continue;
		mBasePosList[ mNumBasePos ].lat = lat;

		double lon = iniFile->readDouble( section, "lon", 200 );
		if ( lon < -180 || lon > 180 ) continue;
		mBasePosList[ mNumBasePos ].lon = lon;

		double height = iniFile->readDouble( section, "height", -100 );
		if ( height < 0 ) continue;
		mBasePosList[ mNumBasePos ].height = height;

		mNumBasePos++;
	}
	
	// Survey-in パラメータ
	strcpy( section, "surveyin" );
	mSurveyinSec = iniFile->readInt( section, "period", mSurveyinSec );
	mSurveyinAccuracy = iniFile->readDouble( section, "accuracy", mSurveyinAccuracy );

	// Google key
	nret = iniFile->readValue( "google", "key", buff, 256 );
	if ( nret > 0 ){
		mGoogleKey = (char*) malloc( strlen(buff) + 1 );
		if ( mGoogleKey ) strcpy( mGoogleKey, buff );
	}

	// JST-PHコネクタ
	mPhUartBaudrate = iniFile->readInt( "jstph", "baudrate", mPhUartBaudrate );
	
	// デバグポート
	mDebugPort = iniFile->readInt( "debug", "port", 0 );
	
	// 液晶のタイプ 　色の濃いタイプ：0　 色の薄いタイプ：1
	mDisplayType = iniFile->readInt( "display", "type", 0 );
	
	// 異常リブート時の動作モード
	mRebootMode = iniFile->readInt( "reboot", "mode", 0 );

	if ( mIniFile )	mIniFile->close();
	return 0;
}

// dot表記のIPアドレスを４バイトの配列に変換する
//
int iniGetIp( char* buff, byte* ip )
{
	int ips[4];
	int nret = sscanf( buff, "%d.%d.%d.%d", ips, ips + 1, ips + 2, ips + 3 );
	if ( nret != 4 ) return -1;
	for( int i=0; i < 4; i++ ) ip[i] = (byte)ips[i];
	return 0;
}

void strToLower( char* buff )
{
	while( *buff ){
		 *buff = tolower( *buff );
		 buff++;
	}
}

// Wifiのアクセスポイントを選択した後、接続する
//
// 戻り値＝ 1以上: 接続済　値はINIファイルのWIFI接続先番号([wifi1]..[wifi3])
//          0: 未接続
//
int wifiConnect()
{
	char buff[256];
	int nret,color;
	
	int idx = 0;
	int command = 0;
	if ( mRunMode == RUN_UI ) {
	j1:	lcdClear();
		lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
		lcdDispText( 3, ">>> Do you use Wifi ?" );
		int useWifi = waitButton( 1, 1, 0, YES, NO, 0 );
		lcdClear();
		if ( ! useWifi  ) return 0;
	
		lcdClear();
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
	}
	else {
		if ( mRunInfo.wifiAp == 0 ) command = 3;
		else {
			command = 2;
			idx = mRunInfo.wifiAp - 1;
			if ( idx < 0 ) idx = 0;
		}
	}

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
				if ( mRunMode == RUN_UI ) {
					lcdClear();
					lcdTextColor( WHITE );
					lcdDispText( 3, "Continue connecting ?" );
					lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
					int continu = waitButton( 1, 1, 0, YES, NO, 0 );
					if ( ! continu  ) {
						goto j1;
					}
				}
				lcdClear();
				msecStart = millis();
				lcdDispText( 3, "Connecting to %s", ssid );
			}
		}
	}
	else if ( command == 3 ){	// Cancel
		return 0;
	}
	lcdTextColor( WHITE );
	lcdClear();
	
	if ( wifiConnected ) return idx + 1;
	else return 0;
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





// Ntrip caster/serverを選択する
//
// 戻り値＝ 0以外: 選択済（mBaseSrcにパラメータ設定済）
//                 -1:UART 1以上：INIファイルの基準局データ取得先番号（[source1]..[source9])
//          0: 選択無し
//
//int ntripSelect( int opeMode, double lat, double lon )
int baseSrcSelect( double lat, double lon )
{
	char buff[256];
	int nret,color;
	
	if ( mRunMode == RUN_UI ) {
		lcdClear();
		lcdDispText( 3, ">>> Do you connect to base station ?" );
		lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
		int yes = waitButton( 1, 1, 0, YES, NO, 0 );
		lcdClear();
		if ( ! yes  ) return 0;
	}

	dbgPrintf( "mNetAccess = %d\r\n", mNetAccess );
	if ( mNetAccess == USE_WIFI ) mBaseRecvClient = new TcpClient( &mWifiClient );
	else if ( mNetAccess == USE_MODEM_3G ) {
		mBaseRecvClient = new TcpClient( &mGsmClient );
	}
	mBaseRecvClient->setAgentName( mNtripClientName );
	
	if ( mNumBaseSrc ) nret = baseSrcSelect_iniFile();
	if ( nret != 0 ) return nret;

	if ( mOpeMode == MODE_BASE ) return 0;

	lcdDispText( 3, ">>> Do you use rtk2go.com ?" );
	lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
	nret = waitButton( 1, 1, 0, YES, NO, 0 );
	if ( nret == NO  ) return 0;

	if ( mRunMode == RUN_UI ) {
		lcdClear();
		lcdDispText( 3, "Connecting to rtk2go.com" );
	
		nret = ntripSelect_caster( "rtk2go.com", 2101, lat, lon );
		lcdClear();
		if ( nret < 0 ){
			lcdDispText( 3, "Ntrip not selected : nret = %d", nret );
			waitButton();
			return 0;
		}
	}

	return 1;
}


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


// Ntripキャスターにアクセスし、マウントポイントを選択する
// 
// 戻り値＝ 0:正常終了
//         負数：エラー
//
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

// Iniファイルに記載された基準局データ取得先を選択する
//
// 戻り値＝ 0以外: 選択済（mBaseSrcにパラメータ設定済）
//                 -1:UART 1以上：INIファイルの基準局データ取得先番号（[source1]..[source9])
//          0: 選択無し
//
//int ntripSelect_iniFile()
int baseSrcSelect_iniFile()
{
	char buff[256];
	int color;

	int idx = 0;
	int command = 0;
	
	if ( mRunMode == RUN_UI ) {
		lcdClear();
		lcdDispButtonText( 2, WHITE, "Next", "Ok", "Cancel" );
		lcdDispText( 1, ">>> Select base station" );
		int lineOffset = 3;
		for( int i=0; i < mNumBaseSrc; i++ ){
			if ( i == 0 ) sprintf( buff, "0 UART(PH connector)" );
			else sprintf( buff, "%d %s %s", i, mBaseSrcList[i].address, mBaseSrcList[i].mountPoint );
			if ( i == 0 ) color = GREEN;
			else color = WHITE;
			lcdTextColor( color );
			lcdDispText( i + lineOffset, buff );
		}
		while(1){
			command = waitButton( 1, 1, 1, 1, 2, 3 );
			if ( command == 1 ){	// next item
				if ( mNumBaseSrc == 1 ) continue;
				if ( idx == 0 ) sprintf( buff, "0 UART(PH connector)" );
				else sprintf( buff, "%d %s %s", idx, mBaseSrcList[idx].address, mBaseSrcList[idx].mountPoint );
				lcdTextColor( WHITE );
				lcdDispText( idx + lineOffset, buff );

				idx++;
				if ( idx == mNumBaseSrc ) idx = 0;
				if ( idx == 0 ) sprintf( buff, "0 UART(PH connector)" );
				else sprintf( buff, "%d %s %s", idx, mBaseSrcList[idx].address, mBaseSrcList[idx].mountPoint );
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
	}
	else {
		command = 2;
		idx = mRunInfo.baseSrc - 1;
		if ( idx < 0 ) idx = 0;
	}

	int wifiConnected = 0;
	unsigned long msecStart = millis();
	if ( command == 2 ){
		memcpy( &mBaseSrc, &mBaseSrcList[idx], sizeof(mBaseSrc) );
		mBaseSrc.valid = true;
		char* address = mBaseSrcList[idx].address;
		if ( strstr( address, "rtk2go" ) ) mBaseSrc.protocol = PROTO_NTRIP;
		if ( strstr( address, "ales" ) ) mBaseSrc.protocol = PROTO_NTRIP_GGA;
		if ( mBaseSrc.ggaPeriod > 0 && mBaseSrc.protocol == PROTO_NTRIP ) mBaseSrc.protocol = PROTO_NTRIP_GGA;
		if ( idx == 0 ) return -1;
		return idx + 1;
	}
	else mBaseSrc.valid = false;

	return 0;
}

// 基準局の座標値を選択する
//
// 戻り値＝ 1: 選択済
//          0: 未選択
//
int basePosSelect()
{
	char buff[256];
	int nret,color;
	if ( mNumBasePos == 0 ) return 0;
	
	lcdClear();
	int command = CANCEL;
	int idx = 0;
	if ( mNumBasePos == 1 ) command = OK;
	else {
		lcdDispButtonText( 2, WHITE, "Next", "Ok", "SurveyIn" );
		lcdDispText( 1, ">>> Select position of base station" );
		int lineOffset = 3;
		for( int i=0; i < mNumBasePos; i++ ){
			sprintf( buff, "%d %s %.8f", i+1, mBasePosList[i].name, mBasePosList[i].lat );
			if ( i == 0 ) color = GREEN;
			else color = WHITE;
			lcdTextColor( color );
			lcdDispText( i + lineOffset, buff );
		}
		while(1){
			command = waitButton( 1, 1, 1, NEXT, OK, CANCEL );
			if ( command == NEXT ){	// next item
				if ( mNumBasePos == 1 ) continue;
				sprintf( buff, "%d %s %.8f", idx+1, mBasePosList[idx].name, mBasePosList[idx].lat );
				lcdTextColor( WHITE );
				lcdDispText( idx + lineOffset, buff );

				idx++;
				if ( idx == mNumBasePos ) idx = 0;
				sprintf( buff, "%d %s %.8f", idx+1, mBasePosList[idx].name, mBasePosList[idx].lat );
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
	}
	if ( command == CANCEL ) return 0;
	
	memcpy( (void*) &mBasePosition, (void*) &mBasePosList[ idx ], sizeof( mBasePosition ) );

	lcdClear();
	lcdDispText( 3, ">>> Do you use this position ?" );
	lcdDispText( 5, "name: %s", mBasePosition.name );
	lcdDispText( 6, "lat: %.8f", mBasePosition.lat );
	lcdDispText( 7, "lon: %.8f", mBasePosition.lon );
	lcdDispText( 8, "height: %.3f", mBasePosition.height );
	lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
	int yes = waitButton( 1, 1, 0, YES, NO, 0 );
	if ( yes ) {
		mBasePosition.valid = true;
		return 1;
	}

	return 0;
}


// GPS受信機を基準局として初期化
//
// 基準局にできるのはシリアル接続の受信機のみ
//
// mode= 1: 座標を自動設定。degLat,degLon,mHeightは使用しない。
//       2: 座標を数値で指定。
//
// 戻り値= 0:正常終了
//        負数:エラー
//        -100: survey-inが中断され、再開できないのでZED-F9Pをリセットする必要がある。
//
int baseInit( int mode, double degLat, double degLon, double mHeight )
{
	int nret;
	double x,y,z,lat,lon,height;
	int secObservation = 0;
	float mrAccuracy = 0;
	struct stUbxStatus ubxStatus;
	memset( &ubxStatus, 0, sizeof(ubxStatus) );

	// output disable:NMEA  enable:UBX,RTCM3
	nret = gpsSetUartPort( IF_UART, 1, mGpsUartBaudrate, 1, 1, 1, 1, 0, 1 );
	if ( nret < 0 ) return -1;

	if ( mode == 1 ) degLat = degLon = mHeight = 0;

j1:
	nret = gpsSetBaseCoordinate( IF_UART, mode, degLat, degLon, mHeight,
					mSurveyinSec, mSurveyinAccuracy );
	if ( nret < 0 ) return -2;

	int retCode = 0;
	if ( mode == 1 ){ // surveyin
		nret = gpsSetMessageRate( IF_UART, 0x01, 0x3b, 1 );	// output Survey-in data
		if ( nret < 0 ) return -3;
j2:		lcdDispText( 1, "*** Survey-in status ***" );
		lcdDispButtonText( 2, WHITE, "", "Cancel", "" );
		x = y = z = 0;
		buttonPressedReset();
		while(1){
			if ( buttonPressed() == BUTTON_B ){	// canceled
				lcdClear();
				lcdDispText( 3, "> Use current mean value ?" );
				gpsEcef2Llh( x, y, z, &lat, &lon, &height );
				lcdDispText( 5, "lat: %.8f", lat );
				lcdDispText( 6, "lon: %.8f", lon );
				lcdDispText( 7, "height: %.3f", height );

				lcdDispText( 9, "accuracy: %.3fm", mrAccuracy );
				
				lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
				int yes = waitButton( 1, 1, 0, YES, NO, 0 );
				lcdClear();
				if ( yes ) {
					nret = 1;
					break;
				}
				else goto j2;
			}
			nret = ubxDecode( IF_UART, &ubxStatus );
			if  (nret < 0 ) {
				continue;
			}
			if ( ubxStatus.statusNum == 10 ){
//dbgPrintf("msgClass=%x id=%x\r\n",ubxStatus.msgClass, ubxStatus.msgId );
				if ( ubxStatus.msgClass != 0x01 || ubxStatus.msgId != 0x3b ){
					ubxStatus.statusNum = 0;
					continue;
				}
			}
			else {
				delay(1);
				continue;
			}
			
			nret = ubxDecodeNavSvin( &ubxStatus, &secObservation, &mrAccuracy, &x, &y, &z );
			if ( nret < 0 ) {
				dbgPrintf("NavSvin error nret=%d\r\n",nret);
				return -100;
			}
			dbgPrintf("Survey in period=%dsec accuracy=%.3fm\r\n", secObservation, mrAccuracy );
			lcdDispText( 3, "Observation period : %dsec", secObservation );
			lcdDispText( 4, "Accuracy : %.3fm          ", mrAccuracy );
			if ( nret == 1 ) break;
			delay(500);
			
		}
		if ( nret == 1 ){
			gpsEcef2Llh( x, y, z, &mBasePosition.lat, &mBasePosition.lon, &mBasePosition.height );
			strcpy( mBasePosition.name, "survey-in" );
			mBasePosition.valid = true;
			lcdDispText( 6, "Survey-in normally end." );
			lcdDispText( 8, "Press any button." );
			lcdDispButtonText( 2, WHITE, "", "", "" );
			waitButton();
		}
		nret = gpsSetMessageRate( IF_UART, 0x01, 0x3b, 0 );  // stop Survey-in data
		lcdClear();
	}
	
	// RTCM message
	nret = gpsSetRtcmMessage( IF_UART );
	if ( nret < 0 ) retCode = -11;
	
	return retCode;
}

// Moving Base mode
int movingBaseInit()
{
	int nret;
	// input ubx:1 nmea:0 rtcm:0
	// output ubx:1 nmea:0 rtcm:1
	nret = gpsSetUartPort( IF_UART, 1, mGpsUartBaudrate, 1, 0, 0, 1, 0, 1 );
	if ( nret < 0 ) return -1;

	// RTCM message
dbgPrintf("moving base 0xF5 0x4d\r\n");
	nret = gpsSetMessageRate( IF_UART, 0xF5, 0x4D, 1 );	// RTCM3.3 1077 GPS MSM7
	if ( nret < 0 ) return -2;
	nret = gpsSetMessageRate( IF_UART, 0xF5, 0x57, 1 );	// RTCM3.3 1087 GLONASS MSM7
	if ( nret < 0 ) return -3;
	nret = gpsSetMessageRate( IF_UART, 0xF5, 0x61, 1 );	// RTCM3.3 1097 Galileo MSM7
	if ( nret < 0 ) return -4;
	nret = gpsSetMessageRate( IF_UART, 0xF5, 0x7F, 1 );	// RTCM3.3 1127 BeiDou MSM7
	if ( nret < 0 ) return -5;
	nret = gpsSetMessageRate( IF_UART, 0xF5, 0xE6, 1 );	// RTCM3.3 1230 GLONASS code-phase biases
	if ( nret < 0 ) return -6;
	nret = gpsSetMessageRate( IF_UART, 0xF5, 0xFE, 1 );	// RTCM3.3 4072-0 Reference station PVT
	if ( nret < 0 ) return -7;
	nret = gpsSetMessageRate( IF_UART, 0xF5, 0xFD, 1 );	// RTCM3.3 4072-1 Additional reference station information
	if ( nret < 0 ) return -8;

	for( int i=1; i < mNumReceivers; i ++ ){
		int i2cAddress = mReceiverType[i];
		if ( i2cAddress ){
			for( int j=0; j < 3; j++ ){
				nret = gpsSetMessageRate( i2cAddress, 0x01, 0x3c, 1 );	// NAV-RELPOSNED
				if ( nret == 0 ) break;
			}
		}

	}

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
	if ( i == 3 && mRunMode == RUN_UI ){
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

	// I2CでアクセスするGPS受信機の数え上げ
	//
j1:	M5.Lcd.println("ZED-F9P I2C");
	mNumI2cReceivers = RECEIVER_MAX - 1;	// taskGetSensorDataで必要
	int numI2cReceivers = 0;
	for( int i=0; i < RECEIVER_MAX; i++ ){
		if ( i == 0 ) mReceiverType[i] = IF_UART;
		else { // I2C
			int i2cAddress = 0x42 + i;
			mReceiverType[i] = i2cAddress;	// taskGetSensorDataで必要
			nret = gpsSetMessageRate( i2cAddress, 0x01, 0x07, 1 );	// NAV-PVT
			if ( nret < 0 ) {
				mReceiverType[i] = 0;
			}
			else {
				M5.Lcd.print("i2c address = ");
				M5.Lcd.print(i2cAddress, HEX);
				M5.Lcd.println(" connected");
				numI2cReceivers++;
			}
		}
		strcpy( mSaveFileName[i], "" );
	}
	
	mNumI2cReceivers = numI2cReceivers;
	if ( mNumI2cReceivers == 2 ) lcdTextSize( 2 );
	dbgPrintf("Number of I2C ZED-F9P = %d\r\n", mNumI2cReceivers );
	
	// 台数確認
	mNumReceivers = mNumI2cReceivers + 1;
	if ( mRunMode == RUN_UI ){
		lcdClear();
		lcdDispText( 3, ">>> Number of M5F9P module is %d ?", mNumReceivers );
		lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
		int yes = waitButton( 1, 1, 0, YES, NO, 0 );
		lcdClear();
		if ( ! yes ) {
			lcdDispText( 3, "Checking the number of modules" );
			goto j1;
		}
	}
	
	// 各受信機の移動局として初期化
	for( int i=0; i < RECEIVER_MAX; i++ )
	{
		int f9pInterface = mReceiverType[i];
		if ( f9pInterface == 0 ) continue;

		// output UBX only
		if ( f9pInterface == IF_UART )
			// input ubx:1 nmea:1 rtcm:1   output ubx:1 nmea:0 rtcm:0
			nret = gpsSetUartPort( f9pInterface, 1, mGpsUartBaudrate, 1, 1, 1, 1, 0, 0 );
		else {	// I2C : UART RTCM only input
			// input ubx:0 nmea:0 rtcm:1   output ubx:1 nmea:0 rtcm:0
			nret = gpsSetUartPort( f9pInterface, 1, mGpsUartBaudrate, 0, 0, 1, 1, 0, 0 );

			// input ubx:1 nmea:0 rtcm:1   output ubx:1 nmea:0 rtcm:0
			nret = gpsSetI2cPort( f9pInterface, 1, 0, 1, 1, 0, 0 );
		}
		if ( nret < 0 ){
			return -1;
		}
		
		// output message
		nret = gpsSetMessageRate( f9pInterface, 0x01, 0x07, 1 );	// NAV-PVT
		nret = gpsSetMessageRate( f9pInterface, 0x01, 0x14, 1 );	// NAV-HPPOSLLH
		nret = gpsSetMessageRate( f9pInterface, 0x01, 0x3c, 1 );	// NAV-RELPOSNED

		// output rate = 1Hz
		nret = gpsSetMeasurementRate( mReceiverType[i], 1000 );
		if ( nret < 0 ) return -2;
	}
}

int gpsRawInit()
{
	int nret;
	// 各受信機のRAWデータ出力設定
	for( int i=0; i < RECEIVER_MAX; i++ )
	{
		int f9pInterface = mReceiverType[i];
		if ( f9pInterface == 0 ) continue;

		// output UBX only
		if ( f9pInterface == IF_UART )
			// input ubx:1 nmea:1 rtcm:1   output ubx:1 nmea:0 rtcm:0
			nret = gpsSetUartPort( f9pInterface, 1, mGpsUartBaudrate, 1, 1, 1, 1, 0, 1 );
		else 	// I2C : UART RTCM only input
			// input ubx:0 nmea:0 rtcm:1   output ubx:1 nmea:0 rtcm:0
			nret = gpsSetUartPort( f9pInterface, 1, mGpsUartBaudrate, 0, 0, 0, 1, 0, 1 );
		if ( nret < 0 ){
			return -1;
		}
		
		// enable message
		if ( mSaveFormat == SAVE_RAW ){
			nret = gpsSetMessageRate( f9pInterface, 0x02, 0x15, 1 );	// RXM-RAWX
			nret = gpsSetMessageRate( f9pInterface, 0x02, 0x13, 1 );	// RXM-SFRBX
		}
		else if ( mSaveFormat == SAVE_RTCM ){
			nret = gpsSetRtcmMessage( f9pInterface );
		}

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


// TCP サーバに接続する。主にAgribus用。
//
int connectTcpServer()
{
	int nret;
	
	// Agribus
	mAgribusReady = false;
	if ( strlen( mAgribusIp ) > 0 && mOpeMode == MODE_ROVER ){
		int yes = YES;
		if ( mRunMode == RUN_UI ){
			lcdClear();
			lcdDispText( 3, ">>> Do you connect to TCP server ?" );
			lcdDispText( 5, " IP= %s", mAgribusIp );
			lcdDispText( 6, " PORT=%d", mAgribusPort );
			lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
			int yes = waitButton( 1, 1, 0, YES, NO, 0 );
			lcdClear();
		}
			
		if ( yes ){
			if ( mNetAccess == USE_WIFI ) mAgribusClient = new WiFiClient();
			if ( mAgribusClient ){
				while(1){
					lcdDispText( 3, "Connecting to AgriBus server (%s).",mAgribusIp );
					bool connected = mAgribusClient->connect( mAgribusIp, mAgribusPort );
					if ( connected ){
						mAgribusReady = true;
						break;
					}
					else if ( mRunMode == RUN_UI ){
						lcdClear();
						lcdDispText( 3, "> Can't connect to AgriBus Server." );
						lcdDispText( 8, ">>> Do you retry ?" );
						lcdDispButtonText( 2, WHITE, "exit", "retry", "" );
						nret = waitButton( 1, 1, 0, 1, 2, 0 );
						lcdClear();
						if ( nret == 1 ) {
							break;
						}
					}
					else delay(1000);
				}
			}
		}
	}
	
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
dbgPrintf("connectBaseSource: nret=%d  src=%s\r\n",nret, mBaseSrc.address );
		if ( nret == 0 ){
			connected = true;
			break;
		}
		else if ( mRunMode == RUN_UI ) {
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
												mBaseSrc.user, mBaseSrc.password, mGGAAddress );
				if ( nret == 0 ) {
					connected = true;
					break;
				}
				else {
					dbgPrintf("Requesting MountPoint is failed. nret=%d !!!\r\n", nret);
				}
			}
			if ( connected ) break;
			if ( mRunMode == RUN_UI ) {
				lcdDispText( 5, "> Can't connect." );
				lcdDispButtonText( 2, WHITE, "Retry", "Cancel", "" );
				lcdDispText( 8, ">>> Do you retry ?" );
				int nret = waitButton( 1, 1, 0, 1, 0, 0 );
				if ( ! nret ) break;
			}
		}
		if ( connected ){
			dbgPrintf("Ntrip OK. host=%s  mount point=%s\r\n", mBaseSrc.address, mBaseSrc.mountPoint);
		}
		lcdClear();
	}
	if ( connected ) return 0;
	else return -4;
}

// NTRIP ServerとしてCasterに接続する
//
// 戻り値＝ 1: 接続完了
//          0: 未接続
//         -1: パスワードエラー
//
int connectNtripAsServer()
{
	int nret,yes;
	
	lcdClear();
	if ( ! mCaster.valid ) {
		return 0;
	}
	else {
		if ( mRunMode == RUN_UI ){
			lcdDispText( 3, "> Do you connect to NTRIP caster ?" );
			lcdDispText( 5, "   caster: %s", mCaster.address );
			lcdDispText( 6, "   mount point: %s", mCaster.mountPoint );
			lcdDispButtonText( 2, WHITE, "Yes", "No", "" );
			yes = waitButton( 1, 1, 0, YES, NO, 0 );
			lcdClear();
		}
		else {
			yes = mRunInfo.baseSend;
		}
		if ( !yes ) return 0;
	}
	
	if ( mNetAccess == USE_WIFI ) mBaseSendClient = new TcpClient( &mWifiClient );
	else if ( mNetAccess == USE_MODEM_3G ) mBaseSendClient = new TcpClient( &mGsmClient );
	else return -2;
	
	mBaseSendClient->setAgentName( mNtripServerName );
	dbgPrintf("Connecting to NTRIP caster\r\n");
	bool connected = false;
	if ( mNetAccess > 0 ){
		while(1){
			lcdClear();
			lcdDispText( 3, "Connecting to NTRIP caster");
			for( int i=0; i < 3; i++ ){
				nret = mBaseSendClient->ntripServerConnect( mCaster.address, mCaster.port, 
											mCaster.mountPoint, mCaster.password );

				if ( nret == 0 ) {
					connected = true;
					break;
				}
				else if ( nret >= -2 ) continue;	// 接続、転送エラーなのでリトライ
				else if ( nret == -3 ){	// パスワードエラー
					if ( mRunMode == RUN_UI ){
						lcdDispAndWaitButton( 3, "Can't connect to NTRIP caster. ( Bad Password )" );
					}
					return -1;
				}
				else break;
			}
			if ( connected ) break;
			if ( mRunMode == RUN_UI ){
				lcdDispText( 5, "> Can't connect.(%d)", nret );
				lcdDispButtonText( 2, WHITE, "Retry", "Cancel", "" );
				lcdDispText( 8, ">>> Do you retry ?" );
				int nret = waitButton( 1, 1, 0, 1, 0, 0 );
				if ( nret == 0 ) break;
			}
		}
		if ( connected ){
			dbgPrintf("Ntrip OK. host=%s  mount point=%s\r\n", mCaster.address, mCaster.mountPoint);
			if ( mRunMode == RUN_UI ){
				lcdClear();
				lcdDispText( 3, "> Connected to NTRIP caster" );
				lcdDispText( 5, "  Caster: %s\r\n  Mountpoint:%s", mCaster.address, mCaster.mountPoint );
				lcdDispText( 10, ">>> Press any button" );
				waitButton();
			}
		}
		lcdClear();
	}
	if ( connected ) return 1;
	else return 0;
}

// ************************************************************
//                         時刻
// ************************************************************

// GPSの1PPSパルスによる割り込み
//
long mLastPPSIsrMicro;

void IRAM_ATTR gpsPPSIsr()
{
	// ノイズ除去
	long micro = esp_timer_get_time();
	if ( micro - mLastPPSIsrMicro < 900 * 1000 ) return;
	mLastPPSIsrMicro = micro;
	
	// タイムスタンプ
	//mGpsPPSMicros = micros();
	mGpsPPSMicros = micro;
	
	// 現在時刻の更新
	struct stDateTime time;
	unsigned long pvtMicros;
	int nret = getPvtTime( &time, &pvtMicros );
	if ( nret < 0 ) mUnixTime++;
	else {
		unsigned long unixTimePvt = dateTime2UnixTime( &time );

		if ( mGpsPPSMicros - pvtMicros < 1000 ) {
			mUnixTime = unixTimePvt + 1;
		}
		else mUnixTime++;
	}
}

void IRAM_ATTR gpsPPSIsr2()
{
	struct stDateTime time;
	unsigned long nret = getDateTime( &time );
//	dbgPrintf( "1pps %d  %d:%d:%d:%d\r\n", nret, time.hour,time.minute,time.second,time.msec);
}

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

// 現在時刻を取得する
//
// dateTime: msecまで有効
//
// 戻り値＝1970-1-1 0:0:0からの秒数
//         0:現在時刻が得られない
//
time_t getDateTime( struct stDateTime* dateTime )
{
	memset( (void*) dateTime, 0, sizeof(stDateTime) );
	int usec = micros() - mGpsPPSMicros;
	if ( usec > 1000000 || usec < 0 ) return 0;
	unsigned long unixTime = mUnixTime;
	if ( usec == 1000000 ) {
		unixTime++;
		usec = 0;
	}
	unixTime2DateTime( unixTime, dateTime );
	
	dateTime->msec = usec / 1000;
	return unixTime;
}

// ************************************************************
//                         Web server
// ************************************************************
//
//  このWeb server機能に関しては下記の著作権者のコードを改変して使用しています。
//
//  SDWebServer - Example WebServer with SD Card backend for esp8266

//  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
//  This file is part of the WebServer library for Arduino environment.

//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.

int webServerInit()
{
	mWebServer = new WebServer( mHttpPort );
	if ( ! mWebServer ) {
		dbgPrintf( "Web server not available\r\n" );
		return -1;
	}
	mWebServer->begin();
//	mWebServer->on( "/", webHandleOnRoot );
	mWebServer->on( "/list", HTTP_GET, webHandleOnList);
	mWebServer->on( "/edit", HTTP_GET, webHandleOnEdit);
	mWebServer->on( "/edit", HTTP_POST, webHandleOnEditPost);
	mWebServer->on( "/map", webHandleOnMap );
	mWebServer->on( "/map.htm", webHandleOnMap );
	mWebServer->onNotFound( webHandleNotFound );
	return 0;
}

// ブラウザでのINIファイルの編集
//
void webHandleOnEdit()
{
	String str = "<html lang=\"ja\"> \
				<head>\
				<meta charset=\"UTF-8\">  \
				</head> \
				<body> ";
	str += "<form method=\"POST\" action=\"/edit\"> \
		  <textarea name=\"msg\" cols=80 rows=50 style=\"overflow:auto;\">";

	File file = SD.open( mIniPath );
	if ( ! file ){
		str = "Ini file not found.<br>";
		mWebServer->send( 404, "text/html", str );
		return;
	}
	str += file.readString();
	file.close();

	str += " </textarea></div><br> \
			 <input type=\"submit\" value=\"Send\">\
			</form> \
			</body></html>";

	mWebServer->send( 200, "text/html", str );
}

// ブラウザからの、INIファイルの更新要求
//
void webHandleOnEditPost()
{
	String msg = mWebServer->arg( "msg" );
	
	int numBytes = msg.length();
	bool ok = true;
	if ( numBytes > 0 ){
		File file = SD.open( mIniPath, FILE_WRITE );
		int nret = file.write( (byte*)msg.c_str(), numBytes );
		if ( nret != numBytes ){
			ok = false;
			dbgPrintf("write error nret=%d\r\n",nret );
		}
		file.close();
		mWebServer->send(200, "text/plain", "File updated.");
	}
	else mWebServer->send(500, "text/plain", "File not updated.");
}


// ブラウザへの地図表示
//
void webHandleOnMap()
{
	int nret;
	char buff[256];
	char latStr[16],lonStr[16],heightStr[8];

	if ( ! mWebServer ) return;

	if ( mOpeMode == MODE_BASE ){
		sprintf( latStr, "%.8f", mBasePosition.lat );
		sprintf( lonStr, "%.8f", mBasePosition.lon );
		sprintf( heightStr, "%.3f", mBasePosition.height );
		
	}
	else {
		double lat = mGpsData[0].lat;
		sprintf( latStr, "%.8f", lat );
		double lon = mGpsData[0].lon;
		sprintf( lonStr, "%.8f", lon );
		double height = mGpsData[0].height;
		sprintf( heightStr, "%.3f", height );
	}
	
	sprintf( buff, "Lat: %s (deg)<br>Lon: %s (deg)<br>Height: %s (m)<br>",
								latStr, lonStr, heightStr );
	if ( ! mSdTotalBytes ) {
		mWebServer->send( 200, "text/html", buff );
		return;
	}

	String path = mHtmlDir + "/map.htm";
	int numPoints = 100;
	if ( numPoints > TRACK_MAX ) numPoints = TRACK_MAX;
	String contents = "";
	String track = "[";
	int j = mTrackIndex - numPoints;
	if ( j < 0 ) j += TRACK_MAX;
	if ( mGpsTrack[j].time == 0 ) j = 0;
	for( int i = 0; i < numPoints; i++ ){
		char buff2[64];
		if ( i > 0 ) strcpy( buff2, "," );
		else strcpy( buff2, "" );
		if ( i >= TRACK_MAX ) break;
		sprintf( buff2 + strlen( buff2 ) , "{lat:%.8f,lng:%.8f}", 
								mGpsTrack[j].lat, mGpsTrack[j].lon );
		track += String( buff2 );
		if ( j == mTrackIndex - 1 ) break;
		j++;
		if ( j == TRACK_MAX ) j = 0;
	}
	track += "]";

	File file = SD.open( path.c_str() );
	if ( ! file ){
		contents += String( path.c_str() ) + " not found.<br>";
		contents += String( buff );
		mWebServer->send( 200, "text/html", contents );
		return;
	}
	while( file.available() ){
		String line = file.readStringUntil( '\n' );
		line.replace( "$KEY", String( mGoogleKey ) );
		line.replace( "$LAT", String( latStr ) );
		line.replace( "$LON", String( lonStr ) );
		line.replace( "$HGT", String( heightStr ) );
		line.replace( "$TRK", track );
		contents += line + "\n";
	}	
	file.close();
	mWebServer->send( 200, "text/html", contents );
}

//
// responseCode= 400: Bad Request
//               404: Not Found
//         
//
void webSendError( int responseCode, String msg )
{
	if ( ! mWebServer ) return;
	mWebServer->send( responseCode, "text/plain", msg + "\r\n" );
}

// ブラウザを使用してのログファイルダウンロード
//
void webHandleOnList()
{
	if ( ! mWebServer || ! mSdTotalBytes ) return;
	WebServer* server = mWebServer;

	if ( ! server->hasArg("dir") ) {
		webSendError( 400, "Directory not spcified" );
		return;
	}
	
	String path = server->arg("dir");
	String absPath = mRootDir;
	if ( path != "/" )  absPath += path;
	if (path != "/" && !SD.exists((char *)absPath.c_str())) {
		webSendError( 404, "Directory " + path + " not found" );
		return;
	}
	File dir = SD.open((char *)absPath.c_str());
	if ( ! dir.isDirectory() ) {
		webSendError( 400, path + " is not directory" );
		return;
	}

	dir.rewindDirectory();
	server->setContentLength( CONTENT_LENGTH_UNKNOWN );

	String listDir = "/list?dir=";
	String output = "<html><head></head><body>";
	output += "<h2>" + path + "</h2><br>\r\n";
	String path1,path2;
	splitPath( path, &path1, &path2 );
	output += "<a href=\"" + listDir + path1 + "\">Parent Directory</a><br>\r\n";

	while(1){
		File entry = dir.openNextFile();
		if ( ! entry ) break;

		bool isDirectory = entry.isDirectory();
		output +="<li><a href=\"";
		if ( isDirectory ) output += listDir;
		String relPath = entry.name();

		relPath.replace( mRootDir, "" );
		if ( ! isDirectory ) output += "$";
		output += relPath;
		output += "\"";
		output += ">";
   
		splitPath( relPath, &path1, &path2 );
		if ( isDirectory ){
			output += "<img src=\"/image/folder24.png\">" + path2 + "</a>";
		}
		else {
			output += path2 +  "</a>   ";
			if ( path2 != "" ){
				int size = entry.size();
				if ( size >= 1000 ) output += String( (int)(size / 1000.0 + 0.5) ) + "KB";
				else output += "1KB";
			}
		}
		output += "</li>\r\n";

		entry.close();
	}
	output += "</body></html>";
	server->send( 200, "text/html", output );
	dir.close();
}


void splitPath( String path, String *path1, String *path2 )
{
	if ( path == "/" ){
		*path1 = "/";
		*path2 = "";
		return;
	}
	if ( path.endsWith( "/" ) )  path = path.substring( 0, path.length() - 2 );
	int pos = path.lastIndexOf( '/' );
	if ( pos > 0 ){
		*path1 = path.substring( 0, pos );
		*path2 = path.substring( pos + 1 );
	}
	else if ( pos == 0 ){
		*path1 = "/";
		*path2 = path.substring( 1 );
	}
	else {
		*path1 = path;
		*path2 = "";
	}
}

void webHandleNotFound() {

	if ( ! mWebServer || ! mSdTotalBytes ) return;

	if ( loadFromSdCard( mWebServer, mWebServer->uri() ) ) return;

	WebServer* server = mWebServer;
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server->uri();
	message += "\nMethod: ";
	message += (server->method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server->args();
	message += "\n";
	for (uint8_t i = 0; i < server->args(); i++) {
		message += " NAME:" + server->argName(i) + "\n VALUE:" + server->arg(i) + "\n";
	}
	server->send(404, "text/plain", message);
}

bool loadFromSdCard( WebServer* server, String path )
{
	String dataType = "text/plain";
	if (path.endsWith("/")) {
		path += "index.htm";
	}

	if (path.endsWith(".src")) {
		path = path.substring(0, path.lastIndexOf("."));
	} else if (path.endsWith(".htm")) {
		dataType = "text/html";
	} else if (path.endsWith(".css")) {
		dataType = "text/css";
	} else if (path.endsWith(".js")) {
		dataType = "application/javascript";
	} else if (path.endsWith(".png")) {
		dataType = "image/png";
	} else if (path.endsWith(".gif")) {
		dataType = "image/gif";
	} else if (path.endsWith(".jpg")) {
		dataType = "image/jpeg";
	} else if (path.endsWith(".ico")) {
		dataType = "image/x-icon";
	} else if (path.endsWith(".xml")) {
		dataType = "text/xml";
	} else if (path.endsWith(".pdf")) {
		dataType = "application/pdf";
	} else if (path.endsWith(".zip")) {
		dataType = "application/zip";
	}

	bool isDownload = path.startsWith( "/$" );
	if ( isDownload ){
		path = mRootDir + path.substring( 2 );
		dataType = "application/octet-stream";
	}
	else if ( ! path.startsWith( "/" ) ) {
		path = mHtmlDir + "/" + path;
	}
	else path = mHtmlDir + path;
	File dataFile = SD.open( path.c_str() );
	if (dataFile.isDirectory()) {
		path += "/index.htm";
		dataType = "text/html";
		dataFile = SD.open(path.c_str());
	}
	if (!dataFile) {
		return false;
	}
 	if ( server->hasArg( "download" ) ){
		dataType = "application/octet-stream";
	}
			
	if (server->streamFile(dataFile, dataType) != dataFile.size()) {
		dbgPrintf("Sent less data than expected!\r\n");
	}

	dataFile.close();
	return true;
}

// ************************************************************
//                         ファイル
// ************************************************************

int sdSave(int receiverNum, struct stGpsData *pGpsData)
{
	int nret;
	char buff[256];
	int i = receiverNum;
	if ( strlen( mSaveFileName[i] ) == 0 ){
		int year = mGpsData[i].year;
		int month = mGpsData[i].month;
		int day = mGpsData[i].day;
		int hour = mGpsData[i].hour;
		int minute = mGpsData[i].minute;
		int sec = mGpsData[i].second;
		nret = setFilePath( i, year, month, day, hour, minute, sec, SAVE_NMEA );
		if ( nret < 0 ) {
			dbgPrintf( "setFilePath nret=%d\r\n", nret );
			mFileSaving = 0;
			for( int i=0; i < RECEIVER_MAX; i++ ) strcpy( mSaveFileName[i], "" );
			return -1;
		}
	}
	
	int numOutBytes = setNmeaData( pGpsData, buff, 256 );
	if ( numOutBytes > 0 ){
		char* p = mSdBuff[i];

		if ( strlen( p ) + numOutBytes > SD_BUFF_MAX || millis() - mSdSavedTime[i] >900 ){
			int saveBytes = strlen( p );
			if ( saveBytes > 0 ){
				mSdSavedTime[i] = millis();
				nret = sdSave( mSaveFileName[i], p, saveBytes );
				if ( nret == saveBytes ){
					 mFileSaved++;
					*p = '\0';
				}
				else {
					dbgPrintf("file save error i=%d n=%d ret=%d\r\n", receiverNum, saveBytes, nret);
				}
			}
		}

		if ( strlen( p ) + numOutBytes < SD_BUFF_MAX ){
			strcat( p + strlen( p ), buff );
		}
	}

	return numOutBytes;
}

int sdSave(int receiverNum, char *buffer, int numBytes )
{
	int nret;
	char buff[256];
	int i = receiverNum;
	if ( strlen( mSaveFileName[i] ) == 0 ){
		int year = mGpsData[i].year;
		int month = mGpsData[i].month;
		int day = mGpsData[i].day;
		int hour = mGpsData[i].hour;
		int minute = mGpsData[i].minute;
		int sec = mGpsData[i].second;
		nret = setFilePath( i, year, month, day, hour, minute, sec, mSaveFormat );
		if ( nret < 0 ) {
			dbgPrintf( "setFilePath nret=%d\r\n", nret );
			mFileSaving = 0;
			for( int i=0; i < RECEIVER_MAX; i++ ) strcpy( mSaveFileName[i], "" );
			return -1;
		}
	}
	
	nret = sdSave( mSaveFileName[i], buffer, numBytes );
	if ( nret == numBytes ){
		mFileSaved++;
	}
	else {
		dbgPrintf("file save error i=%d n=%d ret=%d\r\n", receiverNum, numBytes, nret);
	}

	return nret;
}

int sdSave( String fileName, char *buff, int numBytes )
{
	return sdSave( fileName.c_str(), buff, numBytes, FILE_APPEND );
}


int sdSave( char *fileName, char *buff, int numBytes )
{
	return sdSave( fileName, buff, numBytes, FILE_APPEND );
}


// mode: FILE_WRITE or FILE_APPEND
//
int sdSave( const char *fileName, char *buff, int numBytes, const char* mode )
{
	bool saved = false;
	int nret;
	for( int i = 0; i < 5; i++ ){
		File fd = SD.open(fileName, mode);
		if (! fd){
			delay(10);
			continue;
		}

		nret = fd.write( ( unsigned char *) buff, numBytes );
		if ( nret != numBytes ){
			dbgPrintf( "file write error: try=%d %s\r\n", i+1, strerror(errno) );
			fd.close();
			continue;
		}
		fd.close();
		saved = true;
		break;
	}
	if ( ! saved ) nret = -1;
	return nret;
}


// 保存用ファイル名バッファ(mSaveFileName)にファイル名をセットする。
//
int setFilePath( int receiverNum, int year, int month, int day, int hour, int minute, int sec, int saveFormat )
{
	char buff[256];
	const char *rootDir = mGpsLogDir.c_str();
	char *extension[] = { "log", "ubx", "rtcm3" };

	if ( ! SD.exists( rootDir ) ){
		if ( ! SD.mkdir( rootDir ) ) {
			dbgPrintf( "mkdir error: %s\r\n", rootDir );
			return -1;
		}
	}

	sprintf( buff, "%s/%d%02d%02d", rootDir, year, month, day );
	if ( ! SD.exists( buff ) ){
		if ( ! SD.mkdir( buff ) ) {
			dbgPrintf( "mkdir error: %s\r\n", buff );
			return -2;
		}
	}
	if ( saveFormat > 2 ) saveFormat= 0;
	sprintf( mSaveFileName[receiverNum], "%s/gps_r%d_%d%02d%02d_%02d%02d%02d.%s", 
							buff, receiverNum, year, month, day, hour, minute, sec, extension[ saveFormat ] );
	return 0;
}

// 
// 戻り値＝読み出したバイト数
//         0:ファイルの終わり
//         負数：エラー
//
int sdRead( const char *fileName, char *buff, int numBytes )
{
	int nret;
	bool done = false;
	File fd;
	for( int i = 0; i < 5; i++ ){
		fd = SD.open(fileName, FILE_READ);
		if (! fd) {
			delay(10);
			continue;
		}
	}
	if (! fd) return -1;

	nret = fd.read( ( unsigned char *) buff, numBytes );
	fd.close();
	
	if ( nret == -1 ) return 0;
	return nret;
}


// 実行パラメータを保存する
//
int saveRunInfo( struct stRunInfo *runInfo )
{
	const char *mode = FILE_WRITE;
	int nret = sdSave( mRunInfoPath.c_str(), (char*) runInfo, (int)sizeof( *runInfo ), mode );
	return nret;
}

// 実行パラメータを読み出す
//
int readRunInfo( struct stRunInfo *runInfo )
{
	int nret = sdRead( mRunInfoPath.c_str(), (char*) runInfo, sizeof( *runInfo ) );
	return nret;
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
	mGGAAddress = pbuff;
	sprintf( pbuff, "$GPGGA,%06d.%02d,%.6lf,N,%.6lf,E,%d,%d,%.1lf,%.3lf,M,%.3lf,M,,0000",
			hms, csec, deg2degmin(lat), deg2degmin(lon),  fixGGA, numSats, 
			pdop, altMSL, geoid );
	csum = checksumOf( pbuff + 1, strlen( pbuff ) - 1 );
	sprintf( pbuff + strlen( pbuff ), "*%02x\r\n", csum );
	
	return strlen( buff );
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
//                           地図表示
// ************************************************************

int dispMap( struct stMapData *mapData )
{
	if ( ! mapData ) return -1;
	
	lcdDispJpeg( mapData->path.c_str() );
	lcdDispCursor( 0, mLcdWidth / 2, mLcdHeight / 2, RED );
	return 0;
}

int getMap( struct stMapData *mapData )
{
	Client *client;
	if ( mNetAccess == USE_WIFI ) client = &mWifiClient;
	else if ( mNetAccess == USE_MODEM_3G ) client = &mGsmClient;
	else return -1;
	
	if ( ! mGoogleKey || strlen( mGoogleKey) == 0 ) return -2;
	
	char *mapType = "roadmap";
	if ( mapData->mapType == MAP_SAT ) mapType = "satellite";
	
	int zoomLevel = mapData->zoomLevel;
	double lat = mapData->centerLat;
	double lon = mapData->centerLon;
	int width = mapData->width;
	int height = mapData->height;
	const char *path = mapData->path.c_str();
	
	bool baseRecvReady = mBaseRecvReady;
	mBaseRecvReady = false;
	delay(5);
	int nret = getGoogleMap( client, mapType, lat, lon, zoomLevel,
									width, height, path, mGoogleKey );
	mBaseRecvReady = baseRecvReady;
	if ( nret < 0 ){
		dbgPrintf( "Google map error nret=%d\r\n", nret );
		return -3;
	}

	return 0;

}

// ************************************************************
//                           Soft Serial
// ************************************************************


#define WAIT { while (ESP.getCycleCount()-start < wait); wait += mBitTime; }


int phUartWriteBytes( byte *buffer, int numBytes )
{
	for( int i=0; i < numBytes; i++ ){
		phUartWrite( buffer[i] );
	}
	return numBytes;
}

int phUartWrite( byte b) {
   unsigned long wait = mBitTime;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
taskENTER_CRITICAL( &mux ) ;
   
   digitalWrite(mPinPhOut, HIGH);
   unsigned long start = ESP.getCycleCount();
    // Start bit;
   digitalWrite(mPinPhOut, LOW);
   WAIT;
   for (int i = 0; i < 8; i++) {
     digitalWrite(mPinPhOut, (b & 1) ? HIGH : LOW);
     WAIT;
     b >>= 1;
   }
   // Stop bit
   digitalWrite(mPinPhOut, HIGH);
   WAIT;
taskEXIT_CRITICAL( &mux );

   return 1;
}

// データがあれば読み出す
//
// 戻り値＝読み出したバイト数
//        負数：エラー
//
int phUartReadBytes( byte* buff, int buffSize )
{
	int bytesSaved = mPhUartBuff.read( buff, buffSize );
	return bytesSaved;
}	

void IRAM_ATTR phUartInIsr() {
	if ( digitalRead( mPinPhIn ) ) return;

	unsigned long wait = mBitTime + mBitTime/3 - 500;
	unsigned long start = ESP.getCycleCount();

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
//taskENTER_CRITICAL( &mux ) ;
	byte data = 0;
	for (int i = 0; i < 8; i++) {
		WAIT;
		data >>= 1;
digitalWrite( mPinTest, HIGH);
digitalWrite( mPinTest, LOW);
		if (digitalRead( mPinPhIn )) data |= 0x80;
	}

	// Stop bit
	WAIT;
//taskEXIT_CRITICAL( &mux );
	int nret = mPhUartBuff.write( &data, 1 );
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
