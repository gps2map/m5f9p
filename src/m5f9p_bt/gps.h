#ifndef GPS_H
#define GPS_H

#include <time.h>

#define PI 3.141592653589793
#define DEG2RAD 0.01745329251994
#define RAD2DEG 57.295779513082

typedef int s32;
typedef unsigned int u32;
typedef short s16;
typedef unsigned short u16;
typedef unsigned char u8;

struct stGpsData
{
	bool ubxDone;
	int receiverNum;
	unsigned int iTOW;
	bool highPrecisionDone;
//	unsigned long millis;
	unsigned long micros;	// PVTを取得した時のTick。70分でリセットするので注意
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int msec;
	double lat;
	double lon;
	double height;
	double geoidSep; // geoid separation
	double velocity;	// km/h
	double direction;	// 0-360 deg
	int quality;		// 0 = No fix, 1 = Autonomous GNSS fix, 
						// 2 = Differential GNSS fix, 4 = RTK fixed, 
						// 5= RTK float, 6 = Estimated/Dead reckoning fix:
	int numSatelites;
	double dop;		// Position DOP
	double hDop;		// Horizontal DOP
	double vDop;		// Vertical DOP
};

struct stGpsRelPos
{
	bool ubxDone;
	unsigned int iTOW;
	unsigned long micros;	// RELPOSNEDを取得した時のTick。70分でリセットするので注意
	int north;	// North (cm)
	int east;	// East (cm)
	int down;	// Down (cm)
	int length;	// Length (cm)
	double heading;	// Heading (deg)
	unsigned int flags;
	int quality;		// 0 = No fix, 1 = GNSS fix, 
						// 2 = Differential GNSS fix, 4 = RTK fixed, 
						// 5= RTK float
};

struct stGpsSat
{
	unsigned int iTOW;
	byte gnssId;
	byte svId;
	byte cno;
	char elev;
	short azim;
	short prRes;
	unsigned int flags;
};


#define UBX_BUFF_MAX 3000
struct stUbxStatus
{
	int statusNum;	// 0=idle 1=sync1 done  2=sync2 done 3=class done
					// 4=id done 5=low byte done 6=high byte done
					// 7=receiving payload 8=data done
					// 9=chksum A done 10=data ready
	int msgClass;
	int msgId;
	int numbytes;
	int buffIndex;
	byte  buff[ UBX_BUFF_MAX ];
	byte  chksumA;
	byte  chksumB;
};

double gpsDate2UnixTime( struct stGpsData *gpsData );
int gpsEcef2Llh( double x, double y, double z, double *lat, double *lon, double *height );
int gpsLlh2Ecef( double lat, double lon, double height, double *x, double *y, double *z );
int gpsSetBaudrate( int baudrate );
int gpsWrite( int gpsInterface, char *buff, int numBytes );
//int f9pI2cNumBytes( int i2cAddress );
//int f9pI2cReadBytes( int i2cAddress, int bytesToRead, byte* buffer, int bufferBytes );
//int f9pI2cWriteBytes( int i2cAddress, int bytesToWrite, byte* buffer );
int ubxDecode( int f9pInterface, struct stUbxStatus *ubxStatus );
int ubxBuffDecode( byte *buffer, int numBytes, struct stUbxStatus *ubxStatus, int *decodedBytes );
int ubxDecodeNavPvt( struct stUbxStatus *ubxStatus, struct stGpsData *gpsData);
int ubxDecodeHPPOSLLH( struct stUbxStatus *ubxStatus, struct stGpsData *gpsData);
int ubxDecodeNavSvin( struct stUbxStatus *ubxStatus, int *secObservation,
							float *mrAccuracy, double *x, double *y, double *z );
int ubxDecodeNavRelPosNed( struct stUbxStatus *ubxStatus, struct stGpsRelPos *relPosNed );
int ubxDecodeNavSat( struct stUbxStatus *ubxStatus, stGpsSat *satellites, int maxSatellites );
int ubxDecodeNavDop( struct stUbxStatus *ubxStatus, struct stGpsData *gpsData);
int gpsSetBaseCoordinate( int gpsInterface, int mode, double degLat, double degLon, double mHeight,
					unsigned int secSurveyin, float mSurveyinAccuracy );
int ubxSetCommand( byte msgClass, byte msgId, int numLength, byte *payLoad );
int ubxSendCommand( int gpsInterface, byte msgClass, byte msgId, int numLength, byte *payLoad );
int ubxSendCommand( int gpsInterface, int numBytes, int msecTimeout );
int gpsSetMessageRate( int gpsInterface, byte msgClass, byte msgId, int rate );
int gpsSetMeasurementRate( int gpsInterface, int rate );
int gpsSetUartPort( int gpsInterface, int portId, unsigned int baudRate, 
								int ubxIn, int nmeaIn, int rtcm3In,
								int ubxOut, int nmeaOut, int rtcm3Out );
int gpsReset( int gpsInterface );
int gpsSetBaseCoordinate( int gpsInterface, int mode, double degLat, double degLon, double mHeight,
					unsigned int secSurveyin, float mSurveyinAccuracy );
int gpsGetAck( int gpsInterface, byte msgClass, byte msgId, int msecTimeout );
int gpsCheckRtcm( byte *buffer, int numBytes );

#endif

