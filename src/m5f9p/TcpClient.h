#ifndef TCP_CLIENT
#define TCP_CLIENT

#include <Arduino.h>
#include <Client.h>

class TcpClient {
  public:
	Client *mClient = NULL;
	char mHost[64];
	int mPort;
	char mAgentName[32];
	
	TcpClient( Client *client );
	int connect( const char *host, int port );
	int connected();
	void stop();
	int available();
	int read();
	int read( byte *buff, int buffSize );
	int readLine( char *buff, int buffSize, int msecTimeout );
	int write( byte *buff, int numBytes, int msecTimeout );
	int setAgentName( const char *agentName );

	int httpGet( const char *path, char *response, int responseMax, int *statusCode );
	int ntripRequest( const char *mountPoint, char *user, char *password, char *gga );
	int ntripRequest( const char *mountPoint, char *user, char *password );
	int ntripRequest( const char *path );
	int ntripServerConnect( const char *host, int port, 
								const char *mountPoint, const char *password );

};

#endif