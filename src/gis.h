
int latlon2xy( double phi0, double lamda0, double phi, double lamda, 
					double *x, double *y );
int getPosVector( struct stGpsData *gpsData0, struct stGpsData *gpsData, 
					double *x, double *y, double *z );
double thomasDistance( double phi1, double lamda1, double phi2, double lamda2 );
int getGoogleMap( Client *client, const char *mapType, double lat, double lon, int zoom,
				int width, int height, const char *savePath, const char *apiKey );

