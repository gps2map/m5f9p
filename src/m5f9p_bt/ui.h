#ifndef UI_H
#define UI_H


#define BUTTON_A 1
#define BUTTON_B 2
#define BUTTON_C 3

extern int mLcdWidth;
extern int mLcdHeight;
extern int mLcdRotation;

//int setSineCurve( int frequency, int duration );
//void playSound( int volume );
void speakerOff();
int lcdInit( bool setRotation );
void lcdDispText( int lineNum, char* format, ... );
void lcdTextSize( int textSize );
void lcdClear( int color );
int lcdGetColor16( int color24 );
void lcdTextColor24( int color );
void lcdClear();
void lcdTextColor( int color );
void lcdTextColor( int red, int green, int blue );
void lcdClearLine( int lineNumber );
void lcdDispJpeg( const char *path );
void lcdDispCursor( int type, int x, int y, int color );
int lcdRotate( int degree );
void lcdDispButtonText( int textSize, int color, char *textA, char *textB, char *textC );
void lcdDispButtonText( int textSize, int color, char *textA, char *textB, char *textC, bool clear );
void lcdDispAndWaitButton( int lineNum, char* format, ... );
int waitButton( int useA, int useB, int useC,  int numA, int numB, int numC );
int waitButton();
int buttonPressed();

#endif
