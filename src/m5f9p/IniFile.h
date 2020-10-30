#ifndef INI_FILE_H
#define INI_FILE_H

class IniFile {
  public:
	const char *mPath;
	char *mBuffer;
	unsigned long mFileSize;
	int mReadIndex;
	
	IniFile();
	int open( const char *path );
	int close();
	int readValue( const char *section, const char *key, char *buffer, int buffMaxBytes );
	double readDouble( const char *section, const char *key, double defaultValue );
	int readInt( const char *section, const char *key, int defaultValue );
	int writeValue( const char *section, const char *key, char *value );

  private:
	int readBytesUntil( char c, char *buff, int maxBytes );
	int readLine( char *buff, int maxBytes );
	bool isComment( char *line );
	int getToken( char *text, char startChar, char endChar, char *buffer, int bufferLength );
	int fileInsertLine( const char *path, int lineNum, int pos, char *text );
};

#endif
