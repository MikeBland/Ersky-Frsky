
#include <WiFi.h>

#include "SimpleFTPServer.h"

WiFiServer Server(80) ;
FtpServer ftpSrv ;

uint8_t FrskyWifiState ;
WiFiClient Client ;

uint8_t LedStatus ;

#define UP_IDLE				0
#define UP_HEADER1		1
#define UP_DATA1			2
#define UP_DATA2			3
#define UP_HEADER2		4

uint8_t UploadState ;

const uint8_t IconBase64[] = {
"AAABAAEAEBAAAAEACABoBQAAFgAAACgAAAAQAAAAIAAAAAEACAAAAAAAAAEAAHQOAAB0DgAAAAEAAAABAAAAAAAAAAAIAAAICAAAEBgACBAYAAgYGAAAGCEACBghAAAhKQAIISkAECEpABghKQAQKTEAGCkxABApOQAYKTkAEDE5ACE5QgAYQkoAEEJSACFCUgApSlIAIUpaAClKWgAhUmsAKVJrABhaawAhWmsAKVprACFacwAxWnMAKWNzADFjcwAxWnsAMWN7ADFrewAxc5QAOXOUACl7lABCjK0AOZStADGUtQA5lLUASpy9AFqlvQBSrcYASqXOADmt1gA5rd4AQrXnADm95wBCvecAWr3nAGO95wA5xucAWsbnAEK97wBKve8AMcbvADnG7wAxxvcAOcb3AELG9wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8AAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAEBAAALLAsWLisCFSgnBxYqKigXDzUNGz4yAiA7OQkfOz47Iw80Dxs+MwQiOzMJHTs+OCIPNA8bOzMCIjszCR07MyURDzQPGzszAiI7OBMmOy8HAA81Dxw7MwIiOz05ODs+KgINLQscOzMCIjs7Ozs6OzYSAAIAHTgzAiA+PTs7Oz04IgAAABs7MwIiOD09PT06OyEAAAAcODMCIjs7PT09OzIOAAAABxAPACA7PTs7OzglAAAAAAAAAAAeMzMxMS8fAgAAAAAAAAAAAgcFBwcCAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
} ;

const char *Dirs[5] = { "SPIFFS", "/voice/system", "/voice/user", "/TEXT", "/SCRIPTS"} ;

void _callback(FtpOperation ftpOperation, unsigned int freeSpace, unsigned int totalSpace){
//	Serial.print(">>>>>>>>>>>>>>> _callback " );
//	Serial.print(ftpOperation);
	/* FTP_CONNECT,
	 * FTP_DISCONNECT,
	 * FTP_FREE_SPACE_CHANGE
	 */
//	Serial.print(" ");
//	Serial.print(freeSpace);
//	Serial.print(" ");
//	Serial.println(totalSpace);

	// freeSpace : totalSpace = x : 360

//	if (ftpOperation == FTP_CONNECT) Serial.println(F("CONNECTED"));
//	if (ftpOperation == FTP_DISCONNECT) Serial.println(F("DISCONNECTED"));
};

void _transferCallback(FtpTransferOperation ftpOperation, const char* name, unsigned int transferredSize){
//	Serial.print(">>>>>>>>>>>>>>> _transferCallback " );
//	Serial.print(ftpOperation);
	/* FTP_UPLOAD_START = 0,
	 * FTP_UPLOAD = 1,
	 *
	 * FTP_DOWNLOAD_START = 2,
	 * FTP_DOWNLOAD = 3,
	 *
	 * FTP_TRANSFER_STOP = 4,
	 * FTP_DOWNLOAD_STOP = 4,
	 * FTP_UPLOAD_STOP = 4,
	 *
	 * FTP_TRANSFER_ERROR = 5,
	 * FTP_DOWNLOAD_ERROR = 5,
	 * FTP_UPLOAD_ERROR = 5
	 */
//	Serial.print(" ");
//	Serial.print(name);
//	Serial.print(" ");
//	Serial.println(transferredSize);
};

void startFrskyWifi()
{
	FrskyWifiState = ( WifiType ) ? 1 : 0 ;
	if ( WiFiMode == WIFI_MODE_WEB )
	{
	  Server.begin() ;
	}
	else if ( WiFiMode == WIFI_MODE_FTP )
	{
		ftpSrv.setCallback(_callback);
  	ftpSrv.setTransferCallback(_transferCallback);
		ftpSrv.begin("frskyneo","frskyneo");    //username, password for ftp.   (default 21, 50009 for PASV)
	}
}

uint8_t RequestLength ;
uint8_t Flushing ;
char Request[200] ;
uint32_t FlushTimer ;
#define FILENAME_LENGTH		28
char FileName[FILENAME_LENGTH + 2] ;
uint32_t UploadFileSize ;
uint32_t UploadFileLength ;
uint8_t BoundaryString[80] ;
uint8_t BoundarySize ;
uint8_t DestinationIndex ;
uint32_t ClientTimer ;

File UploadFileHandle ;

uint8_t PageToSend ;

struct t_uploadControl
{
	File spiffsHandle ;
	lfs_file_t lfsHandle ;
	uint8_t destination ;
} UploadControl ;

int32_t fileOpenWrite( char *name )
{
	lfs_dir_t my_dir ;
	char path[100] ;
	char *t ;
	int32_t result ;
	UploadControl.destination = DestinationIndex ;
	if ( DestinationIndex == 0 )	// SPIFFS
	{
		UploadControl.spiffsHandle = SPIFFS.open( FileName, "w") ;
		return 0 ;
	}
	// somewhere on Lfs
	result = lfs_dir_open( &Lfs, &my_dir, Dirs[DestinationIndex] ) ;
	if ( result != 0 )
	{ 
		result = lfs_mkdir( &Lfs, Dirs[DestinationIndex] ) ;
	}
	else
	{
		lfs_dir_close( &Lfs, &my_dir ) ;
	}
	t = ( char *)cpystr( (uint8_t *)path, (uint8_t *)Dirs[DestinationIndex] ) ;
	cpystr( (uint8_t *)t, (uint8_t *)name ) ;
  result = lfs_file_open( &Lfs, &UploadControl.lfsHandle, path, LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC ) ;
	
//	Serial.println( Dirs[DestinationIndex] ) ;
	
	return result ;
}

void fileClose()
{
	if ( UploadControl.destination == 0 )	// SPIFFS
	{
		UploadControl.spiffsHandle.close() ;
		return ;
	}
	// Lfs
	lfs_file_close(&Lfs, &UploadControl.lfsHandle ) ;
}

int32_t fileWrite( uint8_t *data, uint32_t length )
{
  int32_t written ;
	
	if ( UploadControl.destination == 0 )	// SPIFFS
	{
		written = UploadControl.spiffsHandle.write( data, length ) ;
		return written ;
	}
	// Lfs
	written = lfs_file_write( &Lfs, &UploadControl.lfsHandle, data, length ) ;
	return written ;
}

void sendDestOption( WiFiClient client, char *name, uint32_t selected )
{
	uint8_t text[100] ;
	uint8_t *t ;
	client.print("<option value=\"FileDest=") ;
	t = cpystr( text, (uint8_t *) name ) ;
	*t++ = '"' ;
	if ( selected )
	{
		t = cpystr( t, (uint8_t *) " selected" ) ;
	}
	*t++ = '>' ;
	t = cpystr( t, (uint8_t *) name ) ;
	client.print( (char *)text ) ;
	client.println("</option>");
}


void sendUploadForm(WiFiClient client)
{
  					// Return the response
	client.println("HTTP/1.1 200 OK") ;
	client.println("Content-Type: text/html; charset=UTF-8") ;
	client.println("") ;
	client.println("<!DOCTYPE HTML>") ;
	client.println("<html>") ;
	client.println("<head>") ;
	client.println("<title>Frsky Neo Demo</title>") ;
		
	client.print( (char*)"<link href='data:image/x-icon;base64," ) ;
	client.print( (char*)IconBase64 ) ;
	client.print( (char*)"' rel='icon' type='image/x-icon' />" ) ;
  					
	client.println("</head>") ;
	client.println("<body>") ;
	client.println("<a href=\"/\">Refresh Status</a>") ;
	client.println("</br></br>") ;
		
	//check the LED status
	if (LedStatus == 1)
	{
	  client.print("LED is Off</br>") ;
	  client.println("Turn the LED <a href=\"/LED=ON\">ON</a></br>") ;
	}
	else if (LedStatus == 0)
	{
	  client.print("LED is On</br>") ;
	  client.println("Turn the LED <a href=\"/LED=OFF\">OFF</a></br>") ;
	}
		
	client.println("<form action=\"upload\" method=\"post\" enctype=\"multipart/form-data\">");
	client.println("<br><label for=\"Destination\">Choose destination:</label>");
  client.println("<select name=\"Destination\" id=\"Destination\">");

	sendDestOption( client, "SPIFFS", DestinationIndex == 0 ) ;
	sendDestOption( client, "voice/system", DestinationIndex == 1  ) ;
	sendDestOption( client, "voice/user", DestinationIndex == 2  ) ;
	sendDestOption( client, "TEXT", DestinationIndex == 3  ) ;
	sendDestOption( client, "SCRIPTS", DestinationIndex == 4  ) ;
  
	client.println("</select>");
	client.println("<br><br>Select file to upload:");
	client.println("<input type=\"file\" name=\"fileToUpload[]\" multiple id=\"fileToUpload\"><br>");
	client.println("<br><br><input type=\"submit\" value=\"Upload File(s)\" name=\"submit\">");
	client.println("</form>");

	client.println("</br>") ;

	client.println("</br>") ;
	client.println("</body>") ;
	client.println("</html>") ;
			
}

void sendUploadResult(WiFiClient client, uint32_t result)
{
  					// Return the response
	client.println("HTTP/1.1 200 OK") ;
	client.println("Content-Type: text/html; charset=UTF-8") ;
	client.println("") ;
	client.println("<!DOCTYPE HTML>") ;
	client.println("<html>") ;
	client.println("<head>") ;
	client.println("<title>Frsky Neo Demo</title>") ;
		
	client.print( (char*)"<link href='data:image/x-icon;base64," ) ;
	client.print( (char*)IconBase64 ) ;
	client.print( (char*)"' rel='icon' type='image/x-icon' />" ) ;
  					
	client.println("</head>") ;
	client.println("<body>") ;
	client.println("<a href=\"/\">Return to main page</a>") ;
	client.println("</br></br>") ;

	if ( result )
	{
		client.println("Upload Successful") ;
	} 
	else
	{
		client.println("Upload Failed") ;
	}
	
	client.println("</br>") ;

	client.println("</br>") ;
	client.println("</body>") ;
	client.println("</html>") ;
			
}


//uint8_t DebugFileData[10] ;
//uint8_t DebugFileDataIndex ;

uint8_t LastFileData[260+128] ;
//uint8_t EndFileData[256] ;
//uint32_t EndDataIndex ;
uint32_t LastDataIndex ;
uint32_t BoundaryIndex ;
uint32_t BytesWritten ;
uint32_t TotalBytesWritten ;

uint32_t writeToFile( uint8_t byte )
{
	uint32_t localLastDataIndex ;
	
	localLastDataIndex = LastDataIndex ;

//	if ( DebugFileDataIndex < 10 )
//	{
//		DebugFileData[DebugFileDataIndex++] = byte ;
//	}

	// Here we need to locate the boundary for the end of the file

//	if ( UploadFileSize > 1000 )
//	{
//		UploadFileSize -= 1 ;
//		LastFileData[LastDataIndex] = byte ;
//		if ( ++LastDataIndex >= 256 )
//		{
//			// write data to SPIFFS
//			fileWrite( LastFileData, 256 ) ;
////			UploadFileHandle.write( LastFileData, 256 ) ;
//			BytesWritten += 256 ;
//			LastDataIndex = 0 ;
//			FlushTimer = millis() ;
//		}
//	}
//	else
	{
		// Start looking for boundary
		LastFileData[localLastDataIndex] = byte ;
		if ( ++localLastDataIndex >= BoundarySize )
		{
			uint32_t x ;
			uint32_t y ;
			uint32_t z ;
			char *position ;
			y = localLastDataIndex - BoundarySize ;
//			{
			if ( LastFileData[y] == BoundaryString[0] )
			{
				if ( position = strstr( (char *)&LastFileData[y], (char *)BoundaryString ) )
				{
						
					// Found the boundary
					x = position - (char *)LastFileData ;
					fileWrite( LastFileData, x ) ;
//						UploadFileHandle.write( LastFileData, x ) ;
					BytesWritten += x ;
					TotalBytesWritten += x ;
					Serial.println("End of File") ;
					// close file
					fileClose() ;
//						UploadFileHandle.close() ;
//						Flushing = 1 ;
					LastDataIndex = localLastDataIndex ;
					return 1 ;
				}
			}
//			}
//			for ( x = 0 ; x < y ; x += 1 )
//			{
//				if ( LastFileData[x] == BoundaryString[0] )
//				{
//					if ( position = strstr( (char *)&LastFileData[x], (char *)BoundaryString ) )
//					{
						
//						// Found the boundary
//						x = position - (char *)LastFileData ;
//						fileWrite( LastFileData, x ) ;
////						UploadFileHandle.write( LastFileData, x ) ;
//						BytesWritten += x ;
//						TotalBytesWritten += x ;
//						Serial.println("End of File") ;
//						// close file
//						fileClose() ;
////						UploadFileHandle.close() ;
////						Flushing = 1 ;
//						LastDataIndex = localLastDataIndex ;
//						return 1 ;
//					}
//				}
//			}
			if ( y >= 256 )
			{
				fileWrite( LastFileData, y ) ;
//				UploadFileHandle.write( LastFileData, y ) ;
				BytesWritten += y ;
				TotalBytesWritten += y ;
				FlushTimer = millis() ;

				memmove( LastFileData, &LastFileData[y], BoundarySize ) ;
//				for ( x = 0 ; x < BoundarySize ; x += 1 )
//				{
//					LastFileData[x] = LastFileData[x+y] ;
//				}
				localLastDataIndex = BoundarySize ;
			}
		}
	}
	LastDataIndex = localLastDataIndex ;
	return 0 ;
}


void frskyWifi()
{

	if ( WiFiMode == WIFI_MODE_FTP )
	{
		ftpSrv.handleFTP() ;
		return ;
	}

	char c ;

	switch ( FrskyWifiState )
	{
		case 0 :
			if ( WiFi.softAPgetStationNum() )
			{
				FrskyWifiState = 1 ;
			}

		break ;

		case 1 :
  		Client = Server.available() ;
			if ( Client )
			{
				FrskyWifiState = 2 ;
				Serial.println("New Client") ;
				ClientTimer = millis() ;
				UploadState = UP_IDLE ;
				RequestLength = 0 ;
				Request[0] = '\0' ;
				Flushing = 0 ;
			}
		break ;

		case 2 :
		{	
			uint32_t count ;
			if ( Client.connected() )
			{
				if ( count )
				{
					ClientTimer = millis() ;
				}
				count = Client.available() ;
  			while ( count )
				{
					count -= 1 ;
					char c = Client.read() ;             // read a byte, then
					if ( UploadState == UP_DATA2 )
					{
						if ( writeToFile( (uint8_t) c ) )
						{
//							UploadState = UP_IDLE ;
							UploadState = UP_HEADER2 ;
							// Close file
							FlushTimer = millis() ;
//							Flushing = 1 ;
							// Need to send "Upload Done"
							PageToSend = 2 ;
							Serial.println("To Header2") ;
						}
						continue ;
					}
					Serial.write(c) ;                    // print it out the serial monitor
  			  if ( !Flushing)
					{
						Request[RequestLength++] = c ;
						if ( RequestLength > 198 )
						{
							RequestLength = 198 ;
						}
						if (c == '\n')
						{
							Request[RequestLength] = '\0' ;
							if ( UploadState == UP_IDLE )
							{
								// Examine line
					  		if (strstr( Request, "POST /upload" ) )
								{
									UploadState = UP_HEADER1 ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
									Serial.println("Found POST") ;
									FileName[0] = '\0' ;
									DestinationIndex = 0 ;
								}
							}
							else if ( UploadState == UP_HEADER1 )
							{
								char *position ;
								char delimiter ;
								char c ;
								char *dest ;

								Serial.println("In Header1:") ;
								// Examine line
					  		if ( position = strstr( Request, "filename=" ) )
								{
									position += 9 ;
									delimiter = *position++ ;
									dest = FileName ;
									*dest++ = '/' ;
									while ( ( c = *position++ ) != delimiter )
									{
										*dest++ = c ;
										if ( dest >= &FileName[FILENAME_LENGTH] )
										{
											break ;
										}
									}
									*dest = '\0' ;
									Serial.println() ;
									Serial.print("File :") ;
									Serial.println(FileName) ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
									TotalBytesWritten = 0 ;
								}
					  		else if ( position = strstr( Request, "FileDest=" ) )
								{
					  			if ( position = strstr( Request, "voice/system" ) )
									{
										DestinationIndex = 1 ;
									}
					  			else if ( position = strstr( Request, "voice/user" ) )
									{
										DestinationIndex = 2 ;
									}
					  			else if ( position = strstr( Request, "TEXT" ) )
									{
										DestinationIndex = 3 ;
									}
					  			else if ( position = strstr( Request, "SCRIPTS" ) )
									{
										DestinationIndex = 4 ;
									}
									Serial.println() ;
									Serial.print("Destination :") ;
									Serial.println(Dirs[DestinationIndex] ) ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( position = strstr( Request, "boundary=" ) )
								{
									char c ;
									position += 9 ;
									dest = (char *)BoundaryString ;
									*dest++ = '\r' ;
									*dest++ = '\n' ;
									*dest++ = '-' ;
									*dest++ = '-' ;
									while ( ( c = *position++ ) )
									{
										if ( ( c == '\r' ) || ( c == '\n' ) )
										{
											break ;
										}
										*dest++ = c ;
									}
									*dest = '\0' ;
									BoundarySize = dest - (char *)BoundaryString ;
									Serial.println("Found Boundary") ;
									Serial.println((char *)BoundaryString) ;
									Serial.println(BoundarySize) ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( position = strstr( Request, "Content-Length:" ) )
								{
									Serial.println("Found Length") ;
									position += 15 ;
									Serial.println(position) ;

									if ( *position == ' ' )
									{
										position += 1 ;
									}
									Serial.println(position) ;
									UploadFileSize = atol(position) ;
									if ( UploadFileSize )
									{
										Serial.println("Got a length") ;
										UploadFileLength = UploadFileSize ;

									}
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( ( position = strstr( Request, "Content-Type" ) ) && ( FileName[0] ) )
								{
									UploadState = UP_DATA1 ;
									Serial.println("Going to UP_DATA1") ;
//									DebugFileDataIndex = 0 ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( position = strstr( Request, "octet-stream" ) )
								{
									UploadState = UP_DATA1 ;
									Serial.println("Going to UP_DATA1") ;
//									DebugFileDataIndex = 0 ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( position = strstr( Request, "text/plain" ) )
								{
									UploadState = UP_DATA1 ;
									Serial.println("Going to UP_DATA1") ;
//									DebugFileDataIndex = 0 ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
								else
								{
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
							}
							else if ( UploadState == UP_HEADER2 )
							{
								char *position ;
								char delimiter ;
								char c ;
								char *dest ;
								Serial.println("In Header2:") ;
					  		if ( position = strstr( Request, "filename=" ) )
								{
									position += 9 ;
									delimiter = *position++ ;
									dest = FileName ;
									*dest++ = '/' ;
									while ( ( c = *position++ ) != delimiter )
									{
										*dest++ = c ;
										if ( dest >= &FileName[FILENAME_LENGTH] )
										{
											break ;
										}
									}
									*dest = '\0' ;
									Serial.println() ;
									Serial.print("File :") ;
									Serial.println(FileName) ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( ( position = strstr( Request, "Content-Type" ) ) && ( FileName[0] ) )
								{
									UploadState = UP_DATA1 ;
									Serial.println("Going to UP_DATA1") ;
//									DebugFileDataIndex = 0 ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
								else
								{
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
							}
							else if ( UploadState == UP_DATA1 )
							{
								UploadState = UP_DATA2 ;
								Serial.println("Going to UP_DATA2") ;
								// Open SPIFFS file here
								fileOpenWrite( FileName ) ;
//								UploadFileHandle = SPIFFS.open( FileName, "w") ;
//								DebugFileDataIndex = 0 ;
								LastDataIndex = 0 ;
								BoundaryIndex = 0 ;
								RequestLength = 0 ;
								Request[0] = '\0' ;
								BytesWritten = 0 ;
								ClientTimer = millis() ;
							}
							if ( UploadState == UP_IDLE )
							{
								Flushing = 1 ;
								FlushTimer = millis() ;
							}
						}
					}
				}
				if ( ( UploadState == UP_DATA2 ) || ( UploadState == UP_HEADER2 ) )
				{
					if ( (uint32_t) ( millis() - FlushTimer) > 1000 )
					{
						// Timeout uploading file
						// Close file
						// ? delete file ?
						Flushing = 1 ;
						FlushTimer = millis() ;

						if ( UploadState == UP_HEADER2 )
						{
							PageToSend = 2 ;
						}
						else
						{
							PageToSend = 1 ;
						}
						UploadState = UP_IDLE ;

						// Need to send "Upload Failed"
					}
				}
				if ( Flushing )
				{
					if ( (uint32_t) ( millis() - FlushTimer) > 200 )
					{
						Serial.println("End of Flushing") ;
						
						// Send response
					  if (strstr( Request, "/LED=ON" ) )
						{
  						LedStatus = 0 ;
							digitalWrite( 19, 1 ) ;		// Backlight
  					}
			
					  if (strstr( Request, "/LED=OFF" ) )
						{
  						LedStatus = 1 ;
							digitalWrite( 19, 0 ) ;		// Backlight
  					}

						switch ( PageToSend )
						{
							case 0 :
								sendUploadForm( Client ) ;
							break ;
							case 1 :
								sendUploadResult( Client, 0 ) ;
							break ;
							case 2 :
								sendUploadResult( Client, 1 ) ;
							break ;

						}
			 			PageToSend = 0 ;

						Request[0] = '\0' ;
						RequestLength = 0 ;

						Client.flush() ;
						Client.stop() ;
						FrskyWifiState = 1 ;
						Client = 0 ;

						Serial.println( "Sent" ) ;
						Flushing = 0 ;
					}
				}	
				if ( (uint32_t) ( millis() - ClientTimer ) > 10000 )
				{
					// Force disconnect
					Client.flush() ;
					Client.stop() ;
					FrskyWifiState = 1 ;
					Client = 0 ;
				}
			}
		}
		break ;
	}
	if ( WifiType == 0 )
	{
		if ( WiFi.softAPgetStationNum() == 0 )
		{
			FrskyWifiState = 0 ;
		}
	}
}







//void SD_file_download(String filename){
//  if (SD_present) { 
//    File download = SD.open("/"+filename);
//    if (download) {
//      server.sendHeader("Content-Type", "text/text");
//      server.sendHeader("Content-Disposition", "attachment; filename="+filename);
//      server.sendHeader("Connection", "close");
//      server.streamFile(download, "application/octet-stream");
//      download.close();
//    } else ReportFileNotPresent("download"); 
//  } else ReportSDNotPresent();
//}

#if 0

void cryptString( char *text, char *dest )
{
	char c ;
	do
	{
		c = *text++ ;
		if ( c )
		{
			c ^= 0x0D ;
		}
		*dest++ = c ;
	} while (c) ;
}

//  File f = SPIFFS.open("/RIDP.txt", "r");
//	if ( f )
//	{
//		String line = f.readStringUntil('\r\n') ;
//		parse( line, Xssid		, "SSID", 98 ) ;
//		line = f.readStringUntil('\r\n') ;
//		parse( line, Xpw			, "PW", 30 ) ;
//		cryptString( Xpw, Xpw ) ;
//  	f.close();
//	}

//	if ( !RouterStarted )
//	{
//		if ( Xssid[0] && Xpw[0] )
//		{
//		  WiFi.begin( Xssid, Xpw ) ;
//			Serial.println("Start Wifi");
//			RouterStarted = 1 ;
//		}
//	}

#endif


void writeNetIds()
{
	uint8_t data[50] ;
	int32_t result ;
  int32_t written ;
	uint8_t *p = data ;
	uint8_t *q ;
	uint32_t i ;
	uint32_t j ;
	uint32_t k ;

	q = NetId ;
	
	j = 0 ;
	k = 0 ;
	while ( *q )
	{
		*p++ = *q++ ;
		j += 1 ;
		if ( ++k > 19 )
		{
			break ;
		}
	}
	*p++ = 13 ;
	*p++ = 10 ;
	j += 2 ;
	q = NetPass ;
	k = 0 ;
	while ( *q )
	{
		*p++ = *q++ ;
		j += 1 ;
		if ( ++k > 19 )
		{
			break ;
		}
	}
	*p++ = 13 ;
	*p++ = 10 ;
	j += 2 ;
  
	result = lfs_file_open( &Lfs, &UploadControl.lfsHandle, (char *)"/NetId", LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC ) ;
	if ( result == 0 )
	{
    written = lfs_file_write( &Lfs, &UploadControl.lfsHandle, data, j ) ;
		lfs_file_close(&Lfs, &UploadControl.lfsHandle ) ;
	}
}

void readNetIds()
{
	uint8_t data[50] ;
	int32_t result = 0 ;
	int32_t nread ;
	uint32_t i ;
	uint32_t j ;
	uint32_t k ;
	uint8_t *p ;
	uint8_t *dest ;

	for ( i = 0 ; i < 20 ; i += 1 )
	{
		NetId[i] = 0 ;
		NetPass[i] = 0 ;
	}

	result = lfs_file_open( &Lfs, &UploadControl.lfsHandle, (char *)"/NetId", LFS_O_RDWR ) ;

	if ( result == 0 )
	{
		nread = lfs_file_read( &Lfs, &UploadControl.lfsHandle, data, 49 ) ;
		lfs_file_close(&Lfs, &UploadControl.lfsHandle ) ;
		dest = NetId ;
		p = data ;
		j = 0 ;
		k = 0 ;
		for ( i = 0 ; i < nread ; i += 1 )
		{
			if ( ( *p != 13 ) && (*p != 10) )
			{
				if ( j < 19 )
				{
					*dest++ = *p++ ;
					j += 1 ;
				}
				else
				{
					p += 1 ;
				}
			}
			else
			{
				if ( *p == 10 )
				{
					dest = NetPass ;
					k += 1 ;
					j = 0 ;
				}
				p += 1 ;
				if ( k > 1 )
				{
					break ;
				}
			}
		}
		for ( i = 18 ; i > 0 ; i -= 1 )
		{
			if ( NetId[i] == ' ' )
			{
				NetId[i] = 0 ;
			} 
			else
			{
				break ;
			}
		}
		for ( i = 18 ; i > 0 ; i -= 1 )
		{
			if ( NetPass[i] == ' ' )
			{
				NetPass[i] = 0 ;
			} 
			else
			{
				break ;
			}
		}
	}
}


