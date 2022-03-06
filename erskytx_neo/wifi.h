
#include <WiFi.h>
#include <Update.h>

#include "SimpleFTPServer.h"

WiFiServer Server(80) ;
FtpServer ftpSrv ;

uint8_t FrskyWifiState ;
WiFiClient Client ;

//uint8_t LedStatus ;

#define UP_IDLE				0
#define UP_HEADER1		1
#define UP_DATA1			2
#define UP_DATA2			3
#define UP_HEADER2		4


#define DEST_SPIFFS		0
#define DEST_V_SYSTEM	1
#define DEST_V_USER		2
#define DEST_TEXT			3
#define DEST_SCRIPTS	4
#define DEST_UPDATE		5

uint8_t UploadState ;

const uint8_t IconBase64[] = {
"AAABAAEAEBAAAAEACABoBQAAFgAAACgAAAAQAAAAIAAAAAEACAAAAAAAAAEAAHQOAAB0DgAAAAEAAAABAAAAAAAAAAAIAAAICAAAEBgACBAYAAgYGAAAGCEACBghAAAhKQAIISkAECEpABghKQAQKTEAGCkxABApOQAYKTkAEDE5ACE5QgAYQkoAEEJSACFCUgApSlIAIUpaAClKWgAhUmsAKVJrABhaawAhWmsAKVprACFacwAxWnMAKWNzADFjcwAxWnsAMWN7ADFrewAxc5QAOXOUACl7lABCjK0AOZStADGUtQA5lLUASpy9AFqlvQBSrcYASqXOADmt1gA5rd4AQrXnADm95wBCvecAWr3nAGO95wA5xucAWsbnAEK97wBKve8AMcbvADnG7wAxxvcAOcb3AELG9wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8A////AP///wD///8AAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAEBAAALLAsWLisCFSgnBxYqKigXDzUNGz4yAiA7OQkfOz47Iw80Dxs+MwQiOzMJHTs+OCIPNA8bOzMCIjszCR07MyURDzQPGzszAiI7OBMmOy8HAA81Dxw7MwIiOz05ODs+KgINLQscOzMCIjs7Ozs6OzYSAAIAHTgzAiA+PTs7Oz04IgAAABs7MwIiOD09PT06OyEAAAAcODMCIjs7PT09OzIOAAAABxAPACA7PTs7OzglAAAAAAAAAAAeMzMxMS8fAgAAAAAAAAAAAgcFBwcCAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
} ;

const char *Dirs[6] = { "SPIFFS", "/voice/system", "/voice/user", "/TEXT", "/SCRIPTS", "UPDATE"} ;

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


void updateWrite( uint8_t *data, uint32_t length )
{
	if (Update.write( data, length) != length)
	{
		Serial.println("Update Write Fail") ;
	}
}

int32_t fileOpenWrite( char *name )
{
	lfs_dir_t my_dir ;
	char path[100] ;
	char *t ;
	int32_t result ;
	UploadControl.destination = DestinationIndex ;
	if ( DestinationIndex == DEST_SPIFFS )	// SPIFFS
	{
		UploadControl.spiffsHandle = SPIFFS.open( FileName, "w") ;
		return 0 ;
	}
	if ( DestinationIndex == DEST_UPDATE )	// UPDATE
	{
//		UpdateIndex = 0 ;
		// Start update
		if (!Update.begin(UPDATE_SIZE_UNKNOWN)) //start with max available size
		{
			Serial.println("Update Begin Fail") ;
		}			 
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
	
	return result ;
}

void fileClose()
{
	if ( UploadControl.destination == DEST_SPIFFS )	// SPIFFS
	{
		UploadControl.spiffsHandle.close() ;
		return ;
	}
	if ( DestinationIndex == DEST_UPDATE )	// UPDATE
	{
		// End update
		if (Update.end(true))
		{ //true to set the size to the current progress
			Serial.println("Update Success" ) ;
		}
		else
		{
			Serial.println("Update End Fail" ) ;
		}
		return ;
	}
	// Lfs
	lfs_file_close(&Lfs, &UploadControl.lfsHandle ) ;
}

int32_t fileWrite( uint8_t *data, uint32_t length )
{
  int32_t written ;
	
	if ( UploadControl.destination == DEST_SPIFFS )	// SPIFFS
	{
		written = UploadControl.spiffsHandle.write( data, length ) ;
		return written ;
	}
	if ( DestinationIndex == DEST_UPDATE )	// UPDATE
	{
		// write update
		if (Update.write( data, length) != length)
		{
			Serial.println("Update Write Fail" ) ;
		}			 
		return 0 ;
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

void sendLogo(WiFiClient client)
{
	client.println("<img Src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAANYAAAAhCAIAAAAqFsLYAAAAFHRFWHRDcmVhdGlvbiBUaW1lAAfmAwYSCp+YqV4AAAAHdElNRQfmAwYTNDYE8gx0AAAACXBIWXMAAArwAAAK8AFCrDSYAAAOOUlEQVR42u0cWYwcxbWqunuuPbxrdo0xtrGByBbEBIMcBSkkgB1+HOUXDJgoMgSFfJIoYBDmI4JIEYRDwZEcReJQ+CAoEEtREAhzBKI4sjDetQkBbIOE8cZisb27c3VXVd6r6qO6p3umZ5Y1Epmn0U5Nd3W9o169q6qXSilJ91CfvN2uTTa5TaQIL1JK4S8MCA3qjNgDF7Dhb9DFV9PScrwNPSnrAVcfvtpAe1TBidtK7qG5uqQkepwaHRgjBQd10qXD1uJNZOlWVhwnBPS1r4V9iMG8FEKbvRCk8eGC1Jq0Wqe8fpKdfl78+8fe9JsKnegVWx++mpChgmgaRewjRWbHeEMDKqcUlEoh6dwMkY0T5Mh2fuJFxCj7WtiHCDJUEDWIxT7xME6aHeMNotRRCN2Q6g53Pco9IT66X5x+Rw3Vi/fvw1cS7JYrqDTi9D566hXXUxZLEmYRyRbZy39IWVF3Mh0wqFncIeNPwyjqFufcsi3e/PCB4iV/oKxMKOlDH0iKCmLeavHZ9xzvr4WTHrEoEZIUWXWmYi+7kQQqaIJWOK2FupH4qXuBN280rcrQMT71ZwuG6ifIfVDQqgSoMmjtZkm1YVfnSLXGvDlC7WEz+TD9qGkFw0ZajIjKyRuyefwFyet9/euDhlY9kP5fyWMfwc1OCS+aUDjQNa2LYSO4CYZQOvyYmDmgLvTzkj7kL8pkh2605VZWmqwQMrtIxexBotT8y2a/D18+zM8bKhUy9Sw0fsmO0nyEyvpUer8+/P9Bjyroa1SLwqVUB0myD+Q3ks/hxb4R7ANmxJiZ6nYX6pgwX+mqRlJTY+zErGJKb5J4ViYaPYNOpOgZMbqa2vnTHAJjHeZFKtAM5uQxQV5XkpnPs6kislsy04WcJySXoqo7o5qYNuhC3s6M6nxxLJ45dQ8xdotuPuTNn7WEiGz3yINSNBgl1jk30PKqnCcJZOwrpTodIIs3sJ+A9JoNXBj8zETx6KOPHTp0yHYcwXmEKz/7QX+wuULw0dHRHTvuLRaLuZ/vGvQRoV27fr9v3z7bVmQnq1g0fdGFF2MN3OKkjN23497x8fEsdIc/PPzrBx8ELsFYum5zy5brr7rqKiFElu3Ut5544sm33vpHoVDwPHfRokUgmXK5nJPNxx/feeDAAccpuJ679Oyzt2+/C8bJ+azG/uyzz7788isOzKzgNoB7/PnKACggEY1NqIJKLTp6kajeHFRh0mTk3zIL1BakIo1yeXi9+pkuJi3cN974+/539gPRNMIUzA+NkxhOW+poBEcbHhq+5567c0qqN9Bk792799XXXgfh0vTlIn1elIJFP6POvkyRbBUt//xnd4AK6sFb0X1+6uRLL70s1SaWZdkbNmxor4La/b399tt79rzKhQcjVCqVu7dvJ2XSiiKVwTfffGvvv/YKDiis8SVjd975i65EBH8nJib27NnDUUQMDJJN7OFadQYIdqiVmNN2Y8W+0q1gSoBIWbFCGuUrSWGJxEMM6WLSghgYHIAGrDDugRWUDHcJTeKoYRiD8g4NLhp3wD4A5yMji/JLqjfQZFcGgGxScAqca7KzXYpMPh+s2vAnAfYtyyJp7k9fsWyLMWjZzGKgFtrMt9Ekn8hKBb6LhaLHvaGhIcpyBQ/mvJRKJc/zBgcHexBRqVQOZ7ZSHoR0hEvBu40AaXLcQJxaj1G9UkJyi8lGw3YuvKH94Hq1gUDhAwMDqzCaEDItdmz1Ydr80jDfBr8Po506dbo7DruHgGyuyeaeR3yyTQKNxRGTpbrtRya6H1pB13WVKpMsKwhxE0wkNKA/zGTHZEjfAjOpwxt4Fqk1B+zIID4LFgy01xOiu80FHzuYH4G0ckDvcdwjTinj5RxSG6CAYRinXMFGs0FcV5qbdWqdseIAbY78iFUuJNkmMIkBgzlaLpfWr19vCMicSFMFTTvp34KnYBbHxsbQjp4hkMgc2MJi8bL1lykzJgK/GgqFxCNFmtZG36qjtGzliDxRV6V+2X0Sk0RI8ytKDGjcbdpZnZJ448gSjhitH57ncppDm6kzwqf+UihMu67fA9WI0PIAaRY3Fc7d2sUBBQgWIHqkfNWqVU899WQv7H4JQINQlS4ZH3/66ScXODtWwvWz4tAWdHDEVJtcnFRpxEv56PSLwYzq4kZXtBqHVvw2zVDBtIdbyYj5E9uhcsUd9pLvY3vs2ubB2yw2w4UfjkGI4pa/W/jaPapvbsJxXQtc3ZI0m03IobrjOIP/BQap8iXgETwdrdfrEDYtINl+JIxOUuZm0H/fQmmjkOaGquwsJX9LTKAGi162uKQKMvRZUhgtrwq2J8mGIM8bLI9+W3kcTssrycDFxcY/q1XNr3Bdp7Dydv/0A1FeKp8hVO5FHj169Lrrt4R1Qj9lJEauTNRmi6Qp0SF2l2OLz3rssUfbaMOdd931wfsfMMuSUXwTGw38OMQuV19zzU9v/0nnsElgODj136ktW24AZ5oenIW5b1pJRt+1LPbIIw8vW7asnfQVCIzpya5du3bv3g0RFq4BGYxDAsmoU+ww5pEjR1U4hvL9fPqzm7ZutS0L8jYI7zZv3nzLLdva5NRa1JqjT459ct1118OzLfzF4yIqA8WVtm1/9NHHQIZgQq+evCqY5fQBN8OaFKiepdULZM/QvNu6sI8RK8V0TdY+psVzujijpb0FTiUBWzJx4EBGJxn7G1TUwob6pqMjI+1j58mJyffe+w/WgGhCj4mfH0iYPGvleeeR9tYCww69XyHdZnNy8mD6rCTSeRoPrHQyogZydUCTiTHIP5QlPHbsU4AWFCSOlyobRvR7PJ7rHZyc1FMJDF508UWk4+4OWFyFsNloqGfTkomUU82BqdDYhZ/+53bEGSQpk+TbVA1M646y1UKqBqg8Fe77v7TP/gF1Rqls0spqNnJFB5QqA4YMmyrNgdWTwBxkkUHvSPNiMw5RKtDXsXxQrmClwHZsSPcgzmGY2pvKQS1c66JNcqAvwrIDs4eLT3VRtSSZrP3JoDSYopRKXFLo0K5QLHTabgnu4plgrAFZAepEHxMRLLRAySLKbNsChtuUdXwGLWDQUjYSH2SMttjxILWP2Iwtai6xbgB2Wjuc3I44QwJRahOcDYzcoZHAehwwfe6cfArwk2GnefryAqhgjrxE+qUY6Xm85V4WVWGFSK1X9Ep0Zma2PSKsAal6MgcVFES0rDnXc0H01TkVW6TZJH1xbm4OGs2my5QUuPAMIvVsyRRTaGiIwChLYrgvSOeKifTr2MoSoM8xDFiKLY9d9JN0tBU6p0H7pNQi1QpqGmZnZ9H+uS6WI6Xy5jHBh+Uwo8YU3wbzaaT+xS8gFoxkIaPQhsaFDOAJgqevqV2ekaQyYAglc0R/BErKpdKaNWuC6CTck0mc3jYQx/ZOMAAaGx/rtN9P1YrFXqVScd26dQlaYPVzzteuXUPaGonvXHnlQKUCtkTV81rqFomCoPkamJoXmNeJyUmMSikVeU5U0gg1TOq5y89dtnSpi5XUYAZMjTeEqmcMbFij0Xj33XdVBKLOEMsO2fTGa66GTB8Ly4JHA0aBeHsdwWBm//53Dh8+7FvI/I44m6xUmcQ2KUL8KknUIIJObbRQrSAQkqCrVq1+7rk/LWxKq4p2auIJqPszz/yxLdeZKnjzzVvhMx9Cfvv4zod/8zD4OiHcHFsG2s6gnEC4t95669abbuwK3YkTJzZ979rZ07O+N++Ecdu2bfPhDuChhx7aufN3EDJ4EOa2T0dkZ3oyDygkstUewZcJOkbP8+ZZlOnEiQqUSfDJiMc7LoPp6WnwxSDfWFbTGTV+41uGln380+NBvT/fw36JTyHketeBt4TOaU8qx4oRDj4tc75G8dn0ZxCNQDwY36tJ5B/ptKvXKO2pqRNaxHp22xHaRgAyx5UvAPwIB+RzZkp61A/sRY/H4OCp+x/41Yt/e9EJD/iYgWC0sZC1p4PIa9UqJu9SEplHqCq+ZCSQVC8Ht/y3vcEPiHZGQzO4Y8d9r736us9gWhEs2q+PtryNTJHS2lwNQk4/I9ZWsIfjldHBA3+Pznh3LtoLzWkEOonoDL1iEnBCeqRazxAoUK1eq9aqRKZahsQukxG8m3tNMojZclLiVwB7k7YMCibK33Q6wVmdqyGDquSbEdi2SDXW1kliFHPikVXamsZTVTqmEXNpJ1uZPyZTewFBBxYd57eMjfnwuTClIFnchsk/fpiqM1hsofc2FC5EqYL0Xt5n0BTatoM1Cwq5i+ffSCnBJCRJEsZEBnWckJKskzKYQCnhQE+1P5LrzEt8TOoL2mJqd5u1H8F2bGTQwTp2cgaTG98Gd2btUyWIPoMWswmvlQuoLcL8h0PCI0Xi1DzbwpMRVpE1a7UYMlEnJVIUnKlMHgiqQYegQIBoRQM6FFyPxZUHJ6ZkS7jbQnJEuzInEHBAA/I1oFJXOnpQi/wQFFOaMBfQ7mEE3wrWwI1KTzQNmxSmZma1KDFVifOPWgkxXRVBlST1pAxXwZ86usJhpQL9pGNhOT4CPD0zMwPI4FkYoV6vZY0QmPkakOS5HqVpK0qm/pb+NoMvE6GXGTAIoratoa83RI1RyJZV8VYvI+csIS4QFbsh1EEgJuzhcTPxYANrhOt4JUsd+0NfyYYHCbMDCRI6cL6sz3olJuP/Sos6jBLOyqvDnq2gZX355ZcVigXHtl3XW7lyxUJbwW9u2LB4dLHjILrzz1/dwwiawnXr1lVrNTAUSnX8XY6Uk6kxFTRLuDRyPqrQnVUM11cWDQ9fccW39KlpUMcVK5aTHK+bmCOUSqWNGzfCygHbBjnf2jVrCWmX8l966SXAlWKQG4XAtD3GWMMAvRpVTdt2nP8B7hl6pajpQT8AAAAASUVORK5CYII=\"><br>" ) ;
}

void sendIcon(WiFiClient client)
{
	client.print( (char*)"<link href='data:image/x-icon;base64," ) ;
	client.print( (char*)IconBase64 ) ;
	client.print( (char*)"' rel='icon' type='image/x-icon' />" ) ;
}


void sendMainForm(WiFiClient client)
{
	client.println("HTTP/1.1 200 OK") ;
	client.println("Content-Type: text/html; charset=UTF-8") ;
	client.println("") ;
	client.println("<!DOCTYPE HTML>") ;
	client.println("<html>") ;
	client.println("<head>") ;
	client.println("<title>Frsky Neo</title>") ;

	sendIcon(client) ;
  					
	client.println("</head>") ;
	client.println("<body>") ;
	sendLogo(client) ;
	client.println("</br></br>") ;
		
	client.println("<form method=\"post\" >");
	
	client.println("<br><br><input type=\"submit\" value=\"Update\" name=\"Update Firmware\">");
	client.println("<br><br><input type=\"submit\" value=\"Upload File(s)\" name=\"Upload File(s)\">");
	client.println("</form>");

	client.println("</br>") ;

	client.println("</br>") ;
	client.println("</body>") ;
	client.println("</html>") ;
	
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
	client.println("<title>Frsky Neo</title>") ;
		
	sendIcon(client) ;
  					
	client.println("</head>") ;
	client.println("<body>") ;
	sendLogo(client) ;
	client.println("</br></br>") ;
		
	//check the LED status
//	if (LedStatus == 1)
//	{
//	  client.print("LED is Off</br>") ;
//	  client.println("Turn the LED <a href=\"/LED=ON\">ON</a></br>") ;
//	}
//	else if (LedStatus == 0)
//	{
//	  client.print("LED is On</br>") ;
//	  client.println("Turn the LED <a href=\"/LED=OFF\">OFF</a></br>") ;
//	}
		
	client.println("<form action=\"upload\" method=\"post\" enctype=\"multipart/form-data\">");
	client.println("<br><label for=\"Destination\">Choose destination:</label>");
  client.println("<select name=\"Destination\" id=\"Destination\">");

	sendDestOption( client, "SPIFFS", DestinationIndex == DEST_SPIFFS ) ;
	sendDestOption( client, "voice/system", DestinationIndex == DEST_V_SYSTEM  ) ;
	sendDestOption( client, "voice/user", DestinationIndex == DEST_V_USER  ) ;
	sendDestOption( client, "TEXT", DestinationIndex == DEST_TEXT  ) ;
	sendDestOption( client, "SCRIPTS", DestinationIndex == DEST_SCRIPTS  ) ;
	sendDestOption( client, "UPDATE", DestinationIndex == DEST_UPDATE  ) ;
  
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
	client.println("<title>Frsky Neo</title>") ;
		
	sendIcon(client) ;
  					
	client.println("</head>") ;
	client.println("<body>") ;
	sendLogo(client) ;
	client.println("</br></br>") ;
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



uint8_t LastFileData[260+128] ;
uint32_t LastDataIndex ;
uint32_t BoundaryIndex ;
uint32_t BytesWritten ;
uint32_t TotalBytesWritten ;

uint32_t writeToFile( uint8_t byte )
{
	uint32_t localLastDataIndex ;
	
	localLastDataIndex = LastDataIndex ;

	// Start looking for boundary
	LastFileData[localLastDataIndex] = byte ;
	if ( ++localLastDataIndex >= BoundarySize )
	{
		uint32_t x ;
		uint32_t y ;
		uint32_t z ;
		char *position ;
		y = localLastDataIndex - BoundarySize ;
		if ( LastFileData[y] == BoundaryString[0] )
		{
			if ( position = strstr( (char *)&LastFileData[y], (char *)BoundaryString ) )
			{
						
				// Found the boundary
				x = position - (char *)LastFileData ;
				fileWrite( LastFileData, x ) ;
				BytesWritten += x ;
				TotalBytesWritten += x ;
				Serial.println("End of File") ;
				// close file
				fileClose() ;
				LastDataIndex = localLastDataIndex ;
				return 1 ;
			}
		}
		if ( y >= 256 )
		{
			fileWrite( LastFileData, y ) ;
			BytesWritten += y ;
			TotalBytesWritten += y ;
			ClientTimer = FlushTimer = millis() ;

			memmove( LastFileData, &LastFileData[y], BoundarySize ) ;
			localLastDataIndex = BoundarySize ;
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
							UploadState = UP_HEADER2 ;
							// Close file
							FlushTimer = millis() ;
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
									DestinationIndex = DEST_SPIFFS ;
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
										DestinationIndex = DEST_V_SYSTEM ;
									}
					  			else if ( position = strstr( Request, "voice/user" ) )
									{
										DestinationIndex = DEST_V_USER ;
									}
					  			else if ( position = strstr( Request, "TEXT" ) )
									{
										DestinationIndex = DEST_TEXT ;
									}
					  			else if ( position = strstr( Request, "SCRIPTS" ) )
									{
										DestinationIndex = DEST_SCRIPTS ;
									}
					  			else if ( position = strstr( Request, "UPDATE" ) )
									{
										DestinationIndex = DEST_UPDATE ;
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
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( position = strstr( Request, "octet-stream" ) )
								{
									UploadState = UP_DATA1 ;
									Serial.println("Going to UP_DATA1") ;
									UploadFileSize -= RequestLength ;
									RequestLength = 0 ;
									Request[0] = '\0' ;
								}
					  		else if ( position = strstr( Request, "text/plain" ) )
								{
									UploadState = UP_DATA1 ;
									Serial.println("Going to UP_DATA1") ;
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
						
//						// Send response
//					  if (strstr( Request, "/LED=ON" ) )
//						{
//  						LedStatus = 0 ;
//							digitalWrite( 19, 1 ) ;		// Backlight
//  					}
			
//					  if (strstr( Request, "/LED=OFF" ) )
//						{
//  						LedStatus = 1 ;
//							digitalWrite( 19, 0 ) ;		// Backlight
//  					}

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
							case 3 :
								sendMainForm( Client ) ;
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
						if ( DestinationIndex == DEST_UPDATE )	// UPDATE
						{
							requestRestart() ;
						}
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

//void updateFromSpiffs()
//{
//    File file = SPIFFS.open("/erskytx-neo.bin");
  
//    if(!file)
//		{
//        Serial.println("Failed to open file for reading");
//        return;
//    }
       
//    Serial.println("Starting update..");
        
//    size_t fileSize = file.size();
 
//    if(!Update.begin(fileSize))
//		{
//       Serial.println("Cannot do the update");
//       return;
//    };
 
//    Update.writeStream(file);
 
//    if(Update.end())
//		{
//      Serial.println("Successful update");  
//    }
//		else
//		{
//      Serial.println("Error Occurred: " + String(Update.getError()));
//      return;
//    }
     
//    file.close();
 
//    Serial.println("Reset in 4 seconds...");
//}

  
