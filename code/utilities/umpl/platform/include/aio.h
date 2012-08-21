/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: aio.h 5629 2011-06-11 03:13:08Z mcaramello $
 *
 *******************************************************************************/

#ifndef _AIO_H
#define _AIO_H

#include "mltypes.h"
#ifdef WIN32
#include <windows.h>
#endif
#ifdef LINUX
#include <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif


    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /* - Error Codes. - */

#define AIO_SUCCESS 0
#define AIO_ERROR   1

    /* ---------- */
    /* - Enums. - */
    /* ---------- */

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

#ifdef WIN32
#define AIO_RWRSP_SIZE   2048
#endif
#ifdef LINUX
#define AIO_RWRSP_SIZE   512
#endif

#define AIO_RBUFF_MAX    128
#define AIO_RBUFF_SIZE   1024

#define AIO_RBUFF_DYNAMIC  1

#define AIO_INT_MAX        5
#define AIO_INT_SIZE       1

#if defined(WIN32)
    typedef struct {

        char port[128];
        unsigned int     baud;

        HANDLE  aioHndl;
        HANDLE  hDevice;                    // handle to the drive to be examined
        OVERLAPPED ovRead;                  // overlap structure for reading I/O
        OVERLAPPED ovWrite;                 // overlap structure for writing I/O
        HANDLE  hReadSemaphore;             // Semaphore signaling data ready
        HANDLE  hReadEvent;                 // event for detecting received data

        int     rbLen;
        int     rbCnt;

        unsigned char readBuffer[2048];
        unsigned char writeBuffer[1024];

        int     rBuffRIndex;
        int     rBuffWIndex;
#if !AIO_RBUFF_DYNAMIC
        unsigned char      rBuff[AIO_RBUFF_MAX][AIO_RBUFF_SIZE];   // Data Stream Buffer
#else
        unsigned char **rBuff;
#endif
        int rBuffMax;
        int rBuffNumBytes;

        unsigned char runThread;

        int       rwRspCnt;
        unsigned char rwRsp[AIO_RWRSP_SIZE];   // Read/Write Response Buffer
        unsigned char rwRspReady;

        unsigned char intSrc;
        unsigned int  intCnt[AIO_INT_MAX];

#define AIO_MODE_DS    0x00000001
#define AIO_MODE_INT   0x00000002
#define AIO_MODE_DM    0x00000004

        unsigned int mode;

        void(*dataStreamHandler)(unsigned char*, int); // StreamHandler callback

    } tAIOVars,     // new type definition
      AIO_Vars_t;   // backward-compatible type definition

#elif defined(LINUX)

    typedef struct {

        MLU16     port;
        MLU32     baud;
        
        void*  aioHndl;
        int  hDevice;                    // handle to the drive to be examined 
        pthread_mutex_t read_mutex;
        pthread_cond_t  read_cv;
        
        
        int     rbLen;            
        int     rbCnt;                 
        
        MLU8      readBuffer[2048];
	MLU8      writeBuffer[1024];
        
        int     rBuffRIndex;                            
	int     rBuffWIndex;                  
#if !AIO_RBUFF_DYNAMIC 
        MLU8      rBuff[AIO_RBUFF_MAX][AIO_RBUFF_SIZE];   // Data Stream Buffer
#else
        MLU8     *rBuff;
#endif
        int       rBuffMax;
	int       rBuffNumBytes;
        
        MLU8      runThread;
        MLU8      autoProcess;
        
        int       rwRspCnt;
        MLU8      rwRsp[AIO_RWRSP_SIZE];                  // Read/Write Response Buffer
        MLU8      rwRspReady;
        
        MLU8      intSrc;
        MLU32     intCnt[AIO_INT_MAX];
        
#define AIO_MODE_DS    0x00000001
#define AIO_MODE_INT   0x00000002
#define AIO_MODE_DM    0x00000004

        MLU32     mode;
        
        void(*dataStreamHandler)(MLU8*);                 // StreamHandler callback
    } AIO_Vars_t;
#endif

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    int            AIOSetCom         (char const * port, unsigned long dwBaud);
    //unsigned short AIOSetCom         (unsigned short nPort, unsigned int dwBaud);
    int            AIOInit           (char const * port, unsigned long dwBaud);
    int            AIOUartRead       (void * lpBuffer, unsigned long nBufferSize, unsigned long * lpBytesReturned);
    int            AIOUartWrite      (void * lpBuffer, unsigned long nBufferSize, unsigned long * lpBytesWritten);
    int            AIOIsOpen         (void);
    int            AIOUartFlush      (void);
    unsigned long  AIOUartGetQStatus (void);

    int AIOOpen           (unsigned char autoProcess, unsigned char createThread);
    int AIOClose          (void);
    int AIODeviceIoControl(void);
    int AIORead           (void);
    int AIOWrite          (void);
    int AIOSetBufferSize  (unsigned short bufferSize);
    int AIOUpdateBuffer   (void);
    int AIOBufferUpdate   (void);
    int AIOHandler        (void);
    int AIOReadBuffer     (unsigned short cnt, unsigned char bufferMode, unsigned char *rBuff);
    int AIOEmptyBuffer    (void);
    int AIOPktsInBuffer   (unsigned short *pktsInBuffer);
    int AIOBytesInBuffer  (int *bytesInBuffer);
    int AIOCreateMutex    (void);
    int AIOLockMutex      (void);
    int AIOUnlockMutex    (void);

    unsigned long AIOUartGetQStatusRwRsp(void);
    inv_error_t AIOUartReadRwRsp(void * lpBuffer, unsigned long nBufferSize, unsigned long* lpBytesReturned);
    unsigned long AIOUartWaitRwRsp(unsigned long timeout);
#ifdef WIN32
    void AIOSetStreamHandler( void(*func)(unsigned char*, int) );
#endif

    void AIOStopThread    (void);
    void AIORunThread     (void);


#ifdef __cplusplus
}
#endif

#endif // _AIO_H
