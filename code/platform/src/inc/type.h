#ifndef __TYPE_H__
#define __TYPE_H__
typedef int                 HANDLE;
typedef unsigned long       DWORD;
typedef unsigned long *     PDWORD;
typedef unsigned char       BYTE;
typedef unsigned char *     PBYTE;
typedef void *              LPVOID;
typedef const void * const  LPCVOID;
typedef char *              LPSTR;
typedef const char * const  LPCSTR;

#define TRUE   (1)                      /* Boolean true value.   */
#define FALSE  (0)                      /* Boolean false value.  */
typedef unsigned char       BOOL;        /* Boolean value type.   */
typedef unsigned long int   UINT32;      /* Unsigned 32 bit value */
typedef unsigned short      UINT16;      /* Unsigned 16 bit value */
typedef unsigned char       UINT8;       /* Unsigned 8  bit value */
typedef signed long int     INT32;       /* Signed 32 bit value   */
typedef signed short        INT16;       /* Signed 16 bit value   */
typedef signed char         INT8;        /* Signed 8  bit value   */
#endif
