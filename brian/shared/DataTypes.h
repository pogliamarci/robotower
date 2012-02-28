#ifndef DATATYPES_H_
#define DATATYPES_H_

#ifdef UCHAR // Let's get rid of stupid define in DCDT...
#undef UCHAR
#endif

#include <stdlib.h>
#include <vector>
#include <string>
#include <bitset>
#include <sys/time.h>

typedef void VOID; /*!< Void data type, that is a typeless type. Empty type.  */

/*! \file DataTypes.h
 *  \brief Some useful datatypes
 *
 *  This file contains the definitions of some useful datatypes, such as BYTE of UINT32.\n
 *  Many data types have aliases (UCHAR, BYTE, UINT8), so to consider the semantic of the type in addition to the effective fact that they're unsigned char.\n
 *  So a UCHAR represents a character, like 'a' or '&', a BYTE represents 8 bits (without a defined meaning) while a UINT8 represents a small number, like 1 or 55.\n
 *  IMHO this contributes to make the code easier to read and to understand.
*/
typedef char CHAR; /*!< Char. */
typedef unsigned char UCHAR;/*!< Unsigned 8 bit integer. And yes, it's a BYTE.*/

typedef unsigned char BYTE; /*!< Unsigned 8 bit integer, aka BYTE. */
typedef unsigned short int WORD; /*!< Unsigned 16 bit integer, aka WORD. */
typedef unsigned int DWORD; /*!< Unsigned 32 bit integer, aka DWORD. */
typedef unsigned long long int QWORD; /*!< Unsigned 64 bit integer, aka QWORD. */

typedef unsigned char UINT8;/*!< Unsigned 8 bit integer. And yes, it's a BYTE.*/
typedef unsigned short int UINT16; /*!< Unsigned 16 bit integer. */
typedef unsigned int UINT32; /*!< Unsigned 32 bit integer. */
typedef unsigned long long int UINT64; /*!< Unsigned 64 bit integer. */

typedef char INT8;/*!< Signed 8 bit integer.*/
typedef short int INT16; /*!< Signed 16 bit integer. */
typedef int INT32; /*!< Signed 32 bit integer. */
typedef long long int INT64; /*!< Signed 64 bit integer. */

typedef float FLOAT32; /*!< Single precision IEEE 754 (32 bit) floating point number. */
typedef double FLOAT64; /*!< Double precision IEEE 754 (64 bit) floating point number. */

typedef void RAW; /*!< Raw data type, aka void. Useful in the RAW* variant. */

typedef bool BOOL; /*!< Boolean value, aka true or false */

typedef struct timeval TIMESTAMP; /*!< Timestamp. */

typedef std::basic_string<char> String; /*!< A string of char, AKA a string. */
typedef std::vector<String> StringsVector; /*!< A Vector of String. */


#endif /*DATATYPES_H_*/
