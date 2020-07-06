/**
 * @file   typedef.h
 * @author trip2eee@gmail.com
 * @date   20 June 2020
 * @brief  type definition.
 * 
*/

#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#ifdef WIN32
typedef   signed char int8_t;
typedef unsigned char uint8_t;

typedef   signed short int16_t;
typedef unsigned short uint16_t;

typedef   signed int int32_t;
typedef unsigned int uint32_t;

typedef float        float32_t;
typedef double       float64_t;
#else

typedef float        float32_t;
typedef double       float64_t;
#endif

#endif
