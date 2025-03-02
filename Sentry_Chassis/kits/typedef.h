#ifndef TYPEDEF_H
#define TYPEDEF_H

#include "stdio.h"
#define __PACKED __attribute__((packed))
#define __COUNT(arr) sizeof(arr)/sizeof(arr[0])

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef union __PACKED{
	fp32 data;
	uint8_t bytes[4];
} union_fp32;

#endif
