#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

// 引入标准库，代替手动定义
#include <stdint.h> 
#include <stdbool.h>

// 如果旧代码里用了 unsigned char/short 等简写，可以保留下面的兼容宏
typedef uint8_t     uchar;
typedef uint16_t    ushort;
typedef uint32_t    uint;
typedef float fp32;
typedef double fp64;
#endif


