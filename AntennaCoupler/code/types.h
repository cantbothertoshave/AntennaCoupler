

#pragma once

#include <stdint.h>		// might be gcc specific (there is a portable stdint on the net, though)
#include <stdbool.h>	// might be gcc specific (?)


//	Compiler complained about redefine of the uint8..uint32 types... since the stm32f10x.h typedef's this already...
//	I out-commented them in the stm32 file, since there, they ware stated as being for legacy support,
//	i.e. the library stuff seems not to use it itself anymore.
//
//	--> WRONG. It does. Grrr.
//

//typedef int64_t		i64;
//typedef int32_t		i32;
//typedef int16_t 	i16;
//typedef int8_t		i8;
//
//typedef uint64_t	u64;
//typedef uint32_t	u32;
//typedef uint16_t	uint16;
//typedef uint8_t		u8;




typedef int64_t		int64;
typedef int32_t		int32;
typedef int16_t 	int16;
typedef int8_t		int8;

typedef uint64_t	uint64;
typedef uint32_t	uint32;
typedef uint16_t	uint16;
typedef uint8_t		uint8;




