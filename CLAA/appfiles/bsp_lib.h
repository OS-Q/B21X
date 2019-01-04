#ifndef __BSP_LIB_H__
#define __BSP_LIB_H__

	#include <stdint.h>
	
	#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
	#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
	uint16_t 	SubMatch(uint8_t sub[],uint8_t sub_len,uint8_t tar[],uint16_t tar_len);
	uint8_t 	AsciiToHexVal(uint8_t h_b,uint8_t l_b);
	uint8_t 	IsValidNum(uint8_t data[],uint8_t len);
	uint8_t 	IsValidCharFloatNum(uint8_t data[],uint8_t len);
    uint8_t     U8XorCheck(uint8_t data[],uint16_t len);
	uint8_t 	U8SumCheck(uint8_t *dst,uint16_t size);
	uint8_t 	StrLen(const uint8_t *str,uint8_t max_count);
	void 		MemCpy( uint8_t *dst, const uint8_t *src, uint16_t size );
	void 		MemCpyRev( uint8_t *dst, const uint8_t *src, uint16_t size );
	void 		MemSet( uint8_t *dst, uint8_t value, uint16_t size );
	uint8_t 	MemCmp( uint8_t *mem_1, uint8_t *mem_2, uint16_t size );
	uint32_t 	U8ToUint32(uint8_t data[]);
	void 		Uint32ToU8(uint8_t data[],uint32_t val);
	uint32_t 	U8ToUint32Rev(uint8_t data[]);
	void 		Uint32ToU8Rev(uint8_t data[],uint32_t val);
	uint8_t 	Logic1Bits(uint8_t* array,uint8_t array_index);
#endif
