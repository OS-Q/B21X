
#include "include.h"
uint16_t SubMatch(uint8_t sub[],uint8_t sub_len,uint8_t tar[],uint16_t tar_len) 
{ 
	uint16_t i = 0, j = 0,index = 0; 
	
	while((j<sub_len)&&((i+sub_len)<=tar_len))
	{
		if(tar[i+j] == sub[j]) 
		{
			j++;
			if(j == sub_len)
			{
				index = i+j;
				break;
			}
		}
		else 
		{ 
			i ++; 
			j = 0; 
		}
	}
	
	return index; 
} 
uint8_t AsciiToHexVal(uint8_t h_b,uint8_t l_b)
{
	uint8_t hex_val;
	
	if((h_b >= '0')&&(h_b <= '9'))
	{
		hex_val = h_b - '0';
	}
	else if((h_b >= 'A')&&(h_b <= 'F'))
	{
		hex_val = h_b - 'A' + 10;
	}
	else if((h_b >= 'a')&&(h_b <= 'f'))
	{
		hex_val = h_b - 'a' + 10;
	}
	
	hex_val <<= 4;
	
	if((l_b >= '0')&&(l_b <= '9'))
	{
		hex_val |= l_b - '0';
	}
	else if((l_b >= 'A')&&(l_b <= 'F'))
	{
		hex_val |= l_b - 'A' + 10;
	}
	else if((l_b >= 'a')&&(l_b <= 'f'))
	{
		hex_val |= l_b - 'a' + 10;
	}

	return hex_val;
}
uint8_t IsValidNum(uint8_t data[],uint8_t len)
{
	uint8_t i,res = true;
	
	for(i=0;i<len;i++)
	{
		if((data[i] < 0x30)||(data[i] > 0x39))
		{
			break;
		}
	}
	
	if(i != len)
	{
		res = false;
	}

	return res;
}
uint8_t IsValidCharFloatNum(uint8_t data[],uint8_t len)
{
	uint8_t i,res = true;
	
	for(i=0;i<len;i++)
	{
		if(((data[i] >= 0x30)&&(data[i] <= 0x39))||(data[i] == '.'))
		{
			
		}
		else
		{
			break;
		}
	}
	
	if(i != len)
	{
		res = false;
	}

	return res;
}
uint8_t U8XorCheck(uint8_t data[],uint16_t len)
{
	uint8_t result = 0;
	uint16_t i;
	
	for(i=0;i<len;i++)
	{
		result ^= data[i];
	}
	
	return result;
}
uint8_t U8SumCheck(uint8_t *dst,uint16_t size)
{
	uint8_t sum = 0;

	while( size-- )
    {
        sum += *dst++;
    }
	
	return sum;
}
uint8_t StrLen(const uint8_t *str,uint8_t max_count)
{
	uint8_t len;
	
	len = 0x00;
	while(*str != '\0')
	{
		str++;
		len++;
		if(max_count != 0)
		{
			if(len >= max_count)
			{
				len = 0;
				break;
			}
		}
	}
	return len;
}
void MemCpy( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    while( size-- )
    {
        *dst++ = *src++;
    }
}
void MemCpyRev( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}
void MemSet( uint8_t *dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}
uint8_t MemCmp( uint8_t *mem_1, uint8_t *mem_2, uint16_t size )
{
    while( size-- )
    {
        if(*mem_1++ != *mem_2++)
		{
			return false;
		}
    }
	return true;
}
uint32_t U8ToUint32(uint8_t data[])
{
	uint32_t temp_u32;
	
	temp_u32  = data[3] << 24;
	temp_u32 |= data[2] << 16;
	temp_u32 |= data[1] << 8;
	temp_u32 |= data[0];
	
	return temp_u32;
}
void Uint32ToU8(uint8_t data[],uint32_t val)
{
	data[3] = val >> 24;
	data[2] = val >> 16;
	data[1] = val >> 8;
	data[0] = val;
}
uint32_t U8ToUint32Rev(uint8_t data[])
{
	uint32_t temp_u32;
	
	temp_u32  = data[0] << 24;
	temp_u32 |= data[1] << 16;
	temp_u32 |= data[2] << 8;
	temp_u32 |= data[3];
	
	return temp_u32;
}
void Uint32ToU8Rev(uint8_t data[],uint32_t val)
{
	data[0] = val >> 24;
	data[1] = val >> 16;
	data[2] = val >> 8;
	data[3] = val;
}
uint8_t Logic1Bits(uint8_t* array,uint8_t array_index)
{
    uint8_t i,j,bit_num = 0;
	
	for(i=0;i<array_index;i++)
	{
		for(j=0;j<8;j++ )
		{
			if( ( array[i] & ( 1 << j ) ) == ( 1 << j ) )
			{
				bit_num++;
			}
		}
	}

    return bit_num;
}