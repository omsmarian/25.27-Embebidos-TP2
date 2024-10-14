/***************************************************************************//**
  @file     macros.h
  @brief    Generic MACROs
  @author   Group 4: - Oms, Mariano
                     - Solari Raigoso, Agustín
                     - Wickham, Tomás
                     - Vieira, Valentin Ulises
 ******************************************************************************/

#ifndef _MACROS_H_
#define _MACROS_H_

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define BITGET(x,n) 			((x) & (1<<(n)))
#define BITSET(x,n) 			((x) |= (1<<(n)))
#define BITCLR(x,n) 			((x) &= ~(1<<(n)))
#define BITTOG(x,n) 			((x) ^= (1<<(n)))
#define BITINV(x,n) 			((x) = (x) ^ (1<<(n)))
#define BITVAL(x,n) 			((x) >> (n) & 1)
#define BITMSK(x,n) 			((x) & ~(1<<(n)))
#define BITSETVAL(x,n,v)		((x) = (x) ^ ((-(v) ^ (x)) & (1 << (n))))
#define BITCLVAL(x,n,v) 		((x) = (x) & ~(1 << (n)) | ((v) << (n)))
#define BITTOGVAL(x,n,v) 		((x) = (x) ^ ((v) << (n)))
#define BITINVVAL(x,n,v) 		((x) = (x) ^ ((v) << (n)))
#define BITMSKVAL(x,n,v) 		((x) = (x) & ~(1 << (n)))
#define BITRNG(x,m,n) 			((x) & (((1 << ((m) - (n) + 1)) - 1) << (n)))
#define BITRNGVAL(x,m,n,v) 		((x) = (x) & ~(((1 << ((m) - (n) + 1) - 1) << (n)) | ((v) << (n))))
#define BITCPY(x,m,n,y) 		((x) = (x) & ~(((1 << ((m) - (n) + 1) - 1) << (n)) | ((y) & ((1 << ((m) - (n) + 1) - 1)) << (n))))
#define BITCPYVAL(x,m,n,y) 		((x) = (x) & ~(((1 << ((m) - (n) + 1) - 1) << (n)) | ((y) << (n))))
#define BITINS(x,m,n,y) 		((x) = (x) & ~(((1 << ((m) - (n) + 1) - 1) << (n)) | ((y) << (n))))
#define BITEXT(x,m,n) 			((x) >> (n) & ((1 << ((m) - (n) + 1)) - 1))
#define BITEXTVAL(x,m,n,v) 		((x) = (x) & ~(((1 << ((m) - (n) + 1) - 1) << (n)) | ((v) << (n))))
#define BITCNT(x) 				((x) & 1 + BITCNT((x) >> 1))
#define BITREV(x) 				((x) = (((x) & 0x55555555) << 1) | (((x) & 0xAAAAAAAA) >> 1))
#define BITSWP2(x) 				((x) = ((x) & 0x33333333) << 2 | ((x) & 0xCCCCCCCC) >> 2)
#define BITSWP4(x) 				((x) = ((x) & 0x0F0F0F0F) << 4 | ((x) & 0xF0F0F0F0) >> 4)
#define BITSWP8(x) 				((x) = ((x) & 0x00FF00FF) << 8 | ((x) & 0xFF00FF00) >> 8)
#define BITSWP16(x) 			((x) = ((x) & 0x0000FFFF) << 16 | ((x) & 0xFFFF0000) >> 16)
#define BITPAR(x) 				((x) = ((x) & 0x55555555) + (((x) >> 1) & 0x55555555), \
		   						 (x) = ((x) & 0x33333333) + (((x) >> 2) & 0x33333333), \
								 (x) = ((x) & 0x0F0F0F0F) + (((x) >> 4) & 0x0F0F0F0F), \
								 (x) = ((x) & 0x00FF00FF) + (((x) >> 8) & 0x00FF00FF), \
								 (x) = ((x) & 0x0000FFFF) + (((x) >> 16) & 0x0000FFFF))

#define BITROLL_LEFT(x, b)		(x = ((x << 1) & 0xF) | (b & 1))
#define BITROLL_RIGHT(x, b)		(x = (x >> 1) | ((b & 1) << (CHAR_LENGTH - 1)))

#define CAP(x, min, max)		(x = (x < (min) ? (min) : (x > (max) ? (max) : x)))
#define ABS(x)					(((x) < 0) ? -(x) : (x))
#define CMP_IN(min, val, max)	(((min) < (val)) && ((val) < (max)))			// In:  (min, max)
#define CMP_OUT(min, val, max)	(((val) <= (min)) || ((max) <= (val)))			// Out: (-inf, min] U [max, +inf)

#define NUM2ASCII(n)			((n) + '0')
#define ASCII2NUM(c)			((c) - '0')

#define FOR(_i, _n)				for (uint8_t i = _i; i < _n; i++)

#define INT8_MAX_DIGS			4
#define INT16_MAX_DIGS			6
#define INT32_MAX_DIGS			11
#define INT64_MAX_DIGS			20
#define UINT8_MAX_DIGS			3
#define UINT16_MAX_DIGS			5
#define UINT32_MAX_DIGS			10
#define UINT64_MAX_DIGS			20
#define CHAR_MAX_DIGS			3
#define SHORT_MAX_DIGS			5
#define LONG_MAX_DIGS			10
#define LONGLONG_MAX_DIGS		20
#define UCHAR_MAX_DIGS			3
#define USHORT_MAX_DIGS			5
#define ULONG_MAX_DIGS			10
#define ULONGLONG_MAX_DIGS		20

#define M_PI					3.14159265358979323846

/*******************************************************************************
 ******************************************************************************/

#endif // _MACROS_H_
