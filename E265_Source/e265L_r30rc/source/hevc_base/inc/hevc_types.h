#pragma once

#ifndef HANDLE
typedef void*		HANDLE	;
#endif 

namespace hevc
{
#ifdef HEVC_WINDOWS
	#ifndef n8
		typedef __int8				n8;
	#endif // n8
	#ifndef u8
		typedef unsigned __int8		u8;
	#endif // u8
	#ifndef n16
		typedef __int16				n16;
	#endif // n16
	#ifndef u16
		typedef unsigned __int16	u16;
	#endif // u16
	#ifndef n32
		typedef __int32				n32;
	#endif // n32
	#ifndef u32
		typedef unsigned __int32	u32;
	#endif // u32
	#ifndef n64
		typedef __int64				n64;
	#endif // n64
	#ifndef u64
		typedef unsigned __int64	u64;
	#endif // u64
	#ifndef e32
		typedef float				r32;
	#endif
	#ifndef e64
		typedef double				r64;
	#endif
#else // !_MSC_VER
	#ifndef u64
		typedef unsigned long long	u64;
	#endif // u64
	#ifndef n64
		typedef long long			n64;
	#endif // n64
	#ifndef u32
		typedef unsigned int		u32;
	#endif // u32
	#ifndef n32
		typedef int					n32;
	#endif // n32
	#ifndef u16
		typedef unsigned short		u16;
	#endif // u16
	#ifndef n16
		typedef short				n16;
	#endif // n16
	#ifndef u8
		typedef unsigned char		u8;
	#endif // u8
	#ifndef n8
		typedef char				n8;
	#endif // n8
#endif // _MSC_VER
}