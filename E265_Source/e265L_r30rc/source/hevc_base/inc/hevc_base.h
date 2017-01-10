
#ifndef HEVC_BASE_FRAMEWORK_H
#define HEVC_BASE_FRAMEWORK_H

#if WIN32 || _WIN64 // if windows


#ifdef WIN32
#define HEVC_WINDOWS
#endif // WIN32 

#ifdef _WIN64 
#define _FRAMEWORK_64
#endif

#include <windows.h>

#ifdef HEVC_BASE_EXPORTS
#define HEVC_BASE_API __declspec(dllexport)
#else // FRAMEWORK_EXPORTS
#define HEVC_BASE_API __declspec(dllimport)

#pragma message("library for windows")

#if (defined(_MSC_VER) && !defined(HEVC_FRAMEWORK_IMP_LIB))
	#ifdef _DEBUG
		#if (!defined(_WIN64))
			#pragma comment(lib, "hevc_base32d.lib")
			#pragma message("*** hevc_base32d.lib linked")
		#else // defined(_M_IX86)
			#pragma comment(lib, "hevc_base64d.lib")
			#pragma message("*** hevc_base64d.lib linked")
		#endif // defined(_M_IX86)
	#else // _DEBUG
		#if (!defined(_WIN64))
			#pragma comment(lib, "hevc_base32.lib")
			#pragma message("*** hevc_base.lib linked")
		#else // defined(_M_IX86)
			#pragma comment(lib, "hevc_base64.lib")
			#pragma message("*** hevc_base64.lib linked")
		#endif // defined(_M_IX86)
	#endif // _DEBUG
#define HEVC_FRAMEWORK_IMP_LIB
#endif // defined(_MSC_VER) && !defined(FRAMEWORK_IMP_LIB))

#endif // HEVC_BASE_EXPORTS

#else // if not windows

#ifdef __GNUC__
#define HEVC_GCC
#endif // __GNUC__

#endif // WIN32 || _WIN64 // if windows

namespace hevc {

enum EN_BASE_ERROR
{
	NoError					= 0, 
	Invalid_Parameter		= -100,
	Null_Pointer_Exception	= -101,
};

#define KILO		(1024)
#define MEGA		(KILO*KILO)
#define TERA		(MEGA*KILO)

#define IN
#define OUT
#define INOUT

/**
----------------------------------------------------------------------------------
@def     MAX_FRAME_NUM_IN_GOP
@param   GOP 단위 처리를 위한 GOP 최대 처리 개수를 정의합니다.
@brief   
@endcode
----------------------------------------------------------------------------------
*/
#define MAX_FRAME_NUM_IN_IDR                (64)

} // namespace hevc

#include "./hevc_types.h"
#include "./hevc_util.h"

#define HEVC_RET				hevc::n32


#endif // HEVC_BASE_FRAMEWORK_H
