#ifndef HEVC_UTIL_H
#define HEVC_UTIL_H

#define ret_void_out_of_range(val, _min, _max)			if (val < _min || val >= _max) return
#define ret_false_out_of_range(val, _min, _max)			if (val < _min || val >= _max) return false
#define ret_FALSE_out_of_range(val, _min, _max)			if (val < _min || val >= _max) return FALSE
#define ret_NULL_out_of_range(val, _min, _max)			if (val < _min || val >= _max) return NULL
#define ret_value_out_of_range(val, _min, _max, ret)	if (val < _min || val >= _max) return ret
#define ret_NULL_if_null(p)								if (NULL == (p)) return NULL
#define ret_FALSE_if_null(p)							if (NULL == (p)) return FALSE
#define ret_false_if_null(p)							if (NULL == (p)) return false
#define ret_void_if_null(p)								if (NULL == (p)) return
#define ret_value_if_null(p, val)						if (NULL == (p)) return (val)

#define _swap(a, b)			{ a^=b^=a^=b }
#define _toggle(a)			((a)^=1)
#define _rotate(a, n)		((a)=((a)+1)%(n))
#define either(a,b)			((a) || (b))
#define eithern(a,b,n)		((a) == (n) || (b) == (n))
#define	neither(a,b)		(!(a) && !(b))
#define neithern(a,b,n)		((a) != (n) && (b) != (n))
#define neithernn(a,n1,n2)	((a) != (n1) && (a) != (n2))
#define both(a,b)			((a) && (b))
#define bothn(a,b,n)		((a) == (n) && (b) == (n))
#define oneof3(a,b,c)		((a) || (b) || (c))
#define oneof4(a,b,c,d)		((a) || (b) || (c) || (d))
#define oneof3n(a,b,c,n)	((a) == (n) || (b) == (n) || (c) == (n))
#define oneof4n(a,b,c,d,n)	((a) == (n) || (b) == (n) || (c) == (n) || (d) == (n))

#define _MAX_QWORD		0xFFFFFFFFFFFFFFFF
#define _MAX_DWORD		0xFFFFFFFF
#define _MAX_WORD		0xFFFF
#define _MAX_BYTE		0xFF
#define _MAX_NIBBLE		0xF

#if WIN32 || _WIN64

#ifndef UNICODE
#include <string>
#define __stdstr	std::string
#define __stdstrstream	std::stringstream
#else // UNICODE
#include <xstring>
#define __stdstr	std::wstring
#define __stdstrstream		std::wstringstream
#endif

#else // WIN32 || _WIN64
#include <string>
#endif // WIN32 || _WIN64

#if WIN32 || _WIN64
/******************* win32 API helper macro and functions ***************/
#define _OpenRead(filename)			CreateFile(filename, GENERIC_READ, FILE_SHARE_READ, \
	NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)
#define _OpenWrite(filename)		CreateFile(filename, GENERIC_WRITE, FILE_SHARE_WRITE, \
	NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
// open share read/write
#define _OpenReadrw(filename)		CreateFile(filename, GENERIC_READ, \
	FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)
#define _OpenWriterw(filename)		CreateFile(filename, GENERIC_WRITE, \
	FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
#endif // // WIN32 || _WIN64

#ifdef HEVC_BASE_EXPORTS
#define HEVC_BASE_API __declspec(dllexport)
#else // FRAMEWORK_EXPORTS
#define HEVC_BASE_API __declspec(dllimport)
#endif

namespace hevc { namespace util {

	HEVC_BASE_API bool	MakeDirectory( const TCHAR* szDirectory );
	HEVC_BASE_API bool	IsDirExist( const TCHAR* szDirectory );
	HEVC_BASE_API bool	IsFileExist( const TCHAR* szFileName );

	HEVC_BASE_API bool	CopyFile( const TCHAR* szSourceFile, const TCHAR* szTargetFile );

} } // namespaces

#endif // HEVC_UTIL_H