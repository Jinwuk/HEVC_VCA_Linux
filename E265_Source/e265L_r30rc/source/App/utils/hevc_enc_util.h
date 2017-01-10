#ifndef HEVC_UTIL_H_INCLUDED
#define HEVC_UTIL_H_INCLUDED

#include <hevc_base.h>
using namespace hevc;

#include <xiosbase>
#include <sstream>
#include <istream>
#include <hevc_buffer_base.h>

#define TRIM_SPACE " \t\n\v"

static inline std::string str_replace( std::string strSrc, std::string strFind, std::string strReplace )
{
	size_t offset = 0;

	while( true )
	{
		offset = strSrc.find( strFind, offset );

		if( std::string::npos == offset )
			break;

		else		
			strSrc.replace( offset, strFind.length(), strReplace );					
	}

	return strSrc;
}

static inline std::string trim(std::string& s,const std::string& drop = TRIM_SPACE)
{
	std::string r=s.erase(s.find_last_not_of(drop)+1);
	return r.erase(0,r.find_first_not_of(drop));
}

static inline std::string rtrim(std::string s,const std::string& drop = TRIM_SPACE)
{
	return s.erase(s.find_last_not_of(drop)+1);
}

static inline std::string ltrim(std::string s,const std::string& drop = TRIM_SPACE)
{
	return s.erase(0,s.find_first_not_of(drop));
}

static inline n32		hex2n32( std::string str )
{
	std::stringstream ss;

	if ( str.length() > 2 && (str.at(0) == '0')
		 && (str.at(1) == 'x' || str.at(1) == 'X') )
	{
		std::string strTemp = str.substr(2);
		ss << strTemp;
	}
	else
		ss << str;

	n32 nRet;
	ss >> std::hex >> nRet;

	return nRet;
}

static inline __stdstr	n322Hex( n32 val )
{
	std::stringstream ss;
	ss << std::hex << val;
	std::string result( ss.str() );
}

__stdstr strprintf(const char* fmt, ...);

//----------------------------------------------------------------------------------//
// auto_ptr for _mm_malloc, _mm_free
template<class _Ty>
struct aligned_auto_ptr_ref
{	// proxy reference for auto_ptr copying
	explicit aligned_auto_ptr_ref(_Ty *_Right)
		: _Ref(_Right)
	{	// construct from generic pointer to auto_ptr ptr
	}

	_Ty *_Ref;	// generic pointer to auto_ptr ptr
};

template< typename _Ty>
class aligned_auto_ptr
{	// wrap an object pointer to ensure destruction
public:
	typedef aligned_auto_ptr<_Ty> _Myt;
	typedef _Ty element_type;

	explicit aligned_auto_ptr(_Ty *_Ptr = 0) _THROW0()
		: _Myptr(_Ptr)
	{	// construct from object pointer
	}

	aligned_auto_ptr(_Myt& _Right) _THROW0()
		: _Myptr(_Right.release())
	{	// construct by assuming pointer from _Right auto_ptr
	}

	aligned_auto_ptr(aligned_auto_ptr_ref<_Ty> _Right) _THROW0()
	{	// construct by assuming pointer from _Right aligned_auto_ptr_ref
		_Ty *_Ptr = _Right._Ref;
		_Right._Ref = 0;	// release old
		_Myptr = _Ptr;	// reset this
	}

	template<class _Other>
	operator aligned_auto_ptr<_Other>() _THROW0()
	{	// convert to compatible auto_ptr
		return (aligned_auto_ptr<_Other>(*this));
	}

	template<class _Other>
	operator aligned_auto_ptr_ref<_Other>() _THROW0()
	{	// convert to compatible aligned_auto_ptr_ref
		_Other *_Cvtptr = _Myptr;	// test implicit conversion
		aligned_auto_ptr_ref<_Other> _Ans(_Cvtptr);
		_Myptr = 0;	// pass ownership to aligned_auto_ptr_ref
		return (_Ans);
	}

	template<class _Other>
	_Myt& operator=(aligned_auto_ptr<_Other>& _Right) _THROW0()
	{	// assign compatible _Right (assume pointer)
		reset(_Right.release());
		return (*this);
	}

	template<class _Other>
	aligned_auto_ptr(aligned_auto_ptr<_Other>& _Right) _THROW0()
		: _Myptr(_Right.release())
	{	// construct by assuming pointer from _Right
	}

	_Myt& operator=(_Myt& _Right) _THROW0()
	{	// assign compatible _Right (assume pointer)
		reset(_Right.release());
		return (*this);
	}

	_Myt& operator=(aligned_auto_ptr_ref<_Ty> _Right) _THROW0()
	{	// assign compatible _Right._Ref (assume pointer)
		_Ty *_Ptr = _Right._Ref;
		_Right._Ref = 0;	// release old
		reset(_Ptr);	// set new
		return (*this);
	}

	~aligned_auto_ptr()
	{	// destroy the object
		_mm_free( _Myptr );
	}

	_Ty& operator*() const _THROW0()
	{	// return designated value
#if _ITERATOR_DEBUG_LEVEL == 2
		if (_Myptr == 0)
			_DEBUG_ERROR("auto_ptr not dereferencable");
#endif /* _ITERATOR_DEBUG_LEVEL == 2 */

		return (*get());
	}

	_Ty *operator->() const _THROW0()
	{	// return pointer to class object
#if _ITERATOR_DEBUG_LEVEL == 2
		if (_Myptr == 0)
			_DEBUG_ERROR("auto_ptr not dereferencable");
#endif /* _ITERATOR_DEBUG_LEVEL == 2 */

		return (get());
	}

	_Ty *get() const _THROW0()
	{	// return wrapped pointer
		return (_Myptr);
	}

	_Ty *release() _THROW0()
	{	// return wrapped pointer and give up ownership
		_Ty *_Tmp = _Myptr;
		_Myptr = 0;
		return (_Tmp);
	}

	void reset(_Ty *_Ptr = 0)
	{	// destroy designated object and store new pointer
		if (_Ptr != _Myptr)
			_mm_free( _Myptr );
		_Myptr = _Ptr;
	}

private:
	_Ty *_Myptr;	// the wrapped object pointer
};
//----------------------------------------------------------------------------------//
// gdiplus helper
#ifdef HEVC_WINDOWS
#include <gdiplus.h>
using namespace Gdiplus;
class GdiplusInitializer {
public:
	GdiplusInitializer()
	{
		GdiplusStartupInput gdiplusStartupInput;
		GdiplusStartup(&m_gdiplusToken, &gdiplusStartupInput, NULL);
	}
	~GdiplusInitializer()
	{
		GdiplusShutdown(m_gdiplusToken);
	}
private:

	ULONG_PTR m_gdiplusToken;
};

static inline int GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{
	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes

	ImageCodecInfo* pImageCodecInfo = NULL;

	GetImageEncodersSize(&num, &size);
	if(size == 0)
		return -1;  // Failure

	pImageCodecInfo = (ImageCodecInfo*)(malloc(size));
	if(pImageCodecInfo == NULL)
		return -1;  // Failure

	GetImageEncoders(num, size, pImageCodecInfo);

	for(UINT j = 0; j < num; ++j)
	{
		if( wcscmp(pImageCodecInfo[j].MimeType, format) == 0 )
		{
			*pClsid = pImageCodecInfo[j].Clsid;
			free(pImageCodecInfo);
			return j;  // Success
		}    
	}

	free(pImageCodecInfo);
	return -1;  // Failure
}

/**
	----------------------------------------------------------------------------------
	@class		CBufferWriter
	@date		2013/12/19
	@author		wsseo
	@brief		
	@endcode
	----------------------------------------------------------------------------------
	@section MODIFYINFO 수정내용
@endcode
*/
class CBufferWriter : public hevc::buffer::CVirtualBuffer
{
public:
	CBufferWriter();
	CBufferWriter( void* pData, u32 nLength );
	virtual ~CBufferWriter();

	bool	From( void* pData, n32 nLength );
	u32		Add(const void* pData, u32 nLength);

	u32		GetSize() const;
	u32		GetCount() const;
	u32		GetFreeSize() const;
private:
	unsigned char* GetCurrent();
	u32		m_nTail;
};

#endif HEVC_WINDOWS

#endif // HEVC_UTIL_H_INCLUDED