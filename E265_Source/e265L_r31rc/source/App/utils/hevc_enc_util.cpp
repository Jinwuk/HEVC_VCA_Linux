#include "hevc_enc_util.h"

__stdstr strprintf(const char* fmt, ...)
{
	__stdstr strRet;
	va_list vlArgPtr;
	va_start(vlArgPtr, fmt);
	int nLength = _vscprintf(fmt, vlArgPtr) + 1;
	if (nLength > 1)
	{
		char * szTempText = new char[nLength];
		vsprintf_s(szTempText, nLength, fmt, vlArgPtr);
		strRet = szTempText;
		delete [] szTempText;
	}

	va_end(vlArgPtr);
	free(vlArgPtr);

	return strRet;
}

CBufferWriter::CBufferWriter()
{
	m_pbtBuffer = 0;
	m_nSize		= 0;
	m_nTail		= 0;
}

CBufferWriter::CBufferWriter( void* pData, u32 nLength )
{
	m_pbtBuffer = 0;
	m_nSize		= 0;
	m_nTail		= 0;

	From( pData, nLength );
}

CBufferWriter::~CBufferWriter()
{

}

bool	CBufferWriter::From( void* pData, n32 nLength )
{
	if ( ! pData || nLength <= 0 )
		return false;

	m_pbtBuffer = (unsigned char*)pData;
	m_nSize		= nLength;
	m_nTail		= 0;

	return true;
}

u32		CBufferWriter::Add(const void* pData, u32 nLength)
{
	if ( ! pData || nLength <= 0 )
		return 0;

	if ( GetSize() < GetCount() + nLength )
		// buffer overflow
		return 0;

	memcpy_s( GetCurrent(), GetFreeSize(), pData, nLength );

	return nLength;
}

u32		CBufferWriter::GetSize() const
{
	return m_nSize;
}

u32		CBufferWriter::GetCount() const
{
	return m_nTail;
}

u32		CBufferWriter::GetFreeSize() const
{
	return m_nSize - m_nTail;
}

unsigned char* CBufferWriter::GetCurrent()
{
	return (unsigned char*)GetPtr() + m_nTail;
}
