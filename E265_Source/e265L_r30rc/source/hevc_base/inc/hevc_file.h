#ifndef HEVC_FILE_IO_H_INCLIDED
#define HEVC_FILE_IO_H_INCLIDED

#include "./hevc_base.h"
#include "./hevc_util.h"
#include <stdio.h>

namespace hevc
{
namespace io
{

class HEVC_BASE_API CFileAccess
{
public:
	typedef enum { OpenRead=1, OpenWrite, OpenBoth }	Open_Mode;
	typedef enum { Seek_Cur, Seek_Begin, Seek_End }		Seek_Mode;

#ifdef HEVC_WINDOWS
	typedef enum { ShareNone, ShareRead, ShareWrite, ShareBoth } Share_Mode;
	typedef enum { FileOpenExisting, FileOpenNew }		Create_Mode;
#endif

	CFileAccess();

	CFileAccess(const char* szFileName
		, Open_Mode openMode
#ifdef HEVC_WINDOWS
		, Create_Mode fileOpenMode 
		, Share_Mode shareMode = ShareNone
#endif
		);

	~CFileAccess();

	u32		Read(void* pTarget, u32 nLength);
	u32		Write(const void* pSource, u32 nLength);

	u64		Seek(n64 nLength, Seek_Mode seekMode);

	int		Open(
		Open_Mode openMode
#ifdef HEVC_WINDOWS
		, Create_Mode fileOpenMode
		, Share_Mode shareMode
#endif // HEVC_WINDOWS
		);
	void	Close();

	bool	IsOpen();

	const char*	GetFileName()
	{
		return m_szFileName;
	}
	void	SetFileName(const char* szFileName);
	void	SetFileName(const WCHAR* szFileName);

	u64		GetFileSize();
	u64		GetFileRemainBytes();
private:
#ifdef HEVC_WINDOWS	
	HANDLE	m_hFile;
#else
	FILE*	m_pFile;
#endif
	u64			m_nCur;
	char		*m_szFileName;
};

} } // namespaces

#endif // HEVC_FILE_IO_H_INCLIDED

