#ifndef HEVC_BUFFER_BASE_H
#define HEVC_BUFFER_BASE_H

#include "./hevc_base.h"
#include "./hevc_types.h"
#include "./hevc_sync.h"

#ifdef HEVC_WINDOWS
#include <windows.h>
#endif // HEVC_WINDOWS

#ifdef HEVC_BASE_EXPORTS
#define HEVC_BASE_API __declspec(dllexport)
#else // HEVC_BASE_EXPORTS
#define HEVC_BASE_API __declspec(dllimport)
#endif // HEVC_BASE_EXPORTS

namespace hevc {
namespace buffer {

//---------------------------------------------------------------------
class HEVC_BASE_API CBufferBase
{
public:
	CBufferBase();
	virtual ~CBufferBase();

	virtual void*	GetPtr();
	virtual u32	GetLength();
protected:
	unsigned char*	m_pbtBuffer;
	u32	m_nSize;
};

class HEVC_BASE_API CVirtualBuffer : public CBufferBase
{
public:
	CVirtualBuffer();
	CVirtualBuffer(void* p, u32 nLen);
	virtual ~CVirtualBuffer();
};

class CLinearBuffer;
class CLinearBufferMt;

//**********************************************************************************//
class HEVC_BASE_API CBlockBuffer : public hevc::buffer::CBufferBase
{
public:
	CBlockBuffer( u32 blockLen )
		: m_bIs16ByteAligned(false)
	{
		// 16byte align code 삽입 >> SSE SIMD 코드 사용 안정성 확보
		if ( (blockLen%16) == 0 )
		{
			__declspec(align(16)) unsigned char* pBuffer = (unsigned char*)_mm_malloc( blockLen, 16 );
			__super::m_pbtBuffer = pBuffer;
			m_bIs16ByteAligned = true;
		}
		else
		{
			__super::m_pbtBuffer = new unsigned char[ blockLen ];
		}
		m_nSize = blockLen;

		for( int i = 0; i < MAX_FRAME_NUM_IN_IDR; i++ )
		{
			nFrameStartOffset[i] = 0;
			nFrameTypeInGop[i] = 0;
			nPicPresentationOrder[i] = 0;
			nPicDecodingOrder[i] = 0;
		}

		bNewStream = false;
	}
	virtual ~CBlockBuffer()
	// 16 byte align 삭제 코드 삽입
	{
		if ( m_bIs16ByteAligned )
		{
			_mm_free( __super::m_pbtBuffer );
			__super::m_pbtBuffer = 0;
			__super::m_nSize = 0;
		}
	}

	u32		GetFilled()			{ return m_nFilled; }
	void	SetFilled( u32 sz ) { m_nFilled = sz; }

	u64			nTimestamp[MAX_FRAME_NUM_IN_IDR];
	u64			nPTS;
	u64			nDTS;

	u32			nFrameType;
	n32			nFrameCount;		// Real frame count
	n32			nFrameStartOffset[MAX_FRAME_NUM_IN_IDR];		// GOP 내부 각 frame 위치 정보
	n32			nFrameTypeInGop[MAX_FRAME_NUM_IN_IDR];			// GOP 내부 각 frame 타입 정보
	n32			nPicPresentationOrder[MAX_FRAME_NUM_IN_IDR];	// GOP 내부 각 frame의 재생 순서
	n32			nPicDecodingOrder[MAX_FRAME_NUM_IN_IDR];		// GOP 내부 각 frame의 디코딩 순서

	// additional information
	bool		bNewStream; // hls segment가 새로 시작됨을 표시하거나 기타 다른 새로운 데이타가 시작될 경우 사용

	// hevc encoder dll interface data: type dll interface info
	void *EncoderIF;
	void *DLLFunction;

protected:
	u32	m_nFilled;
	bool	m_bIs16ByteAligned;
};

//---------------------------------------------------------------------
// cyclic byte queue
class HEVC_BASE_API CCircularBuffer : public CBufferBase
{
public:
	CCircularBuffer();
	virtual ~CCircularBuffer();
	bool	IsEmpty();
	bool	IsFull();
	bool	HasSpace(u32 nCount);
	u32		GetSize();
	u32		GetAllocSize();
	u32		GetCount();
	u32		GetEntered();
	u32		GetFreeSize();
	
	virtual bool	Create(u32 nAllocSize);
	virtual void	Release();
	virtual u32		Peek(void* pBuffer, u32 nCount);
	virtual u32		Pop(void* pBuffer, u32 nCount);
	virtual u32		Push(const void* pBuffer, u32 nCount);
	virtual u32		Throw(u32 nCount);
	virtual void	Clear();

	bool	IsSeperated(u32 nLength);
	void*	GetHeadPtr(u32 nLength);

	//---------------------------------------------------------------------
	// data get/set functions
	virtual u32	Peek(CLinearBuffer* const pBuf, u32 nCount);
	virtual u32	Pop(CLinearBuffer* const pBuf);
	virtual u32	Push(CLinearBuffer* const pBuf);
protected:
	u32	m_nEnteredCount;
	u32	m_nFront;
	u32	m_nBack;
};

//---------------------------------------------------------------------
// - for multi thread
class HEVC_BASE_API CCircularBufferMt : public CCircularBuffer
{
public:
	CCircularBufferMt();
	virtual ~CCircularBufferMt();


	virtual bool	Create(u32 nAllocSize);
	virtual void	Release();
	virtual u32		Peek(void* pBuffer, u32 nCount);
	virtual u32		Pop(void* pBuffer, u32 nCount);
	virtual u32		Push(const void* pBuffer, u32 nCount);
	virtual u32		Throw(u32 nCount);
	virtual void	Clear();

	//---------------------------------------------------------------------
	virtual u32		Peek(CLinearBuffer* const pBuf, u32 nCount);
	virtual u32		Pop(CLinearBuffer* const pBuf);
	virtual u32		Push(CLinearBuffer* const pBuf);
protected:
	util::CSyncLock	m_cs;
};

//---------------------------------------------------------------------
// non-cyclic byte queue
class CLinearBufferSync;

class HEVC_BASE_API CLinearBuffer : public CBufferBase
{
	friend class CLinearBufferSync;
public:
	CLinearBuffer(u32 nBufferSize);
	CLinearBuffer();
	virtual ~CLinearBuffer(void);

	virtual u32		Create(u32 nBufferSize);
	virtual u32		Create(const void* pBuf, u32 nBufferSize);
	virtual void	Clear();

	virtual u32		Get(void* pBuffer, u32 nLength);
	virtual u32		Set(const void* pBuffer, u32 nLength);
	virtual u32		Add(const void* pBuffer, u32 nLength);
	virtual u32		Throw(u32 nLength);
	virtual u32		AddTail(u32 nLength);
	virtual u32		AddHead(u32 nLength);

	u32		GetSize() const;
	u32		GetCount() const;
	u32		GetFreeSize() const;
	bool	IsStart() const;

	virtual void	Reset();
	virtual u32		GetLength() { return GetCount(); }
	void*			GetBuffer() const;
	void*			GetHead() const;
	void*			GetTail() const;

	const CLinearBuffer& operator = (const CLinearBuffer&);
protected:
	virtual void Lock() {};
	virtual void Unlock() {};
	bool	m_bLocked;

	void	Release();

	u32	m_nCurrent_Head;
	u32	m_nCurrent_Tail;
};

//---------------------------------------------------------------------
// - for multi thread
class HEVC_BASE_API CLinearBufferMt : public CLinearBuffer
{
public:
	CLinearBufferMt(u32 nBufferSize);
	CLinearBufferMt();
	virtual ~CLinearBufferMt(void);

	virtual u32		Create(u32 nBufferSize);
	virtual void	Clear();

	virtual u32		Get(void* pBuffer, u32 nLength);
	virtual u32		Set(const void* pBuffer, u32 nLength);
	virtual u32		Add(const void* pBuffer, u32 nLength);
	virtual u32		Throw(u32 nLength);

	virtual void	Reset();
protected:
	virtual void Lock();
	virtual void Unlock();

	util::CSyncLock m_cs;
};

// - WARNING !!!!!
// to synchronize CLinearBufferMt , use this class
// DO NOT USE this class with Create/Clear/Get/set/Add/Throw/Reset function at same namespace
// USE this class with GetTail/GetHead/AddTail/AddHead function set.
class HEVC_BASE_API CLinearBufferSync
{
public:
	CLinearBufferSync(CLinearBuffer*);
	~CLinearBufferSync();
private:
	CLinearBuffer*	m_pBuf;
};

} // namespace Data
} // namespace Framework


#endif // HEVC_BUFFER_BASE_H