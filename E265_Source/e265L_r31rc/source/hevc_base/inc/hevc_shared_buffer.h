#ifndef BLOCK_BUFFER_H_DEFINES
#define BLOCK_BUFFER_H_DEFINES

//----------------------------------------------------------------------------------//
/*
  * encoder handler set (device - encoder(a/v) - muxer data owner && buffer creator )
  * author : wonseok seo
  * first draft : 20130710 
//----------------------------------------------------------------------------------//
  * revision history 
  * 20130717 : IsAvailablInPhysical function added.
*/
//----------------------------------------------------------------------------------//

#include "./hevc_base.h"
#include "./hevc_types.h"

namespace hevc { namespace buffer {

enum EN_BUFFER_TYPE
{
	BT_Unknown = -1,
	BT_Block,
	BT_Circular
};

enum EN_BUFFER_ID
{
	BUF_AUDIO_RAW,		// YUV
	BUF_VIDEO_RAW,		// PCM
	BUF_VIDEO_VPP,		// VPP video
	BUF_AUDIO_ES,		// AAC packet
	BUF_VIDEO_ES,		// NAL...
	BUF_AUDIO_PES,		// Audio PES
	BUF_VIDEO_PES,		// Video PES
	BUF_TS,				// TS Packet
	BUF_RTSP,			// RTSP
	BUF_ASI,			// 
};

enum EN_SHARED_BUFFER_ERROR
{
	// EN_BASE_ERROR::NoError = 0
	SB_Not_Initialized		= -1,
	SB_Already_Initalized	= -2,
	SB_Underflow			= -3,
	SB_Overflow				= -4,
	SB_NotEnoughMemory		= -5,
};

//**********************************************************************************//
//----------------------------------------------------------------------------------//
// initialize library																//
// have to call at first of program initialized										//
// returns EN_SHARED_BUFFER_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	Init( void );

//----------------------------------------------------------------------------------//
// Check options of all of buffer size for all shared buffer instance will be		//
//	created in physical memory (not virtual)										//
// returns 0 : available, not 0 : unavailable										//
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	IsAvailablInPhysical( void );

//----------------------------------------------------------------------------------//
// Create buffer instances															//
//	- set buffer option by SetOption() functions before call this function			//
// returns EN_SHARED_BUFFER_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	Create( void );

//----------------------------------------------------------------------------------//
// cleanup block buffer list & cleanup circular buffer								//
//----------------------------------------------------------------------------------//
HEVC_BASE_API void	Cleanup( void );

//----------------------------------------------------------------------------------//
// release buffer instance
// have to call before program exit
//----------------------------------------------------------------------------------//
HEVC_BASE_API void	Release( void );

//----------------------------------------------------------------------------------//
// set Block buffer options
// returns EN_SHARED_BUFFER_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	SetOption( EN_BUFFER_ID, u32 nBlockSize, u32 nBlockCount );

//----------------------------------------------------------------------------------//
// set Circular buffer options
// returns EN_SHARED_BUFFER_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	SetOption( EN_BUFFER_ID, u32 nQueueSize );

//----------------------------------------------------------------------------------//
// get buffer type from buffer id
//----------------------------------------------------------------------------------//
HEVC_BASE_API EN_BUFFER_TYPE GetBufferType( EN_BUFFER_ID );

//----------------------------------------------------------------------------------//
// get buffer size(byte) from buffer id
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	GetBufferSize( EN_BUFFER_ID );

//----------------------------------------------------------------------------------//
// get buffer size(byte) from buffer id
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	GetBufferFreeSize( EN_BUFFER_ID );


//**********************************************************************************//
// block buffer manipulation functions

/**
----------------------------------------------------------------------------------
@enum    EN_FRAME_TYPE
@brief   struct S_Block_Info에 저장되는 data type 정의
@endcode
----------------------------------------------------------------------------------
*/
enum EN_FRAME_TYPE
{
	FT_I_Frame = 0,
	FT_B_Frame,
	FT_GPB_Frame,
	FT_SPS,
	FT_PPS,
	FT_GOP,			// GOP 단위 처리를 위한 정의
};

/**
----------------------------------------------------------------------------------
@struct          S_Block_Info
@brief           block buffer info
@endcode
----------------------------------------------------------------------------------
*/
struct S_Block_Info
{
	// buffer 정보
	void*		pBuffPtr;			// allocated buffer pointer
	u32			nBuffLength;		// buffer allocated size
	u32			nFilledLength;		// buffer used size
	const void*	pBufferHandle;		// meta data for buffer manipulation

	// video specific data
	n32			nFrameType;			// EN_FRAME_TYPE
	n32			nFrameCount;		// Real frame count
	u64			nTimestamp[MAX_FRAME_NUM_IN_IDR];
	n32			nFrameStartOffset[MAX_FRAME_NUM_IN_IDR];		// GOP 내부 각 frame 위치 정보
	n32			nFrameTypeInGop[MAX_FRAME_NUM_IN_IDR];			// GOP 내부 각 frame 타입 정보
	n32			nPicPresentationOrder[MAX_FRAME_NUM_IN_IDR];	// GOP 내부 각 frame의 재생 순서
	n32			nPicDecodingOrder[MAX_FRAME_NUM_IN_IDR];		// GOP 내부 각 frame의 디코딩 순서

	// additional information
	bool		bNewStream; // hls segment가 새로 시작됨을 표시하거나 기타 다른 새로운 데이타가 시작될 경우 사용
};

// block buffer utility functions
inline void* GetCurrentPtr(S_Block_Info* pBlock)
{
	if (!pBlock || !pBlock->pBuffPtr || pBlock->nBuffLength < 1) return 0;
	return (u8*)pBlock->pBuffPtr + pBlock->nFilledLength;
}

inline u32	GetRemain(S_Block_Info* pBlock)
{
	if (!pBlock || !pBlock->pBuffPtr || pBlock->nBuffLength < 1) return 0;
	return pBlock->nBuffLength - pBlock->nFilledLength;
}

//----------------------------------------------------------------------------------//
// Get 1 block from free list
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	GetFree( EN_BUFFER_ID, S_Block_Info* );

//----------------------------------------------------------------------------------//
// Set 1 block to free list
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	SetFree( EN_BUFFER_ID, const S_Block_Info* );

//----------------------------------------------------------------------------------//
// Get 1 block from used list
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	GetUsed( EN_BUFFER_ID, S_Block_Info* );

//----------------------------------------------------------------------------------//
// Set 1 block to used list
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	SetUsed( EN_BUFFER_ID, const S_Block_Info* );

//----------------------------------------------------------------------------------//
// Get 1 block allocated size 
// returns error code
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	Get1BlockSize( EN_BUFFER_ID );

//----------------------------------------------------------------------------------//
// Get block allocated count 
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API u32	GetBlockCount( EN_BUFFER_ID );

//----------------------------------------------------------------------------------//
// Get used block allocated bytes
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API u32	GetUsedBufferLength( EN_BUFFER_ID );

//----------------------------------------------------------------------------------//
// Get free block allocated bytes
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API u32	GetFreeBufferLength( EN_BUFFER_ID );


//**********************************************************************************//
// circular buffer manipulation functions
//----------------------------------------------------------------------------------//
// query circular buffer allocated size
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	GetFifoSize( EN_BUFFER_ID );

//----------------------------------------------------------------------------------//
// query circular buffer has free space to be used
//----------------------------------------------------------------------------------//
HEVC_BASE_API bool	HasFreeSpace( EN_BUFFER_ID, u32 );

//----------------------------------------------------------------------------------//
// Get circular buffer filled length (byte)
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	GetFilledLength( EN_BUFFER_ID );

//----------------------------------------------------------------------------------//
// Push data to circular buffer 
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	Push( EN_BUFFER_ID, const void* pDataPtr, u32 nLength );

//----------------------------------------------------------------------------------//
// Pop data from circular buffer 
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	Pop( EN_BUFFER_ID, void* pTargetPtr, u32 nLength );

//----------------------------------------------------------------------------------//
// Peek data from circular buffer 
// if return value less then 0 error value is EN_SHARED_BUFFER_ERROR or EN_BASE_ERROR
//----------------------------------------------------------------------------------//
HEVC_BASE_API n32	Peek( EN_BUFFER_ID, void* pTargetPtr, u32 nLength );

//**********************************************************************************//

} /* namespace buffer */ } /* namespace hevc */

#endif // BLOCK_BUFFER_H_DEFINES
