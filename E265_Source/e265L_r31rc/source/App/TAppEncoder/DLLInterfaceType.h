/*
*********************************************************************************************

   Copyright (c) 2006 Electronics and Telecommunications Research Institute (ETRI) All Rights Reserved.

   Following acts are STRICTLY PROHIBITED except when a specific prior written permission is obtained from 
   ETRI or a separate written agreement with ETRI stipulates such permission specifically:

      a) Selling, distributing, sublicensing, renting, leasing, transmitting, redistributing or otherwise transferring 
          this software to a third party;
      b) Copying, transforming, modifying, creating any derivatives of, reverse engineering, decompiling, 
          disassembling, translating, making any attempt to discover the source code of, the whole or part of 
          this software in source or binary form; 
      c) Making any copy of the whole or part of this software other than one copy for backup purposes only; and 
      d) Using the name, trademark or logo of ETRI or the names of contributors in order to endorse or promote 
          products derived from this software.

   This software is provided "AS IS," without a warranty of any kind. ALL EXPRESS OR IMPLIED CONDITIONS, 
   REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED WARRANTY OF MERCHANTABILITY, FITNESS 
   FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT, ARE HEREBY EXCLUDED. IN NO EVENT WILL ETRI 
   (OR ITS LICENSORS, IF ANY) BE LIABLE FOR ANY LOST REVENUE, PROFIT OR DATA, OR FOR DIRECT, 
   INDIRECT, SPECIAL, CONSEQUENTIAL, INCIDENTAL OR PUNITIVE DAMAGES, HOWEVER CAUSED AND 
   REGARDLESS OF THE THEORY OF LIABILITY, ARISING FROM, OUT OF OR IN CONNECTION WITH THE USE 
   OF OR INABILITY TO USE THIS SOFTWARE, EVEN IF ETRI HAS BEEN ADVISED OF THE POSSIBILITY OF 
   SUCH DAMAGES.

   Any permitted redistribution of this software must retain the copyright notice, conditions, and disclaimer 
   as specified above.

*********************************************************************************************
*/

/**    
    @file    DLLInterfaceType.h
    @date    2014/03/21
    @author  석진욱 (jnwseok@etri.re.kr) ETRI
    @brief   The Highest level of Defition to DLL Files
*/


#ifndef _DLLINTERFACEYPE_H_
#define  _DLLINTERFACEYPE_H_

/// Single Encoder/SmartGuru 에서는 1로 세팅, Multiple Encoder Test 에서는 0으로 세팅 
#define	ETRI_SINGLE_ENCODER_ENABLE	1 	/// 2013 12 21 by Seok For TDllEncoder/TAppDLLEncoder

#if (defined(TDLLMULTIENCODER_EXPORTS)||defined(_MULTIPLE_ENCODER))
#define	ETRI_DLL_UNIFICATION		0
#undef	ETRI_SINGLE_ENCODER_ENABLE
#else
#define	ETRI_DLL_UNIFICATION		ETRI_SINGLE_ENCODER_ENABLE
#endif

/// 2014 4 21 by Seok : Service Predefinition
//#if	ETRI_SINGLE_ENCODER_ENABLE
//#define	ENCODER_TYPE	"[SINGLE_ENCODER]"
//#else
//#define	ENCODER_TYPE	"[MULTIPLE_ENCODER]"
//#endif

// ========================  Buffer Size  ===========================
#define	MAX_INPUT_BUFFER_SIZE    	33177600   	//< 3840x2160x2 (Y,U,V 4:2:2) x 2 (16bit)
#define	MAX_ANNEXB_BUFFER_SIZE   	8388608   	//< 1024(1K) x 1024(1K) x 64 / 8 = 8Mbyte : 80Mbps
#define	MAX_TEMP_BUFFER_SIZE		(MAX_ANNEXB_BUFFER_SIZE >> 3)	

/**
----------------------------------------------------------------------------------
@def     MAX_GOP_NUM
@brief    GOP 단위 처리를 위한 GOP 최대 처리 개수를 정의합니다.
@endcode
----------------------------------------------------------------------------------
*/
#define MAX_FRAME_NUM_IN_GOP                (32)

#include "TLibCommon/ETRI_HEVC_define.h"
#include "TLibVideoIO/TVideoIOYuv.h"
 
// ====================================================================================================================
// ETRI Main DLL Function & Interface
// ====================================================================================================================
//#define	MAX_ANNEXB_BUFFER_SIZE	8388608	//1024(1K) x 1024(1K) x 64 / 8 = 8Mbyte : 80Mbps

#if (ETRI_DLL_INTERFACE)
#define	writeAnnexB				ETRI_writeAnnexB
#define	ETRI_StreamInterface	ETRI_fstream
#else
#define	writeAnnexB				HM_writeAnnexB
#define	ETRI_StreamInterface	std::ostream
#endif

typedef struct InterfaceInfo
{
	int		argc;								///< to DLL
	char**	argv; 								///< to DLL
	
	int    	iNumEncoded;						///< Encoded Frame 갯수  ((From DLL))
	bool   	bEos;								///< End of Sequence : 입력 파일의 EoF, Encoding Frame 수 등등을 사용하여 얼마든지 제어 할 수 있다. (to DLL/From DLL : Default : From DLL)

	///............. Member Parameters in TEncCfg, TEncTop 
	int   	m_aiPad[2];							///< Frame의 Padding 크기  (From DLL)
	int*   	m_piFrameRcvd;						///< 인코더가 받은 프레임의 수  (From DLL) 

	///............. Input (YUV) and Output (HEVC) Data
	char* 	m_pchInputFile;						///< Name of Input File		(From DLL)
	char* 	m_pchBitstreamFile;					///< Output이 File일 경우 Bitstream File 이름 	(From DLL)

#if ETRI_DLL_INTERFACE
	ETRI_fstream*	m_pcHandle;						///< Pointer of Input Data Structure
	ETRI_fstream*	m_pcRecStr;						///< Pointer of Reconstructure Data Structure
	ETRI_StreamInterface*	FStream;				///< Output Stream : C++ STL ,혹은 ETRI_fstream 구조체를 사용하여 버퍼제어 : 마치 파일처럼 버퍼를 제어할 수 있다.  
#endif

	unsigned char *	ptrData;   					///< YUV Data for Interface			(to DLL)	
	unsigned char *	AnnexBData;   				///< HEVC Bitstream Data for Interface	(From DLL)
	unsigned char *	ptrRec;						///< HEVC Bitstream Data for Interface (From DLL)

	bool 	is16bit;
	int    	m_iSourceWidth;						///< source width in pixel	(From DLL)
	int    	m_iSourceHeight;					///< source height in pixel (From DLL)
	
	int    	FrameSize;							///< Size of YUV Data per 1 Frame : Y + U + V Total (From DLL)
	int    	AnnexBFrameSize;					///< AnnexB Size perFrame (Output Frame Size) (From DLL)

	void* 	pcPicYuvOrg;						///< 입력 Frame to Encoder DLL (From DLL)
	void* 	pcPicYuvRec;						///< Reconstruction Frame        (From DLL) 

	///............. Other Control Parameters
	bool 	ServiceFunctionIndex[16];			///< 0 : Print Summary (to stdout) (to DLL)
												///< 1 : HEVC header Information 

	//............. Control (HEVC Bitstream) Data
	struct _CTRParam
	{
		bool			bInBufferOff;
		bool			bOutBufferOff;
		bool			bRecBufferOff;
		short			iDllIdx;

		int				*piPOCLastIdx;
		int				*piNumPicRcvdIdx;
		unsigned int	*puiNumAllPicCodedIdx;
		bool			bAnalyzeClear;
		bool			bStartEncoding;

		bool			bIOThreadGoOn;
		bool			bIOWorkGoOn;
		bool			bENThreadGoOn;
		bool			bENWorkGoOn;

		int				iFramestobeEncoded;
		unsigned int	uiFrameCount;
		int				iNumEncoders;
		int				iNumofGOPforME;
		int				iFullIORDProcess;
		unsigned int	uiEncodedFrame;
		int				iInfiniteProcessing;
		unsigned int	uiTotalBytes;
		unsigned int	uiFrameRates;
		
		unsigned int	uiMultipleEncoder;
		unsigned int	uiMultipleEncoderGoON;
		void 			*hDllEvent;
		void			*hTimer;
		double			dbTotalEncodingTime;

		short			sSliceIndex;

		unsigned short		usUseProcessGroup;
		unsigned short		usPorcGroup;
		unsigned long long	ullAffinityMask;

		bool			bChangeFrameTobeEncoded;

#if ETRI_BUGFIX_DLL_INTERFACE
		unsigned int	uiNumofEncodedGOPforME;
		int				*piIntraPeriod;
#endif
	} CTRParam; ///< Control Parameter 		(to/From DLL : Not Implemented)

	int*   	DbgInfo; ///< Internal Debug Information    (From DLL)

	///............. System Integration  ..... : 2013 10 24 by Seok
	unsigned int			nFrameCount;		///< Frame Count in GOP
	unsigned int			nGOPsize;									///< GOP Size from Config
	unsigned int			nFrameStartOffset[MAX_FRAME_NUM_IN_GOP];	///< Frame Start offset in bitstream (per GOP)
	unsigned int			nFrameTypeInGop[MAX_FRAME_NUM_IN_GOP];		
	unsigned int			nPicPresentationOrder[MAX_FRAME_NUM_IN_GOP];///< Pic POC
	unsigned int			nPicDecodingOrder[MAX_FRAME_NUM_IN_GOP];	///< Pic POC
	UInt64					nTimestamp[MAX_FRAME_NUM_IN_GOP];			///< Time Stamp Information	From Outside : 2014 06 20 Modified.
	short                   nSliceIndex[MAX_FRAME_NUM_IN_GOP];			///< Slice Index 0 - 7, //SliceEncoder by yhee

	///............. User Defined Data
	char*	UserDefinedParameter; 				///< MAX_HEADER_BUFFER_SIZE Byte 영역의 사용자 데이터를 사용하여 추가 User Data를 사용한다. (From DLL)
	//............. HeaderCount	
	unsigned int*	em_HeaderCount; 			///< totalCount	(From DLL)
	//unsigned int	em_GOPStartInBuffer;  		///totalCount, two buffer struc.

	/* . . . . . Example  
	short Param0 = *(short *)&UserDefinedParameter[0];
	int	 Param1 = *(int *)&UserDefinedParameter[2];		//Short 형은 2 Byte 이므로 
	int	 Param2 = *(int *)&UserDefinedParameter[6];		//int 형은 4 Byte 이므로 
	char* Param3 = *(char *)&UserDefinedParameter[10];	//int 형은 4 Byte 이므로 .... 64 Byte 길이의 Char String을 사용한다고 가정하면...
	void* Param3 = *(char *)&UserDefinedParameter[74];	//Void* 형은 4 Byte 이던가 8 byte 던가... 헷갈리네...

	즉, UserDefinedParameter 의 Index만 확실히 알면, Interface는 이것으로 규정되어 진다.
	*/

} ETRI_Interface;

#endif
