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

#ifndef	ETRI_HEVC_DEFINE_H
#define ETRI_HEVC_DEFINE_H

#if 0
#ifdef __cplusplus
extern "C" {
#endif
#endif

#include <emmintrin.h> 	// SSE 2.0
#include <tmmintrin.h> 	// SSE 3.0
#include <smmintrin.h> 	// SSE 4.1
#include <nmmintrin.h> 	// SSE 4.2	

#if defined(_WIN32) || defined(_WIN64)
#define	_ETRI_WINDOWS_APPLICATION	1
#else
#define	_ETRI_WINDOWS_APPLICATION	0
#endif

#if (_ETRI_WINDOWS_APPLICATION)
#include <tchar.h> 		//For Debug
#include <direct.h>		//For Debug Path when you use VC IDE
#include <conio.h>
#else
#include <errno.h>      //For Debug
#endif

#define	E265_VERSION				"e265L_r31 (Based on HM15.0)"	
#define	E265_DEV_VERSION			"2017.01.20"
//==========================================================================
//	ETRI Debug Tools 
//==========================================================================
#define	ERROR_DLL_BUF_SZ			1024
#define	DEBUGFILEPRPX 				"HEVC_debugTH.txt"
#define	_dbgMsgSIZE   				ERROR_DLL_BUF_SZ	
//#define ETRI_PRINT_MSG				1

#if defined(_DEBUG) 
#define	_FILE_DEBUG					1
#define	_GLOBAL_DEBUG  				1
#define	_INVERSEDEBUG  				0	///< It is an inverse predefinition of _DEBUG @ 2015 8 17 by Seok				
#define	_YHEEDEBUG  				0	///< For yhee, pls keep this option.	
#else 
#define	_FILE_DEBUG					0
#define	_GLOBAL_DEBUG  				0	///< 2013 9 15 by Seok : to stop the Threa Processing
#define	_REALTIME_DEBUG				1
#define	_INVERSEDEBUG  				1	///< It is an inverse predefinition of _DEBUG @ 2015 8 17 by Seok				
#define	_YHEEDEBUG  				0	///< For yhee, pls keep this option.	
#endif

#define WHERESTR  "[LINE:%d %s] "
#define WHEREARG  __LINE__, __FUNCTION__ 

#if (_ETRI_WINDOWS_APPLICATION)
#define EDPRINTF(Type,_fmt,...) 	fprintf(Type, WHERESTR _fmt, WHEREARG, __VA_ARGS__);
#define ESPRINTF(Key,Type,_fmt,...) if (Key) {fprintf(Type, WHERESTR _fmt, WHEREARG, __VA_ARGS__);}
#define ETPRINTF(Key,_fmt,...) 		if (Key) {_tprintf(_T(WHERESTR _fmt), WHEREARG, __VA_ARGS__);}
#define	E_PAUSE	{EDPRINTF(stdout, "Push Any Key \n"); _gettch();}
#else
#define EDPRINTF(Type,_fmt,...) 	fprintf(Type, WHERESTR _fmt, WHEREARG, ##__VA_ARGS__);
#define ESPRINTF(Key,Type,_fmt,...)    
#define ETPRINTF(Key,_fmt,...)
#define	E_PAUSE                         

#define vsprintf_s(buf, size, fmt, ...) vsprintf(buf, fmt, ##__VA_ARGS__)
#define sprintf_s(buf, size, fmt, ...)	sprintf(buf, fmt, ##__VA_ARGS__)
#define fprintf_s(stream, fmt, ...)		fprintf(stream, fmt, ##__VA_ARGS__)
#define _vscprintf(fmt, ...)			vsnprintf(NULL, 0, fmt, ##__VA_ARGS__)
#define printf_s						printf
#define strtok_s	strtok_r
#define	_chdir		chdir
#endif

#define	noop		/// No Operation @ 2016 2 1 by Seok
//==========================================================================
//	Linux Porting 
//==========================================================================
#if (_ETRI_WINDOWS_APPLICATION)
#define ALIGNED(x)					__declspec(align(x))
#else
#define ALIGNED(x)					__attribute__((aligned(x)))
#define _aligned_malloc(x,y)		_mm_malloc(x,y)
#define	_aligned_free(x)			_mm_free(x)
//#define __cdecl						__attribute__((__cdecl__))
#define __cdecl
#define __stdcall
#endif

#if (_ETRI_WINDOWS_APPLICATION)
typedef       __int32             Int32;
typedef       __int16             Int16;
typedef       __int8              Int8;
typedef		  unsigned int		  UINT;

#else
typedef       int				  Int32;
typedef       short               Int16;
typedef       char                Int8;

typedef		  void *			  HANDLE;
typedef		  unsigned long		  DWORD;
typedef		  unsigned char		  BYTE;
typedef		  unsigned int		  UINT;
#endif

//==========================================================================
//	ETRI Service Tools 
//==========================================================================
#define	CounterPrint(idx) 			if((idx & 0x07) == 0) {fprintf(stderr, ".");}
#define	PrintOnOFF(x)   			((x)? "Active" : "OFF")
#define	ETRI_sABS(x)    			(((x) < 0)? -(x) : (x))
#define	ETRI_sMIN(x, y) 			(((x) < (y))? (x) : (y))
#define	ETRI_get3State(x) 			((x>0)? (1):((x<0)? (-1):(0)))	///< ((x>0)? (1):((x<0)? (-1):(0)))2015 10 6 by Seok
#define	GetBoolVal(x)				((x)? "TRUE":"FALSE")	

#define	MIN_INT32					-2147483647 	///< max. value of signed 32-bit integer
#define	ETRI_NULLPOINTER			0xcccccccc
#define	DeleteParam(x)				if(x) {delete x; x = nullptr;}
#define	DeleteDimParam(x)			if(x) {delete[] x; x = nullptr;}
#define	ETRI_MallocFree(x) 			if(x) {free(x); x = nullptr;}	
#define	ETRI_AlignedMallocFree(x) 	if(x) {_aligned_free(x); x = nullptr;}	

#define	FINDDBGLOCATION(Type,pcCU,uhDepth)		EDPRINTF(Type, "POC: %d Addr: %d Depth: %d ID: %d (%d, %d) \n", pcCU->getSlice()->getPOC(), pcCU->getAddr(), uhDepth, pcCU->getZorderIdxInCU(), pcCU->getCUPelX(), pcCU->getCUPelY())

// ================================================================================================================
// ETRI_Configuration for Development
// ================================================================================================================
#define	ETRI_TRACE_ENABLE  	  		0				//If 1, Trace is enable: 0, Trace is disable 
#define	ETRI_VERBOSE_RATE  	  		0				//Verbose Rate
// ================================================================================================================

//Insert Headers to every IDR period
#define	ETRI_WriteIDRHeader			0
#define	ETRI_WriteVUIHeader			1
#if ETRI_WriteVUIHeader
#define	SG_EnableFractionalFrameRate 	0					/// For Fractional Frame Rate such as 29.97, 59.94 fps : wsseo@2015-08-24. fix fps
#endif

#define CABAC_INIT_PRESENT_FLAG				  1      ///<Default = 1 for HM, 0 for ETRI_MULTITHREAD_2,  move from TypeDef.h by yhee

// ================================================================================================================
// ETRI Encoder e265 Version
// ================================================================================================================
#define ETRI_MODIFICATION_V00 					1							///< Inclu. DLL, Tile Parallel
#define ETRI_MODIFICATION_V01 					1							///< Inclu. SIMD ver. 1 
#define ETRI_MODIFICATION_V02 					1							///< Inclu. CU Structure ver. 2 
#define ETRI_MODIFICATION_V03 					(1 && ETRI_MODIFICATION_V02)	///< 

#define ETRI_E265_VERSIONNUMBER				(ETRI_MODIFICATION_V00 + ETRI_MODIFICATION_V01 + ETRI_MODIFICATION_V02 + ETRI_MODIFICATION_V03)	
	
// ========================================================================
// ETRI MVClip to avoid the slice boundary reference
// ========================================================================
#define ETRI_SliceEncoder_MVClip				1
#define ETRI_SliceFrameEncoding				    0 //slice-frame encoding enable

// ================================================================================================================
// 
//	e265 Version V0: Tile Parallel, DLL
//
// ================================================================================================================
#if  ETRI_MODIFICATION_V00			
#define ETRI_RETURN_ORIGINAL					0						///< Same to the HM. @ 2015 5 25 by Seok
#define ETRI_MULTITHREAD						!ETRI_RETURN_ORIGINAL	///< Parallel Operation. @ 2015 5 25 by Seok
#define ETRI_MULTITHREAD_2						1						///< GOP/Frame Parallel Porting. @ 2015.10.26 by yhee
#define ETRI_THREADPOOL_OPT						1						///< Use quram threadpool
#define ETRI_DLL_INTERFACE 						1						///< Compile option for ETRI_DLL. @ 2015.06.15 e265 porting by yhee
#define ETRI_E265_PH01							1						///< For Version Management of Developing E265 @ 2015 5 26 by Seok
#define ETRI_MAX_TILES							64 					   ///< 65 is the max num for windows 
#define ETRI_SIMD_REMAIN_16bit					ETRI_DLL_INTERFACE


// ========================================================================
// Release/Debug mode setting
// ========================================================================
/// Full Release Mode :  1  : Prohibit Any printing of Information 2014 4 11 by Seok 
/// If you want to erase the information of each frame, it shoud be set to 1
#define ETRI_FULL_RELEASEMODE		(1 & ETRI_DLL_INTERFACE)	
#if ETRI_FULL_RELEASEMODE
#define	RLS_TYPE					"[FULL_RELEASE]"
#else
#define	RLS_TYPE					"[INFORMATION]"
#endif

// ========================================================================
// Rate Control
// ========================================================================
#define KAIST_RC   1
#if KAIST_RC
#define ETRI_DQP_FIX                1
#define KAIST_HRD			1  // Hypothetical reference decoder with a true decoding order (online); KAIST_HRD_print IS USED WITH KAIST_HRD FOR CONFIRMING RESULTS
#define KAIST_HRD_print		0  // Hypothetical reference decoder with a true decoding order (offline)
#define KAIST_HARDCODING_QP 0  //for testing quality balance
#define KAIST_HARDCODING_TARGETBIT 0  //for testing quality balance
#define KAIST_USEPREPS			0  // use preprocessing for RC
#define ETRI_RC_FIX         1
#if KAIST_USEPREPS
#define KAIST_SCENECHANGE	1
#endif
#endif

// ========================================================================
// ETRI_MULTITHREAD
// ========================================================================
#if ETRI_MULTITHREAD
#define ETRI_TILE_PARALLEL						(1 && !_GLOBAL_DEBUG)		///< Tile Parallel: Parallel Operation. @ 2015 5 25 by Seok
#define ETRI_TILE_SERIAL_TEST					!ETRI_TILE_PARALLEL		///< Tile Parallel: Serial Test. @ 2015 5 25 by Seok
#endif

#if ETRI_MULTITHREAD_2
// ========================================================================
// ETRI Threadpool Optimization
// ========================================================================
#if ETRI_THREADPOOL_OPT
#if ETRI_SliceFrameEncoding
#define ETRI_THREADPOOL_OPT_MAX_THREAD			17	// max thread of quram threadpool
#else
#define ETRI_THREADPOOL_OPT_MAX_THREAD			64	// max thread of quram threadpool
#endif
#define ETRI_TILE_THEAD_OPT						1
#define ETRI_FRAME_THEAD_OPT					1
#define ETRI_COPYTOPIC_MULTITHREAD				1
#define ETRI_THREAD_LOAD_BALANCING				(0 & ETRI_SliceFrameEncoding) //only working with slice encoding with mutlpiple PPS option
												// When 0, Test for Multiple Slice. When operation of mutiplke Slice is good, it must set 1.
												// LoadBalancing And Mutiple PPS are Combined. by Yhee
// ========================================================================
// ETRI Multiple PPS Support
// ========================================================================
#define ETRI_MultiplePPS						(ETRI_THREAD_LOAD_BALANCING )
#if ETRI_MultiplePPS  
#define ETRI_MAX_AdditionalPPS					2
#endif

#endif


//========================================================================================================
//	ETRI Frame Parallel Definition
//========================================================================================================
#define MAX_THREAD_SIZE			64
#if ETRI_SliceFrameEncoding
#define MAX_THREAD_GOP			3   // Frame-thread num
#if ETRI_MultiplePPS
#define MAX_THREAD_TILE			16	// Tile thread num
#else
#define MAX_THREAD_TILE			8	// Tile thread num
#endif
#else
#define MAX_THREAD_GOP			8   // Frame-thread num
#define MAX_THREAD_TILE			30	// Tile for frame encoding thread num
#endif

#define ETRI_TILE_ONLY			1   //ThreadPool-based tile ony, If ETRI_MULTITHREAD_2=0, then ETRI_V0 tile_only ver.
#define ETRI_FRAME_PARALLEL		2  // Frame-parallelization option
#define ETRI_GOP_PARALLEL		3  // GOP-parallelization option

#define ETRI_SEQUENCE			1  //when ETRI_TILE_ONLY on,  ETRI_THREAD_SEL should be ETRI_SEQUENCE // yhee
#define ETRI_SEQ_ARRAY			2  //Serial Test
#define ETRI_MULTI_ARRAY		3  //Parallel Test
								//for r27 serial option: ETRI_FRAME_PARALLEL && ETRI_SEQ_ARRAY
#define ETRI_PARALLEL_SEL		ETRI_GOP_PARALLEL //ETRI_FRAME_PARALLEL (for r27 serial woring) //ETRI_TILE_ONLY
#define ETRI_THREAD_SEL			ETRI_MULTI_ARRAY //ETRI_SEQ_ARRAY (for r27 serial working) 

#ifdef CABAC_INIT_PRESENT_FLAG
#undef CABAC_INIT_PRESENT_FLAG
#define CABAC_INIT_PRESENT_FLAG		0
#endif
#define PSNR_DISPLAY				1  // 0: not display, 1: display

#endif

// ========================================================================
// ETRI_DLL_INTERFACE
//
// Notice : ETRI_SliceEncoderHeader
//    0 : DLL 동작 Test를 위한 전체 영상 인코딩 (Header 정상)
//    1 : Slice 분산 처리 지원 DLL (Header 가 분산 처리용)
// ========================================================================
#if ETRI_DLL_INTERFACE
#define ETRI_DLL_DEBUG 							1
#define ETRI_EXIT(x)   							throw(x)
#define APP_TYPE  								"DLL_APPLICATION"
//Header management for Slice ES Merge
#define ETRI_SliceEncoderHeader					ETRI_SliceFrameEncoding	///< Header Modification for SliceDLL by yhee 2015.09.20 : 
#if ETRI_SliceEncoderHeader
#define ETRI_Header_NoTile						0 	//remove tile setting for slice_encoder at PPS

#define ETRI_Header_NumSlices					6
//#define ETRI_Header_NumSlices					7 //Num of Slice to split
#define ETRI_Header_ImgYWidth					3840
#define ETRI_Header_ImgYHeight					2160
#define ETRI_Header_MaxSliceSegAddress			2040 //60(3840/64) x34(2160/64 + 1)
#endif
#else
#define ETRI_EXIT(x)   							exit(x)
#define APP_TYPE  								"EXE_APPLICATION"
//For validate test w/App.exe, pls keep the following, yhee 2015.12.17
//#define ETRI_SliceEncoderHeader					1	///< Header Modification for SliceDLL by yhee 2015.09.20
//#if ETRI_SliceEncoderHeader
//#define ETRI_Header_NoTile						0 //remove tile setting for slice_encoder at PPS
//#endif
//#define ETRI_Header_SliceIdx					0	///< Caution!!!! FirstSliceIdx 0, FollowingSliceIdx value: 1 ~ 7 for 8 slice
//#define ETRI_Header_NumSlices					6
//#define ETRI_Header_ImgYWidth					3840
//#define ETRI_Header_ImgYHeight					2160
//#define ETRI_Header_MaxSliceSegAddress			2040 //60(3840/64) x34(2160/64 + 1)
#endif

#endif //end of #if  ETRI_MODIFICATION_V00																

// ================================================================================================================
// 
//	e265 Version V1: SIMD and SW Optimization
//
// ================================================================================================================

// ========================================================================
// ETRI SIMD 
// ========================================================================
#define ETRI_SIMD 								ETRI_MODIFICATION_V01
#if ETRI_SIMD  										///< Integrated by KMS 2015.07.01
#include <nmmintrin.h>  							///< SSE 4.2

#define ETRI_SIMD_TR  							1 	///< Forward/Inverse DST and DCT of All TB Sizes (by JDS)
#define ETRI_SIMD_MD  							1 	///< Mode Decision with SAD, HAD, and SSE (by JSH & JDS)
#define ETRI_SIMD_INTRA  						1 	///< Intra Prediction: Angular, DC, Planar (by KJH)
#if ETRI_SIMD_INTRA
#define ETRI_SIMD_INTRA_FIX_COMPATIBILITY       1   ///< FIX FOR COMPILIER COMPATIBILITY
#if ETRI_SIMD_INTRA_FIX_COMPATIBILITY
#define ETRI_SIMD_FIX_INTRA_DC                  1
#define ETRI_SIMD_FIX_INTRA_ANGULAR_PREDICTION  1
#define ETRI_SIMD_FIX_INTRA_PLANAR_PREDICTION   1
#endif 
#endif 
#define ETRI_SIMD_INTERPOLATION  				1 	///< Interpolation (by SJW & KJH)
#define ETRI_SIMD_INTRA_RECONSTRUCTION   	   	1 	///< Integrated from ETRI RExt (by KJH, integrated by LSC)
#define ETRI_SIMD_YUV  							1 	///< Integrated from ETRI V12 (by ?, integrated by LSC)
#define ETRI_SIMD_DEQUANTIZATION  				1
#define ETRI_SIMD_INTRA_RESIDUAL  				1
#define ETRI_SIMD_REMOVE_HIGH_FREQ  			1
#define ETRI_SIMD_VideoIOYUV					ETRI_DLL_INTERFACE 	///< This SIMD does not work for now. ETRI_readPlane when ETRI_DLL_INTERFACE = 1

#define ETRI_SIMD_MATRIX_TRANSFORM              1   ///< MATRIX TRANSFORM FOR 32x32, 16x16, 8X8 AND 4x4 
#define ETRI_SIMD_COPY_DATA                     1
#if ETRI_SIMD_COPY_DATA
#define ETRI_SIMD_COPY_TO_PIC_YUV			    1
#define ETRI_SIMD_COPY_FROM_PIC_YUV				1
#define ETRI_SIMD_COPY_PART_YUV					1
#define ETRI_SIMD_COPY_TO_PIC                   1
#endif 
#define ETRI_SIMD_EXTENEDED_PIC_BORDER          1
#endif

// ========================================================================
// Fast Quantization Methods
// ========================================================================
#define ETRI_RDOQ_OPTIMIZATION					ETRI_MODIFICATION_V01

#if ETRI_RDOQ_OPTIMIZATION
#define ETRI_RDOQ_CODE_OPTIMIZATION 			1
#define ETRI_RDOQ_SIMD_OPTIMIZATION 			1
#define ETRI_Remove_Redundant_EntoropyLoader 	0	///< Now it is inactive. Hoever, for making E265V01:02 it should be active as Lossless Code Optimization @ 2015 5 12 by Seok
//#define ETRI_RDOQ_ZONAL_CODING   				0
//#define ETRI_RDOQ_ROUGH_ESTIMATION 			0
#endif

// ========================================================================
// Source Code Optimization 
// ========================================================================
#define ETRI_CODE_OPTIMIZATION					ETRI_MODIFICATION_V01

#if ETRI_CODE_OPTIMIZATION
#define ETRI_MEMSET_OPTIMIZATION				1
#define ETRI_MEMSET_OPTIMIZATION_Debug  		_INVERSEDEBUG	///< if on, debug/release mode mismatch occurs at the "initEstData()" optimization 
//#define ETRI_REDUNDANCY_OPTIMIZATION 			0
//#define ETRI_CU_INIT_FUNCTION_OPTIMIZATION	0
#define ETRI_SET_FUNCTION_OPTIMIZATION  		1
#define ETRI_COMPONENT_BIT_OPTIMIZATION 		1 
#define ETRI_CLIP_OPTIMIZATION					1
#define ETRI_GETSIGCTXINC_OPTIMIZATION  		1
#if ETRI_GETSIGCTXINC_OPTIMIZATION
#define ETRI_GETSIGCTXINC_OPTIMIZATION_DEC   	0
#endif
#define ETRI_CODECOEFFNXN_OPTIMIZATION  		1
#define ETRI_CODELASTSIGNIFICANTXY_OPT  		1
#define ETRI_SCALING_LIST_OPTIMIZATION  		1

#define ETRI_CU_CREATE_MODIFICATION 			1
#if ETRI_CU_CREATE_MODIFICATION
#define ETRI_TRANSFORM_SKIP_OPTIMIZATION    	1
#if ETRI_TRANSFORM_SKIP_OPTIMIZATION
#define ETRI_MEMCPY_OPTIMIZATION				1
#endif
#define ETRI_IPCM_OPTIMIZATION					1
#define ETRI_LOSSLESS_OPTIMIZATION  			1
#define ETRI_COEFF_MEMSET_REMOVAL   			1
#define ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION  1
#if ETRI_DQP_FIX
#define ETRI_SLICE_SEGMENT_OPTIMIZATION   		0
#else 
#define ETRI_SLICE_SEGMENT_OPTIMIZATION   		1
#endif 
#endif

#define ETRI_CODE_CLEANUP     					1
#if ETRI_CODE_CLEANUP
#define ETRI_UNNECESSARY_CODE_REMOVAL   		0   
#define ETRI_REMOVE_SAO_RELATED_CODE    		1
#define ETRI_REMOVE_AMP_RELATED_CODE    		1
//#define ETRI_REFSAMPLE_REDUNDANCY_REMOVE    	1
//#define ETRI_EM_BASED_MOTION_ESTIMATION  		0
#define ETRI_EM_OPERATION_OPTIMIZATION  		1
//#define ETRI_EM_UPDATE_ENCODE_OPTIMIZATION  	0
#endif

#define ETRI_INITESTDATA_OPTIMIZATION 			1
#define ETRI_AVAILABILITY_CHECK_OPTIMIZATION 	1
//#define ETRI_COMPRESSMOTION_OPTIMIZATION      0
//#define ETRI_Remove_Multi_Slice_Coding_Loop		0 	///< Now it is 0 and No Implementation. However, for making E265V01:02 it should be active.

#define ETRI_FURTHER_CODE_OPTIMIZATION          1
#if ETRI_FURTHER_CODE_OPTIMIZATION

#if KAIST_RC
#define ETRI_CONTEXT_MODEL_CODE_CLEANUP         0
#else
#define ETRI_CONTEXT_MODEL_CODE_CLEANUP         1
#endif
#define ETRI_INTRA_PREDICTION_CLEANUP           1        
#define ETRI_XCOPYYUV2PIC_CLEANUP				1
#define ETRI_REDUNDANCY_OPTIMIZATION			1
#if KAIST_RC
#define ETRI_REMOVE_XCHECKDQP                   0
#else
#define ETRI_REMOVE_XCHECKDQP                   1
#endif
#if ETRI_DQP_FIX
#define ETRI_CU_INIT_FUNCTION_OPTIMIZATION      0
#else 
#define ETRI_CU_INIT_FUNCTION_OPTIMIZATION      1
#endif 
#define ETRI_MEMSET_FURTHER_OPTIMIZATION        1
#if ETRI_MEMCPY_OPTIMIZATION
#define ETRI_MEMCPY_TCOMDATACU_OPTIMZIZATION    1
#endif
#if ETRI_INITESTDATA_OPTIMIZATION
#undef ETRI_MEMSET_OPTIMIZATION_Debug
#define ETRI_MEMSET_OPTIMIZATION_Debug          0 
#endif 

#define ETRI_NO_STRONGINTRASMOOTHING			1
#endif 
//-----------------------------------------------------
// Sub Definition
//-----------------------------------------------------

#if ETRI_REMOVE_AMP_RELATED_CODE
#ifdef AMP_SAD
#undef AMP_SAD 
#endif
#define AMP_SAD   								0           ///< dedicated SAD functions for AMP
#else
#define AMP_SAD   								1           ///< dedicated SAD functions for AMP
#endif

/**
 ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION  is strongly combined with ADAPTIVE_QP_SELECTION.
 However, when you set  ADAPTIVE_QP_SELECTION to be zero as follows, it occures a compile error on the TComTrQuant:Line 4409
 Consequently, we should decide whether to use ADAPTIVE_QP_SELECTION or not, insterad of following codes.
 @ 2015 0731 by seok

#if ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION  
#ifdef ADAPTIVE_QP_SELECTION
#undef  ADAPTIVE_QP_SELECTION
#endif
#define  ADAPTIVE_QP_SELECTION    0
#endif
*/

#endif	///< Coincides to #if ETRI_CODE_OPTIMIZATION @ 2015 7 31 by Seok

// ================================================================================================================
// 
//	e265 Version V2: Structure of Fast Algorithm
//
// ================================================================================================================
/**
*	Pre Definitions : 
*	ETRI_SET_FUNCTION_OPTIMIZATION = 1 ; 
*	ETRI_FIXED_ESDOFF = 1
*	ETRI_LOSSLESS_OPTIMIZATION = 1;
*	rpcTempCU->getCUTransquantBypass(0) = false;
*	m_pcEncCfg->getUseFastDecisionForMerge() : true
*	bTransquantBypassFlag = false by ETRI_LOSSLESS_OPTIMIZATION;
* 
*	Intra
*	ETRI_REDUNDANCY_OPTIMIZATION : Non defined 
*	HHI_RQT_INTRA_SPEEDUP_MOD = 0
* 	HHI_RQT_INTRA_SPEEDUP = 1
*   	checkTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip() : False
*	m_pcEncCfg->getUseTransformSkipFast() : False;
*	ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION = 1
*	ETRI_TRANSFORM_SKIP_OPTIMIZATION = 1
*	ADAPTIVE_QP_SELECTION = 1
*	
*/

#define	ETRI_IdAxTempCU_Skip	   	0
#define	ETRI_IdAxTempCU_Merge   	1
#define	ETRI_IdAxTempCU_Inter   	2
#define	ETRI_IdAxTempCU_Intra   	3
#define	ETRI_IdAxTempCU_Inter2NxN   4
#define	ETRI_IdAxTempCU_InterNx2N   5
//#define	ETRI_nAXTempCU   	   	  	(ETRI_IdAxTempCU_Intra + 4) 	///< I don't know the reason why only when ETRI_nAXTempCU=6, the result of encoding is different. @ 2015 8 26 by Seok
#define	ETRI_nAXTempCU   	   	  	(ETRI_IdAxTempCU_Intra + 3) 	///< I don't know the reason why only when ETRI_nAXTempCU=6, the result of encoding is different. @ 2015 8 26 by Seok
#define	ETRI_IdAxTempCU_BESTRD 	(ETRI_nAXTempCU + 1)		///< 2015 4 26 by Seok

#define	ETRI_PRED					0
#define	ETRI_RDOQ					1
#define	ETRI_AdditionalInter		2

#if  ETRI_MODIFICATION_V02
// ====================================================================================================================
// Predefinitions for Algorithm 
// ====================================================================================================================
#define	ETRI_SKIP_64x64LCU			0								///< Default=1 for V02, FIXME to on/off, SKIP 64x64 LCU @ 2015 9 1 by Seok
#define	ETRI_ENABLE_2NxNNx2NProc 	0	///< Default=0 for V02, FIXME to on/off, Off Enable 2NxN & Nx2N Processing @ 2015 9 1 by Seok
#define	ETRI_FIXED_ESDOFF			1	///< Default=1 for V02, Check V1 Option : 2015 0801 by seok

#define	ETRI_RDOOffBestMergeCand	1	///< Default=1, on/off, Choose Best MergeCand based on HAD in SKIP/Merge@ 2015 8 26 by Seok : 
#define	ETRI_FRDOOffBestMergeCand	(1 && ETRI_RDOOffBestMergeCand)	///< ETRI_SKIP_MergeSkip. Depends on  ETRI_RDOOffBestMergeCand @ 2015 8 26 by Seok : 
#define	ETRI_POSTESD				(1 && ETRI_RDOOffBestMergeCand)	///< General ESD. Depends on  ETRI_RDOOffBestMergeCand @ 2015 9 3 by Seok
#define	ETRI_FPOSTESD				(1 && ETRI_POSTESD)				///< Fast ESD Depends on ETRI_POSTESD @ 2015 9 3 by Seok

#define ETRI_GFParallelCompliance 	 1	///1	(Default)	///< Set QpParam Init and Use for ETRI_FRDOOffBestMergeCand for GFParallel, 2015.10.30
#if (ETRI_GFParallelCompliance && !ETRI_MULTITHREAD_2)
#ifdef	CABAC_INIT_PRESENT_FLAG
#undef	CABAC_INIT_PRESENT_FLAG
#define CABAC_INIT_PRESENT_FLAG		 !ETRI_GFParallelCompliance ///<Default=0 for GFParallel
#endif
#endif

#if ETRI_DQP_FIX
#define	ETRI_DEV_0731				0	///< Rec. to delete by yhee, TEncCU  First Revision for 2015 7 31 by Seok
#else 
#define	ETRI_DEV_0731				1	///< Rec. to delete by yhee, TEncCU  First Revision for 2015 7 31 by Seok
#endif 
// ========================================================================
// ETRI_MVClip For Slice Encoder
// ========================================================================
#ifdef	ETRI_SliceEncoder_MVClip
#undef	ETRI_SliceEncoder_MVClip

#define	ETRI_SliceEncoder_MVClip  	(1 && !ETRI_ENABLE_2NxNNx2NProc)	///< Default=1 for V02, Clip the mv for ETRI slice encoder 2015.09.25 by yhee
#define	ETRI_SliceEncoder_MVClipCheck (0 && ETRI_SliceEncoder_MVClip)  	///< Default=0 for V02, Print Error message w/ETRI_SliceEncoder_MVClip option
#endif

// ====================================================================================================================
// ETRI FAST MOTION ESTIMATION  
// ====================================================================================================================
#define ETRI_INC_RASTER				1 /// 1 for iRaster=50, 0 for iRaster=5 ///< Default=1 for V03

#define ETRI_FAST_MOTION_ESTIMATION 1            ///< Default=1 for V03
#if ETRI_FAST_MOTION_ESTIMATION
#define ETRI_NOT_QUARTERPEL_ME		1           ///< Default=1 for V03
#define ETRI_FME_HALFPEL_DIAMOND_ON	1           ///< Default=1 for V03
#define ETRI_STATUS_FAST_ME_INFORM	1			
#endif 

// ====================================================================================================================
// ETRI FAST CU METHODS
// ====================================================================================================================
#define ETRI_FAST_CU_METHODS		1

#if ETRI_FAST_CU_METHODS
#define ETRI_ADAPTIVE_CTU_SIZE      1
#if ETRI_ADAPTIVE_CTU_SIZE			
#define ETRI_ADAPTIVE_MAXCTU_SIZE   1
#if ETRI_ADAPTIVE_MAXCTU_SIZE
#define	ETRI_F64SLiceLevel 			1			///< Indicate the Slice Level accommodates to 32x32 [0:3] 0 : GPB, @ 2015 11 16 by Seok	
#ifdef ETRI_SKIP_64x64LCU
#define ETRI_SKIP_64x64LCU			0
#define ETRI_SKIP_64x64LCU_BUGFIX   1
#endif 
#endif 
#define ETRI_ADAPTIVE_MINCTU_SIZE   1
#define ETRI_ADAPTIVE_CTU_SIZE_BUGFIX 1 
#endif 
#define ETRI_CU_MODE_INHERITANCE	1
#if ETRI_CU_MODE_INHERITANCE		
#define ETRI_CU_INTRA_MODE_INHERITANCE	1
#endif 
#define ETRI_REVISE_ECU				1
#endif

// ====================================================================================================================
// ETRI FAST PU METHODS
// ====================================================================================================================
#define ETRI_FAST_PU_METHODS        1
#if ETRI_FAST_PU_METHODS
#define ETRI_DISABLE_INTRA4x4_GPB   1
#define ETRI_INTRA_LUMA_MODE_DECS	1	
#define ETRI_COARSE_SAD_SUBSAMPLING 1
#endif 

// ====================================================================================================================
// ETRI FAST QUANTIZATION METHODS 
// ====================================================================================================================
#define ETRI_FAST_RDOQ_OPTIMIZATION     1
#if ETRI_FAST_RDOQ_OPTIMIZATION
#define ETRI_RDOQ_ZONAL_CODING          1
#if ETRI_RDOQ_ZONAL_CODING
#define ETRI_INTER_RDOQ_ZONAL_CODING	1
#endif 
#define ETRI_RDOQ_ROUGH_ESTIMATION		1
#endif 

// ====================================================================================================================
// ETRI OPENMP DEBLOCKING FILTER
// ====================================================================================================================
#define ETRI_OMP_DEBLK                             !ETRI_SliceFrameEncoding  // Support multi-threaded deblocking based on OPENMP
#if ETRI_OMP_DEBLK
#ifdef __clang__
#include "/usr/lib/gcc/x86_64-redhat-linux/4.8.5/include/omp.h"
#else
#include <omp.h>
#endif
#define MAX_NUM_THREAD                             6 
#if ETRI_MULTITHREAD_2
#define ETRI_OMP_DEBLK_FOR_MULTITHREAD_2		   1
#else 
#define ETRI_OMP_DEBLK_FOR_MULTITHREAD			   1		
#endif 
#endif 

// ====================================================================================================================
// Index of Parameters 
// ====================================================================================================================
#define	ETRI_IdHighCost   	   	  	0
#define	ETRI_IdForcedMerge			1
#define	ETRI_IdFastAlgorithmON		2	///< 2015 4 21 by Seok : Related to Fast CU Prunning
#define	ETRI_IdCUPrunningON 		3	///< 2015 4 13 by Seok : Fast CU Prunning
#define	ETRI_Id2NxNProcessing		4	///< 2015 8 01 by Seok : [TRUE] Turn On 2NxN and AMP OPeration [FALSE] Turn Off 2NxN and AMP OPeration  
#define	ETRI_IdAdditionalInter		5	///< 2015 8 01 by Seok : [TRUE] Turn On Additional Inter Processing When Previous Inter is SKIP and INTRA Pred Cost is not sufficiently smaller than SKIP/Merge
#define	ETRI_IdFastIntraSKIP  		6	/// 2015 11 28 by Seok : For Fast INTRA SKIP
#define	ETRI_IdEarlyMerge			7	/// 2015 11 18 by Seok : Early Merge Operation	
#define	ETRI_IdxMAXCTUSIZE			8	/// For ETRI_FAST_CU_METHODS @ 2015 11 25 by seok Originally, ETRI_IdxMAXCTUSIZE - 6
#define	ETRI_IdAllModesSKIPPED		9	/// 2015 11 18 by Seok : Early Merge Operation	
#define	ETRI_IdFASTINPRED			10


#define	ETRI_IdNoBestCU   	   	  	11	/// 2015 11 18 by Seok : Early Merge Operation	


#define	ETRI_nControlParam 			16	///< 2015 8 01 by Seok : Total Number of em_bControlParam Index

#define	ETRI_IdLuma   	   	  		0
#define	ETRI_IdChromaU   	   	  	1
#define	ETRI_IdChromaV   	   	  	2
#define	ETRI_IdColorComponent		4

#define	ETRI_IdWidTh   				0	///< For em_uiLevelInfo Indec : 2015 9 19 by Seok
#define	ETRI_IdSliceDepth  			1
#define	ETRI_IdCUDepth   			2
#define	ETRI_IdLevelLuma   			3
#define	ETRI_IdLevelCb				4
#define	ETRI_IdLevelCr				5
#define	ETRI_IdHADLuma   			6
#define	ETRI_IdHADCb   				7
#define	ETRI_IdHADCr   				8
#define	ETRI_IdHADUpdate   			9	/// Inter Prediction Only : Active, Other Mode : inActive
#define	ETRI_nLevelInfo    			10

#if FAST_UDI_USE_MPM
#define	ETRI_MAXINTRAMODE 		10
#else
#define	ETRI_MAXINTRAMODE 		11
#endif

#define	ETRI_nDbgInfo				16	///< 2014 7 17 by Seok : Number of Debug Info Data  32 * 8 = 256 Bytes
#define	ETRI_nDBGInfo_TotalInfo		0
#define	ETRI_nDBGInfo_CASE01		1
#define	ETRI_nDBGInfo_CASE02		2
#define	ETRI_nDBGInfo_CASE03		3
#define	ETRI_nDBGInfo_CASE04		4
#define	ETRI_nDBGInfo_CASE05		5
#define	ETRI_nDBGInfo_CASE06		6
#define	ETRI_nDBGInfo_CASE07		7
#define	ETRI_nDBGInfo_CULevelScope 	8
#define	ETRI_nDBGInfo_CASE09		9
#define	ETRI_nDBGInfo_CASE010		10
#define	ETRI_nDBGInfo_CASE011		11
#define	ETRI_nDBGInfo_CASE012		12
#define	ETRI_nDBGInfo_CASE013		13
#define	ETRI_nDBGInfo_ChkPoint		14	///< Main Check Point Index in Debug Info @ 2015 8 26 by Seok

#define	ETRI_nDBGInfo_Control		(ETRI_nDBGInfo_CULevelScope + 7)

// ====================================================================================================================
// For Conformance to Decoder Solution 
// ====================================================================================================================
#define 	ETRI_ExperimentalBroadcasting	1
#if ETRI_ExperimentalBroadcasting
#define 	ETRI_RC_DEBUG_CONDITION		0
#endif

#endif	///<  ETRI_MODIFICATION_V02

#if defined(ETRI_MODIFICATION_V02)
#if defined(_DEBUG)
#define	ETRI_MODV2_DEBUG			(1 && ETRI_MODIFICATION_V02)		///< Debug Destroy Process in Debug MODE @ 2015 8 31 by Seok
#else
#define	ETRI_MODV2_DEBUG			(0 && ETRI_MODIFICATION_V02)		///< Debug Destroy Process in Release MODE @ 2015 8 31 by Seok
#endif
#else
#define	ETRI_MODV2_DEBUG			0
#endif


// ================================================================================================================
// 
//	e265 Version V3: Fast Encoding Algorithms
//
// ================================================================================================================
#if ETRI_MODIFICATION_V03

#define	SET_V3Condition(OPTION, CONDITION)	(OPTION && CONDITION)
// ========================================================================
// ETRI_Fast IME/FME
// ========================================================================
#define	ETRI_AMVP_ACCELERATION	0	/// 0 Defaulkt : 1 : 2: @ 2015 9 28 by Seok Recommend 0 or 2
#define	ETRI_FAST_INTEGERME		0   // 0 For now, it is not working
#define	ETRI_FAST_BIPRED 			(0 && ETRI_FAST_INTEGERME)

#if ETRI_FAST_INTEGERME
#define	ETRI_SIMD_BaseUnit_16bit	8
#define	ETRI_SIMD_LogUnit			3
#define	ETRI_HOR_nSAD				8
#define	ETRI_VER_nSAD				9
#define	ETRI_AUXMEM_SIZE			3	// 2013 5 2 by Seok : For Memory interface between IME and FME (Auxilary Memory)	
#endif
#define	ETRI_IME_DEBUGLEVEL 		2	//[0] No debug [1] Cmpare to the result of HM Original; [2] Check ETRI Algorithm Only
#define	ETRI_IME_DEBUG 			((0 && ETRI_FAST_INTEGERME)? (ETRI_IME_DEBUGLEVEL):(0))
#define	ETRI_FME_DEBUG 			(0  && ETRI_FAST_INTEGERME)

///< 2015 10 10 by Seok : FME Debug를 위하여 MultiThread TIle Processing을 잠시 중단시키기
#if (ETRI_FME_DEBUG && ETRI_MULTITHREAD)
#if (defined(ETRI_TILE_SERIAL_TEST) && !ETRI_TILE_SERIAL_TEST) 
#undef ETRI_TILE_SERIAL_TEST	
#define ETRI_TILE_SERIAL_TEST		1		///< Tile Parallel: Serial Test. @ 2015 5 25 by Seok
#endif
#endif
#else	/// 2015 10 8 by Seok : ETRI_MODIFICATION_V03
#define 	ETRI_FAST_INTEGERME		0
#define	ETRI_FAST_BIPRED 			0
#define	SET_V3Condition(OPTION, CONDITION)	(false)
#endif

// ========================================================================
// ETRI_Fast PU Mode Decision
// ========================================================================
#define	ETRI_FASTPUProcessing		1	
#if ETRI_FASTPUProcessing
#define	ETRI_FFESD   				1  	///< Fast ESD when SKLevel <= (Threshold = ETRI_FFESD ; Default= 1 @ 2015 11 19 by Seok
#define	ETRI_FastIntraSKIP 			1 	///< Fast INTRA SKIP on Prediction Stage (Chroma Prediction SKIP @ 2015 11 19 by Seok : <MD5 Error>
#define	ETRI_TEST_FASTINPRED		1	/// 2015 11 9 by Seok : For Test When this procesdure would be stable it should be equal to ETRI_FASTPUProcessing
#define	ETRI_TEST_FASTITPRED		1 	/// 2015 11 9 by Seok : For Test When this procesdure would be stable it should be equal to ETRI_FASTPUProcessing

#define	ETRI_FAST_MGITRDOQ  		1	/// Merge/INTRA RDOQ SKIP @ 2015 11 21 by Seok : [OK FIX]
#define	ETRI_FAST_MGINRDOQ  		1	/// Merge/INTER RDOQ SKIP @ 2015 11 21 by Seok
#define	ETRI_FAST_INITRDOQ   		1	/// INTER/INTRA RDOQ SKIP @ 2015 11 21 by Seok
#if ETRI_FFESD   				
#define	ETRI_FFESDLEVEL 			1	/// FFESD Activation Threshold @ 2015 12 3 by Seok
#endif

#define	ETRI_TEST_1201				1	///< Test CU Decision
#define	ETRI_TEST_1202				1	///< Test CU Decision  :: GOOD

#define	ETRI_TEST_1203				0	///<   ALL CU Prediction Mode is SKIP For Ultimate Speed Test @ 2015 12 21 by Seok
#define	ETRI_TEST_1204				0	///<   ALL CU Depth is only 1 For Ultimate Speed Test @ 2015 12 21 by Seok

#define	ETRI_FPU_STATISTICS 		(0 && !ETRI_MULTITHREAD_2)	///< For Debug, Look at the TencSlice.cpp @ 2015 11 16 by Seok

#if ETRI_CU_INTRA_MODE_INHERITANCE		///< Avoid the Corruption of algorithm @ 2015 12 15 by Seok	
#undef	ETRI_CU_INTRA_MODE_INHERITANCE
#define	ETRI_CU_INTRA_MODE_INHERITANCE	0
#endif

#else
#define	ETRI_FFESD   				0
#endif

#define 	ETRI_FAST_xGetICRateFUNCTIONS 	1
#if ETRI_FAST_xGetICRateFUNCTIONS
#define	ETRI_DOUBLE_CONVERSION	1
#endif
#define 	ETRI_SIMD_REMAIN_PART  	1
#define 	ETRI_SIMD_REMAIN_32bit     0x03 	// 2013.12.02 by pooney

#if (_ETRI_WINDOWS_APPLICATION)
#define	ETRI_CLZ(id, x)   		_BitScanReverse(&id, x)		/// MSB로 부터 Scan 하여 x의 첫번째로 1이 나타나는 bit 위치 (위치는 LSB 기준) @2016 1 30 by Seok
#define	ETRI_CFZ(id, x)   		_BitScanForward(&id, x)		/// LSB로 부터 Scan 하여 x의 첫번째로 1이 나타나는 bit 위치 (위치는 LSB 기준) @ 2016 1 30 by Seok
#endif

#define	ETRI_FastMinInt(x, y) 	(y + ((x - y) & (0 - ((x - y) >> 31)))) 	/// x, y중 Min 값을 IF 없이 찾는 MACRO @ 2016 1 30 by Seok

// ====================================================================================================================
// ETRI_BugFix
// ====================================================================================================================
#define ETRI_BUGFIX_CHECK_LEADINGPICTURES		1   ///< BugFix e265V0 for encoding over one IDR, by yhee 2015.07.15. 
#define	ETRI_BUGFIX_ETRI_EvalHAD_Cr					1   ///< BugFix e265 r18 for typo for cr TEncCU::ETRI_EvalHAD
#define	ETRI_BUGFIX_Miss_Init_ETRI_Level			1   ///< BugFix e265 r18 for missing initialize for TEncCu::ETRI_Init_CUPartitionLevel
#define	ETRI_BUGFIX_EarlySkipDecision 				1	///< have to @ 2015 12 3 by Seok : w.r.t. ETRI_FFESD

#define	ETRI_BUGFIX_ForcedSKIP 						(1 && ETRI_FASTPUProcessing)	///< It's not a bug but only difference of philosophy @ 2015.12.10 

#if ETRI_DLL_INTERFACE
#define	ETRI_BUGFIX_DLL_INTERFACE 					1 	///< BugFix For Encoder DLL to System Integration by 2016 1 14 by Seok
#endif

#ifndef ETRI_LOSSLESS_OPTIMIZATION
#define ETRI_LOSSLESS_OPTIMIZATION	0
#endif
 
#ifndef ETRI_E265_PH01
#define ETRI_E265_PH01				ETRI_MODIFICATION_V00
#endif

// ========================================================================
//	ETRI Service Functions on Orther Memory Area seperated to Encoder Core
// ========================================================================
 void ETRI_Service_Init(int iApplicationNameIdx);


// ========================================================================
//	[VUI & SEI] Parameter signaling for HDR/WCG (by Dongsan Jun, 20160124)
// ========================================================================
#define ETRI_HDR_WCG_ENCODER		1		     // 0: Default, 1: HDR/WCG

/// When ETRI_HDR_WCG_ENCODER == 1,  FULL_NBIT is automatically set 1
#if (ETRI_HDR_WCG_ENCODER && !FULL_NBIT)
#undef FULL_NBIT
#undef DISTORTION_PRECISION_ADJUSTMENT

//----------------------------------------------------------------------------------------
//	Undefine ETRI_FASTPUProcessing
//	These algorithms should be tested by algorithm to algorithm
// 	However, these algorithms are not all tested by this time (2015.04.05)
//	Therefore, if you want HDR/WCG test, set ETRI_FASTPUProcessing to be 0 at the line number 627
//    - However, It reveals that these options does not effect on the processing with FULL_NBIT.
//----------------------------------------------------------------------------------------
#if 0
#if ETRI_FASTPUProcessing
#undef ETRI_FASTPUProcessing
#if ETRI_FFESD   				
#undef ETRI_FFESDLEVEL 			
#endif
#undef ETRI_FastIntraSKIP 			
#undef ETRI_TEST_FASTINPRED		
#undef ETRI_TEST_FASTITPRED		
#undef ETRI_FAST_MGITRDOQ  		
#undef ETRI_FAST_MGINRDOQ  		
#undef ETRI_FAST_INITRDOQ   		
#undef ETRI_TEST_1201				
#undef ETRI_TEST_1202				

#define ETRI_FASTPUProcessing 		0
#define ETRI_FFESD   					0
#endif
#endif

//----------------------------------------------------------------------------------------
//	Set Full NBIT when ETRI_HDR_WCG_ENCODER is 1
//----------------------------------------------------------------------------------------
#define FULL_NBIT 1 ///< When enabled, compute costs using full sample bitdepth.  When disabled, compute costs as if it is 8-bit source video.
#if FULL_NBIT
#define DISTORTION_PRECISION_ADJUSTMENT(x) 0
#else
#define DISTORTION_PRECISION_ADJUSTMENT(x) (x)
#endif

#endif

 //============================================================================
 // Additional optimization and fast algorithms coded by hanilee.
 //============================================================================
//#if 1
#define ETRI_READ_PLANE_10BIT_SIMD                 0//!ETRI_DLL_INTERFACE
#define ETRI_TRANSFORM_TYPE_CONVERSION_SIMD        1
#if ETRI_TRANSFORM_TYPE_CONVERSION_SIMD
#include <immintrin.h>
#endif 
#define ETRI_CLEAR_FUNC_SIMD                       1
#define ETRI_SSE_SIMD_OPT                          1

#define ETRI_MOTION_COMORESSION                    1

#define ETRI_PU_2NxN_Nx2N_CODE_CLEANUP             1
#if _DEBUG
#define ETRI_DEBUG_CODE_CLEANUP                    1
#else 
#define ETRI_DEBUG_CODE_CLEANUP                    0
#endif 
#define ETRI_CODE_FURTHER_CLEANUP                  1 

#define ETRI_NO_RATE_CALC_IN_RMD                   1
#define ETRI_CHROMA_ONLY_DM_MODE                   1
#define ETRI_RESIDUAL_BYPASS_MODE                  1 
#if ETRI_RESIDUAL_BYPASS_MODE
#define ETRI_RESIDUAL_BYPASS_MODE_EXCEPT_LSCP      1
#define ETRI_RESIDUAL_BYPASS_MODE_EXCEPT_SCGF      1 
#define ETRI_RESIDUAL_BYPASS_MODE_EXCEPT_SCF       1
#define ETRI_RESIDUAL_BYPASS_MODE_EXCEPT_CALG1F    0
#define ETRI_RESIDUAL_BYPASS_MODE_EXCEPT_CALG2F    0
#endif
#define ETRI_SIMPLE_MODE_SKIP_IN_RMD               1 
#if ETRI_SIMPLE_MODE_SKIP_IN_RMD
#define ETRI_B_SLICE_SIMPLE_MODE_SKIP_IN_RMD       0
#define ETRI_TWO_SIMPLE_MODE_SKIP_IN_RMD           1
#endif

#define ETRI_VERTICAL_SEARCH_RANGE_LIMIT           1 
#if ETRI_VERTICAL_SEARCH_RANGE_LIMIT
#define ETRI_VERTICAL_SEARCH_RANGE_LIMIT_MINUS_2   1
#endif

#if ETRI_REVISE_ECU
#define ETRI_REVISE_ECU_INTRA_CHECKING             1
#endif 

#define ETRI_FASTPUPROCESSING_TURN_OFF		       1
#if ETRI_FASTPUPROCESSING_TURN_OFF		
#if ETRI_FastIntraSKIP
#undef ETRI_FastIntraSKIP
#define ETRI_FastIntraSKIP                         0
#endif 
#if ETRI_TEST_FASTINPRED            
#undef ETRI_TEST_FASTINPRED
#define ETRI_TEST_FASTINPRED                       0
#endif 
#if ETRI_TEST_FASTITPRED
#undef ETRI_TEST_FASTITPRED         
#define ETRI_TEST_FASTITPRED                       0 
#endif 
#if ETRI_TEST_1201
#undef ETRI_TEST_1201
#define ETRI_TEST_1201                             0
#endif 
#if ETRI_TEST_1202
#undef ETRI_TEST_1202       
#define ETRI_TEST_1202                             0
#endif 
#if !ETRI_CU_INTRA_MODE_INHERITANCE
#undef ETRI_CU_INTRA_MODE_INHERITANCE
#define ETRI_CU_INTRA_MODE_INHERITANCE             1
#endif 
#if ETRI_FAST_xGetICRateFUNCTIONS
#undef ETRI_FAST_xGetICRateFUNCTIONS
#define ETRI_FAST_xGetICRateFUNCTIONS              0
#if ETRI_DOUBLE_CONVERSION
#undef ETRI_DOUBLE_CONVERSION
#define ETRI_DOUBLE_CONVERSION                     0 
#endif 
#endif 
#endif 
#ifdef __GNUC__
#define ETRI_GNUC_TIME_FUNCTION                    1 
#endif 

#define ETRI_OMP_MOTION_COMPRESSION                !ETRI_SliceFrameEncoding
#if ETRI_OMP_MOTION_COMPRESSION
#define MAX_NUM_MOTION_THREADS                     6
#endif 
//#endif
#endif	//#ifndef	ETRI_HEVC_DEFINE_H
