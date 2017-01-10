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

#include <hevc_base.h>
#include <windows.h>
#include <fstream>

#include <hevc_shared_buffer.h> /// wsseo@2014-06-20
using namespace hevc;

// ====================================================================================================================
// Helper functions
// ====================================================================================================================
bool	InitBuffer();
void	ReleaseBuffer();


#include "TAppDLLEncoder.h"

#ifdef __FILENAME__
#undef __FILENAME__
#define __FILENAME__ 	"TAppDLLEncoderMain.cpp"
#endif

using namespace std;

// ====================================================================================================================
// Main function
// ====================================================================================================================
#define 	ETRI_LimitIteration 32
#define 	ETRI_DBG_Main	0

int main(int argc, char* argv[])
{
	// --------------------------------------------------------------------------------------------
	// Default Initialization  : for DLL Mandatory 
	// --------------------------------------------------------------------------------------------

	HINSTANCE	m_hdll = NULL;
	HANDLE  	m_hMDLL_Event = NULL;		/// 2014 5 17 by Seok 

#if ETRI_SINGLE_ENCODER_ENABLE
// 2013 12 21 by Seok : Saingle/SmartGuru DLL
	if ((m_hdll = LoadLibrary("TDllEncoder_HM101r_x86v09F135.dll")) == NULL)
		error_dll("LoadLibrary(lib_ETRI_HEVC) Error", 0);
#else
// 2013 12 21 by Seok : Multiple Encoder DLL
	if ((m_hdll = LoadLibrary("TDllMultiEncoder_HM101r_x86v09F135.dll")) == NULL)
		error_dll("LoadLibrary(lib_ETRI_HEVC) Error", 0);
#endif

	ETRI_HEVC_Constructor 	= (void *(__cdecl *)(int argc, char *argv[]))GetProcAddress(m_hdll, "ETRI_EncoderConstruct");
	DLLNull_ErrorMSG      	(ETRI_HEVC_Constructor,  "ETRI_EncoderConstruct() not found", 0);

	ETRI_HEVC_Init_func    	= (bool (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_EncoderInitilization");
	DLLNull_ErrorMSG       	(ETRI_HEVC_Init_func,  "ETRI_HEVC_Init_func() not found", 0);

	ETRI_HEVC_Encode_func  	= (bool (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_EncoderMainFunc");
	DLLNull_ErrorMSG       	(ETRI_HEVC_Encode_func,  "ETRI_HEVC_Encode_func() not found", 0);

	ETRI_HEVC_Destroyer    	= (void (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_EncoderDestroy");
	DLLNull_ErrorMSG       	(ETRI_HEVC_Destroyer,  "ETRI_HEVC_Destroyer() not found", 0);

	ETRI_HEVC_Service_func 	= (void (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_printSummary");
	DLLNull_ErrorMSG       	(ETRI_HEVC_Service_func,  "ETRI_HEVC_Service_func() not found", 0);

	ETRI_HEVC_GetEncInterface = (ETRI_Interface *(__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_GetEncInterface");
	DLLNull_ErrorMSG       	(ETRI_HEVC_GetEncInterface,  "ETRI_HEVC_GetEncInterface() not found", 0);

	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);			// IDLE_PRIORITY_CLASS, NORMAL_, HIGH_, REALTIME_
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);	// THREAD_PRIORITY_LOWEST, _BELOW_NORMAL,  _NORMAL, _ABOVE_NORMAL, _HIGHEST, _TIME_CRITICAL

	// --------------------------------------------------------------------------------------------
	// create application encoder class
	// --------------------------------------------------------------------------------------------
	void *m_hTAppEncTop             = ETRI_HEVC_Constructor( argc, argv );					// <Mandatoty> 
	ETRI_Interface *m_pEncInterface = ETRI_HEVC_GetEncInterface( m_hTAppEncTop );

	m_pEncInterface->CTRParam.bInBufferOff 	= true;	//	When Input Data is gotton From  Memory Pool  <Mandatoty>
	m_pEncInterface->CTRParam.bOutBufferOff = true;	//  When Outnput Data is put to  Memory Pool   <Mandatoty>
	m_pEncInterface->CTRParam.bRecBufferOff = true;	//  Reconstruction data Buffer is allocated in the DLL. However, it is not used. 

	//  =========== Application : you can modify the codes to get or put data from memory, file, device and etc.....

	fstream	IYUVFile, bitstreamFile;	// FIle : (IYUVFile : YUV Planary Data) (bitstreamFile : HEVC Bitstream)		

	// --------------------------------------------------------------------------------------------
	//  Default : Encoder Initialization  : 
	// --------------------------------------------------------------------------------------------
	/*   We can get the following data after ETRI_HEVC_Init_func from EncoderIF
		Input Data            : EncoderIF->ptrData 		     (Input YUV), 
		Size of Input Data : EncoderIF->FrameSize		     (Frame Size : Pixels per frame : fixed data)
		Output Data          : EncoderIF->AnnexBData	     (HEVC stream)
		Size of Input Data : EncoderIF->AnnexBFrameSize (Frame Size : HEVC stream size : variable data)
	*/

	/* For Change of Frames to be Encoded  /// 2015 6 14 by Seok */
	unsigned int	uiForcedFramesTobeEncoded         = 8;		//if This value is larger than 0 then ForcedFramesToBeEncoded is active
	m_pEncInterface->CTRParam.bChangeFrameTobeEncoded = false;	//No Change of Frames to be Encoded

	// --------------------------------------------------------------------------------------------
	// encoder Initialization 
	// --------------------------------------------------------------------------------------------	

	Init_Encode(m_hTAppEncTop);

	bool	_bOperation       = true; 
	bool	_iMultipleEncoder = m_pEncInterface->CTRParam.uiMultipleEncoder > 0;
	bool	_InfiniteProcess  = (ETRI_getInfiniteProcessing(m_pEncInterface))? true : false;
	bool	_UseInternalTimer = true;

	int 	_iLimitFrames, _inTestGOPs;
	int 	_inFrames, *_ptrinFrames = &_inFrames;	
	int 	_FramesTobeEncoded; 
	int 	_Available_Encoder; 
	int 	_nGOPsforME;

	int		*_MonitorCondition;

	_FramesTobeEncoded 	= m_pEncInterface->CTRParam.iFramestobeEncoded;
	_Available_Encoder 	= m_pEncInterface->CTRParam.iNumEncoders;
	_nGOPsforME			= m_pEncInterface->CTRParam.iNumofGOPforME;

	_iLimitFrames	= (_iMultipleEncoder)? (_FramesTobeEncoded * _nGOPsforME *  (int)_Available_Encoder) : m_pEncInterface->nGOPsize;	

	ETRI_dbgMsg(ETRI_DBG_Main, 2, "[LINE:%d %s %s] EncoderIF->nFrameStartOffset : %x \n", __LINE__, __FUNCTION__, __FILENAME__, m_pEncInterface->nFrameStartOffset);
	ETRI_dbgMsg(ETRI_DBG_Main, 2, "[LINE:%d %s %s] _iLimitFrames : %d\n", __LINE__, __FUNCTION__, __FILENAME__, _iLimitFrames);

	// ===================== Simulation : Construction and Initialization of Memory pool =====================
	
	/// wsseo@2014-06-20
	// memory pool construct

	Memory_Pool_Constructor(m_pEncInterface);
	Memory_Pool_Initialization(IYUVFile, bitstreamFile, m_pEncInterface);

	// --------------------------------------------------------------------------------------------
	// Ready encoding
	// --------------------------------------------------------------------------------------------	
	_MonitorCondition = &m_pEncInterface->iNumEncoded;
	ETRI_dbgMsg(true, 2, "[LINE:%d %s %s] MultipleEncoder : %d \n", __LINE__, __FUNCTION__, __FILENAME__, _iMultipleEncoder);
	if (_iMultipleEncoder)
	{
		_MonitorCondition = m_pEncInterface->m_piFrameRcvd;
		m_hMDLL_Event = CreateEvent(NULL, FALSE, FALSE, NULL);
		m_pEncInterface->CTRParam.hDllEvent = m_hMDLL_Event;
		ETRI_dbgMsg(ETRI_DBG_Main, 0, "\n[LINE: %d %s %s] m_hMDLL_Event : %x \n", __LINE__, __FUNCTION__, __FILENAME__, m_hMDLL_Event);
	}

	/// 2014 6 10 by Seok : Test Parameter for Infinite Processing
	_inTestGOPs = (_InfiniteProcess)? ETRI_LimitIteration : 0;	/// 2014 6 10 by Seok : For 4 GOPs to Infinite Processing
	_inFrames   	=  0;

	// --------------------------------------------------------------------------------------------
	// encoding
	// --------------------------------------------------------------------------------------------	
	//--------------------------      starting time       --------------------------
	m_pEncInterface->ServiceFunctionIndex[0] = true;				// TRUE : Show the result of encoding
	m_pEncInterface->ServiceFunctionIndex[1] = true;				// TRUE : Show the Verification of HEVC Header Information in Debug.trxt
	m_pEncInterface->ServiceFunctionIndex[2] = _UseInternalTimer;	// TRUE : When You use Internal Timer in DLL
	m_pEncInterface->ServiceFunctionIndex[3] = !_UseInternalTimer;	// FALSE : When You use Internal Timer in DLL
	m_pEncInterface->ServiceFunctionIndex[4] = false;				// For Using Outernal Timer in DLL

	ETRI_HEVC_Service_func(m_hTAppEncTop);							// Service Function, Not necessary


#if 0

	m_pEncInterface->CTRParam.bChangeFrameTobeEncoded = true;
	m_pEncInterface->CTRParam.iFramestobeEncoded]     = uiForcedFramesTobeEncoded;	/// 2015 6 14 by Seok : Test COde

#endif

	// ===================== Simulation : Encoding =====================
	double dResult; long lBefore = clock(); 

	while (_bOperation)
	{
		if (*m_pEncInterface->m_piFrameRcvd)
			Memory_Pool_PutFrame(bitstreamFile, m_pEncInterface);
	
		ETRI_RefreshEncoder(m_pEncInterface, false);

		/// 2014 6 7 by Seok : Check the Infinite DLL Processing within finite GOPs to ES bits
		if (ETRI_dbgTest(_Available_Encoder, _FramesTobeEncoded, _ptrinFrames, _inTestGOPs, _InfiniteProcess))	 break;
	
		_bOperation = !m_pEncInterface->bEos;
		if (m_pEncInterface->bEos)	continue;
	
		Memory_Pool_GetFrame(IYUVFile, m_pEncInterface);
		
		//ETRI_HEVC_Encode_func(blockInfo);
		Encode(m_hTAppEncTop);
		_inFrames++;

		/// 2014 6 17 by Seok : Debug
		if( *_MonitorCondition )
		{
			bool _dbgkey = false; int _dbgStyle = 1, iIdx = 0;
			char* _Code;
			unsigned int  _TotalBits = 0;


			ETRI_dbgMsg(_dbgkey, _dbgStyle, "======= EStoTS Param ====\n");
			ETRI_dbgMsg(_dbgkey, _dbgStyle, "[LINE:%d %s] *EncoderIF->iNumEncoded: %d \n", __LINE__, __FUNCTION__, m_pEncInterface->iNumEncoded);
			ETRI_dbgMsg(_dbgkey, _dbgStyle, "[Index FType DTS  PTS  TStap] Offset   (bytes:bits)\n");
			
			for( int k = 0; k < m_pEncInterface->iNumEncoded; k++ )
			{
				ETRI_dbgMsg(_dbgkey, _dbgStyle, "[%3d: %3d : %3d : %3d] %7d %8d ", k, 
												m_pEncInterface->nFrameTypeInGop[k],
												m_pEncInterface->nPicDecodingOrder[k],
												m_pEncInterface->nPicPresentationOrder[k],
												m_pEncInterface->nFrameStartOffset[k],
												m_pEncInterface->nFrameStartOffset[k] << 3 );

				iIdx       +=  k == 0 ? m_pEncInterface->nFrameStartOffset[k-1] : 0;
				_Code       = (char *)m_pEncInterface->AnnexBData + iIdx;
				_TotalBits += m_pEncInterface->nFrameStartOffset[k] << 3;

				ETRI_dbgMsg(_dbgkey, _dbgStyle, "Code:@%7x %x %x %x %x \n", iIdx, _Code[0], _Code[1], _Code[2], _Code[3]); 				
			}
		}

		/// 2014 6 3 by Seok : Debug
		fprintf(stderr, ".");
	}
 
	dResult = (double)(clock()-lBefore) / CLOCKS_PER_SEC;
	m_pEncInterface->CTRParam.dbTotalEncodingTime = dResult;

	// ===================== Simulation : Encoding : Fin ===============

	//Service Function for	the result of Encoding
	m_pEncInterface->ServiceFunctionIndex[3] = _UseInternalTimer;	// TRUE : When You use Internal Timer in DLL
	m_pEncInterface->ServiceFunctionIndex[4] =  true;				// TRUE : When You use Internal Timer in DLL

	ETRI_HEVC_Service_func(m_hTAppEncTop);							// Service Function, Not necessary

	bool _dbgkey = false; int _dbgStyle = 0;
	ETRI_dbgMsg(_dbgkey, _dbgStyle, "ETRI_HEVC_Service_func @ TAPPDLLEncoderMain.cpp @Line : %d \n", __LINE__);

	// --------------------------------------------------------------------------------------------
	//  encoder Destroy : destroy application encoder class
	// --------------------------------------------------------------------------------------------

	ETRI_HEVC_Destroyer(&m_hTAppEncTop);	//DLL Function
	
	//Close the file and Free a buffer for Input Process
	fprintf(stdout, "Total Encoding Time: %12.3f sec.\n", dResult);

	IYUVFile.close();
	bitstreamFile.close();
	FreeLibrary(m_hdll);


	//	===================== Simulation : Final process of Memory pool =====================
	if (_iMultipleEncoder)
	{
		CloseHandle(m_hMDLL_Event);
	}
	
	/// 2014 4 28 by Seok : Service 
	printf("Compiled @%s, @%s in TAppDllEncoderMain.cpp\n", __DATE__, __TIME__);

	return 0;
}

/**
----------------------------------------------------------------------------------
@fn			bool InitBuffer()
@param		nYuvWidth			image width
@param		nYuvHeight			image height
@param		nYuvBitsPerPixel	bits per pixel
@return		succeeded or not
@brief		일반적인 내용을 출력합니다.
@endcode
----------------------------------------------------------------------------------
*/
bool	InitBuffer(	u32 nYuvWidth, 
					u32 nYuvHeight, 
					u32 nYuvBitsPerPixel )
{
	return false;
}

/**
----------------------------------------------------------------------------------
@fn			void PrintExplanation()
@param		none
@return		none
@brief		일반적인 내용을 출력합니다.
@endcode
----------------------------------------------------------------------------------
*/
void	ReleaseBuffer()
{

}
