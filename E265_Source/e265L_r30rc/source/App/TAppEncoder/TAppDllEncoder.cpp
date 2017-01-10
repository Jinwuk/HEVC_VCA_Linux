/*
*********************************************************************************************

   Copyright (c) 2015 Electronics and Telecommunications Research Institute (ETRI) All Rights Reserved.

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

#include "TDllEncoder.h"
#include "TAppDllEncoder.h"

#if (_ETRI_WINDOWS_APPLICATION)
#include <windows.h>
#else
#include <dlfcn.h>
#include <sys/resource.h>
#include <pthread.h>
#include "ETRI_HEVC_define.h"
#endif
#include <fstream>

#ifdef __FILENAME__
#undef __FILENAME__
#define __FILENAME__ 	"TAppDllEncoder.cpp"
#endif
using namespace std;


/// This value is the number of iterations when the Infinite Processing is active @ 2016 1 15 by Seok
#define	NUM_ITERATION	1	/// Recommend (MAX_INT >> 3) or (MAX_INT >> 4) 

int main( int argc, char* argv[] )
{
	std::string strExeFilePath = argv[0];
	strExeFilePath = strExeFilePath.substr( 0, strExeFilePath.rfind('\\') + 1 );
	::_chdir( strExeFilePath.c_str() );

	std::string strExeFileName = argv[0];
	strExeFileName = strExeFileName.substr( strExeFileName.rfind('\\') + 1 );	
#if (_ETRI_WINDOWS_APPLICATION)
	::SetConsoleTitleA( strExeFileName.c_str() );
#else
	printf("%c]0;%s%c", '\033', strExeFileName.c_str(), '\007');
#endif

	// --------------------------------------------------------------------------------------------
	// Load DLL
	// --------------------------------------------------------------------------------------------		
#if (_ETRI_WINDOWS_APPLICATION)
	HINSTANCE	m_hdll = ::LoadLibrary( "TDllEncoder_e265SingleDLL.dll" ); 
	HANDLE  	m_hMDLL_Event = NULL;
	if( m_hdll == NULL )
	{
		error_dll( "main() - ::LoadLibrary failed.", 0 );
	}
	else
	{
		ETRI_HEVC_Constructor 	= (void *(__cdecl *)(int argc, char *argv[]))GetProcAddress(m_hdll, "ETRI_EncoderConstruct");
		DLLNull_ErrorMSG(ETRI_HEVC_Constructor, "ETRI_EncoderConstruct() not found", 0);

		ETRI_HEVC_Init_func    	= (bool (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_EncoderInitilization");
		DLLNull_ErrorMSG(ETRI_HEVC_Init_func, "ETRI_HEVC_Init_func() not found", 0);

		ETRI_HEVC_Encode_func  	= (bool (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_EncoderMainFunc");
		DLLNull_ErrorMSG(ETRI_HEVC_Encode_func, "ETRI_HEVC_Encode_func() not found", 0);

		ETRI_HEVC_Destroyer    	= (void (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_EncoderDestroy");
		DLLNull_ErrorMSG(ETRI_HEVC_Destroyer, "ETRI_HEVC_Destroyer() not found", 0);

		ETRI_HEVC_Service_func 	= (void (__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_printSummary");
		DLLNull_ErrorMSG(ETRI_HEVC_Service_func, "ETRI_HEVC_Service_func() not found", 0);

		ETRI_HEVC_GetEncInterface = (ETRI_Interface *(__cdecl *)(void *hTAppEncTop))GetProcAddress(m_hdll, "ETRI_GetEncInterface");
		DLLNull_ErrorMSG(ETRI_HEVC_GetEncInterface, "ETRI_HEVC_GetEncInterface() not found", 0);
	}

	::SetPriorityClass( GetCurrentProcess(), REALTIME_PRIORITY_CLASS );			// IDLE_PRIORITY_CLASS, NORMAL_, HIGH_, REALTIME_
	::SetThreadPriority( GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL );	// THREAD_PRIORITY_LOWEST, _BELOW_NORMAL,  _NORMAL, _ABOVE_NORMAL, _HIGHEST, _TIME_CRITICAL
#else
	void *m_hdll;
	if ((m_hdll = dlopen("libTDllEncoder.so", 2)) == NULL)
	{
		error_dll("LoadLibrary(lib_ETRI_HEVC) Error", 0);
	}
	else
	{
		ETRI_HEVC_Constructor 	= (void *(__cdecl *)(int argc, char *argv[]))dlsym(m_hdll, "ETRI_EncoderConstruct");
		DLLNull_ErrorMSG      	(ETRI_HEVC_Constructor,  "ETRI_EncoderConstruct() not found", 0);

		ETRI_HEVC_Init_func    	= (bool (__cdecl *)(void *hTAppEncTop))dlsym(m_hdll, "ETRI_EncoderInitilization");
		DLLNull_ErrorMSG       	(ETRI_HEVC_Init_func,  "ETRI_HEVC_Init_func() not found", 0);

		ETRI_HEVC_Encode_func  	= (bool (__cdecl *)(void *hTAppEncTop))dlsym(m_hdll, "ETRI_EncoderMainFunc");
		DLLNull_ErrorMSG       	(ETRI_HEVC_Encode_func,  "ETRI_HEVC_Encode_func() not found", 0);

		ETRI_HEVC_Destroyer    	= (void (__cdecl *)(void *hTAppEncTop))dlsym(m_hdll, "ETRI_EncoderDestroy");
		DLLNull_ErrorMSG       	(ETRI_HEVC_Destroyer,  "ETRI_HEVC_Destroyer() not found", 0);

		ETRI_HEVC_Service_func 	= (void (__cdecl *)(void *hTAppEncTop))dlsym(m_hdll, "ETRI_printSummary");
		DLLNull_ErrorMSG       	(ETRI_HEVC_Service_func,  "ETRI_HEVC_Service_func() not found", 0);

		ETRI_HEVC_GetEncInterface = (ETRI_Interface *(__cdecl *)(void *hTAppEncTop))dlsym(m_hdll, "ETRI_GetEncInterface");
		DLLNull_ErrorMSG(ETRI_HEVC_GetEncInterface, "ETRI_HEVC_GetEncInterface() not found", 0);
	}

	// should be confirmed - shcho, 160524
	setpriority(PRIO_PROCESS, getpid(), -20);

	pthread_t thId = pthread_self();
	pthread_attr_t thAttr;
	int policy = 0;
	int max_prio_for_policy = 0;

	pthread_attr_init(&thAttr);
	pthread_attr_getschedpolicy(&thAttr, &policy);
	max_prio_for_policy = sched_get_priority_max(policy);

	pthread_setschedprio(thId, max_prio_for_policy);
	pthread_attr_destroy(&thAttr);
	
#endif
	// --------------------------------------------------------------------------------------------
	// Create EncoderIf
	// --------------------------------------------------------------------------------------------	
	void *m_hTAppEncTop             = ETRI_HEVC_Constructor( argc, argv );					// <Mandatoty> 
	ETRI_Interface *m_pEncInterface = ETRI_HEVC_GetEncInterface( m_hTAppEncTop );
	
	/////< Control Param ?? for what? 
	/// --------------------------------------------------------------------------------------------
	/// TRUE   : Encoder provide a Allocated Memory for IO Processing
	/// FALSE  : Encoder doesn't allocate Memory for IO Processing. 
	/// 			When it is false, an allocated memory outside of Encoder should be exist
	///		- InBuffer : just mapping the address of outside memory to the pointer in Encoder with Memory_Pool_GetFrame
	/// --------------------------------------------------------------------------------------------
	m_pEncInterface->CTRParam.bInBufferOff   =  true; 	//  When Input Data is gotton From  Memory Pool  <Mandatoty>
	m_pEncInterface->CTRParam.bOutBufferOff = true;    	//  When Outnput Data is put to  Memory Pool   <Mandatoty>
	m_pEncInterface->CTRParam.bRecBufferOff = true;	//  Reconstruction data Buffer is allocated in the DLL. However, it is not used. 

	// --------------------------------------------------------------------------------------------
	// Input/Output Data Setting
	// --------------------------------------------------------------------------------------------	
	fstream	IYUVFile, bitstreamFile;			// FIle : (IYUVFile : YUV Planary Data) (bitstreamFile : HEVC Bitstream)		

	// --------------------------------------------------------------------------------------------
	// Input SliceDLL Index
	// --------------------------------------------------------------------------------------------	
	short   _nSliceIndex = 0; //0-7 Input Slice Index by yhee	
	m_pEncInterface->CTRParam.sSliceIndex = _nSliceIndex;

	///< Initilaizie system interface 
	Init_Encode( m_hTAppEncTop );
	
	// --------------------------------------------------------------------------------------------
	// Test Infinite Encoding
	// --------------------------------------------------------------------------------------------	
	bool	_bOperation = true; 
	bool	_InfiniteProcess  = ETRI_getInfiniteProcessing(m_pEncInterface) ? true : false;
	//bool	_UseInternalTimer = true;

	int 	_inTestGOPs;
	int 	_inFrames, *_ptrinFrames = &_inFrames;	
	int 	_FramesTobeEncoded; 
	int 	_Available_Encoder; 
	int 	_nGOPsforME;

	int		*_MonitorCondition;

	///TImestamp Update @ 2016 1 16 by Seok
	int	_iFrameIndexinIDRGOP = 0;

	_FramesTobeEncoded 	= m_pEncInterface->CTRParam.iFramestobeEncoded;
	_Available_Encoder 	= m_pEncInterface->CTRParam.iNumEncoders;
	_nGOPsforME			= m_pEncInterface->CTRParam.iNumofGOPforME;

	// --------------------------------------------------------------------------------------------
	// Memory Pool IO
	// --------------------------------------------------------------------------------------------	
	Memory_Pool_Constructor( m_pEncInterface );
	Memory_Pool_Initialization( IYUVFile, bitstreamFile, m_pEncInterface ); //File open	
	Memory_Pool_PushInputData( IYUVFile, bitstreamFile, m_pEncInterface ); //If Full IO is active then this function is also active

	// --------------------------------------------------------------------------------------------
	// encoding
	// --------------------------------------------------------------------------------------------	
	// ===================== Set the variables =====================
	_MonitorCondition = &m_pEncInterface->iNumEncoded;
	_inTestGOPs = (_InfiniteProcess) ? NUM_ITERATION : 0;	///< The number of IDRperiod iteration for infinite process
	_inTestGOPs = _inTestGOPs * _nGOPsforME; //infinite loop
	_inFrames   	=  0;

	m_pEncInterface->ServiceFunctionIndex[0] = true;		// TRUE: Print encoding summary	
	// --------------------------------------------------------------------------------------------
	// Debug Information for Infinite Processing  
	// If tou want to print out the information of infinite processing, 
	// Let "true" to be the condition of the following "If" code;
	// --------------------------------------------------------------------------------------------	

#if ETRI_THREADPOOL_OPT
  fprintf(stderr, "ETRI_THREADPOOL_OPT \n");
#if ETRI_TILE_THEAD_OPT
  fprintf(stderr, "ETRI_TILE_THEAD_OPT 1 \n");
#endif
#if ETRI_FRAME_THEAD_OPT
  fprintf(stderr, "ETRI_FRAME_THEAD_OPT 1 \n");
#endif
#if ETRI_THREAD_LOAD_BALANCING
  fprintf(stderr, "ETRI_THREAD_LOAD_BALANCING 1 \n");
#endif
#if ETRI_COPYTOPIC_MULTITHREAD
  fprintf(stderr, "ETRI_COPYTOPIC_MULTITHREAD 1 \n");
#endif
#else
  fprintf(stderr, "ETRI_THREADPOOL_OPT off \n");
#endif

	if (_InfiniteProcess && true)
	{
		FILE* dbgType = stderr;
		EDPRINTF(dbgType,"-------------Information of Infinite Processing--------------\n");
		EDPRINTF(dbgType," The Number of GOP in Infinite Processing        : %d \n", _inTestGOPs);
		EDPRINTF(dbgType," The Number of Available Encoder                 : %d \n", _Available_Encoder);
		EDPRINTF(dbgType," The Number of_FramesTobeEncoded (IDR Length)    : %d \n", _FramesTobeEncoded);
		EDPRINTF(dbgType," Total Number of Frames for Infinite Processing  : %d \n", (_FramesTobeEncoded * _Available_Encoder * _inTestGOPs));
	}

	if (m_pEncInterface->CTRParam.iFullIORDProcess > 0)
	{
		FILE* dbgType = stderr;
		EDPRINTF(dbgType,"-------------Information of Full RD IO Processing--------------\n");
		EDPRINTF(dbgType," The Number of GOP in Infinite Processing        : %d \n", _nGOPsforME);
		EDPRINTF(dbgType," The Number of Available Encoder                 : %d \n", _Available_Encoder);
		EDPRINTF(dbgType," The Number of_FramesTobeEncoded (IDR Length)    : %d \n", _FramesTobeEncoded);
		EDPRINTF(dbgType," Total Number of Frames for Full RD IO Processing  : %d \n", (_FramesTobeEncoded * _Available_Encoder * _nGOPsforME));
	}
	
	// --------------------------------------------------------------------------------------------	

	double dResult;
#if (_ETRI_WINDOWS_APPLICATION)
	long lBefore = clock(); 
#else
	timespec lBefore;
	clock_gettime(CLOCK_MONOTONIC, &lBefore);
#endif

	// ===================== Simulation : Encoding =====================
	while( _bOperation )
	{

		if( *m_pEncInterface->m_piFrameRcvd )
			Memory_Pool_PutFrame(bitstreamFile, m_pEncInterface); ///< Link OutlockInfo.pBuffPtr to File ptr		

		///<Infinite process stop condition
		if( ETRI_dbgTest(_Available_Encoder, _FramesTobeEncoded, _ptrinFrames, _inTestGOPs, _InfiniteProcess) )
			break;

		///<Refresh Encoder every IDR based on  _FramesTobeEncoded
		ETRI_RefreshEncoder( m_pEncInterface, false );

		_bOperation = !m_pEncInterface->bEos;
		if( m_pEncInterface->bEos )	
			continue;

		Memory_Pool_GetFrame( IYUVFile, m_pEncInterface ); ///< link InblockInfo.pBuffPtr to IYUVFile
		m_pEncInterface->nTimestamp[_iFrameIndexinIDRGOP] = _inFrames; 
//		m_pEncInterface->nTimestamp[0] = _inFrames;	

		///< Encoding
		Encode( m_hTAppEncTop );

		///< Encoding count	
		_inFrames++;

		///< Frame Index Update @ 2016 1 16 by Seok
		_iFrameIndexinIDRGOP = ( *_MonitorCondition )? 0 : (_iFrameIndexinIDRGOP+1);

		///< Print TS Param data	
		if( *_MonitorCondition )
		{		
			bool _dbgkey = false; //false: no print dbgmsg, true: print dbgmsg
			int _dbgStyle = 1, iIdx = 0;
			char* _Code;

			ETRI_dbgMsg( _dbgkey, _dbgStyle, "======= EStoTS Param ====\n" );
			ETRI_dbgMsg( _dbgkey, _dbgStyle, "[LINE:%d %s] m_pEncInterface->iNumEncoded: %d \n", __LINE__, __FUNCTION__, m_pEncInterface->iNumEncoded );
			ETRI_dbgMsg( _dbgkey, _dbgStyle, "[Index FType DTS  PTS  TStap] Offset   (bytes:bits)\n");
			
			//DLLInterface Parsing format, Pls do not change the print format.
			for( int k = 0; k < m_pEncInterface->iNumEncoded; k++ )
			{
				ETRI_dbgMsg(_dbgkey, _dbgStyle, "[%3d:%3d:%3d:%3d:%3d:%7d:%8d:", k, 
																				 m_pEncInterface->nFrameTypeInGop[k],
																				 //EncoderIF->nSliceIndex[k],
																				 m_pEncInterface->nPicDecodingOrder[k],
																				 m_pEncInterface->nPicPresentationOrder[k],
																				 m_pEncInterface->nTimestamp[k],
																				 m_pEncInterface->nFrameStartOffset[k],
																				 (m_pEncInterface->nFrameStartOffset[k] << 3) );
				iIdx += ((k) ? m_pEncInterface->nFrameStartOffset[k-1] : 0);
				_Code = (char *)m_pEncInterface->AnnexBData + iIdx;
				ETRI_dbgMsg(_dbgkey, _dbgStyle, "Code @%7x:%x:%x:%x:%x: \n", iIdx, _Code[0], _Code[1], _Code[2], _Code[3]); 
			}
		}

		///< Print Summary
		#if 0
		if((_inFrames % m_pEncInterface->CTRParam.iFramestobeEncoded) == 0)
		{
			ETRI_HEVC_Service_func( m_hTAppEncTop );
		}			
		#endif
	}

#if (_ETRI_WINDOWS_APPLICATION)
	/// Check Finish TIme @ 2016 2 1 by Seok
	dResult = (double)(clock()-lBefore) / CLOCKS_PER_SEC;
#else
	timespec iCurrTime;
	clock_gettime(CLOCK_MONOTONIC, &iCurrTime);
	dResult = (iCurrTime.tv_sec - lBefore.tv_sec);
	dResult += (iCurrTime.tv_nsec - lBefore.tv_nsec) / 1000000000.0;
#endif

	///< PrintSummary	
	//ETRI_HEVC_Service_func( m_hTAppEncTop );		

	///< Destroy
	Memory_Pool_Free(m_pEncInterface);
	ETRI_HEVC_Destroyer( &m_hTAppEncTop );	

	///< Print MainEncodingTime
	fprintf(stdout, "Total Encoding Time: %12.3f sec.\n", dResult);

	IYUVFile.close();
	bitstreamFile.close();

#if (_ETRI_WINDOWS_APPLICATION)
	FreeLibrary(m_hdll);  
#else
	dlclose(m_hdll);  
#endif

#if 0
	/// Sometimes... It is not necessary to conduct ::getch(). @ 2016 1 4 by Seok
	EDPRINTF(stderr, "The Encoder wait for your key stroke. please push ENTER key for stable finish!! ^.^\n");	
	::getchar();

#endif
	return 0;
}
