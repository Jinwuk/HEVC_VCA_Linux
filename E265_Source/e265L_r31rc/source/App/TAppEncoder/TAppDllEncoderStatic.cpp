#include <windows.h>
#include <fstream>
#include <stdio.h>
#include <time.h>
#include "TDllEncoder.h"
#include "TAppDllEncoderStatic.h"

using namespace std;
#define	NUM_ITERATION	1


int main(int argc, char* argv[])
{
	std::string strExeFilePath = argv[0];
	strExeFilePath = strExeFilePath.substr( 0, strExeFilePath.rfind('\\') + 1 );
	::_chdir( strExeFilePath.c_str() );

	std::string strExeFileName = argv[0];
	strExeFileName = strExeFileName.substr( strExeFileName.rfind('\\') + 1 );	
	::SetConsoleTitleA( strExeFileName.c_str() );

	// --------------------------------------------------------------------------------------------
	// Create EncoderIf
	// --------------------------------------------------------------------------------------------	
	EncoderMain EncMain;

	void *m_hTAppEncTop             = EncMain.EncoderConstruct( argc, argv );					// <Mandatoty> 
	ETRI_Interface *m_pEncInterface = EncMain.GetEncInterface();
	
	/////< Control Param ?? for what? 
	m_pEncInterface->CTRParam.bInBufferOff 	= true;	//	When Input Data is gotton From  Memory Pool  <Mandatoty>
	m_pEncInterface->CTRParam.bOutBufferOff = true;	//  When Outnput Data is put to  Memory Pool   <Mandatoty>
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
	EncMain.EncoderInitilization();
	
	// --------------------------------------------------------------------------------------------
	// Test Infinite Encoding
	// --------------------------------------------------------------------------------------------	
	bool	_bOperation       = true; 
	bool	_iMultipleEncoder = m_pEncInterface->CTRParam.uiMultipleEncoder > 0;
	bool	_InfiniteProcess  = ETRI_getInfiniteProcessing(m_pEncInterface) ? true : false;
	//bool	_UseInternalTimer = true;

	int 	_iLimitFrames, _inTestGOPs;
	int 	_inFrames, *_ptrinFrames = &_inFrames;	
	int 	_FramesTobeEncoded; 
	int 	_Available_Encoder; 
	int 	_nGOPsforME;

	int		*_MonitorCondition;

	_FramesTobeEncoded 	= m_pEncInterface->CTRParam.iFramestobeEncoded;
	_Available_Encoder 	= m_pEncInterface->CTRParam.iNumEncoders;
	_nGOPsforME			= m_pEncInterface->CTRParam.iNumofGOPforME;
#if ETRI_DLL_GOPParallel
	_iLimitFrames = _FramesTobeEncoded; //intra period
	//_iLimitFrames = (_FramesTobeEncoded * _nGOPsforME);
#else
	_iLimitFrames = EncoderIF->nGOPsize;
#endif	
	
	
	// --------------------------------------------------------------------------------------------
	// Memory Pool IO
	// --------------------------------------------------------------------------------------------	
	Memory_Pool_Constructor( m_pEncInterface );
	Memory_Pool_Initialization( IYUVFile, bitstreamFile, m_pEncInterface ); //File open	

	// --------------------------------------------------------------------------------------------
	// encoding
	// --------------------------------------------------------------------------------------------	
	// ===================== Set the variables =====================
	_MonitorCondition = &m_pEncInterface->iNumEncoded;
	_inTestGOPs = (_InfiniteProcess) ? NUM_ITERATION : 0;	///< The number of IDRperiod iteration for infinite process
	_inTestGOPs = _inTestGOPs * _nGOPsforME; //infinite loop
	_inFrames = 0;

	m_pEncInterface->ServiceFunctionIndex[0] = true;		// TRUE: Print encoding summary	
	double dResult; long lBefore = clock();
	bool	e_bOperation = true;

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
		m_pEncInterface->nTimestamp[0] = _inFrames;	


		///< Encoding
		EncMain.EncoderMainFunc();

		///< Encoding count	
		_inFrames++;

		///< Print TS Param data	
		if( *_MonitorCondition )
		{		
			bool _dbgkey  = true; //false: no print dbgmsg, true: print dbgmsg
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
		if( _InfiniteProcess && _inFrames == _FramesTobeEncoded )
		{
			EncMain.EncoderPrintSummary();
		}			
	}

	///< PrintSummary	
	EncMain.EncoderPrintSummary();

	///< Destroy
	EncMain.EncoderDestroy();
	dResult = (double)(clock()-lBefore) / CLOCKS_PER_SEC;

	///< Print MainEncodingTime
	fprintf(stdout, "Total Encoding Time: %12.3f sec.\n", dResult);

	IYUVFile.close();
	bitstreamFile.close();

#if 0
	/// Sometimes... It is not necessary to conduct ::getch(). @ 2016 1 4 by Seok
	EDPRINTF(stderr, "The Encoder wait for your key stroke. please push ENTER key for stable finish!! ^.^\n");	
	::getchar();

#endif
	return 0;
}
