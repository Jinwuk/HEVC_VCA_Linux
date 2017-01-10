/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TAppEncTop.h
    \brief    Encoder application class (header)
*/

#ifndef __TAPPENCTOP__
#define __TAPPENCTOP__

#include <list>
#include <ostream>

#include "TLibEncoder/TEncTop.h"
#include "TLibVideoIO/TVideoIOYuv.h"
#include "TLibCommon/AccessUnit.h"
#include "TAppEncCfg.h"
#include "DLLInterfaceType.h"

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder application class
class TAppEncTop : public TAppEncCfg
{
private:
  // class interface
  TEncTop                    m_cTEncTop;                    ///< encoder class
  TVideoIOYuv                m_cTVideoIOYuvInputFile;       ///< input YUV file
  TVideoIOYuv                m_cTVideoIOYuvReconFile;       ///< output reconstruction file
  
  TComList<TComPicYuv*>      m_cListPicYuvRec;              ///< list of reconstruction YUV files
  
  Int                        m_iFrameRcvd;                  ///< number of received frames
  
  UInt m_essentialBytes;
  UInt m_totalBytes;

#if ETRI_DLL_INTERFACE
  /* When a memory block is allocated in the outside of DLL, such as in a static Library or an application layer,
  an allocated memory block is already free when it enters DLL, and it caused an error when the free function is called. 
  Therefore, we should indicate a signal to DLL, so that it is not necessary to free a memblock allocated in an outside of DLL. */
  bool em_DLL_Apllication;		// 2013 6 14 by Seok : To slove a memory free and Allocation Problems between DLL and Static Library.
  UInt em_FrameBytes;			// 2013 10 24 by Seok : For Frame Offset	
#endif

protected:
  // initialization
  Void  xCreateLib        ();                               ///< create files & encoder class
  Void  xInitLibCfg       ();                               ///< initialize internal variables
  Void  xInitLib          (Bool isFieldCoding);             ///< initialize encoder class
  //Void  xDestroyLib       ();                               ///< destroy encoder class
  
  /// obtain required buffers
  Void  xGetBuffer(TComPicYuv*& rpcPicYuvRec);

  // file I/O 
# if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
  Void ETRI_xWriteOutput(ETRI_StreamInterface& bitstreamFile, Int iNumEncoded, AccessUnit_t* accessUnits, TComList<TComPicYuv*>* pcListPicYuvRec);
#else
  Void ETRI_xWriteOutput(ETRI_StreamInterface& bitstreamFile, Int iNumEncoded, AccessUnit_t* accessUnits);
#endif
#endif
  Void  xWriteOutput(ETRI_StreamInterface& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits); ///< write bitstream to file
  void  rateStatsAccum(const AccessUnit& au, const std::vector<UInt>& stats);
#if !ETRI_DLL_INTERFACE
  Void  xDestroyLib       ();      
  Void  xDeleteBuffer     ();
  void  printRateSummary	  ();
#endif

#if ETRI_DLL_INTERFACE

# if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)  
  TComList<TComPic*> em_cListPic;  
  TComList<TComPicYuv*> em_cListPicYuvRec;  
#endif
  AccessUnit_t *em_poutputAccessUnits;
#else
  list<AccessUnit> em_outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process	
#endif //ETRI_MULTITHREAD_2

#endif //ETRI_DLL_INTERFACE

public:
  TAppEncTop();
  virtual ~TAppEncTop();

#if ETRI_DLL_INTERFACE
	InterfaceInfo	 e_ETRIInterface; 
	// chang HM function to public
	void  printRateSummary	();
	Void  xDeleteBuffer     ();
	Void  xDestroyLib       ();    
# if ETRI_MULTITHREAD_2
	// ETRI_xSetBuffer(TComList<TComPicYuv*>* pcListPicYuvRec, int nCount);
	//Void ETRI_xDeleteBuffer(TComList<TComPicYuv*>* pcListPicYuvRec);
	Void ETRI_xDeleteAU();
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)  
	Void ETRI_xDeletePicBuffers();
#endif
# endif
	// chang HM function 
	Void  encode						(InterfaceInfo& eETRIInterface);  ///< main encoding function using DLLInterface	
	// ETRI function 
	Void  ETRI_DLLEncoderInitialize	(InterfaceInfo& eETRIInterface);
	//Void  ETRI_Free					(InterfaceInfo& eETRIInterface);
	Void  ETRI_FstreamInterface		(InterfaceInfo& eETRIInterface)
	{
		eETRIInterface.m_pcHandle	= m_cTVideoIOYuvInputFile.GetptrETRIfstream(true);	//Input YUV	
		eETRIInterface.FStream		= m_cTVideoIOYuvInputFile.GetptrETRIfstream(false);	//Output Bitstream
		eETRIInterface.m_pcRecStr	= m_cTVideoIOYuvReconFile.GetptrETRIfstream(true);	//Input YUV	
	 };	
#else
	Void  encode      ();                               ///< main encoding function
#endif
  TEncTop& getTEncTop  ()   { return  m_cTEncTop; }      ///< return encoder class pointer reference

  //////////////////////////////////////////////////////////////////////  
# if ETRI_MULTITHREAD_2
  Void ETRI_xSetBuffer(TComList<TComPicYuv*>* pcListPicYuvRec, int nCount);
  Void ETRI_xDeleteBuffer(TComList<TComPicYuv*>* pcListPicYuvRec);
//#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
//  Void ETRI_xWriteOutput(std::ostream& bitstreamFile, Int iNumEncoded, AccessUnit_t* accessUnits, TComList<TComPicYuv*>* pcListPicYuvRec);
//#else
//  Void ETRI_xWriteOutput(std::ostream& bitstreamFile, Int iNumEncoded, AccessUnit_t* accessUnits);
//#endif
# endif

};// END CLASS DEFINITION TAppEncTop

//! \}

#endif // __TAPPENCTOP__

