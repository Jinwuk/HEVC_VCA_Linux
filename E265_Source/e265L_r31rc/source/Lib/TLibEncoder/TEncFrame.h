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
	\file   	TEncFrame.h
   	\brief    	Frame encoder class (header)
	\author		Jinwuk Seok 
	\date		2015.05.22
*/

#ifndef __TENCFRAME__
#define __TENCFRAME__

#include <list>

#include <stdlib.h>

#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComLoopFilter.h"
#include "TLibCommon/AccessUnit.h"
#include "TEncSampleAdaptiveOffset.h"
#include "TEncSlice.h"
#include "TEncEntropy.h"
#include "TEncCavlc.h"
#include "TEncSbac.h"
#include "SEIwrite.h"

#include "TEncAnalyze.h"
#include "TEncRateCtrl.h"
#include "TEncGOP.h"

#include <vector>

#include <signal.h>

#if (_MSC_VER >= 1600)
#include <functional>
#endif


//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncGOP;
// ====================================================================================================================
// Class definition
// ====================================================================================================================
#if ETRI_MULTITHREAD_2 
// gplusplus [[
typedef struct{
	int nPOC;
	int nRefNum;
	int *pRefPOC;
} ETRI_RefPic_t;
// ]]
#endif
class TEncFrame 
{
private:
//		TComPicYuv*	em_pcPicYuvRecOut;
		TEncTop* 		em_pcEncTop;
		TEncGOP* 		em_pcGOPEncoder;					///< For Internal Configure Only Use it in the Init @ 2015 5 23 by Seok
#if !ETRI_MULTITHREAD_2 // gplusplus_151005 Not used 
		TEncSlice*		em_pcSliceEncoder;
		TEncSearch*		em_pcSearch;
#if KAIST_RC
		TEncRateCtrl*  	em_pcRateCtrl;
#endif
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD
		TComLoopFilter* em_pcLoopFilter[MAX_NUM_THREAD];
#else 
		TComLoopFilter*	em_pcLoopFilter;
#endif 
		TEncSampleAdaptiveOffset*  em_pcSAO;

		SEIWriter*		em_pcseiWriter;

		TEncEntropy*   	em_pcEntropyCoder;		
		TEncCavlc* 	  	em_pcCavlcCoder;

		TEncSbac* 	 	em_pcSbacCoder;
		TEncBinCABAC*	em_pcBinCABAC;
		TComBitCounter*	em_pcBitCounter;
		TComRdCost*  	em_pcRdCost;
		TComOutputBitstream*	em_pcBitstreamRedirect;
		Int    			em_iMTFrameIdx;
		Int				em_iNumSubstreams;
#endif

		AccessUnit*		em_pcAU;	///< 2015 5 23 by Seok ???
//		TComList<TComPicYuv*>*	rcListPicYuvRecOut;	///< 2015 5 23 by Seok : ???
	
		
		Int    		em_iGOPid;
		Int    		em_iPOCLast;
		Int    		em_iNumPicRcvd;

		Int			em_IRAPGOPid;

///----- For Frame Compression : according to the member variables in TEncGOP @ 2015 5 24 by Seok : with 2 Tabs
		Int    		em_iGopSize;
		Int    		em_iLastIDR;			///< 2015 5 24 by Seok : Int  ETRI_getiLastIDR		()	{return m_iLastIDR;}

		NalUnitType	em_associatedIRAPType;	///< Is it necessary to a member variable ??  @ 2015 5 24 by Seok
		Int    		em_associatedIRAPPOC ;	///< Is it necessary to a member variable ??  @ 2015 5 24 by Seok
		Int			em_iLastRecoveryPicPOC;

		///< ----- Clean Decoding Refresh @ 2015 5 24 by Seok
		Int    		em_pocCRA;
		Bool   		em_bRefreshPending;
		std::vector<Int>	*em_storedStartCUAddrForEncodingSlice;
		std::vector<Int>	*em_storedStartCUAddrForEncodingSliceSegment;

		Bool 		em_bLastGOP;
		Bool   		em_bisField;
		Bool   		em_bisTff;
		Bool  		em_bFirst;   				///< Some what serious. Don't get it from GOP Encoder. If you do that, Speed would be slow. @2015 5 26 by Seok		

		UInt 			em_cpbRemovalDelay;		///< Some what serious. Don't get it from GOP Encoder. If you do that, Speed would be slow. @2015 5 26 by Seok

		UInt*		accumBitsDU;
		UInt*		accumNalsDU;
		Double 		em_dEncTime;

		SEIScalableNesting em_scalableNestingSEI;

#if ETRI_DLL_INTERFACE	// 2013 10 23 by Seok
		UInt	FrameTypeInGOP;
		UInt	FramePOC;
		UInt	FrameEncodingOrder;
#endif

#if ETRI_MULTITHREAD_2
/////////////////////////////////////////////////////////////////
// gplusplus
		// Search
		TEncSearch		em_cSearch;				///< encoder search class

		//TEncCavlc*		em_pcCavlcCoder;		///< CAVLC encoder

		// coding tool
		TComTrQuant		em_cTrQuant;			///< transform & quantization class
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD_2
		TComLoopFilter	em_cLoopFilter[MAX_NUM_THREAD];
#else 
		TComLoopFilter	em_cLoopFilter;			///< deblocking filter class
#endif 
		TEncSampleAdaptiveOffset  em_cEncSAO;	///< sample adaptive offset class
		TEncEntropy   	em_cEntropyCoder;		///< entropy encoder
		TEncCavlc 	  	em_cCavlcCoder;			///< CAVLC encoder
		TEncSbac 	 	em_cSbacCoder;			///< SBAC encoder
		TEncBinCABAC	em_cBinCoderCABAC;		///< bin coder CABAC
		TEncSbac*		em_pcSbacCoders;		///< SBAC encoders (to encode substreams )	
		TEncBinCABAC*   em_pcBinCoderCABACs;	///< bin coders CABAC (one per substream)

		// processing unit
		//TEncTop* 		em_pcEncTop;
		//TEncGOP* 		em_pcGOPEncoder;		///< GOP encoder
		TEncSlice		em_cSliceEncoder;		///< slice encoder 
		TEncCu			em_cCuEncoder;			///< CU encoder

		// RD cost computation
		TComBitCounter	em_cBitCounter;			///< bit counter for RD optimization
		TComRdCost  	em_cRdCost;				///< RD cost computation class
		TEncSbac***		em_pppcRDSbacCoder;		///< temporal storage for RD computation
		TEncSbac		em_cRDGoOnSbacCoder;	///< going on SBAC model for RD stage

#if FAST_BIT_EST
		TEncBinCABACCounter***	em_pppcBinCoderCABAC;      ///< temporal CABAC state storage for RD computation
		TEncBinCABACCounter     em_cRDGoOnBinCoderCABAC;   ///< going on bin coder CABAC for RD stage
#else
		TEncBinCABAC***         em_pppcBinCoderCABAC;      ///< temporal CABAC state storage for RD computation
		TEncBinCABAC    		em_cRDGoOnBinCoderCABAC;   ///< going on bin coder CABAC for RD stage
#endif
#if KAIST_RC
		TEncRateCtrl*  	em_pcRateCtrl;
#endif
		SEIWriter		em_cseiWriter;

		TEncTile*    	em_pcTileEncoder;				///< Tile Encoder @ 2015 5 17 by Seok

//--------------------------------------------------------------------------------------------------------
//	For WPP Coders
//--------------------------------------------------------------------------------------------------------
		Int     					em_iNumSubstreams;                ///< # of top-level elements allocated.
		TComBitCounter*         	em_pcBitCounters;                 ///< bit counters for RD optimization per substream
		TComRdCost*   				em_pcRdCosts;                     ///< RD cost computation class per substream
		TEncSbac****   				em_ppppcRDSbacCoders;             ///< temporal storage for RD computation per substream
		TEncSbac*   				em_pcRDGoOnSbacCoders;            ///< going on SBAC model for RD stage per substream
		TEncBinCABAC****         	em_ppppcBinCodersCABAC;           ///< temporal CABAC state storage for RD computation per substream
		TEncBinCABAC*           	em_pcRDGoOnBinCodersCABAC;        ///< going on bin coder CABAC for RD stage per substream

		std::vector<Int>	em_cstoredStartCUAddrForEncodingSlice;
		std::vector<Int>	em_cstoredStartCUAddrForEncodingSliceSegment;

		Bool        em_activeParameterSetSEIPresentInAU;
		Bool        em_bufferingPeriodSEIPresentInAU;
		Bool        em_pictureTimingSEIPresentInAU;
		Bool        em_nestedBufferingPeriodSEIPresentInAU;
		Bool        em_nestedPictureTimingSEIPresentInAU;

#endif

public:
	TEncFrame();
	virtual ~TEncFrame();	

	// -------------------------------------------------------------------------------------------------------------------
	// Creation and Initilization 
	// -------------------------------------------------------------------------------------------------------------------
#if ETRI_MULTITHREAD_2
	Void 	create	(TEncTop* pcEncTop);
	Void 	init  	(TEncGOP* pcGOPEncoder);
	Void    ETRI_setFrameParameter(Int iGOPid,  Int pocLast, Int iNumPicRcvd, Int IRAPGOPid, int iLastIDR, UInt* uiaccumBitsDU, UInt* uiaccumNalsDU, Bool isField, Bool isTff);
#else
	Void 	create();
	Void 	init(TEncTop* pcCfg, TEncGOP* pcGOPEncoder, Int iFrameIdx);
	Void   	ETRI_setFrameParameter(Int iGOPid,  Int pocLast, Int iNumPicRcvd, Int IRAPGOPid, int iLastIDR, UInt* uiaccumBitsDU, UInt* uiaccumNalsDU, TComOutputBitstream*& pcBitstreamRedirect, Bool isField, Bool isTff);
#endif	
	
	Void 	destroy	();
	// -------------------------------------------------------------------------------------------------------------------
	// member access functions
	// -------------------------------------------------------------------------------------------------------------------

	Void 	ETRI_ResetFrametoGOPParameter();
	Double 	ETRI_getdEncTime  	 		()	{return em_dEncTime;}
	AccessUnit* ETRI_getAccessUnitFrame	()	{return em_pcAU;}
#if ETRI_DLL_INTERFACE	// 2015 06 15 by yhee
	UInt	ETRI_getFrameTypeInGOP			()	{return FrameTypeInGOP;}
	UInt	ETRI_getFramePOC				()	{return FramePOC;}
	UInt	ETRI_getFrameEncodingOrder		()	{return FrameEncodingOrder;}
#endif

	// -------------------------------------------------------------------------------------------------------------------
	// Auxiliary Compression functions (Employee in ETRI_CompressFrame)
	// -------------------------------------------------------------------------------------------------------------------
	UInt ETRI_Select_UiColDirection(Int iGOPid, UInt uiColDir); 																		
	Void ETRI_SliceDataInitialization(TComPic* pcPic, TComSlice* pcSlice);																
	Void ETRI_SetReferencePictureSetforSlice(TComPic* pcPic, TComSlice* pcSlice, Int iGOPid, Int pocCurr, Bool isField, TComList<TComPic*>& rcListPic);
	Void ETRI_refPicListModification(TComSlice* pcSlice, TComRefPicListModification* refPicListModification, TComList<TComPic*>& rcListPic, Int iGOPid, UInt& uiColDir);
	Void ETRI_NoBackPred_TMVPset(TComSlice* pcSlice, Int iGOPid);
	Void ETRI_setMvdL1ZeroFlag(TComPic* pcPic, TComSlice* pcSlice);
#if KAIST_RC
	Void ETRI_RateControlSlice(Int pocCurr, TComPic* pcPic, TComSlice* pcSlice, int iGOPid, ETRI_SliceInfo& ReturnValue);
#endif
	Void ETRI_setCUAddressinFrame(TComPic* pcPic, TComSlice* pcSlice, ETRI_SliceInfo& ReturnValue);
	Void ETRI_EvalCodingOrderMAPandInverseCOMAP(TComPic* pcPic);
	void ETRI_setStartCUAddr(TComSlice* pcSlice, ETRI_SliceInfo& ReturnValue);
	void ETRI_SetNextSlice_with_IF(TComPic* pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue);
	void ETRI_LoopFilter(TComPic* pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue);
#if ETRI_MULTITHREAD_2
	void ETRI_WriteSeqHeader(TComPic* pcPic, TComSlice*& pcSlice, AccessUnit& accessUnit, Int& actualTotalBits, Bool bFirst);
#else
	void ETRI_WriteSeqHeader(TComPic* pcPic, TComSlice*& pcSlice, AccessUnit& accessUnit, Int& actualTotalBits);
#endif
	void ETRI_WriteSOPDescriptionInSEI(Int iGOPid, Int pocCurr, TComSlice*& pcSlice, AccessUnit& accessUnit, Bool& writeSOP, Bool isField); 
	void ETRI_setPictureTimingSEI(TComSlice*& pcSlice, SEIPictureTiming& pictureTimingSEI, Int IRAPGOPid, ETRI_SliceInfo& SliceInfo);	
	void ETRI_writeHRDInfo(TComSlice*& pcSlice, AccessUnit& accessUnit, SEIScalableNesting& scalableNestingSEI);						
	void ETRI_Ready4WriteSlice(TComPic* pcPic,  Int pocCurr, TComSlice*& pcSlice, AccessUnit& accessUnit, ETRI_SliceInfo& SliceInfo);
	void ETRI_ReInitSliceData(TComPic* pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue);										
	Void ETRI_ResetSliceBoundaryData(TComPic* pcPic, TComSlice*& pcSlice, Bool& skippedSlice, Bool& bRtValue, ETRI_SliceInfo& ReturnValue);
	void ETRI_SetSliceEncoder (TComPic* pcPic, TComSlice*& pcSlice, TComOutputBitstream* pcSubstreamsOut, TComOutputBitstream*& pcBitstreamRedirect, 
										TEncSbac* pcSbacCoders,  OutputNALUnit& nalu, ETRI_SliceInfo& ReturnValue); 					
	void ETRI_WriteOutSlice(TComPic* pcPic, TComSlice*& pcSlice, TComOutputBitstream* pcSubstreamsOut, TComOutputBitstream*& pcBitstreamRedirect, 
						TEncSbac* pcSbacCoders, AccessUnit& accessUnit, OutputNALUnit& nalu, ETRI_SliceInfo& ReturnValue);				

	void ETRI_SAO_Process(TComPic* pcPic, TComSlice* pcSlice, ETRI_SliceInfo& ReturnValue); 											
	void ETRI_WriteOutSEI(TComPic* pcPic, TComSlice*& pcSlice, AccessUnit& accessUnit);
#if KAIST_RC
    void ETRI_RateControlForGOP(TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue);
#endif
	void ETRI_WriteOutHRDModel(TComSlice*& pcSlice, SEIPictureTiming& pictureTimingSEI, AccessUnit& accessUnit, ETRI_SliceInfo& ReturnValue);

	__inline Void ETRI_UpdateSliceAfterCompression(TComPic*& pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue);							

	// -------------------------------------------------------------------------------------------------------------------
	// Compression functions
	// -------------------------------------------------------------------------------------------------------------------
#if ETRI_MULTITHREAD_2
#if ETRI_THREADPOOL_OPT
	Void ETRI_CompressFrame(Int pocCurr, TComPic* pcPic, TComPicYuv* pcPicYuvRecOut, TComList<TComPic*>& rcListPic, std::list<AccessUnit>& accessUnitsInGOP, Bool bFirst, pthread_mutex_t *mutex, pthread_cond_t *cond, int *bRefPicAvailable, QphotoThreadPool *threadpool, Bool bDefault = true);
#else
	Void ETRI_CompressFrame(Int pocCurr, TComPic* pcPic, TComPicYuv* pcPicYuvRecOut, TComList<TComPic*>& rcListPic, std::list<AccessUnit>& accessUnitsInGOP, Bool bFirst, Bool bDefault = true);
#endif

#else
	Void 	ETRI_CompressFrame(Int iTimeOffset, Int pocCurr, TComPic* pcPic, TComPicYuv* pcPicYuvRecOut, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP);
#endif

// 	Void ETRI_UpdateSliceAfterCompression(TComPic*& pcPic, TComSlice*& pcSlice, ETRI_SliceInfo& ReturnValue);					
//	void ETRI_xCalculateAddPSNR(TComPic* pcPic, TComList<TComPic*>& rcListPic, AccessUnit& accessUnit, Double dEncTime, Bool isField, Bool isTff); 

#if ETRI_MULTITHREAD_2
////////////////////////////////////////////////////////////////
// gplusplus
	TEncSearch*             ETRI_getPredSearch         () { return  &em_cSearch;              }
  
	TComTrQuant*            ETRI_getTrQuant            () { return  &em_cTrQuant;             }
#if ETRI_OMP_DEBLK_FOR_MULTITHREAD_2
	TComLoopFilter          ETRI_getLoopFilter(int i) { return  em_cLoopFilter[i]; }
#else 
	TComLoopFilter*         ETRI_getLoopFilter         () { return  &em_cLoopFilter;          }
#endif 
	TEncSampleAdaptiveOffset* ETRI_getSAO              () { return  &em_cEncSAO;              }
	TEncGOP*                ETRI_getGOPEncoder         () { return  em_pcGOPEncoder;          }
	TEncSlice*              ETRI_getSliceEncoder       () { return  &em_cSliceEncoder;        }
	TEncCu*                 ETRI_getCuEncoder          () { return  &em_cCuEncoder;           }
	TEncEntropy*            ETRI_getEntropyCoder       () { return  &em_cEntropyCoder;        }
	TEncCavlc*              ETRI_getCavlcCoder         () { return  &em_cCavlcCoder;          }
	TEncSbac*               ETRI_getSbacCoder          () { return  &em_cSbacCoder;           }
	TEncBinCABAC*           ETRI_getBinCABAC           () { return  &em_cBinCoderCABAC;			  }
	TEncSbac*               ETRI_getSbacCoders     	   () { return  em_pcSbacCoders;      }
	TEncBinCABAC*           ETRI_getBinCABACs          () { return  em_pcBinCoderCABACs;      }
  
	TComBitCounter*         ETRI_getBitCounter         () { return  &em_cBitCounter;          }
	TComRdCost*             ETRI_getRdCost             () { return  &em_cRdCost;              }
	TEncSbac***             ETRI_getRDSbacCoder        () { return  em_pppcRDSbacCoder;       }
    TEncSbac*               ETRI_getRDGoOnSbacCoder    () { return  &em_cRDGoOnSbacCoder;     }
	TComBitCounter*         ETRI_getBitCounters        () { return  em_pcBitCounters;         }
	TComRdCost*             ETRI_getRdCosts            () { return  em_pcRdCosts;             }
	TEncSbac****            ETRI_getRDSbacCoders       () { return  em_ppppcRDSbacCoders;     }
	TEncSbac*               ETRI_getRDGoOnSbacCoders   () { return  em_pcRDGoOnSbacCoders;   }
#if KAIST_RC
	TEncRateCtrl*           ETRI_getRateCtrl() { return em_pcRateCtrl; }
	TEncTop* getEncTop() { return 		em_pcEncTop; }
#endif

	TEncTile*   			ETRI_getTileEncoder    	()	{return em_pcTileEncoder;  		}
	Void ETRI_createWPPCoders(Int iNumSubstreams);
	Void ETRI_xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream*& codedSliceData);
	Void ETRI_preLoopFilterPicAll( TComPic* pcPic, UInt64& ruiDist, UInt64& ruiBits );

	// SEI
	Bool 	ETRI_getactiveParameterSetSEIPresentInAU	()	{return em_activeParameterSetSEIPresentInAU;}	///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
	Bool& 	ETRI_getpictureTimingSEIPresentInAU 		()	{return em_pictureTimingSEIPresentInAU;}   		///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
	Bool&	ETRI_getbufferingPeriodSEIPresentInAU 		()	{return em_bufferingPeriodSEIPresentInAU;}  		///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok
	Bool& 	ETRI_getnestedBufferingPeriodSEIPresentInAU	()	{return	em_nestedBufferingPeriodSEIPresentInAU;}	///< For ETRI_writeHRDInfo @ 2015 5 24 by Seok

	Void ETRI_xResetNonNestedSEIPresentFlags()
	{
		em_activeParameterSetSEIPresentInAU = false;
		em_bufferingPeriodSEIPresentInAU	 = false;
		em_pictureTimingSEIPresentInAU 	 = false;
	}

	Void ETRI_xResetNestedSEIPresentFlags()
	{
		em_nestedBufferingPeriodSEIPresentInAU    = false;
		em_nestedPictureTimingSEIPresentInAU	   = false;
	}

	SEIActiveParameterSets* ETRI_xCreateSEIActiveParameterSets (TComSPS *sps);
	SEIFramePacking* ETRI_xCreateSEIFramePacking();
	SEIDisplayOrientation* ETRI_xCreateSEIDisplayOrientation();
	SEIToneMappingInfo*  ETRI_xCreateSEIToneMappingInfo();
	Void ETRI_xCreateLeadingSEIMessages (/*SEIMessages seiMessages,*/ AccessUnit &accessUnit, TComSPS *sps);
	Void ETRI_GetRefPic(ETRI_RefPic_t *pRefPic, Int pocCurr, TComPic* pcPic, TComList<TComPic*>& rcListPic, int iGOPid, bool bisField, int iPOCLast, int iNumPicRcvd, int iLastIDR);
#endif
};
//! \}
#endif
