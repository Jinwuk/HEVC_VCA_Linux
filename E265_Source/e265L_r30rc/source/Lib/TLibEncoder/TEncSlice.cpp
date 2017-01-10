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

/** \file     TEncSlice.cpp
	\brief    slice encoder class
*/

#include "TEncTop.h"
#include "TEncSlice.h"
#include <math.h>

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

#if ETRI_THREADPOOL_OPT
pthread_mutex_t g_jobMutex = PTHREAD_MUTEX_INITIALIZER;
extern QphotoThreadPool *gop_QphotoPool;
#endif


TEncSlice::TEncSlice()
{
	m_apcPicYuvPred = NULL;
	m_apcPicYuvResi = NULL;
  
	m_pdRdPicLambda = NULL;
	m_pdRdPicQp     = NULL;
	m_piRdPicQp     = NULL;
	m_pcBufferSbacCoders    = NULL;
	m_pcBufferBinCoderCABACs  = NULL;
	m_pcBufferLowLatSbacCoders    = NULL;
	m_pcBufferLowLatBinCoderCABACs  = NULL;
}

TEncSlice::~TEncSlice()
{
	for (std::vector<TEncSbac*>::iterator i = CTXMem.begin(); i != CTXMem.end(); i++)
	{
		delete (*i);
	}
}

Void TEncSlice::initCtxMem(  UInt i )                
{   
	for (std::vector<TEncSbac*>::iterator j = CTXMem.begin(); j != CTXMem.end(); j++)
	{
		delete (*j);
	}
	CTXMem.clear(); 
	CTXMem.resize(i); 
}

Void TEncSlice::create( Int iWidth, Int iHeight, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth )
{
	// create prediction picture
	if ( m_apcPicYuvPred == NULL )
	{
		m_apcPicYuvPred  = new TComPicYuv;
		m_apcPicYuvPred->create( iWidth, iHeight, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
	}
  
	// create residual picture
	if( m_apcPicYuvResi == NULL )
	{
		m_apcPicYuvResi  = new TComPicYuv;
		m_apcPicYuvResi->create( iWidth, iHeight, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
	}
#if ETRI_MULTITHREAD_2
	em_hThreadPoolTile.Create(threadProcessingTile, MAX_THREAD_TILE);
#endif
}

Void TEncSlice::destroy()
{
#if ETRI_MULTITHREAD_2
	em_hThreadPoolTile.FinishThread();
#endif

	// destroy prediction picture
	if ( m_apcPicYuvPred )
	{
		m_apcPicYuvPred->destroy();
		delete m_apcPicYuvPred;
		m_apcPicYuvPred  = NULL;
	}
  
	// destroy residual picture
	if ( m_apcPicYuvResi )
	{
		m_apcPicYuvResi->destroy();
		delete m_apcPicYuvResi;
		m_apcPicYuvResi  = NULL;
	}
  
	// free lambda and QP arrays
	if ( m_pdRdPicLambda ) { xFree( m_pdRdPicLambda ); m_pdRdPicLambda = NULL; }
	if ( m_pdRdPicQp     ) { xFree( m_pdRdPicQp     ); m_pdRdPicQp     = NULL; }
	if ( m_piRdPicQp     ) { xFree( m_piRdPicQp     ); m_piRdPicQp     = NULL; }

	if ( m_pcBufferSbacCoders )
	{
		delete[] m_pcBufferSbacCoders;
	}
	if ( m_pcBufferBinCoderCABACs )
	{
		delete[] m_pcBufferBinCoderCABACs;
	}
	if ( m_pcBufferLowLatSbacCoders )
		delete[] m_pcBufferLowLatSbacCoders;
	if ( m_pcBufferLowLatBinCoderCABACs )
		delete[] m_pcBufferLowLatBinCoderCABACs;

#if !(_ETRI_WINDOWS_APPLICATION)  
	if(&em_TileSemaphore)
		sem_destroy(&em_TileSemaphore);
#endif
}

#if ETRI_MULTITHREAD_2
Void TEncSlice::init( TEncTop* pcEncTop, TEncFrame* pcEncFrame )
{
	m_pcCfg             = pcEncTop;
	m_pcListPic         = pcEncTop->getListPic();
  
	m_pcGOPEncoder      = pcEncTop->getGOPEncoder();
	em_pcEncFrame		  = pcEncFrame;	

	m_pcCuEncoder       = pcEncFrame->ETRI_getCuEncoder();
	m_pcPredSearch      = pcEncFrame->ETRI_getPredSearch();
  
	m_pcEntropyCoder    = pcEncFrame->ETRI_getEntropyCoder();
	m_pcCavlcCoder      = pcEncFrame->ETRI_getCavlcCoder();
	m_pcSbacCoder       = pcEncFrame->ETRI_getSbacCoder();
	m_pcBinCABAC        = pcEncFrame->ETRI_getBinCABAC();
	m_pcTrQuant         = pcEncFrame->ETRI_getTrQuant();
  
	m_pcBitCounter      = pcEncFrame->ETRI_getBitCounter();
	m_pcRdCost          = pcEncFrame->ETRI_getRdCost();
	m_pppcRDSbacCoder   = pcEncFrame->ETRI_getRDSbacCoder();
	m_pcRDGoOnSbacCoder = pcEncFrame->ETRI_getRDGoOnSbacCoder();
  
	// create lambda and QP arrays
	m_pdRdPicLambda     = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
	m_pdRdPicQp         = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
	m_piRdPicQp         = (Int*   )xMalloc( Int,    m_pcCfg->getDeltaQpRD() * 2 + 1 );
#if KAIST_RC
	m_pcRateCtrl = pcEncFrame->ETRI_getRateCtrl();
#endif

	//ETRI Code :  2015 5 18 by Seok	
	em_pcTileEncoder = pcEncFrame->ETRI_getTileEncoder();	///Get Tile Encoder @  2015 5 18 by Seok
	em_uiTileIdx		= 0;

#if 0 //ETRI_E265_PH01
	EDPRINTF(stderr, " Compiled @%s  [%s] \n\n", __DATE__, __TIME__);
#endif


	//ETRI Code :  2015 5 18 by Seok	
	em_uiTileIdx		= 0;

#if 0 //ETRI_E265_PH01
	EDPRINTF(stderr, " Compiled @%s  [%s] \n\n", __DATE__, __TIME__);
#endif
#if ETRI_THREADPOOL_OPT
#else
	em_hThreadPoolTile.Init();
#endif
	
}

#else
Void TEncSlice::init( TEncTop* pcEncTop )
{
	m_pcCfg             = pcEncTop;
	m_pcListPic         = pcEncTop->getListPic();
  
	m_pcGOPEncoder      = pcEncTop->getGOPEncoder();
	m_pcCuEncoder       = pcEncTop->getCuEncoder();
	m_pcPredSearch      = pcEncTop->getPredSearch();
  
	m_pcEntropyCoder    = pcEncTop->getEntropyCoder();
	m_pcCavlcCoder      = pcEncTop->getCavlcCoder();
	m_pcSbacCoder       = pcEncTop->getSbacCoder();
	m_pcBinCABAC        = pcEncTop->getBinCABAC();
	m_pcTrQuant         = pcEncTop->getTrQuant();
  
	m_pcBitCounter      = pcEncTop->getBitCounter();
	m_pcRdCost          = pcEncTop->getRdCost();
	m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
	m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();
  
	// create lambda and QP arrays
	m_pdRdPicLambda     = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
	m_pdRdPicQp         = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
	m_piRdPicQp         = (Int*   )xMalloc( Int,    m_pcCfg->getDeltaQpRD() * 2 + 1 );
#if KAIST_RC
	m_pcRateCtrl        = pcEncTop->getRateCtrl();
#endif

	//ETRI Code :  2015 5 18 by Seok	
	em_pcTileEncoder = pcEncTop->ETRI_getTileEncoder();	///Get Tile Encoder @  2015 5 18 by Seok
	em_uiTileIdx		= 0;


	//EDPRINTF(stderr, " Compiled @%s  [%s] \n\n", __DATE__, __TIME__);


}
#endif


/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param iNumPicRcvd   number of received pictures
 \param iTimeOffset   POC offset for hierarchical structure
 \param iDepth        temporal layer depth
 \param rpcSlice      slice header class
 \param pSPS          SPS associated with the slice
 \param pPPS          PPS associated with the slice
 */

Void TEncSlice::initEncSlice( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, bool isField )
{
	Double dQP;
	Double dLambda;
  
	rpcSlice = pcPic->getSlice(0);
	rpcSlice->setSPS( pSPS );
	rpcSlice->setPPS( pPPS );
	rpcSlice->setSliceBits(0);
	rpcSlice->setPic( pcPic );
	rpcSlice->initSlice();
	rpcSlice->setPicOutputFlag( true );
	rpcSlice->setPOC( pocCurr );
#if  ETRI_SliceEncoderHeader
#if ETRI_DLL_INTERFACE
  short sSliceIndex  = m_pcCfg->ETRI_getETRI_SliceIndex();
#else
  short sSliceIndex = ETRI_Header_SliceIdx;
#endif
  rpcSlice->ETRI_setSliceIndex(sSliceIndex);
#endif
  
	// depth computation based on GOP size
	Int depth;
	{
#if FIX_FIELD_DEPTH    
		Int poc = rpcSlice->getPOC();
		if(isField)
		{
			poc = (poc/2)%(m_pcCfg->getGOPSize()/2);
		}
		else
		{
			poc = poc%m_pcCfg->getGOPSize();   
		}
#else
		Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif
		if ( poc == 0 )
		{
			depth = 0;
		}
		else
		{
			Int step = m_pcCfg->getGOPSize();
			depth    = 0;
			for( Int i=step>>1; i>=1; i>>=1 )
			{
				for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
				{
					if ( j == poc )
					{
						i=0;
						break;
					}
				}
				step >>= 1;
				depth++;
			}
		}
#if FIX_FIELD_DEPTH  
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
		if(poc != 0)
		{
#endif
			if(isField && rpcSlice->getPOC()%2 == 1)
			{
				depth ++;
			}
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
		}
#endif
#endif
	}
  
	// slice type
	SliceType eSliceType;
  
	eSliceType=B_SLICE;
#if EFFICIENT_FIELD_IRAP
	if(!(isField && pocLast == 1))
	{
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
		if(m_pcCfg->getDecodingRefreshType() == 3)
		{
			eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
		}
		else
		{
			eSliceType = (pocLast == 0 || (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
		}
#else
		eSliceType = (pocLast == 0 || (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#endif
#if EFFICIENT_FIELD_IRAP
	}
#endif
  
	rpcSlice->setSliceType    ( eSliceType );
  
	// ------------------------------------------------------------------------------------------------------------------
	// Non-referenced frame marking
	// ------------------------------------------------------------------------------------------------------------------
  
	if(pocLast == 0)
	{
		rpcSlice->setTemporalLayerNonReferenceFlag(false);
	}
	else
	{
		rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
	}
	rpcSlice->setReferenced(true);
  
	// ------------------------------------------------------------------------------------------------------------------
	// QP setting
	// ------------------------------------------------------------------------------------------------------------------
  
	dQP = m_pcCfg->getQP();
	if(eSliceType!=I_SLICE)
	{
		if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffsetY() ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
		{
			dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
		}
	}
  
	// modify QP
	Int* pdQPs = m_pcCfg->getdQPs();
	if ( pdQPs )
	{
		dQP += pdQPs[ rpcSlice->getPOC() ];
	}
	// ------------------------------------------------------------------------------------------------------------------
	// Lambda computation
	// ------------------------------------------------------------------------------------------------------------------
  
	Int iQP;
	Double dOrigQP = dQP;

	// pre-compute lambda and QP values for all possible QP candidates
	for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
	{
		// compute QP value
		dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
    
		// compute lambda value
		Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
		Int    SHIFT_QP = 12;

		Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );

#if FULL_NBIT //CHECKME to V0 by yhee 20151028
		Int    bitdepth_luma_qp_scale = 6 * (g_bitDepthY - 8);
#else
		Int    bitdepth_luma_qp_scale = 0;
#endif
		Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
		Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
		// Case #1: I or P-slices (key-frame)
		Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
		if ( eSliceType==I_SLICE )
		{
			dQPFactor=0.57*dLambda_scale;
		}
		dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

		if ( depth>0 )
		{
#if FULL_NBIT
			dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
			dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
		}
    
		// if hadamard is used in ME process
		if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
		{
			dLambda *= 0.95;
		}
    
		iQP = max( -pSPS->getQpBDOffsetY(), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );

		m_pdRdPicLambda[iDQpIdx] = dLambda;
		m_pdRdPicQp    [iDQpIdx] = dQP;
		m_piRdPicQp    [iDQpIdx] = iQP;
	}
  
	// obtain dQP = 0 case
	dLambda = m_pdRdPicLambda[0];
	dQP     = m_pdRdPicQp    [0];
	iQP     = m_piRdPicQp    [0];
  
	if( rpcSlice->getSliceType( ) != I_SLICE )
	{
		dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
	}

	// store lambda
	m_pcRdCost ->setLambda( dLambda );
// for RDO
	// in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
	Double weight[2] = { 1.0, 1.0 };
	Int qpc;
	Int chromaQPOffset;

	chromaQPOffset = rpcSlice->getPPS()->getChromaCbQpOffset() + rpcSlice->getSliceQpDeltaCb();
	qpc = Clip3( 0, 57, iQP + chromaQPOffset);
	weight[0] = pow( 2.0, (iQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
	m_pcRdCost->setCbDistortionWeight(weight[0]);

	chromaQPOffset = rpcSlice->getPPS()->getChromaCrQpOffset() + rpcSlice->getSliceQpDeltaCr();
	qpc = Clip3( 0, 57, iQP + chromaQPOffset);
	weight[1] = pow( 2.0, (iQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
	m_pcRdCost->setCrDistortionWeight(weight[1]);

	const Double lambdaArray[3] = {dLambda, (dLambda / weight[0]), (dLambda / weight[1])};
  
#if RDOQ_CHROMA_LAMBDA 
// for RDOQ
	m_pcTrQuant->setLambdas( lambdaArray );
#else
	m_pcTrQuant->setLambda( dLambda );
#endif

// For SAO
	rpcSlice->setLambdas( lambdaArray );
  
#if HB_LAMBDA_FOR_LDC
	// restore original slice type
  
#if EFFICIENT_FIELD_IRAP
	if(!(isField && pocLast == 1))
	{
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
		if(m_pcCfg->getDecodingRefreshType() == 3)
		{
			eSliceType = (pocLast == 0 || (pocCurr)           % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;

		}
		else
		{
			eSliceType = (pocLast == 0 || (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
		}
#else
		eSliceType = (pocLast == 0 || (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#endif
#if EFFICIENT_FIELD_IRAP
	}
#endif // EFFICIENT_FIELD_IRAP
  
	rpcSlice->setSliceType        ( eSliceType );
#endif
  
	if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
	{
		dQP = xGetQPValueAccordingToLambda( dLambda );
		iQP = max( -pSPS->getQpBDOffsetY(), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
	}

	rpcSlice->setSliceQp          ( iQP );
#if ADAPTIVE_QP_SELECTION
	rpcSlice->setSliceQpBase      ( iQP );
#endif
	rpcSlice->setSliceQpDelta     ( 0 );
	rpcSlice->setSliceQpDeltaCb   ( 0 );
	rpcSlice->setSliceQpDeltaCr   ( 0 );
	rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
	rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
  
	if ( m_pcCfg->getDeblockingFilterMetric() )
	{
		rpcSlice->setDeblockingFilterOverrideFlag(true);
		rpcSlice->setDeblockingFilterDisable(false);
		rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
		rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
	} else
		if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
		{
			rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
			rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
			rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
			rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
			if ( !rpcSlice->getDeblockingFilterDisable())
			{
				if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
				{
					rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
					rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
					rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
					rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
				}
				else
				{
					rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
					rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
					rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
					rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
				}
			}
		}
		else
		{
			rpcSlice->setDeblockingFilterOverrideFlag( false );
			rpcSlice->setDeblockingFilterDisable( false );
			rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
			rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
		}

		rpcSlice->setDepth            ( depth );
  
		pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
		if(eSliceType==I_SLICE)
		{
			pcPic->setTLayer(0);
		}
		rpcSlice->setTLayer( pcPic->getTLayer() );

		assert( m_apcPicYuvPred );
		assert( m_apcPicYuvResi );
  
		pcPic->setPicYuvPred( m_apcPicYuvPred );
		pcPic->setPicYuvResi( m_apcPicYuvResi );
		rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
		rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
		rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
		rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
		rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
		xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
}

Void TEncSlice::resetQP( TComPic* pic, Int sliceQP, Double lambda )
{
	TComSlice* slice = pic->getSlice(0);

	// store lambda
	slice->setSliceQp( sliceQP );
#if ADAPTIVE_QP_SELECTION
	slice->setSliceQpBase ( sliceQP );
#endif
	m_pcRdCost ->setLambda( lambda );
	// for RDO
	// in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
	Double weight[2] = { 1.0, 1.0 };
	Int qpc;
	Int chromaQPOffset;

	chromaQPOffset = slice->getPPS()->getChromaCbQpOffset() + slice->getSliceQpDeltaCb();
	qpc = Clip3( 0, 57, sliceQP + chromaQPOffset);
	weight[0] = pow( 2.0, (sliceQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
	m_pcRdCost->setCbDistortionWeight(weight[0]);

	chromaQPOffset = slice->getPPS()->getChromaCrQpOffset() + slice->getSliceQpDeltaCr();
	qpc = Clip3( 0, 57, sliceQP + chromaQPOffset);
	weight[1] = pow( 2.0, (sliceQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
	m_pcRdCost->setCrDistortionWeight(weight[1]);

	const Double lambdaArray[3] = {lambda, (lambda / weight[0]), (lambda / weight[1])};
  
#if RDOQ_CHROMA_LAMBDA 
	// for RDOQ
	m_pcTrQuant->setLambdas( lambdaArray );
#else
	m_pcTrQuant->setLambda( lambda );
#endif

	// For SAO
	slice->setLambdas( lambdaArray );
}
// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncSlice::setSearchRange( TComSlice* pcSlice )
{
	Int iCurrPOC = pcSlice->getPOC();
	Int iRefPOC;
	Int iGOPSize = m_pcCfg->getGOPSize();
	Int iOffset = (iGOPSize >> 1);
	Int iMaxSR = m_pcCfg->getSearchRange();
	Int iNumPredDir = pcSlice->isInterP() ? 1 : 2;
 
	for (Int iDir = 0; iDir <= iNumPredDir; iDir++)
	{
		//RefPicList e = (RefPicList)iDir;
		RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
		for (Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
		{
			iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
			Int iNewSR = Clip3(8, iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
			m_pcPredSearch->setAdaptiveSearchRange(iDir, iRefIdx, iNewSR);
		}
	}
}

/**
 - multi-loop slice encoding for different slice QP
 .
 \param rpcPic    picture class
 */
Void TEncSlice::precompressSlice( TComPic*& rpcPic )
{
	// if deltaQP RD is not used, simply return
	if ( m_pcCfg->getDeltaQpRD() == 0 )
	{
		return;
	}

	if ( m_pcCfg->getUseRateCtrl() )
	{
		printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
		assert(0);
	}
  
	TComSlice* pcSlice        = rpcPic->getSlice(getSliceIdx());
	Double     dPicRdCostBest = MAX_DOUBLE;
	UInt       uiQpIdxBest = 0;
  
	Double dFrameLambda;
#if FULL_NBIT
	Int    SHIFT_QP = 12 + 6 * (g_bitDepthY - 8);
#else
	Int    SHIFT_QP = 12;
#endif
  
	// set frame lambda
	if (m_pcCfg->getGOPSize() > 1)
	{
		dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
	}
	else
	{
		dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
	}
	m_pcRdCost      ->setFrameLambda(dFrameLambda);
  
	// for each QP candidate
	for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
	{
		pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );
#if ADAPTIVE_QP_SELECTION
		pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
#endif
		m_pcRdCost    ->setLambda              ( m_pdRdPicLambda[uiQpIdx] );
		// for RDO
		// in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
		Int iQP = m_piRdPicQp    [uiQpIdx];
		Double weight[2] = { 1.0, 1.0 };
		Int qpc;
		Int chromaQPOffset;

		chromaQPOffset = pcSlice->getPPS()->getChromaCbQpOffset() + pcSlice->getSliceQpDeltaCb();
		qpc = Clip3( 0, 57, iQP + chromaQPOffset);
		weight[0] = pow( 2.0, (iQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
		m_pcRdCost->setCbDistortionWeight(weight[0]);

		chromaQPOffset = pcSlice->getPPS()->getChromaCrQpOffset() + pcSlice->getSliceQpDeltaCr();
		qpc = Clip3( 0, 57, iQP + chromaQPOffset);
		weight[1] = pow( 2.0, (iQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
		m_pcRdCost->setCrDistortionWeight(weight[1]);

		const Double lambdaArray[3] = {m_pdRdPicLambda[uiQpIdx], (m_pdRdPicLambda[uiQpIdx] / weight[0]), (m_pdRdPicLambda[uiQpIdx] / weight[1])};
#if RDOQ_CHROMA_LAMBDA 
		// for RDOQ
		m_pcTrQuant->setLambdas( lambdaArray );
#else
		m_pcTrQuant   ->setLambda              ( m_pdRdPicLambda[uiQpIdx] );
#endif
		// For SAO
		pcSlice->setLambdas( lambdaArray );
    
		// try compress
		compressSlice   ( rpcPic );
    
		Double dPicRdCost;
		UInt64 uiPicDist        = m_uiPicDist;
		UInt64 uiALFBits        = 0;
    
#if ETRI_MULTITHREAD_2
		em_pcEncFrame->ETRI_preLoopFilterPicAll( rpcPic, uiPicDist, uiALFBits );
#else
		m_pcGOPEncoder->preLoopFilterPicAll( rpcPic, uiPicDist, uiALFBits );
#endif
    
		// compute RD cost and choose the best
		dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits + uiALFBits, uiPicDist, true, DF_SSE_FRAME);
    
		if ( dPicRdCost < dPicRdCostBest )
		{
			uiQpIdxBest    = uiQpIdx;
			dPicRdCostBest = dPicRdCost;
		}
	}
  
	// set best values
	pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );
#if ADAPTIVE_QP_SELECTION
	pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
#endif
	m_pcRdCost    ->setLambda              ( m_pdRdPicLambda[uiQpIdxBest] );
	// in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
	Int iQP = m_piRdPicQp    [uiQpIdxBest];
	Double weight[2] = { 1.0, 1.0 };
	Int qpc;
	Int chromaQPOffset;

	chromaQPOffset = pcSlice->getPPS()->getChromaCbQpOffset() + pcSlice->getSliceQpDeltaCb();
	qpc = Clip3( 0, 57, iQP + chromaQPOffset);
	weight[0] = pow( 2.0, (iQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
	m_pcRdCost->setCbDistortionWeight(weight[0]);

	chromaQPOffset = pcSlice->getPPS()->getChromaCrQpOffset() + pcSlice->getSliceQpDeltaCr();
	qpc = Clip3( 0, 57, iQP + chromaQPOffset);
	weight[1] = pow( 2.0, (iQP-g_aucChromaScale[qpc])/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
	m_pcRdCost->setCrDistortionWeight(weight[1]);

	const Double lambdaArray[3] = {m_pdRdPicLambda[uiQpIdxBest], (m_pdRdPicLambda[uiQpIdxBest] / weight[0]), (m_pdRdPicLambda[uiQpIdxBest] / weight[1])};
#if RDOQ_CHROMA_LAMBDA 
	// for RDOQ 
	m_pcTrQuant->setLambdas( lambdaArray );
#else
	m_pcTrQuant   ->setLambda              ( m_pdRdPicLambda[uiQpIdxBest] );
#endif
	// For SAO
	pcSlice->setLambdas( lambdaArray );
}

/** \param rpcPic   picture class
 */
Void TEncSlice::calCostSliceI(TComPic*& rpcPic)
{
	UInt    uiCUAddr;
	UInt    uiStartCUAddr;
	UInt    uiBoundingCUAddr;
	Int     iSumHad, shift = g_bitDepthY-8, offset = (shift>0)?(1<<(shift-1)):0;;
	Double  iSumHadSlice = 0;

	rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
	TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
	xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );

#if KAIST_RC
	Int intra_size = m_pcCfg->getIntraPeriod();
	Int iIDRIndex = rpcPic->getPOC() / intra_size;
	Int iIDRModulus = rpcPic->getPOC() % intra_size;
	TRCPic *tRCPic = m_pcRateCtrl->getTRCPic(iIDRModulus);
#endif

	UInt uiEncCUOrder;
	uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU()); 
	for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
		uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
		uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
	{
		// initialize CU encoder
		TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
		pcCU->initCU( rpcPic, uiCUAddr );

		Int height  = min( pcSlice->getSPS()->getMaxCUHeight(),pcSlice->getSPS()->getPicHeightInLumaSamples() - uiCUAddr / rpcPic->getFrameWidthInCU() * pcSlice->getSPS()->getMaxCUHeight() );
		Int width   = min( pcSlice->getSPS()->getMaxCUWidth(),pcSlice->getSPS()->getPicWidthInLumaSamples() - uiCUAddr % rpcPic->getFrameWidthInCU() * pcSlice->getSPS()->getMaxCUWidth() );

		iSumHad = m_pcCuEncoder->updateLCUDataISlice(pcCU, uiCUAddr, width, height);

#if KAIST_RC
		(tRCPic->getLCU(uiCUAddr)).m_costIntra = (iSumHad + offset) >> shift;
		iSumHadSlice += (tRCPic->getLCU(uiCUAddr)).m_costIntra;
#endif
	}
#if KAIST_RC
	tRCPic->setTotalIntraCost(iSumHadSlice);
#endif
}

#if ETRI_MODIFICATION_V00
// ====================================================================================================================
// ETRI Compress Slice Functions @ 2015 5 11 by Seok
// ====================================================================================================================
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initilization of Weighted Prediction : However, when weighted prediction is not active by the config,  this function is not operated by if .
			Including Adaptive QP Selection for Slice Compression.
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncSlice::ETRI_InitWeightedPrediction(TComSlice* pcSlice)	
{
	//------------------------------------------------------------------------------
	//  Weighted Prediction parameters estimation.
	//------------------------------------------------------------------------------
	// calculate AC/DC values for current picture
	if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
	{
		xCalcACDCParamSlice(pcSlice);
	}

	Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

	if ( bWp_explicit )
	{
		//------------------------------------------------------------------------------
		//  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
		//------------------------------------------------------------------------------
		if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
		{
			printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
		}

		xEstimateWPParamSlice( pcSlice );
		pcSlice->initWpScaling();

		// check WP on/off
		xCheckWPEnable( pcSlice );
	}

#if ADAPTIVE_QP_SELECTION
	if( m_pcCfg->getUseAdaptQpSelect() )
	{
		m_pcTrQuant->clearSliceARLCnt();
		if(pcSlice->getSliceType()!=I_SLICE)
		{
			Int qpBase = pcSlice->getSliceQpBase();
			pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
		}
	}
#endif

}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Get SABACRD coder. However it can be optimized almost codes using ETRI_Remove_BufferSbacCoders
	@return :  iNumSubstreams
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncSlice::ETRI_GetUseSABACRD(TEncSbac****& ppppcRDSbacCoders, CI_IDX  e_CI_IDX, TComSlice* pcSlice, TComPic*& rpcPic, Int& iNumSubstreams)	//Return Value :  iNumSubstreams
{

	UInt uiTilesAcross  = 0;

	iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
	uiTilesAcross = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
	delete[] m_pcBufferSbacCoders;
	delete[] m_pcBufferBinCoderCABACs;
	m_pcBufferSbacCoders	   = new TEncSbac	 [uiTilesAcross];
	m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];

	for (Int ui = 0; ui < uiTilesAcross; ui++)
		m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );

	for (UInt ui = 0; ui < uiTilesAcross; ui++)
		m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][e_CI_IDX]);	//init. state

	for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
		ppppcRDSbacCoders[ui][0][e_CI_IDX]->load(m_pppcRDSbacCoder[0][e_CI_IDX]);


	delete[] m_pcBufferLowLatSbacCoders;
	delete[] m_pcBufferLowLatBinCoderCABACs;
	m_pcBufferLowLatSbacCoders	 = new TEncSbac    [uiTilesAcross];
	m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];

	for (Int ui = 0; ui < uiTilesAcross; ui++)
		m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );

	for (UInt ui = 0; ui < uiTilesAcross; ui++)
		m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][e_CI_IDX]);  //init. state


}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initialization of Dependent Slices. However, since there are a a lot of Information of Tile in HM, we have to analyse this function. 
			Moreover, it can be optimized by odes using ETRI_Remove_BufferSbacCoders
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncSlice::ETRI_InitDependentSlices(TComPic*& rpcPic, TComSlice* pcSlice,  TEncSbac**** ppppcRDSbacCoders, ETRI_InfoofCU& pETRI_InfoofCU)
{

	if( *pETRI_InfoofCU.depSliceSegmentsEnabled )
	{
		UInt*	uiCUAddr	= pETRI_InfoofCU.uiCUAddr;
		UInt*	uiTileCol  	= pETRI_InfoofCU.uiTileCol;
		UInt*	uiTileStartLCU= pETRI_InfoofCU.uiTileStartLCU;
		UInt*	uiWidthInLCUs = pETRI_InfoofCU.uiWidthInLCUs;
		UInt*	uiLin		= pETRI_InfoofCU.uiLin;
		UInt*	uiSubStrm	= pETRI_InfoofCU.uiSubStrm;
		UInt*	uiCol		= pETRI_InfoofCU.uiCol;
		UInt* 	uiStartCUAddr= pETRI_InfoofCU.uiStartCUAddr;
		UInt* 	uiTileLCUX	= pETRI_InfoofCU.uiTileLCUX;
		
		Int*  	iNumSubstreams	=pETRI_InfoofCU.iNumSubstreams;
		
		if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(*uiCUAddr != *uiTileStartLCU))
		{
			if( m_pcCfg->getWaveFrontsynchro() )
			{
				*uiTileCol = rpcPic->getPicSym()->getTileIdxMap(*uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
				m_pcBufferSbacCoders[*uiTileCol].loadContexts( CTXMem[1] );
				Int iNumSubstreamsPerTile = *iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
				*uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( *uiStartCUAddr /rpcPic->getNumPartInCU()); 
				*uiLin 	= *uiCUAddr / *uiWidthInLCUs;
				*uiSubStrm = rpcPic->getPicSym()->getTileIdxMap(rpcPic->getPicSym()->getCUOrderMap(*uiCUAddr)) * iNumSubstreamsPerTile + *uiLin % iNumSubstreamsPerTile;
				if ( (*uiCUAddr % ((*uiWidthInLCUs) + 1)) >= *uiWidthInLCUs	)
				{
					*uiTileLCUX = *uiTileStartLCU % *uiWidthInLCUs;
					*uiCol	  = *uiCUAddr % *uiWidthInLCUs;
					if(*uiCol==*uiTileStartLCU)
						CTXMem[0]->loadContexts(m_pcSbacCoder);
				}
			}
			m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
			ppppcRDSbacCoders[*uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
		}
		else
		{
			if(m_pcCfg->getWaveFrontsynchro()){
				CTXMem[1]->loadContexts(m_pcSbacCoder);}
			CTXMem[0]->loadContexts(m_pcSbacCoder);
		}
	}


}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initialization of RD Coders for SubStream.  Set ppppcRDSbacCoders & m_pppcRDSbacCoder[0][CI_CURR_BEST]
			For Two Types of Substream such as Tile and WPP. Remember : 
	 		Int iNumSubstreamsPerTile = iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
			// iNumSubstreams has already been multiplied. 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncSlice::ETRI_InitRDCoderForSubStream(TEncSbac****& ppppcRDSbacCoders, TComPic*& rpcPic, TComDataCU*& pcCU, TComSlice* pcSlice, TComBitCounter* pcBitCounters, ETRI_InfoofCU& pETRI_InfoofCU)
{

	UInt*	uiCUAddr	= pETRI_InfoofCU.uiCUAddr;	
	UInt*	uiTileCol		= pETRI_InfoofCU.uiTileCol;
	UInt*	uiTileStartLCU = pETRI_InfoofCU.uiTileStartLCU;
	UInt*	uiWidthInLCUs = pETRI_InfoofCU.uiWidthInLCUs;
	UInt*	uiLin		= pETRI_InfoofCU.uiLin;
	UInt*	uiSubStrm	= pETRI_InfoofCU.uiSubStrm;
	UInt*	uiCol		= pETRI_InfoofCU.uiCol;
	UInt*	uiTileLCUX	= pETRI_InfoofCU.uiTileLCUX;

	// inherit from TR if necessary, select substream to use.
	*uiTileCol = rpcPic->getPicSym()->getTileIdxMap(*uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
	*uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(*uiCUAddr))->getFirstCUAddr();
	*uiTileLCUX = *uiTileStartLCU % *uiWidthInLCUs;
	//UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
	*uiCol	  = *uiCUAddr % *uiWidthInLCUs;
	*uiLin	  = *uiCUAddr / *uiWidthInLCUs;
	if (pcSlice->getPPS()->getNumSubstreams() > 1)
	{	// independent tiles => substreams are "per tile".  iNumSubstreams has already been multiplied.
		Int iNumSubstreamsPerTile = *pETRI_InfoofCU.iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
		*uiSubStrm = rpcPic->getPicSym()->getTileIdxMap(*uiCUAddr)*iNumSubstreamsPerTile + *uiLin % iNumSubstreamsPerTile;
	}
	else
	{	// dependent tiles => substreams are "per frame".
		*uiSubStrm = *uiLin % *pETRI_InfoofCU.iNumSubstreams;
	}

#if !ETRI_REMOVE_CODER_RESET
	if ( ((pcSlice->getPPS()->getNumSubstreams() > 1) ||*pETRI_InfoofCU.depSliceSegmentsEnabled) && (*uiCol == *uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
	{
		// We'll sync if the TR is available.
		TComDataCU *pcCUUp = pcCU->getCUAbove();
		UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
		UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
		TComDataCU *pcCUTR = NULL;
		if ( pcCUUp && ((*uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
		{
			pcCUTR = rpcPic->getCU( *uiCUAddr - uiWidthInCU + 1 );
		}
		if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
			(pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
			((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(*uiCUAddr)))
			)
			)
		{
			// TR not available.
		}
		else
		{
			// TR is available, we use it.
			ppppcRDSbacCoders[*uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[*uiTileCol] );
		}
	}
	m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[*uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code


	// reset the entropy coder
	if( *uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(*uiCUAddr))->getFirstCUAddr() &&	  // must be first CU of tile
		*uiCUAddr!=0 && 																							  // cannot be first CU of picture
		*uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
		*uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())	  // cannot be first CU of slice
	{
		SliceType sliceType = pcSlice->getSliceType();
		if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE){
			sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
		}

		m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
		m_pcEntropyCoder->setEntropyCoder	( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
		m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
		m_pcEntropyCoder->setEntropyCoder	( m_pcSbacCoder, pcSlice );
	}

#endif

	// set go-on entropy coder
	m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
	m_pcEntropyCoder->setBitstream( &pcBitCounters[*uiSubStrm] );

	((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);


}


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initialization of Rate Control Based on SBAC Coding Tool 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TEncSlice::ETRI_InitRateControlSBACRD(TComPic*& rpcPic, TComDataCU*& pcCU, TComSlice* pcSlice, TComBitCounter* pcBitCounters)
{
	if ( m_pcCfg->getUseRateCtrl() )
	{
#if KAIST_RC
		Int intra_size = m_pcCfg->getIntraPeriod();;
		Int iIDRIndex = pcCU->getPic()->getPOC() / intra_size;
		Int iIDRModulus = pcCU->getPic()->getPOC() % intra_size;
		TRCPic* tRCPic = m_pcRateCtrl->getTRCPic(iIDRModulus);
#endif

		Int estQP 	   = pcSlice->getSliceQp();
		Double estLambda = -1.0;
		Double bpp	   = -1.0;

		if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
		{
			estQP = pcSlice->getSliceQp();
		}
		else
		{
#if KAIST_RC
			bpp = tRCPic->getLCUTargetBpp(pcCU->getAddr(), pcSlice->getSliceType());
			if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
			{
				estLambda = tRCPic->getLCUEstLambdaAndQP(pcCU->getAddr(), bpp, pcSlice->getSliceQp(), &estQP);
			}
			else
			{
				estLambda = tRCPic->getLCUEstLambda(pcCU->getAddr(), bpp);
				estQP = tRCPic->getLCUEstQP(pcCU->getAddr(), estLambda, pcSlice->getSliceQp());
			}
#endif
			estQP	  = Clip3( -pcSlice->getSPS()->getQpBDOffsetY(), MAX_QP, estQP );

			m_pcRdCost->setLambda(estLambda);
#if RDOQ_CHROMA_LAMBDA
// set lambda for RDOQ
			Double weight=m_pcRdCost->getChromaWeight();
			const Double lambdaArray[3] = { estLambda, (estLambda / weight), (estLambda / weight) };
			m_pcTrQuant->setLambdas( lambdaArray );
#else
			m_pcTrQuant->setLambda( estLambda );
#endif
		}
#if KAIST_RC
		tRCPic->getLCU(pcCU->getAddr()).m_QP = estQP;
#endif
#if ADAPTIVE_QP_SELECTION
		pcCU->getSlice()->setSliceQpBase( estQP );
#endif
	}


}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief:  restore entropy coder to an initial stage after comress CU
	@retval:  TRUE : Break for multiple CU process  FALSE : Non Break
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncSlice::ETRI_RestoreEntropyCoder(TEncSbac****& ppppcRDSbacCoders, TEncBinCABAC*& pppcRDSbacCoder, 
										 TComDataCU*& pcCU, TComSlice* pcSlice, TComBitCounter* pcBitCounters, ETRI_InfoofCU& pETRI_InfoofCU)
{
	UInt*	uiSubStrm 	= pETRI_InfoofCU.uiSubStrm;
	UInt*	uiCol 		= pETRI_InfoofCU.uiCol;
	UInt*	uiTileLCUX	= pETRI_InfoofCU.uiTileLCUX;
	UInt*	uiTileCol		= pETRI_InfoofCU.uiTileCol;

	*pETRI_InfoofCU.bBreak = false;

	// restore entropy coder to an initial stage
	m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
	m_pcEntropyCoder->setBitstream( &pcBitCounters[*uiSubStrm] );
	m_pcCuEncoder->setBitCounter( &pcBitCounters[*uiSubStrm] );
	m_pcBitCounter = &pcBitCounters[*uiSubStrm];

	pppcRDSbacCoder->setBinCountingEnableFlag( true );
	m_pcBitCounter->resetBits();
	pppcRDSbacCoder->setBinsCoded( 0 );

	m_pcCuEncoder->encodeCU( pcCU );

	pppcRDSbacCoder->setBinCountingEnableFlag( false );
	if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
	{
		pcSlice->setNextSlice( true );
		*pETRI_InfoofCU.bBreak = true; return ; 	/// 2015 5 15 by Seok : break;
	}
	if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
	{
		pcSlice->setNextSliceSegment( true );
		*pETRI_InfoofCU.bBreak = true; return ;		/// 2015 5 15 by Seok : break;
	}


#if !ETRI_REMOVE_CODER_RESET
	ppppcRDSbacCoders[*uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
	//Store probabilties of second LCU in line into buffer
	if ( ( *uiCol == (*uiTileLCUX) + 1) && (*pETRI_InfoofCU.depSliceSegmentsEnabled || (pcSlice->getPPS()->getNumSubstreams() > 1)) && m_pcCfg->getWaveFrontsynchro())
	{
		m_pcBufferSbacCoders[*uiTileCol].loadContexts(ppppcRDSbacCoders[*uiSubStrm][0][CI_CURR_BEST]);
	}
#endif

}

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief:  Update Parameters of Rate Control after comress CU. In V12, ETRI_RateControlSBACRD is same function
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if KAIST_RC
Void TEncSlice::ETRI_RestoreRateControl(TComPic*& rpcPic, TComDataCU*& pcCU, ETRI_InfoofCU& pETRI_InfoofCU)
{
	if ( m_pcCfg->getUseRateCtrl() )
	{
		Int 		actualQP		  	= g_RCInvalidQPValue;
		Double	actualLambda 	= m_pcRdCost->getLambda();
		Int 		actualBits	  	= pcCU->getTotalBits();
		Int 		numberOfEffectivePixels  = 0;
		for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
		{
			if ( pcCU->getPredictionMode( idx ) != MODE_NONE && ( !pcCU->isSkipped( idx ) ) )
			{
				numberOfEffectivePixels = numberOfEffectivePixels + 16;
				break;
			}
		}

		actualQP = ( numberOfEffectivePixels == 0 )? g_RCInvalidQPValue : pcCU->getQP(0);
		m_pcRdCost->setLambda(*pETRI_InfoofCU.oldLambda);

#if KAIST_RC
		Int intra_size = m_pcCfg->getIntraPeriod();
		Int iIDRIndex = pcCU->getPic()->getPOC() / intra_size;
		Int iIDRModulus = pcCU->getPic()->getPOC() % intra_size;
		TRCPic* tRCPic = m_pcRateCtrl->getTRCPic(iIDRModulus);
		tRCPic->updateAfterLCU(pcCU->getAddr(), actualBits, actualQP, actualLambda,
			pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
#endif
	}

}
#endif


/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief:  Update Parameters of Slice Segment after comress CU
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
Void TEncSlice::ETRI_UpdateSliceSegemnt(TComSlice* pcSlice, ETRI_InfoofCU& pETRI_InfoofCU)
{

	if ((pcSlice->getPPS()->getNumSubstreams() > 1) && !*pETRI_InfoofCU.depSliceSegmentsEnabled)
	{
		pcSlice->setNextSlice( true );
	}
	if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
	{
		if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
		{
			pcSlice->setNextSlice( true );
		}
		else
		{
			pcSlice->setNextSliceSegment( true );
		}
	}
	if( *pETRI_InfoofCU.depSliceSegmentsEnabled )
	{
		if (m_pcCfg->getWaveFrontsynchro())
		{
			CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[*pETRI_InfoofCU.uiTileCol] );//ctx 2.LCU
		}
		CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
	}

}

/**
=====================================================================================================================
	@brief: For New Class such as TEncTile, TEncWPP and Others
	@param: TComPic*& rpcPic
	@param: TComSlice* pcSlice
	@param: TEncSbac****& ppppcRDSbacCoders [*uiSubStrm][0][CI_CURR_BEST] : Header Information, From pcEncTop in spite of Parallel.
	@param: TEncBinCABAC*& pppcRDSbacCoder  [CI_CURR_BEST]: Residual Information (DCT/Q), From TEncTile Class when Parallel.
	@param: TComBitCounter* pcBitCounters  : Bit Counter for RD cost. From TEncTile Class when Parallel. 
	@author: Jinwuk Seok  2015 5 11 
=====================================================================================================================
*/
Void TEncSlice::ETRI_CompressUnit(TComPic*& rpcPic, TComSlice* pcSlice, TEncSbac****& ppppcRDSbacCoders, TEncBinCABAC*& pppcRDSbacCoder, TComBitCounter* pcBitCounters, ETRI_InfoofCU& pETRI_InfoofCU)
{
	UInt uiEncCUOrder = 0;
	UInt ETRI_StartCUOrder = pETRI_InfoofCU.ETRI_StartCUOrder;
	UInt ETRI_FinalCUOrder = pETRI_InfoofCU.ETRI_FinalCUOrder;
	UInt uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(ETRI_StartCUOrder); 	

	for(uiEncCUOrder = ETRI_StartCUOrder; uiEncCUOrder < ETRI_FinalCUOrder; uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
	{
		*pETRI_InfoofCU.uiCUAddr = uiCUAddr;				///Indicate the Cu Address to all Functions : 2015 5 20 by Seok

		// initialize CU encoder
		TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
		pcCU->initCU( rpcPic, uiCUAddr );
		
		ETRI_InitRDCoderForSubStream(ppppcRDSbacCoders, rpcPic, pcCU, pcSlice, pcBitCounters, pETRI_InfoofCU);			///Initilization Entropy Coder (TEncSBAC for CU Compression,	m_pcRDGoOnSbacCoder) @ 2015 5 15 by Seok
		ETRI_InitRateControlSBACRD(rpcPic, pcCU, pcSlice, pcBitCounters);											///Initilization of Rate Control @ 2015 5 15 by Seok
		
		// run CU encoder
		m_pcCuEncoder->compressCU( pcCU );
		
		// Restore CU encoder
		ETRI_RestoreEntropyCoder(ppppcRDSbacCoders, pppcRDSbacCoder, pcCU, pcSlice, pcBitCounters, pETRI_InfoofCU); 	///Restore Entropy Coder to Initial Stage @ 2015 5 15 by Seok
		if (*pETRI_InfoofCU.bBreak){break;} 																///Break signal Generated from  ETRI_RestoreEntropyCoder (Almost negligable) @ 2015 5 15 by Seok
#if KAIST_RC
		ETRI_RestoreRateControl(rpcPic, pcCU, pETRI_InfoofCU);													///Restore Rate Control State @ 2015 5 15 by Seok 
#endif

		pETRI_InfoofCU.u64PicTotalBits 	+= pcCU->getTotalBits();
		pETRI_InfoofCU.u64PicDist		+= pcCU->getTotalDistortion();
		pETRI_InfoofCU.dPicRdCost		+= pcCU->getTotalCost();

	}

}


/**
=====================================================================================================================
	@brief: Macro and Service Function for ETRI_compressSlice
	@author: Jinwuk Seok  2015 5 11 
=====================================================================================================================
*/
/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Wrapper Function to Tile Compression Function. 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if _ETRI_WINDOWS_APPLICATION
Void TEncSlice::ETRI_CompressTile(TEncTile* e_pcTileEncoder) 
{
	e_pcTileEncoder->ETRI_CompressUnit();
}
#else
Void* TEncSlice::ETRI_CompressTile(void* param)
{
	TEncTile* e_pcTileEncoder = (TEncTile*)param;
	e_pcTileEncoder->ETRI_CompressUnit();
	return NULL;
}
#endif

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Initilization of TEncBinCABAC Coder (pppcRDSbacCoder) for  Slice Encoding :
			Including some member params of TencSlice 
	@author: Jinwuk Seok  2015 5 11 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncSlice::ETRI_InitcompressSlice(TEncBinCABAC*& pppcRDSbacCoder, CI_IDX  e_CI_IDX, TComSlice* pcSlice)	
{
	// initialize cost values
	m_uiPicTotalBits  = 0;
	m_dPicRdCost	  = 0;
	m_uiPicDist 	  = 0;
	
	// set entropy coder : HERE, e_CI_IDX = CI_CURR_BEST 
	m_pcSbacCoder->init( m_pcBinCABAC );
	m_pcEntropyCoder->setEntropyCoder	( m_pcSbacCoder, pcSlice );
	m_pcEntropyCoder->resetEntropy		();
	m_pppcRDSbacCoder[0][e_CI_IDX]->load(m_pcSbacCoder);
	pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][e_CI_IDX]->getEncBinIf();
	pppcRDSbacCoder->setBinCountingEnableFlag( false );
	pppcRDSbacCoder->setBinsCoded( 0 );

	((TEncBinCABAC *)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinsCoded(0);	///ETRI_MULTITHREAD_BUGFIX : @ 2015 5 19 by Seok	

}

__inline Void TEncSlice::ETRI_SetSliceRDOResult(UInt uiNumTiles)
{
	ETRI_InfoofCU*	e_InfoOfCU = nullptr;
	for(UInt iTileIdx = 0; iTileIdx < uiNumTiles; iTileIdx++)
	{
		e_InfoOfCU = em_pcTileEncoder[iTileIdx].ETRI_getInfoofCU(); 			
		m_uiPicTotalBits	+= e_InfoOfCU->u64PicTotalBits;
		m_uiPicDist 		+= e_InfoOfCU->u64PicDist;
		m_dPicRdCost		+= e_InfoOfCU->dPicRdCost;
	}
}	

/**
------------------------------------------------------------------------------------------------------------------------------------------------
	@brief: Print Debug Information for ETRI_FPU_STATISTICS
	@author: Jinwuk Seok  2015 11 17 
------------------------------------------------------------------------------------------------------------------------------------------------
*/
__inline Void TEncSlice::ETRI_SliceDebugInfo(TComSlice* pcSlice, UInt uiNumTiles)
{
#if (ETRI_MODIFICATION_V03 && ETRI_FPU_STATISTICS)
	/// 2015 11 14 by Seok : It's For Debug or Analysis
	TEncCu* 	e_pcCUEncoder = nullptr;
	UInt*	e_uiDBGInfo = nullptr;
	UInt		uiSliceDBGData[ETRI_nDbgInfo];

	memset(&uiSliceDBGData[0], 0, ETRI_nDbgInfo * sizeof(Int));
	
	for(UInt iTileIdx = 0; iTileIdx < uiNumTiles; iTileIdx++)
	{
		e_pcCUEncoder 	= em_pcTileEncoder[iTileIdx].ETRI_getTileCuEncoder();
		e_uiDBGInfo 		= e_pcCUEncoder->ETRI_getCUDebugData();
		
		uiSliceDBGData[0] 	+= e_uiDBGInfo[ETRI_nDBGInfo_TotalInfo];
		uiSliceDBGData[1] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE01];
		uiSliceDBGData[2] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE02];
		uiSliceDBGData[3] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE03];
		uiSliceDBGData[4] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE04];
		uiSliceDBGData[5] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE05];
		uiSliceDBGData[6] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE06];
		uiSliceDBGData[7] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE07];
		uiSliceDBGData[9] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE09];
		uiSliceDBGData[10] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE010];
		uiSliceDBGData[11] 	+= e_uiDBGInfo[ETRI_nDBGInfo_CASE011];
		uiSliceDBGData[12]  += e_uiDBGInfo[ETRI_nDBGInfo_CASE012];
		uiSliceDBGData[13]  += e_uiDBGInfo[ETRI_nDBGInfo_CASE013];

	}
	
	if (pcSlice->getPOC() == 0)
	{
		EDPRINTF(stderr, "Case1 : SKIP/MERGE/INTER/INTRA 중 하나만 수행하는 경우 \n");
		EDPRINTF(stderr, "Case2 : ALL Modes SKipped \n");
		EDPRINTF(stderr, "Case3 : FFESD \n");
		EDPRINTF(stderr, "Case4 :  \n");
		EDPRINTF(stderr, "Case5 :  \n");
		EDPRINTF(stderr, "Case6 : \n");
		EDPRINTF(stderr, "Case7 : Total Merge 갯수\n");
		EDPRINTF(stderr, "Case9 : INTER/INTRA 미 분리\n");
		EDPRINTF(stderr, "Case10: \n");
		EDPRINTF(stderr, "Case11: \n");
		EDPRINTF(stderr, "Case12: \n");
		EDPRINTF(stderr, "Case13: Distortion Distribution & bSubBranch \n");


		EDPRINTF(stderr, "[PC:SD:QP]  ");
		fprintf(stderr, "Total    ");   
		fprintf(stderr, "Case1    ");	
		fprintf(stderr, "Case2    ");
		fprintf(stderr, "Case3    ");
		fprintf(stderr, "Case4    ");
		fprintf(stderr, "Case5    ");
		fprintf(stderr, "Case6    ");
		fprintf(stderr, "Case7    ");
		fprintf(stderr, "Case9    ");
		fprintf(stderr, "Case10   ");
		fprintf(stderr, "Case11   ");
		fprintf(stderr, "Case12   ");
		fprintf(stderr, "Case13   ");
		fprintf(stderr, "\n");
	}
	else
	{
		EDPRINTF(stderr, "[%2d:%2d:%2d] %6d   %6d   %6d   %6d   %6d   %6d   %6d   %6d   %6d   %6d   %6d   %6d   %6d\n",   
						pcSlice->getPOC(), pcSlice->getDepth(), pcSlice->getSliceQp(),
						uiSliceDBGData[0],	uiSliceDBGData[1], uiSliceDBGData[2], uiSliceDBGData[3], uiSliceDBGData[4], uiSliceDBGData[5],
						uiSliceDBGData[6], uiSliceDBGData[7], uiSliceDBGData[9], uiSliceDBGData[10], uiSliceDBGData[11], uiSliceDBGData[12],
						uiSliceDBGData[13]);
	}

#endif
}



#define	ETRI_SetInfoofCU(InfoCU, uiCUAddr, uiTileCol, uiTileStartLCU, uiWidthInLCUs, uiLin, uiSubStrm, uiCol, uiStartCUAddr, 	uiBoundingCUAddr, \
	uiTileLCUX, iNumSubstreams, depSliceSegmentsEnabled, oldLambda, bBreak) \
	InfoCU.uiCUAddr 		= &uiCUAddr; \
	InfoCU.uiTileCol		= &uiTileCol; \
	InfoCU.uiTileStartLCU 	= &uiTileStartLCU; \
	InfoCU.uiWidthInLCUs 	= &uiWidthInLCUs; \
	InfoCU.uiLin    		= &uiLin; \
	InfoCU.uiSubStrm 		= &uiSubStrm; \
	InfoCU.uiCol    		= &uiCol; \
	InfoCU.uiStartCUAddr 	= &uiStartCUAddr; \
	InfoCU.uiBoundingCUAddr = &uiBoundingCUAddr; \
	InfoCU.uiTileLCUX    	= &uiTileLCUX; \
	InfoCU.iNumSubstreams	= &iNumSubstreams; \
	InfoCU.depSliceSegmentsEnabled = &depSliceSegmentsEnabled; \
	InfoCU.oldLambda    	= &oldLambda; \
	InfoCU.bBreak    		= &bBreak; \
	InfoCU.u64PicTotalBits 	= 0; \
	InfoCU.u64PicDist   	= 0; \
	InfoCU.dPicRdCost  	= 0.0; 

/**
=======================================================================================================================
	@brief :  The main function of Slice encoding. it is included almost basic structure of HEVC encoder
	@param: TComPic*& rpcPic
	@param: Bool bOperation 	Decide ETRI_compressSice or Original HM compressSlice
	@author: Jinwuk Seok : 2015 5 14 
=======================================================================================================================
*/
Void TEncSlice::ETRI_compressSlice( TComPic*& rpcPic, Bool bOperation)
{
	if (!bOperation ||!ETRI_MODIFICATION_V00)	{compressSlice(rpcPic);};	/// ETRI_MODIFICATION_V00 for Code consistency : 2015 5 14 by Seok

	UInt	uiCUAddr;
	UInt	uiStartCUAddr;
	UInt	uiBoundingCUAddr;
	Int		iNumSubstreams = 1;

	UInt uiWidthInLCUs = rpcPic->getPicSym()->getFrameWidthInCU();    /// UInt uiHeightInLCUs = rpcPic->getPicSym()->getFrameHeightInCU(); 
	UInt uiCol=0, uiLin=0, uiSubStrm=0;
	UInt uiTileCol = 0, uiTileStartLCU=0, uiTileLCUX=0;
	UInt uiNumTiles=0;	  

	Bool bBreak = false;

	Double oldLambda = m_pcRdCost->getLambda();

	rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
	TEncBinCABAC*	pppcRDSbacCoder = nullptr;
	TComSlice*		pcSlice	 		= rpcPic->getSlice(getSliceIdx());

#if ETRI_MULTITHREAD_2
	TEncSbac****  	ppppcRDSbacCoders	= em_pcEncFrame->ETRI_getRDSbacCoders();
	TComBitCounter*	pcBitCounters   	= em_pcEncFrame->ETRI_getBitCounters();
#else
	TEncTop*   		pcEncTop    		= (TEncTop*) m_pcCfg;
	TEncSbac****  	ppppcRDSbacCoders	= pcEncTop->getRDSbacCoders();
	TComBitCounter*	pcBitCounters   	= pcEncTop->getBitCounters();
#endif
	Bool depSliceSegmentsEnabled   	= pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();

	// ====================================================================================================================
	// Ready for Slice Encoding
	//====================================================================================================================
	xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );		///<Generating uiStartCUAddr, uiBoundingCUAddr @ 2015 5 15 by Seok

	uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
	uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();

	ETRI_InfoofCU pETRI_InfoofCU;
	ETRI_SetInfoofCU(pETRI_InfoofCU, uiCUAddr, uiTileCol, uiTileStartLCU, uiWidthInLCUs, uiLin,uiSubStrm, uiCol,uiStartCUAddr, uiBoundingCUAddr, \
		uiTileLCUX, iNumSubstreams,depSliceSegmentsEnabled,oldLambda,bBreak);

	ETRI_InitcompressSlice(pppcRDSbacCoder, CI_CURR_BEST, pcSlice);							///Set  pppcRDSbacCoder = m_pppcRDSbacCoder[0][ CI_CURR_BEST]->load(m_pcSbacCoder) @ 2015 5 15 by Seok
	ETRI_InitWeightedPrediction(pcSlice);													///For WeightedPrediction, However almost all codes are Skipped. But We should Check the Codes for useful Predictions @ 2015 5 15 by Seok
	ETRI_GetUseSABACRD(ppppcRDSbacCoders, CI_CURR_BEST, pcSlice, rpcPic, iNumSubstreams); 	///Set ppppcRDSbacCoders @ 2015 5 15 by Seok		
	ETRI_InitDependentSlices(rpcPic, pcSlice, ppppcRDSbacCoders, pETRI_InfoofCU);			///When We don't use  DependentSlices, this function is fully skpped @ 2015 5 15 by Seok

	// ====================================================================================================================
	// Compress Slice Main Process
	//====================================================================================================================
	// for every CU in slice

//	UInt uiProcessingType = PARALLEL_SERIAL;
//	UInt uiProcessingType = PARALLEL_TILE_SERIAL;
	UInt uiProcessingType = ORIGINAL_PROC;	//Macro change by yhee, 2015.07.01
#if ETRI_MULTITHREAD
	uiProcessingType = (ETRI_TILE_SERIAL_TEST)? PARALLEL_TILE_SERIAL: PARALLEL_TILE;
#endif
	
	switch (uiProcessingType)
	{
	case PARALLEL_TILE:
	{
#if ETRI_MULTITHREAD_2
		  int i;
#if ETRI_DEBUG_CODE_CLEANUP
			ESPRINTF((pcSlice->getPOC() == 0), stderr, "========== ETRI_RETURN_PARALLEL_TILE =========\n");
#endif 
#if 0 //_YHEEDEBUG
						  EDPRINTF(stderr, "POC: %d\n", pcSlice->getPOC());
#endif
			
#if ETRI_MultiplePPS
						  //smple version
						  UInt uiNumTile = pcSlice->getPic()->getPicSym()->getNumTiles();
#else
						  UInt uiNumTile = (m_pcCfg->getNumColumnsMinus1() + 1) * (m_pcCfg->getNumRowsMinus1() + 1);
#endif
			ETRI_InfoofCU **rpCurrInfo = new ETRI_InfoofCU*[uiNumTile];

			for (i = 0; i < uiNumTile; i++)
			{
				em_pcTileEncoder[i].ETRI_initTileCoders(rpcPic, pcSlice, m_pcRdCost, m_pcTrQuant, m_pcCavlcCoder, m_pcSbacCoder, m_pcBinCABAC
#if KAIST_RC
          , m_pcRateCtrl
#endif
          );
				if (i == 0)	em_pcTileEncoder[i].ETRI_getTileCUAddr(rpCurrInfo[i], NULL, pETRI_InfoofCU, i);
				else em_pcTileEncoder[i].ETRI_getTileCUAddr(rpCurrInfo[i], rpCurrInfo[i - 1], pETRI_InfoofCU, i);
			}

#if ETRI_THREADPOOL_OPT

			struct timeval tv;
			tv.tv_sec = 0;
			tv.tv_usec = 50; // 10 us

			const int nJob = MAX_THREAD_TILE; //yhee 2016.09.21

			EncTileJob* job[nJob];

			pthread_mutex_t encMutex = PTHREAD_MUTEX_INITIALIZER;
			pthread_cond_t  encCond = PTHREAD_COND_INITIALIZER;

			EncTileInfo encInfo;
			encInfo.endCount = new int[1];
			*encInfo.endCount = 0;
			encInfo.mutex = &encMutex;
			encInfo.cond = &encCond;
			encInfo.nTile = uiNumTile;

#if ETRI_THREAD_LOAD_BALANCING
			bool bBalanced = false;
			int nBalancedTile = 0;
			int nWaitingThread = 0;

			pthread_mutex_t lbMutex = PTHREAD_MUTEX_INITIALIZER;
			pthread_cond_t lbCond = PTHREAD_COND_INITIALIZER;


			TileLoadBalanceInfo lbInfo;
			lbInfo.bBalanced = &bBalanced;
			lbInfo.nBalancedTile = &nBalancedTile;
			lbInfo.nMaxThread = 8;
			lbInfo.nTile = uiNumTile;
			lbInfo.nWaitingThread = &nWaitingThread;
			lbInfo.mutex = &lbMutex;
			lbInfo.cond = &lbCond;

			//lbInfo.sliceCnt = tempSliceCnt++;
#endif		

#if ETRI_THREAD_LOAD_BALANCING
			pthread_mutex_lock(&g_jobMutex);
#endif
			for (i = 0; i < uiNumTile; i++)
			{
				encInfo.id = i;
#if ETRI_THREAD_LOAD_BALANCING
				lbInfo.id = i;
				job[i] = EncTileJob::createJob(this, encInfo, lbInfo);
#else
				job[i] = EncTileJob::createJob(this, encInfo);
#endif
				gop_QphotoPool->run(job[i]);
			}
#if ETRI_THREAD_LOAD_BALANCING
			pthread_mutex_unlock(&g_jobMutex);
#endif

#if ETRI_TILE_THEAD_OPT
			pthread_mutex_lock(&encMutex);
			if (*encInfo.endCount < uiNumTile)
				pthread_cond_wait(&encCond, &encMutex);
			pthread_mutex_unlock(&encMutex);
#else
			bool bEnd = false;
			while (1)
			{
				bEnd = false;
				pthread_mutex_lock(&encMutex);

				if (*encInfo.endCount >= uiNumTile)
					bEnd = true;
				pthread_mutex_unlock(&encMutex);

				if (bEnd)
					break;
				else
				{
#if !(_ETRI_WINDOWS_APPLICATION)
					usleep(50);
#else
					//select(0, NULL, NULL, NULL, &tv); // sleep 10 us
					Sleep(1);
#endif
				}
			}
#endif

			if(encInfo.endCount)
				delete encInfo.endCount;
			encInfo.endCount = NULL;
			
#else
			for (i = 0; i < uiNumTile; i++)
			{
				// 여유 있는 스레드를 찾는다.
				em_hThreadPoolTile.AddThread(this, i, false);
							  //Sleep(1);
			}

			// 모든 스레드가 끝날때까지 기다린다.
			em_hThreadPoolTile.GetAllFreeThreadWaiting();
#endif
						  
			for (i = 0; i < uiNumTile; i++)
			{
				m_uiPicTotalBits += rpCurrInfo[i]->u64PicTotalBits;
				m_uiPicDist += rpCurrInfo[i]->u64PicDist;
				m_dPicRdCost += rpCurrInfo[i]->dPicRdCost;
			}

			delete[] rpCurrInfo;
			/// 2015 11 17 by Seok : For Debug, It shoud be REMOVE : ETRI_FPU_STATISTICS
			ETRI_SliceDebugInfo(pcSlice, uiNumTiles);
#else

#if (_ETRI_WINDOWS_APPLICATION)
		{
				ESPRINTF((pcSlice->getPOC() == 0), stderr, "========== PARALLEL [TILE:THREAD] =========\n");
				HANDLE		handles[ETRI_MAX_TILES];

				//initialize for SAO  : Is it Possible to move ???? 
				m_pcEntropyCoder->setEntropyCoder(m_pcRDGoOnSbacCoder, pcSlice);
				m_pcEntropyCoder->setBitstream(&pcBitCounters[0]);

				ETRI_InfoofCU	*e_InfoOfCU = nullptr, *e_PrevOfCU = nullptr;

				uiNumTiles = em_pcTileEncoder[0].ETRI_getTotalNumbetOfTile();
				for (UInt iTileIdx = 0; iTileIdx < uiNumTiles; iTileIdx++)
				{
					em_pcTileEncoder[iTileIdx].ETRI_initTileCoders(rpcPic, pcSlice, m_pcRdCost, m_pcTrQuant, m_pcCavlcCoder, m_pcSbacCoder, m_pcBinCABAC
#if KAIST_RC
            , m_pcRateCtrl
#endif
            );
					em_pcTileEncoder[iTileIdx].ETRI_getTileCUAddr(e_InfoOfCU, e_PrevOfCU, pETRI_InfoofCU, iTileIdx);

					handles[iTileIdx] = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ETRI_CompressTile, (LPVOID)&em_pcTileEncoder[iTileIdx], 0, NULL);
				}
				DWORD dwWaitResult = WaitForMultipleObjects(uiNumTiles, handles, true, INFINITE);
				for (UInt uiTileIdx = 0; uiTileIdx < uiNumTiles; uiTileIdx++)
					CloseHandle(handles[uiTileIdx]);

				ETRI_SetSliceRDOResult(uiNumTiles);	///After Parallel Processing, Summation of each RD Results in Parallel Processing Units @ 2015 5 19 by Seok
			}
#else //Works on Linux by yhee 2016.04.19
		{
			if (pcSlice->getPOC() == 0)
				fprintf(stderr, "========== PARALLEL [TILE:THREAD] =========\n");
			m_pcEntropyCoder->setEntropyCoder(m_pcRDGoOnSbacCoder, pcSlice);
			m_pcEntropyCoder->setBitstream(&pcBitCounters[0]);
			ETRI_InfoofCU	*e_InfoOfCU = nullptr, *e_PrevOfCU = nullptr;
			uiNumTiles = em_pcTileEncoder[0].ETRI_getTotalNumbetOfTile();

			sem_init(&em_TileSemaphore, 0, 1 * uiNumTiles);

			for (UInt iTileIdx = 0; iTileIdx < uiNumTiles; iTileIdx++)
			{
				//fprintf(stdout, "Tild ID: %d\n", iTileIdx);	
				em_pcTileEncoder[iTileIdx].ETRI_initTileCoders(rpcPic, pcSlice, m_pcRdCost, m_pcTrQuant, m_pcCavlcCoder, m_pcSbacCoder, m_pcBinCABAC, m_pcRateCtrl);
				em_pcTileEncoder[iTileIdx].ETRI_getTileCUAddr(e_InfoOfCU, e_PrevOfCU, pETRI_InfoofCU, iTileIdx);
				int success = pthread_create(&em_TileThreadID[iTileIdx], NULL, &TEncSlice::ETRI_CompressTile, (void *)&em_pcTileEncoder[iTileIdx]);
				if (success != 0){
					fprintf(stderr, "Error at pthread_create: %s\n", strerror(errno));
					exit(1);
				}
				if (sem_wait(&em_TileSemaphore) < 0) {
					fprintf(stderr, "Sem_wait failed: %s\n", strerror(errno));
					exit(1);
				}
			}

			for (UInt uiTileIdx = 0; uiTileIdx < uiNumTiles; uiTileIdx++)
				pthread_join(em_TileThreadID[uiTileIdx], NULL);
			//printf("\nAll threads are done now\n");

			ETRI_SetSliceRDOResult(uiNumTiles);
		}
#endif //(_ETRI_WINDOWS_APPLICATION)

#endif //ETRI_MULTITHREAD_2
						
		}
		break;
		

	case PARALLEL_TILE_SERIAL:
		{
#if (_ETRI_WINDOWS_APPLICATION)
								 ESPRINTF((pcSlice->getPOC() == 0), stderr, "========== PARALLEL [TILE:SERIAL] =========\n");
#else
								 if (pcSlice->getPOC() == 0)
									 fprintf(stderr, "========== PARALLEL [TILE:SERIAL] =========\n");
#endif

#if ETRI_MULTITHREAD_2
#if ETRI_MultiplePPS
								 //more simplify
								 UInt uiNumTile = pcSlice->getPic()->getPicSym()->getNumTiles();
#else
								 UInt uiNumTile = (m_pcCfg->getNumColumnsMinus1() + 1) * (m_pcCfg->getNumRowsMinus1() + 1);
#endif
			ETRI_InfoofCU **rpCurrInfo = new ETRI_InfoofCU*[uiNumTile];

			for (int i = 0; i < uiNumTile; i++)
			{
				em_pcTileEncoder[i].ETRI_initTileCoders(rpcPic, pcSlice, m_pcRdCost, m_pcTrQuant, m_pcCavlcCoder, m_pcSbacCoder, m_pcBinCABAC
#if KAIST_RC
          , m_pcRateCtrl
#endif
          );
				if (i == 0)	em_pcTileEncoder[i].ETRI_getTileCUAddr(rpCurrInfo[i], NULL, pETRI_InfoofCU, i);
				else em_pcTileEncoder[i].ETRI_getTileCUAddr(rpCurrInfo[i], rpCurrInfo[i - 1], pETRI_InfoofCU, i);									 
				em_pcTileEncoder[i].ETRI_CompressUnit();
			}								 							 
			for (int i = 0; i < uiNumTile; i++)
			{
				m_uiPicTotalBits += rpCurrInfo[i]->u64PicTotalBits;
				m_uiPicDist += rpCurrInfo[i]->u64PicDist;
				m_dPicRdCost += rpCurrInfo[i]->dPicRdCost;
			}

			delete[] rpCurrInfo;
#else					

			//initialize for SAO  : Is it Possible to move ???? 
			m_pcEntropyCoder->setEntropyCoder(m_pcRDGoOnSbacCoder, pcSlice);
			m_pcEntropyCoder->setBitstream(&pcBitCounters[0]);

			ETRI_InfoofCU	*e_InfoOfCU = nullptr, *e_PrevOfCU = nullptr;

			uiNumTiles = em_pcTileEncoder[0].ETRI_getTotalNumbetOfTile();
			for (UInt iTileIdx = 0; iTileIdx < uiNumTiles; iTileIdx++)
			{
				em_pcTileEncoder[iTileIdx].ETRI_initTileCoders(rpcPic, pcSlice, m_pcRdCost, m_pcTrQuant, m_pcCavlcCoder, m_pcSbacCoder, m_pcBinCABAC
#if KAIST_RC
          , m_pcRateCtrl
#endif
          );
				em_pcTileEncoder[iTileIdx].ETRI_getTileCUAddr(e_InfoOfCU, e_PrevOfCU, pETRI_InfoofCU, iTileIdx);

				em_pcTileEncoder[iTileIdx].ETRI_CompressUnit();
			}
			ETRI_SetSliceRDOResult(uiNumTiles);	///After Parallel Processing, Summation of each RD Results in Parallel Processing Units @ 2015 5 19 by Seok
#endif
		}
		break;

	case PARALLEL_WPP:
		{


		}
		break;

	default:
		{
			

#if (_ETRI_WINDOWS_APPLICATION)			
			ESPRINTF((pcSlice->getPOC() == 0), stderr, "========== ORIGINAL_PROC =========\n");
#else
			if (pcSlice->getPOC() == 0)
				fprintf(stderr, "========== ORIGINAL_PROC =========\n");
#endif

			pETRI_InfoofCU.ETRI_StartCUOrder = uiStartCUAddr / rpcPic->getNumPartInCU();
			pETRI_InfoofCU.ETRI_FinalCUOrder = (uiBoundingCUAddr + (rpcPic->getNumPartInCU() - 1)) / rpcPic->getNumPartInCU();

			ETRI_CompressUnit(rpcPic, pcSlice, ppppcRDSbacCoders, pppcRDSbacCoder, pcBitCounters, pETRI_InfoofCU);

			m_uiPicTotalBits = pETRI_InfoofCU.u64PicTotalBits;
			m_uiPicDist = pETRI_InfoofCU.u64PicDist;
			m_dPicRdCost = pETRI_InfoofCU.dPicRdCost;
		}
		break;
	}

	// ====================================================================================================================
	// Final Processing to each Slice
	//====================================================================================================================

	ETRI_UpdateSliceSegemnt(pcSlice, pETRI_InfoofCU);
	xRestoreWPparam( pcSlice );
}
//====================================================================================================================
#endif		///#if ETRI_MODIFICATION_V00


/**
 \param  rpcPic        picture class
 \retval rpcBitstream  bitstream class
 */
Void TEncSlice::encodeSlice   ( TComPic*& rpcPic, TComOutputBitstream* pcSubstreams )
{
	UInt       uiCUAddr;
	UInt       uiStartCUAddr;
	UInt       uiBoundingCUAddr;
	TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());

	uiStartCUAddr=pcSlice->getSliceSegmentCurStartCUAddr();
	uiBoundingCUAddr=pcSlice->getSliceSegmentCurEndCUAddr();
	// choose entropy coder
	{
		m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
		m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
	}
  
	m_pcCuEncoder->setBitCounter( NULL );
	m_pcBitCounter = NULL;
	// Appropriate substream bitstream is switched later.
	// for every CU
#if ENC_DEC_TRACE
	g_bJustDoIt = g_bEncDecTraceEnable;
#endif
	DTRACE_CABAC_VL( g_nSymbolCounter++ );
	DTRACE_CABAC_T( "\tPOC: " );
	DTRACE_CABAC_V( rpcPic->getPOC() );
	DTRACE_CABAC_T( "\n" );
#if ENC_DEC_TRACE
	g_bJustDoIt = g_bEncDecTraceDisable;
#endif

#if ETRI_MULTITHREAD_2
	TEncSbac* pcSbacCoders = em_pcEncFrame->ETRI_getSbacCoders(); //coder for each substream
#else
	TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
	TEncSbac* pcSbacCoders = pcEncTop->getSbacCoders(); //coder for each substream
#endif
	Int iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();

	UInt uiBitsOriginallyInSubstreams = 0;
	{
		UInt uiTilesAcross = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
		for (UInt ui = 0; ui < uiTilesAcross; ui++)
		{
			m_pcBufferSbacCoders[ui].load(m_pcSbacCoder); //init. state
		}
    
		for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
		{
			uiBitsOriginallyInSubstreams += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
		}

		for (UInt ui = 0; ui < uiTilesAcross; ui++)
		{
			m_pcBufferLowLatSbacCoders[ui].load(m_pcSbacCoder);  //init. state
		}
	}

	UInt uiWidthInLCUs  = rpcPic->getPicSym()->getFrameWidthInCU();
	UInt uiCol=0, uiLin=0, uiSubStrm=0;
	UInt uiTileCol      = 0;
	UInt uiTileStartLCU = 0;
	UInt uiTileLCUX     = 0;
	Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
	uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());  /* for tiles, uiStartCUAddr is NOT the real raster scan address, it is actually
																							  an encoding order index, so we need to convert the index (uiStartCUAddr)
																							  into the real raster scan address (uiCUAddr) via the CUOrderMap */
	uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
	if( depSliceSegmentsEnabled )
	{
		if( pcSlice->isNextSlice()||
			uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr())
		{
			if(m_pcCfg->getWaveFrontsynchro())
			{
				CTXMem[1]->loadContexts(m_pcSbacCoder);
			}
			CTXMem[0]->loadContexts(m_pcSbacCoder);
		}
		else
		{
			if(m_pcCfg->getWaveFrontsynchro())
			{
				uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
				m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
				Int iNumSubstreamsPerTile = iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
				uiLin     = uiCUAddr / uiWidthInLCUs;
				uiSubStrm = rpcPic->getPicSym()->getTileIdxMap(rpcPic->getPicSym()->getCUOrderMap( uiCUAddr))*iNumSubstreamsPerTile
					+ uiLin%iNumSubstreamsPerTile;
				if ( (uiCUAddr%uiWidthInLCUs+1) >= uiWidthInLCUs  )
				{
					uiCol     = uiCUAddr % uiWidthInLCUs;
					uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
					if(uiCol==uiTileLCUX)
					{
						CTXMem[0]->loadContexts(m_pcSbacCoder);
					}
				}
			}
			pcSbacCoders[uiSubStrm].loadContexts( CTXMem[0] );
		}
	}

	UInt uiEncCUOrder;
	for( uiEncCUOrder = uiStartCUAddr /rpcPic->getNumPartInCU();
		uiEncCUOrder < (uiBoundingCUAddr+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU();
		uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
	{
		uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
		uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
		uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
		//UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
		uiCol     = uiCUAddr % uiWidthInLCUs;
		uiLin     = uiCUAddr / uiWidthInLCUs;
		if (pcSlice->getPPS()->getNumSubstreams() > 1)
		{
			// independent tiles => substreams are "per tile".  iNumSubstreams has already been multiplied.
			Int iNumSubstreamsPerTile = iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
			uiSubStrm = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)*iNumSubstreamsPerTile
				+ uiLin%iNumSubstreamsPerTile;
		}
		else
		{
			// dependent tiles => substreams are "per frame".
			uiSubStrm = uiLin % iNumSubstreams;
		}

		m_pcEntropyCoder->setBitstream( &pcSubstreams[uiSubStrm] );
		// Synchronize cabac probabilities with upper-right LCU if it's available and we're at the start of a line.
		if (((pcSlice->getPPS()->getNumSubstreams() > 1) || depSliceSegmentsEnabled) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
		{
			// We'll sync if the TR is available.
			TComDataCU *pcCUUp = rpcPic->getCU( uiCUAddr )->getCUAbove();
			UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
			UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
			TComDataCU *pcCUTR = NULL;
			if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
			{
				pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
			}
			if ( (true/*bEnforceSliceRestriction*/ &&
				((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
				(pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
				((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
				))
				)
			{
				// TR not available.
			}
			else
			{
				// TR is available, we use it.
				pcSbacCoders[uiSubStrm].loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
			}
		}
		m_pcSbacCoder->load(&pcSbacCoders[uiSubStrm]);  //this load is used to simplify the code (avoid to change all the call to m_pcSbacCoder)

		// reset the entropy coder
		if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
			uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
			uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
			uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
		{
			{
				// We're crossing into another tile, tiles are independent.
				// When tiles are independent, we have "substreams per tile".  Each substream has already been terminated, and we no longer
				// have to perform it here.
				if (pcSlice->getPPS()->getNumSubstreams() > 1)
				{
					; // do nothing.
				}
				else
				{
					SliceType sliceType  = pcSlice->getSliceType();
					if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
					{
						sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
					}
					m_pcEntropyCoder->updateContextTables( sliceType, pcSlice->getSliceQp() );
					// Byte-alignment in slice_data() when new tile
					pcSubstreams[uiSubStrm].writeByteAlignment();
				}
			}
			{
				UInt numStartCodeEmulations = pcSubstreams[uiSubStrm].countStartCodeEmulations();
				UInt uiAccumulatedSubstreamLength = 0;
				for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
				{
					uiAccumulatedSubstreamLength += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
				}
				// add bits coded in previous dependent slices + bits coded so far
				// add number of emulation prevention byte count in the tile
				pcSlice->addTileLocation( ((pcSlice->getTileOffstForMultES() + uiAccumulatedSubstreamLength - uiBitsOriginallyInSubstreams) >> 3) + numStartCodeEmulations );
			}
		}

		TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );    
		if ( pcSlice->getSPS()->getUseSAO() )
		{
			if (pcSlice->getSaoEnabledFlag()||pcSlice->getSaoEnabledFlagChroma())
			{
				SAOBlkParam& saoblkParam = (rpcPic->getPicSym()->getSAOBlkParam())[uiCUAddr];
				Bool sliceEnabled[NUM_SAO_COMPONENTS];
				sliceEnabled[SAO_Y] = pcSlice->getSaoEnabledFlag();
				sliceEnabled[SAO_Cb]= sliceEnabled[SAO_Cr]= pcSlice->getSaoEnabledFlagChroma();

				Bool leftMergeAvail = false;
				Bool aboveMergeAvail= false;
				//merge left condition
				Int rx = (uiCUAddr % uiWidthInLCUs);
				if(rx > 0)
				{
					leftMergeAvail = rpcPic->getSAOMergeAvailability(uiCUAddr, uiCUAddr-1);
				}

				//merge up condition
				Int ry = (uiCUAddr / uiWidthInLCUs);
				if(ry > 0)
				{
					aboveMergeAvail = rpcPic->getSAOMergeAvailability(uiCUAddr, uiCUAddr-uiWidthInLCUs);
				}

				m_pcEntropyCoder->encodeSAOBlkParam(saoblkParam,sliceEnabled, leftMergeAvail, aboveMergeAvail);
			}
		}

#if ENC_DEC_TRACE
		g_bJustDoIt = g_bEncDecTraceEnable;
#endif
		if ( (m_pcCfg->getSliceMode()!=0 || m_pcCfg->getSliceSegmentMode()!=0) &&
			uiCUAddr == rpcPic->getPicSym()->getCUOrderMap((uiBoundingCUAddr+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU()-1) )
		{
			m_pcCuEncoder->encodeCU( pcCU );
		}
		else
		{
			m_pcCuEncoder->encodeCU( pcCU );
		}
#if ENC_DEC_TRACE
		g_bJustDoIt = g_bEncDecTraceDisable;
#endif    
		pcSbacCoders[uiSubStrm].load(m_pcSbacCoder);   //load back status of the entropy coder after encoding the LCU into relevant bitstream entropy coder
		//Store probabilties of second LCU in line into buffer
		if ( (depSliceSegmentsEnabled || (pcSlice->getPPS()->getNumSubstreams() > 1)) && (uiCol == uiTileLCUX+1) && m_pcCfg->getWaveFrontsynchro())
		{
			m_pcBufferSbacCoders[uiTileCol].loadContexts( &pcSbacCoders[uiSubStrm] );
		}
	}
	if( depSliceSegmentsEnabled )
	{
		if (m_pcCfg->getWaveFrontsynchro())
		{
			CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
		}
		CTXMem[0]->loadContexts( m_pcSbacCoder );//ctx end of dep.slice
	}
#if ADAPTIVE_QP_SELECTION
	if( m_pcCfg->getUseAdaptQpSelect() )
	{
		m_pcTrQuant->storeSliceQpNext(pcSlice);
	}
#endif
	if (pcSlice->getPPS()->getCabacInitPresentFlag())
	{
		if  (pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
		{
			pcSlice->getPPS()->setEncCABACTableIdx( pcSlice->getSliceType() );
		}
		else
		{
			m_pcEntropyCoder->determineCabacInitIdx();
		}
	}
}

/** Determines the starting and bounding LCU address of current slice / dependent slice
 * \param bEncodeSlice Identifies if the calling function is compressSlice() [false] or encodeSlice() [true]
 * \returns Updates uiStartCUAddr, uiBoundingCUAddr with appropriate LCU address
 */
Void TEncSlice::xDetermineStartAndBoundingCUAddr  ( UInt& startCUAddr, UInt& boundingCUAddr, TComPic*& rpcPic, Bool bEncodeSlice )
{
	TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());
	UInt uiStartCUAddrSlice, uiBoundingCUAddrSlice;
	UInt tileIdxIncrement;
	UInt tileIdx;
	UInt tileWidthInLcu;
	UInt tileHeightInLcu;
	UInt tileTotalCount;

	uiStartCUAddrSlice        = pcSlice->getSliceCurStartCUAddr();
	UInt uiNumberOfCUsInFrame = rpcPic->getNumCUsInFrame();
	uiBoundingCUAddrSlice     = uiNumberOfCUsInFrame;
	if (bEncodeSlice) 
	{
		UInt uiCUAddrIncrement;
		switch (m_pcCfg->getSliceMode())
		{
		case FIXED_NUMBER_OF_LCU:
			uiCUAddrIncrement        = m_pcCfg->getSliceArgument();
			uiBoundingCUAddrSlice    = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU()) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		case FIXED_NUMBER_OF_BYTES:
			uiCUAddrIncrement        = rpcPic->getNumCUsInFrame();
			uiBoundingCUAddrSlice    = pcSlice->getSliceCurEndCUAddr();
			break;
		case FIXED_NUMBER_OF_TILES:
			tileIdx                = rpcPic->getPicSym()->getTileIdxMap(
				rpcPic->getPicSym()->getCUOrderMap(uiStartCUAddrSlice/rpcPic->getNumPartInCU())
				);
			uiCUAddrIncrement        = 0;
			tileTotalCount         = (rpcPic->getPicSym()->getNumColumnsMinus1()+1) * (rpcPic->getPicSym()->getNumRowsMinus1()+1);

			for(tileIdxIncrement = 0; tileIdxIncrement < m_pcCfg->getSliceArgument(); tileIdxIncrement++)
			{
				if((tileIdx + tileIdxIncrement) < tileTotalCount)
				{
					tileWidthInLcu   = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidth();
					tileHeightInLcu  = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeight();
					uiCUAddrIncrement += (tileWidthInLcu * tileHeightInLcu * rpcPic->getNumPartInCU());
				}
			}

			uiBoundingCUAddrSlice    = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU()) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		default:
			uiCUAddrIncrement        = rpcPic->getNumCUsInFrame();
			uiBoundingCUAddrSlice    = uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		} 
		// WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
		if (pcSlice->getPPS()->getNumSubstreams() > 1 && (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()) != 0))
		{
			uiBoundingCUAddrSlice = min(uiBoundingCUAddrSlice, uiStartCUAddrSlice - (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU())) + (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()));
		}
		pcSlice->setSliceCurEndCUAddr( uiBoundingCUAddrSlice );
	}
	else
	{
		UInt uiCUAddrIncrement     ;
		switch (m_pcCfg->getSliceMode())
		{
		case FIXED_NUMBER_OF_LCU:
			uiCUAddrIncrement        = m_pcCfg->getSliceArgument();
			uiBoundingCUAddrSlice    = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU()) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		case FIXED_NUMBER_OF_TILES:
			tileIdx                = rpcPic->getPicSym()->getTileIdxMap(
				rpcPic->getPicSym()->getCUOrderMap(uiStartCUAddrSlice/rpcPic->getNumPartInCU())
				);
			uiCUAddrIncrement        = 0;
			tileTotalCount         = (rpcPic->getPicSym()->getNumColumnsMinus1()+1) * (rpcPic->getPicSym()->getNumRowsMinus1()+1);

			for(tileIdxIncrement = 0; tileIdxIncrement < m_pcCfg->getSliceArgument(); tileIdxIncrement++)
			{
				if((tileIdx + tileIdxIncrement) < tileTotalCount)
				{
					tileWidthInLcu   = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidth();
					tileHeightInLcu  = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeight();
					uiCUAddrIncrement += (tileWidthInLcu * tileHeightInLcu * rpcPic->getNumPartInCU());
				}
			}

			uiBoundingCUAddrSlice    = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU()) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		default:
			uiCUAddrIncrement        = rpcPic->getNumCUsInFrame();
			uiBoundingCUAddrSlice    = uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		} 
		// WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
		if (pcSlice->getPPS()->getNumSubstreams() > 1 && (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()) != 0))
		{
			uiBoundingCUAddrSlice = min(uiBoundingCUAddrSlice, uiStartCUAddrSlice - (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU())) + (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()));
		}
		pcSlice->setSliceCurEndCUAddr( uiBoundingCUAddrSlice );
	}

	Bool tileBoundary = false;
	if ((m_pcCfg->getSliceMode() == FIXED_NUMBER_OF_LCU || m_pcCfg->getSliceMode() == FIXED_NUMBER_OF_BYTES) && 
		(m_pcCfg->getNumRowsMinus1() > 0 || m_pcCfg->getNumColumnsMinus1() > 0))
	{
		UInt lcuEncAddr = (uiStartCUAddrSlice+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU();
		UInt lcuAddr = rpcPic->getPicSym()->getCUOrderMap(lcuEncAddr);
		UInt startTileIdx = rpcPic->getPicSym()->getTileIdxMap(lcuAddr);
		UInt tileBoundingCUAddrSlice = 0;
		while (lcuEncAddr < uiNumberOfCUsInFrame && rpcPic->getPicSym()->getTileIdxMap(lcuAddr) == startTileIdx)
		{
			lcuEncAddr++;
			lcuAddr = rpcPic->getPicSym()->getCUOrderMap(lcuEncAddr);
		}
		tileBoundingCUAddrSlice = lcuEncAddr*rpcPic->getNumPartInCU();
    
		if (tileBoundingCUAddrSlice < uiBoundingCUAddrSlice)
		{
			uiBoundingCUAddrSlice = tileBoundingCUAddrSlice;
			pcSlice->setSliceCurEndCUAddr( uiBoundingCUAddrSlice );
			tileBoundary = true;
		}
	}

	// Dependent slice
	UInt startCUAddrSliceSegment, boundingCUAddrSliceSegment;
	startCUAddrSliceSegment    = pcSlice->getSliceSegmentCurStartCUAddr();
	boundingCUAddrSliceSegment = uiNumberOfCUsInFrame;
	if (bEncodeSlice) 
	{
		UInt uiCUAddrIncrement;
		switch (m_pcCfg->getSliceSegmentMode())
		{
		case FIXED_NUMBER_OF_LCU:
			uiCUAddrIncrement               = m_pcCfg->getSliceSegmentArgument();
			boundingCUAddrSliceSegment    = ((startCUAddrSliceSegment + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU() ) ? (startCUAddrSliceSegment + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		case FIXED_NUMBER_OF_BYTES:
			uiCUAddrIncrement               = rpcPic->getNumCUsInFrame();
			boundingCUAddrSliceSegment    = pcSlice->getSliceSegmentCurEndCUAddr();
			break;
		case FIXED_NUMBER_OF_TILES:
			tileIdx                = rpcPic->getPicSym()->getTileIdxMap(
				rpcPic->getPicSym()->getCUOrderMap(pcSlice->getSliceSegmentCurStartCUAddr()/rpcPic->getNumPartInCU())
				);
			uiCUAddrIncrement        = 0;
			tileTotalCount         = (rpcPic->getPicSym()->getNumColumnsMinus1()+1) * (rpcPic->getPicSym()->getNumRowsMinus1()+1);

			for(tileIdxIncrement = 0; tileIdxIncrement < m_pcCfg->getSliceSegmentArgument(); tileIdxIncrement++)
			{
				if((tileIdx + tileIdxIncrement) < tileTotalCount)
				{
					tileWidthInLcu   = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidth();
					tileHeightInLcu  = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeight();
					uiCUAddrIncrement += (tileWidthInLcu * tileHeightInLcu * rpcPic->getNumPartInCU());
				}
			}
			boundingCUAddrSliceSegment    = ((startCUAddrSliceSegment + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU() ) ? (startCUAddrSliceSegment + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		default:
			uiCUAddrIncrement               = rpcPic->getNumCUsInFrame();
			boundingCUAddrSliceSegment    = uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		} 
		// WPP: if a slice segment does not start at the beginning of a CTB row, it must end within the same CTB row
		if (pcSlice->getPPS()->getNumSubstreams() > 1 && (startCUAddrSliceSegment % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()) != 0))
		{
			boundingCUAddrSliceSegment = min(boundingCUAddrSliceSegment, startCUAddrSliceSegment - (startCUAddrSliceSegment % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU())) + (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()));
		}
		pcSlice->setSliceSegmentCurEndCUAddr( boundingCUAddrSliceSegment );
	}
	else
	{
		UInt uiCUAddrIncrement;
		switch (m_pcCfg->getSliceSegmentMode())
		{
		case FIXED_NUMBER_OF_LCU:
			uiCUAddrIncrement               = m_pcCfg->getSliceSegmentArgument();
			boundingCUAddrSliceSegment    = ((startCUAddrSliceSegment + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU() ) ? (startCUAddrSliceSegment + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		case FIXED_NUMBER_OF_TILES:
			tileIdx                = rpcPic->getPicSym()->getTileIdxMap(
				rpcPic->getPicSym()->getCUOrderMap(pcSlice->getSliceSegmentCurStartCUAddr()/rpcPic->getNumPartInCU())
				);
			uiCUAddrIncrement        = 0;
			tileTotalCount         = (rpcPic->getPicSym()->getNumColumnsMinus1()+1) * (rpcPic->getPicSym()->getNumRowsMinus1()+1);

			for(tileIdxIncrement = 0; tileIdxIncrement < m_pcCfg->getSliceSegmentArgument(); tileIdxIncrement++)
			{
				if((tileIdx + tileIdxIncrement) < tileTotalCount)
				{
					tileWidthInLcu   = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidth();
					tileHeightInLcu  = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeight();
					uiCUAddrIncrement += (tileWidthInLcu * tileHeightInLcu * rpcPic->getNumPartInCU());
				}
			}
			boundingCUAddrSliceSegment    = ((startCUAddrSliceSegment + uiCUAddrIncrement) < uiNumberOfCUsInFrame*rpcPic->getNumPartInCU() ) ? (startCUAddrSliceSegment + uiCUAddrIncrement) : uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		default:
			uiCUAddrIncrement               = rpcPic->getNumCUsInFrame();
			boundingCUAddrSliceSegment    = uiNumberOfCUsInFrame*rpcPic->getNumPartInCU();
			break;
		} 
		// WPP: if a slice segment does not start at the beginning of a CTB row, it must end within the same CTB row
		if (pcSlice->getPPS()->getNumSubstreams() > 1 && (startCUAddrSliceSegment % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()) != 0))
		{
			boundingCUAddrSliceSegment = min(boundingCUAddrSliceSegment, startCUAddrSliceSegment - (startCUAddrSliceSegment % (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU())) + (rpcPic->getFrameWidthInCU()*rpcPic->getNumPartInCU()));
		}
		pcSlice->setSliceSegmentCurEndCUAddr( boundingCUAddrSliceSegment );
	}
	if ((m_pcCfg->getSliceSegmentMode() == FIXED_NUMBER_OF_LCU || m_pcCfg->getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES) && 
		(m_pcCfg->getNumRowsMinus1() > 0 || m_pcCfg->getNumColumnsMinus1() > 0))
	{
		UInt lcuEncAddr = (startCUAddrSliceSegment+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU();
		UInt lcuAddr = rpcPic->getPicSym()->getCUOrderMap(lcuEncAddr);
		UInt startTileIdx = rpcPic->getPicSym()->getTileIdxMap(lcuAddr);
		UInt tileBoundingCUAddrSlice = 0;
		while (lcuEncAddr < uiNumberOfCUsInFrame && rpcPic->getPicSym()->getTileIdxMap(lcuAddr) == startTileIdx)
		{
			lcuEncAddr++;
			lcuAddr = rpcPic->getPicSym()->getCUOrderMap(lcuEncAddr);
		}
		tileBoundingCUAddrSlice = lcuEncAddr*rpcPic->getNumPartInCU();

		if (tileBoundingCUAddrSlice < boundingCUAddrSliceSegment)
		{
			boundingCUAddrSliceSegment = tileBoundingCUAddrSlice;
			pcSlice->setSliceSegmentCurEndCUAddr( boundingCUAddrSliceSegment );
			tileBoundary = true;
		}
	}

	if(boundingCUAddrSliceSegment>uiBoundingCUAddrSlice)
	{
		boundingCUAddrSliceSegment = uiBoundingCUAddrSlice;
		pcSlice->setSliceSegmentCurEndCUAddr(uiBoundingCUAddrSlice);
	}

	//calculate real dependent slice start address
	UInt uiInternalAddress = rpcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurStartCUAddr()) % rpcPic->getNumPartInCU();
	UInt uiExternalAddress = rpcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurStartCUAddr()) / rpcPic->getNumPartInCU();
	UInt uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
	UInt uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
	UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
	UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
	while((uiPosX>=uiWidth||uiPosY>=uiHeight)&&!(uiPosX>=uiWidth&&uiPosY>=uiHeight))
	{
		uiInternalAddress++;
		if(uiInternalAddress>=rpcPic->getNumPartInCU())
		{
			uiInternalAddress=0;
			uiExternalAddress = rpcPic->getPicSym()->getCUOrderMap(rpcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
		}
		uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
		uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
	}
	UInt uiRealStartAddress = rpcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*rpcPic->getNumPartInCU()+uiInternalAddress);
  
	pcSlice->setSliceSegmentCurStartCUAddr(uiRealStartAddress);
	startCUAddrSliceSegment=uiRealStartAddress;
  
	//calculate real slice start address
	uiInternalAddress = rpcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceCurStartCUAddr()) % rpcPic->getNumPartInCU();
	uiExternalAddress = rpcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceCurStartCUAddr()) / rpcPic->getNumPartInCU();
	uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
	uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
	uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
	uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
	while((uiPosX>=uiWidth||uiPosY>=uiHeight)&&!(uiPosX>=uiWidth&&uiPosY>=uiHeight))
	{
		uiInternalAddress++;
		if(uiInternalAddress>=rpcPic->getNumPartInCU())
		{
			uiInternalAddress=0;
			uiExternalAddress = rpcPic->getPicSym()->getCUOrderMap(rpcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
		}
		uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
		uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
	}
	uiRealStartAddress = rpcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*rpcPic->getNumPartInCU()+uiInternalAddress);
  
	pcSlice->setSliceCurStartCUAddr(uiRealStartAddress);
	uiStartCUAddrSlice=uiRealStartAddress;
  
	// Make a joint decision based on reconstruction and dependent slice bounds
	startCUAddr    = max(uiStartCUAddrSlice   , startCUAddrSliceSegment   );
	boundingCUAddr = min(uiBoundingCUAddrSlice, boundingCUAddrSliceSegment);


	if (!bEncodeSlice)
	{
		// For fixed number of LCU within an entropy and reconstruction slice we already know whether we will encounter end of entropy and/or reconstruction slice
		// first. Set the flags accordingly.
		if ( (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
			|| (m_pcCfg->getSliceMode()==0 && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
			|| (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU && m_pcCfg->getSliceSegmentMode()==0) 
			|| (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
			|| (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceSegmentMode()==0) 
			|| (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceMode()==0)
			|| tileBoundary
)
		{
			if (uiBoundingCUAddrSlice < boundingCUAddrSliceSegment)
			{
				pcSlice->setNextSlice       ( true );
				pcSlice->setNextSliceSegment( false );
			}
			else if (uiBoundingCUAddrSlice > boundingCUAddrSliceSegment)
			{
				pcSlice->setNextSlice       ( false );
				pcSlice->setNextSliceSegment( true );
			}
			else
			{
				pcSlice->setNextSlice       ( true );
				pcSlice->setNextSliceSegment( true );
			}
		}
		else
		{
			pcSlice->setNextSlice       ( false );
			pcSlice->setNextSliceSegment( false );
		}
	}
}

Double TEncSlice::xGetQPValueAccordingToLambda ( Double lambda )
{
	return 4.2005*log(lambda) + 13.7122;
}



#if ETRI_MODIFICATION_V00 
Void TEncSlice::compressSlice( TComPic*& rpcPic ){return ;}		/// 2015 5 14 by Seok : Peudo Function for ETRI_compressSlice when ETRI_MODIFICATION_V00 is active
#else
Void TEncSlice::compressSlice( TComPic*& rpcPic )
{
	UInt  uiCUAddr;
	UInt   uiStartCUAddr;
	UInt   uiBoundingCUAddr;
	rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
	TEncBinCABAC* pppcRDSbacCoder = NULL;
	TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
	xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
  
	// initialize cost values
	m_uiPicTotalBits  = 0;
	m_dPicRdCost      = 0;
	m_uiPicDist       = 0;
  
	// set entropy coder
	m_pcSbacCoder->init( m_pcBinCABAC );
	m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
	m_pcEntropyCoder->resetEntropy      ();
	m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
	pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
	pppcRDSbacCoder->setBinCountingEnableFlag( false );
	pppcRDSbacCoder->setBinsCoded( 0 );
  
	//------------------------------------------------------------------------------
	//  Weighted Prediction parameters estimation.
	//------------------------------------------------------------------------------
	// calculate AC/DC values for current picture
	if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
	{
		xCalcACDCParamSlice(pcSlice);
	}

	Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

	if ( bWp_explicit )
	{
		//------------------------------------------------------------------------------
		//  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
		//------------------------------------------------------------------------------
		if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
		{
			printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
		}

		xEstimateWPParamSlice( pcSlice );
		pcSlice->initWpScaling();

		// check WP on/off
		xCheckWPEnable( pcSlice );
	}

#if ADAPTIVE_QP_SELECTION
	if( m_pcCfg->getUseAdaptQpSelect() )
	{
		m_pcTrQuant->clearSliceARLCnt();
		if(pcSlice->getSliceType()!=I_SLICE)
		{
			Int qpBase = pcSlice->getSliceQpBase();
			pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
		}
	}
#endif
	TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
	TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
	TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
	Int  iNumSubstreams = 1;
	UInt uiTilesAcross  = 0;

	iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
	uiTilesAcross = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
	delete[] m_pcBufferSbacCoders;
	delete[] m_pcBufferBinCoderCABACs;
	m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
	m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
	for (Int ui = 0; ui < uiTilesAcross; ui++)
	{
		m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
	}
	for (UInt ui = 0; ui < uiTilesAcross; ui++)
	{
		m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
	}

	for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
	{
		ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
	}

	delete[] m_pcBufferLowLatSbacCoders;
	delete[] m_pcBufferLowLatBinCoderCABACs;
	m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
	m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
	for (Int ui = 0; ui < uiTilesAcross; ui++)
	{
		m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
	}
	for (UInt ui = 0; ui < uiTilesAcross; ui++)
		m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state

	UInt uiWidthInLCUs  = rpcPic->getPicSym()->getFrameWidthInCU();
	//UInt uiHeightInLCUs = rpcPic->getPicSym()->getFrameHeightInCU();
	UInt uiCol=0, uiLin=0, uiSubStrm=0;
	UInt uiTileCol      = 0;
	UInt uiTileStartLCU = 0;
	UInt uiTileLCUX     = 0;
	Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
	uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
	uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
	if( depSliceSegmentsEnabled )
	{
		if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
		{
			if( m_pcCfg->getWaveFrontsynchro() )
			{
				uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
				m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
				Int iNumSubstreamsPerTile = iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
				uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU()); 
				uiLin     = uiCUAddr / uiWidthInLCUs;
				uiSubStrm = rpcPic->getPicSym()->getTileIdxMap(rpcPic->getPicSym()->getCUOrderMap(uiCUAddr))*iNumSubstreamsPerTile
					+ uiLin%iNumSubstreamsPerTile;
				if ( (uiCUAddr%uiWidthInLCUs+1) >= uiWidthInLCUs  )
				{
					uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
					uiCol     = uiCUAddr % uiWidthInLCUs;
					if(uiCol==uiTileStartLCU)
					{
						CTXMem[0]->loadContexts(m_pcSbacCoder);
					}
				}
			}
			m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
			ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
		}
		else
		{
			if(m_pcCfg->getWaveFrontsynchro())
			{
				CTXMem[1]->loadContexts(m_pcSbacCoder);
			}
			CTXMem[0]->loadContexts(m_pcSbacCoder);
		}
	}
	// for every CU in slice
	UInt uiEncCUOrder;
	for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
		uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
		uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
	{
		// initialize CU encoder
		TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
		pcCU->initCU( rpcPic, uiCUAddr );

		// inherit from TR if necessary, select substream to use.
		uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
		uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
		uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
		//UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
		uiCol     = uiCUAddr % uiWidthInLCUs;
		uiLin     = uiCUAddr / uiWidthInLCUs;
		if (pcSlice->getPPS()->getNumSubstreams() > 1)
		{
			// independent tiles => substreams are "per tile".  iNumSubstreams has already been multiplied.
			Int iNumSubstreamsPerTile = iNumSubstreams/rpcPic->getPicSym()->getNumTiles();
			uiSubStrm = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)*iNumSubstreamsPerTile
				+ uiLin%iNumSubstreamsPerTile;
		}
		else
		{
			// dependent tiles => substreams are "per frame".
			uiSubStrm = uiLin % iNumSubstreams;
		}
		if ( ((pcSlice->getPPS()->getNumSubstreams() > 1) || depSliceSegmentsEnabled ) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
		{
			// We'll sync if the TR is available.
			TComDataCU *pcCUUp = pcCU->getCUAbove();
			UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
			UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
			TComDataCU *pcCUTR = NULL;
			if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
			{
				pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
			}
			if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
				(pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
				((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
				)
				)
			{
				// TR not available.
			}
			else
			{
				// TR is available, we use it.
				ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
			}
		}
		m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code

		// reset the entropy coder
		if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
			uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
			uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
			uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
		{
			SliceType sliceType = pcSlice->getSliceType();
			if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
			{
				sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
			}
			m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
			m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
			m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
			m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
		}

		// set go-on entropy coder
		m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
		m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );

		((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);

		Double oldLambda = m_pcRdCost->getLambda();
		if ( m_pcCfg->getUseRateCtrl() )
		{
			Int estQP        = pcSlice->getSliceQp();
			Double estLambda = -1.0;
			Double bpp       = -1.0;

			if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
			{
				estQP = pcSlice->getSliceQp();
			}
			else
			{
				bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
				if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
				{
					estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
				}
				else
				{
					estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
					estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
				}

				estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffsetY(), MAX_QP, estQP );

				m_pcRdCost->setLambda(estLambda);
#if RDOQ_CHROMA_LAMBDA
// set lambda for RDOQ
				Double weight=m_pcRdCost->getChromaWeight();
				const Double lambdaArray[3] = { estLambda, (estLambda / weight), (estLambda / weight) };
				m_pcTrQuant->setLambdas( lambdaArray );
#else
				m_pcTrQuant->setLambda( estLambda );
#endif
			}

			m_pcRateCtrl->setRCQP( estQP );
#if ADAPTIVE_QP_SELECTION
			pcCU->getSlice()->setSliceQpBase( estQP );
#endif
		}

		// run CU encoder
		m_pcCuEncoder->compressCU( pcCU );

		// restore entropy coder to an initial stage
		m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
		m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
		m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
		m_pcBitCounter = &pcBitCounters[uiSubStrm];
		pppcRDSbacCoder->setBinCountingEnableFlag( true );
		m_pcBitCounter->resetBits();
		pppcRDSbacCoder->setBinsCoded( 0 );
		m_pcCuEncoder->encodeCU( pcCU );

		pppcRDSbacCoder->setBinCountingEnableFlag( false );
		if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
		{
			pcSlice->setNextSlice( true );
			break;
		}
		if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
		{
			pcSlice->setNextSliceSegment( true );
			break;
		}

		ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
		//Store probabilties of second LCU in line into buffer
		if ( ( uiCol == uiTileLCUX+1) && (depSliceSegmentsEnabled || (pcSlice->getPPS()->getNumSubstreams() > 1)) && m_pcCfg->getWaveFrontsynchro())
		{
			m_pcBufferSbacCoders[uiTileCol].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
		}

		if ( m_pcCfg->getUseRateCtrl() )
		{

			Int actualQP        = g_RCInvalidQPValue;
			Double actualLambda = m_pcRdCost->getLambda();
			Int actualBits      = pcCU->getTotalBits();
			Int numberOfEffectivePixels    = 0;
			for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
			{
				if ( pcCU->getPredictionMode( idx ) != MODE_NONE && ( !pcCU->isSkipped( idx ) ) )
				{
					numberOfEffectivePixels = numberOfEffectivePixels + 16;
					break;
				}
			}

			if ( numberOfEffectivePixels == 0 )
			{
				actualQP = g_RCInvalidQPValue;
			}
			else
			{
				actualQP = pcCU->getQP( 0 );
			}
			m_pcRdCost->setLambda(oldLambda);

			m_pcRateCtrl->getRCPic()->updateAfterLCU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
				pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
		}
    
		m_uiPicTotalBits += pcCU->getTotalBits();
		m_dPicRdCost     += pcCU->getTotalCost();
		m_uiPicDist      += pcCU->getTotalDistortion();
	}
	if ((pcSlice->getPPS()->getNumSubstreams() > 1) && !depSliceSegmentsEnabled)
	{
		pcSlice->setNextSlice( true );
	}
	if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
	{
		if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
		{
			pcSlice->setNextSlice( true );
		}
		else
		{
			pcSlice->setNextSliceSegment( true );
		}
	}
	if( depSliceSegmentsEnabled )
	{
		if (m_pcCfg->getWaveFrontsynchro())
		{
			CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
		}
		CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
	}
	xRestoreWPparam( pcSlice );
}



#endif	/// 2015 5 14 by Seok : #if !ETRI_MODIFICATION_V00
#if ETRI_MULTITHREAD_2
void TEncSlice::threadProcessingTile(void *param, int num)
{
	TEncSlice *pcSlice = (TEncSlice *)param;
	pcSlice->em_pcTileEncoder[num].ETRI_CompressUnit();
}
#endif
//! \}

#if ETRI_THREADPOOL_OPT
#if ETRI_THREAD_LOAD_BALANCING
EncTileJob* EncTileJob::createJob(void *param, EncTileInfo info, TileLoadBalanceInfo lbInfo)
{
	EncTileJob *job = new EncTileJob(param, info, lbInfo);
	return job;
}
#else
EncTileJob* EncTileJob::createJob(void *param, EncTileInfo info)
{
	EncTileJob *job = new EncTileJob(param, info);
	return job;
}
#endif

#if ETRI_THREAD_LOAD_BALANCING

EncTileJob::EncTileJob(void *param, EncTileInfo info, TileLoadBalanceInfo lbInfo)
{
	m_encInfo = info;
	m_lbInfo = lbInfo;
	m_encSlice = (TEncSlice *)param;
}

#else
EncTileJob::EncTileJob(void *param, EncTileInfo info)
{
	m_encInfo = info;
	m_encSlice = (TEncSlice *)param;
}
#endif
void EncTileJob::run(void *)
{
	EncTileInfo info = m_encInfo;

#if ETRI_THREAD_LOAD_BALANCING
	if (info.nTile == 16)
		m_encSlice->em_pcTileEncoder[info.id].ETRI_CompressUnit(m_lbInfo);
	else
		m_encSlice->em_pcTileEncoder[info.id].ETRI_CompressUnit();
#else
	m_encSlice->em_pcTileEncoder[info.id].ETRI_CompressUnit();
#endif

#if ETRI_TILE_THEAD_OPT
	pthread_mutex_lock(info.mutex);
	if ((*(info.endCount))++ == info.nTile - 1)
	{
		pthread_cond_broadcast(info.cond);
	}
		pthread_mutex_unlock(info.mutex);
#else
	pthread_mutex_lock(info.mutex);
	(*(info.endCount))++;
	pthread_mutex_unlock(info.mutex);
#endif
}
#endif