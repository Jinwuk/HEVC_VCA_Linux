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

/** \file     TAppEncTop.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>	// gplusplus

#include "TLibEncoder/TEncTop.h"

#include "TAppEncTop.h"
#include "TLibEncoder/AnnexBwrite.h"

#if !(_ETRI_WINDOWS_APPLICATION)
#include <unistd.h>
#endif

using namespace std;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppEncTop::TAppEncTop()
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;

  ETRI_Service_Init(0);		///< ETRI's Initial Function 2015 5 22 by Seok
}

TAppEncTop::~TAppEncTop()
{
}

Void TAppEncTop::xInitLibCfg()
{
  TComVPS vps;
  
  vps.setMaxTLayers                       ( m_maxTempLayer );
  if (m_maxTempLayer == 1)
  {
    vps.setTemporalNestingFlag(true);
  }
  vps.setMaxLayers                        ( 1 );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setNumReorderPics                 ( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering             ( m_maxDecPicBuffering[i], i );
  }
  m_cTEncTop.setVPS(&vps);

  m_cTEncTop.setProfile(m_profile);
  m_cTEncTop.setLevel(m_levelTier, m_level);
  m_cTEncTop.setProgressiveSourceFlag(m_progressiveSourceFlag);
  m_cTEncTop.setInterlacedSourceFlag(m_interlacedSourceFlag);
  m_cTEncTop.setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
  m_cTEncTop.setFrameOnlyConstraintFlag(m_frameOnlyConstraintFlag);
  
  m_cTEncTop.setFrameRate                    ( m_iFrameRate );
  m_cTEncTop.setFrameRateF					 ( m_fFrameRate ); // wsseo@2015-08-24. fix fps
  m_cTEncTop.setFrameSkip                    ( m_FrameSkip );
  m_cTEncTop.setSourceWidth                  ( m_iSourceWidth );
  m_cTEncTop.setSourceHeight                 ( m_iSourceHeight );
  m_cTEncTop.setConformanceWindow            ( m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
  m_cTEncTop.setFramesToBeEncoded            ( m_framesToBeEncoded );
  
  //====== Coding Structure ========
  m_cTEncTop.setIntraPeriod                  ( m_iIntraPeriod );
  m_cTEncTop.setDecodingRefreshType          ( m_iDecodingRefreshType );
  m_cTEncTop.setGOPSize                      ( m_iGOPSize );
  m_cTEncTop.setGopList                      ( m_GOPList );
  m_cTEncTop.setExtraRPSs                    ( m_extraRPSs );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    m_cTEncTop.setNumReorderPics             ( m_numReorderPics[i], i );
    m_cTEncTop.setMaxDecPicBuffering         ( m_maxDecPicBuffering[i], i );
  }
  for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cTEncTop.setLambdaModifier( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cTEncTop.setQP                           ( m_iQP );
  
  m_cTEncTop.setPad                          ( m_aiPad );
    
  m_cTEncTop.setMaxTempLayer                 ( m_maxTempLayer );
  m_cTEncTop.setUseAMP( m_enableAMP );
  
  //===== Slice ========
  
  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable            ( m_bLoopFilterDisable       );
  m_cTEncTop.setLoopFilterOffsetInPPS        ( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset         ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTop.setLoopFilterTcOffset           ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTop.setDeblockingFilterControlPresent( m_DeblockingFilterControlPresent);
  m_cTEncTop.setDeblockingFilterMetric       ( m_DeblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTop.setFastSearch                   ( m_iFastSearch  );
  m_cTEncTop.setSearchRange                  ( m_iSearchRange );
  m_cTEncTop.setBipredSearchRange            ( m_bipredSearchRange );

  //====== Quality control ========
  m_cTEncTop.setMaxDeltaQP                   ( m_iMaxDeltaQP  );
  m_cTEncTop.setMaxCuDQPDepth                ( m_iMaxCuDQPDepth  );

  m_cTEncTop.setChromaCbQpOffset               ( m_cbQpOffset     );
  m_cTEncTop.setChromaCrQpOffset            ( m_crQpOffset  );

#if ADAPTIVE_QP_SELECTION
  m_cTEncTop.setUseAdaptQpSelect             ( m_bUseAdaptQpSelect   );
#endif

  m_cTEncTop.setUseAdaptiveQP                ( m_bUseAdaptiveQP  );
  m_cTEncTop.setQPAdaptationRange            ( m_iQPAdaptationRange );
  
  //====== Tool list ========
  m_cTEncTop.setDeltaQpRD                    ( m_uiDeltaQpRD  );
  m_cTEncTop.setUseASR                       ( m_bUseASR      );
  m_cTEncTop.setUseHADME                     ( m_bUseHADME    );
  m_cTEncTop.setdQPs                         ( m_aidQP        );
  m_cTEncTop.setUseRDOQ                      ( m_useRDOQ     );
  m_cTEncTop.setUseRDOQTS                    ( m_useRDOQTS   );
  m_cTEncTop.setRDpenalty                 ( m_rdPenalty );
  m_cTEncTop.setQuadtreeTULog2MaxSize        ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize        ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter      ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra      ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setUseFastEnc                   ( m_bUseFastEnc  );
  m_cTEncTop.setUseEarlyCU                   ( m_bUseEarlyCU  ); 
  m_cTEncTop.setUseFastDecisionForMerge      ( m_useFastDecisionForMerge  );
  m_cTEncTop.setUseCbfFastMode            ( m_bUseCbfFastMode  );
  m_cTEncTop.setUseEarlySkipDetection            ( m_useEarlySkipDetection );

  m_cTEncTop.setUseTransformSkip             ( m_useTransformSkip      );
  m_cTEncTop.setUseTransformSkipFast         ( m_useTransformSkipFast  );
  m_cTEncTop.setUseConstrainedIntraPred      ( m_bUseConstrainedIntraPred );
  m_cTEncTop.setPCMLog2MinSize          ( m_uiPCMLog2MinSize);
  m_cTEncTop.setUsePCM                       ( m_usePCM );
  m_cTEncTop.setPCMLog2MaxSize               ( m_pcmLog2MaxSize);
  m_cTEncTop.setMaxNumMergeCand              ( m_maxNumMergeCand );

  // [HDR/WCG Encoding] VUI/SEI parameter setting (by Dongsan Jun, 20160124)
#if ETRI_HDR_WCG_ENCODER
  m_cTEncTop.setMasteringDisplaySEI(m_masteringDisplay);
#endif
  

  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP                   ( m_useWeightedPred      );
  m_cTEncTop.setWPBiPred                ( m_useWeightedBiPred   );
  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2 ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode               ( m_sliceMode                );
  m_cTEncTop.setSliceArgument           ( m_sliceArgument            );

  //====== Dependent Slice ========
  m_cTEncTop.setSliceSegmentMode        ( m_sliceSegmentMode         );
  m_cTEncTop.setSliceSegmentArgument    ( m_sliceSegmentArgument     );
  Int iNumPartInCU = 1<<(m_uiMaxCUDepth<<1);
  if(m_sliceSegmentMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceSegmentArgument ( m_sliceSegmentArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceArgument ( m_sliceArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_TILES)
  {
    m_cTEncTop.setSliceArgument ( m_sliceArgument );
  }
  
  if(m_sliceMode == 0 )
  {
#if !ETRI_SliceEncoderHeader 
	  m_bLFCrossSliceBoundaryFlag = true;
#endif   
  }
  m_cTEncTop.setLFCrossSliceBoundaryFlag( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO ( m_bUseSAO );
  m_cTEncTop.setMaxNumOffsetsPerPic (m_maxNumOffsetsPerPic);

  m_cTEncTop.setSaoLcuBoundary (m_saoLcuBoundary);
  m_cTEncTop.setPCMInputBitDepthFlag  ( m_bPCMInputBitDepthFlag); 
  m_cTEncTop.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag); 

  m_cTEncTop.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);
  m_cTEncTop.setRecoveryPointSEIEnabled( m_recoveryPointSEIEnabled );
  m_cTEncTop.setBufferingPeriodSEIEnabled( m_bufferingPeriodSEIEnabled );
  m_cTEncTop.setPictureTimingSEIEnabled( m_pictureTimingSEIEnabled );
  m_cTEncTop.setToneMappingInfoSEIEnabled                 ( m_toneMappingInfoSEIEnabled );
  m_cTEncTop.setTMISEIToneMapId                           ( m_toneMapId );
  m_cTEncTop.setTMISEIToneMapCancelFlag                   ( m_toneMapCancelFlag );
  m_cTEncTop.setTMISEIToneMapPersistenceFlag              ( m_toneMapPersistenceFlag );
  m_cTEncTop.setTMISEICodedDataBitDepth                   ( m_toneMapCodedDataBitDepth );
  m_cTEncTop.setTMISEITargetBitDepth                      ( m_toneMapTargetBitDepth );
  m_cTEncTop.setTMISEIModelID                             ( m_toneMapModelId );
  m_cTEncTop.setTMISEIMinValue                            ( m_toneMapMinValue );
  m_cTEncTop.setTMISEIMaxValue                            ( m_toneMapMaxValue );
  m_cTEncTop.setTMISEISigmoidMidpoint                     ( m_sigmoidMidpoint );
  m_cTEncTop.setTMISEISigmoidWidth                        ( m_sigmoidWidth );
  m_cTEncTop.setTMISEIStartOfCodedInterva                 ( m_startOfCodedInterval );
  m_cTEncTop.setTMISEINumPivots                           ( m_numPivots );
  m_cTEncTop.setTMISEICodedPivotValue                     ( m_codedPivotValue );
  m_cTEncTop.setTMISEITargetPivotValue                    ( m_targetPivotValue );
  m_cTEncTop.setTMISEICameraIsoSpeedIdc                   ( m_cameraIsoSpeedIdc );
  m_cTEncTop.setTMISEICameraIsoSpeedValue                 ( m_cameraIsoSpeedValue );
  m_cTEncTop.setTMISEIExposureIndexIdc                    ( m_exposureIndexIdc );
  m_cTEncTop.setTMISEIExposureIndexValue                  ( m_exposureIndexValue );
  m_cTEncTop.setTMISEIExposureCompensationValueSignFlag   ( m_exposureCompensationValueSignFlag );
  m_cTEncTop.setTMISEIExposureCompensationValueNumerator  ( m_exposureCompensationValueNumerator );
  m_cTEncTop.setTMISEIExposureCompensationValueDenomIdc   ( m_exposureCompensationValueDenomIdc );
  m_cTEncTop.setTMISEIRefScreenLuminanceWhite             ( m_refScreenLuminanceWhite );
  m_cTEncTop.setTMISEIExtendedRangeWhiteLevel             ( m_extendedRangeWhiteLevel );
  m_cTEncTop.setTMISEINominalBlackLevelLumaCodeValue      ( m_nominalBlackLevelLumaCodeValue );
  m_cTEncTop.setTMISEINominalWhiteLevelLumaCodeValue      ( m_nominalWhiteLevelLumaCodeValue );
  m_cTEncTop.setTMISEIExtendedWhiteLevelLumaCodeValue     ( m_extendedWhiteLevelLumaCodeValue );
  m_cTEncTop.setFramePackingArrangementSEIEnabled( m_framePackingSEIEnabled );
  m_cTEncTop.setFramePackingArrangementSEIType( m_framePackingSEIType );
  m_cTEncTop.setFramePackingArrangementSEIId( m_framePackingSEIId );
  m_cTEncTop.setFramePackingArrangementSEIQuincunx( m_framePackingSEIQuincunx );
  m_cTEncTop.setFramePackingArrangementSEIInterpretation( m_framePackingSEIInterpretation );
  m_cTEncTop.setDisplayOrientationSEIAngle( m_displayOrientationSEIAngle );
  m_cTEncTop.setTemporalLevel0IndexSEIEnabled( m_temporalLevel0IndexSEIEnabled );
  m_cTEncTop.setGradualDecodingRefreshInfoEnabled( m_gradualDecodingRefreshInfoEnabled );
  m_cTEncTop.setDecodingUnitInfoSEIEnabled( m_decodingUnitInfoSEIEnabled );
  m_cTEncTop.setSOPDescriptionSEIEnabled( m_SOPDescriptionSEIEnabled );
  m_cTEncTop.setScalableNestingSEIEnabled( m_scalableNestingSEIEnabled );
  m_cTEncTop.setTileUniformSpacingFlag     ( m_tileUniformSpacingFlag );
  m_cTEncTop.setNumColumnsMinus1           ( m_numTileColumnsMinus1 );
  m_cTEncTop.setNumRowsMinus1              ( m_numTileRowsMinus1 );
#if ETRI_MultiplePPS
  if (em_NumAdditionalPPS > 0)
  {	  
	  m_cTEncTop.ETRI_setNumAddtionalPPS(em_NumAdditionalPPS);
	  m_cTEncTop.ETRI_setMultipleTile((ETRI_PPSTile_t*)em_MultipleTile);
  }
#endif
  if(!m_tileUniformSpacingFlag)
  {
    m_cTEncTop.setColumnWidth              ( m_tileColumnWidth );
    m_cTEncTop.setRowHeight                ( m_tileRowHeight );
  }
  m_cTEncTop.xCheckGSParameters();
  Int uiTilesCount          = (m_numTileRowsMinus1+1) * (m_numTileColumnsMinus1+1);
  if(uiTilesCount == 1)
  {
    m_bLFCrossTileBoundaryFlag = true; 
  }
  m_cTEncTop.setLFCrossTileBoundaryFlag( m_bLFCrossTileBoundaryFlag );
  m_cTEncTop.setWaveFrontSynchro           ( m_iWaveFrontSynchro );
  m_cTEncTop.setWaveFrontSubstreams        ( m_iWaveFrontSubstreams );
  m_cTEncTop.setTMVPModeId ( m_TMVPModeId );
  m_cTEncTop.setUseScalingListId           ( m_useScalingListId  );
  m_cTEncTop.setScalingListFile            ( m_scalingListFile   );
  m_cTEncTop.setSignHideFlag(m_signHideFlag);
  m_cTEncTop.setUseRateCtrl         ( m_RCEnableRateControl );
  m_cTEncTop.setTargetBitrate       ( m_RCTargetBitrate );
  m_cTEncTop.setKeepHierBit         ( m_RCKeepHierarchicalBit );
  m_cTEncTop.setLCULevelRC          ( m_RCLCULevelRC );
  m_cTEncTop.setUseLCUSeparateModel ( m_RCUseLCUSeparateModel );
  m_cTEncTop.setInitialQP           ( m_RCInitialQP );
  m_cTEncTop.setForceIntraQP        ( m_RCForceIntraQP );
#if KAIST_RC
  m_cTEncTop.setCpbSaturationEnabled(m_RCCpbSaturationEnabled);
  m_cTEncTop.setCpbSize(m_RCCpbSize);
  m_cTEncTop.setInitialCpbFullness(m_RCInitialCpbFullness);
#endif
  m_cTEncTop.setTransquantBypassEnableFlag(m_TransquantBypassEnableFlag);
  m_cTEncTop.setCUTransquantBypassFlagForceValue(m_CUTransquantBypassFlagForce);
  m_cTEncTop.setUseRecalculateQPAccordingToLambda( m_recalculateQPAccordingToLambda );
  m_cTEncTop.setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );
  m_cTEncTop.setActiveParameterSetsSEIEnabled ( m_activeParameterSetsSEIEnabled ); 
  m_cTEncTop.setVuiParametersPresentFlag( m_vuiParametersPresentFlag );
  m_cTEncTop.setAspectRatioInfoPresentFlag( m_aspectRatioInfoPresentFlag);
  m_cTEncTop.setAspectRatioIdc( m_aspectRatioIdc );
  m_cTEncTop.setSarWidth( m_sarWidth );
  m_cTEncTop.setSarHeight( m_sarHeight );
  m_cTEncTop.setOverscanInfoPresentFlag( m_overscanInfoPresentFlag );
  m_cTEncTop.setOverscanAppropriateFlag( m_overscanAppropriateFlag );
  m_cTEncTop.setVideoSignalTypePresentFlag( m_videoSignalTypePresentFlag );
  m_cTEncTop.setVideoFormat( m_videoFormat );
  m_cTEncTop.setVideoFullRangeFlag( m_videoFullRangeFlag );
  m_cTEncTop.setColourDescriptionPresentFlag( m_colourDescriptionPresentFlag );
  m_cTEncTop.setColourPrimaries( m_colourPrimaries );
  m_cTEncTop.setTransferCharacteristics( m_transferCharacteristics );
  m_cTEncTop.setMatrixCoefficients( m_matrixCoefficients );
  m_cTEncTop.setChromaLocInfoPresentFlag( m_chromaLocInfoPresentFlag );
  m_cTEncTop.setChromaSampleLocTypeTopField( m_chromaSampleLocTypeTopField );
  m_cTEncTop.setChromaSampleLocTypeBottomField( m_chromaSampleLocTypeBottomField );
  m_cTEncTop.setNeutralChromaIndicationFlag( m_neutralChromaIndicationFlag );
  m_cTEncTop.setDefaultDisplayWindow( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset, m_defDispWinBottomOffset );
  m_cTEncTop.setFrameFieldInfoPresentFlag( m_frameFieldInfoPresentFlag );
  m_cTEncTop.setPocProportionalToTimingFlag( m_pocProportionalToTimingFlag );
  m_cTEncTop.setNumTicksPocDiffOneMinus1   ( m_numTicksPocDiffOneMinus1    );
  m_cTEncTop.setBitstreamRestrictionFlag( m_bitstreamRestrictionFlag );
  m_cTEncTop.setTilesFixedStructureFlag( m_tilesFixedStructureFlag );
  m_cTEncTop.setMotionVectorsOverPicBoundariesFlag( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTop.setMinSpatialSegmentationIdc( m_minSpatialSegmentationIdc );
  m_cTEncTop.setMaxBytesPerPicDenom( m_maxBytesPerPicDenom );
  m_cTEncTop.setMaxBitsPerMinCuDenom( m_maxBitsPerMinCuDenom );
  m_cTEncTop.setLog2MaxMvLengthHorizontal( m_log2MaxMvLengthHorizontal );
  m_cTEncTop.setLog2MaxMvLengthVertical( m_log2MaxMvLengthVertical );

#if ETRI_DLL_INTERFACE
    ///< ETRI Encoding Option
  m_cTEncTop.ETRI_setETRI_EncoderOptions(em_iETRI_EncoderOptions);
  m_cTEncTop.ETRI_setETRI_numofGOPforME(em_iETRI_numofGOPforME);
  m_cTEncTop.ETRI_setETRI_FullIOProcessOption (em_iETRI_FullIOProcessOption);
  m_cTEncTop.ETRI_setETRI_InfiniteProcessingOption(em_iETRI_InfiniteProcessing);
  m_cTEncTop.ETRI_setETRI_ETRI_ColorSpaceYV12Option(em_iETRI_ColorSpaceYV12);
  m_cTEncTop.ETRI_setETRI_SliceIndex(em_sETRI_SliceIndex);
#endif


}

Void TAppEncTop::xCreateLib()
{
  // Video I/O
  m_cTVideoIOYuvInputFile.open( m_pchInputFile,     false, m_inputBitDepthY, m_inputBitDepthC, m_internalBitDepthY, m_internalBitDepthC );  // read  mode
  m_cTVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1]);
#if ETRI_DLL_INTERFACE  
  m_cTVideoIOYuvInputFile.ETRI_setYV12Enable(em_iETRI_ColorSpaceYV12); // 2015 07 11 by seok : Set Color Space YV12	
#endif

  if (m_pchReconFile)
  {
    m_cTVideoIOYuvReconFile.open(m_pchReconFile, true, m_outputBitDepthY, m_outputBitDepthC, m_internalBitDepthY, m_internalBitDepthC);  // write mode
#if ETRI_MULTITHREAD_2
	m_cTEncTop.ETRI_setReconFileOk(true);
#endif
  }

  
  // Neo Decoder
  m_cTEncTop.create();
}

Void TAppEncTop::xDestroyLib()
{
#if !ETRI_DLL_INTERFACE	// 2013 6 14 by Seok
  // Video I/O
  m_cTVideoIOYuvInputFile.close();
  m_cTVideoIOYuvReconFile.close();
#endif  
  // Neo Decoder
  m_cTEncTop.destroy();
}

Void TAppEncTop::xInitLib(Bool isFieldCoding)
{
  m_cTEncTop.init(isFieldCoding);
}

//#if ETRI_DLL_INTERFACE
//Void TAppEncTop::ETRI_Free(InterfaceInfo& eETRIInterface)
//{
//#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)	
//	if (!IsBadReadPtr(eETRIInterface.nFrameStartOffset, m_iIntraPeriod))	{ xFree(eETRIInterface.nFrameStartOffset);    	eETRIInterface.nFrameStartOffset = nullptr; }
//	if (!IsBadReadPtr(eETRIInterface.nFrameTypeInGop, m_iIntraPeriod)) 		{ xFree(eETRIInterface.nFrameTypeInGop);      	eETRIInterface.nFrameTypeInGop = nullptr; }
//	if (!IsBadReadPtr(eETRIInterface.nPicDecodingOrder, m_iIntraPeriod))		{ xFree(eETRIInterface.nPicDecodingOrder);   	eETRIInterface.nPicDecodingOrder = nullptr; }
//	if (!IsBadReadPtr(eETRIInterface.nPicPresentationOrder, m_iIntraPeriod))	{ xFree(eETRIInterface.nPicPresentationOrder); 	eETRIInterface.nPicPresentationOrder = nullptr; }
//	if (!IsBadReadPtr(eETRIInterface.nSliceIndex, m_iIntraPeriod))				{ xFree(eETRIInterface.nSliceIndex); 	eETRIInterface.nSliceIndex = nullptr; }
//#else
//	if (!IsBadReadPtr(eETRIInterface.nFrameStartOffset, eETRIInterface.nGOPsize))		{xFree(eETRIInterface.nFrameStartOffset);    	eETRIInterface.nFrameStartOffset = nullptr;}
//	if (!IsBadReadPtr(eETRIInterface.nFrameTypeInGop, eETRIInterface.nGOPsize)) 		{xFree(eETRIInterface.nFrameTypeInGop);      	eETRIInterface.nFrameTypeInGop = nullptr;}
//	if (!IsBadReadPtr(eETRIInterface.nPicDecodingOrder, eETRIInterface.nGOPsize))		{xFree(eETRIInterface.nPicDecodingOrder);   	eETRIInterface.nPicDecodingOrder = nullptr;}
//	if (!IsBadReadPtr(eETRIInterface.nPicPresentationOrder, eETRIInterface.nGOPsize))	{xFree(eETRIInterface.nPicPresentationOrder); 	eETRIInterface.nPicPresentationOrder = nullptr;}
//	if (!IsBadReadPtr(eETRIInterface.nSliceIndex, eETRIInterface.nGOPsize))				{ xFree(eETRIInterface.nSliceIndex); 	eETRIInterface.nSliceIndex = nullptr; }
//#endif
//}
//#endif

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal variable
 - until the end of input YUV file, call encoding function in TEncTop class
 - delete allocated buffers
 - destroy internal class
 .
 */
#if ETRI_DLL_INTERFACE
///< encode using InterfaceInfo parameter by yhee 2015.06.10.
Void TAppEncTop::encode(InterfaceInfo& eETRIInterface)
{
	///< Infinite Process Setting, Refresh the encoder
	Bool 	bAnalyserClear = eETRIInterface.CTRParam.bAnalyzeClear;	
	if ((*m_cTEncTop.ETRI_getpiPOCLast() == -1) && bAnalyserClear)
	{
		//refresh slice index by yhee
		short sSliceIndex = *(short *)&eETRIInterface.CTRParam.sSliceIndex;
		m_cTEncTop.ETRI_Clear_AnalyzeParameter();
		m_cTEncTop.ETRI_Clear_ListPic(bAnalyserClear);
		m_cTEncTop.ETRI_setETRI_SliceIndex(sSliceIndex);
	}

#if ETRI_BUGFIX_DLL_INTERFACE
	//2016 02 22 by Jinwuk Seok
	if (e_ETRIInterface.bEos)
	{
		m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd+1);
		m_framesToBeEncoded = m_iFrameRcvd + 1;
	}
#endif

	//GOP Parallel
	int  iNumPeriod = 0;

#if ETRI_MULTITHREAD_2
	if (m_pchReconFile)
#endif
	// get buffers
	{
	xGetBuffer((TComPicYuv*&)eETRIInterface.pcPicYuvRec); //zeroone 20140612
	}	
	// read input YUV file
	m_cTVideoIOYuvInputFile.read( (TComPicYuv*)eETRIInterface.pcPicYuvOrg, m_aiPad ); //zeroone 20140612
    
	// increase number of received frames
	m_iFrameRcvd++;
	eETRIInterface.bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
	Bool flush = 0;
	if (m_cTVideoIOYuvInputFile.isEof())
	{
		flush = true;
		eETRIInterface.bEos = true;
		m_iFrameRcvd--;
		m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
	}
	if ( m_isField )
	{
#if ETRI_MULTITHREAD_2
		printf("Is not supported the Field function\n");
		return;
#else
		m_cTEncTop.encode( eETRIInterface.bEos, flush ? 0 : (TComPicYuv*)eETRIInterface.pcPicYuvOrg, m_cListPicYuvRec, em_outputAccessUnits, eETRIInterface.iNumEncoded, m_isTopFieldFirst);
#endif
	}
	else
	{
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
		m_cTEncTop.ETRI_encode(eETRIInterface.bEos, flush ? 0 : (TComPicYuv*)eETRIInterface.pcPicYuvOrg, em_cListPicYuvRec, em_cListPic, em_poutputAccessUnits, eETRIInterface.iNumEncoded);
#else
		m_cTEncTop.ETRI_encode(eETRIInterface.bEos, flush ? 0 : (TComPicYuv*)eETRIInterface.pcPicYuvOrg, m_cListPicYuvRec, em_poutputAccessUnits, eETRIInterface.iNumEncoded);
#endif
#else
		m_cTEncTop.encode( eETRIInterface.bEos, flush ? 0 : (TComPicYuv*)eETRIInterface.pcPicYuvOrg, m_cListPicYuvRec, em_outputAccessUnits, eETRIInterface.iNumEncoded );
#endif
	}
	// write bistream to file if necessary
	if ( eETRIInterface.iNumEncoded > 0 )
	{
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
		ETRI_xWriteOutput(*eETRIInterface.FStream, eETRIInterface.iNumEncoded, em_poutputAccessUnits, &em_cListPicYuvRec);

		if (eETRIInterface.iNumEncoded >= m_iIntraPeriod)
		{
			iNumPeriod++;
			m_cTEncTop.ETRI_xPocReArrayPicBuffer(&em_cListPic, iNumPeriod*m_iIntraPeriod);
		}
#else
		ETRI_xWriteOutput(*eETRIInterface.FStream, eETRIInterface.iNumEncoded, em_poutputAccessUnits);
#endif
#else
		xWriteOutput(*eETRIInterface.FStream, eETRIInterface.iNumEncoded, em_outputAccessUnits);
		em_outputAccessUnits.clear();
#endif
	}
	return;	
}

Void TAppEncTop::ETRI_DLLEncoderInitialize	(InterfaceInfo& eETRIInterface)
{
	create(e_ETRIInterface.argc, e_ETRIInterface.argv);
	// initialize internal class & member variables
	xInitLibCfg();
	xCreateLib();
	xInitLib(m_isField);

	TComPicYuv* 	  pcPicYuvOrg = new TComPicYuv;
	pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );

	eETRIInterface.pcPicYuvOrg = (void *)pcPicYuvOrg;
	eETRIInterface.pcPicYuvRec = NULL;
	eETRIInterface.iNumEncoded = 0;
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)	
	em_poutputAccessUnits = new AccessUnit_t[m_iIntraPeriod];
	m_cTEncTop.ETRI_xSetNewPicBuffer(&em_cListPic, m_iIntraPeriod + 1);
	if (m_pchReconFile)
		ETRI_xSetBuffer(&em_cListPicYuvRec, m_iIntraPeriod + 1);
#else
	em_poutputAccessUnits = new AccessUnit_t[m_iGOPSize];
#endif	
#endif

	eETRIInterface.bEos = false;
	eETRIInterface.m_aiPad[0] = m_aiPad[0];
	eETRIInterface.m_aiPad[1] = m_aiPad[1];
	eETRIInterface.m_piFrameRcvd      = (Int *)&m_iFrameRcvd;
	eETRIInterface.m_pchBitstreamFile = m_pchBitstreamFile;
	eETRIInterface.m_pchInputFile = m_pchInputFile;
	ETRI_FstreamInterface(eETRIInterface);

	eETRIInterface.m_iSourceHeight = m_iSourceHeight;
	eETRIInterface.m_iSourceWidth = m_iSourceWidth;
	eETRIInterface.is16bit = m_inputBitDepthY > 8 || m_inputBitDepthC > 8;
	eETRIInterface.nGOPsize = m_iGOPSize;


	///<Initialize CTRParam
	eETRIInterface.CTRParam.piPOCLastIdx         = m_cTEncTop.ETRI_getpiPOCLast();
	eETRIInterface.CTRParam.piNumPicRcvdIdx      = m_cTEncTop.ETRI_getpiNumPicRcvd();
	eETRIInterface.CTRParam.puiNumAllPicCodedIdx = m_cTEncTop.ETRI_getpiNumAllPicCoded();;
	eETRIInterface.CTRParam.bAnalyzeClear        = false;
	eETRIInterface.CTRParam.iFramestobeEncoded   = m_framesToBeEncoded;
	eETRIInterface.CTRParam.iNumEncoders         = em_iETRI_EncoderOptions;	
	eETRIInterface.CTRParam.iNumofGOPforME       = em_iETRI_numofGOPforME;	
	eETRIInterface.CTRParam.iFullIORDProcess     = em_iETRI_FullIOProcessOption;

#if ETRI_BUGFIX_DLL_INTERFACE
	eETRIInterface.CTRParam.uiNumofEncodedGOPforME = 0;
	eETRIInterface.CTRParam.piIntraPeriod        = (Int *)&m_iIntraPeriod;
#endif

	///<Set InfiniteProcessing Condition
	Int iETRI_InfiniteProcessing = em_iETRI_InfiniteProcessing;

	em_iETRI_InfiniteProcessing =  (iETRI_InfiniteProcessing & 0x01);
	eETRIInterface.CTRParam.iInfiniteProcessing = em_iETRI_InfiniteProcessing;
	m_cTEncTop.ETRI_setETRI_InfiniteProcessingOption(em_iETRI_InfiniteProcessing);

	em_bETRI_FullReleaseMode = ((iETRI_InfiniteProcessing & 0x010)>>1);	

	///<Set SliceInden from System
	em_sETRI_SliceIndex = *(short *)&eETRIInterface.CTRParam.sSliceIndex;
	m_cTEncTop.ETRI_setETRI_SliceIndex(em_sETRI_SliceIndex);
}

#else
Void TAppEncTop::encode()
{
	fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
	if (!bitstreamFile)
	{
		fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
		exit(EXIT_FAILURE);
	}

	TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
	TComPicYuv*       pcPicYuvRec = NULL;

	// initialize internal class & member variables
	xInitLibCfg();
	xCreateLib();
	xInitLib(m_isField);

	// main encoder loop
	Int   iNumEncoded = 0;
	Bool  bEos = false;
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	int  iNumPeriod = 0;
#endif
#endif
	// allocate original YUV buffer
	pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );

#if ETRI_MULTITHREAD_2 //revision of 1006v3 GOP

#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	AccessUnit_t *e_poutputAccessUnits = new AccessUnit_t[m_iIntraPeriod];
	TComList<TComPic*> cListPic;
	m_cTEncTop.ETRI_xSetNewPicBuffer(&cListPic, m_iIntraPeriod + 1);

	TComList<TComPicYuv*> cListPicYuvRec;
	
	if (m_pchReconFile)
	ETRI_xSetBuffer(&cListPicYuvRec, m_iIntraPeriod + 1);
#else
	AccessUnit_t *e_poutputAccessUnits = new AccessUnit_t[m_iGOPSize];
#endif

#else
	list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process
#endif
#if ETRI_MULTITHREAD_2
	// starting time
	double dResult;
#if (_ETRI_WINDOWS_APPLICATION)
	long lBefore = clock();
#else
	timespec lBefore;
	clock_gettime(CLOCK_MONOTONIC, &lBefore); // Works on Linux by yhee 2016.04.19
#endif
	//long lBefore = clock();
#endif

	while ( !bEos )
	{
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL != ETRI_GOP_PARALLEL)
		if (m_pchReconFile)
			xGetBuffer(pcPicYuvRec);		
#endif
#else	// get buffers
		xGetBuffer(pcPicYuvRec);
#endif
		// read input YUV file
		m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, m_aiPad );

		// increase number of received frames
		m_iFrameRcvd++;

		bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
		Bool flush = 0;
		// if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
		if (m_cTVideoIOYuvInputFile.isEof())
		{
			flush = true;
			bEos = true;
			m_iFrameRcvd--;
			m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
		}

		// call encoding function for one frame
		if ( m_isField )
		{
#if ETRI_MULTITHREAD_2
			printf("Is not supported the Field function\n");
			break;
#else
			m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst);
#endif
		}
		else
		{
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
			m_cTEncTop.ETRI_encode(bEos, flush ? 0 : pcPicYuvOrg, cListPicYuvRec, cListPic, e_poutputAccessUnits, iNumEncoded);
#else
			m_cTEncTop.ETRI_encode(bEos, flush ? 0 : pcPicYuvOrg, m_cListPicYuvRec, e_poutputAccessUnits, iNumEncoded);
#endif
#else
			m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
#endif
		}

		// write bistream to file if necessary
		if ( iNumEncoded > 0 )
		{
#if ETRI_MULTITHREAD_2
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
			ETRI_xWriteOutput(bitstreamFile, iNumEncoded, e_poutputAccessUnits, &cListPicYuvRec);

			if(iNumEncoded >= m_iIntraPeriod)
			{
				iNumPeriod++;
				m_cTEncTop.ETRI_xPocReArrayPicBuffer(&cListPic, iNumPeriod*m_iIntraPeriod); 
			}
#else
			ETRI_xWriteOutput(bitstreamFile, iNumEncoded, e_poutputAccessUnits);
#endif
#else
			xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
			outputAccessUnits.clear();
#endif
		}
	}
#if ETRI_MULTITHREAD_2
#if (_ETRI_WINDOWS_APPLICATION)
	dResult = (double)(clock() - lBefore) / CLOCKS_PER_SEC;
#else
	timespec iCurrTime;
	clock_gettime(CLOCK_MONOTONIC, &iCurrTime); // Works on Linux by yhee 2016.04.19
	dResult = (iCurrTime.tv_sec - lBefore.tv_sec);
	dResult += (iCurrTime.tv_nsec - lBefore.tv_nsec) / 1000000000.0;
#endif
	printf("\n Total Encoding Time: %12.3f sec.\n", dResult);
#endif

	m_cTEncTop.printSummary(m_isField);

	// delete original YUV buffer
	pcPicYuvOrg->destroy();
	delete pcPicYuvOrg;
	pcPicYuvOrg = NULL;

#if ETRI_MULTITHREAD_2
	delete[] e_poutputAccessUnits;
	e_poutputAccessUnits = NULL;
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
	m_cTEncTop.ETRI_deletePicBuffer(&cListPic);
	ETRI_xDeleteBuffer(&cListPicYuvRec);
#endif
#else
	// delete used buffers in encoder class
	m_cTEncTop.deletePicBuffer();
#endif

	// delete buffers & classes
	xDeleteBuffer();
	xDestroyLib();

	printRateSummary();

	return;
}
#endif
// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - application has picture buffer list with size of GOP
 - picture buffer list acts as ring buffer
 - end of the list has the latest picture
 .
 */
Void TAppEncTop::xGetBuffer( TComPicYuv*& rpcPicYuvRec)
{
  assert( m_iGOPSize > 0 );
  
  // org. buffer
  if ( m_cListPicYuvRec.size() == (UInt)m_iGOPSize )
  {
    rpcPicYuvRec = m_cListPicYuvRec.popFront();

  }
  else
  {
    rpcPicYuvRec = new TComPicYuv;
    
    rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );

  }
  m_cListPicYuvRec.pushBack( rpcPicYuvRec );
}

Void TAppEncTop::xDeleteBuffer( )
{
  TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_cListPicYuvRec.begin();
  
  Int iSize = Int( m_cListPicYuvRec.size() );
  
  for ( Int i = 0; i < iSize; i++ )
  {
    TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
    pcPicYuvRec->destroy();
    delete pcPicYuvRec; pcPicYuvRec = NULL;
  }
  
}

/** \param iNumEncoded  number of encoded frames
 */
#if ETRI_MULTITHREAD_2
Void TAppEncTop::xWriteOutput(ETRI_StreamInterface& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits)
#else 
Void TAppEncTop::xWriteOutput(ETRI_StreamInterface& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits)
#endif
{
  if (m_isField)
  {
#if ETRI_MULTITHREAD_2
	  printf("ETRI_MULTITHREAD_2 dose not support \n");
#endif
    //Reinterlace fields
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();
    
    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }
    
    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      TComPicYuv*  pcPicYuvRecTop  = *(iterPicYuvRec++);
      TComPicYuv*  pcPicYuvRecBottom  = *(iterPicYuvRec++);
      
      if (m_pchReconFile)
      {
        m_cTVideoIOYuvReconFile.write( pcPicYuvRecTop, pcPicYuvRecBottom, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, m_isTopFieldFirst );
      }
      
      const AccessUnit& auTop = *(iterBitstream++);
      const vector<UInt>& statsTop = writeAnnexB(bitstreamFile, auTop);
      rateStatsAccum(auTop, statsTop);
      
      const AccessUnit& auBottom = *(iterBitstream++);
      const vector<UInt>& statsBottom = writeAnnexB(bitstreamFile, auBottom);
      rateStatsAccum(auBottom, statsBottom);
#if ETRI_DLL_INTERFACE	// 2013 10 24 by Seok
	  e_ETRIInterface.nFrameStartOffset[i]= em_FrameBytes;
	  m_cTEncTop.ETRI_getFrameInfoforDLL(i, e_ETRIInterface.nPicDecodingOrder[i], e_ETRIInterface.nFrameTypeInGop[i], e_ETRIInterface.nPicPresentationOrder[i], e_ETRIInterface.nSliceIndex[i]);
#endif
    }
  }
  else
  {
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();
    
	
	if (m_pchReconFile)
	{
		for (i = 0; i < iNumEncoded; i++)
    {
      --iterPicYuvRec;
    }
	}
    
	for (i = 0; i < iNumEncoded; i++)
	{
		
		if (m_pchReconFile)
    {
      TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
      if (m_pchReconFile)
      {
        m_cTVideoIOYuvReconFile.write( pcPicYuvRec, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
      }
		}

      const AccessUnit& au = *(iterBitstream++);
      const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
      rateStatsAccum(au, stats);
#if ETRI_DLL_INTERFACE	
		e_ETRIInterface.nFrameStartOffset[i] = em_FrameBytes;
		m_cTEncTop.ETRI_getFrameInfoforDLL(i, e_ETRIInterface.nPicDecodingOrder[i], e_ETRIInterface.nFrameTypeInGop[i], e_ETRIInterface.nPicPresentationOrder[i], e_ETRIInterface.nSliceIndex[i]);
#endif
    }
  }
}

/**
 *
 */
void TAppEncTop::rateStatsAccum(const AccessUnit& au, const std::vector<UInt>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<UInt>::const_iterator it_stats = annexBsizes.begin();
#if ETRI_DLL_INTERFACE	// 2013 10 24 by Seok
  em_FrameBytes = 0;
#endif

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TSA_R:
    case NAL_UNIT_CODED_SLICE_TSA_N:
    case NAL_UNIT_CODED_SLICE_STSA_R:
    case NAL_UNIT_CODED_SLICE_STSA_N:
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_RADL_N:
    case NAL_UNIT_CODED_SLICE_RADL_R:
    case NAL_UNIT_CODED_SLICE_RASL_N:
    case NAL_UNIT_CODED_SLICE_RASL_R:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
#if ETRI_DLL_INTERFACE	// 2013 10 24 by Seok
	em_FrameBytes += *it_stats;
#endif
  }
}

void TAppEncTop::printRateSummary()
{
  Double time = (Double) m_iFrameRcvd / m_iFrameRate;
  printf("Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time);
#if VERBOSE_RATE
  printf("Bytes for SPS/PPS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
#endif
}
#if ETRI_MULTITHREAD_2
//////////////////////////////////////////////////////////////////////
// gplusplus

Void TAppEncTop::ETRI_xSetBuffer( TComList<TComPicYuv*>* pcListPicYuvRec, int nCount )
{
  TComPicYuv* rpcPicYuvRec;

  assert( nCount > 0 );
  
  for(int i = 0; i < nCount; i++)
  {
    rpcPicYuvRec = new TComPicYuv;
	rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );

	pcListPicYuvRec->pushBack( rpcPicYuvRec );
  }
}

Void TAppEncTop::ETRI_xDeleteBuffer( TComList<TComPicYuv*>* pcListPicYuvRec )
{
  TComList<TComPicYuv*>::iterator iterPicYuvRec  = pcListPicYuvRec->begin();
  
  Int iSize = Int( pcListPicYuvRec->size() );
  
  for ( Int i = 0; i < iSize; i++ )
  {
    TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
    pcPicYuvRec->destroy();
    delete pcPicYuvRec; pcPicYuvRec = NULL;
  }  
}

#if ETRI_DLL_INTERFACE
Void TAppEncTop::ETRI_xDeleteAU()
{
	delete[] em_poutputAccessUnits;
	em_poutputAccessUnits = NULL;
}
#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)  
Void TAppEncTop::ETRI_xDeletePicBuffers()
{
	TComList<TComPicYuv*>* pcListPicYuvRec = &em_cListPicYuvRec;
	TComList<TComPicYuv*>::iterator iterPicYuvRec = pcListPicYuvRec->begin();

	Int iSize = Int(pcListPicYuvRec->size());

	for (Int i = 0; i < iSize; i++)
    {
		TComPicYuv*  pcPicYuvRec = *(iterPicYuvRec++);
		pcPicYuvRec->destroy();
		delete pcPicYuvRec; pcPicYuvRec = NULL;
    }
	m_cTEncTop.ETRI_deletePicBuffer(&em_cListPic);
}
#endif
#endif

#if (ETRI_PARALLEL_SEL == ETRI_GOP_PARALLEL)
Void TAppEncTop::ETRI_xWriteOutput(ETRI_StreamInterface& bitstreamFile, Int iNumEncoded, AccessUnit_t* accessUnits, TComList<TComPicYuv*>* pcListPicYuvRec)
{
    Int i;

    TComList<TComPicYuv*>::iterator iterPicYuvRec = pcListPicYuvRec->begin();
	int offset = 0;
	for(i = 0; i < iNumEncoded; i++)
	{
		if (m_pchReconFile)
		{
			TComPicYuv*  pcPicYuvRec = *(iterPicYuvRec++);  
			if (!pcPicYuvRec->getbUsed())
				pcPicYuvRec = *(iterPicYuvRec++);

			//printf("$$$ RconFile : POC = %d\n", pcPicYuvRec->getPoc());
			m_cTVideoIOYuvReconFile.write( pcPicYuvRec, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
			pcPicYuvRec->setbUsed(false);
		}

		while(accessUnits[i+offset].pos < 0)
		{
			offset++;
		}

		list<AccessUnit>::const_iterator iterBitstream = accessUnits[i+offset].outputAccessUnits.begin();
			const AccessUnit& au = *(iterBitstream++);
			const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
			rateStatsAccum(au, stats);
#if ETRI_DLL_INTERFACE	
		e_ETRIInterface.nFrameStartOffset[i] = em_FrameBytes;  //skip offset em_frameencoder[]
		m_cTEncTop.ETRI_getFrameInfoforDLL(i + offset, e_ETRIInterface.nPicDecodingOrder[i], e_ETRIInterface.nFrameTypeInGop[i], e_ETRIInterface.nPicPresentationOrder[i], e_ETRIInterface.nSliceIndex[i]);
#endif
		accessUnits[i + offset].outputAccessUnits.clear();
	}
}
#else
Void TAppEncTop::ETRI_xWriteOutput(ETRI_StreamInterface& bitstreamFile, Int iNumEncoded, AccessUnit_t* accessUnits)
{
    Int i;

    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
	int offset = 0;
	
	if (m_pchReconFile)
	{
		for ( i = 0; i < iNumEncoded; i++ )
		{
			--iterPicYuvRec;
		}
	}

	for(i = 0; i < iNumEncoded; i++)
	{
		
		if (m_pchReconFile)
		{
			TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
			m_cTVideoIOYuvReconFile.write( pcPicYuvRec, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
		}

		while(accessUnits[i+offset].pos < 0)
		{
			offset++;
		}

		list<AccessUnit>::const_iterator iterBitstream = accessUnits[i+offset].outputAccessUnits.begin();		
		{
		const AccessUnit& au = *(iterBitstream++);
		const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
		rateStatsAccum(au, stats);
		}
#if ETRI_DLL_INTERFACE	// 2013 10 24 by Seok
		e_ETRIInterface.nFrameStartOffset[i] = em_FrameBytes;
		m_cTEncTop.ETRI_getFrameInfoforDLL(i, e_ETRIInterface.nPicDecodingOrder[i], e_ETRIInterface.nFrameTypeInGop[i], e_ETRIInterface.nPicPresentationOrder[i], e_ETRIInterface.nSliceIndex[i]);
#endif
		accessUnits[i+offset].outputAccessUnits.clear();
	}
}
#endif
#endif
//! \}
