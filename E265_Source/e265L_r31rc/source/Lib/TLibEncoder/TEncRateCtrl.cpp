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

/** \file     TEncRateCtrl.cpp
    \brief    Rate control manager class
*/
#include "TEncRateCtrl.h"
#include "../TLibCommon/TComPic.h"

#include <cmath>

using namespace std;

#if KAIST_RC
TEncRateCtrl::TEncRateCtrl()
{
// 	m_Pics = NULL;
// 	m_picPara = NULL;
// 	m_LCUPara = NULL;
}

TEncRateCtrl::~TEncRateCtrl()
{
  //destroy();
}

Void TEncRateCtrl::destroy()
{
	delete[] m_picPara;
	m_picPara = NULL;

	for (Int j = 0; j < m_intraSize; j++)
	{
		delete[] m_Pics[j].m_remainingCostIntra;
		delete[] m_Pics[j].m_LCULeft;
		delete[] m_Pics[j].m_bitsLeft;
		delete[] m_Pics[j].m_width_tile;
		delete[] m_Pics[j].m_height_tile;
		delete[] m_Pics[j].m_numberOfLCUinTile;
		delete[] m_Pics[j].m_numberOfPixel;
		delete[] m_Pics[j].m_pixelsLeft;
		delete[] m_Pics[j].m_LCUs;
		delete[] m_Pics[j].m_LCUPara;

		m_Pics[j].m_remainingCostIntra=NULL;
		m_Pics[j].m_LCULeft = NULL;
		m_Pics[j].m_bitsLeft = NULL;
		m_Pics[j].m_width_tile = NULL;
		m_Pics[j].m_height_tile = NULL;
		m_Pics[j].m_numberOfLCUinTile = NULL;
		m_Pics[j].m_numberOfPixel = NULL;
		m_Pics[j].m_pixelsLeft = NULL;
		m_Pics[j].m_LCUs = NULL;
		m_Pics[j].m_LCUPara = NULL;
	}

	delete[] m_Pics;
	delete[] m_sliceActualBits;
	delete[] m_sliceTotalTargetBits;
	delete[] m_cpbState;
	delete[] m_cpbStateFlag;
	delete[] m_tileIdxMap;

	m_Pics = NULL;
	m_sliceActualBits = NULL;
	m_sliceTotalTargetBits = NULL;
	m_cpbState = NULL;
	m_cpbStateFlag = NULL;
	m_tileIdxMap = NULL;

	if (m_useLCUSeparateModel)
	{
		for (Int i = 0; i < m_numFrameLevel; i++)
		{
			delete[] m_LCUPara[i];
			m_LCUPara[i] = NULL;
		}
		delete[] m_LCUPara;
		m_LCUPara = NULL;
	}

#if KAIST_USEPREPS
	delete[] m_costPOC;
	m_costPOC = NULL;
	delete[] m_CostGOP;
	m_CostGOP = NULL;
#endif
}

Void TEncRateCtrl::init(Int numTileCol, Int numTileRow, Int* tileColWidth, Int* tileRowHeight, Int totalFrames, Int targetBitrate, Int frameRate, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Bool useLCUSeparateModel, Int intraSize, Int GOPSize, GOPEntry  GOPList[MAX_GOP])
{
  //destroy();
  m_numTile = numTileCol*numTileRow;
  m_bpp = (Double)(targetBitrate / (Double)(frameRate*picWidth*picHeight));
  m_totalFrames = totalFrames;
  m_targetRate = targetBitrate;
  m_frameRate = frameRate;
  m_intraSize = intraSize;
  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_LCUWidth = LCUWidth;
  m_LCUHeight = LCUHeight;
  m_useLCUSeparateModel = useLCUSeparateModel;

  m_numberOfPixel = m_picWidth * m_picHeight;
  m_targetBits = (Int64)m_totalFrames * (Int64)m_targetRate / (Int64)m_frameRate;
  m_seqTargetBpp = (Double)m_targetRate / (Double)m_frameRate / (Double)m_numberOfPixel;
  if (m_seqTargetBpp < 0.03)
  {
	  m_alphaUpdate = 0.01;
	  m_betaUpdate = 0.005;
  }
  else if (m_seqTargetBpp < 0.08)
  {
	  m_alphaUpdate = 0.05;
	  m_betaUpdate = 0.025;
  }
  else if (m_seqTargetBpp < 0.2)
  {
	  m_alphaUpdate = 0.1;
	  m_betaUpdate = 0.05;
  }
  else if (m_seqTargetBpp < 0.5)
  {
	  m_alphaUpdate = 0.2;
	  m_betaUpdate = 0.1;
  }
  else
  {
	  m_alphaUpdate = 0.4;
	  m_betaUpdate = 0.2;
  }

  Int picWidthInBU = (m_picWidth  % m_LCUWidth) == 0 ? m_picWidth / m_LCUWidth : m_picWidth / m_LCUWidth + 1;
  Int picHeightInBU = (m_picHeight % m_LCUHeight) == 0 ? m_picHeight / m_LCUHeight : m_picHeight / m_LCUHeight + 1;
  m_numberOfLCU = picWidthInBU * picHeightInBU;

  Bool isLowdelay = true;
  for (Int i = 0; i<GOPSize - 1; i++)
  {
	  if (GOPList[i].m_POC > GOPList[i + 1].m_POC)
	  {
		  isLowdelay = false;
		  break;
	  }
  }
  Int numberOfLevel = 1;
  Int keepHierBits = 1;
  if (keepHierBits > 0)
  {
	  numberOfLevel = Int(log((Double)GOPSize) / log(2.0) + 0.5) + 1;
  }
  if (!isLowdelay && GOPSize == 8)
  {
	  numberOfLevel = Int(log((Double)GOPSize) / log(2.0) + 0.5) + 1;
  }
  numberOfLevel++;    // intra picture
  
  m_numFrameLevel = numberOfLevel;
  m_picPara = new TRCParameter[m_numFrameLevel];
  for (Int i = 0; i<m_numFrameLevel; i++)
  {
	  if (i>0)
      {
        m_picPara[i].m_alpha = 3.2003;
        m_picPara[i].m_beta  = -1.367;
      }
      else
      {
        m_picPara[i].m_alpha = ALPHA;   
        m_picPara[i].m_beta  = BETA2;
      }
	  m_picPara[i].m_fromWhichPOC = 0;
	  m_picPara[i].m_NofUpdate = 0;
  }

  if (m_bpp > 0.2)
  {
	  m_weight4bpp[0] = 113;
	  m_weight4bpp[1] = 32;
	  m_weight4bpp[2] = 15;
	  m_weight4bpp[3] = 17;
	  m_weight4bpp[4] = 5;
  }
  else if (m_bpp > 0.1)
  {
	  m_weight4bpp[0] = 132;
	  m_weight4bpp[1] = 38;
	  m_weight4bpp[2] = 20;
	  m_weight4bpp[3] = 18;
	  m_weight4bpp[4] = 6;
  }
  else if (m_bpp > 0.05)
  {
	  m_weight4bpp[0] = 151;
	  m_weight4bpp[1] = 44;
	  m_weight4bpp[2] = 25;
	  m_weight4bpp[3] = 19;
	  m_weight4bpp[4] = 7;
  }
  else
  {
	  m_weight4bpp[0] = 170;
	  m_weight4bpp[1] = 50;
	  m_weight4bpp[2] = 30;
	  m_weight4bpp[3] = 20;
	  m_weight4bpp[4] = 8;
  }  

  m_GOPID2Level[0] = 1;
  m_GOPID2Level[1] = 2;
  m_GOPID2Level[2] = 3;
  m_GOPID2Level[3] = 3;
  m_GOPID2Level[4] = 4;
  m_GOPID2Level[5] = 4;
  m_GOPID2Level[6] = 4;
  m_GOPID2Level[7] = 4;

  m_TargetBitsForIDR = (Int)((Double)m_intraSize * targetBitrate / frameRate + 0.5);

  KAIST_NUM_IDR_ENC = totalFrames / m_intraSize;

  m_cpbSize = targetBitrate;
  m_bufferingRate = (Int)(targetBitrate / frameRate);
    
  m_Pics = new TRCPic[m_intraSize];
  m_sliceActualBits = new Int[m_intraSize];
  m_sliceTotalTargetBits = new Int[m_intraSize];
  m_cpbState = new Int[m_intraSize];
  m_cpbStateFlag = new Int[m_intraSize];

  Int  columnIdx = 0;
  Int  rowIdx = 0;
  Int* tileBoundaryCol = new Int[numTileCol];
  Int* tileBoundaryRow = new Int[numTileRow];
  m_tileIdxMap = new int[m_numberOfLCU];
  for (Int col = 0; col < numTileCol - 1; col++)
  {
	  tileBoundaryCol[col] = 0;
	  for (Int x = 0; x <= col; x++)
		  tileBoundaryCol[col] += tileColWidth[x];
  }
  tileBoundaryCol[numTileCol - 1] = picWidthInBU;
  tileColWidth[numTileCol - 1] = tileBoundaryCol[numTileCol - 1] - (numTileCol>1?tileBoundaryCol[numTileCol - 2]:0);
  for (Int row = 0; row < numTileRow - 1; row++)
  {
	  tileBoundaryRow[row] = 0;
	  for (Int y = 0; y <= row; y++)
		  tileBoundaryRow[row] += tileRowHeight[y];
  }
  tileBoundaryRow[numTileRow - 1] = picHeightInBU;
  tileRowHeight[numTileRow - 1] = tileBoundaryRow[numTileRow - 1] - (numTileRow>1?tileBoundaryRow[numTileRow - 2]:0);
  //initialize the TileIdxMap
  for (Int i = 0; i < m_numberOfLCU; i++)
  {
	  for (Int col = 0; col < numTileCol; col++)
	  {
		  if (i % picWidthInBU < tileBoundaryCol[col])
		  {
			  columnIdx = col;
			  break;
		  }
	  }
	  for (Int row = 0; row < numTileRow; row++)
	  {
		  if (i / picWidthInBU < tileBoundaryRow[row])
		  {
			  rowIdx = row;
			  break;
		  }
	  }
	  m_tileIdxMap[i] = rowIdx * numTileCol + columnIdx;
  }

  delete[] tileBoundaryCol;
  delete[] tileBoundaryRow;
  tileBoundaryCol = NULL;
  tileBoundaryRow = NULL;

  Int LCUIdx;
	for (Int j = 0; j < m_intraSize; j++)
	{
		m_sliceActualBits[j] = 0;
		m_sliceTotalTargetBits[j] = 0;
		m_cpbState[j] = (UInt)(m_cpbSize*0.9);//0.9=pcCfg->getInitialCpbFullness()
		m_cpbStateFlag[j] = 1;

		m_Pics[j].m_frameLevel = 0;
		m_Pics[j].m_POC = 0;
		m_Pics[j].m_totalCostIntra = 0;
		m_Pics[j].m_numberOfLCU = m_numberOfLCU;

		m_Pics[j].m_picQP = 0;
		m_Pics[j].m_picLambda = 0.0;

		m_Pics[j].m_outputBit = 0;
		m_Pics[j].m_targetBit = 0;

		m_Pics[j].m_intraSize = m_intraSize;

		m_Pics[j].m_pcRateCtrl = this;

		m_Pics[j].m_picPara.m_alpha = 0;
		m_Pics[j].m_picPara.m_beta = 0;
		m_Pics[j].m_picPara.m_fromWhichPOC = 0;
		m_Pics[j].m_picPara.m_NofUpdate = 0;

		m_Pics[j].m_remainingCostIntra = new Double[m_numTile];
		m_Pics[j].m_LCULeft = new Int[m_numTile];
		m_Pics[j].m_bitsLeft = new Int[m_numTile];
		m_Pics[j].m_width_tile = new Int[m_numTile];
		m_Pics[j].m_height_tile = new Int[m_numTile];
		m_Pics[j].m_numberOfLCUinTile = new Int[m_numTile];
		m_Pics[j].m_numberOfPixel = new Int[m_numTile];
		m_Pics[j].m_pixelsLeft = new Int[m_numTile];

		m_Pics[j].m_LCUs = new TRCLCU[m_Pics[j].m_numberOfLCU];
		m_Pics[j].m_LCUPara = new TRCParameter[m_Pics[j].m_numberOfLCU];
		for (Int tileIdx = 0; tileIdx < m_numTile; tileIdx++)
		{
			m_Pics[j].m_remainingCostIntra[tileIdx] = 0;
			m_Pics[j].m_width_tile[tileIdx] = 0;
			m_Pics[j].m_height_tile[tileIdx] = 0;
			m_Pics[j].m_numberOfPixel[tileIdx] = 0;
		}
		for (Int x = 0; x < picWidthInBU; x++)
		{
			for (Int y = 0; y < picHeightInBU; y++)
			{
				LCUIdx = y*picWidthInBU + x;
				m_Pics[j].m_LCUs[LCUIdx].m_actualBits = 0;
				m_Pics[j].m_LCUs[LCUIdx].m_QP = 0;
				m_Pics[j].m_LCUs[LCUIdx].m_lambda = 0.0;
				m_Pics[j].m_LCUs[LCUIdx].m_targetBits = 0;
				m_Pics[j].m_LCUs[LCUIdx].m_bitWeight = 1.0;
				Int currWidth = ((x == picWidthInBU - 1) ? picWidth - LCUWidth *(picWidthInBU - 1) : LCUWidth);
				Int currHeight = ((y == picHeightInBU - 1) ? picHeight - LCUHeight*(picHeightInBU - 1) : LCUHeight);
				m_Pics[j].m_LCUs[LCUIdx].m_numberOfPixel = currWidth * currHeight;

				m_Pics[j].m_LCUPara[LCUIdx].m_alpha = 0;
				m_Pics[j].m_LCUPara[LCUIdx].m_beta = 0;
				m_Pics[j].m_LCUPara[LCUIdx].m_fromWhichPOC = 0;
				m_Pics[j].m_LCUPara[LCUIdx].m_NofUpdate = 0;

				m_Pics[j].m_width_tile[m_tileIdxMap[LCUIdx]] += currWidth;
				m_Pics[j].m_height_tile[m_tileIdxMap[LCUIdx]] += currHeight;
				m_Pics[j].m_numberOfPixel[m_tileIdxMap[LCUIdx]] += currWidth * currHeight;
			}
		}

		for (Int yTile = 0; yTile < numTileRow; yTile++)
		{
			for (Int xTile = 0; xTile < numTileCol; xTile++)
			{
				m_Pics[j].m_bitsLeft[yTile*numTileCol + xTile] = 0;				  
				m_Pics[j].m_pixelsLeft[yTile*numTileCol + xTile] = m_Pics[j].m_numberOfPixel[yTile*numTileCol + xTile];				  
				m_Pics[j].m_numberOfLCUinTile[yTile*numTileCol + xTile] = tileColWidth[xTile] * tileRowHeight[yTile];
				m_Pics[j].m_LCULeft[yTile*numTileCol + xTile] = tileColWidth[xTile] * tileRowHeight[yTile];
			}
		}
  }


  if (m_useLCUSeparateModel)
  {
	  m_LCUPara = new TRCParameter*[m_numFrameLevel];
	  for (Int i = 0; i < m_numFrameLevel; i++)
	  {
		  m_LCUPara[i] = new TRCParameter[m_Pics[0].m_numberOfLCU];
		  for (Int j = 0; j < m_Pics[0].m_numberOfLCU; j++)
		  {
			  m_LCUPara[i][j].m_alpha = m_picPara[i].m_alpha;
			  m_LCUPara[i][j].m_beta = m_picPara[i].m_beta;
			  m_LCUPara[i][j].m_fromWhichPOC = 0;
			  m_LCUPara[i][j].m_NofUpdate = 0;
		  }
	  }
  }

#if KAIST_USEPREPS
  m_costPOC = new Double [m_intraSize];
  for (Int j = 0; j < m_intraSize; j++)
	  m_costPOC[j] = 0;
  m_CostGOP = new Double[m_intraSize / GOPSize];
  for (Int i = 0; i < m_intraSize / GOPSize; i++)
	  m_CostGOP[i] = 0;
  m_CostIDR = 0;

  int CodingOrder[32] = { 0, 8, 4, 2, 6, 1, 3, 5, 7, 16, 12, 10, 14, 9, 11, 13, 15, 24, 20, 18, 22, 17, 19, 21, 23, 28, 26, 30, 25, 27, 29, 31 };
  char rname[64] = { 0, };
  memcpy(rname, inputFile, 3);
  sprintf(&rname[3], "_results.bin\0");
  FILE *pCost;
  pCost = fopen(rname, "rb");   // cost file opened

  unsigned short results[32][2][2040];
  unsigned short costCTU[32][360] = { 0, };
  Int Sliceidx = 0;
  for (int k = 0; k < 32; k++)
  {
	  fread(results[CodingOrder[k]][0], sizeof(unsigned short), 2040, pCost); // costs read
	  if (k % 32 != 0)
		  fread(results[CodingOrder[k]][1], sizeof(unsigned short), 2040, pCost); // MV read
  }
  switch (inputFile[7])
  {
  case '1':
	  Sliceidx = 1;
	  for (Int k = 0; k < 32; k++)
	  {
		  memcpy(costCTU[k], results[k][0], sizeof(unsigned short)* 360);
		  for (Int i = 0; i < 360; i++)
			  m_costPOC[k] += costCTU[k][i];
	  }
	  break;
  case '2':
	  Sliceidx = 2;
	  for (Int k = 0; k < 32; k++)
	  {
		  memcpy(costCTU[k], &results[k][0][360], sizeof(unsigned short)* 360);
		  for (Int i = 0; i < 360; i++)
			  m_costPOC[k] += costCTU[k][i];
	  }
	  break;
  case '3':
	  Sliceidx = 3;
	  for (Int k = 0; k < 32; k++)
	  {
		  memcpy(costCTU[k], &results[k][0][360 * 2], sizeof(unsigned short)* 360);
		  for (Int i = 0; i < 360; i++)
			  m_costPOC[k] += costCTU[k][i];
	  }
	  break;
  case '4':
	  Sliceidx = 4;
	  for (Int k = 0; k < 32; k++)
	  {
		  memcpy(costCTU[k], &results[k][0][360 * 3], sizeof(unsigned short)* 360);
		  for (Int i = 0; i < 360; i++)
			  m_costPOC[k] += costCTU[k][i];
	  }
	  break;
  case '5':
	  Sliceidx = 5;
	  for (Int k = 0; k < 32; k++)
	  {
		  memcpy(costCTU[k], &results[k][0][360 * 4], sizeof(unsigned short)* 360);
		  for (Int i = 0; i < 360; i++)
			  m_costPOC[k] += costCTU[k][i];
	  }
	  break;
  case '6':
	  Sliceidx = 6;
	  for (Int k = 0; k < 32; k++)
	  {
		  memcpy(costCTU[k], &results[k][0][360 * 5], sizeof(unsigned short)* 240);
		  for (Int i = 0; i < 240; i++)
			  m_costPOC[k] += costCTU[k][i];
	  }
	  break;
  }

  for (Int i = 0; i < m_intraSize; i++)
	  m_CostIDR += m_costPOC[i];
  for (Int i = 1; i < m_intraSize; i++)
	  m_CostGOP[(i-1) / GOPSize] += m_costPOC[i];

#if KAIST_SCENECHANGE
  m_iSceneChange = 0;
  Int testPOC[5] = { 16, 24, 28, 30, 31 };
  for (Int i = 0; i< 5; i++)
  {
	  if (m_costPOC[testPOC[i]] > 5 * m_costPOC[testPOC[i] - 8])
		  m_iSceneChange = testPOC[i];
  }
#endif

#endif
}

Int		TEncRateCtrl::xEstimateVirtualBuffer(Int iIDRModulus)
{
	Int estimatedCpbFullness = 0;
	Int encOrder2POC[32] = { 0, 8, 4, 2, 1, 3, 6, 5, 7, 16, 12, 10, 9, 11, 14, 13, 15, 24, 20, 18, 17, 19, 22, 21, 23, 28, 26, 25, 27, 30, 29, 31 };// 32 between 23 and 28
	Int poc2encOrder[32] = { 0, 4, 3, 5, 2, 7, 6, 8, 1, 12, 11, 13, 10, 15, 14, 16, 9, 20, 19, 21, 18, 23, 22, 24, 17, 27, 26, 28, 25, 30, 29, 31 };

	Int index4hrd = poc2encOrder[iIDRModulus];

	Int	bits = m_sliceActualBits[iIDRModulus];
	Int lastBits = 0;
	for (Int i = 0; i < index4hrd; i++)
	{
		if (m_sliceActualBits[i])
			lastBits += m_sliceActualBits[i];
		else
			lastBits += m_sliceTotalTargetBits[i];
	}
	estimatedCpbFullness = m_cpbState[0] - lastBits - bits + (index4hrd + 1)*m_bufferingRate;

	// prevent overflow
	if (estimatedCpbFullness - m_sliceTotalTargetBits[iIDRModulus] >(Int)(m_cpbSize*0.9f)) // iIDRModulus == poc2enc[index4hrd+1]
	{
		m_sliceTotalTargetBits[iIDRModulus] = estimatedCpbFullness - (Int)(m_cpbSize*0.9f);
	}

	estimatedCpbFullness -= m_bufferingRate;
	// prevent underflow
	if (estimatedCpbFullness - m_sliceTotalTargetBits[iIDRModulus] < (Int)(m_cpbSize*0.1f))
	{
		m_sliceTotalTargetBits[iIDRModulus] = max(200, estimatedCpbFullness - (Int)(m_cpbSize*0.1f));
	}

	estimatedCpbFullness += m_bufferingRate;

	//assign desired bits for the below frames
	Int RCFlag = 0;
	if (RCFlag)
	{
		switch (RCFlag)
		{
		case 7:
			m_sliceTotalTargetBits[iIDRModulus] = estimatedCpbFullness - (Int)(m_cpbSize*0.225f);
			break;
		case 15:
			m_sliceTotalTargetBits[iIDRModulus] = estimatedCpbFullness - (Int)(m_cpbSize*0.45f);
			break;
		case 23:
			m_sliceTotalTargetBits[iIDRModulus] = estimatedCpbFullness - (Int)(m_cpbSize*0.675f);
			break;
		case 31:
			m_sliceTotalTargetBits[iIDRModulus] = estimatedCpbFullness - (Int)(m_cpbSize*0.9f);
			break;
		default:
			break;
		}
	}

	return m_sliceTotalTargetBits[iIDRModulus];
}



//picture level

TRCPic::TRCPic()
{
	m_LCUs = NULL;
}

TRCPic::~TRCPic()
{
// 	if (m_LCUs != NULL)
// 	{
// 		delete [] m_LCUs;
// 		m_LCUs = NULL;
// 	}
}

Int TRCPic::calcFrameTargetBit(Int iPOC)//GOP size 8, intra size 32에 대해서만 작동함 hard coding
{
	Int intraSize = m_pcRateCtrl->getIntraSize();

	Int iIDRModulus = iPOC % intraSize;
	Int remainBits = 0;
	Int frameTargetBit = 0;
	Int GOPTargetBit = 0;
	Int BSliceTargetBit = 0;//target bits for all of B slices within an IDR period

	Int GOPSize = 8;
#if !KAIST_USEPREPS
	if (iIDRModulus == 0)
	{
		remainBits = m_pcRateCtrl->getTargetBitsForIDR();
		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits / intraSize); // 1.0 by default
		return frameTargetBit;
	}
#else
	Int GOPidx = (iIDRModulus - 1) / GOPSize;
	Double* costPOC = m_pcRateCtrl->m_costPOC;
	Double* CostGOP = m_pcRateCtrl->m_CostGOP;
	Double CostIDR = m_pcRateCtrl->m_CostIDR;
	if (iIDRModulus == 0)
	{
		remainBits = m_pcRateCtrl->getTargetBitsForIDR();
		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits * costPOC[0]/ CostIDR); // 1.0 by default
		return frameTargetBit;
	}
#endif


	BSliceTargetBit = m_pcRateCtrl->getTargetBitsForIDR() - m_pcRateCtrl->m_sliceActualBits[0];
	Int* weight4bpp = m_pcRateCtrl->getBitRatio();
	
	Int GOPtype = 0; // 0: include level 1, 1: not include level 1
#if !KAIST_USEPREPS
	if (iIDRModulus > 24)
	{
		GOPTargetBit = (Int)((weight4bpp[3] * 1.0 / weight4bpp[0])*BSliceTargetBit);
		GOPtype = 1;
	}
	else
		GOPTargetBit = (Int)((weight4bpp[1] * 1.0 / weight4bpp[0])*BSliceTargetBit);
#else
	if (iIDRModulus > 24)
		GOPtype = 1;
	GOPTargetBit = (Int)((CostGOP[GOPidx] / (CostIDR - costPOC[0]))*BSliceTargetBit);
#endif

	Double remainCost = 0;
	Double currCost = 0;
	switch (iIDRModulus % 8)
	{
	case 0:
		remainBits = GOPTargetBit;
#if !KAIST_USEPREPS
		remainCost = Double(weight4bpp[1]);
		currCost = Double(weight4bpp[2]);
#else
		remainCost = CostGOP[GOPidx];
		currCost = costPOC[iIDRModulus];
#endif

		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;
	case 4:
		if (GOPtype == 0)
		{
			remainBits = GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 4];
#if !KAIST_USEPREPS
			remainCost = Double(weight4bpp[3]);
			currCost = Double(weight4bpp[4]);
#else
			remainCost = CostGOP[GOPidx] - costPOC[iIDRModulus + 4];
			currCost = costPOC[iIDRModulus];
#endif
		}
		else
		{
			remainBits = GOPTargetBit;
#if !KAIST_USEPREPS
			remainCost = Double(weight4bpp[3]);
			currCost = Double(weight4bpp[4]);
#else
			remainCost = CostGOP[GOPidx];
			currCost = costPOC[iIDRModulus];
#endif
		}

		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;
	case 2:
		if (GOPtype == 0)
		{
			remainBits = GOPTargetBit - (m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 2] + m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 6]);
#if !KAIST_USEPREPS
			remainCost = 12.0;
			currCost = 4.0;
#else
			remainCost = CostGOP[GOPidx] - costPOC[iIDRModulus + 2] - costPOC[iIDRModulus + 6];
			currCost = costPOC[iIDRModulus];
#endif
		}
		else
		{
			remainBits = GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 2];
#if !KAIST_USEPREPS
			remainCost = 12.0;
			currCost = 4.0;
#else
			remainCost = CostGOP[GOPidx] - costPOC[iIDRModulus + 2];
			currCost = costPOC[iIDRModulus];
#endif
		}

		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;
	case 6:
		if (GOPtype == 0)
		{
			remainBits = GOPTargetBit - (m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 2] + m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 2]);
#if !KAIST_USEPREPS
			remainCost = 12.0;
			currCost = 4.0;
#else
			remainCost = CostGOP[GOPidx] - costPOC[iIDRModulus - 2] - costPOC[iIDRModulus + 2];
			currCost = costPOC[iIDRModulus];
#endif
		}
		else
		{
			remainBits = GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 2];
#if !KAIST_USEPREPS
			remainCost = 12.0;
			currCost = 4.0;
#else
			remainCost = CostGOP[GOPidx] - costPOC[iIDRModulus - 2];
			currCost = costPOC[iIDRModulus];
#endif
		}

		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;
	case 1:
#if !KAIST_USEPREPS
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - (m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 7] + m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 3])) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 3]) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);

		remainCost = 2.0;
		currCost = 1.0;
#else
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 7] - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 3]) * (costPOC[GOPSize * GOPidx + 1] + costPOC[GOPSize * GOPidx + 2] + costPOC[GOPSize * GOPidx + 3]) / (CostGOP[GOPidx] -costPOC[iIDRModulus + 7] - costPOC[iIDRModulus +3]) - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 3]) * (costPOC[GOPSize * GOPidx + 1] + costPOC[GOPSize * GOPidx + 2] + costPOC[GOPSize * GOPidx + 3]) / (CostGOP[GOPidx] - costPOC[iIDRModulus+3] )- m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);

		remainCost = costPOC[GOPSize * GOPidx + 1] + costPOC[GOPSize * GOPidx + 3];
		currCost = costPOC[iIDRModulus];
#endif

		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;
	case 3:
#if !KAIST_USEPREPS
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - (m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 5] + m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1])) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);

		remainCost = 2.0;
		currCost = 1.0;
#else
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 5] - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]) * (costPOC[GOPSize * GOPidx + 1] + costPOC[GOPSize * GOPidx + 2] + costPOC[GOPSize * GOPidx + 3]) / (CostGOP[GOPidx] - costPOC[iIDRModulus + 5] - costPOC[iIDRModulus + 1]) - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]) * (costPOC[GOPSize * GOPidx + 1] + costPOC[GOPSize * GOPidx + 2] + costPOC[GOPSize * GOPidx + 3]) / (CostGOP[GOPidx] - costPOC[iIDRModulus + 1]) - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);

		remainCost = costPOC[GOPSize * GOPidx + 1] + costPOC[GOPSize * GOPidx + 3];
		currCost = costPOC[iIDRModulus];
#endif
		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;
	case 5:
#if !KAIST_USEPREPS
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - (m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 3] + m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1])) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);

		remainCost = 2.0;
		currCost = 1.0;
#else
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 3] - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]) * (costPOC[GOPSize * GOPidx + 5] + costPOC[GOPSize * GOPidx + 6] + costPOC[GOPSize * GOPidx + 7]) / (CostGOP[GOPidx] - costPOC[iIDRModulus + 3] - costPOC[iIDRModulus - 1]) - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]) * (costPOC[GOPSize * GOPidx + 5] + costPOC[GOPSize * GOPidx + 6] + costPOC[GOPSize * GOPidx + 7]) / (CostGOP[GOPidx] - costPOC[iIDRModulus - 1]) - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1]);

		remainCost = costPOC[GOPSize * GOPidx + 5] + costPOC[GOPSize * GOPidx + 7];
		currCost = costPOC[iIDRModulus];
#endif

		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;
	case 7:
#if !KAIST_USEPREPS
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - (m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1] + m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 3])) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 3]) / 2.0 - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);

		remainCost = 2.0;
		currCost = 1.0;
#else
		if (GOPtype == 0)
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus + 1] - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 3]) * (costPOC[GOPSize * GOPidx + 5] + costPOC[GOPSize * GOPidx + 6] + costPOC[GOPSize * GOPidx + 7]) / (CostGOP[GOPidx] - costPOC[iIDRModulus + 1] - costPOC[iIDRModulus - 3]) - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);
		else
			remainBits = (Int)((GOPTargetBit - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 3]) * (costPOC[GOPSize * GOPidx + 5] + costPOC[GOPSize * GOPidx + 6] + costPOC[GOPSize * GOPidx + 7]) / (CostGOP[GOPidx] - costPOC[iIDRModulus - 3]) - m_pcRateCtrl->m_sliceActualBits[iIDRModulus - 1]);

		remainCost = costPOC[GOPSize * GOPidx + 5] + costPOC[GOPSize * GOPidx + 7];
		currCost = costPOC[iIDRModulus];
#endif

		if (remainBits < 0) remainBits = 0;
		frameTargetBit = (Int)(remainBits*currCost / remainCost);
		break;

	}

	return frameTargetBit;
}

Int TRCPic::getRefineBitsForIntra(Int orgBits)
{
#if ETRI_MULTITHREAD_2
	Int iIDRIndex = m_POC / m_intraSize;
#endif
	Double alpha=0.25, beta=0.5582;
	Int iIntraBits;

	if (orgBits*40 < m_pcRateCtrl->m_numberOfPixel)
	{
		alpha=0.25;
	}
	else
	{
		alpha=0.30;
	}

	iIntraBits = (Int)(alpha* pow(m_totalCostIntra*4.0/(Double)orgBits, beta)*(Double)orgBits+0.5);

	return iIntraBits;
}

Void TRCPic::getLCUInitTargetBits(Int bits)
{
	Int numTile = m_pcRateCtrl->m_numTile;

	Int* iAvgBits = new Int[numTile];
	for (Int i = 0; i < numTile; i++)
	{
		iAvgBits[i] = 0;
		m_remainingCostIntra[i] = 0;
	}

	Int* tileMap = m_pcRateCtrl->m_tileIdxMap;

	for (Int i=m_numberOfLCU-1; i>=0; i--)
	{
		iAvgBits[tileMap[i]] += Int(bits * getLCU(i).m_costIntra / m_totalCostIntra);
		getLCU(i).m_targetBitsLeft = iAvgBits[tileMap[i]];
		m_remainingCostIntra[tileMap[i]] += getLCU(i).m_costIntra;
	}

	delete[] iAvgBits;
}

Double TRCPic::estimatePicLambda(SliceType eSliceType)
{
	Double alpha = m_pcRateCtrl->getPicPara(m_frameLevel).m_alpha;
	Double beta = m_pcRateCtrl->getPicPara(m_frameLevel).m_beta;
	Int fromWhichPOC = m_pcRateCtrl->getPicPara(m_frameLevel).m_fromWhichPOC;
	Int NofUpdate = m_pcRateCtrl->getPicPara(m_frameLevel).m_NofUpdate;

	Bool bSceneChange = false;
	if (bSceneChange)
	{
		if (m_frameLevel > 0)
		{
			alpha = 3.2003;
			beta = -1.367;
		}
		else
		{
			alpha = ALPHA;
			beta = BETA2;
		}
	}

	if (NofUpdate == 0)
	{
		Bool bNotUseInit = m_frameLevel > 1 ? true : false;
		if (bNotUseInit)
		{
			Double alphaInher = m_pcRateCtrl->getPicPara(m_frameLevel - 1).m_alpha;
			Double betaInher = m_pcRateCtrl->getPicPara(m_frameLevel - 1).m_beta;
			alpha = 0.95*alphaInher;
			beta = 0.95*betaInher;
		}
	}

	TRCParameter rcPara;
	rcPara.m_alpha = alpha;
	rcPara.m_beta = beta;
	rcPara.m_fromWhichPOC = fromWhichPOC;
	rcPara.m_NofUpdate = NofUpdate;
	setPicPara(rcPara);

	Double bpp = (Double)m_targetBit / (Double)m_pcRateCtrl->m_numberOfPixel;
	Double estLambda;
	
	if (eSliceType == I_SLICE)
	{
		alpha = ALPHA;
		beta = BETA2;
		estLambda = calculateLambdaIntra(alpha, beta, pow(m_totalCostIntra / (Double)m_pcRateCtrl->m_numberOfPixel, BETA1), bpp);
	}
	else
	{
		estLambda = alpha * pow(bpp, beta);
	}
	estLambda = Clip3(0.1, 10000.0, estLambda);
	m_picLambda = estLambda;

	Double totalWeight = 0.0;
	// initial BU bit allocation weight
	for (Int i = 0; i < m_numberOfLCU; i++)
	{
		Double alphaLCU, betaLCU;
		if (m_pcRateCtrl->getUseLCUSeparateModel())
		{
			alphaLCU = m_pcRateCtrl->getLCUPara(m_frameLevel, i).m_alpha;
			betaLCU = m_pcRateCtrl->getLCUPara(m_frameLevel, i).m_beta;
		}
		else
		{
			alphaLCU = m_pcRateCtrl->getPicPara(m_frameLevel).m_alpha;
			betaLCU = m_pcRateCtrl->getPicPara(m_frameLevel).m_beta;
		}

		m_LCUs[i].m_bitWeight = m_LCUs[i].m_numberOfPixel * pow(estLambda / alphaLCU, 1.0 / betaLCU);

		if (m_LCUs[i].m_bitWeight < 0.01)
		{
			m_LCUs[i].m_bitWeight = 0.01;
		}
		totalWeight += m_LCUs[i].m_bitWeight;
	}
	for (Int i = 0; i < m_numberOfLCU; i++)
	{
		Double BUTargetBits = m_targetBit * m_LCUs[i].m_bitWeight / totalWeight;
		m_LCUs[i].m_bitWeight = BUTargetBits;
	}

	return estLambda;
}

Double TRCPic::calculateLambdaIntra(double alpha, double beta, double MADPerPixel, double bitsPerPixel)
{
	return ( (alpha/256.0) * pow( MADPerPixel/bitsPerPixel, beta ) );
}

Int TRCPic::estimatePicQP(Double lambda, TComSlice* pcSlice)
{
	Int QP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 ); 
	QP = Clip3(MIN_QP, MAX_QP, QP);
	QP = Clip3(-pcSlice->getSPS()->getQpBDOffsetY(), MAX_QP, QP);
	m_picQP = QP;
	return QP;
}

Double TRCPic::getLCUTargetBpp(Int   LCUIdx, SliceType eSliceType)
{
	Double bpp      = -1.0;
	Int avgBits     = 0;

	Int* tileMap = m_pcRateCtrl->m_tileIdxMap;

	if (eSliceType == I_SLICE)
	{
		Int noOfLCUsLeft = m_LCULeft[tileMap[LCUIdx]];
		Int bitrateWindow = min(4,noOfLCUsLeft);
		Double MAD      = getLCU(LCUIdx).m_costIntra;

		if (m_remainingCostIntra[tileMap[LCUIdx]] > 0.1)
		{
			Double weightedBitsLeft = (m_bitsLeft[tileMap[LCUIdx]] * bitrateWindow + (m_bitsLeft[tileMap[LCUIdx]] - getLCU(LCUIdx).m_targetBitsLeft)*noOfLCUsLeft) / (Double)bitrateWindow;
			avgBits = Int(MAD*weightedBitsLeft / m_remainingCostIntra[tileMap[LCUIdx]]);
		}
		else
		{
			avgBits = Int(m_bitsLeft[tileMap[LCUIdx]] / m_LCULeft[tileMap[LCUIdx]]);
		}
		m_remainingCostIntra[tileMap[LCUIdx]] -= MAD;
	}
	else
	{
		Double totalWeight = 0;
		for (Int i = LCUIdx; i < m_numberOfLCU; i++)
		{
			if (tileMap[LCUIdx] == tileMap[i])
				totalWeight += m_LCUs[i].m_bitWeight;
		}
		Int realInfluenceLCU = min(g_RCLCUSmoothWindowSize, m_LCULeft[tileMap[LCUIdx]]);
		avgBits = (Int)(m_LCUs[LCUIdx].m_bitWeight - (totalWeight - m_bitsLeft[tileMap[LCUIdx]]) / realInfluenceLCU + 0.5);
	}

	if (avgBits < 1)
	{
		avgBits = 1;
	}

	bpp = (Double)avgBits / (Double)m_LCUs[LCUIdx].m_numberOfPixel;
	m_LCUs[LCUIdx].m_targetBits = avgBits;

	return bpp;
}

Double TRCPic::getLCUEstLambdaAndQP(Int   LCUIdx, Double bpp, Int clipPicQP, Int *estQP)
{
	
	Double   alpha = getPicPara().m_alpha;
	Double   beta  = getPicPara().m_beta;

	Double costPerPixel = getLCU(LCUIdx).m_costIntra/(Double)getLCU(LCUIdx).m_numberOfPixel;
	costPerPixel = pow(costPerPixel, BETA1);
	Double estLambda = calculateLambdaIntra(alpha, beta, costPerPixel, bpp);

	Int clipNeighbourQP = g_RCInvalidQPValue;
	for (int i=LCUIdx-1; i>=0; i--)
	{
		if ((getLCU(i)).m_QP > g_RCInvalidQPValue)
		{
			clipNeighbourQP = getLCU(i).m_QP;
			break;
		}
	}

	Int minQP = clipPicQP - 2;
	Int maxQP = clipPicQP + 2;

	if (clipNeighbourQP > g_RCInvalidQPValue)
	{
		maxQP = min(clipNeighbourQP + 1, maxQP);
		minQP = max(clipNeighbourQP - 1, minQP);
	}

	Double maxLambda = exp(((Double)(maxQP + 0.49) - 13.7122) / 4.2005);
	Double minLambda = exp(((Double)(minQP - 0.49) - 13.7122) / 4.2005);

	estLambda = Clip3(minLambda, maxLambda, estLambda);

	*estQP = Int(4.2005 * log(estLambda) + 13.7122 + 0.5);
	*estQP = Clip3(minQP, maxQP, *estQP);

	return estLambda;
}

Double TRCPic::getLCUEstLambda(Int   LCUIdx, Double bpp)
{
	Double alpha;
	Double beta;
	Int NofUpdate;
	Int fromWhichPOC;
	if (m_pcRateCtrl->getUseLCUSeparateModel())
	{
		alpha = getLCUPara( LCUIdx).m_alpha;
		beta = getLCUPara( LCUIdx).m_beta;
		NofUpdate = getLCUPara(LCUIdx).m_NofUpdate;
		fromWhichPOC = getLCUPara(LCUIdx).m_fromWhichPOC;
	}
	else
	{
		alpha = getPicPara( ).m_alpha;
		beta  = getPicPara( ).m_beta;
		NofUpdate = getPicPara( ).m_NofUpdate;
		fromWhichPOC = getPicPara( ).m_fromWhichPOC;
	}
	TRCParameter rcPara;
	rcPara.m_alpha = alpha;
	rcPara.m_beta = beta;
	rcPara.m_NofUpdate = NofUpdate;
	rcPara.m_fromWhichPOC = fromWhichPOC;
	setLCUPara(LCUIdx, rcPara);

	Double estLambda = alpha * pow( bpp, beta );
	//for Lambda clip, picture level clip
	Double clipPicLambda = m_picLambda;

	//for Lambda clip, LCU level clip
	Double clipNeighbourLambda = -1.0;
	for (int i = LCUIdx - 1; i >= 0; i--)
	{
		if (m_LCUs[i].m_lambda > 0)
		{
			clipNeighbourLambda = m_LCUs[i].m_lambda;
			break;
		}
	}

	if (clipNeighbourLambda > 0.0)
	{
		estLambda = Clip3(clipNeighbourLambda * pow(2.0, -1.0 / 3.0), clipNeighbourLambda * pow(2.0, 1.0 / 3.0), estLambda);
	}

	if (clipPicLambda > 0.0)
	{
		estLambda = Clip3(clipPicLambda * pow(2.0, -2.0 / 3.0), clipPicLambda * pow(2.0, 2.0 / 3.0), estLambda);
	}
	else
	{
		estLambda = Clip3(10.0, 1000.0, estLambda);
	}

	if (estLambda < 0.1)
	{
		estLambda = 0.1;
	}

	return estLambda;
}

Int TRCPic::getLCUEstQP(Int LCUIdx, Double lambda, Int clipPicQP)
{
	Int estQP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 );

	//for Lambda clip, LCU level clip
	Int clipNeighbourQP = g_RCInvalidQPValue;
	for ( int i=LCUIdx - 1; i>=0; i-- )
	{
		if ( (getLCU(i)).m_QP > g_RCInvalidQPValue )
		{
			clipNeighbourQP = getLCU(i).m_QP;
			break;
		}
	}

	if ( clipNeighbourQP > g_RCInvalidQPValue )
	{
		estQP = Clip3( clipNeighbourQP - 1, clipNeighbourQP + 1, estQP );
	}

	estQP = Clip3(clipPicQP - 2, clipPicQP + 2, estQP);

	return estQP;
}

Void TRCPic::updateAfterLCU(Int LCUIdx, Int bits, Int QP, Double lambda, Bool updateLCUParameter)
{
	m_LCUs[LCUIdx].m_actualBits = bits;
	m_LCUs[LCUIdx].m_QP         = QP;
	m_LCUs[LCUIdx].m_lambda     = lambda;

	Int* tileMap = m_pcRateCtrl->m_tileIdxMap;

	m_LCULeft[tileMap[LCUIdx]]--;
	m_bitsLeft[tileMap[LCUIdx]] -= bits;
	m_pixelsLeft[tileMap[LCUIdx]] -= m_LCUs[LCUIdx].m_numberOfPixel;

	if ( !updateLCUParameter )
	{
		return;
	}

	if (!m_pcRateCtrl->getUseLCUSeparateModel())
	{
		return;
	}

	Double alpha = getLCUPara( LCUIdx).m_alpha;
	Double beta = getLCUPara( LCUIdx).m_beta;
	Int NofUpdate = getLCUPara(LCUIdx).m_NofUpdate;
	Int fromWhichPOC = getLCUPara(LCUIdx).m_fromWhichPOC;

	Int LCUActualBits = m_LCUs[LCUIdx].m_actualBits;
	Int LCUTotalPixels = m_LCUs[LCUIdx].m_numberOfPixel;
	Double bpp = (Double)LCUActualBits / (Double)LCUTotalPixels;
	Double calLambda = alpha * pow(bpp, beta);
	Double inputLambda = m_LCUs[LCUIdx].m_lambda;

	if (inputLambda < 0.01 || calLambda < 0.01 || bpp < 0.0001)
	{
		alpha *= (1.0 - m_pcRateCtrl->getAlphaUpdate() / 2.0);
		beta *= (1.0 - m_pcRateCtrl->getBetaUpdate() / 2.0);

		alpha = Clip3(g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha);
		beta = Clip3(g_RCBetaMinValue, g_RCBetaMaxValue, beta);
	}
	else
	{
		calLambda = Clip3(inputLambda / 10.0, inputLambda * 10.0, calLambda);
		alpha += m_pcRateCtrl->getAlphaUpdate() * (log(inputLambda) - log(calLambda)) * alpha;
		double lnbpp = log(bpp);
		lnbpp = Clip3(-5.0, -0.1, lnbpp);
		beta += m_pcRateCtrl->getBetaUpdate() * (log(inputLambda) - log(calLambda)) * lnbpp;

		alpha = Clip3(g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha);
		beta = Clip3(g_RCBetaMinValue, g_RCBetaMaxValue, beta);
	}
	TRCParameter rcPara;
	rcPara.m_alpha = alpha;
	rcPara.m_beta = beta;
	rcPara.m_fromWhichPOC = m_POC;
	rcPara.m_NofUpdate = getLCUPara(LCUIdx).m_NofUpdate + 1;
	if (m_pcRateCtrl->getLCUPara(m_frameLevel, LCUIdx).m_NofUpdate < rcPara.m_NofUpdate)
		m_pcRateCtrl->setLCUPara(m_frameLevel, LCUIdx, rcPara);
	else if ((m_pcRateCtrl->getLCUPara(m_frameLevel, LCUIdx).m_NofUpdate == rcPara.m_NofUpdate) && m_pcRateCtrl->getLCUPara(m_frameLevel, LCUIdx).m_fromWhichPOC < rcPara.m_fromWhichPOC)
		m_pcRateCtrl->setLCUPara(m_frameLevel, LCUIdx, rcPara);
}

Double TRCPic::calAverageQP()
{
	Int totalQPs = 0;
	Int numTotalLCUs = 0;

	Int i;
	for ( i=0; i<m_numberOfLCU; i++ )
	{
		if ( m_LCUs[i].m_QP > 0 )
		{
			totalQPs += m_LCUs[i].m_QP;
			numTotalLCUs++;
		}
	}

	Double avgQP = 0.0;
	if ( numTotalLCUs == 0 )
	{
		avgQP = g_RCInvalidQPValue;
	}
	else
	{
		avgQP = ((Double)totalQPs) / ((Double)numTotalLCUs);
	}
	return avgQP;
}

Double TRCPic::calAverageLambda()
{
	Double totalLambdas = 0.0;
	Int numTotalLCUs = 0;

	Int i;
	for ( i=0; i<m_numberOfLCU; i++ )
	{
		if ( m_LCUs[i].m_lambda > 0.01 )
		{
			totalLambdas += log( m_LCUs[i].m_lambda );
			numTotalLCUs++;
		}
	}

	Double avgLambda; 
	if( numTotalLCUs == 0 )
	{
		avgLambda = -1.0;
	}
	else
	{
		avgLambda = pow(2.7183, totalLambdas / numTotalLCUs);
	}
	return avgLambda;
}

Void TRCPic::updateAfterPicture(Int actualHeaderBits, Int actualTotalBits, Double averageQP, Double averageLambda, SliceType eSliceType)
{
	m_outputBit = actualTotalBits;
	if ( averageQP > 0.0 )
		m_picQP             = Int( averageQP + 0.5 );
	else
		m_picQP             = g_RCInvalidQPValue;
	m_picLambda           = averageLambda;

#if ETRI_MULTITHREAD_2
	Int iIDRIndex = m_POC / m_intraSize;
#endif

	Double alpha = getPicPara().m_alpha;
	Double beta = getPicPara().m_beta;
	Int fromWhichPOC = getPicPara().m_fromWhichPOC;
	Int NofUpdate = getPicPara().m_NofUpdate;

	if (eSliceType == I_SLICE)
	{
		updateAlphaBetaIntra(&alpha, &beta);
	}
	else
	{
		// update parameters
		Double picActualBits = (Double)m_outputBit;
		Double picActualBpp = picActualBits / (Double)m_pcRateCtrl->m_numberOfPixel;
		Double calLambda = alpha * pow(picActualBpp, beta);
		Double inputLambda = m_picLambda;

		if (inputLambda < 0.01 || calLambda < 0.01 || picActualBpp < 0.0001)
		{
			alpha *= (1.0 - m_pcRateCtrl->getAlphaUpdate() / 2.0);
			beta *= (1.0 - m_pcRateCtrl->getBetaUpdate() / 2.0);

			alpha = Clip3(g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha);
			beta = Clip3(g_RCBetaMinValue, g_RCBetaMaxValue, beta);
		}
		else
		{
			calLambda = Clip3(inputLambda / 10.0, inputLambda * 10.0, calLambda);
			alpha += m_pcRateCtrl->getAlphaUpdate() * (log(inputLambda) - log(calLambda)) * alpha;
			double lnbpp = log(picActualBpp);
			lnbpp = Clip3(-5.0, -0.1, lnbpp);
			beta += m_pcRateCtrl->getBetaUpdate() * (log(inputLambda) - log(calLambda)) * lnbpp;

			alpha = Clip3(g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha);
			beta = Clip3(g_RCBetaMinValue, g_RCBetaMaxValue, beta);
		}
	}

	TRCParameter rcPara;
	rcPara.m_alpha = alpha;
	rcPara.m_beta = beta;
	rcPara.m_fromWhichPOC = m_POC;
	rcPara.m_NofUpdate = getPicPara().m_NofUpdate + 1;


	if (m_pcRateCtrl->getPicPara(m_frameLevel).m_NofUpdate < rcPara.m_NofUpdate)
		m_pcRateCtrl->setPicPara(m_frameLevel, rcPara);
	else if ((m_pcRateCtrl->getPicPara(m_frameLevel).m_NofUpdate == rcPara.m_NofUpdate) && m_pcRateCtrl->getPicPara(m_frameLevel).m_fromWhichPOC < rcPara.m_fromWhichPOC)
		m_pcRateCtrl->setPicPara(m_frameLevel, rcPara);
}

Void TRCPic::updateAlphaBetaIntra(double *alpha, double *beta)
{
	Double lnbpp = log(pow(m_totalCostIntra / (Double)m_pcRateCtrl->m_numberOfPixel, BETA1));
	Double diffLambda = (*beta)*(log((Double)m_outputBit) - log((Double)m_targetBit));

	diffLambda = Clip3(-0.125, 0.125, 0.25*diffLambda);
	*alpha    =  (*alpha) * exp(diffLambda);
	*beta     =  (*beta) + diffLambda / lnbpp;
}

#endif


#if (ETRI_DLL_INTERFACE)
/**
----------------------------------------------------------------------
	@brief  DLL의 경우  Rate Control을 IDR 단위로 Restart 시키기 위한 함수
	@Param	bool bValue  	:   Restart Active [True] else [False] \n 
			int iTotalFrame :   Generally it is eaual to Total Frame	
	@Date	2014.9.19
	@Author Jinwuk Seok 
----------------------------------------------------------------------
*/
#if !KAIST_RC
#if 0
Void TEncRateCtrl::ETRI_setRCRestart(bool bValue, int iTotalFrame)
{
	em_bRCRestart = bValue;

	//Restart RC 
	if (em_bRCRestart)
	{	
		m_encRCSeq->ETRI_setFramesLeft(iTotalFrame);
		m_encRCSeq->ETRI_setBitsLeft(m_encRCSeq->getTargetBits());
	}
}
#endif
#endif
#endif
