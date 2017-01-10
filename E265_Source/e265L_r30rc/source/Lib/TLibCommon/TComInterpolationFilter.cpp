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

/**
 * \file
 * \brief Implementation of TComInterpolationFilter class
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "TComRom.h"
#include "TComInterpolationFilter.h"
#include <assert.h>


//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

#if ETRI_SIMD_INTERPOLATION
ALIGNED(16) const Short TComInterpolationFilter::m_lumaFilter[4][NTAPS_LUMA] =
{
	{ 0, 0, 0, 64, 0, 0, 0, 0 },    //abs 64  
	{ -1, 4, -10, 58, 17, -5, 1, 0 },  //abs 96
	{ -1, 4, -11, 40, 40, -11, 4, -1 },  //abs 112
	{ 0, 1, -5, 17, 58, -10, 4, -1 }    //abs 96
};

ALIGNED(16) const Short TComInterpolationFilter::m_chromaFilter[8][NTAPS_CHROMA] =
{
	{ 0, 64, 0, 0 },
	{ -2, 58, 10, -2 },
	{ -4, 54, 16, -2 },
	{ -6, 46, 28, -4 },
	{ -4, 36, 36, -4 },
	{ -4, 28, 46, -6 },
	{ -2, 16, 54, -4 },
	{ -2, 10, 58, -2 }
};
#else
const Short TComInterpolationFilter::m_lumaFilter[4][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0, 0,  0 },
  { -1, 4, -10, 58, 17,  -5, 1,  0 },
  { -1, 4, -11, 40, 40, -11, 4, -1 },
  {  0, 1,  -5, 17, 58, -10, 4, -1 }
};

const Short TComInterpolationFilter::m_chromaFilter[8][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -2, 58, 10, -2 },
  { -4, 54, 16, -2 },
  { -6, 46, 28, -4 },
  { -4, 36, 36, -4 },
  { -4, 28, 46, -6 },
  { -2, 16, 54, -4 },
  { -2, 10, 58, -2 }
};
#endif

// ====================================================================================================================
// ETRI Definition
// ====================================================================================================================
#if ETRI_SIMD_INTERPOLATION
#define  ETRI_SIMD_FILTERCOPY	1

TComInterpolationFilter::TComInterpolationFilter()
{
	init();
	
	//---------------------------------------------------------------------------
	//	ETRI Modification for V01 Revision
	//---------------------------------------------------------------------------
	//ESPRINTF( ETRI_MODIFICATION_V01, stderr, " Compiled @%s  [%s] \n", __DATE__, __TIME__);
}

TComInterpolationFilter::~TComInterpolationFilter()
{
	free(em_pfFilterFunc);
}

Void TComInterpolationFilter::init()
{
	em_pfFilterCopy[0] 	= &TComInterpolationFilter::ETRI_pFFilterCopy_00;
	em_pfFilterCopy[1] 	= &TComInterpolationFilter::ETRI_pFFilterCopy_01;
	em_pfFilterCopy[2] 	= &TComInterpolationFilter::ETRI_pFFilterCopy_02;

	em_pfFilterFunc = nullptr;
	em_pfFilterFunc = (FilterFunc *)calloc(ETRI_nStaticFunc, sizeof(FilterFunc));

	em_pfFilterFunc[0] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_4T8W;
	em_pfFilterFunc[1] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_4T8WLast;
	em_pfFilterFunc[2] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_4T4W;
	em_pfFilterFunc[3] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_4T4WLast;
	em_pfFilterFunc[4] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T8WR;
	em_pfFilterFunc[5] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T8WRLast;
	em_pfFilterFunc[6] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T8W;
	em_pfFilterFunc[7] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T8WLast;
	em_pfFilterFunc[8] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T4W;
	em_pfFilterFunc[9] 	= (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T4WLast;
	em_pfFilterFunc[10] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T2W;
	em_pfFilterFunc[11] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterV_8T2WLast;
	em_pfFilterFunc[12] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_4T8W;
	em_pfFilterFunc[13] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_4T8WLast;
	em_pfFilterFunc[14] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_4T4W;
	em_pfFilterFunc[15] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_4T4WLast;
	em_pfFilterFunc[16] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T8WR;
	em_pfFilterFunc[17] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T8WRLast;
	em_pfFilterFunc[18] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T8W;
	em_pfFilterFunc[19] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T8WLast;
	em_pfFilterFunc[20] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T4W;
	em_pfFilterFunc[21] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T4WLast;
	em_pfFilterFunc[22] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T2W;
	em_pfFilterFunc[23] = (FilterFunc)&TComInterpolationFilter::ETRI_FilterH_8T2WLast;

}
#else
#define  ETRI_SIMD_FILTERCOPY  0
#endif

// ====================================================================================================================
// SIMD Instructions for Bit Limitation
/**
	xmm0 		: Input/Output Data
	xmm_mx 	: Vector of MaxVal
	xmm_z 		: Vector of MinVal 	= 0
*/
// ====================================================================================================================
 ///< Support 8 and 10 bit Clipping : 2015 8 7 by Seok
#define	ETRI_CLIP_BITLIMIT(xmm0, xmm_mx, xmm_z ) \
	xmm0 = _mm_min_epi16(xmm0, xmm_mx); \
	xmm0 = _mm_max_epi16(xmm0, xmm_z); 

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterCopy(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast)
{
#if !ETRI_SIMD_INTERPOLATION
  Int row, col;
#if ETRI_SIMD_FILTERCOPY
	__m128i xmm0, xmm1, xmm2;
	__m128i xmm_mx, xmm_z;	///< 2015 8 6 by Seok

	int 	remainder, awidth, idx;
//	EDPRINTF(stderr, "filterCopy ; width: %d \n", width);
	ALIGNED(16) Short est[8];
	if ( isFirst == isLast )
	{
		///< 2015 8 5 by Seok : Confirmed Code 
		if (width < 4)
		{
			for (row = 0; row < height; row++)
			{
				*(int *)dst = *(int *)src;		//just width = 2 case
				src += srcStride;
				dst += dstStride;
			}			 
		}
		else if (width < 8)
		{
			for (row = 0; row < height; row++)
			{
				*(Int64 *)dst = *(Int64 *)src;	//just 4 case 
				src += srcStride;
				dst += dstStride;
			} 			 
		}
		else
		{
			for (row = 0; row < height; row++)
			{
				for(col=0; col<width; col+=8)
				{
					xmm0 = _mm_loadu_si128((__m128i const *)(&src[col]));
					_mm_storeu_si128((__m128i *)(&dst[col]), xmm0);
				}
				src += srcStride;
				dst += dstStride;
			} 			 
		}
	}
	else if ( isFirst )
	{
		Short shift = IF_INTERNAL_PREC - bitDepth;
		Short val; 
		xmm1 = _mm_set1_epi16((Short)IF_INTERNAL_OFFS);
		if (width < 4)
		{
			///< 2015 8 5 by Seok : Confirmed Code 
			remainder = width & 0x01;
			for (row = 0; row < height; row++)
			{
				for (col = 0; col < width; col++)
				{
					val = src[col] << shift;
					dst[col] = val - (Short)IF_INTERNAL_OFFS;
				}
				src += srcStride;
				dst += dstStride;
			}		   
		}
		else if (width < 8)
		{
			///< 2015 8 5 by Seok : Confirmed Code 
			remainder = width & 0x03;	// width : 4<= width < 8
			if (remainder)
			{
				for (row = 0; row < height; row++)
				{
 					xmm0 = _mm_loadu_si128((__m128i const *)src);
					xmm0 = _mm_slli_epi16(xmm0, (short)shift);
					xmm0 = _mm_sub_epi16(xmm0, xmm1);
					_mm_store_si128((__m128i *)est, xmm0);
					*(Int64 *)dst = *(Int64 *)est;
  
					for(idx=0; idx<remainder; idx++)
						dst[4+idx] = est[4+idx];
 					src += srcStride;
					dst += dstStride;
				}		 
			}
			else
			{
				///< 2015 8 5 by Seok : Confirmed Code 
				for (row = 0; row < height; row++)
				{
 					xmm0 = _mm_loadu_si128((__m128i const *)src);
					xmm0 = _mm_slli_epi16(xmm0, (short)shift);		///< val = src[col] << shift; 
					xmm0 = _mm_sub_epi16(xmm0, xmm1);			///< dst[col] = val - (Short)IF_INTERNAL_OFFS; 
					_mm_store_si128((__m128i *)est, xmm0);
					*(Int64 *)dst = *(Int64 *)est;
 					src += srcStride;
					dst += dstStride;
				}		 
			}
		}
		else
		{
			///< 2015 8 5 by Seok : Confirmed Code 
			remainder = width & 0x07;	
			awidth      = width - remainder;
			if (remainder)
			{
				for (row = 0; row < height; row++)
				{
					for(col=0; col<awidth; col+=8)
					{
						xmm0 = _mm_loadu_si128((__m128i const *)(src+col));
						xmm0 = _mm_slli_epi16(xmm0, (short)shift);
						xmm0 = _mm_sub_epi16(xmm0, xmm1);
						_mm_storeu_si128((__m128i *)(dst+col), xmm0);
					}			
					for (idx = 0; idx < remainder; idx++)
					{
						val = src[col+idx] << shift;
						dst[col+idx] = val - (Short)IF_INTERNAL_OFFS;
					}
					src += srcStride;
					dst += dstStride;
				}
			}
			else
			{
				for (row = 0; row < height; row++)
				{
					for(col=0; col<awidth; col+=8)
					{
						xmm0 = _mm_loadu_si128((__m128i const *)(src+col));
						xmm0 = _mm_slli_epi16(xmm0, (short)shift);
						xmm0 = _mm_sub_epi16(xmm0, xmm1);
						_mm_storeu_si128((__m128i *)(dst+col), xmm0);
					}			
					src += srcStride;
					dst += dstStride;
				}
			}
		}

	}
	else
	{
		Int shift = IF_INTERNAL_PREC - bitDepth;
		Short offset = IF_INTERNAL_OFFS;
		offset += shift?(1 << (shift - 1)):0;
		Short maxVal = (1 << bitDepth) - 1;
		Short minVal = 0;
		xmm1 = _mm_set1_epi16(offset);
		if (width < 4)
		{
			///< 2015 8 5 by Seok : Confirmed Code 
			for (row = 0; row < height; row++)
			{
				for (col = 0; col < width; col++)
				{
					Short val = (src[ col ] + offset ) >> shift;
					val = (val < minVal)? minVal : ((val > maxVal)? maxVal : val);
					dst[col] = val;
				}
				src += srcStride;
				dst += dstStride;
			}			 
		}
		else if (width < 8)
		{
			remainder = width & 0x03;	// width : 4<= width < 8
			xmm_mx = _mm_set1_epi16(maxVal);	///< max Value : 2015 8 6 by Seok
			xmm_z    = _mm_setzero_si128();		///< min Value : 2015 8 6 by Seok

			if (remainder)
			{
				///< 2015 8 5 by Seok : Confirmed Code 
				for (row = 0; row < height; row++)
				{
					xmm0 = _mm_loadu_si128((__m128i const *)src);
					xmm0 = _mm_add_epi16(xmm0, xmm1);			//val + offset
					xmm0 = _mm_srai_epi16(xmm0, (short)shift);		//(val + offset)>>shift

					ETRI_CLIP_BITLIMIT(xmm0, xmm_mx,xmm_z);

					_mm_store_si128((__m128i *)est, xmm0);
					*(Int64 *)dst = *(Int64 *)est;
					for(idx=0; idx<remainder; idx++)
						dst[4+idx] = est[4+idx];
					src += srcStride;
					dst += dstStride;
				}			 
			}
			else
			{
				for (row = 0; row < height; row++)
				{
					xmm0 = _mm_loadu_si128((__m128i const *)src);
					xmm0 = _mm_add_epi16(xmm0, xmm1);			//val + offset
					xmm0 = _mm_srai_epi16(xmm0, (short)shift);		//(val + offset)>>shift
#if 1				
					xmm_cmx = _mm_cmpgt_epi16(xmm0, xmm_mx);	// cmax = (xmm0 > maxVal)? 0xffff: 0x0
					xmm_cmn = _mm_cmplt_epi16(xmm0, xmm_z);		// cmin = (xmm0 < minVal)? 0xffff: 0x0
					xmm2 = _mm_and_si128(xmm_cmx, xmm_mx);		// cmax & maxVal
					xmm0 = _mm_andnot_si128(xmm_cmx, xmm0);		// /cmax & xmm0
					xmm0 = _mm_or_si128(xmm0, xmm2);   			// (cmax & maxVal) || (/cmax & xmm0)
					xmm0 = _mm_andnot_si128(xmm_cmn, xmm0);		//  /cmin & xmm0		
#else
					xmm0 = _mm_packus_epi16(xmm0, _mm_setzero_si128());  //pcaked : 0-7 : xmm0, 8-15 : zero : 8bit
					xmm0 = _mm_unpacklo_epi8(xmm0,_mm_setzero_si128());	//retrieve data to 16bit 
#endif

					_mm_store_si128((__m128i *)dst, xmm0);
					src += srcStride;
					dst += dstStride;
				}			 
			}
		}
		else
		{
			///< 2015 8 5 by Seok : Here, bug exist
			remainder = width & 0x07;	
			awidth      = width - remainder;
			xmm_mx = _mm_set1_epi16(maxVal);	///< max Value : 2015 8 6 by Seok
			xmm_z    = _mm_setzero_si128();		///< min Value : 2015 8 6 by Seok

			if (remainder)
			{
				for (row = 0; row < height; row++)
				{
					for (col = 0; col < awidth; col+=8)
					{
						xmm0 = _mm_loadu_si128((__m128i const *)(src+col));
						xmm0 = _mm_add_epi16(xmm0, xmm1);					//val + offset
						xmm0 = _mm_srai_epi16(xmm0, (short)shift);				//(val + offset)>>shift
#if 1
						xmm_cmx = _mm_cmpgt_epi16(xmm0, xmm_mx);	// cmax = (xmm0 > maxVal)? 0xffff: 0x0
						xmm_cmn = _mm_cmplt_epi16(xmm0, xmm_z);   	// cmin = (xmm0 < minVal)? 0xffff: 0x0
						xmm2 = _mm_and_si128(xmm_cmx, xmm_mx);		// cmax & maxVal : MAX CLIP  
						xmm0 = _mm_andnot_si128(xmm_cmn, xmm0);  	// /cmin & xmm0 : MIN CLIP
						xmm0 = _mm_andnot_si128(xmm_cmx, xmm0);  	///cmx &  /cmin & xmm0 : 
						xmm0 = _mm_or_si128(xmm0, xmm2);   			// (cmax & maxVal) || (/cmin & xmm0) 
#else
						xmm0 = _mm_packus_epi16(xmm0, _mm_setzero_si128());  //pcaked : 0-7 : xmm0, 8-15 : zero : 8bit
						xmm0 = _mm_unpacklo_epi8(xmm0,_mm_setzero_si128()); //retrieve data to 16bit 
#endif
						_mm_storeu_si128((__m128i *)(dst+col), xmm0);
					}
					for (idx = 0; idx < remainder; idx++)
					{
						Short val = (src[ col + idx] + offset ) >> shift;
						val = (val < minVal)? minVal : ((val > maxVal)? maxVal : val);
						dst[col + idx] = val;
					}
					src += srcStride;
					dst += dstStride;
				}			 
			}
			else
			{
				for (row = 0; row < height; row++)
				{
					for (col = 0; col < awidth; col+=8)
					{
						xmm0 = _mm_loadu_si128((__m128i const *)(src+col));
						xmm0 = _mm_add_epi16(xmm0, xmm1);					//val + offset
						xmm0 = _mm_srai_epi16(xmm0, (short)shift);				//(val + offset)>>shift
#if 1
						xmm_cmx = _mm_cmpgt_epi16(xmm0, xmm_mx);	// cmax = (xmm0 > maxVal)? 0xffff: 0x0
						xmm_cmn = _mm_cmplt_epi16(xmm0, xmm_z);   	// cmin = (xmm0 < minVal)? 0xffff: 0x0
						xmm2 = _mm_and_si128(xmm_cmx, xmm_mx);		// cmax & maxVal
						xmm0 = _mm_andnot_si128(xmm_cmx, xmm0);   	// /cmax & xmm0
						xmm0 = _mm_or_si128(xmm0, xmm2);  			// (cmax & maxVal) || (/cmax & xmm0)
						xmm0 = _mm_andnot_si128(xmm_cmn, xmm0);		// /cmin & xmm0 	
#else
						xmm0 = _mm_packus_epi16(xmm0, _mm_setzero_si128());  //pcaked : 0-7 : xmm0, 8-15 : zero : 8bit
						xmm0 = _mm_unpacklo_epi8(xmm0,_mm_setzero_si128()); //retrieve data to 16bit 
#endif
						_mm_storeu_si128((__m128i *)(dst+col), xmm0);
					}
					src += srcStride;
					dst += dstStride;
				}			 
			}
		}
	}
#else
  if ( isFirst == isLast )
  {
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        dst[col] = src[col];
      }
      
      src += srcStride;
      dst += dstStride;
    }              
  }
  else if ( isFirst )
  {
    Int shift = IF_INTERNAL_PREC - bitDepth;
    
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Short val = src[col] << shift;
        dst[col] = val - (Short)IF_INTERNAL_OFFS;
      }
      
      src += srcStride;
      dst += dstStride;
    }          
  }
  else
  {
    Int shift = IF_INTERNAL_PREC - bitDepth;
    Short offset = IF_INTERNAL_OFFS;
    offset += shift?(1 << (shift - 1)):0;
    Short maxVal = (1 << bitDepth) - 1;
    Short minVal = 0;
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Short val = src[ col ];
        val = ( val + offset ) >> shift;
        if (val < minVal) val = minVal;
        if (val > maxVal) val = maxVal;
        dst[col] = val;
      }
      
      src += srcStride;
      dst += dstStride;
    }              
  }
#endif
#endif ///#if !ETRI_JWS_SIMD
}

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
template<Int N, Bool isVertical, Bool isFirst, Bool isLast>
Void TComInterpolationFilter::filter(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Short const *coeff)
{
#if !ETRI_SIMD_INTERPOLATION
  Int row, col;
  
  Short c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }
  
  Int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  Int offset;
  Short maxVal;
  Int headRoom = IF_INTERNAL_PREC - bitDepth;
  Int shift = IF_FILTER_PREC;
  if ( isLast )
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
    maxVal = (1 << bitDepth) - 1;
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
    maxVal = 0;
  }
  
  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      Int sum;
      
      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
      }
      if ( N == 8 )
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];        
      }
      
      Short val = ( sum + offset ) >> shift;
      if ( isLast )
      {
        val = ( val < 0 ) ? 0 : val;
        val = ( val > maxVal ) ? maxVal : val;        
      }
      dst[col] = val;
    }
    
    src += srcStride;
    dst += dstStride;
  }    
#endif ///< !ETRI_SIMD_INTERPOLATION
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void TComInterpolationFilter::filterHor(Int bitDepth, Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Bool isLast, Short const *coeff)
{
#if ETRI_SIMD_INTERPOLATION
	if ( isLast )
	{
		ETRI_filter<N, false, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else
	{
		ETRI_filter<N, false, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
#else
	if ( isLast )
	{
		filter<N, false, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else
	{
		filter<N, false, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
#endif
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDpeth   Sample bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void TComInterpolationFilter::filterVer(Int bitDepth, Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, Short const *coeff)
{
#if ETRI_SIMD_INTERPOLATION
	if ( isFirst && isLast )
	{
		ETRI_filter<N, true, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else if ( isFirst && !isLast )
	{
		ETRI_filter<N, true, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else if ( !isFirst && isLast )
	{
		ETRI_filter<N, true, false, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else
	{
		ETRI_filter<N, true, false, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}      
#else
	if ( isFirst && isLast )
	{
		filter<N, true, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else if ( isFirst && !isLast )
	{
		filter<N, true, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else if ( !isFirst && isLast )
	{
		filter<N, true, false, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}
	else
	{
		filter<N, true, false, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
	}      
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of luma samples (horizontal)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterHorLuma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast )
{
	assert(frac >= 0 && frac < 4);

	if ( frac == 0 )
	{
		ETRI_macroFilterCopy(g_bitDepthY, src, srcStride, dst, dstStride, width, height, true, isLast );
	}
	else
	{
		filterHor<NTAPS_LUMA>(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac]);
	}
}

/**
 * \brief Filter a block of luma samples (vertical)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterVerLuma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast )
{
	assert(frac >= 0 && frac < 4);

	if ( frac == 0 )
	{
		ETRI_macroFilterCopy(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isFirst, isLast );
	}
	else
	{
		filterVer<NTAPS_LUMA>(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac]);
	}
}

/**
 * \brief Filter a block of chroma samples (horizontal)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterHorChroma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast )
{
	assert(frac >= 0 && frac < 8);

	if ( frac == 0 )
	{
		ETRI_macroFilterCopy(g_bitDepthC, src, srcStride, dst, dstStride, width, height, true, isLast );
	}
	else
	{
		filterHor<NTAPS_CHROMA>(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac]);
	}
}

/**
 * \brief Filter a block of chroma samples (vertical)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterVerChroma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast )
{
	assert(frac >= 0 && frac < 8);

	if ( frac == 0 )
	{
		ETRI_macroFilterCopy(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isFirst, isLast );
	}
	else
	{
		filterVer<NTAPS_CHROMA>(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac]);
	}
}


#if ETRI_SIMD_INTERPOLATION
// ====================================================================================================================
// ETRI SIMD for TComInterpolation Filter 
// ====================================================================================================================
#if defined(_WIN64)
#define _mm_Estore_si128 	_mm_store_si128
#else
#define _mm_Estore_si128 	_mm_store_si128
#endif

#if ETRI_SIMD_FILTERCOPY
 Void TComInterpolationFilter::ETRI_filterCopy(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast)
{
	//  int remainder = ((width<4)? (width & 0x01):((width<8)? (width & 0x03) : (width & (0x07))));
	int Condition = (isFirst == isLast) ? 0 : ((isFirst) ? 1 : 2);

	//EDPRINTF(stderr, "bitDepth: %d \n", bitDepth);	/// bitDepth is 10 whether 8 bit or 10 bit @ 2015 8 5 by Seok
	em_pfFilterCopy[Condition](bitDepth, src, srcStride, dst, dstStride, height, width);
}

Void TComInterpolationFilter::ETRI_pFFilterCopy_00(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int height, int width)
{
	Int     row, col;
	__m128i   xmm0;

	if (width < 4)
	{
		for (row = 0; row < height; row++)
		{
			*(int *)dst = *(int *)src;    //just width = 2 case
			src += srcStride;
			dst += dstStride;
		}
	}
	else if (width < 8)
	{
		for (row = 0; row < height; row++)
		{
			*(Int64 *)dst = *(Int64 *)src;  //just 4 case 
			src += srcStride;
			dst += dstStride;
		}
	}
	else
	{
		///< Faster Code is possible @ 2015 8 5 by Seok
		for (row = 0; row < height; row++)
		{
			for (col = 0; col<width; col += 8)
			{
				xmm0 = _mm_loadu_si128((__m128i const *)(&src[col]));
				_mm_Estore_si128((__m128i *)(&dst[col]), xmm0);
			}
			src += srcStride;
			dst += dstStride;
		}
	}
}

Void TComInterpolationFilter::ETRI_pFFilterCopy_01(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int height, int width)
{
	Int     row, col;
	int     remainder, awidth, idx;

	Short shift = IF_INTERNAL_PREC - bitDepth;
	Short val;

	__m128i   xmm0, xmm1;
	ALIGNED(16) Short est[8];

	xmm1 = _mm_set1_epi16((Short)IF_INTERNAL_OFFS);

	if (width < 4){
		for (row = 0; row < height; row++)
		{
			for (col = 0; col < width; col++)
			{
				val = src[col] << shift;
				dst[col] = val - (Short)IF_INTERNAL_OFFS;
			}
			src += srcStride;
			dst += dstStride;
		}
	}
	else if (width < 8)
	{
		remainder = width & 0x03;  // width : 4<= width < 8
		if (remainder)
		{
			for (row = 0; row < height; row++)
			{
				xmm0 = _mm_loadu_si128((__m128i const *)src);
				xmm0 = _mm_slli_epi16(xmm0, (short)shift);
				xmm0 = _mm_sub_epi16(xmm0, xmm1);
				_mm_Estore_si128((__m128i *)est, xmm0);
				*(Int64 *)dst = *(Int64 *)est;

				for (idx = 0; idx<remainder; idx++)
					dst[4 + idx] = est[4 + idx];
				src += srcStride;
				dst += dstStride;
			}
		}
		else
		{
			for (row = 0; row < height; row++)
			{
				xmm0 = _mm_loadu_si128((__m128i const *)src);
				xmm0 = _mm_slli_epi16(xmm0, (short)shift);
				xmm0 = _mm_sub_epi16(xmm0, xmm1);
				_mm_Estore_si128((__m128i *)est, xmm0);
				*(Int64 *)dst = *(Int64 *)est;

				src += srcStride;
				dst += dstStride;
			}
		}
	}
	else
	{
		remainder = width & 0x07;
		awidth = width - remainder;

		if (remainder)
		{
			for (row = 0; row < height; row++)
			{
				for (col = 0; col<awidth; col += 8)
				{
					xmm0 = _mm_loadu_si128((__m128i const *)(src + col));
					xmm0 = _mm_slli_epi16(xmm0, (short)shift);
					xmm0 = _mm_sub_epi16(xmm0, xmm1);
					_mm_Estore_si128((__m128i *)(dst + col), xmm0);
				}

				for (idx = 0; idx < remainder; idx++)
				{
					val = src[col + idx] << shift;
					dst[col + idx] = val - (Short)IF_INTERNAL_OFFS;
				}
				src += srcStride;
				dst += dstStride;
			}
		}
		else
		{
			for (row = 0; row < height; row++)
			{
				for (col = 0; col<awidth; col += 8)
				{
					xmm0 = _mm_loadu_si128((__m128i const *)(src + col));
					xmm0 = _mm_slli_epi16(xmm0, (short)shift);
					xmm0 = _mm_sub_epi16(xmm0, xmm1);
					_mm_Estore_si128((__m128i *)(dst + col), xmm0);
				}
				src += srcStride;
				dst += dstStride;
			}
		}
	}
}

Void TComInterpolationFilter::ETRI_pFFilterCopy_02(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int height, int width)
{
	int     row, col, remainder, awidth, idx;

	Int     shift = IF_INTERNAL_PREC - bitDepth;
	Short   offset = IF_INTERNAL_OFFS;
	offset += shift ? (1 << (shift - 1)) : 0;
	Short   maxVal = (1 << bitDepth) - 1;
	Short   minVal = 0;

	__m128i   xmm0, xmm1;
	__m128i   xmm_mx, xmm_z;
	ALIGNED(16) Short est[8];

	xmm1 = _mm_set1_epi16(offset);
	xmm_mx = _mm_set1_epi16(maxVal);	///< max Value : 2015 8 6 by Seok
	xmm_z	 = _mm_setzero_si128(); 	///< min Value : 2015 8 6 by Seok

	if (width < 4)
	{
		for (row = 0; row < height; row++)
		{
			for (col = 0; col < width; col++)
			{
				Short val = (src[col] + offset) >> shift;
				val = (val < minVal) ? minVal : ((val > maxVal) ? maxVal : val);
				dst[col] = val;
			}
			src += srcStride;
			dst += dstStride;
		}
	}
	else if (width < 8)
	{
		remainder = width & 0x03;  // width : 4<= width < 8
		if (remainder)
		{
			for (row = 0; row < height; row++)
			{
				xmm0 = _mm_loadu_si128((__m128i const *)src);
				xmm0 = _mm_add_epi16(xmm0, xmm1);      //val + offset
				xmm0 = _mm_srai_epi16(xmm0, (short)shift);    //(val + offset)>>shift

				ETRI_CLIP_BITLIMIT(xmm0, xmm_mx, xmm_z);

				_mm_Estore_si128((__m128i *)est, xmm0);
				*(Int64 *)dst = *(Int64 *)est;

				for (idx = 0; idx<remainder; idx++)
					dst[4 + idx] = est[4 + idx];

				src += srcStride;
				dst += dstStride;
			}
		}
		else
		{
			for (row = 0; row < height; row++)
			{
				xmm0 = _mm_loadu_si128((__m128i const *)src);
				xmm0 = _mm_add_epi16(xmm0, xmm1);      //val + offset
				xmm0 = _mm_srai_epi16(xmm0, (short)shift);    //(val + offset)>>shift

				ETRI_CLIP_BITLIMIT(xmm0, xmm_mx,xmm_z);

				_mm_Estore_si128((__m128i *)dst, xmm0);

				src += srcStride;
				dst += dstStride;
			}
		}
	}
	else
	{
		remainder = width & 0x07;
		awidth = width - remainder;

		if (remainder)
		{
			for (row = 0; row < height; row++)
			{
				for (col = 0; col < awidth; col += 8)
				{
					xmm0 = _mm_loadu_si128((__m128i const *)(src + col));
					xmm0 = _mm_add_epi16(xmm0, xmm1);          //val + offset
					xmm0 = _mm_srai_epi16(xmm0, (short)shift);        //(val + offset)>>shift

					ETRI_CLIP_BITLIMIT(xmm0, xmm_mx,xmm_z);

					_mm_Estore_si128((__m128i *)(dst + col), xmm0);
				}

				for (idx = 0; idx < remainder; idx++)
				{
					Short val = (src[col + idx] + offset) >> shift;
					val = (val < minVal) ? minVal : ((val > maxVal) ? maxVal : val);
					dst[col + idx] = val;
				}
				src += srcStride;
				dst += dstStride;
			}
		}
		else
		{
			for (row = 0; row < height; row++)
			{
				for (col = 0; col < awidth; col += 8)
				{
					xmm0 = _mm_loadu_si128((__m128i const *)(src + col));
					xmm0 = _mm_add_epi16(xmm0, xmm1);          //val + offset
					xmm0 = _mm_srai_epi16(xmm0, (short)shift);        //(val + offset)>>shift

					ETRI_CLIP_BITLIMIT(xmm0, xmm_mx,xmm_z );
					
					_mm_Estore_si128((__m128i *)(dst + col), xmm0);
				}
				src += srcStride;
				dst += dstStride;
			}
		}
	}
}
#else
///< 2015 8 5 by Seok : To avoid Compile Error
Void TComInterpolationFilter::ETRI_pFFilterCopy_00(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int height, int width){}
Void TComInterpolationFilter::ETRI_pFFilterCopy_01(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int height, int width){}
Void TComInterpolationFilter::ETRI_pFFilterCopy_02(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int height, int width){}
#endif

// ====================================================================================================================
// ETRI FIlter Macro
// ====================================================================================================================

#define  ETRI_SIMD_VERTICAL_LOAD(xmm, esrc, col, cStride) \
  xmm[0] = _mm_loadu_si128((__m128i const *)(esrc+= col)); \
  xmm[1] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[2] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[3] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[4] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[5] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[6] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[7] = _mm_loadu_si128((__m128i const *)(esrc+= cStride));

#define  ETRI_SIMD_VERTICAL_4TLOAD(xmm, esrc, col, cStride) \
  xmm[0] = _mm_loadu_si128((__m128i const *)(esrc+= col)); \
  xmm[1] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[2] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); \
  xmm[3] = _mm_loadu_si128((__m128i const *)(esrc+= cStride)); 

#define ETRI_SIMD_SERIAL_LOAD(xmm, esrc, col, cmm) \
  xmm[0] = _mm_loadu_si128((__m128i const *)(&esrc[col])); \
  xmm[7] = _mm_loadu_si128((__m128i const *)(&esrc[col+7])); \
  xmm[1] = _mm_srli_si128(xmm[0], 2); \
  xmm[2] = _mm_slli_si128(xmm[7], 12); \
  xmm[1] = _mm_or_si128(xmm[1], xmm[2]); \
  xmm[2] = _mm_srli_si128(xmm[0], 4); \
  xmm[3] = _mm_slli_si128(xmm[7], 10); \
  xmm[2] = _mm_or_si128(xmm[2], xmm[3]); \
  xmm[3] = _mm_srli_si128(xmm[0], 6); \
  xmm[4] = _mm_slli_si128(xmm[7], 8); \
  xmm[3] = _mm_or_si128(xmm[3], xmm[4]); \
  xmm[4] = _mm_srli_si128(xmm[0], 8); \
  xmm[5] = _mm_slli_si128(xmm[7], 6); \
  xmm[4] = _mm_or_si128(xmm[4], xmm[5]); \
  xmm[5] = _mm_srli_si128(xmm[0], 10); \
  xmm[6] = _mm_slli_si128(xmm[7], 4); \
  xmm[5] = _mm_or_si128(xmm[5], xmm[6]); \
  xmm[6] = _mm_srli_si128(xmm[0], 12); \
  cmm     = _mm_slli_si128(xmm[7], 2); \
  xmm[6] = _mm_or_si128(xmm[6], cmm);

#define ETRI_SIMD_8x8_TRANSPOSE(Lmm, tmm, xmm) \
  tmm[0] = _mm_unpacklo_epi16(Lmm[0], Lmm[1]); \
  tmm[1] = _mm_unpackhi_epi16(Lmm[0], Lmm[1]); \
  tmm[2] = _mm_unpacklo_epi16(Lmm[2], Lmm[3]); \
  tmm[3] = _mm_unpackhi_epi16(Lmm[2], Lmm[3]); \
  tmm[4] = _mm_unpacklo_epi16(Lmm[4], Lmm[5]); \
  tmm[5] = _mm_unpackhi_epi16(Lmm[4], Lmm[5]); \
  tmm[6] = _mm_unpacklo_epi16(Lmm[6], Lmm[7]); \
  tmm[7] = _mm_unpackhi_epi16(Lmm[6], Lmm[7]); \
  xmm[0] = _mm_unpacklo_epi32(tmm[0], tmm[2]); \
  xmm[1] = _mm_unpackhi_epi32(tmm[0], tmm[2]); \
  xmm[2] = _mm_unpacklo_epi32(tmm[1], tmm[3]); \
  xmm[3] = _mm_unpackhi_epi32(tmm[1], tmm[3]); \
  xmm[4] = _mm_unpacklo_epi32(tmm[4], tmm[6]); \
  xmm[5] = _mm_unpackhi_epi32(tmm[4], tmm[6]); \
  xmm[6] = _mm_unpacklo_epi32(tmm[5], tmm[7]); \
  xmm[7] = _mm_unpackhi_epi32(tmm[5], tmm[7]); \
  tmm[0] = _mm_unpacklo_epi64(xmm[0], xmm[4]); \
  tmm[1] = _mm_unpackhi_epi64(xmm[0], xmm[4]); \
  tmm[2] = _mm_unpacklo_epi64(xmm[1], xmm[5]); \
  tmm[3] = _mm_unpackhi_epi64(xmm[1], xmm[5]); \
  tmm[4] = _mm_unpacklo_epi64(xmm[2], xmm[6]); \
  tmm[5] = _mm_unpackhi_epi64(xmm[2], xmm[6]); \
  tmm[6] = _mm_unpacklo_epi64(xmm[3], xmm[7]); \
  tmm[7] = _mm_unpackhi_epi64(xmm[3], xmm[7]);


#define ETRI_SIMD_4x8_TRANSPOSE(xmm, tmm) \
  tmm[0] = _mm_unpacklo_epi16(xmm[0], xmm[1]); \
  tmm[1] = _mm_unpackhi_epi16(xmm[0], xmm[1]); \
  tmm[2] = _mm_unpacklo_epi16(xmm[2], xmm[3]); \
  tmm[3] = _mm_unpackhi_epi16(xmm[2], xmm[3]); \
  xmm[0] = _mm_unpacklo_epi32(tmm[0], tmm[2]); \
  xmm[1] = _mm_unpackhi_epi32(tmm[0], tmm[2]); \
  xmm[2] = _mm_unpacklo_epi32(tmm[1], tmm[3]); \
  xmm[3] = _mm_unpackhi_epi32(tmm[1], tmm[3]); 


#define ETRI_SIMD_8TAB_FILTER(xmm, tmm, cmm, shift) \
  xmm[0] = _mm_madd_epi16(tmm[0], cmm[0]); \
  xmm[1] = _mm_madd_epi16(tmm[1], cmm[0]); \
  xmm[2] = _mm_madd_epi16(tmm[2], cmm[0]); \
  xmm[3] = _mm_madd_epi16(tmm[3], cmm[0]); \
  xmm[4] = _mm_madd_epi16(tmm[4], cmm[0]); \
  xmm[5] = _mm_madd_epi16(tmm[5], cmm[0]); \
  xmm[6] = _mm_madd_epi16(tmm[6], cmm[0]); \
  xmm[7] = _mm_madd_epi16(tmm[7], cmm[0]); \
  xmm[0] =  _mm_hadd_epi32(xmm[0], xmm[1]); \
  xmm[2] =  _mm_hadd_epi32(xmm[2], xmm[3]); \
  xmm[4] =  _mm_hadd_epi32(xmm[4], xmm[5]); \
  xmm[6] =  _mm_hadd_epi32(xmm[6], xmm[7]); \
  xmm[0] =  _mm_hadd_epi32(xmm[0], xmm[2]); \
  xmm[4] =  _mm_hadd_epi32(xmm[4], xmm[6]); \
  xmm[0] = _mm_add_epi32(xmm[0], cmm[1]); \
  xmm[4] = _mm_add_epi32(xmm[4], cmm[1]); \
  xmm[0] = _mm_srai_epi32(xmm[0], shift); \
  xmm[4] = _mm_srai_epi32(xmm[4], shift); \
  xmm[0] = _mm_packs_epi32(xmm[0], xmm[4]); \


#define ETRI_SIMD_4TAB_FILTER(xmm, tmm, cmm, shift) \
  xmm[0] = _mm_madd_epi16(tmm[0], cmm[0]); \
  xmm[1] = _mm_madd_epi16(tmm[1], cmm[0]); \
  xmm[2] = _mm_madd_epi16(tmm[2], cmm[0]); \
  xmm[3] = _mm_madd_epi16(tmm[3], cmm[0]); \
  xmm[0] = _mm_hadd_epi32(xmm[0], xmm[1]); \
  xmm[2] = _mm_hadd_epi32(xmm[2], xmm[3]); \
  xmm[0] = _mm_add_epi32(xmm[0], cmm[1]); \
  xmm[2] = _mm_add_epi32(xmm[2], cmm[1]); \
  xmm[0] = _mm_srai_epi32(xmm[0], shift); \
  xmm[2] = _mm_srai_epi32(xmm[2], shift); \
  xmm[0] = _mm_packs_epi32(xmm[0], xmm[2]); 


/**
====================================================================================================================
	@brief	ETRI "static" Filter Function. Because the function is static, it requires TComInterpolationFilter object to indicate another static function or function pointer.
	@param	 N 		Filter Tab number
	@param 	isVertical	indicate Horizontal (False) or Vertical (True) 
	@param 	isFirst 	indicate First Process (generally, horizontal function is first) or last process	
	@param 	isLast	indicate Last Process (generally, vertical function is last) or last process. However, under some special case, a first process is a last process.	
	@author	Jinwuk Seok 2015 0808
====================================================================================================================
*/
template<int N, bool isVertical, bool isFirst, bool isLast>
Void TComInterpolationFilter::ETRI_filter(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Short const *coeff)
{
	Int     offset;
	Int     headRoom = IF_INTERNAL_PREC - bitDepth;
	Int     shift = IF_FILTER_PREC;
	ALIGNED(16) Short c[8];

	if (isLast)
	{
		shift += (isFirst) ? 0 : headRoom;
		offset = 1 << (shift - 1);
		offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
	}
	else
	{
		shift -= (isFirst) ? headRoom : 0;
		offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
	}

	if (isVertical)
	{
		if (N == 4)
		{
			src -= srcStride;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)coeff;

			if ((width >= 8) && ((width & 0x07) == 0))
			{
				if (isLast)
				ETRI_FilterV_4T8WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[1]
				else
				ETRI_FilterV_4T8W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);		/// em_pfFilterFunc[0]
			}
			else
			{
				if (isLast)
				ETRI_FilterV_4T4WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[3]
				else
				ETRI_FilterV_4T4W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);		/// em_pfFilterFunc[2]
			}
		}
		else //if ( N == 8 )
		{
			src -= 3 * srcStride;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)&coeff[4];

			if (width >= 8)
			{
				if ((width & 0x07) > 0)
				{
					if (isLast)
					ETRI_FilterV_8T8WRLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[5]
					else
					ETRI_FilterV_8T8WR(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[4]
				}
				else
				{
					if (isLast)
					ETRI_FilterV_8T8WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[7]
					else
					ETRI_FilterV_8T8W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[6]	
				}
			}
			else
			{
				if (width >= 4)
				{
					if (isLast)
					ETRI_FilterV_8T4WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[9]
					else
					ETRI_FilterV_8T4W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[8]
				}
				else
				{
					if (isLast)
					ETRI_FilterV_8T2WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[11]
					else
					ETRI_FilterV_8T2W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[10]
				}
			}
		}
	}
	else    //Horizontal 
	{
		if (N == 4)
		{
			src -= 1;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)coeff;

			if ((width >= 8) && ((width & 0x07) == 0))
			{
				if (isLast)
				ETRI_FilterH_4T8WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[13]
				else
				ETRI_FilterH_4T8W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[12]
			}
			else
			{
				if (isLast)
				ETRI_FilterH_4T4WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[15]
				else
				ETRI_FilterH_4T4W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[14]
			}
		}
		else //if ( N == 8 )
		{
			src -= 3;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)&coeff[4];

			if (width >= 8)
			{
				if (width & 0x07)
				{
					if (isLast)
					ETRI_FilterH_8T8WRLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[17]
					else
					ETRI_FilterH_8T8WR(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[16]	
				}
				else
				{
					if (isLast)
					ETRI_FilterH_8T8WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[19]	
					else
					ETRI_FilterH_8T8W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[18]		
				}
			}
			else
			{  // width < 8
				if (width >= 4)
				{
					if (isLast)
					ETRI_FilterH_8T4WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[21]	
					else
					ETRI_FilterH_8T4W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[20]	
				}
				else
				{
					if (isLast)
					ETRI_FilterH_8T2WLast(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[23]	
					else
					ETRI_FilterH_8T2W(bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);	/// em_pfFilterFunc[22]	
				}
			} //if (width >=8)
		}// if N==8?
	}//End of Horizontal
}

/**
====================================================================================================================
	@brief	ETRI "static" Filter Functions with SIMD. 
	@author	Jinwuk Seok 2015 0808
====================================================================================================================
*/
Void TComInterpolationFilter::ETRI_FilterV_8T8WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], Lmm[8], cmm[8];

	esrc = Osrc = (Short *)src;
	Short maxVal = (1 << bitDepth) - 1; 	///< 2015 8 7 by Seok

	cmm[0] = _mm_set1_epi32(*(Int32 *)c);
	cmm[1] = _mm_set1_epi32(*(Int32 *)(c + 2));
	cmm[2] = _mm_set1_epi32(*(Int32 *)(c + 4));
	cmm[3] = _mm_set1_epi32(*(Int32 *)(c + 6));
	cmm[4] = _mm_set1_epi32((Int32)offset);
	cmm[5] = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok
	cmm[6] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok


	for (row = col = 0; col < width; col += 8)
	{
		Lmm[0] = _mm_loadu_si128((__m128i const *)(esrc += col));
		Lmm[1] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[2] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[3] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[4] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[5] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[6] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[7] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

		xmm[0] = _mm_unpacklo_epi16(Lmm[0], Lmm[1]);
		xmm[1] = _mm_unpackhi_epi16(Lmm[0], Lmm[1]);
		xmm[2] = _mm_unpacklo_epi16(Lmm[2], Lmm[3]);
		xmm[3] = _mm_unpackhi_epi16(Lmm[2], Lmm[3]);
		xmm[4] = _mm_unpacklo_epi16(Lmm[4], Lmm[5]);
		xmm[5] = _mm_unpackhi_epi16(Lmm[4], Lmm[5]);
		xmm[6] = _mm_unpacklo_epi16(Lmm[6], Lmm[7]);
		xmm[7] = _mm_unpackhi_epi16(Lmm[6], Lmm[7]);

		xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
		xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
		xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
		xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
		xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
		xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
		xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
		xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

		xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
		xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
		xmm[0] = _mm_srai_epi32(xmm[0], shift);
		xmm[1] = _mm_srai_epi32(xmm[1], shift);
		xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

		ETRI_CLIP_BITLIMIT(xmm[0], cmm[5], cmm[6]);

		_mm_store_si128((__m128i *)&dst[col], xmm[0]);

		for (row = 1; row < height; row++)
		{
			Lmm[(row - 1) & 0x07] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

			xmm[0] = _mm_unpacklo_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
			xmm[1] = _mm_unpackhi_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
			xmm[2] = _mm_unpacklo_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
			xmm[3] = _mm_unpackhi_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
			xmm[4] = _mm_unpacklo_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
			xmm[5] = _mm_unpackhi_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
			xmm[6] = _mm_unpacklo_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \
			xmm[7] = _mm_unpackhi_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \

			xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
			xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
			xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
			xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
			xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
			xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
			xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
			xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

			xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
			xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
			xmm[0] = _mm_srai_epi32(xmm[0], shift);
			xmm[1] = _mm_srai_epi32(xmm[1], shift);
			xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

			ETRI_CLIP_BITLIMIT(xmm[0], cmm[5], cmm[6]);

			_mm_store_si128((__m128i *)&dst[col + row*dstStride], xmm[0]);
		}
		esrc = Osrc;
	}
}

Void TComInterpolationFilter::ETRI_FilterH_8T8W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int  row, col;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], cmm[3];

	esrc = Osrc = (Short *)src;
	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col += 8)
		{
			ETRI_SIMD_SERIAL_LOAD(xmm, esrc, col, cmm[2]);
			ETRI_SIMD_8TAB_FILTER(xmm, xmm, cmm, shift);
			_mm_store_si128((__m128i *)&dst[col], xmm[0]);
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}//for (row = 0; row < height; row++)
}

Void TComInterpolationFilter::ETRI_FilterV_8T8WRLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    k, row, col, Qwidth, Remainder = width & 0x07;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], Lmm[8], cmm[8];

	esrc = Osrc = (Short *)src;
	Qwidth = width - Remainder;

	Short   maxVal = (1 << bitDepth) - 1;    //2013 03 04 by seok

	cmm[0] = _mm_set1_epi32(*(Int32 *)c);
	cmm[1] = _mm_set1_epi32(*(Int32 *)(c + 2));
	cmm[2] = _mm_set1_epi32(*(Int32 *)(c + 4));
	cmm[3] = _mm_set1_epi32(*(Int32 *)(c + 6));
	cmm[4] = _mm_set1_epi32((Int32)offset);
	cmm[5] = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok
	cmm[6] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok

	for (row = col = 0; col < width; col += 8)
	{
		Lmm[0] = _mm_loadu_si128((__m128i const *)(esrc += col));
		Lmm[1] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[2] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[3] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[4] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[5] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[6] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[7] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

		xmm[0] = _mm_unpacklo_epi16(Lmm[0], Lmm[1]);
		xmm[1] = _mm_unpackhi_epi16(Lmm[0], Lmm[1]);
		xmm[2] = _mm_unpacklo_epi16(Lmm[2], Lmm[3]);
		xmm[3] = _mm_unpackhi_epi16(Lmm[2], Lmm[3]);
		xmm[4] = _mm_unpacklo_epi16(Lmm[4], Lmm[5]);
		xmm[5] = _mm_unpackhi_epi16(Lmm[4], Lmm[5]);
		xmm[6] = _mm_unpacklo_epi16(Lmm[6], Lmm[7]);
		xmm[7] = _mm_unpackhi_epi16(Lmm[6], Lmm[7]);

		xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
		xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
		xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
		xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
		xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
		xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
		xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
		xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

		xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
		xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
		xmm[0] = _mm_srai_epi32(xmm[0], shift);
		xmm[1] = _mm_srai_epi32(xmm[1], shift);
		xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

		ETRI_CLIP_BITLIMIT(xmm[0], cmm[5], cmm[6]);

		_mm_store_si128((__m128i *)&dst[col], xmm[0]);

		for (row = 1; row < height; row++)
		{
			Lmm[(row - 1) & 0x07] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

			xmm[0] = _mm_unpacklo_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
			xmm[1] = _mm_unpackhi_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
			xmm[2] = _mm_unpacklo_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
			xmm[3] = _mm_unpackhi_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
			xmm[4] = _mm_unpacklo_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
			xmm[5] = _mm_unpackhi_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
			xmm[6] = _mm_unpacklo_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \
			xmm[7] = _mm_unpackhi_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \

			xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
			xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
			xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
			xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
			xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
			xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
			xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
			xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

			xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
			xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
			xmm[0] = _mm_srai_epi32(xmm[0], shift);
			xmm[1] = _mm_srai_epi32(xmm[1], shift);
			xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

			ETRI_CLIP_BITLIMIT(xmm[0], cmm[5], cmm[6]);

			_mm_store_si128((__m128i *)&dst[col + row*dstStride], xmm[0]);
		}
		esrc = Osrc;
	}


	for (row = 0; row < height; row++)
	{
		for (k = 0; k < Remainder; k++)
		{
			Int sum;
			sum = *(esrc += (Qwidth + k)) * c[0];
			sum += *(esrc += srcStride) * c[1];
			sum += *(esrc += srcStride) * c[2];
			sum += *(esrc += srcStride) * c[3];
			sum += *(esrc += srcStride) * c[4];
			sum += *(esrc += srcStride) * c[5];
			sum += *(esrc += srcStride) * c[6];
			sum += *(esrc += srcStride) * c[7];

			Short val = (sum + offset) >> shift;
			val = (val < 0) ? 0 : val;
			val = (val > maxVal) ? maxVal : val;
			dst[Qwidth + k] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}

}

Void TComInterpolationFilter::ETRI_FilterH_8T8WR(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int row, col;
	int k, Qwidth, Remainder = width & 0x07;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], cmm[3];
	esrc = Osrc = (Short *)src;
	Qwidth = width - Remainder;

	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	for (row = 0; row < height; row++)
	{
		for (col = 0; col < Qwidth; col += 8)
		{
			ETRI_SIMD_SERIAL_LOAD(xmm, esrc, col, cmm[2]);
			ETRI_SIMD_8TAB_FILTER(xmm, xmm, cmm, shift);
			_mm_store_si128((__m128i *)&dst[col], xmm[0]);
			esrc = Osrc;
		}

		for (k = 0; k < Remainder; k++)
		{
			Int sum;
			sum = *(esrc += Qwidth + k) * c[0];
			sum += *(esrc += 1) * c[1];
			sum += *(esrc += 1) * c[2];
			sum += *(esrc += 1) * c[3];
			sum += *(esrc += 1) * c[4];
			sum += *(esrc += 1) * c[5];
			sum += *(esrc += 1) * c[6];
			sum += *(esrc += 1) * c[7];

			Short val = (sum + offset) >> shift;
			dst[Qwidth + k] = val;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}//for (row = 0; row < height; row++)


}



Void TComInterpolationFilter::ETRI_FilterV_8T4WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int   row, k;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], Lmm[8], cmm[8];
	ALIGNED(16) Int16 est[8];

	esrc = Osrc = (Short *)src;
	Short   maxVal = (1 << bitDepth) - 1;    //2013 03 04 by seok

	k = width & 0x03;

	cmm[0] = _mm_set1_epi32(*(Int32 *)c);
	cmm[1] = _mm_set1_epi32(*(Int32 *)(c + 2));
	cmm[2] = _mm_set1_epi32(*(Int32 *)(c + 4));
	cmm[3] = _mm_set1_epi32(*(Int32 *)(c + 6));
	cmm[4] = _mm_set1_epi32((Int32)offset);
	cmm[5] = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok
	cmm[6] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok


	Lmm[0] = _mm_loadu_si128((__m128i const *)(esrc));
	Lmm[1] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
	Lmm[2] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
	Lmm[3] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
	Lmm[4] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
	Lmm[5] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
	Lmm[6] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
	Lmm[7] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

	xmm[0] = _mm_unpacklo_epi16(Lmm[0], Lmm[1]);
	xmm[1] = _mm_unpackhi_epi16(Lmm[0], Lmm[1]);
	xmm[2] = _mm_unpacklo_epi16(Lmm[2], Lmm[3]);
	xmm[3] = _mm_unpackhi_epi16(Lmm[2], Lmm[3]);
	xmm[4] = _mm_unpacklo_epi16(Lmm[4], Lmm[5]);
	xmm[5] = _mm_unpackhi_epi16(Lmm[4], Lmm[5]);
	xmm[6] = _mm_unpacklo_epi16(Lmm[6], Lmm[7]);
	xmm[7] = _mm_unpackhi_epi16(Lmm[6], Lmm[7]);

	xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
	xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
	xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
	xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
	xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
	xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
	xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
	xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
	xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
	xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
	xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
	xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
	xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
	xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

	xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
	xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
	xmm[0] = _mm_srai_epi32(xmm[0], shift);
	xmm[1] = _mm_srai_epi32(xmm[1], shift);
	xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

	ETRI_CLIP_BITLIMIT(xmm[0], cmm[5], cmm[6]);

	_mm_store_si128((__m128i *)est, xmm[0]);

	*(Int64 *)dst = *(Int64 *)est;
	for (k = 0; k<(width & 0x03); k++)
		dst[4 + k] = est[4 + k];

	dst += dstStride;
	for (row = 1; row < height; row++)
	{
		Lmm[(row - 1) & 0x07] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

		xmm[0] = _mm_unpacklo_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
		xmm[1] = _mm_unpackhi_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
		xmm[2] = _mm_unpacklo_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
		xmm[3] = _mm_unpackhi_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
		xmm[4] = _mm_unpacklo_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
		xmm[5] = _mm_unpackhi_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
		xmm[6] = _mm_unpacklo_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \
		xmm[7] = _mm_unpackhi_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \

		xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
		xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
		xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
		xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
		xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
		xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
		xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
		xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

		xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
		xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
		xmm[0] = _mm_srai_epi32(xmm[0], shift);
		xmm[1] = _mm_srai_epi32(xmm[1], shift);
		xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

		ETRI_CLIP_BITLIMIT(xmm[0], cmm[5], cmm[6]);

		_mm_store_si128((__m128i *)est, xmm[0]);

		*(Int64 *)dst = *(Int64 *)est;
		for (k = 0; k<(width & 0x03); k++)
			dst[4 + k] = est[4 + k];

		dst += dstStride;
	}
}


Void TComInterpolationFilter::ETRI_FilterH_8T4W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int  row, col;
	Int16 *esrc, *Osrc, k;
	__m128i xmm[8], cmm[3];
	ALIGNED(16) Int16 est[8];


	esrc = Osrc = (Short *)src;
	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	for (col = 0, row = 0; row < height; row++)
	{
		ETRI_SIMD_SERIAL_LOAD(xmm, esrc, col, cmm[2]);
		ETRI_SIMD_8TAB_FILTER(xmm, xmm, cmm, shift);
		_mm_store_si128((__m128i *)est, xmm[0]);

		*(Int64 *)dst = *(Int64 *)est;

		for (k = 0; k<(width & 0x03); k++)
			dst[4 + k] = est[4 + k];

		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}


Void TComInterpolationFilter::ETRI_FilterV_4T8WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int     row, col;
	Int16 *esrc, *Osrc;
	__m128i xmm[4], tmm[4], cmm[4];

	esrc = Osrc = (Short *)src;
	Short maxVal = (1 << bitDepth) - 1; 	///< 2015 8 7 by Seok
	
	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	cmm[2] = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok
	cmm[3] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col += 8)
		{
			ETRI_SIMD_VERTICAL_4TLOAD(xmm, esrc, col, srcStride);
			ETRI_SIMD_4x8_TRANSPOSE(xmm, tmm);
			ETRI_SIMD_4TAB_FILTER(xmm, xmm, cmm, shift);

			ETRI_CLIP_BITLIMIT(xmm[0], cmm[2], cmm[3]);

			_mm_store_si128((__m128i *)&dst[col], xmm[0]);

			esrc = Osrc;
		}
		Osrc += srcStride;
		dst += dstStride;
		esrc = Osrc;
	}
}

Void TComInterpolationFilter::ETRI_FilterH_4T8W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col;

	Int16 *esrc, *Osrc;
	__m128i xmm[4], cmm[3];

	esrc = Osrc = (Short *)src;
	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col += 8)
		{
			xmm[0] = _mm_loadu_si128((__m128i const *)(&esrc[col])); \
			xmm[3] = _mm_loadu_si128((__m128i const *)(&esrc[col + 3])); \
			xmm[1] = _mm_srli_si128(xmm[0], 2); \
			xmm[2] = _mm_slli_si128(xmm[3], 4); \
			xmm[1] = _mm_or_si128(xmm[1], xmm[2]); \
			xmm[2] = _mm_srli_si128(xmm[0], 4); \
			cmm[2] = _mm_slli_si128(xmm[3], 2); \
			xmm[2] = _mm_or_si128(xmm[2], cmm[2]); \

			xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]); //(7 6) (5 4) (3 2) (1 0)
			xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]); //(8 7) (6 5) (4 3) (2 1)
			xmm[2] = _mm_madd_epi16(xmm[2], cmm[0]); //(9 8) (7 6) (5 4) (3 2)
			xmm[3] = _mm_madd_epi16(xmm[3], cmm[0]);  //(10 9) (8 7) (6 5) (4 3)
			xmm[0] = _mm_hadd_epi32(xmm[0], xmm[1]);  //(8 7 6 5) (4 3 2 1) (7 6 5 4) (3 2 1 0)
			xmm[2] = _mm_hadd_epi32(xmm[2], xmm[3]);  //(10 9 8 7) (6 5 4 3) (9 8 7 6) (5 4 3 2)

			xmm[1] = _mm_unpacklo_epi32(xmm[0], xmm[2]);
			xmm[3] = _mm_unpackhi_epi32(xmm[0], xmm[2]);
			xmm[0] = _mm_unpacklo_epi32(xmm[1], xmm[3]);
			xmm[2] = _mm_unpackhi_epi32(xmm[1], xmm[3]);

			xmm[0] = _mm_add_epi32(xmm[0], cmm[1]);
			xmm[2] = _mm_add_epi32(xmm[2], cmm[1]);
			xmm[0] = _mm_srai_epi32(xmm[0], shift);
			xmm[2] = _mm_srai_epi32(xmm[2], shift);
			xmm[0] = _mm_packs_epi32(xmm[0], xmm[2]);

			_mm_store_si128((__m128i *)&dst[col], xmm[0]);

			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}


}



Void TComInterpolationFilter::ETRI_FilterV_4T4WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int     row, col;
	Int16 *esrc, *Osrc;

	esrc = Osrc = (Short *)src;
	Short   maxVal = (1 << bitDepth) - 1;    //2013 03 04 by seok

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += srcStride) * c[1];
			sum += *(esrc += srcStride) * c[2];
			sum += *(esrc += srcStride) * c[3];

			Short val = (sum + offset) >> shift;
			val = (val < 0) ? 0 : val;
			val = (val >  maxVal) ? maxVal : val;

			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}

Void TComInterpolationFilter::ETRI_FilterH_4T4W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int   row, col;

	Int16 *esrc, *Osrc;
	esrc = Osrc = (Short *)src;
	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += 1) * c[1];
			sum += *(esrc += 1) * c[2];
			sum += *(esrc += 1) * c[3];

			Short val = (sum + offset) >> shift;
			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}

}

Void TComInterpolationFilter::ETRI_FilterV_8T2WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col;
	Int16 *esrc, *Osrc;
	esrc = Osrc = (Short *)src;

	Short  maxVal = (1 << bitDepth) - 1;    //2013 03 04 by seok

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += srcStride) * c[1];
			sum += *(esrc += srcStride) * c[2];
			sum += *(esrc += srcStride) * c[3];
			sum += *(esrc += srcStride) * c[4];
			sum += *(esrc += srcStride) * c[5];
			sum += *(esrc += srcStride) * c[6];
			sum += *(esrc += srcStride) * c[7];

			Short val = (sum + offset) >> shift;
			val = (val < 0) ? 0 : val;
			val = (val > maxVal) ? maxVal : val;

			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}

Void TComInterpolationFilter::ETRI_FilterH_8T2W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col;
	Int16 *esrc, *Osrc;
	esrc = Osrc = (Short *)src;

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += 1) * c[1];
			sum += *(esrc += 1) * c[2];
			sum += *(esrc += 1) * c[3];
			sum += *(esrc += 1) * c[4];
			sum += *(esrc += 1) * c[5];
			sum += *(esrc += 1) * c[6];
			sum += *(esrc += 1) * c[7];

			Short val = (sum + offset) >> shift;

			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}

//------------------------- Almost usless code

Void TComInterpolationFilter::ETRI_FilterV_4T4W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int     row, col;
	Int16 *esrc, *Osrc;
	esrc = Osrc = (Short *)src;

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += srcStride) * c[1];
			sum += *(esrc += srcStride) * c[2];
			sum += *(esrc += srcStride) * c[3];

			Short val = (sum + offset) >> shift;
			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}

Void TComInterpolationFilter::ETRI_FilterV_8T8WR(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col, k, Qwidth, Remainder = width & 0x07;

	Int16 *esrc, *Osrc;
	__m128i xmm[8], tmm[8], cmm[3];
	esrc = Osrc = (Short *)src;
	Qwidth = width - Remainder;

	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	for (row = 0; row < height; row++)
	{
		for (col = 0; col < Qwidth; col += 8)
		{
			ETRI_SIMD_VERTICAL_LOAD(xmm, esrc, col, srcStride);
			ETRI_SIMD_8x8_TRANSPOSE(xmm, tmm, xmm);
			ETRI_SIMD_8TAB_FILTER(xmm, tmm, cmm, shift);
			_mm_store_si128((__m128i *)&dst[col], xmm[0]);
			esrc = Osrc;
		}

		for (k = 0; k < Remainder; k++)
		{
			Int sum;
			sum = *(esrc += (Qwidth + k)) * c[0];
			sum += *(esrc += srcStride) * c[1];
			sum += *(esrc += srcStride) * c[2];
			sum += *(esrc += srcStride) * c[3];
			sum += *(esrc += srcStride) * c[4];
			sum += *(esrc += srcStride) * c[5];
			sum += *(esrc += srcStride) * c[6];
			sum += *(esrc += srcStride) * c[7];

			Short val = (sum + offset) >> shift;
			dst[Qwidth + k] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}

}


Void TComInterpolationFilter::ETRI_FilterV_4T8W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int     row, col;
	Int16 *esrc, *Osrc;
	__m128i xmm[4], tmm[4], cmm[3];

	esrc = Osrc = (Short *)src;
	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col += 8)
		{
			ETRI_SIMD_VERTICAL_4TLOAD(xmm, esrc, col, srcStride);
			ETRI_SIMD_4x8_TRANSPOSE(xmm, tmm);
			ETRI_SIMD_4TAB_FILTER(xmm, xmm, cmm, shift);

			_mm_store_si128((__m128i *)&dst[col], xmm[0]);

			esrc = Osrc;
		}

		Osrc += srcStride;
		dst += dstStride;
		esrc = Osrc;
	}

}


Void TComInterpolationFilter::ETRI_FilterV_8T8W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], Lmm[8], cmm[5];
	esrc = Osrc = (Short *)src;


	cmm[0] = _mm_set1_epi32(*(Int32 *)c);
	cmm[1] = _mm_set1_epi32(*(Int32 *)(c + 2));
	cmm[2] = _mm_set1_epi32(*(Int32 *)(c + 4));
	cmm[3] = _mm_set1_epi32(*(Int32 *)(c + 6));
	cmm[4] = _mm_set1_epi32((Int32)offset);

	for (row = col = 0; col < width; col += 8)
	{
		Lmm[0] = _mm_loadu_si128((__m128i const *)(esrc += col));
		Lmm[1] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[2] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[3] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[4] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[5] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[6] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));
		Lmm[7] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

		xmm[0] = _mm_unpacklo_epi16(Lmm[0], Lmm[1]);
		xmm[1] = _mm_unpackhi_epi16(Lmm[0], Lmm[1]);
		xmm[2] = _mm_unpacklo_epi16(Lmm[2], Lmm[3]);
		xmm[3] = _mm_unpackhi_epi16(Lmm[2], Lmm[3]);
		xmm[4] = _mm_unpacklo_epi16(Lmm[4], Lmm[5]);
		xmm[5] = _mm_unpackhi_epi16(Lmm[4], Lmm[5]);
		xmm[6] = _mm_unpacklo_epi16(Lmm[6], Lmm[7]);
		xmm[7] = _mm_unpackhi_epi16(Lmm[6], Lmm[7]);

		xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
		xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
		xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
		xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
		xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
		xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
		xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
		xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
		xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
		xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

		xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
		xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
		xmm[0] = _mm_srai_epi32(xmm[0], shift);
		xmm[1] = _mm_srai_epi32(xmm[1], shift);
		xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

		_mm_store_si128((__m128i *)&dst[col], xmm[0]);

		for (row = 1; row < height; row++)
		{
			Lmm[(row - 1) & 0x07] = _mm_loadu_si128((__m128i const *)(esrc += srcStride));

			xmm[0] = _mm_unpacklo_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
			xmm[1] = _mm_unpackhi_epi16(Lmm[row & 0x07], Lmm[(row + 1) & 0x07]); \
			xmm[2] = _mm_unpacklo_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
			xmm[3] = _mm_unpackhi_epi16(Lmm[(row + 2) & 0x07], Lmm[(row + 3) & 0x07]); \
			xmm[4] = _mm_unpacklo_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
			xmm[5] = _mm_unpackhi_epi16(Lmm[(row + 4) & 0x07], Lmm[(row + 5) & 0x07]); \
			xmm[6] = _mm_unpacklo_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \
			xmm[7] = _mm_unpackhi_epi16(Lmm[(row + 6) & 0x07], Lmm[(row + 7) & 0x07]); \

			xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]);
			xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]);
			xmm[2] = _mm_madd_epi16(xmm[2], cmm[1]);
			xmm[3] = _mm_madd_epi16(xmm[3], cmm[1]);
			xmm[4] = _mm_madd_epi16(xmm[4], cmm[2]);
			xmm[5] = _mm_madd_epi16(xmm[5], cmm[2]);
			xmm[6] = _mm_madd_epi16(xmm[6], cmm[3]);
			xmm[7] = _mm_madd_epi16(xmm[7], cmm[3]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[2]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[4]);
			xmm[0] = _mm_add_epi32(xmm[0], xmm[6]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[3]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[5]);
			xmm[1] = _mm_add_epi32(xmm[1], xmm[7]);

			xmm[0] = _mm_add_epi32(xmm[0], cmm[4]);
			xmm[1] = _mm_add_epi32(xmm[1], cmm[4]);
			xmm[0] = _mm_srai_epi32(xmm[0], shift);
			xmm[1] = _mm_srai_epi32(xmm[1], shift);
			xmm[0] = _mm_packs_epi32(xmm[0], xmm[1]);

			_mm_store_si128((__m128i *)&dst[col + row*dstStride], xmm[0]);

		}
		esrc = Osrc;
	}

}


Void TComInterpolationFilter::ETRI_FilterV_8T4W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int k, row, col;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], tmm[8], cmm[3];
	ALIGNED(16) Int16 est[8];
	esrc = Osrc = (Short *)src;

	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);

	for (col = 0, row = 0; row < height; row++)
	{
		ETRI_SIMD_VERTICAL_LOAD(xmm, esrc, col, srcStride);
		ETRI_SIMD_8x8_TRANSPOSE(xmm, tmm, xmm);
		ETRI_SIMD_8TAB_FILTER(xmm, tmm, cmm, shift);
		_mm_store_si128((__m128i *)est, xmm[0]);

		*(Int64 *)dst = *(Int64 *)est;

		for (k = 0; k<(width & 0x03); k++)
			dst[4 + k] = est[4 + k];

		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}

}


Void TComInterpolationFilter::ETRI_FilterV_8T2W(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col;
	Int16 *esrc, *Osrc;
	esrc = Osrc = (Short *)src;

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += srcStride) * c[1];
			sum += *(esrc += srcStride) * c[2];
			sum += *(esrc += srcStride) * c[3];
			sum += *(esrc += srcStride) * c[4];
			sum += *(esrc += srcStride) * c[5];
			sum += *(esrc += srcStride) * c[6];
			sum += *(esrc += srcStride) * c[7];

			Short val = (sum + offset) >> shift;
			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}


Void TComInterpolationFilter::ETRI_FilterH_8T2WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int    row, col;
	Int16 *esrc, *Osrc;
	esrc = Osrc = (Short *)src;

	Short   maxVal = (1 << bitDepth) - 1;    //2013 03 04 by seok  : No call this function

	EDPRINTF(stderr, "Active !!!\n");

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += 1) * c[1];
			sum += *(esrc += 1) * c[2];
			sum += *(esrc += 1) * c[3];
			sum += *(esrc += 1) * c[4];
			sum += *(esrc += 1) * c[5];
			sum += *(esrc += 1) * c[6];
			sum += *(esrc += 1) * c[7];

			Short val = (sum + offset) >> shift;
			val = (val < 0) ? 0 : val;
			val = (val > maxVal) ? maxVal : val;

			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}

}

Void TComInterpolationFilter::ETRI_FilterH_4T8WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int  row, col;

	Int16 *esrc, *Osrc;
	__m128i xmm[4], cmm[4], cmm_mx;

	esrc = Osrc = (Short *)src;
	Short maxVal = (1 << bitDepth) - 1; 	///< 2015 8 7 by Seok

	cmm_mx = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok
	cmm[0] = _mm_load_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	cmm[3] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col += 8)
		{
			xmm[0] = _mm_loadu_si128((__m128i const *)(&esrc[col])); \
			xmm[3] = _mm_loadu_si128((__m128i const *)(&esrc[col + 3])); \
			xmm[1] = _mm_srli_si128(xmm[0], 2); \
			xmm[2] = _mm_slli_si128(xmm[3], 4); \
			xmm[1] = _mm_or_si128(xmm[1], xmm[2]); \
			xmm[2] = _mm_srli_si128(xmm[0], 4); \
			cmm[2] = _mm_slli_si128(xmm[3], 2); \
			xmm[2] = _mm_or_si128(xmm[2], cmm[2]); \

			xmm[0] = _mm_madd_epi16(xmm[0], cmm[0]); //(7 6) (5 4) (3 2) (1 0)
			xmm[1] = _mm_madd_epi16(xmm[1], cmm[0]); //(8 7) (6 5) (4 3) (2 1)
			xmm[2] = _mm_madd_epi16(xmm[2], cmm[0]); //(9 8) (7 6) (5 4) (3 2)
			xmm[3] = _mm_madd_epi16(xmm[3], cmm[0]);  //(10 9) (8 7) (6 5) (4 3)
			xmm[0] = _mm_hadd_epi32(xmm[0], xmm[1]);  //(8 7 6 5) (4 3 2 1) (7 6 5 4) (3 2 1 0)
			xmm[2] = _mm_hadd_epi32(xmm[2], xmm[3]);  //(10 9 8 7) (6 5 4 3) (9 8 7 6) (5 4 3 2)

			xmm[1] = _mm_unpacklo_epi32(xmm[0], xmm[2]);	//(9 8 7 6)(7 6 5 4)(5 4 3 2)(3 2 1 0)
			xmm[3] = _mm_unpackhi_epi32(xmm[0], xmm[2]);	//(10 9 8 7)(8 7 6 5)(6 5 4 3)(4 3 2 1)
			xmm[0] = _mm_unpacklo_epi32(xmm[1], xmm[3]);	//(6 5 4 3)(5 4 3 2)(4 3 2 1)(3 2 1 0)
			xmm[2] = _mm_unpackhi_epi32(xmm[1], xmm[3]);	//(10 9 8 7)(9 8 7 6)(8 7 6 5)(7 6 5 4)

			xmm[0] = _mm_add_epi32(xmm[0], cmm[1]);
			xmm[2] = _mm_add_epi32(xmm[2], cmm[1]);
			xmm[0] = _mm_srai_epi32(xmm[0], shift);
			xmm[2] = _mm_srai_epi32(xmm[2], shift);
			xmm[0] = _mm_packs_epi32(xmm[0], xmm[2]);

			ETRI_CLIP_BITLIMIT(xmm[0], cmm_mx, cmm[3]);
			_mm_store_si128((__m128i *)&dst[col], xmm[0]);

			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}

/**
	@ brief 	Inactive Function??? : Almost inactive function!!! \
			In most cases, the width of the images are multiple of 8, owing to the minimum width of LCU is 8. \
			Thus, with respect to the horizaontal filtering, a remainder of 8 does not exist. \
			However, for the vertical filtering, since there are not the limitation of height that should be mutiple of 8,\
			there is a remainder of 8 in vertical filtering.
*/
Void TComInterpolationFilter::ETRI_FilterH_8T8WRLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int row, col;
	int k, Qwidth, Remainder = width & 0x07;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], cmm[4], cmm_mx;
	esrc = Osrc = (Short *)src;
	Qwidth = width - Remainder;

	Short  maxVal = (1 << bitDepth) - 1;    //2013 03 04 by seok

	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	cmm[3] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok

	cmm_mx = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < Qwidth; col += 8)
		{
			ETRI_SIMD_SERIAL_LOAD(xmm, esrc, col, cmm[2]);
			ETRI_SIMD_8TAB_FILTER(xmm, xmm, cmm, shift);

			ETRI_CLIP_BITLIMIT(xmm[0], cmm_mx, cmm[3]);

			_mm_storeu_si128((__m128i *)&dst[col], xmm[0]);
			esrc = Osrc;
		}

		for (k = 0; k < Remainder; k++)
		{
			Int sum;
			sum = *(esrc += Qwidth + k) * c[0];
			sum += *(esrc += 1) * c[1];
			sum += *(esrc += 1) * c[2];
			sum += *(esrc += 1) * c[3];
			sum += *(esrc += 1) * c[4];
			sum += *(esrc += 1) * c[5];
			sum += *(esrc += 1) * c[6];
			sum += *(esrc += 1) * c[7];

			Short val = (sum + offset) >> shift;
			val = (val < 0) ? 0 : val;
			val = (val > maxVal) ? maxVal : val;

			dst[Qwidth + k] = val;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}//for (row = 0; row < height; row++)
}

Void TComInterpolationFilter::ETRI_FilterH_8T8WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int  row, col;
	Int16 *esrc, *Osrc;
	__m128i xmm[8], cmm[4], cmm_mx;

	esrc = Osrc = (Short *)src;
	Short maxVal = (1 << bitDepth) - 1; 	///< 2015 8 7 by Seok

	cmm_mx = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok

	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	cmm[3] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col += 8)
		{
			ETRI_SIMD_SERIAL_LOAD(xmm, esrc, col, cmm[2]);
			ETRI_SIMD_8TAB_FILTER(xmm, xmm, cmm, shift);

			ETRI_CLIP_BITLIMIT(xmm[0], cmm_mx, cmm[3]);

			_mm_store_si128((__m128i *)&dst[col], xmm[0]);
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}//for (row = 0; row < height; row++)
}

Void TComInterpolationFilter::ETRI_FilterH_8T4WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int  row, col;
	Int16 *esrc, *Osrc, k;
	__m128i xmm[8], cmm[4], cmm_mx;
	ALIGNED(16) Int16 est[8];

	esrc = Osrc = (Short *)src;
	Short  maxVal = (1 << bitDepth) - 1;	//2013 03 04 by seok

	///< 2015 8 7 by Seok
	cmm_mx = _mm_set1_epi16(maxVal);		///< xmm_mx : 2015 8 7 by Seok

	cmm[0] = _mm_loadu_si128((__m128i const *)c);
	cmm[1] = _mm_set1_epi32((Int32)offset);
	cmm[3] = _mm_setzero_si128();			///< xmm_mn : 2015 8 7 by Seok

	for (col = 0, row = 0; row < height; row++)
	{
		ETRI_SIMD_SERIAL_LOAD(xmm, esrc, col, cmm[2]);
		ETRI_SIMD_8TAB_FILTER(xmm, xmm, cmm, shift);
		ETRI_CLIP_BITLIMIT(xmm[0], cmm_mx, cmm[3]);

		_mm_store_si128((__m128i *)est, xmm[0]);

		*(Int64 *)dst = *(Int64 *)est;

		for (k = 0; k<(width & 0x03); k++)
			dst[4 + k] = est[4 + k];

		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}

Void TComInterpolationFilter::ETRI_FilterH_4T4WLast(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int offset, Int shift, Short *c)
{
	int row, col;
	Int16 *esrc, *Osrc;
	esrc = Osrc = (Short *)src;

	Short  maxVal = (1 << bitDepth) - 1;    //2013 03 04 by seok

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;
			sum = *(esrc += col) * c[0];
			sum += *(esrc += 1) * c[1];
			sum += *(esrc += 1) * c[2];
			sum += *(esrc += 1) * c[3];

			Short val = (sum + offset) >> shift;
			val = (val < 0) ? 0 : val;
			val = (val > maxVal) ? maxVal : val;

			dst[col] = val;
			esrc = Osrc;
		}
		Osrc += srcStride; esrc = Osrc;
		dst += dstStride;
	}
}

#endif

#if 0
/**
====================================================================================================================
	@brief	Old Code based on Function Pointer 
====================================================================================================================
*/
template<int N, bool isVertical, bool isFirst, bool isLast>
Void TComInterpolationFilter::ETRI_filter(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Short const *coeff)
{
	Int     offset, IdisLast;
	Int     headRoom = IF_INTERNAL_PREC - bitDepth;
	Int     shift = IF_FILTER_PREC;
	__declspec(align(16)) Short c[8];

	Int    fnIdx;
	TComInterpolationFilter ETRIIF;

	if (isLast)
	{
		shift += (isFirst) ? 0 : headRoom;
		offset = 1 << (shift - 1);
		offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
		IdisLast = 1;
	}
	else
	{
		shift -= (isFirst) ? headRoom : 0;
		offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
		IdisLast = 0;
	}

	if (isVertical)
	{
		if (N == 4)
		{
			src -= srcStride;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)coeff;

			fnIdx = (((width >= 8) && ((width & 0x07) == 0)) ? 0 : 2) + IdisLast;
			ETRIIF.em_pfFilterFunc[fnIdx](bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);
		}
		else //if ( N == 8 )
		{
			src -= 3 * srcStride;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)&coeff[4];

			if (width >= 8)
			{
				fnIdx = ((width & 0x07) ? 4 : 6) + IdisLast;
				ETRIIF.em_pfFilterFunc[fnIdx](bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);
			}
			else
			{
				fnIdx = ((width >= 4) ? 8 : 10) + IdisLast;
				ETRIIF.em_pfFilterFunc[fnIdx](bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);
			}
		}
	}
	else    //Horizontal 
	{
		if (N == 4)
		{
			src -= 1;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)coeff;

			fnIdx = (((width >= 8) && ((width & 0x07) == 0)) ? 12 : 14) + IdisLast;
			ETRIIF.em_pfFilterFunc[fnIdx](bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);
		}
		else //if ( N == 8 )
		{
			src -= 3;
			*(Int64 *)c = *(Int64 *)coeff;
			*(Int64 *)&c[4] = *(Int64 *)&coeff[4];

			if (width >= 8)
			{
				fnIdx = ((width & 0x07) ? 16 : 18) + IdisLast;
				ETRIIF.em_pfFilterFunc[fnIdx](bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);
			}
			else
			{  // width < 8
				fnIdx = ((width >= 4) ? 20 : 22) + IdisLast;
				ETRIIF.em_pfFilterFunc[fnIdx](bitDepth, src, srcStride, dst, dstStride, width, height, offset, shift, c);

			} //if (width >=8)
		}// if N==8?
	}//End of Horizontal
}
#endif



//! \}
