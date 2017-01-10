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

/** \file     TComTrQuant.cpp
    \brief    transform and quantization class
*/

#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include "TComTrQuant.h"
#include "TComPic.h"
#include "ContextTables.h"

typedef struct
{
  Int    iNNZbeforePos0;
  Double d64CodedLevelandDist; // distortion and level cost only
  Double d64UncodedDist;    // all zero coded block distortion
  Double d64SigCost;
  Double d64SigCost_0;
} coeffGroupRDStats;

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define RDOQ_CHROMA                 1           ///< use of RDOQ in chroma

// ====================================================================================================================
// Tables
// ====================================================================================================================

// RDOQ parameter

// ====================================================================================================================
// Qp class member functions
// ====================================================================================================================

QpParam::QpParam()
{
}

#if ETRI_SIMD_MATRIX_TRANSFORM

#define ALIGN_VAR_8(T, var)  ALIGNED(8)  T var
#define ALIGN_VAR_16(T, var) ALIGNED(16) T var
#define ALIGN_VAR_32(T, var) ALIGNED(32) T var

#define SHIFT_32T_1ST  6
#define ADD_32T_1      32
#define SHIFT_32T_2ND  11
#define ADD_32T_2      1024

#define SHIFT_16T_1ST  5
#define ADD_16T_1      16
#define SHIFT_16T_2ND  10
#define ADD_16T_2      512

#define SHIFT_IT_1ST 7
#define ADD_IT_1     64
#define SHIFT_IT_2ND 10
#define ADD_IT_2     512

const Short g_aiIT4[4][4] =
{
    { 64, 83, 64, 36 },
    { 64, 36, -64, -83 },
    { 64, -36, -64, 83 },
    { 64, -83, 64, -36 }
};
const Short g_as_IDST_MAT_4[4][4] =
{
    { 29, 74, 84, 55 },
    { 55, 74, -29, -84 },
    { 74, 0, -74, 74 },
    { 84, -74, 55, -29 },
};
const Short g_aiIT8[8][8] =
{
    { 64, 89, 83, 75, 64, 50, 36, 18 },
    { 64, 75, 36, -18, -64, -89, -83, -50 },
    { 64, 50, -36, -89, -64, 18, 83, 75 },
    { 64, 18, -83, -50, 64, 75, -36, -89 },
    { 64, -18, -83, 50, 64, -75, -36, 89 },
    { 64, -50, -36, 89, -64, -18, 83, -75 },
    { 64, -75, 36, 18, -64, 89, -83, 50 },
    { 64, -89, 83, -75, 64, -50, 36, -18 }
};

ALIGN_VAR_32(static const int16_t, tab_dct_8[][8]) =
{
    { 0x0100, 0x0F0E, 0x0706, 0x0908, 0x0302, 0x0D0C, 0x0504, 0x0B0A },

    { 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, -64, 64, -64, 64, -64, 64, -64 },
    { 83, 36, 83, 36, 83, 36, 83, 36 },
    { 36, -83, 36, -83, 36, -83, 36, -83 },
    { 89, 18, 75, 50, 89, 18, 75, 50 },
    { 75, -50, -18, -89, 75, -50, -18, -89 },
    { 50, 75, -89, 18, 50, 75, -89, 18 },
    { 18, -89, -50, 75, 18, -89, -50, 75 },

    { 83, 83, -83, -83, 36, 36, -36, -36 },
    { 36, 36, -36, -36, -83, -83, 83, 83 },
    { 89, -89, 18, -18, 75, -75, 50, -50 },
    { 75, -75, -50, 50, -18, 18, -89, 89 },
    { 50, -50, 75, -75, -89, 89, 18, -18 },
    { 18, -18, -89, 89, -50, 50, 75, -75 },
};
ALIGN_VAR_32(static const int16_t, tab_idct_8x8[12][8]) =
{
    { 89, 75, 89, 75, 89, 75, 89, 75 },
    { 50, 18, 50, 18, 50, 18, 50, 18 },
    { 75, -18, 75, -18, 75, -18, 75, -18 },
    { -89, -50, -89, -50, -89, -50, -89, -50 },
    { 50, -89, 50, -89, 50, -89, 50, -89 },
    { 18, 75, 18, 75, 18, 75, 18, 75 },
    { 18, -50, 18, -50, 18, -50, 18, -50 },
    { 75, -89, 75, -89, 75, -89, 75, -89 },
    { 64, 64, 64, 64, 64, 64, 64, 64 },
    { 64, -64, 64, -64, 64, -64, 64, -64 },
    { 83, 36, 83, 36, 83, 36, 83, 36 },
    { 36, -83, 36, -83, 36, -83, 36, -83 }
};

ALIGN_VAR_32(static const int16_t, tab_dct_16_0[][8]) =
{
    { 0x0F0E, 0x0D0C, 0x0B0A, 0x0908, 0x0706, 0x0504, 0x0302, 0x0100 },  // 0
    { 0x0100, 0x0F0E, 0x0706, 0x0908, 0x0302, 0x0D0C, 0x0504, 0x0B0A },  // 1
    { 0x0100, 0x0706, 0x0302, 0x0504, 0x0F0E, 0x0908, 0x0D0C, 0x0B0A },  // 2
    { 0x0F0E, 0x0908, 0x0D0C, 0x0B0A, 0x0100, 0x0706, 0x0302, 0x0504 },  // 3
};

ALIGN_VAR_32(static const int16_t, tab_dct_16_1[][8]) =
{
    { 90, 87, 80, 70, 57, 43, 25, 9 },         //  0
    { 87, 57, 9, -43, -80, -90, -70, -25 },    //  1
    { 80, 9, -70, -87, -25, 57, 90, 43 },      //  2
    { 70, -43, -87, 9, 90, 25, -80, -57 },     //  3
    { 57, -80, -25, 90, -9, -87, 43, 70 },     //  4
    { 43, -90, 57, 25, -87, 70, 9, -80 },      //  5
    { 25, -70, 90, -80, 43, 9, -57, 87 },      //  6
    { 9, -25, 43, -57, 70, -80, 87, -90 },     //  7
    { 83, 83, -83, -83, 36, 36, -36, -36 },    //  8
    { 36, 36, -36, -36, -83, -83, 83, 83 },    //  9
    { 89, 89, 18, 18, 75, 75, 50, 50 },        // 10
    { 75, 75, -50, -50, -18, -18, -89, -89 },  // 11
    { 50, 50, 75, 75, -89, -89, 18, 18 },      // 12
    { 18, 18, -89, -89, -50, -50, 75, 75 },    // 13

#define MAKE_COEF(a0, a1, a2, a3, a4, a5, a6, a7) \
                  { (a0), -(a0), (a3), -(a3), (a1), -(a1), (a2), -(a2) \
                  }, \
                  { (a7), -(a7), (a4), -(a4), (a6), -(a6), (a5), -(a5) },

    MAKE_COEF(90, 87, 80, 70, 57, 43, 25, 9)
    MAKE_COEF(87, 57, 9, -43, -80, -90, -70, -25)
    MAKE_COEF(80, 9, -70, -87, -25, 57, 90, 43)
    MAKE_COEF(70, -43, -87, 9, 90, 25, -80, -57)
    MAKE_COEF(57, -80, -25, 90, -9, -87, 43, 70)
    MAKE_COEF(43, -90, 57, 25, -87, 70, 9, -80)
    MAKE_COEF(25, -70, 90, -80, 43, 9, -57, 87)
    MAKE_COEF(9, -25, 43, -57, 70, -80, 87, -90)
#undef MAKE_COEF
};
ALIGN_VAR_32(static const int16_t, tab_dct_32_0[][8]) =
{
    { 0x0F0E, 0x0100, 0x0908, 0x0706, 0x0D0C, 0x0302, 0x0B0A, 0x0504 },  // 0
};

ALIGN_VAR_32(static const int16_t, tab_dct_32_1[][8]) =
{
    { 89, -89, 18, -18, 75, -75, 50, -50 },          //  0
    { 75, -75, -50, 50, -18, 18, -89, 89 },          //  1
    { 50, -50, 75, -75, -89, 89, 18, -18 },          //  2
    { 18, -18, -89, 89, -50, 50, 75, -75 },          //  3

#define MAKE_COEF8(a0, a1, a2, a3, a4, a5, a6, a7) \
                  { (a0), (a7), (a3), (a4), (a1), (a6), (a2), (a5) \
                  }, \

    MAKE_COEF8(90, 87, 80, 70, 57, 43, 25, 9)   //  4
    MAKE_COEF8(87, 57, 9, -43, -80, -90, -70, -25)   //  5
    MAKE_COEF8(80, 9, -70, -87, -25, 57, 90, 43)   //  6
    MAKE_COEF8(70, -43, -87, 9, 90, 25, -80, -57)   //  7
    MAKE_COEF8(57, -80, -25, 90, -9, -87, 43, 70)   //  8
    MAKE_COEF8(43, -90, 57, 25, -87, 70, 9, -80)   //  9
    MAKE_COEF8(25, -70, 90, -80, 43, 9, -57, 87)   // 10
    MAKE_COEF8(9, -25, 43, -57, 70, -80, 87, -90)   // 11
#undef MAKE_COEF8

#define MAKE_COEF16(a00, a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12, a13, a14, a15) \
                  { (a00), (a07), (a03), (a04), (a01), (a06), (a02), (a05) }, \
                  { (a15), (a08), (a12), (a11), (a14), (a09), (a13), (a10) },

    MAKE_COEF16(90, 90, 88, 85, 82, 78, 73, 67, 61, 54, 46, 38, 31, 22, 13, 4)    // 12
    MAKE_COEF16(90, 82, 67, 46, 22, -4, -31, -54, -73, -85, -90, -88, -78, -61, -38, -13)    // 14
    MAKE_COEF16(88, 67, 31, -13, -54, -82, -90, -78, -46, -4, 38, 73, 90, 85, 61, 22)    // 16
    MAKE_COEF16(85, 46, -13, -67, -90, -73, -22, 38, 82, 88, 54, -4, -61, -90, -78, -31)    // 18
    MAKE_COEF16(82, 22, -54, -90, -61, 13, 78, 85, 31, -46, -90, -67, 4, 73, 88, 38)    // 20
    MAKE_COEF16(78, -4, -82, -73, 13, 85, 67, -22, -88, -61, 31, 90, 54, -38, -90, -46)    // 22
    MAKE_COEF16(73, -31, -90, -22, 78, 67, -38, -90, -13, 82, 61, -46, -88, -4, 85, 54)    // 24
    MAKE_COEF16(67, -54, -78, 38, 85, -22, -90, 4, 90, 13, -88, -31, 82, 46, -73, -61)    // 26
    MAKE_COEF16(61, -73, -46, 82, 31, -88, -13, 90, -4, -90, 22, 85, -38, -78, 54, 67)    // 28
    MAKE_COEF16(54, -85, -4, 88, -46, -61, 82, 13, -90, 38, 67, -78, -22, 90, -31, -73)    // 30
    MAKE_COEF16(46, -90, 38, 54, -90, 31, 61, -88, 22, 67, -85, 13, 73, -82, 4, 78)    // 32
    MAKE_COEF16(38, -88, 73, -4, -67, 90, -46, -31, 85, -78, 13, 61, -90, 54, 22, -82)    // 34
    MAKE_COEF16(31, -78, 90, -61, 4, 54, -88, 82, -38, -22, 73, -90, 67, -13, -46, 85)    // 36
    MAKE_COEF16(22, -61, 85, -90, 73, -38, -4, 46, -78, 90, -82, 54, -13, -31, 67, -88)    // 38
    MAKE_COEF16(13, -38, 61, -78, 88, -90, 85, -73, 54, -31, 4, 22, -46, 67, -82, 90)    // 40
    MAKE_COEF16(4, -13, 22, -31, 38, -46, 54, -61, 67, -73, 78, -82, 85, -88, 90, -90)    // 42
#undef MAKE_COEF16

    {
        64, 64, 64, 64, 64, 64, 64, 64
    },                                  // 44

    { 64, 64, -64, -64, -64, -64, 64, 64 },  // 45

    { 83, 83, 36, 36, -36, -36, -83, -83 },  // 46
    { -83, -83, -36, -36, 36, 36, 83, 83 },  // 47

    { 36, 36, -83, -83, 83, 83, -36, -36 },  // 48
    { -36, -36, 83, 83, -83, -83, 36, 36 },  // 49

#define MAKE_COEF16(a00, a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12, a13, a14, a15) \
                                    { (a00), (a00), (a01), (a01), (a02), (a02), (a03), (a03) }, \
                                    { (a04), (a04), (a05), (a05), (a06), (a06), (a07), (a07) }, \
                                    { (a08), (a08), (a09), (a09), (a10), (a10), (a11), (a11) }, \
                                    { (a12), (a12), (a13), (a13), (a14), (a14), (a15), (a15) },

    MAKE_COEF16(89, 75, 50, 18, -18, -50, -75, -89, -89, -75, -50, -18, 18, 50, 75, 89) // 50
    MAKE_COEF16(75, -18, -89, -50, 50, 89, 18, -75, -75, 18, 89, 50, -50, -89, -18, 75) // 54

    // TODO: convert below table here
#undef MAKE_COEF16

        {
            50, 50, -89, -89, 18, 18, 75, 75
        },                                  // 58
        { -75, -75, -18, -18, 89, 89, -50, -50 },  // 59
        { -50, -50, 89, 89, -18, -18, -75, -75 },  // 60
        { 75, 75, 18, 18, -89, -89, 50, 50 },  // 61

        { 18, 18, -50, -50, 75, 75, -89, -89 },  // 62
        { 89, 89, -75, -75, 50, 50, -18, -18 },  // 63
        { -18, -18, 50, 50, -75, -75, 89, 89 },  // 64
        { -89, -89, 75, 75, -50, -50, 18, 18 },  // 65

        { 90, 90, 87, 87, 80, 80, 70, 70 },  // 66
        { 57, 57, 43, 43, 25, 25, 9, 9 },  // 67
        { -9, -9, -25, -25, -43, -43, -57, -57 },  // 68
        { -70, -70, -80, -80, -87, -87, -90, -90 },  // 69

        { 87, 87, 57, 57, 9, 9, -43, -43 },  // 70
        { -80, -80, -90, -90, -70, -70, -25, -25 },  // 71
        { 25, 25, 70, 70, 90, 90, 80, 80 },  // 72
        { 43, 43, -9, -9, -57, -57, -87, -87 },  // 73

        { 80, 80, 9, 9, -70, -70, -87, -87 },  // 74
        { -25, -25, 57, 57, 90, 90, 43, 43 },  // 75
        { -43, -43, -90, -90, -57, -57, 25, 25 },  // 76
        { 87, 87, 70, 70, -9, -9, -80, -80 },  // 77

        { 70, 70, -43, -43, -87, -87, 9, 9 },  // 78
        { 90, 90, 25, 25, -80, -80, -57, -57 },  // 79
        { 57, 57, 80, 80, -25, -25, -90, -90 },  // 80
        { -9, -9, 87, 87, 43, 43, -70, -70 },  // 81

        { 57, 57, -80, -80, -25, -25, 90, 90 },  // 82
        { -9, -9, -87, -87, 43, 43, 70, 70 },  // 83
        { -70, -70, -43, -43, 87, 87, 9, 9 },  // 84
        { -90, -90, 25, 25, 80, 80, -57, -57 },  // 85

        { 43, 43, -90, -90, 57, 57, 25, 25 },  // 86
        { -87, -87, 70, 70, 9, 9, -80, -80 },  // 87
        { 80, 80, -9, -9, -70, -70, 87, 87 },  // 88
        { -25, -25, -57, -57, 90, 90, -43, -43 },  // 89

        { 25, 25, -70, -70, 90, 90, -80, -80 },  // 90
        { 43, 43, 9, 9, -57, -57, 87, 87 },  // 91
        { -87, -87, 57, 57, -9, -9, -43, -43 },  // 92
        { 80, 80, -90, -90, 70, 70, -25, -25 },  // 93

        { 9, 9, -25, -25, 43, 43, -57, -57 },  // 94
        { 70, 70, -80, -80, 87, 87, -90, -90 },  // 95
        { 90, 90, -87, -87, 80, 80, -70, -70 },  // 96
        { 57, 57, -43, -43, 25, 25, -9, -9 },  // 97

#define MAKE_COEF16(a00, a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12, a13, a14, a15) \
                                                      { (a00), -(a00), (a01), -(a01), (a02), -(a02), (a03), -(a03) }, \
                                                      { (a04), -(a04), (a05), -(a05), (a06), -(a06), (a07), -(a07) }, \
                                                      { (a08), -(a08), (a09), -(a09), (a10), -(a10), (a11), -(a11) }, \
                                                      { (a12), -(a12), (a13), -(a13), (a14), -(a14), (a15), -(a15) },

        MAKE_COEF16(90, 90, 88, 85, 82, 78, 73, 67, 61, 54, 46, 38, 31, 22, 13, 4)    // 98
        MAKE_COEF16(90, 82, 67, 46, 22, -4, -31, -54, -73, -85, -90, -88, -78, -61, -38, -13)     //102
        MAKE_COEF16(88, 67, 31, -13, -54, -82, -90, -78, -46, -4, 38, 73, 90, 85, 61, 22)     //106
        MAKE_COEF16(85, 46, -13, -67, -90, -73, -22, 38, +82, 88, 54, -4, -61, -90, -78, -31)     //110
        MAKE_COEF16(82, 22, -54, -90, -61, 13, 78, 85, +31, -46, -90, -67, 4, 73, 88, 38)     //114
        MAKE_COEF16(78, -4, -82, -73, 13, 85, 67, -22, -88, -61, 31, 90, 54, -38, -90, -46)     //118
        MAKE_COEF16(73, -31, -90, -22, 78, 67, -38, -90, -13, 82, 61, -46, -88, -4, 85, 54)     //122
        MAKE_COEF16(67, -54, -78, 38, 85, -22, -90, 4, +90, 13, -88, -31, 82, 46, -73, -61)     //126
        MAKE_COEF16(61, -73, -46, 82, 31, -88, -13, 90, -4, -90, 22, 85, -38, -78, 54, 67)     //130
        MAKE_COEF16(54, -85, -4, 88, -46, -61, 82, 13, -90, 38, 67, -78, -22, 90, -31, -73)     //134
        MAKE_COEF16(46, -90, 38, 54, -90, 31, 61, -88, +22, 67, -85, 13, 73, -82, 4, 78)     //138
        MAKE_COEF16(38, -88, 73, -4, -67, 90, -46, -31, +85, -78, 13, 61, -90, 54, 22, -82)     //142
        MAKE_COEF16(31, -78, 90, -61, 4, 54, -88, 82, -38, -22, 73, -90, 67, -13, -46, 85)     //146
        MAKE_COEF16(22, -61, 85, -90, 73, -38, -4, 46, -78, 90, -82, 54, -13, -31, 67, -88)     //150
        MAKE_COEF16(13, -38, 61, -78, 88, -90, 85, -73, +54, -31, 4, 22, -46, 67, -82, 90)     //154
        MAKE_COEF16(4, -13, 22, -31, 38, -46, 54, -61, +67, -73, 78, -82, 85, -88, 90, -90)     //158

#undef MAKE_COEF16
};


#endif 
// ====================================================================================================================
// TComTrQuant class member functions
// ====================================================================================================================

TComTrQuant::TComTrQuant()
{
  m_cQP.clear();
  
  // allocate temporary buffers
  m_plTempCoeff  = new Int[ MAX_CU_SIZE*MAX_CU_SIZE ];
  
  // allocate bit estimation class  (for RDOQ)
  m_pcEstBitsSbac = new estBitsSbacStruct;
  initScalingList();
}

TComTrQuant::~TComTrQuant()
{
  // delete temporary buffers
  if ( m_plTempCoeff )
  {
    delete [] m_plTempCoeff;
    m_plTempCoeff = NULL;
  }
  
  // delete bit estimation class
  if ( m_pcEstBitsSbac )
  {
    delete m_pcEstBitsSbac;
  }
  destroyScalingList();
}

#if ADAPTIVE_QP_SELECTION
Void TComTrQuant::storeSliceQpNext(TComSlice* pcSlice)
{
  Int qpBase = pcSlice->getSliceQpBase();
  Int sliceQpused = pcSlice->getSliceQp();
  Int sliceQpnext;
  Double alpha = qpBase < 17 ? 0.5 : 1;
  
  Int cnt=0;
  for(Int u=1; u<=LEVEL_RANGE; u++)
  { 
    cnt += m_sliceNsamples[u] ;
  }

  if( !m_useRDOQ )
  {
    sliceQpused = qpBase;
    alpha = 0.5;
  }

  if( cnt > 120 )
  {
    Double sum = 0;
    Int k = 0;
    for(Int u=1; u<LEVEL_RANGE; u++)
    {
      sum += u*m_sliceSumC[u];
      k += u*u*m_sliceNsamples[u];
    }

    Int v;
    Double q[MAX_QP+1] ;
    for(v=0; v<=MAX_QP; v++)
    {
      q[v] = (Double)(g_invQuantScales[v%6] * (1<<(v/6)))/64 ;
    }

    Double qnext = sum/k * q[sliceQpused] / (1<<ARL_C_PRECISION);

    for(v=0; v<MAX_QP; v++)
    {
      if(qnext < alpha * q[v] + (1 - alpha) * q[v+1] )
      {
        break;
      }
    }
    sliceQpnext = Clip3(sliceQpused - 3, sliceQpused + 3, v);
  }
  else
  {
    sliceQpnext = sliceQpused;
  }

  m_qpDelta[qpBase] = sliceQpnext - qpBase; 
}

Void TComTrQuant::initSliceQpDelta()
{
  for(Int qp=0; qp<=MAX_QP; qp++)
  {
    m_qpDelta[qp] = qp < 17 ? 0 : 1;
  }
}

Void TComTrQuant::clearSliceARLCnt()
{ 
  memset(m_sliceSumC, 0, sizeof(Double)*(LEVEL_RANGE+1));
  memset(m_sliceNsamples, 0, sizeof(Int)*(LEVEL_RANGE+1));
}
#endif


/** Set qP for Quantization.
 * \param qpy QPy
 * \param bLowpass
 * \param eSliceType
 * \param eTxtType
 * \param qpBdOffset
 * \param chromaQPOffset
 *
 * return void  
 */
Void TComTrQuant::setQPforQuant( Int qpy, TextType eTxtType, Int qpBdOffset, Int chromaQPOffset)
{
  Int qpScaled;

  if(eTxtType == TEXT_LUMA)
  {
    qpScaled = qpy + qpBdOffset;
  }
  else
  {
    qpScaled = Clip3( -qpBdOffset, 57, qpy + chromaQPOffset );

    if(qpScaled < 0)
    {
      qpScaled = qpScaled + qpBdOffset;
    }
    else
    {
      qpScaled = g_aucChromaScale[ qpScaled ] + qpBdOffset;
    }
  }
  m_cQP.setQpParam( qpScaled );
}

#if MATRIX_MULT
/** NxN forward transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param block pointer to input data (residual)
 *  \param coeff pointer to output data (transform coefficients)
 *  \param uiStride stride of input data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
void xTr(Int bitDepth, Pel *block, Int *coeff, UInt uiStride, UInt uiTrSize, UInt uiMode)
{
  Int i,j,k,iSum;
  Int tmp[32*32];
  const Short *iT;
  UInt uiLog2TrSize = g_aucConvertToBit[ uiTrSize ] + 2;

  if (uiTrSize==4)
  {
    iT  = g_aiT4[0];
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[0];
  }
  else
  {
    assert(0);
  }

  Int shift_1st = uiLog2TrSize - 1 + bitDepth-8; // log2(N) - 1 + g_bitDepth-8
  Int add_1st = 1<<(shift_1st-1);
  Int shift_2nd = uiLog2TrSize + 6;
  Int add_2nd = 1<<(shift_2nd-1);

  /* Horizontal transform */

  if (uiTrSize==4)
  {
    if (uiMode != REG_DCT && g_aucDCTDSTMode_Hor[uiMode])
    {
      iT  =  g_as_DST_MAT_4[0];
    }
  }
  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*block[j*uiStride+k];
      }
      tmp[i*uiTrSize+j] = (iSum + add_1st)>>shift_1st;
    }
  }
  
  /* Vertical transform */
  if (uiTrSize==4)
  {
    if (uiMode != REG_DCT && g_aucDCTDSTMode_Vert[uiMode])
    {
      iT  =  g_as_DST_MAT_4[0];
    }
    else
    {
      iT  = g_aiT4[0];
    }
  }
  for (i=0; i<uiTrSize; i++)
  {                 
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*tmp[j*uiTrSize+k];        
      }
      coeff[i*uiTrSize+j] = (iSum + add_2nd)>>shift_2nd; 
    }
  }
}

/** NxN inverse transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param coeff pointer to input data (transform coefficients)
 *  \param block pointer to output data (residual)
 *  \param uiStride stride of output data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
void xITr(Int *coeff, Pel *block, UInt uiStride, UInt uiTrSize, UInt uiMode)
{
  Int i,j,k,iSum;
  Int tmp[32*32];
  const Short *iT;
  
  if (uiTrSize==4)
  {
    iT  = g_aiT4[0];
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[0];
  }
  else
  {
    assert(0);
  }
  
  Int shift_1st = SHIFT_INV_1ST;
  Int add_1st = 1<<(shift_1st-1);
  Int shift_2nd = SHIFT_INV_2ND - g_bitDepth-8;
  Int add_2nd = 1<<(shift_2nd-1);
  if (uiTrSize==4)
  {
    if (uiMode != REG_DCT && g_aucDCTDSTMode_Vert[uiMode] ) // Check for DCT or DST
    {
      iT  =  g_as_DST_MAT_4[0];
    }
  }
  
  /* Horizontal transform */
  for (i=0; i<uiTrSize; i++)
  {    
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {        
        iSum += iT[k*uiTrSize+i]*coeff[k*uiTrSize+j]; 
      }
      tmp[i*uiTrSize+j] = Clip3(-32768, 32767, (iSum + add_1st)>>shift_1st); // Clipping is normative
    }
  }   
  
  if (uiTrSize==4)
  {
    if (uiMode != REG_DCT && g_aucDCTDSTMode_Hor[uiMode] )   // Check for DCT or DST
    {
      iT  =  g_as_DST_MAT_4[0];
    }
    else  
    {
      iT  = g_aiT4[0];
    }
  }
  
  /* Vertical transform */
  for (i=0; i<uiTrSize; i++)
  {   
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {        
        iSum += iT[k*uiTrSize+j]*tmp[i*uiTrSize+k];
      }
      block[i*uiStride+j] = Clip3(-32768, 32767, (iSum + add_2nd)>>shift_2nd); // Clipping is non-normative
    }
  }
}

#else //MATRIX_MULT

/** 4x4 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 */

void partialButterfly4(Short *src,Short *dst,Int shift, Int line)
{
	Int add = 1 << (shift - 1);

#if ETRI_SIMD_TR
	// [JDS]: Variable and Constant for SIMD code  
	__m128i xmm[6], cmm[3];

	// [JDS]: Data load and alignment for 16 pixels
	xmm[0] = _mm_loadu_si128((__m128i*)(src));
	xmm[1] = _mm_loadu_si128((__m128i*)(src + 8));
	cmm[0] = _mm_shuffle_epi32(_mm_shufflehi_epi16(_mm_shufflelo_epi16(xmm[0], _MM_SHUFFLE(2, 1, 3, 0)), _MM_SHUFFLE(2, 1, 3, 0)), 216);
	cmm[1] = _mm_shuffle_epi32(_mm_shufflehi_epi16(_mm_shufflelo_epi16(xmm[1], _MM_SHUFFLE(2, 1, 3, 0)), _MM_SHUFFLE(2, 1, 3, 0)), 216);
	xmm[0] = _mm_unpacklo_epi64(cmm[0], cmm[1]);
	xmm[1] = _mm_unpackhi_epi64(cmm[0], cmm[1]);

	// [JDS]: Compute "dst" 
	cmm[0] = _mm_set1_epi32(add);
	cmm[1] = _mm_set1_epi16(64);
	xmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[1]), _mm_madd_epi16(xmm[1], cmm[1])), cmm[0]), shift);
	xmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0], cmm[1]), _mm_madd_epi16(xmm[1], cmm[1])), cmm[0]), shift);

	cmm[1] = _mm_unpacklo_epi16(_mm_set1_epi16(83), _mm_set1_epi16(-83));
	cmm[2] = _mm_unpacklo_epi16(_mm_set1_epi16(36), _mm_set1_epi16(-36));
	xmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[1]), _mm_madd_epi16(xmm[1], cmm[2])), cmm[0]), shift);
	xmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0], cmm[2]), _mm_madd_epi16(xmm[1], cmm[1])), cmm[0]), shift);

	// [JDS]: Data combine to make "dst" 
	_mm_storeu_si128((__m128i*)(dst), _mm_packs_epi32(xmm[2], xmm[4]));
	_mm_storeu_si128((__m128i*)(dst + 8), _mm_packs_epi32(xmm[3], xmm[5]));
#else 

  Int j;
  Int E[2],O[2];

  for (j=0; j<line; j++)
  {    
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0] = (g_aiT4[0][0]*E[0] + g_aiT4[0][1]*E[1] + add)>>shift;
    dst[2*line] = (g_aiT4[2][0]*E[0] + g_aiT4[2][1]*E[1] + add)>>shift;
    dst[line] = (g_aiT4[1][0]*O[0] + g_aiT4[1][1]*O[1] + add)>>shift;
    dst[3*line] = (g_aiT4[3][0]*O[0] + g_aiT4[3][1]*O[1] + add)>>shift;

    src += 4;
    dst ++;
  }
#endif // #if ETRI_SIMD_TR
}

// Fast DST Algorithm. Full matrix multiplication for DST and Fast DST algorithm 
// give identical results
void fastForwardDst(Short *block,Short *coeff,Int shift)  // input block, output coeff
{
  Int rnd_factor = 1<<(shift-1);

#if ETRI_SIMD_TR
  // [JDS]: Variables for SIMD code 
  __m128i xmm[8], cmm[8], xmmZero;

  // [JDS]: Set the constants
  xmmZero = _mm_setzero_si128();
  cmm[0] = _mm_set1_epi32(29);
  cmm[1] = _mm_set1_epi32(55);
  cmm[2] = _mm_set1_epi32(74);
  cmm[3] = _mm_set1_epi32(rnd_factor);

  // [JDS]: Data load & alignment
  xmm[4] = _mm_unpacklo_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i*)(block)), _mm_loadu_si128((__m128i*)(block + 8))),
	  _mm_unpackhi_epi16(_mm_loadu_si128((__m128i*)(block)), _mm_loadu_si128((__m128i*)(block + 8))));
  xmm[5] = _mm_unpackhi_epi16(_mm_unpacklo_epi16(_mm_loadu_si128((__m128i*)(block)), _mm_loadu_si128((__m128i*)(block + 8))),
	  _mm_unpackhi_epi16(_mm_loadu_si128((__m128i*)(block)), _mm_loadu_si128((__m128i*)(block + 8))));
  xmm[0] = _mm_unpacklo_epi16(xmm[4], _mm_cmpgt_epi16(xmmZero, xmm[4]));
  xmm[1] = _mm_unpackhi_epi16(xmm[4], _mm_cmpgt_epi16(xmmZero, xmm[4]));
  xmm[2] = _mm_unpacklo_epi16(xmm[5], _mm_cmpgt_epi16(xmmZero, xmm[5]));
  xmm[3] = _mm_unpackhi_epi16(xmm[5], _mm_cmpgt_epi16(xmmZero, xmm[5]));

  // [JDS]: Precompute 
  cmm[4] = _mm_sub_epi32(_mm_add_epi32(xmm[0], xmm[1]), xmm[3]);

  // [JDS]: Compute intermediate variables                                    
  xmm[4] = _mm_add_epi32(xmm[0], xmm[3]);
  xmm[5] = _mm_add_epi32(xmm[1], xmm[3]);
  xmm[6] = _mm_sub_epi32(xmm[0], xmm[1]);
  xmm[7] = _mm_mullo_epi32(xmm[2], cmm[2]);

  // [JDS]: Compute coefficients
  xmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_mullo_epi32(xmm[4], cmm[0]), _mm_mullo_epi32(xmm[5], cmm[1])), xmm[7]), cmm[3]), shift);
  xmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(cmm[2], cmm[4]), cmm[3]), shift);
  xmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_mullo_epi32(xmm[6], cmm[0]), _mm_mullo_epi32(xmm[4], cmm[1])), xmm[7]), cmm[3]), shift);
  xmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_mullo_epi32(xmm[6], cmm[1]), _mm_mullo_epi32(xmm[5], cmm[0])), xmm[7]), cmm[3]), shift);

  // [JDS]: Data combine 
  _mm_storeu_si128((__m128i*)(coeff), _mm_packs_epi32(xmm[0], xmm[1]));
  _mm_storeu_si128((__m128i*)(coeff + 8), _mm_packs_epi32(xmm[2], xmm[3]));
#else 
  Int i, c[4];
  for (i=0; i<4; i++)
  {
    // Intermediate Variables
    c[0] = block[4*i+0] + block[4*i+3];
    c[1] = block[4*i+1] + block[4*i+3];
    c[2] = block[4*i+0] - block[4*i+1];
    c[3] = 74* block[4*i+2];

    coeff[   i] =  ( 29 * c[0] + 55 * c[1]         + c[3]               + rnd_factor ) >> shift;
    coeff[ 4+i] =  ( 74 * (block[4*i+0]+ block[4*i+1] - block[4*i+3])   + rnd_factor ) >> shift;
    coeff[ 8+i] =  ( 29 * c[2] + 55 * c[0]         - c[3]               + rnd_factor ) >> shift;
    coeff[12+i] =  ( 55 * c[2] - 29 * c[1]         + c[3]               + rnd_factor ) >> shift;
  }
#endif // #if ETRI_SIMD_TR
}

void fastInverseDst(Short *tmp,Short *block,Int shift)  // input tmp, output block
{
  Int rnd_factor = 1<<(shift-1);

#if ETRI_SIMD_TR
  // [JDS]: Variables for SIMD code
  __m128i xmm[8], cmm[8];

  // [JDS]: Set the constants
  cmm[0] = _mm_setzero_si128();
  cmm[1] = _mm_set1_epi32(29);
  cmm[2] = _mm_set1_epi32(55);
  cmm[3] = _mm_set1_epi32(74);
  cmm[4] = _mm_set1_epi32(rnd_factor);

  // [JDS]: Data load & alignment
  xmm[0] = _mm_unpacklo_epi16(_mm_loadu_si128((__m128i*)(tmp)), _mm_cmpgt_epi16(cmm[0], _mm_loadu_si128((__m128i*)(tmp))));
  xmm[1] = _mm_unpackhi_epi16(_mm_loadu_si128((__m128i*)(tmp)), _mm_cmpgt_epi16(cmm[0], _mm_loadu_si128((__m128i*)(tmp))));
  xmm[2] = _mm_unpacklo_epi16(_mm_loadu_si128((__m128i*)(tmp + 8)), _mm_cmpgt_epi16(cmm[0], _mm_loadu_si128((__m128i*)(tmp + 8))));
  xmm[3] = _mm_unpackhi_epi16(_mm_loadu_si128((__m128i*)(tmp + 8)), _mm_cmpgt_epi16(cmm[0], _mm_loadu_si128((__m128i*)(tmp + 8))));

  // [JDS]: Precompute 
  cmm[5] = _mm_add_epi32(_mm_sub_epi32(xmm[0], xmm[2]), xmm[3]);

  // [JDS]: Compute intermediate variables   
  xmm[4] = _mm_add_epi32(xmm[0], xmm[2]);
  xmm[5] = _mm_add_epi32(xmm[2], xmm[3]);
  xmm[6] = _mm_sub_epi32(xmm[0], xmm[3]);
  xmm[7] = _mm_mullo_epi32(xmm[1], cmm[3]);

  // [JDS]: Compute coefficients
  xmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_mullo_epi32(xmm[4], cmm[1]), _mm_mullo_epi32(xmm[5], cmm[2])), xmm[7]), cmm[4]), shift);
  xmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_mullo_epi32(xmm[6], cmm[2]), _mm_mullo_epi32(xmm[5], cmm[1])), xmm[7]), cmm[4]), shift);
  xmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(cmm[5], cmm[3]), cmm[4]), shift);
  xmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_mullo_epi32(xmm[4], cmm[2]), _mm_mullo_epi32(xmm[6], cmm[1])), xmm[7]), cmm[4]), shift);

  // [JDS]: Data transpose & combine
  _mm_storeu_si128((__m128i*)(block), _mm_unpacklo_epi16(_mm_unpacklo_epi16(_mm_packs_epi32(xmm[0], xmm[1]), _mm_packs_epi32(xmm[2], xmm[3])), _mm_unpackhi_epi16(_mm_packs_epi32(xmm[0], xmm[1]), _mm_packs_epi32(xmm[2], xmm[3]))));
  _mm_storeu_si128((__m128i*)(block + 8), _mm_unpackhi_epi16(_mm_unpacklo_epi16(_mm_packs_epi32(xmm[0], xmm[1]), _mm_packs_epi32(xmm[2], xmm[3])), _mm_unpackhi_epi16(_mm_packs_epi32(xmm[0], xmm[1]), _mm_packs_epi32(xmm[2], xmm[3]))));
#else
  Int i, c[4];
  for (i=0; i<4; i++)
  {  
    // Intermediate Variables
    c[0] = tmp[  i] + tmp[ 8+i];
    c[1] = tmp[8+i] + tmp[12+i];
    c[2] = tmp[  i] - tmp[12+i];
    c[3] = 74* tmp[4+i];

    block[4*i+0] = Clip3( -32768, 32767, ( 29 * c[0] + 55 * c[1]     + c[3]               + rnd_factor ) >> shift );
    block[4*i+1] = Clip3( -32768, 32767, ( 55 * c[2] - 29 * c[1]     + c[3]               + rnd_factor ) >> shift );
    block[4*i+2] = Clip3( -32768, 32767, ( 74 * (tmp[i] - tmp[8+i]  + tmp[12+i])      + rnd_factor ) >> shift );
    block[4*i+3] = Clip3( -32768, 32767, ( 55 * c[0] + 29 * c[2]     - c[3]               + rnd_factor ) >> shift );
  }
#endif // #if ETRI_SIMD_TR
}

void partialButterflyInverse4(Short *src,Short *dst,Int shift, Int line)
{
  Int add = 1<<(shift-1);


#if ETRI_SIMD_TR
  // [JDS]: Variable for SIMD code  
  __m128i xmm[6], cmm[3];

  // [JDS]: Data load and alignment for 16 pixels 
  xmm[0] = _mm_unpacklo_epi16(_mm_loadu_si128((__m128i*)(src)), _mm_loadu_si128((__m128i*)(src + 8)));
  xmm[1] = _mm_unpackhi_epi16(_mm_loadu_si128((__m128i*)(src)), _mm_loadu_si128((__m128i*)(src + 8)));

  // [JDS]: Set the constant and Compute "E" and "O"  (E0->E1->O0->O1)
  cmm[0] = _mm_set1_epi32(add);
  cmm[1] = _mm_set1_epi16(64);
  xmm[2] = _mm_madd_epi16(xmm[0], cmm[1]);
  xmm[3] = _mm_madd_epi16(xmm[0], _mm_unpacklo_epi16(cmm[1], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1]))));

  cmm[1] = _mm_set1_epi16(83);
  cmm[2] = _mm_set1_epi16(36);
  xmm[4] = _mm_madd_epi16(xmm[1], _mm_unpacklo_epi16(cmm[1], cmm[2]));
  xmm[5] = _mm_madd_epi16(xmm[1], _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1]))));

  // Compute "dst"
  xmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(xmm[2], xmm[4]), cmm[0]), shift);
  xmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(xmm[2], xmm[4]), cmm[0]), shift);
  xmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(xmm[3], xmm[5]), cmm[0]), shift);
  xmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(xmm[3], xmm[5]), cmm[0]), shift);

  // [JDS]: Convert "int" to "short" and transpose
  xmm[0] = _mm_packs_epi32(xmm[0], xmm[2]);
  xmm[1] = _mm_packs_epi32(xmm[4], xmm[1]);
  xmm[2] = _mm_unpacklo_epi16(xmm[0], xmm[1]);
  xmm[3] = _mm_unpackhi_epi16(xmm[0], xmm[1]);
  xmm[0] = _mm_unpacklo_epi16(xmm[2], xmm[3]);
  xmm[1] = _mm_unpackhi_epi16(xmm[2], xmm[3]);
  _mm_storeu_si128((__m128i*)(dst), xmm[0]);
  _mm_storeu_si128((__m128i*)(dst + 8), xmm[1]);
#else

  Int j;
  Int E[2], O[2];

  for (j=0; j<line; j++)
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */    
    O[0] = g_aiT4[1][0]*src[line] + g_aiT4[3][0]*src[3*line];
    O[1] = g_aiT4[1][1]*src[line] + g_aiT4[3][1]*src[3*line];
    E[0] = g_aiT4[0][0]*src[0] + g_aiT4[2][0]*src[2*line];
    E[1] = g_aiT4[0][1]*src[0] + g_aiT4[2][1]*src[2*line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3( -32768, 32767, (E[0] + O[0] + add)>>shift );
    dst[1] = Clip3( -32768, 32767, (E[1] + O[1] + add)>>shift );
    dst[2] = Clip3( -32768, 32767, (E[1] - O[1] + add)>>shift );
    dst[3] = Clip3( -32768, 32767, (E[0] - O[0] + add)>>shift );
            
    src   ++;
    dst += 4;
  }
#endif // #if ETRI_SIMD_TR
}


void partialButterfly8(Short *src,Short *dst,Int shift, Int line)
{
  Int add = 1<<(shift-1);

#if ETRI_SIMD_TR
  __m128i xmm[8], cmm[8], ADD;

  // [JDS]: Data load for 8x8(64) src samples
  cmm[0] = _mm_loadu_si128((__m128i*)(src));    cmm[1] = _mm_loadu_si128((__m128i*)(src + 8));  cmm[2] = _mm_loadu_si128((__m128i*)(src + 16)); cmm[3] = _mm_loadu_si128((__m128i*)(src + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + 32)); cmm[5] = _mm_loadu_si128((__m128i*)(src + 40)); cmm[6] = _mm_loadu_si128((__m128i*)(src + 48)); cmm[7] = _mm_loadu_si128((__m128i*)(src + 56));

  // [JDS]: Data alignment to apply madd --> Merit compared to previous codes: Remove SIMD operations for E, EE, O, and OO
  xmm[0] = _mm_unpacklo_epi16(cmm[0], cmm[1]); xmm[1] = _mm_unpackhi_epi16(cmm[0], cmm[1]); xmm[2] = _mm_unpacklo_epi16(cmm[2], cmm[3]); xmm[3] = _mm_unpackhi_epi16(cmm[2], cmm[3]);
  cmm[0] = _mm_unpacklo_epi32(xmm[0], xmm[2]); cmm[1] = _mm_unpackhi_epi32(xmm[0], xmm[2]); cmm[2] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[1], xmm[3]), 78); cmm[3] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[1], xmm[3]), 78);
  xmm[0] = _mm_unpacklo_epi16(cmm[0], cmm[3]); xmm[1] = _mm_unpackhi_epi16(cmm[0], cmm[3]); xmm[2] = _mm_unpacklo_epi16(cmm[1], cmm[2]); xmm[3] = _mm_unpackhi_epi16(cmm[1], cmm[2]);
  xmm[4] = _mm_unpacklo_epi16(cmm[4], cmm[5]); xmm[5] = _mm_unpackhi_epi16(cmm[4], cmm[5]); xmm[6] = _mm_unpacklo_epi16(cmm[6], cmm[7]); xmm[7] = _mm_unpackhi_epi16(cmm[6], cmm[7]);
  cmm[4] = _mm_unpacklo_epi32(xmm[4], xmm[6]); cmm[5] = _mm_unpackhi_epi32(xmm[4], xmm[6]); cmm[6] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[5], xmm[7]), 78); cmm[7] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[5], xmm[7]), 78);
  xmm[4] = _mm_unpacklo_epi16(cmm[4], cmm[7]); xmm[5] = _mm_unpackhi_epi16(cmm[4], cmm[7]); xmm[6] = _mm_unpacklo_epi16(cmm[5], cmm[6]); xmm[7] = _mm_unpackhi_epi16(cmm[5], cmm[6]);

  // [JDS]: Compute dst[0]
  ADD = _mm_set1_epi32(add);  cmm[0] = _mm_set1_epi16(64);
  cmm[1] = _mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[0]), _mm_madd_epi16(xmm[3], cmm[0]));   cmm[2] = _mm_add_epi32(_mm_madd_epi16(xmm[1], cmm[0]), _mm_madd_epi16(xmm[2], cmm[0]));
  cmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[1], cmm[2]), ADD), shift);         cmm[1] = _mm_add_epi32(_mm_madd_epi16(xmm[4], cmm[0]), _mm_madd_epi16(xmm[7], cmm[0]));
  cmm[2] = _mm_add_epi32(_mm_madd_epi16(xmm[5], cmm[0]), _mm_madd_epi16(xmm[6], cmm[0]));   cmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[1], cmm[2]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst), _mm_packs_epi32(cmm[3], cmm[4]));

  // [JDS]: Compute dst[4*line]
  cmm[1] = _mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[0]), _mm_madd_epi16(xmm[3], cmm[0]));   cmm[2] = _mm_add_epi32(_mm_madd_epi16(xmm[1], cmm[0]), _mm_madd_epi16(xmm[2], cmm[0]));
  cmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(cmm[1], cmm[2]), ADD), shift);        cmm[1] = _mm_add_epi32(_mm_madd_epi16(xmm[4], cmm[0]), _mm_madd_epi16(xmm[7], cmm[0]));
  cmm[2] = _mm_add_epi32(_mm_madd_epi16(xmm[5], cmm[0]), _mm_madd_epi16(xmm[6], cmm[0]));   cmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(cmm[1], cmm[2]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + 4 * line), _mm_packs_epi32(cmm[3], cmm[4]));

  // [JDS]: Compute dst[2*line]
  cmm[0] = _mm_set1_epi16(83);  cmm[1] = _mm_set1_epi16(36);
  cmm[2] = _mm_sub_epi32(_mm_madd_epi16(xmm[0], cmm[0]), _mm_madd_epi16(xmm[3], cmm[0]));   cmm[3] = _mm_sub_epi32(_mm_madd_epi16(xmm[1], cmm[1]), _mm_madd_epi16(xmm[2], cmm[1]));
  cmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[2], cmm[3]), ADD), shift);         cmm[2] = _mm_sub_epi32(_mm_madd_epi16(xmm[4], cmm[0]), _mm_madd_epi16(xmm[7], cmm[0]));
  cmm[3] = _mm_sub_epi32(_mm_madd_epi16(xmm[5], cmm[1]), _mm_madd_epi16(xmm[6], cmm[1]));   cmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[2], cmm[3]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + 2 * line), _mm_packs_epi32(cmm[4], cmm[5]));

  // [JDS]: Compute dst[6*line]
  cmm[2] = _mm_sub_epi32(_mm_madd_epi16(xmm[0], cmm[1]), _mm_madd_epi16(xmm[3], cmm[1]));   cmm[3] = _mm_sub_epi32(_mm_madd_epi16(xmm[1], cmm[0]), _mm_madd_epi16(xmm[2], cmm[0]));
  cmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(cmm[2], cmm[3]), ADD), shift);        cmm[2] = _mm_sub_epi32(_mm_madd_epi16(xmm[4], cmm[1]), _mm_madd_epi16(xmm[7], cmm[1]));
  cmm[3] = _mm_sub_epi32(_mm_madd_epi16(xmm[5], cmm[0]), _mm_madd_epi16(xmm[6], cmm[0]));   cmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(cmm[2], cmm[3]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + 6 * line), _mm_packs_epi32(cmm[4], cmm[5]));

  // [JDS]: Compute dst[line]
  cmm[0] = _mm_unpacklo_epi16(_mm_set1_epi16(89), _mm_set1_epi16(-89));            cmm[1] = _mm_unpacklo_epi16(_mm_set1_epi16(75), _mm_set1_epi16(-75));
  cmm[2] = _mm_unpacklo_epi16(_mm_set1_epi16(50), _mm_set1_epi16(-50));            cmm[3] = _mm_unpacklo_epi16(_mm_set1_epi16(18), _mm_set1_epi16(-18));
  cmm[4] = _mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[0]), _mm_madd_epi16(xmm[1], cmm[1]));    cmm[5] = _mm_add_epi32(_mm_madd_epi16(xmm[2], cmm[2]), _mm_madd_epi16(xmm[3], cmm[3]));
  cmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[4], cmm[5]), ADD), shift);      cmm[4] = _mm_add_epi32(_mm_madd_epi16(xmm[4], cmm[0]), _mm_madd_epi16(xmm[5], cmm[1]));
  cmm[5] = _mm_add_epi32(_mm_madd_epi16(xmm[6], cmm[2]), _mm_madd_epi16(xmm[7], cmm[3]));   cmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[4], cmm[5]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + line), _mm_packs_epi32(cmm[6], cmm[7]));

  // [JDS]: Compute dst[3*line]
  cmm[4] = _mm_sub_epi32(_mm_madd_epi16(xmm[0], cmm[1]), _mm_madd_epi16(xmm[1], cmm[3]));   cmm[5] = _mm_add_epi32(_mm_madd_epi16(xmm[2], cmm[0]), _mm_madd_epi16(xmm[3], cmm[2]));
  cmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(cmm[4], cmm[5]), ADD), shift);        cmm[4] = _mm_sub_epi32(_mm_madd_epi16(xmm[4], cmm[1]), _mm_madd_epi16(xmm[5], cmm[3]));
  cmm[5] = _mm_add_epi32(_mm_madd_epi16(xmm[6], cmm[0]), _mm_madd_epi16(xmm[7], cmm[2]));   cmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(cmm[4], cmm[5]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + 3 * line), _mm_packs_epi32(cmm[6], cmm[7]));

  // [JDS]: Compute dst[5*line]
  cmm[4] = _mm_sub_epi32(_mm_madd_epi16(xmm[0], cmm[2]), _mm_madd_epi16(xmm[1], cmm[0]));   cmm[5] = _mm_add_epi32(_mm_madd_epi16(xmm[2], cmm[3]), _mm_madd_epi16(xmm[3], cmm[1]));
  cmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[4], cmm[5]), ADD), shift);        cmm[4] = _mm_sub_epi32(_mm_madd_epi16(xmm[4], cmm[2]), _mm_madd_epi16(xmm[5], cmm[0]));
  cmm[5] = _mm_add_epi32(_mm_madd_epi16(xmm[6], cmm[3]), _mm_madd_epi16(xmm[7], cmm[1]));   cmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[4], cmm[5]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + 5 * line), _mm_packs_epi32(cmm[6], cmm[7]));

  // [JDS]: Compute dst[7*line]
  cmm[4] = _mm_sub_epi32(_mm_madd_epi16(xmm[0], cmm[3]), _mm_madd_epi16(xmm[1], cmm[2]));   cmm[5] = _mm_sub_epi32(_mm_madd_epi16(xmm[2], cmm[1]), _mm_madd_epi16(xmm[3], cmm[0]));
  cmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[4], cmm[5]), ADD), shift);        cmm[4] = _mm_sub_epi32(_mm_madd_epi16(xmm[4], cmm[3]), _mm_madd_epi16(xmm[5], cmm[2]));
  cmm[5] = _mm_sub_epi32(_mm_madd_epi16(xmm[6], cmm[1]), _mm_madd_epi16(xmm[7], cmm[0]));   cmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(cmm[4], cmm[5]), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + 7 * line), _mm_packs_epi32(cmm[6], cmm[7]));
#else

  Int j, k;
  Int E[4], O[4];
  Int EE[2], EO[2];

  for (j=0; j<line; j++)
  {  
    /* E and O*/
    for (k=0;k<4;k++)
    {
      E[k] = src[k] + src[7-k];
      O[k] = src[k] - src[7-k];
    }    
    /* EE and EO */
    EE[0] = E[0] + E[3];    
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    dst[0] = (g_aiT8[0][0]*EE[0] + g_aiT8[0][1]*EE[1] + add)>>shift;
    dst[4*line] = (g_aiT8[4][0]*EE[0] + g_aiT8[4][1]*EE[1] + add)>>shift; 
    dst[2*line] = (g_aiT8[2][0]*EO[0] + g_aiT8[2][1]*EO[1] + add)>>shift;
    dst[6*line] = (g_aiT8[6][0]*EO[0] + g_aiT8[6][1]*EO[1] + add)>>shift; 

    dst[line] = (g_aiT8[1][0]*O[0] + g_aiT8[1][1]*O[1] + g_aiT8[1][2]*O[2] + g_aiT8[1][3]*O[3] + add)>>shift;
    dst[3*line] = (g_aiT8[3][0]*O[0] + g_aiT8[3][1]*O[1] + g_aiT8[3][2]*O[2] + g_aiT8[3][3]*O[3] + add)>>shift;
    dst[5*line] = (g_aiT8[5][0]*O[0] + g_aiT8[5][1]*O[1] + g_aiT8[5][2]*O[2] + g_aiT8[5][3]*O[3] + add)>>shift;
    dst[7*line] = (g_aiT8[7][0]*O[0] + g_aiT8[7][1]*O[1] + g_aiT8[7][2]*O[2] + g_aiT8[7][3]*O[3] + add)>>shift;

    src += 8;
    dst ++;
  }
#endif // #if ETRI_SIMD_TR
}


void partialButterflyInverse8(Short *src,Short *dst,Int shift, Int line)
{
  Int add = 1<<(shift-1);

#if ETRI_SIMD_TR
  // [JDS]: Variable for SIMD code  
  __m128i xmm[8], cmm[8], omm[8], emm[8];

  // [JDS]: Data load and alignment
  cmm[0] = _mm_loadu_si128((__m128i*)(src));      cmm[1] = _mm_loadu_si128((__m128i*)(src + line));    cmm[2] = _mm_loadu_si128((__m128i*)(src + line * 2));  cmm[3] = _mm_loadu_si128((__m128i*)(src + line * 3));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + line * 4));  cmm[5] = _mm_loadu_si128((__m128i*)(src + line * 5));  cmm[6] = _mm_loadu_si128((__m128i*)(src + line * 6));  cmm[7] = _mm_loadu_si128((__m128i*)(src + line * 7));
  xmm[0] = _mm_unpacklo_epi16(cmm[1], cmm[3]);    xmm[1] = _mm_unpackhi_epi16(cmm[1], cmm[3]);    xmm[2] = _mm_unpacklo_epi16(cmm[5], cmm[7]);    xmm[3] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  xmm[4] = _mm_unpacklo_epi16(cmm[0], cmm[4]);    xmm[5] = _mm_unpackhi_epi16(cmm[0], cmm[4]);    xmm[6] = _mm_unpacklo_epi16(cmm[2], cmm[6]);    xmm[7] = _mm_unpackhi_epi16(cmm[2], cmm[6]);

  // [JDS]: Set the constant and Compute "O[k]"
  cmm[0] = _mm_set1_epi32(add);  cmm[1] = _mm_set1_epi16(89);  cmm[2] = _mm_set1_epi16(75);  cmm[3] = _mm_set1_epi16(50);  cmm[4] = _mm_set1_epi16(18);
  cmm[5] = _mm_unpacklo_epi16(cmm[1], cmm[2]);  cmm[6] = _mm_unpacklo_epi16(cmm[3], cmm[4]);
  omm[0] = _mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[5]), _mm_madd_epi16(xmm[2], cmm[6]));    omm[1] = _mm_add_epi32(_mm_madd_epi16(xmm[1], cmm[5]), _mm_madd_epi16(xmm[3], cmm[6]));
  cmm[5] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  cmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])), _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));
  omm[2] = _mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[5]), _mm_madd_epi16(xmm[2], cmm[6]));    omm[3] = _mm_add_epi32(_mm_madd_epi16(xmm[1], cmm[5]), _mm_madd_epi16(xmm[3], cmm[6]));
  cmm[5] = _mm_unpacklo_epi16(cmm[3], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));    cmm[6] = _mm_unpacklo_epi16(cmm[4], cmm[2]);
  omm[4] = _mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[5]), _mm_madd_epi16(xmm[2], cmm[6]));    omm[5] = _mm_add_epi32(_mm_madd_epi16(xmm[1], cmm[5]), _mm_madd_epi16(xmm[3], cmm[6]));
  cmm[5] = _mm_unpacklo_epi16(cmm[4], _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));
  cmm[6] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  omm[6] = _mm_add_epi32(_mm_madd_epi16(xmm[0], cmm[5]), _mm_madd_epi16(xmm[2], cmm[6]));    omm[7] = _mm_add_epi32(_mm_madd_epi16(xmm[1], cmm[5]), _mm_madd_epi16(xmm[3], cmm[6]));

  // [JDS]: Set the constant and Compute "E[k]"
  cmm[1] = _mm_set1_epi16(64);  cmm[2] = _mm_set1_epi16(83);  cmm[3] = _mm_set1_epi16(36);    cmm[5] = _mm_unpacklo_epi16(cmm[2], cmm[3]);
  emm[0] = _mm_add_epi32(_mm_madd_epi16(xmm[4], cmm[1]), _mm_madd_epi16(xmm[6], cmm[5]));    emm[1] = _mm_add_epi32(_mm_madd_epi16(xmm[5], cmm[1]), _mm_madd_epi16(xmm[7], cmm[5]));
  emm[6] = _mm_sub_epi32(_mm_madd_epi16(xmm[4], cmm[1]), _mm_madd_epi16(xmm[6], cmm[5]));    emm[7] = _mm_sub_epi32(_mm_madd_epi16(xmm[5], cmm[1]), _mm_madd_epi16(xmm[7], cmm[5]));
  cmm[4] = _mm_unpacklo_epi16(cmm[1], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  cmm[5] = _mm_unpacklo_epi16(cmm[3], _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])));
  emm[2] = _mm_add_epi32(_mm_madd_epi16(xmm[4], cmm[4]), _mm_madd_epi16(xmm[6], cmm[5]));    emm[3] = _mm_add_epi32(_mm_madd_epi16(xmm[5], cmm[4]), _mm_madd_epi16(xmm[7], cmm[5]));
  emm[4] = _mm_sub_epi32(_mm_madd_epi16(xmm[4], cmm[4]), _mm_madd_epi16(xmm[6], cmm[5]));    emm[5] = _mm_sub_epi32(_mm_madd_epi16(xmm[5], cmm[4]), _mm_madd_epi16(xmm[7], cmm[5]));

  // [JDS]: Compute "dst" 
  xmm[0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0], omm[0]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1], omm[1]), cmm[0]), shift));
  xmm[1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2], omm[2]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3], omm[3]), cmm[0]), shift));
  xmm[2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4], omm[4]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5], omm[5]), cmm[0]), shift));
  xmm[3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6], omm[6]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7], omm[7]), cmm[0]), shift));
  xmm[4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6], omm[6]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7], omm[7]), cmm[0]), shift));
  xmm[5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4], omm[4]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5], omm[5]), cmm[0]), shift));
  xmm[6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2], omm[2]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3], omm[3]), cmm[0]), shift));
  xmm[7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0], omm[0]), cmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1], omm[1]), cmm[0]), shift));

  // [JDS]: 8x8 Transpose for "dst"
  cmm[0] = _mm_unpacklo_epi16(xmm[0], xmm[2]);  cmm[1] = _mm_unpacklo_epi16(xmm[1], xmm[3]);  cmm[2] = _mm_unpacklo_epi16(xmm[4], xmm[6]);  cmm[3] = _mm_unpacklo_epi16(xmm[5], xmm[7]);
  cmm[4] = _mm_unpackhi_epi16(xmm[0], xmm[2]);  cmm[5] = _mm_unpackhi_epi16(xmm[1], xmm[3]);  cmm[6] = _mm_unpackhi_epi16(xmm[4], xmm[6]);  cmm[7] = _mm_unpackhi_epi16(xmm[5], xmm[7]);
  xmm[0] = _mm_unpacklo_epi16(cmm[0], cmm[1]);  xmm[1] = _mm_unpacklo_epi16(cmm[2], cmm[3]);  xmm[2] = _mm_unpacklo_epi16(cmm[4], cmm[5]);  xmm[3] = _mm_unpacklo_epi16(cmm[6], cmm[7]);
  xmm[4] = _mm_unpackhi_epi16(cmm[0], cmm[1]);  xmm[5] = _mm_unpackhi_epi16(cmm[2], cmm[3]);  xmm[6] = _mm_unpackhi_epi16(cmm[4], cmm[5]);  xmm[7] = _mm_unpackhi_epi16(cmm[6], cmm[7]);

  _mm_storeu_si128((__m128i*)(dst), _mm_unpacklo_epi64(xmm[0], xmm[1]));  _mm_storeu_si128((__m128i*)(dst + line), _mm_unpackhi_epi64(xmm[0], xmm[1]));
  _mm_storeu_si128((__m128i*)(dst + 2 * line), _mm_unpacklo_epi64(xmm[4], xmm[5]));  _mm_storeu_si128((__m128i*)(dst + 3 * line), _mm_unpackhi_epi64(xmm[4], xmm[5]));
  _mm_storeu_si128((__m128i*)(dst + 4 * line), _mm_unpacklo_epi64(xmm[2], xmm[3]));  _mm_storeu_si128((__m128i*)(dst + 5 * line), _mm_unpackhi_epi64(xmm[2], xmm[3]));
  _mm_storeu_si128((__m128i*)(dst + 6 * line), _mm_unpacklo_epi64(xmm[6], xmm[7]));  _mm_storeu_si128((__m128i*)(dst + 7 * line), _mm_unpackhi_epi64(xmm[6], xmm[7]));
#else

  Int j, k;
  Int E[4], O[4];
  Int EE[2], EO[2];

  for (j=0; j<line; j++) 
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<4;k++)
    {
      O[k] = g_aiT8[ 1][k]*src[line] + g_aiT8[ 3][k]*src[3*line] + g_aiT8[ 5][k]*src[5*line] + g_aiT8[ 7][k]*src[7*line];
    }

    EO[0] = g_aiT8[2][0]*src[ 2*line ] + g_aiT8[6][0]*src[ 6*line ];
    EO[1] = g_aiT8[2][1]*src[ 2*line ] + g_aiT8[6][1]*src[ 6*line ];
    EE[0] = g_aiT8[0][0]*src[ 0      ] + g_aiT8[4][0]*src[ 4*line ];
    EE[1] = g_aiT8[0][1]*src[ 0      ] + g_aiT8[4][1]*src[ 4*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */ 
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];
    for (k=0;k<4;k++)
    {
      dst[ k   ] = Clip3( -32768, 32767, (E[k] + O[k] + add)>>shift );
      dst[ k+4 ] = Clip3( -32768, 32767, (E[3-k] - O[3-k] + add)>>shift );
    }   
    src ++;
    dst += 8;
  }
#endif // #if ETRI_SIMD_TR
}


void partialButterfly16(Short *src,Short *dst,Int shift, Int line)
{
  Int add = 1<<(shift-1);

#if ETRI_SIMD_TR
  // [JDS]: Variable and Constant for SIMD code  
  __m128i xmm[4][8], cmm[8], rmm[8], ADD;

  // [JDS]: Data load and Data alignment "LL~HH" to apply madd --> Merit compared to previous codes: Remove SIMD operations for E, EE, EEE, O, EO, and EEO
  cmm[0] = _mm_loadu_si128((__m128i*)(src));      cmm[1] = _mm_loadu_si128((__m128i*)(src + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(src + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(src + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(src + 40));    cmm[6] = _mm_loadu_si128((__m128i*)(src + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(src + 56));
  xmm[0][0] = _mm_unpacklo_epi16(cmm[0], cmm[2]);    xmm[0][1] = _mm_unpackhi_epi16(cmm[0], cmm[2]);    xmm[0][2] = _mm_unpacklo_epi16(cmm[1], cmm[3]);    xmm[0][3] = _mm_unpackhi_epi16(cmm[1], cmm[3]);
  xmm[0][4] = _mm_unpacklo_epi16(cmm[4], cmm[6]);    xmm[0][5] = _mm_unpackhi_epi16(cmm[4], cmm[6]);    xmm[0][6] = _mm_unpacklo_epi16(cmm[5], cmm[7]);    xmm[0][7] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi32(xmm[0][0], xmm[0][4]);  cmm[1] = _mm_unpackhi_epi32(xmm[0][0], xmm[0][4]);  cmm[2] = _mm_unpacklo_epi32(xmm[0][1], xmm[0][5]);  cmm[3] = _mm_unpackhi_epi32(xmm[0][1], xmm[0][5]);
  cmm[4] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[0][2], xmm[0][6]), 78);  cmm[5] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[0][2], xmm[0][6]), 78);
  cmm[6] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[0][3], xmm[0][7]), 78);  cmm[7] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[0][3], xmm[0][7]), 78);
  xmm[0][0] = _mm_unpacklo_epi16(cmm[0], cmm[7]);    xmm[0][1] = _mm_unpackhi_epi16(cmm[0], cmm[7]);    xmm[0][2] = _mm_unpacklo_epi16(cmm[1], cmm[6]);    xmm[0][3] = _mm_unpackhi_epi16(cmm[1], cmm[6]);
  xmm[0][4] = _mm_unpacklo_epi16(cmm[2], cmm[5]);    xmm[0][5] = _mm_unpackhi_epi16(cmm[2], cmm[5]);    xmm[0][6] = _mm_unpacklo_epi16(cmm[3], cmm[4]);    xmm[0][7] = _mm_unpackhi_epi16(cmm[3], cmm[4]);

  cmm[0] = _mm_loadu_si128((__m128i*)(src + 64));    cmm[1] = _mm_loadu_si128((__m128i*)(src + 72));    cmm[2] = _mm_loadu_si128((__m128i*)(src + 80));    cmm[3] = _mm_loadu_si128((__m128i*)(src + 88));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + 96));    cmm[5] = _mm_loadu_si128((__m128i*)(src + 104));    cmm[6] = _mm_loadu_si128((__m128i*)(src + 112));    cmm[7] = _mm_loadu_si128((__m128i*)(src + 120));
  xmm[1][0] = _mm_unpacklo_epi16(cmm[0], cmm[2]);    xmm[1][1] = _mm_unpackhi_epi16(cmm[0], cmm[2]);    xmm[1][2] = _mm_unpacklo_epi16(cmm[1], cmm[3]);    xmm[1][3] = _mm_unpackhi_epi16(cmm[1], cmm[3]);
  xmm[1][4] = _mm_unpacklo_epi16(cmm[4], cmm[6]);    xmm[1][5] = _mm_unpackhi_epi16(cmm[4], cmm[6]);    xmm[1][6] = _mm_unpacklo_epi16(cmm[5], cmm[7]);    xmm[1][7] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi32(xmm[1][0], xmm[1][4]);  cmm[1] = _mm_unpackhi_epi32(xmm[1][0], xmm[1][4]);  cmm[2] = _mm_unpacklo_epi32(xmm[1][1], xmm[1][5]);  cmm[3] = _mm_unpackhi_epi32(xmm[1][1], xmm[1][5]);
  cmm[4] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[1][2], xmm[1][6]), 78);  cmm[5] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[1][2], xmm[1][6]), 78);
  cmm[6] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[1][3], xmm[1][7]), 78);  cmm[7] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[1][3], xmm[1][7]), 78);
  xmm[1][0] = _mm_unpacklo_epi16(cmm[0], cmm[7]);    xmm[1][1] = _mm_unpackhi_epi16(cmm[0], cmm[7]);    xmm[1][2] = _mm_unpacklo_epi16(cmm[1], cmm[6]);    xmm[1][3] = _mm_unpackhi_epi16(cmm[1], cmm[6]);
  xmm[1][4] = _mm_unpacklo_epi16(cmm[2], cmm[5]);    xmm[1][5] = _mm_unpackhi_epi16(cmm[2], cmm[5]);    xmm[1][6] = _mm_unpacklo_epi16(cmm[3], cmm[4]);    xmm[1][7] = _mm_unpackhi_epi16(cmm[3], cmm[4]);

  cmm[0] = _mm_loadu_si128((__m128i*)(src + 128));    cmm[1] = _mm_loadu_si128((__m128i*)(src + 136));    cmm[2] = _mm_loadu_si128((__m128i*)(src + 144));    cmm[3] = _mm_loadu_si128((__m128i*)(src + 152));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + 160));    cmm[5] = _mm_loadu_si128((__m128i*)(src + 168));    cmm[6] = _mm_loadu_si128((__m128i*)(src + 176));    cmm[7] = _mm_loadu_si128((__m128i*)(src + 184));
  xmm[2][0] = _mm_unpacklo_epi16(cmm[0], cmm[2]);    xmm[2][1] = _mm_unpackhi_epi16(cmm[0], cmm[2]);    xmm[2][2] = _mm_unpacklo_epi16(cmm[1], cmm[3]);    xmm[2][3] = _mm_unpackhi_epi16(cmm[1], cmm[3]);
  xmm[2][4] = _mm_unpacklo_epi16(cmm[4], cmm[6]);    xmm[2][5] = _mm_unpackhi_epi16(cmm[4], cmm[6]);    xmm[2][6] = _mm_unpacklo_epi16(cmm[5], cmm[7]);    xmm[2][7] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi32(xmm[2][0], xmm[2][4]);  cmm[1] = _mm_unpackhi_epi32(xmm[2][0], xmm[2][4]);  cmm[2] = _mm_unpacklo_epi32(xmm[2][1], xmm[2][5]);  cmm[3] = _mm_unpackhi_epi32(xmm[2][1], xmm[2][5]);
  cmm[4] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[2][2], xmm[2][6]), 78);  cmm[5] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[2][2], xmm[2][6]), 78);
  cmm[6] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[2][3], xmm[2][7]), 78);  cmm[7] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[2][3], xmm[2][7]), 78);
  xmm[2][0] = _mm_unpacklo_epi16(cmm[0], cmm[7]);    xmm[2][1] = _mm_unpackhi_epi16(cmm[0], cmm[7]);    xmm[2][2] = _mm_unpacklo_epi16(cmm[1], cmm[6]);    xmm[2][3] = _mm_unpackhi_epi16(cmm[1], cmm[6]);
  xmm[2][4] = _mm_unpacklo_epi16(cmm[2], cmm[5]);    xmm[2][5] = _mm_unpackhi_epi16(cmm[2], cmm[5]);    xmm[2][6] = _mm_unpacklo_epi16(cmm[3], cmm[4]);    xmm[2][7] = _mm_unpackhi_epi16(cmm[3], cmm[4]);

  cmm[0] = _mm_loadu_si128((__m128i*)(src + 192));    cmm[1] = _mm_loadu_si128((__m128i*)(src + 200));    cmm[2] = _mm_loadu_si128((__m128i*)(src + 208));    cmm[3] = _mm_loadu_si128((__m128i*)(src + 216));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + 224));    cmm[5] = _mm_loadu_si128((__m128i*)(src + 232));    cmm[6] = _mm_loadu_si128((__m128i*)(src + 240));    cmm[7] = _mm_loadu_si128((__m128i*)(src + 248));
  xmm[3][0] = _mm_unpacklo_epi16(cmm[0], cmm[2]);    xmm[3][1] = _mm_unpackhi_epi16(cmm[0], cmm[2]);    xmm[3][2] = _mm_unpacklo_epi16(cmm[1], cmm[3]);    xmm[3][3] = _mm_unpackhi_epi16(cmm[1], cmm[3]);
  xmm[3][4] = _mm_unpacklo_epi16(cmm[4], cmm[6]);    xmm[3][5] = _mm_unpackhi_epi16(cmm[4], cmm[6]);    xmm[3][6] = _mm_unpacklo_epi16(cmm[5], cmm[7]);    xmm[3][7] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi32(xmm[3][0], xmm[3][4]);  cmm[1] = _mm_unpackhi_epi32(xmm[3][0], xmm[3][4]);  cmm[2] = _mm_unpacklo_epi32(xmm[3][1], xmm[3][5]);  cmm[3] = _mm_unpackhi_epi32(xmm[3][1], xmm[3][5]);
  cmm[4] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[3][2], xmm[3][6]), 78);  cmm[5] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[3][2], xmm[3][6]), 78);
  cmm[6] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[3][3], xmm[3][7]), 78);  cmm[7] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[3][3], xmm[3][7]), 78);
  xmm[3][0] = _mm_unpacklo_epi16(cmm[0], cmm[7]);    xmm[3][1] = _mm_unpackhi_epi16(cmm[0], cmm[7]);    xmm[3][2] = _mm_unpacklo_epi16(cmm[1], cmm[6]);    xmm[3][3] = _mm_unpackhi_epi16(cmm[1], cmm[6]);
  xmm[3][4] = _mm_unpacklo_epi16(cmm[2], cmm[5]);    xmm[3][5] = _mm_unpackhi_epi16(cmm[2], cmm[5]);    xmm[3][6] = _mm_unpacklo_epi16(cmm[3], cmm[4]);    xmm[3][7] = _mm_unpackhi_epi16(cmm[3], cmm[4]);

  // [JDS]: Set the constands and compute dst(0), dst(8), dst(4), and dst(12)
  ADD = _mm_set1_epi32(add);  cmm[0] = _mm_set1_epi16(64);     cmm[1] = _mm_set1_epi16(83);        cmm[2] = _mm_set1_epi16(36);
  rmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[0][4], cmm[0])));
  rmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[0][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][5], cmm[0])));
  rmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[0]), _mm_madd_epi16(xmm[1][4], cmm[0])));
  rmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[0]), _mm_madd_epi16(xmm[1][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][5], cmm[0])));
  rmm[4] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[0]), _mm_madd_epi16(xmm[2][4], cmm[0])));
  rmm[5] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[0]), _mm_madd_epi16(xmm[2][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][5], cmm[0])));
  rmm[6] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[0]), _mm_madd_epi16(xmm[3][4], cmm[0])));
  rmm[7] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[0]), _mm_madd_epi16(xmm[3][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][5], cmm[0])));
  _mm_storeu_si128((__m128i*)(dst), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + 8), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[0][4], cmm[0])));
  rmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[0][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][5], cmm[0])));
  rmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[0]), _mm_madd_epi16(xmm[1][4], cmm[0])));
  rmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[0]), _mm_madd_epi16(xmm[1][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][5], cmm[0])));
  rmm[4] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[0]), _mm_madd_epi16(xmm[2][4], cmm[0])));
  rmm[5] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[0]), _mm_madd_epi16(xmm[2][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][5], cmm[0])));
  rmm[6] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][7], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[0]), _mm_madd_epi16(xmm[3][4], cmm[0])));
  rmm[7] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[0]), _mm_madd_epi16(xmm[3][6], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][5], cmm[0])));
  _mm_storeu_si128((__m128i*)(dst + (8 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (8 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[0][7], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[0][4], cmm[1])));
  rmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[2]), _mm_madd_epi16(xmm[0][6], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[2]), _mm_madd_epi16(xmm[0][5], cmm[2])));
  rmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[1]), _mm_madd_epi16(xmm[1][7], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[1]), _mm_madd_epi16(xmm[1][4], cmm[1])));
  rmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[2]), _mm_madd_epi16(xmm[1][6], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[2]), _mm_madd_epi16(xmm[1][5], cmm[2])));
  rmm[4] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[1]), _mm_madd_epi16(xmm[2][7], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[1]), _mm_madd_epi16(xmm[2][4], cmm[1])));
  rmm[5] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[2]), _mm_madd_epi16(xmm[2][6], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[2]), _mm_madd_epi16(xmm[2][5], cmm[2])));
  rmm[6] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[1]), _mm_madd_epi16(xmm[3][7], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[1]), _mm_madd_epi16(xmm[3][4], cmm[1])));
  rmm[7] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[2]), _mm_madd_epi16(xmm[3][6], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[2]), _mm_madd_epi16(xmm[3][5], cmm[2])));
  _mm_storeu_si128((__m128i*)(dst + (4 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (4 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[2]), _mm_madd_epi16(xmm[0][7], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[2]), _mm_madd_epi16(xmm[0][4], cmm[2])));
  rmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[0][6], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[0][5], cmm[1])));
  rmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[2]), _mm_madd_epi16(xmm[1][7], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[2]), _mm_madd_epi16(xmm[1][4], cmm[2])));
  rmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[1]), _mm_madd_epi16(xmm[1][6], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[1]), _mm_madd_epi16(xmm[1][5], cmm[1])));
  rmm[4] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[2]), _mm_madd_epi16(xmm[2][7], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[2]), _mm_madd_epi16(xmm[2][4], cmm[2])));
  rmm[5] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[1]), _mm_madd_epi16(xmm[2][6], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[1]), _mm_madd_epi16(xmm[2][5], cmm[1])));
  rmm[6] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[2]), _mm_madd_epi16(xmm[3][7], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[2]), _mm_madd_epi16(xmm[3][4], cmm[2])));
  rmm[7] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[1]), _mm_madd_epi16(xmm[3][6], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[1]), _mm_madd_epi16(xmm[3][5], cmm[1])));
  _mm_storeu_si128((__m128i*)(dst + (12 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (12 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[6], rmm[7]), ADD), shift)));

  // [JDS]: Set the constands and compute dst(2), dst(6), dst(10), and dst(14)
  cmm[0] = _mm_set1_epi16(89); cmm[1] = _mm_set1_epi16(75);     cmm[2] = _mm_set1_epi16(50);        cmm[3] = _mm_set1_epi16(18);
  rmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][7], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[0][6], cmm[1])));
  rmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[2]), _mm_madd_epi16(xmm[0][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[3]), _mm_madd_epi16(xmm[0][4], cmm[3])));
  rmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][7], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[1]), _mm_madd_epi16(xmm[1][6], cmm[1])));
  rmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[2]), _mm_madd_epi16(xmm[1][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[3]), _mm_madd_epi16(xmm[1][4], cmm[3])));
  rmm[4] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][7], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[1]), _mm_madd_epi16(xmm[2][6], cmm[1])));
  rmm[5] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[2]), _mm_madd_epi16(xmm[2][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[3]), _mm_madd_epi16(xmm[2][4], cmm[3])));
  rmm[6] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][7], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[1]), _mm_madd_epi16(xmm[3][6], cmm[1])));
  rmm[7] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[2]), _mm_madd_epi16(xmm[3][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[3]), _mm_madd_epi16(xmm[3][4], cmm[3])));
  _mm_storeu_si128((__m128i*)(dst + (2 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (2 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[0][7], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[3]), _mm_madd_epi16(xmm[0][6], cmm[3])));
  rmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[2]), _mm_madd_epi16(xmm[0][4], cmm[2])));
  rmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[1]), _mm_madd_epi16(xmm[1][7], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[3]), _mm_madd_epi16(xmm[1][6], cmm[3])));
  rmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[2]), _mm_madd_epi16(xmm[1][4], cmm[2])));
  rmm[4] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[1]), _mm_madd_epi16(xmm[2][7], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[3]), _mm_madd_epi16(xmm[2][6], cmm[3])));
  rmm[5] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[2]), _mm_madd_epi16(xmm[2][4], cmm[2])));
  rmm[6] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[1]), _mm_madd_epi16(xmm[3][7], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[3]), _mm_madd_epi16(xmm[3][6], cmm[3])));
  rmm[7] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[2]), _mm_madd_epi16(xmm[3][4], cmm[2])));
  _mm_storeu_si128((__m128i*)(dst + (6 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (6 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[2]), _mm_madd_epi16(xmm[0][7], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[0][6], cmm[0])));
  rmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[3]), _mm_madd_epi16(xmm[0][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[0][4], cmm[1])));
  rmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[2]), _mm_madd_epi16(xmm[1][7], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[0]), _mm_madd_epi16(xmm[1][6], cmm[0])));
  rmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[3]), _mm_madd_epi16(xmm[1][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[1]), _mm_madd_epi16(xmm[1][4], cmm[1])));
  rmm[4] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[2]), _mm_madd_epi16(xmm[2][7], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[0]), _mm_madd_epi16(xmm[2][6], cmm[0])));
  rmm[5] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[3]), _mm_madd_epi16(xmm[2][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[1]), _mm_madd_epi16(xmm[2][4], cmm[1])));
  rmm[6] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[2]), _mm_madd_epi16(xmm[3][7], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[0]), _mm_madd_epi16(xmm[3][6], cmm[0])));
  rmm[7] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[3]), _mm_madd_epi16(xmm[3][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[1]), _mm_madd_epi16(xmm[3][4], cmm[1])));
  _mm_storeu_si128((__m128i*)(dst + (10 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (10 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[3]), _mm_madd_epi16(xmm[0][7], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[2]), _mm_madd_epi16(xmm[0][6], cmm[2])));
  rmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[0][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[0][4], cmm[0])));
  rmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[3]), _mm_madd_epi16(xmm[1][7], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[2]), _mm_madd_epi16(xmm[1][6], cmm[2])));
  rmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[1]), _mm_madd_epi16(xmm[1][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[0]), _mm_madd_epi16(xmm[1][4], cmm[0])));
  rmm[4] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[3]), _mm_madd_epi16(xmm[2][7], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[2]), _mm_madd_epi16(xmm[2][6], cmm[2])));
  rmm[5] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[1]), _mm_madd_epi16(xmm[2][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[0]), _mm_madd_epi16(xmm[2][4], cmm[0])));
  rmm[6] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[3]), _mm_madd_epi16(xmm[3][7], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[2]), _mm_madd_epi16(xmm[3][6], cmm[2])));
  rmm[7] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[1]), _mm_madd_epi16(xmm[3][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[0]), _mm_madd_epi16(xmm[3][4], cmm[0])));
  _mm_storeu_si128((__m128i*)(dst + (14 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (14 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  // [JDS]: Set the constands and compute dst(1), dst(3), ..... , and dst(15)
  cmm[0] = _mm_unpacklo_epi16(_mm_set1_epi16(90), _mm_set1_epi16(-90));  cmm[1] = _mm_unpacklo_epi16(_mm_set1_epi16(87), _mm_set1_epi16(-87));
  cmm[2] = _mm_unpacklo_epi16(_mm_set1_epi16(80), _mm_set1_epi16(-80));  cmm[3] = _mm_unpacklo_epi16(_mm_set1_epi16(70), _mm_set1_epi16(-70));
  cmm[4] = _mm_unpacklo_epi16(_mm_set1_epi16(57), _mm_set1_epi16(-57));  cmm[5] = _mm_unpacklo_epi16(_mm_set1_epi16(43), _mm_set1_epi16(-43));
  cmm[6] = _mm_unpacklo_epi16(_mm_set1_epi16(25), _mm_set1_epi16(-25));  cmm[7] = _mm_unpacklo_epi16(_mm_set1_epi16(9), _mm_set1_epi16(-9));

  rmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][1], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[2]), _mm_madd_epi16(xmm[0][3], cmm[3])));
  rmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[4]), _mm_madd_epi16(xmm[0][5], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[6]), _mm_madd_epi16(xmm[0][7], cmm[7])));
  rmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][1], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[2]), _mm_madd_epi16(xmm[1][3], cmm[3])));
  rmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[4]), _mm_madd_epi16(xmm[1][5], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[6]), _mm_madd_epi16(xmm[1][7], cmm[7])));
  rmm[4] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][1], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[2]), _mm_madd_epi16(xmm[2][3], cmm[3])));
  rmm[5] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[4]), _mm_madd_epi16(xmm[2][5], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[6]), _mm_madd_epi16(xmm[2][7], cmm[7])));
  rmm[6] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][1], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[2]), _mm_madd_epi16(xmm[3][3], cmm[3])));
  rmm[7] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[4]), _mm_madd_epi16(xmm[3][5], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[6]), _mm_madd_epi16(xmm[3][7], cmm[7])));
  _mm_storeu_si128((__m128i*)(dst + (line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[0][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[7]), _mm_madd_epi16(xmm[0][3], cmm[5])));
  rmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[2]), _mm_madd_epi16(xmm[0][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[3]), _mm_madd_epi16(xmm[0][7], cmm[6])));
  rmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[1]), _mm_madd_epi16(xmm[1][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[7]), _mm_madd_epi16(xmm[1][3], cmm[5])));
  rmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[2]), _mm_madd_epi16(xmm[1][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[3]), _mm_madd_epi16(xmm[1][7], cmm[6])));
  rmm[4] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[1]), _mm_madd_epi16(xmm[2][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[7]), _mm_madd_epi16(xmm[2][3], cmm[5])));
  rmm[5] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[2]), _mm_madd_epi16(xmm[2][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[3]), _mm_madd_epi16(xmm[2][7], cmm[6])));
  rmm[6] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[1]), _mm_madd_epi16(xmm[3][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[7]), _mm_madd_epi16(xmm[3][3], cmm[5])));
  rmm[7] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[2]), _mm_madd_epi16(xmm[3][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[3]), _mm_madd_epi16(xmm[3][7], cmm[6])));
  _mm_storeu_si128((__m128i*)(dst + (3 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (3 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[2]), _mm_madd_epi16(xmm[0][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[3]), _mm_madd_epi16(xmm[0][3], cmm[1])));
  rmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[6]), _mm_madd_epi16(xmm[0][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[0][7], cmm[5])));
  rmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[2]), _mm_madd_epi16(xmm[1][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[3]), _mm_madd_epi16(xmm[1][3], cmm[1])));
  rmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[6]), _mm_madd_epi16(xmm[1][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[0]), _mm_madd_epi16(xmm[1][7], cmm[5])));
  rmm[4] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[2]), _mm_madd_epi16(xmm[2][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[3]), _mm_madd_epi16(xmm[2][3], cmm[1])));
  rmm[5] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[6]), _mm_madd_epi16(xmm[2][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[0]), _mm_madd_epi16(xmm[2][7], cmm[5])));
  rmm[6] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[2]), _mm_madd_epi16(xmm[3][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[3]), _mm_madd_epi16(xmm[3][3], cmm[1])));
  rmm[7] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[6]), _mm_madd_epi16(xmm[3][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[0]), _mm_madd_epi16(xmm[3][7], cmm[5])));
  _mm_storeu_si128((__m128i*)(dst + (5 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (5 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[3]), _mm_madd_epi16(xmm[0][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[0][3], cmm[7])));
  rmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[0][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[2]), _mm_madd_epi16(xmm[0][7], cmm[4])));
  rmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[3]), _mm_madd_epi16(xmm[1][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[1]), _mm_madd_epi16(xmm[1][3], cmm[7])));
  rmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[0]), _mm_madd_epi16(xmm[1][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[2]), _mm_madd_epi16(xmm[1][7], cmm[4])));
  rmm[4] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[3]), _mm_madd_epi16(xmm[2][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[1]), _mm_madd_epi16(xmm[2][3], cmm[7])));
  rmm[5] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[0]), _mm_madd_epi16(xmm[2][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[2]), _mm_madd_epi16(xmm[2][7], cmm[4])));
  rmm[6] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[3]), _mm_madd_epi16(xmm[3][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[1]), _mm_madd_epi16(xmm[3][3], cmm[7])));
  rmm[7] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[0]), _mm_madd_epi16(xmm[3][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[2]), _mm_madd_epi16(xmm[3][7], cmm[4])));
  _mm_storeu_si128((__m128i*)(dst + (7 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (7 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[4]), _mm_madd_epi16(xmm[0][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[6]), _mm_madd_epi16(xmm[0][3], cmm[0])));
  rmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[7]), _mm_madd_epi16(xmm[0][5], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[5]), _mm_madd_epi16(xmm[0][7], cmm[3])));
  rmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[4]), _mm_madd_epi16(xmm[1][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[6]), _mm_madd_epi16(xmm[1][3], cmm[0])));
  rmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[7]), _mm_madd_epi16(xmm[1][5], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[5]), _mm_madd_epi16(xmm[1][7], cmm[3])));
  rmm[4] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[4]), _mm_madd_epi16(xmm[2][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[6]), _mm_madd_epi16(xmm[2][3], cmm[0])));
  rmm[5] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[7]), _mm_madd_epi16(xmm[2][5], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[5]), _mm_madd_epi16(xmm[2][7], cmm[3])));
  rmm[6] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[4]), _mm_madd_epi16(xmm[3][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[6]), _mm_madd_epi16(xmm[3][3], cmm[0])));
  rmm[7] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[7]), _mm_madd_epi16(xmm[3][5], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[5]), _mm_madd_epi16(xmm[3][7], cmm[3])));
  _mm_storeu_si128((__m128i*)(dst + (9 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (9 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[5]), _mm_madd_epi16(xmm[0][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[4]), _mm_madd_epi16(xmm[0][3], cmm[6])));
  rmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[0][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[7]), _mm_madd_epi16(xmm[0][7], cmm[2])));
  rmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[5]), _mm_madd_epi16(xmm[1][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[4]), _mm_madd_epi16(xmm[1][3], cmm[6])));
  rmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[1]), _mm_madd_epi16(xmm[1][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[7]), _mm_madd_epi16(xmm[1][7], cmm[2])));
  rmm[4] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[5]), _mm_madd_epi16(xmm[2][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[4]), _mm_madd_epi16(xmm[2][3], cmm[6])));
  rmm[5] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[1]), _mm_madd_epi16(xmm[2][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[7]), _mm_madd_epi16(xmm[2][7], cmm[2])));
  rmm[6] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[5]), _mm_madd_epi16(xmm[3][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[4]), _mm_madd_epi16(xmm[3][3], cmm[6])));
  rmm[7] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[1]), _mm_madd_epi16(xmm[3][5], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[7]), _mm_madd_epi16(xmm[3][7], cmm[2])));
  _mm_storeu_si128((__m128i*)(dst + (11 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (11 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[6]), _mm_madd_epi16(xmm[0][1], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][3], cmm[2])));
  rmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[5]), _mm_madd_epi16(xmm[0][5], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[4]), _mm_madd_epi16(xmm[0][7], cmm[1])));
  rmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[6]), _mm_madd_epi16(xmm[1][1], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][3], cmm[2])));
  rmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[5]), _mm_madd_epi16(xmm[1][5], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[4]), _mm_madd_epi16(xmm[1][7], cmm[1])));
  rmm[4] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[6]), _mm_madd_epi16(xmm[2][1], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][3], cmm[2])));
  rmm[5] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[5]), _mm_madd_epi16(xmm[2][5], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[4]), _mm_madd_epi16(xmm[2][7], cmm[1])));
  rmm[6] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[6]), _mm_madd_epi16(xmm[3][1], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][3], cmm[2])));
  rmm[7] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[5]), _mm_madd_epi16(xmm[3][5], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[4]), _mm_madd_epi16(xmm[3][7], cmm[1])));
  _mm_storeu_si128((__m128i*)(dst + (13 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (13 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));

  rmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[7]), _mm_madd_epi16(xmm[0][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[5]), _mm_madd_epi16(xmm[0][3], cmm[4])));
  rmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[3]), _mm_madd_epi16(xmm[0][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[0][7], cmm[0])));
  rmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[7]), _mm_madd_epi16(xmm[1][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[5]), _mm_madd_epi16(xmm[1][3], cmm[4])));
  rmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[3]), _mm_madd_epi16(xmm[1][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[1]), _mm_madd_epi16(xmm[1][7], cmm[0])));
  rmm[4] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[7]), _mm_madd_epi16(xmm[2][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[5]), _mm_madd_epi16(xmm[2][3], cmm[4])));
  rmm[5] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[3]), _mm_madd_epi16(xmm[2][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[1]), _mm_madd_epi16(xmm[2][7], cmm[0])));
  rmm[6] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[7]), _mm_madd_epi16(xmm[3][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[5]), _mm_madd_epi16(xmm[3][3], cmm[4])));
  rmm[7] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[3]), _mm_madd_epi16(xmm[3][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[1]), _mm_madd_epi16(xmm[3][7], cmm[0])));
  _mm_storeu_si128((__m128i*)(dst + (15 * line)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[0], rmm[1]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[2], rmm[3]), ADD), shift)));
  _mm_storeu_si128((__m128i*)(dst + (15 * line + 8)), _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[4], rmm[5]), ADD), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(rmm[6], rmm[7]), ADD), shift)));
#else

Int j, k;
Int E[8], O[8];
Int EE[4], EO[4];
Int EEE[2], EEO[2];

  for (j=0; j<line; j++) 
  {    
    /* E and O*/
    for (k=0;k<8;k++)
    {
      E[k] = src[k] + src[15-k];
      O[k] = src[k] - src[15-k];
    } 
    /* EE and EO */
    for (k=0;k<4;k++)
    {
      EE[k] = E[k] + E[7-k];
      EO[k] = E[k] - E[7-k];
    }
    /* EEE and EEO */
    EEE[0] = EE[0] + EE[3];    
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];

    dst[ 0      ] = (g_aiT16[ 0][0]*EEE[0] + g_aiT16[ 0][1]*EEE[1] + add)>>shift;        
    dst[ 8*line ] = (g_aiT16[ 8][0]*EEE[0] + g_aiT16[ 8][1]*EEE[1] + add)>>shift;    
    dst[ 4*line ] = (g_aiT16[ 4][0]*EEO[0] + g_aiT16[ 4][1]*EEO[1] + add)>>shift;        
    dst[ 12*line] = (g_aiT16[12][0]*EEO[0] + g_aiT16[12][1]*EEO[1] + add)>>shift;

    for (k=2;k<16;k+=4)
    {
      dst[ k*line ] = (g_aiT16[k][0]*EO[0] + g_aiT16[k][1]*EO[1] + g_aiT16[k][2]*EO[2] + g_aiT16[k][3]*EO[3] + add)>>shift;      
    }

    for (k=1;k<16;k+=2)
    {
      dst[ k*line ] = (g_aiT16[k][0]*O[0] + g_aiT16[k][1]*O[1] + g_aiT16[k][2]*O[2] + g_aiT16[k][3]*O[3] + 
        g_aiT16[k][4]*O[4] + g_aiT16[k][5]*O[5] + g_aiT16[k][6]*O[6] + g_aiT16[k][7]*O[7] + add)>>shift;
    }

    src += 16;
    dst ++; 

  }
#endif // #if ETRI_SIMD_TR
}


void partialButterflyInverse16(Short *src,Short *dst,Int shift, Int line)
{
  Int add = 1<<(shift-1);

#if ETRI_SIMD_TR
  // [JDS]: Variable for SIMD code (O[k]에 대해 먼저 연산 후, E[k]에 대해 연산 수행) 
  __m128i xmm[4][4], omm[8][4], emm[8][4], cmm[16]; // Total 96

  // [JDS]: Data load and alignment for O[k] 
  cmm[0] = _mm_loadu_si128((__m128i*)(src + 8 * 2));    cmm[1] = _mm_loadu_si128((__m128i*)(src + 8 * 3));    cmm[2] = _mm_loadu_si128((__m128i*)(src + 8 * 6));    cmm[3] = _mm_loadu_si128((__m128i*)(src + 8 * 7));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + 8 * 10));  cmm[5] = _mm_loadu_si128((__m128i*)(src + 8 * 11));  cmm[6] = _mm_loadu_si128((__m128i*)(src + 8 * 14));  cmm[7] = _mm_loadu_si128((__m128i*)(src + 8 * 15));
  cmm[8] = _mm_loadu_si128((__m128i*)(src + 8 * 18));  cmm[9] = _mm_loadu_si128((__m128i*)(src + 8 * 19));  cmm[10] = _mm_loadu_si128((__m128i*)(src + 8 * 22));  cmm[11] = _mm_loadu_si128((__m128i*)(src + 8 * 23));
  cmm[12] = _mm_loadu_si128((__m128i*)(src + 8 * 26));  cmm[13] = _mm_loadu_si128((__m128i*)(src + 8 * 27));  cmm[14] = _mm_loadu_si128((__m128i*)(src + 8 * 30));  cmm[15] = _mm_loadu_si128((__m128i*)(src + 8 * 31));
  xmm[0][0] = _mm_unpacklo_epi16(cmm[0], cmm[2]);  xmm[0][1] = _mm_unpackhi_epi16(cmm[0], cmm[2]);    xmm[0][2] = _mm_unpacklo_epi16(cmm[1], cmm[3]);    xmm[0][3] = _mm_unpackhi_epi16(cmm[1], cmm[3]);
  xmm[1][0] = _mm_unpacklo_epi16(cmm[4], cmm[6]);    xmm[1][1] = _mm_unpackhi_epi16(cmm[4], cmm[6]);    xmm[1][2] = _mm_unpacklo_epi16(cmm[5], cmm[7]);    xmm[1][3] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  xmm[2][0] = _mm_unpacklo_epi16(cmm[8], cmm[10]);  xmm[2][1] = _mm_unpackhi_epi16(cmm[8], cmm[10]);   xmm[2][2] = _mm_unpacklo_epi16(cmm[9], cmm[11]);   xmm[2][3] = _mm_unpackhi_epi16(cmm[9], cmm[11]);
  xmm[3][0] = _mm_unpacklo_epi16(cmm[12], cmm[14]);  xmm[3][1] = _mm_unpackhi_epi16(cmm[12], cmm[14]);   xmm[3][2] = _mm_unpacklo_epi16(cmm[13], cmm[15]);   xmm[3][3] = _mm_unpackhi_epi16(cmm[13], cmm[15]);

  // [JDS]: Set the constant for "O[k]"
  cmm[0] = _mm_set1_epi16(90);  cmm[1] = _mm_set1_epi16(87);  cmm[2] = _mm_set1_epi16(80);  cmm[3] = _mm_set1_epi16(70);
  cmm[4] = _mm_set1_epi16(57);  cmm[5] = _mm_set1_epi16(43);  cmm[6] = _mm_set1_epi16(25);  cmm[7] = _mm_set1_epi16(9);
  cmm[8] = _mm_unpacklo_epi16(cmm[0], cmm[1]);  cmm[9] = _mm_unpacklo_epi16(cmm[2], cmm[3]);  cmm[10] = _mm_unpacklo_epi16(cmm[4], cmm[5]);  cmm[11] = _mm_unpacklo_epi16(cmm[6], cmm[7]);
  omm[0][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[0][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[0][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[0][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  cmm[8] = _mm_unpacklo_epi16(cmm[1], cmm[4]);  cmm[9] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));
  cmm[10] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])), _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  cmm[11] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])), _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])));
  omm[1][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[1][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[1][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[1][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  cmm[8] = _mm_unpacklo_epi16(cmm[2], cmm[7]);  cmm[9] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])), _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  cmm[10] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])), cmm[4]);  cmm[11] = _mm_unpacklo_epi16(cmm[0], cmm[5]);
  omm[2][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[2][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[2][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[2][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  cmm[8] = _mm_unpacklo_epi16(cmm[3], _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));  cmm[9] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])), cmm[7]);
  cmm[10] = _mm_unpacklo_epi16(cmm[0], cmm[6]);  cmm[11] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])), _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  omm[3][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[3][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[3][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[3][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  cmm[8] = _mm_unpacklo_epi16(cmm[4], _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])));  cmm[9] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])), cmm[0]);
  cmm[10] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])), _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));  cmm[11] = _mm_unpacklo_epi16(cmm[5], cmm[3]);
  omm[4][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[4][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[4][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[4][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  cmm[8] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));  cmm[9] = _mm_unpacklo_epi16(cmm[4], cmm[6]);
  cmm[10] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])), cmm[3]);  cmm[11] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])));
  omm[5][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[5][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[5][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[5][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  cmm[8] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));  cmm[9] = _mm_unpacklo_epi16(cmm[0], _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])));
  cmm[10] = _mm_unpacklo_epi16(cmm[5], cmm[7]);  cmm[11] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])), cmm[1]);
  omm[6][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[6][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[6][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[6][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  cmm[8] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])));  cmm[9] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  cmm[10] = _mm_unpacklo_epi16(cmm[3], _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])));  cmm[11] = _mm_unpacklo_epi16(cmm[1], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  omm[7][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[1][0], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[3][0], cmm[11])));
  omm[7][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[11])));
  omm[7][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[1][2], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[3][2], cmm[11])));
  omm[7][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[11])));

  // [JDS]: Data load and alignment for E[k] 
  cmm[0] = _mm_loadu_si128((__m128i*)(src));      cmm[1] = _mm_loadu_si128((__m128i*)(src + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(src + 8 * 4));    cmm[3] = _mm_loadu_si128((__m128i*)(src + 8 * 5));
  cmm[4] = _mm_loadu_si128((__m128i*)(src + 8 * 8));    cmm[5] = _mm_loadu_si128((__m128i*)(src + 8 * 9));    cmm[6] = _mm_loadu_si128((__m128i*)(src + 8 * 12));  cmm[7] = _mm_loadu_si128((__m128i*)(src + 8 * 13));
  cmm[8] = _mm_loadu_si128((__m128i*)(src + 8 * 16));  cmm[9] = _mm_loadu_si128((__m128i*)(src + 8 * 17));  cmm[10] = _mm_loadu_si128((__m128i*)(src + 8 * 20));  cmm[11] = _mm_loadu_si128((__m128i*)(src + 8 * 21));
  cmm[12] = _mm_loadu_si128((__m128i*)(src + 8 * 24));  cmm[13] = _mm_loadu_si128((__m128i*)(src + 8 * 25));  cmm[14] = _mm_loadu_si128((__m128i*)(src + 8 * 28));  cmm[15] = _mm_loadu_si128((__m128i*)(src + 8 * 29));
  xmm[0][0] = _mm_unpacklo_epi16(cmm[0], cmm[8]);  xmm[0][1] = _mm_unpackhi_epi16(cmm[0], cmm[8]);    xmm[0][2] = _mm_unpacklo_epi16(cmm[1], cmm[9]);    xmm[0][3] = _mm_unpackhi_epi16(cmm[1], cmm[9]);
  xmm[1][0] = _mm_unpacklo_epi16(cmm[4], cmm[12]);   xmm[1][1] = _mm_unpackhi_epi16(cmm[4], cmm[12]);   xmm[1][2] = _mm_unpacklo_epi16(cmm[5], cmm[13]);   xmm[1][3] = _mm_unpackhi_epi16(cmm[5], cmm[13]);
  xmm[2][0] = _mm_unpacklo_epi16(cmm[2], cmm[6]);  xmm[2][1] = _mm_unpackhi_epi16(cmm[2], cmm[6]);    xmm[2][2] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[2][3] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[3][0] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[3][1] = _mm_unpackhi_epi16(cmm[10], cmm[14]);   xmm[3][2] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[3][3] = _mm_unpackhi_epi16(cmm[11], cmm[15]);

  // [JDS]: Set the constant for "E[k]"
  cmm[0] = _mm_set1_epi16(64);  cmm[1] = _mm_set1_epi16(83);  cmm[2] = _mm_set1_epi16(36);  cmm[3] = _mm_set1_epi16(89);  cmm[4] = _mm_set1_epi16(75);  cmm[5] = _mm_set1_epi16(50);  cmm[6] = _mm_set1_epi16(18);
  cmm[7] = _mm_unpacklo_epi16(cmm[1], cmm[2]);  cmm[8] = _mm_unpacklo_epi16(cmm[3], cmm[4]);  cmm[9] = _mm_unpacklo_epi16(cmm[5], cmm[6]);
  emm[0][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[1][0], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[8]), _mm_madd_epi16(xmm[3][0], cmm[9])));
  emm[0][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[1][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[8]), _mm_madd_epi16(xmm[3][1], cmm[9])));
  emm[0][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[1][2], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[8]), _mm_madd_epi16(xmm[3][2], cmm[9])));
  emm[0][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[1][3], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[8]), _mm_madd_epi16(xmm[3][3], cmm[9])));
  emm[7][0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[1][0], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[8]), _mm_madd_epi16(xmm[3][0], cmm[9])));
  emm[7][1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[1][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[8]), _mm_madd_epi16(xmm[3][1], cmm[9])));
  emm[7][2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[1][2], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[8]), _mm_madd_epi16(xmm[3][2], cmm[9])));
  emm[7][3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[1][3], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[8]), _mm_madd_epi16(xmm[3][3], cmm[9])));

  cmm[8] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));  cmm[9] = _mm_unpacklo_epi16(cmm[4], _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));
  emm[3][0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[1][0], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[8]), _mm_madd_epi16(xmm[3][0], cmm[9])));
  emm[3][1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[1][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[8]), _mm_madd_epi16(xmm[3][1], cmm[9])));
  emm[3][2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[1][2], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[8]), _mm_madd_epi16(xmm[3][2], cmm[9])));
  emm[3][3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[1][3], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[8]), _mm_madd_epi16(xmm[3][3], cmm[9])));
  emm[4][0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[1][0], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[8]), _mm_madd_epi16(xmm[3][0], cmm[9])));
  emm[4][1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[1][1], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[8]), _mm_madd_epi16(xmm[3][1], cmm[9])));
  emm[4][2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[1][2], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[8]), _mm_madd_epi16(xmm[3][2], cmm[9])));
  emm[4][3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[1][3], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[8]), _mm_madd_epi16(xmm[3][3], cmm[9])));

  cmm[7] = _mm_unpacklo_epi16(cmm[0], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));  cmm[8] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  cmm[9] = _mm_unpacklo_epi16(cmm[4], _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])));  cmm[10] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])), _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));
  emm[1][0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[7]), _mm_madd_epi16(xmm[1][0], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[9]), _mm_madd_epi16(xmm[3][0], cmm[10])));
  emm[1][1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[7]), _mm_madd_epi16(xmm[1][1], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[9]), _mm_madd_epi16(xmm[3][1], cmm[10])));
  emm[1][2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[7]), _mm_madd_epi16(xmm[1][2], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[9]), _mm_madd_epi16(xmm[3][2], cmm[10])));
  emm[1][3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[7]), _mm_madd_epi16(xmm[1][3], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[9]), _mm_madd_epi16(xmm[3][3], cmm[10])));
  emm[6][0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[7]), _mm_madd_epi16(xmm[1][0], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[9]), _mm_madd_epi16(xmm[3][0], cmm[10])));
  emm[6][1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[7]), _mm_madd_epi16(xmm[1][1], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[9]), _mm_madd_epi16(xmm[3][1], cmm[10])));
  emm[6][2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[7]), _mm_madd_epi16(xmm[1][2], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[9]), _mm_madd_epi16(xmm[3][2], cmm[10])));
  emm[6][3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[7]), _mm_madd_epi16(xmm[1][3], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[9]), _mm_madd_epi16(xmm[3][3], cmm[10])));

  cmm[9] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));  cmm[10] = _mm_unpacklo_epi16(cmm[6], cmm[4]);
  emm[2][0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[7]), _mm_madd_epi16(xmm[1][0], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[9]), _mm_madd_epi16(xmm[3][0], cmm[10])));
  emm[2][1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[7]), _mm_madd_epi16(xmm[1][1], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[9]), _mm_madd_epi16(xmm[3][1], cmm[10])));
  emm[2][2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[7]), _mm_madd_epi16(xmm[1][2], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[9]), _mm_madd_epi16(xmm[3][2], cmm[10])));
  emm[2][3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[7]), _mm_madd_epi16(xmm[1][3], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[9]), _mm_madd_epi16(xmm[3][3], cmm[10])));
  emm[5][0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[7]), _mm_madd_epi16(xmm[1][0], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[9]), _mm_madd_epi16(xmm[3][0], cmm[10])));
  emm[5][1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[7]), _mm_madd_epi16(xmm[1][1], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[9]), _mm_madd_epi16(xmm[3][1], cmm[10])));
  emm[5][2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[7]), _mm_madd_epi16(xmm[1][2], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[9]), _mm_madd_epi16(xmm[3][2], cmm[10])));
  emm[5][3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[7]), _mm_madd_epi16(xmm[1][3], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[9]), _mm_madd_epi16(xmm[3][3], cmm[10])));

  // [JDS]: Compute "dst" 
  xmm[0][0] = _mm_set1_epi32(add);
  cmm[0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][0], omm[0][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][1], omm[0][1]), xmm[0][0]), shift));
  cmm[1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][0], omm[1][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][1], omm[1][1]), xmm[0][0]), shift));
  cmm[2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][0], omm[2][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][1], omm[2][1]), xmm[0][0]), shift));
  cmm[3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][0], omm[3][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][1], omm[3][1]), xmm[0][0]), shift));
  cmm[4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][0], omm[4][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][1], omm[4][1]), xmm[0][0]), shift));
  cmm[5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][0], omm[5][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][1], omm[5][1]), xmm[0][0]), shift));
  cmm[6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][0], omm[6][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][1], omm[6][1]), xmm[0][0]), shift));
  cmm[7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][0], omm[7][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][1], omm[7][1]), xmm[0][0]), shift));
  cmm[8] = _mm_unpacklo_epi16(cmm[0], cmm[2]);  cmm[9] = _mm_unpacklo_epi16(cmm[1], cmm[3]);  cmm[10] = _mm_unpacklo_epi16(cmm[4], cmm[6]);  cmm[11] = _mm_unpacklo_epi16(cmm[5], cmm[7]);
  cmm[12] = _mm_unpackhi_epi16(cmm[0], cmm[2]);  cmm[13] = _mm_unpackhi_epi16(cmm[1], cmm[3]);  cmm[14] = _mm_unpackhi_epi16(cmm[4], cmm[6]);  cmm[15] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi16(cmm[8], cmm[9]);  cmm[1] = _mm_unpacklo_epi16(cmm[10], cmm[11]);  cmm[2] = _mm_unpacklo_epi16(cmm[12], cmm[13]);  cmm[3] = _mm_unpacklo_epi16(cmm[14], cmm[15]);
  cmm[4] = _mm_unpackhi_epi16(cmm[8], cmm[9]);  cmm[5] = _mm_unpackhi_epi16(cmm[10], cmm[11]);  cmm[6] = _mm_unpackhi_epi16(cmm[12], cmm[13]);  cmm[7] = _mm_unpackhi_epi16(cmm[14], cmm[15]);
  _mm_storeu_si128((__m128i*)(dst), _mm_unpacklo_epi64(cmm[0], cmm[1]));  _mm_storeu_si128((__m128i*)(dst + line), _mm_unpackhi_epi64(cmm[0], cmm[1]));
  _mm_storeu_si128((__m128i*)(dst + 2 * line), _mm_unpacklo_epi64(cmm[4], cmm[5]));  _mm_storeu_si128((__m128i*)(dst + 3 * line), _mm_unpackhi_epi64(cmm[4], cmm[5]));
  _mm_storeu_si128((__m128i*)(dst + 4 * line), _mm_unpacklo_epi64(cmm[2], cmm[3]));  _mm_storeu_si128((__m128i*)(dst + 5 * line), _mm_unpackhi_epi64(cmm[2], cmm[3]));
  _mm_storeu_si128((__m128i*)(dst + 6 * line), _mm_unpacklo_epi64(cmm[6], cmm[7]));  _mm_storeu_si128((__m128i*)(dst + 7 * line), _mm_unpackhi_epi64(cmm[6], cmm[7]));

  cmm[7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][0], omm[0][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][1], omm[0][1]), xmm[0][0]), shift));
  cmm[6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][0], omm[1][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][1], omm[1][1]), xmm[0][0]), shift));
  cmm[5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][0], omm[2][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][1], omm[2][1]), xmm[0][0]), shift));
  cmm[4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][0], omm[3][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][1], omm[3][1]), xmm[0][0]), shift));
  cmm[3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][0], omm[4][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][1], omm[4][1]), xmm[0][0]), shift));
  cmm[2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][0], omm[5][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][1], omm[5][1]), xmm[0][0]), shift));
  cmm[1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][0], omm[6][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][1], omm[6][1]), xmm[0][0]), shift));
  cmm[0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][0], omm[7][0]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][1], omm[7][1]), xmm[0][0]), shift));
  cmm[8] = _mm_unpacklo_epi16(cmm[0], cmm[2]);  cmm[9] = _mm_unpacklo_epi16(cmm[1], cmm[3]);  cmm[10] = _mm_unpacklo_epi16(cmm[4], cmm[6]);  cmm[11] = _mm_unpacklo_epi16(cmm[5], cmm[7]);
  cmm[12] = _mm_unpackhi_epi16(cmm[0], cmm[2]);  cmm[13] = _mm_unpackhi_epi16(cmm[1], cmm[3]);  cmm[14] = _mm_unpackhi_epi16(cmm[4], cmm[6]);  cmm[15] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi16(cmm[8], cmm[9]);  cmm[1] = _mm_unpacklo_epi16(cmm[10], cmm[11]);  cmm[2] = _mm_unpacklo_epi16(cmm[12], cmm[13]);  cmm[3] = _mm_unpacklo_epi16(cmm[14], cmm[15]);
  cmm[4] = _mm_unpackhi_epi16(cmm[8], cmm[9]);  cmm[5] = _mm_unpackhi_epi16(cmm[10], cmm[11]);  cmm[6] = _mm_unpackhi_epi16(cmm[12], cmm[13]);  cmm[7] = _mm_unpackhi_epi16(cmm[14], cmm[15]);
  _mm_storeu_si128((__m128i*)(dst + 8), _mm_unpacklo_epi64(cmm[0], cmm[1]));   _mm_storeu_si128((__m128i*)(dst + line + 8), _mm_unpackhi_epi64(cmm[0], cmm[1]));
  _mm_storeu_si128((__m128i*)(dst + 2 * line + 8), _mm_unpacklo_epi64(cmm[4], cmm[5]));   _mm_storeu_si128((__m128i*)(dst + 3 * line + 8), _mm_unpackhi_epi64(cmm[4], cmm[5]));
  _mm_storeu_si128((__m128i*)(dst + 4 * line + 8), _mm_unpacklo_epi64(cmm[2], cmm[3]));  _mm_storeu_si128((__m128i*)(dst + 5 * line + 8), _mm_unpackhi_epi64(cmm[2], cmm[3]));
  _mm_storeu_si128((__m128i*)(dst + 6 * line + 8), _mm_unpacklo_epi64(cmm[6], cmm[7]));   _mm_storeu_si128((__m128i*)(dst + 7 * line + 8), _mm_unpackhi_epi64(cmm[6], cmm[7]));

  cmm[0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][2], omm[0][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][3], omm[0][3]), xmm[0][0]), shift));
  cmm[1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][2], omm[1][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][3], omm[1][3]), xmm[0][0]), shift));
  cmm[2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][2], omm[2][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][3], omm[2][3]), xmm[0][0]), shift));
  cmm[3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][2], omm[3][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][3], omm[3][3]), xmm[0][0]), shift));
  cmm[4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][2], omm[4][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][3], omm[4][3]), xmm[0][0]), shift));
  cmm[5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][2], omm[5][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][3], omm[5][3]), xmm[0][0]), shift));
  cmm[6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][2], omm[6][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][3], omm[6][3]), xmm[0][0]), shift));
  cmm[7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][2], omm[7][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][3], omm[7][3]), xmm[0][0]), shift));
  cmm[8] = _mm_unpacklo_epi16(cmm[0], cmm[2]);  cmm[9] = _mm_unpacklo_epi16(cmm[1], cmm[3]);  cmm[10] = _mm_unpacklo_epi16(cmm[4], cmm[6]);  cmm[11] = _mm_unpacklo_epi16(cmm[5], cmm[7]);
  cmm[12] = _mm_unpackhi_epi16(cmm[0], cmm[2]);  cmm[13] = _mm_unpackhi_epi16(cmm[1], cmm[3]);  cmm[14] = _mm_unpackhi_epi16(cmm[4], cmm[6]);  cmm[15] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi16(cmm[8], cmm[9]);  cmm[1] = _mm_unpacklo_epi16(cmm[10], cmm[11]);   cmm[2] = _mm_unpacklo_epi16(cmm[12], cmm[13]);  cmm[3] = _mm_unpacklo_epi16(cmm[14], cmm[15]);
  cmm[4] = _mm_unpackhi_epi16(cmm[8], cmm[9]);  cmm[5] = _mm_unpackhi_epi16(cmm[10], cmm[11]);   cmm[6] = _mm_unpackhi_epi16(cmm[12], cmm[13]);  cmm[7] = _mm_unpackhi_epi16(cmm[14], cmm[15]);
  _mm_storeu_si128((__m128i*)(dst + 8 * line), _mm_unpacklo_epi64(cmm[0], cmm[1]));  _mm_storeu_si128((__m128i*)(dst + 9 * line), _mm_unpackhi_epi64(cmm[0], cmm[1]));
  _mm_storeu_si128((__m128i*)(dst + 10 * line), _mm_unpacklo_epi64(cmm[4], cmm[5]));  _mm_storeu_si128((__m128i*)(dst + 11 * line), _mm_unpackhi_epi64(cmm[4], cmm[5]));
  _mm_storeu_si128((__m128i*)(dst + 12 * line), _mm_unpacklo_epi64(cmm[2], cmm[3]));  _mm_storeu_si128((__m128i*)(dst + 13 * line), _mm_unpackhi_epi64(cmm[2], cmm[3]));
  _mm_storeu_si128((__m128i*)(dst + 14 * line), _mm_unpacklo_epi64(cmm[6], cmm[7]));  _mm_storeu_si128((__m128i*)(dst + 15 * line), _mm_unpackhi_epi64(cmm[6], cmm[7]));

  cmm[7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][2], omm[0][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][3], omm[0][3]), xmm[0][0]), shift));
  cmm[6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][2], omm[1][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][3], omm[1][3]), xmm[0][0]), shift));
  cmm[5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][2], omm[2][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][3], omm[2][3]), xmm[0][0]), shift));
  cmm[4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][2], omm[3][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][3], omm[3][3]), xmm[0][0]), shift));
  cmm[3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][2], omm[4][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][3], omm[4][3]), xmm[0][0]), shift));
  cmm[2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][2], omm[5][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][3], omm[5][3]), xmm[0][0]), shift));
  cmm[1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][2], omm[6][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][3], omm[6][3]), xmm[0][0]), shift));
  cmm[0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][2], omm[7][2]), xmm[0][0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][3], omm[7][3]), xmm[0][0]), shift));
  cmm[8] = _mm_unpacklo_epi16(cmm[0], cmm[2]);  cmm[9] = _mm_unpacklo_epi16(cmm[1], cmm[3]);  cmm[10] = _mm_unpacklo_epi16(cmm[4], cmm[6]);  cmm[11] = _mm_unpacklo_epi16(cmm[5], cmm[7]);
  cmm[12] = _mm_unpackhi_epi16(cmm[0], cmm[2]);  cmm[13] = _mm_unpackhi_epi16(cmm[1], cmm[3]);  cmm[14] = _mm_unpackhi_epi16(cmm[4], cmm[6]);  cmm[15] = _mm_unpackhi_epi16(cmm[5], cmm[7]);
  cmm[0] = _mm_unpacklo_epi16(cmm[8], cmm[9]);  cmm[1] = _mm_unpacklo_epi16(cmm[10], cmm[11]);  cmm[2] = _mm_unpacklo_epi16(cmm[12], cmm[13]);  cmm[3] = _mm_unpacklo_epi16(cmm[14], cmm[15]);
  cmm[4] = _mm_unpackhi_epi16(cmm[8], cmm[9]);  cmm[5] = _mm_unpackhi_epi16(cmm[10], cmm[11]);  cmm[6] = _mm_unpackhi_epi16(cmm[12], cmm[13]);  cmm[7] = _mm_unpackhi_epi16(cmm[14], cmm[15]);
  _mm_storeu_si128((__m128i*)(dst + 8 * line + 8), _mm_unpacklo_epi64(cmm[0], cmm[1]));  _mm_storeu_si128((__m128i*)(dst + 9 * line + 8), _mm_unpackhi_epi64(cmm[0], cmm[1]));
  _mm_storeu_si128((__m128i*)(dst + 10 * line + 8), _mm_unpacklo_epi64(cmm[4], cmm[5]));  _mm_storeu_si128((__m128i*)(dst + 11 * line + 8), _mm_unpackhi_epi64(cmm[4], cmm[5]));
  _mm_storeu_si128((__m128i*)(dst + 12 * line + 8), _mm_unpacklo_epi64(cmm[2], cmm[3]));  _mm_storeu_si128((__m128i*)(dst + 13 * line + 8), _mm_unpackhi_epi64(cmm[2], cmm[3]));
  _mm_storeu_si128((__m128i*)(dst + 14 * line + 8), _mm_unpacklo_epi64(cmm[6], cmm[7]));  _mm_storeu_si128((__m128i*)(dst + 15 * line + 8), _mm_unpackhi_epi64(cmm[6], cmm[7]));
#else

Int j, k;
Int E[8], O[8];
Int EE[4], EO[4];
Int EEE[2], EEO[2];

  for (j=0; j<line; j++)
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<8;k++)
    {
      O[k] = g_aiT16[ 1][k]*src[ line] + g_aiT16[ 3][k]*src[ 3*line] + g_aiT16[ 5][k]*src[ 5*line] + g_aiT16[ 7][k]*src[ 7*line] + 
        g_aiT16[ 9][k]*src[ 9*line] + g_aiT16[11][k]*src[11*line] + g_aiT16[13][k]*src[13*line] + g_aiT16[15][k]*src[15*line];
    }
    for (k=0;k<4;k++)
    {
      EO[k] = g_aiT16[ 2][k]*src[ 2*line] + g_aiT16[ 6][k]*src[ 6*line] + g_aiT16[10][k]*src[10*line] + g_aiT16[14][k]*src[14*line];
    }
    EEO[0] = g_aiT16[4][0]*src[ 4*line ] + g_aiT16[12][0]*src[ 12*line ];
    EEE[0] = g_aiT16[0][0]*src[ 0      ] + g_aiT16[ 8][0]*src[ 8*line  ];
    EEO[1] = g_aiT16[4][1]*src[ 4*line ] + g_aiT16[12][1]*src[ 12*line ];
    EEE[1] = g_aiT16[0][1]*src[ 0      ] + g_aiT16[ 8][1]*src[ 8*line  ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */ 
    for (k=0;k<2;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+2] = EEE[1-k] - EEO[1-k];
    }    
    for (k=0;k<4;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+4] = EE[3-k] - EO[3-k];
    }    
    for (k=0;k<8;k++)
    {
      dst[k]   = Clip3( -32768, 32767, (E[k] + O[k] + add)>>shift );
      dst[k+8] = Clip3( -32768, 32767, (E[7-k] - O[7-k] + add)>>shift );
    }   
    src ++; 
    dst += 16;
  }
#endif // #if ETRI_SIMD_TR
}


void partialButterfly32(Short *src,Short *dst,Int shift, Int line)
{
  Int add = 1<<(shift-1);

#if ETRI_SIMD_TR
  // [JDS]: Variable and Constant for SIMD code  
  __m128i xmm[8][16], cmm[16], rmm[8], tmm[4], ADD;
  short* Tmp0 = src;  short *Tmp1 = Tmp0 + 128;  short *Tmp2 = Tmp0 + 256;  short *Tmp3 = Tmp0 + 384;  short *Tmp4 = Tmp0 + 512;  short *Tmp5 = Tmp0 + 640;  short *Tmp6 = Tmp0 + 768;  short *Tmp7 = Tmp0 + 896;

  // [JDS]: Data alignment to apply madd --> Merit compared to previous codes: Remove SIMD operations for E, EE, EEE,EEEE, O, EO, EEO, and EEEO
  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp0));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp0 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp0 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp0 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp0 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp0 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp0 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp0 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp0 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp0 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp0 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp0 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp0 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp0 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp0 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp0 + 120));
  xmm[0][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[0][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[0][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[0][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[0][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[0][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[0][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[0][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[0][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[0][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[0][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[0][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[0][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[0][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[0][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[0][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[0][0], xmm[0][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[0][0], xmm[0][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[0][1], xmm[0][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[0][1], xmm[0][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[0][2], xmm[0][10]); cmm[5] = _mm_unpackhi_epi32(xmm[0][2], xmm[0][10]); cmm[6] = _mm_unpacklo_epi32(xmm[0][3], xmm[0][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[0][3], xmm[0][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[0][4], xmm[0][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[0][4], xmm[0][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[0][5], xmm[0][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[0][5], xmm[0][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[0][6], xmm[0][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[0][6], xmm[0][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[0][7], xmm[0][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[0][7], xmm[0][15]), 78);
  xmm[0][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[0][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[0][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[0][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[0][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[0][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[0][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[0][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[0][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[0][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[0][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[0][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[0][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[0][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[0][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[0][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp1));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp1 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp1 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp1 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp1 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp1 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp1 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp1 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp1 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp1 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp1 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp1 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp1 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp1 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp1 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp1 + 120));
  xmm[1][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[1][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[1][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[1][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[1][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[1][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[1][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[1][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[1][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[1][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[1][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[1][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[1][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[1][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[1][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[1][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[1][0], xmm[1][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[1][0], xmm[1][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[1][1], xmm[1][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[1][1], xmm[1][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[1][2], xmm[1][10]); cmm[5] = _mm_unpackhi_epi32(xmm[1][2], xmm[1][10]); cmm[6] = _mm_unpacklo_epi32(xmm[1][3], xmm[1][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[1][3], xmm[1][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[1][4], xmm[1][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[1][4], xmm[1][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[1][5], xmm[1][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[1][5], xmm[1][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[1][6], xmm[1][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[1][6], xmm[1][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[1][7], xmm[1][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[1][7], xmm[1][15]), 78);
  xmm[1][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[1][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[1][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[1][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[1][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[1][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[1][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[1][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[1][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[1][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[1][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[1][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[1][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[1][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[1][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[1][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp2));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp2 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp2 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp2 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp2 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp2 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp2 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp2 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp2 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp2 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp2 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp2 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp2 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp2 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp2 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp2 + 120));
  xmm[2][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[2][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[2][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[2][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[2][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[2][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[2][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[2][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[2][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[2][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[2][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[2][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[2][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[2][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[2][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[2][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[2][0], xmm[2][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[2][0], xmm[2][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[2][1], xmm[2][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[2][1], xmm[2][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[2][2], xmm[2][10]); cmm[5] = _mm_unpackhi_epi32(xmm[2][2], xmm[2][10]); cmm[6] = _mm_unpacklo_epi32(xmm[2][3], xmm[2][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[2][3], xmm[2][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[2][4], xmm[2][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[2][4], xmm[2][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[2][5], xmm[2][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[2][5], xmm[2][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[2][6], xmm[2][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[2][6], xmm[2][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[2][7], xmm[2][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[2][7], xmm[2][15]), 78);
  xmm[2][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[2][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[2][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[2][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[2][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[2][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[2][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[2][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[2][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[2][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[2][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[2][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[2][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[2][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[2][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[2][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp3));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp3 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp3 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp3 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp3 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp3 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp3 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp3 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp3 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp3 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp3 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp3 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp3 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp3 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp3 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp3 + 120));
  xmm[3][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[3][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[3][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[3][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[3][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[3][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[3][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[3][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[3][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[3][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[3][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[3][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[3][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[3][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[3][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[3][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[3][0], xmm[3][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[3][0], xmm[3][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[3][1], xmm[3][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[3][1], xmm[3][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[3][2], xmm[3][10]); cmm[5] = _mm_unpackhi_epi32(xmm[3][2], xmm[3][10]); cmm[6] = _mm_unpacklo_epi32(xmm[3][3], xmm[3][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[3][3], xmm[3][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[3][4], xmm[3][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[3][4], xmm[3][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[3][5], xmm[3][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[3][5], xmm[3][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[3][6], xmm[3][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[3][6], xmm[3][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[3][7], xmm[3][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[3][7], xmm[3][15]), 78);
  xmm[3][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[3][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[3][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[3][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[3][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[3][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[3][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[3][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[3][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[3][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[3][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[3][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[3][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[3][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[3][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[3][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp4));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp4 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp4 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp4 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp4 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp4 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp4 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp4 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp4 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp4 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp4 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp4 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp4 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp4 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp4 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp4 + 120));
  xmm[4][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[4][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[4][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[4][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[4][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[4][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[4][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[4][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[4][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[4][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[4][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[4][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[4][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[4][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[4][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[4][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[4][0], xmm[4][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[4][0], xmm[4][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[4][1], xmm[4][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[4][1], xmm[4][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[4][2], xmm[4][10]); cmm[5] = _mm_unpackhi_epi32(xmm[4][2], xmm[4][10]); cmm[6] = _mm_unpacklo_epi32(xmm[4][3], xmm[4][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[4][3], xmm[4][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[4][4], xmm[4][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[4][4], xmm[4][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[4][5], xmm[4][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[4][5], xmm[4][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[4][6], xmm[4][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[4][6], xmm[4][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[4][7], xmm[4][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[4][7], xmm[4][15]), 78);
  xmm[4][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[4][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[4][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[4][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[4][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[4][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[4][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[4][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[4][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[4][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[4][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[4][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[4][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[4][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[4][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[4][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp5));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp5 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp5 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp5 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp5 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp5 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp5 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp5 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp5 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp5 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp5 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp5 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp5 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp5 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp5 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp5 + 120));
  xmm[5][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[5][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[5][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[5][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[5][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[5][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[5][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[5][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[5][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[5][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[5][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[5][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[5][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[5][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[5][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[5][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[5][0], xmm[5][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[5][0], xmm[5][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[5][1], xmm[5][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[5][1], xmm[5][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[5][2], xmm[5][10]); cmm[5] = _mm_unpackhi_epi32(xmm[5][2], xmm[5][10]); cmm[6] = _mm_unpacklo_epi32(xmm[5][3], xmm[5][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[5][3], xmm[5][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[5][4], xmm[5][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[5][4], xmm[5][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[5][5], xmm[5][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[5][5], xmm[5][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[5][6], xmm[5][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[5][6], xmm[5][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[5][7], xmm[5][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[5][7], xmm[5][15]), 78);
  xmm[5][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[5][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[5][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[5][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[5][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[5][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[5][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[5][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[5][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[5][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[5][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[5][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[5][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[5][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[5][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[5][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp6));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp6 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp6 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp6 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp6 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp6 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp6 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp6 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp6 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp6 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp6 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp6 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp6 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp6 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp6 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp6 + 120));
  xmm[6][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[6][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[6][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[6][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[6][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[6][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[6][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[6][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[6][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[6][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[6][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[6][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[6][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[6][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[6][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[6][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[6][0], xmm[6][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[6][0], xmm[6][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[6][1], xmm[6][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[6][1], xmm[6][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[6][2], xmm[6][10]); cmm[5] = _mm_unpackhi_epi32(xmm[6][2], xmm[6][10]); cmm[6] = _mm_unpacklo_epi32(xmm[6][3], xmm[6][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[6][3], xmm[6][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[6][4], xmm[6][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[6][4], xmm[6][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[6][5], xmm[6][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[6][5], xmm[6][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[6][6], xmm[6][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[6][6], xmm[6][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[6][7], xmm[6][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[6][7], xmm[6][15]), 78);
  xmm[6][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[6][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[6][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[6][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[6][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[6][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[6][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[6][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[6][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[6][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[6][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[6][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[6][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[6][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[6][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[6][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  cmm[0] = _mm_loadu_si128((__m128i*)(Tmp7));     cmm[1] = _mm_loadu_si128((__m128i*)(Tmp7 + 8));    cmm[2] = _mm_loadu_si128((__m128i*)(Tmp7 + 16));    cmm[3] = _mm_loadu_si128((__m128i*)(Tmp7 + 24));
  cmm[4] = _mm_loadu_si128((__m128i*)(Tmp7 + 32));    cmm[5] = _mm_loadu_si128((__m128i*)(Tmp7 + 40));   cmm[6] = _mm_loadu_si128((__m128i*)(Tmp7 + 48));    cmm[7] = _mm_loadu_si128((__m128i*)(Tmp7 + 56));
  cmm[8] = _mm_loadu_si128((__m128i*)(Tmp7 + 64));    cmm[9] = _mm_loadu_si128((__m128i*)(Tmp7 + 72));   cmm[10] = _mm_loadu_si128((__m128i*)(Tmp7 + 80));    cmm[11] = _mm_loadu_si128((__m128i*)(Tmp7 + 88));
  cmm[12] = _mm_loadu_si128((__m128i*)(Tmp7 + 96));    cmm[13] = _mm_loadu_si128((__m128i*)(Tmp7 + 104));  cmm[14] = _mm_loadu_si128((__m128i*)(Tmp7 + 112));   cmm[15] = _mm_loadu_si128((__m128i*)(Tmp7 + 120));
  xmm[7][0] = _mm_unpacklo_epi16(cmm[0], cmm[4]);   xmm[7][1] = _mm_unpackhi_epi16(cmm[0], cmm[4]);   xmm[7][2] = _mm_unpacklo_epi16(cmm[1], cmm[5]);    xmm[7][3] = _mm_unpackhi_epi16(cmm[1], cmm[5]);
  xmm[7][4] = _mm_unpacklo_epi16(cmm[2], cmm[6]);   xmm[7][5] = _mm_unpackhi_epi16(cmm[2], cmm[6]);   xmm[7][6] = _mm_unpacklo_epi16(cmm[3], cmm[7]);    xmm[7][7] = _mm_unpackhi_epi16(cmm[3], cmm[7]);
  xmm[7][8] = _mm_unpacklo_epi16(cmm[8], cmm[12]);  xmm[7][9] = _mm_unpackhi_epi16(cmm[8], cmm[12]);  xmm[7][10] = _mm_unpacklo_epi16(cmm[9], cmm[13]);   xmm[7][11] = _mm_unpackhi_epi16(cmm[9], cmm[13]);
  xmm[7][12] = _mm_unpacklo_epi16(cmm[10], cmm[14]);  xmm[7][13] = _mm_unpackhi_epi16(cmm[10], cmm[14]);  xmm[7][14] = _mm_unpacklo_epi16(cmm[11], cmm[15]);   xmm[7][15] = _mm_unpackhi_epi16(cmm[11], cmm[15]);
  cmm[0] = _mm_unpacklo_epi32(xmm[7][0], xmm[7][8]);  cmm[1] = _mm_unpackhi_epi32(xmm[7][0], xmm[7][8]);  cmm[2] = _mm_unpacklo_epi32(xmm[7][1], xmm[7][9]);   cmm[3] = _mm_unpackhi_epi32(xmm[7][1], xmm[7][9]);
  cmm[4] = _mm_unpacklo_epi32(xmm[7][2], xmm[7][10]); cmm[5] = _mm_unpackhi_epi32(xmm[7][2], xmm[7][10]); cmm[6] = _mm_unpacklo_epi32(xmm[7][3], xmm[7][11]);  cmm[7] = _mm_unpackhi_epi32(xmm[7][3], xmm[7][11]);
  cmm[8] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[7][4], xmm[7][12]), 78);  cmm[9] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[7][4], xmm[7][12]), 78);
  cmm[10] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[7][5], xmm[7][13]), 78);  cmm[11] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[7][5], xmm[7][13]), 78);
  cmm[12] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[7][6], xmm[7][14]), 78);  cmm[13] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[7][6], xmm[7][14]), 78);
  cmm[14] = _mm_shuffle_epi32(_mm_unpacklo_epi32(xmm[7][7], xmm[7][15]), 78);  cmm[15] = _mm_shuffle_epi32(_mm_unpackhi_epi32(xmm[7][7], xmm[7][15]), 78);
  xmm[7][0] = _mm_unpacklo_epi16(cmm[0], cmm[15]);  xmm[7][1] = _mm_unpackhi_epi16(cmm[0], cmm[15]);  xmm[7][2] = _mm_unpacklo_epi16(cmm[1], cmm[14]);   xmm[7][3] = _mm_unpackhi_epi16(cmm[1], cmm[14]);
  xmm[7][4] = _mm_unpacklo_epi16(cmm[2], cmm[13]);  xmm[7][5] = _mm_unpackhi_epi16(cmm[2], cmm[13]);  xmm[7][6] = _mm_unpacklo_epi16(cmm[3], cmm[12]);   xmm[7][7] = _mm_unpackhi_epi16(cmm[3], cmm[12]);
  xmm[7][8] = _mm_unpacklo_epi16(cmm[4], cmm[11]);  xmm[7][9] = _mm_unpackhi_epi16(cmm[4], cmm[11]);  xmm[7][10] = _mm_unpacklo_epi16(cmm[5], cmm[10]);   xmm[7][11] = _mm_unpackhi_epi16(cmm[5], cmm[10]);
  xmm[7][12] = _mm_unpacklo_epi16(cmm[6], cmm[9]);   xmm[7][13] = _mm_unpackhi_epi16(cmm[6], cmm[9]);   xmm[7][14] = _mm_unpacklo_epi16(cmm[7], cmm[8]);    xmm[7][15] = _mm_unpackhi_epi16(cmm[7], cmm[8]);

  // [JDS]: Set the constands and compute "dst" --> dst(0), dst(16), dst(8), and dst(24)
  ADD = _mm_set1_epi32(add);  cmm[0] = _mm_set1_epi16(64);  cmm[1] = _mm_set1_epi16(83);  cmm[2] = _mm_set1_epi16(36);

  // [JDS]:  dst[0]
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[0]), _mm_madd_epi16(xmm[0][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[0][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[0][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[0][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[0][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[0]), _mm_madd_epi16(xmm[0][10], cmm[0])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[0]), _mm_madd_epi16(xmm[1][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[0]), _mm_madd_epi16(xmm[1][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[0]), _mm_madd_epi16(xmm[1][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[0]), _mm_madd_epi16(xmm[1][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[0]), _mm_madd_epi16(xmm[1][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[0]), _mm_madd_epi16(xmm[1][10], cmm[0])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[0]), _mm_madd_epi16(xmm[2][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[0]), _mm_madd_epi16(xmm[2][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[0]), _mm_madd_epi16(xmm[2][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[0]), _mm_madd_epi16(xmm[2][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[0]), _mm_madd_epi16(xmm[2][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[0]), _mm_madd_epi16(xmm[2][10], cmm[0])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[0]), _mm_madd_epi16(xmm[3][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[0]), _mm_madd_epi16(xmm[3][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[0]), _mm_madd_epi16(xmm[3][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[0]), _mm_madd_epi16(xmm[3][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[0]), _mm_madd_epi16(xmm[3][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[0]), _mm_madd_epi16(xmm[3][10], cmm[0])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[0]), _mm_madd_epi16(xmm[4][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[0]), _mm_madd_epi16(xmm[4][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[0]), _mm_madd_epi16(xmm[4][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[0]), _mm_madd_epi16(xmm[4][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[0]), _mm_madd_epi16(xmm[4][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[0]), _mm_madd_epi16(xmm[4][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[0]), _mm_madd_epi16(xmm[4][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[0]), _mm_madd_epi16(xmm[4][10], cmm[0])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[0]), _mm_madd_epi16(xmm[5][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[0]), _mm_madd_epi16(xmm[5][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[0]), _mm_madd_epi16(xmm[5][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[0]), _mm_madd_epi16(xmm[5][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[0]), _mm_madd_epi16(xmm[5][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[0]), _mm_madd_epi16(xmm[5][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[0]), _mm_madd_epi16(xmm[5][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[0]), _mm_madd_epi16(xmm[5][10], cmm[0])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[0]), _mm_madd_epi16(xmm[6][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[0]), _mm_madd_epi16(xmm[6][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[0]), _mm_madd_epi16(xmm[6][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[0]), _mm_madd_epi16(xmm[6][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[0]), _mm_madd_epi16(xmm[6][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[0]), _mm_madd_epi16(xmm[6][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[0]), _mm_madd_epi16(xmm[6][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[0]), _mm_madd_epi16(xmm[6][10], cmm[0])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[0]), _mm_madd_epi16(xmm[7][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[0]), _mm_madd_epi16(xmm[7][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[0]), _mm_madd_epi16(xmm[7][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[0]), _mm_madd_epi16(xmm[7][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[0]), _mm_madd_epi16(xmm[7][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[0]), _mm_madd_epi16(xmm[7][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[0]), _mm_madd_epi16(xmm[7][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[0]), _mm_madd_epi16(xmm[7][10], cmm[0])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst), _mm_packs_epi32(rmm[0], rmm[1]));  _mm_storeu_si128((__m128i*)(dst + 8), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + 16), _mm_packs_epi32(rmm[4], rmm[5]));  _mm_storeu_si128((__m128i*)(dst + 24), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[16*line]
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[0]), _mm_madd_epi16(xmm[0][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[0][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[0][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[0][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[0][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[0]), _mm_madd_epi16(xmm[0][10], cmm[0])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[0]), _mm_madd_epi16(xmm[1][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[0]), _mm_madd_epi16(xmm[1][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[0]), _mm_madd_epi16(xmm[1][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[0]), _mm_madd_epi16(xmm[1][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[0]), _mm_madd_epi16(xmm[1][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[0]), _mm_madd_epi16(xmm[1][10], cmm[0])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[0]), _mm_madd_epi16(xmm[2][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[0]), _mm_madd_epi16(xmm[2][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[0]), _mm_madd_epi16(xmm[2][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[0]), _mm_madd_epi16(xmm[2][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[0]), _mm_madd_epi16(xmm[2][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[0]), _mm_madd_epi16(xmm[2][10], cmm[0])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[0]), _mm_madd_epi16(xmm[3][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[0]), _mm_madd_epi16(xmm[3][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[0]), _mm_madd_epi16(xmm[3][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[0]), _mm_madd_epi16(xmm[3][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[0]), _mm_madd_epi16(xmm[3][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[0]), _mm_madd_epi16(xmm[3][10], cmm[0])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[0]), _mm_madd_epi16(xmm[4][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[0]), _mm_madd_epi16(xmm[4][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[0]), _mm_madd_epi16(xmm[4][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[0]), _mm_madd_epi16(xmm[4][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[0]), _mm_madd_epi16(xmm[4][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[0]), _mm_madd_epi16(xmm[4][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[0]), _mm_madd_epi16(xmm[4][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[0]), _mm_madd_epi16(xmm[4][10], cmm[0])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[0]), _mm_madd_epi16(xmm[5][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[0]), _mm_madd_epi16(xmm[5][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[0]), _mm_madd_epi16(xmm[5][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[0]), _mm_madd_epi16(xmm[5][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[0]), _mm_madd_epi16(xmm[5][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[0]), _mm_madd_epi16(xmm[5][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[0]), _mm_madd_epi16(xmm[5][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[0]), _mm_madd_epi16(xmm[5][10], cmm[0])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[0]), _mm_madd_epi16(xmm[6][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[0]), _mm_madd_epi16(xmm[6][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[0]), _mm_madd_epi16(xmm[6][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[0]), _mm_madd_epi16(xmm[6][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[0]), _mm_madd_epi16(xmm[6][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[0]), _mm_madd_epi16(xmm[6][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[0]), _mm_madd_epi16(xmm[6][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[0]), _mm_madd_epi16(xmm[6][10], cmm[0])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[0]), _mm_madd_epi16(xmm[7][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[0]), _mm_madd_epi16(xmm[7][8], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[0]), _mm_madd_epi16(xmm[7][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[0]), _mm_madd_epi16(xmm[7][11], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[0]), _mm_madd_epi16(xmm[7][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[0]), _mm_madd_epi16(xmm[7][9], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[0]), _mm_madd_epi16(xmm[7][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[0]), _mm_madd_epi16(xmm[7][10], cmm[0])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (16 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (16 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (16 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (16 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[8*line]
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[0][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[0][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[0][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[0][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[2]), _mm_madd_epi16(xmm[0][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[2]), _mm_madd_epi16(xmm[0][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[2]), _mm_madd_epi16(xmm[0][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[2]), _mm_madd_epi16(xmm[0][10], cmm[2])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[1]), _mm_madd_epi16(xmm[1][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[1]), _mm_madd_epi16(xmm[1][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[1]), _mm_madd_epi16(xmm[1][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[1]), _mm_madd_epi16(xmm[1][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[2]), _mm_madd_epi16(xmm[1][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[2]), _mm_madd_epi16(xmm[1][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[2]), _mm_madd_epi16(xmm[1][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[2]), _mm_madd_epi16(xmm[1][10], cmm[2])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[1]), _mm_madd_epi16(xmm[2][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[1]), _mm_madd_epi16(xmm[2][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[1]), _mm_madd_epi16(xmm[2][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[1]), _mm_madd_epi16(xmm[2][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[2]), _mm_madd_epi16(xmm[2][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[2]), _mm_madd_epi16(xmm[2][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[2]), _mm_madd_epi16(xmm[2][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[2]), _mm_madd_epi16(xmm[2][10], cmm[2])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[1]), _mm_madd_epi16(xmm[3][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[1]), _mm_madd_epi16(xmm[3][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[1]), _mm_madd_epi16(xmm[3][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[1]), _mm_madd_epi16(xmm[3][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[2]), _mm_madd_epi16(xmm[3][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[2]), _mm_madd_epi16(xmm[3][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[2]), _mm_madd_epi16(xmm[3][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[2]), _mm_madd_epi16(xmm[3][10], cmm[2])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[1]), _mm_madd_epi16(xmm[4][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[1]), _mm_madd_epi16(xmm[4][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[1]), _mm_madd_epi16(xmm[4][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[1]), _mm_madd_epi16(xmm[4][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[2]), _mm_madd_epi16(xmm[4][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[2]), _mm_madd_epi16(xmm[4][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[2]), _mm_madd_epi16(xmm[4][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[2]), _mm_madd_epi16(xmm[4][10], cmm[2])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[1]), _mm_madd_epi16(xmm[5][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[1]), _mm_madd_epi16(xmm[5][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[1]), _mm_madd_epi16(xmm[5][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[1]), _mm_madd_epi16(xmm[5][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[2]), _mm_madd_epi16(xmm[5][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[2]), _mm_madd_epi16(xmm[5][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[2]), _mm_madd_epi16(xmm[5][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[2]), _mm_madd_epi16(xmm[5][10], cmm[2])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[1]), _mm_madd_epi16(xmm[6][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[1]), _mm_madd_epi16(xmm[6][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[1]), _mm_madd_epi16(xmm[6][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[1]), _mm_madd_epi16(xmm[6][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[2]), _mm_madd_epi16(xmm[6][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[2]), _mm_madd_epi16(xmm[6][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[2]), _mm_madd_epi16(xmm[6][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[2]), _mm_madd_epi16(xmm[6][10], cmm[2])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[1]), _mm_madd_epi16(xmm[7][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[1]), _mm_madd_epi16(xmm[7][8], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[1]), _mm_madd_epi16(xmm[7][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[1]), _mm_madd_epi16(xmm[7][11], cmm[1])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[2]), _mm_madd_epi16(xmm[7][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[2]), _mm_madd_epi16(xmm[7][9], cmm[2])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[2]), _mm_madd_epi16(xmm[7][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[2]), _mm_madd_epi16(xmm[7][10], cmm[2])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (8 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (8 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (8 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (8 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[24*line]
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[2]), _mm_madd_epi16(xmm[0][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[2]), _mm_madd_epi16(xmm[0][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[2]), _mm_madd_epi16(xmm[0][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[2]), _mm_madd_epi16(xmm[0][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[0][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[0][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[0][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[0][10], cmm[1])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[2]), _mm_madd_epi16(xmm[1][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[2]), _mm_madd_epi16(xmm[1][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[2]), _mm_madd_epi16(xmm[1][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[2]), _mm_madd_epi16(xmm[1][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[1]), _mm_madd_epi16(xmm[1][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[1]), _mm_madd_epi16(xmm[1][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[1]), _mm_madd_epi16(xmm[1][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[1]), _mm_madd_epi16(xmm[1][10], cmm[1])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[2]), _mm_madd_epi16(xmm[2][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[2]), _mm_madd_epi16(xmm[2][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[2]), _mm_madd_epi16(xmm[2][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[2]), _mm_madd_epi16(xmm[2][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[1]), _mm_madd_epi16(xmm[2][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[1]), _mm_madd_epi16(xmm[2][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[1]), _mm_madd_epi16(xmm[2][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[1]), _mm_madd_epi16(xmm[2][10], cmm[1])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[2]), _mm_madd_epi16(xmm[3][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[2]), _mm_madd_epi16(xmm[3][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[2]), _mm_madd_epi16(xmm[3][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[2]), _mm_madd_epi16(xmm[3][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[1]), _mm_madd_epi16(xmm[3][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[1]), _mm_madd_epi16(xmm[3][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[1]), _mm_madd_epi16(xmm[3][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[1]), _mm_madd_epi16(xmm[3][10], cmm[1])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[2]), _mm_madd_epi16(xmm[4][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[2]), _mm_madd_epi16(xmm[4][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[2]), _mm_madd_epi16(xmm[4][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[2]), _mm_madd_epi16(xmm[4][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[1]), _mm_madd_epi16(xmm[4][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[1]), _mm_madd_epi16(xmm[4][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[1]), _mm_madd_epi16(xmm[4][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[1]), _mm_madd_epi16(xmm[4][10], cmm[1])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[2]), _mm_madd_epi16(xmm[5][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[2]), _mm_madd_epi16(xmm[5][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[2]), _mm_madd_epi16(xmm[5][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[2]), _mm_madd_epi16(xmm[5][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[1]), _mm_madd_epi16(xmm[5][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[1]), _mm_madd_epi16(xmm[5][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[1]), _mm_madd_epi16(xmm[5][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[1]), _mm_madd_epi16(xmm[5][10], cmm[1])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[2]), _mm_madd_epi16(xmm[6][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[2]), _mm_madd_epi16(xmm[6][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[2]), _mm_madd_epi16(xmm[6][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[2]), _mm_madd_epi16(xmm[6][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[1]), _mm_madd_epi16(xmm[6][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[1]), _mm_madd_epi16(xmm[6][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[1]), _mm_madd_epi16(xmm[6][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[1]), _mm_madd_epi16(xmm[6][10], cmm[1])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[2]), _mm_madd_epi16(xmm[7][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[2]), _mm_madd_epi16(xmm[7][8], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[2]), _mm_madd_epi16(xmm[7][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[2]), _mm_madd_epi16(xmm[7][11], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[1]), _mm_madd_epi16(xmm[7][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[1]), _mm_madd_epi16(xmm[7][9], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[1]), _mm_madd_epi16(xmm[7][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[1]), _mm_madd_epi16(xmm[7][10], cmm[1])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (24 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (24 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (24 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (24 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]: Set the constands and compute "dst" --> dst(4), dst(12), dst(20), and dst(28)
  cmm[0] = _mm_set1_epi16(89);  cmm[1] = _mm_set1_epi16(75);  cmm[2] = _mm_set1_epi16(50);  cmm[3] = _mm_set1_epi16(18);

  // [JDS]:  dst[4*line]
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[0]), _mm_madd_epi16(xmm[0][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[0][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[0][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[2]), _mm_madd_epi16(xmm[0][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[2]), _mm_madd_epi16(xmm[0][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[3]), _mm_madd_epi16(xmm[0][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[3]), _mm_madd_epi16(xmm[0][11], cmm[3])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[0]), _mm_madd_epi16(xmm[1][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[1]), _mm_madd_epi16(xmm[1][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[1]), _mm_madd_epi16(xmm[1][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[2]), _mm_madd_epi16(xmm[1][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[2]), _mm_madd_epi16(xmm[1][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[3]), _mm_madd_epi16(xmm[1][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[3]), _mm_madd_epi16(xmm[1][11], cmm[3])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[0]), _mm_madd_epi16(xmm[2][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[1]), _mm_madd_epi16(xmm[2][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[1]), _mm_madd_epi16(xmm[2][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[2]), _mm_madd_epi16(xmm[2][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[2]), _mm_madd_epi16(xmm[2][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[3]), _mm_madd_epi16(xmm[2][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[3]), _mm_madd_epi16(xmm[2][11], cmm[3])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[0]), _mm_madd_epi16(xmm[3][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[1]), _mm_madd_epi16(xmm[3][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[1]), _mm_madd_epi16(xmm[3][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[2]), _mm_madd_epi16(xmm[3][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[2]), _mm_madd_epi16(xmm[3][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[3]), _mm_madd_epi16(xmm[3][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[3]), _mm_madd_epi16(xmm[3][11], cmm[3])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[0]), _mm_madd_epi16(xmm[4][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[0]), _mm_madd_epi16(xmm[4][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[1]), _mm_madd_epi16(xmm[4][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[1]), _mm_madd_epi16(xmm[4][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[2]), _mm_madd_epi16(xmm[4][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[2]), _mm_madd_epi16(xmm[4][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[3]), _mm_madd_epi16(xmm[4][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[3]), _mm_madd_epi16(xmm[4][11], cmm[3])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[0]), _mm_madd_epi16(xmm[5][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[0]), _mm_madd_epi16(xmm[5][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[1]), _mm_madd_epi16(xmm[5][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[1]), _mm_madd_epi16(xmm[5][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[2]), _mm_madd_epi16(xmm[5][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[2]), _mm_madd_epi16(xmm[5][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[3]), _mm_madd_epi16(xmm[5][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[3]), _mm_madd_epi16(xmm[5][11], cmm[3])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[0]), _mm_madd_epi16(xmm[6][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[0]), _mm_madd_epi16(xmm[6][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[1]), _mm_madd_epi16(xmm[6][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[1]), _mm_madd_epi16(xmm[6][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[2]), _mm_madd_epi16(xmm[6][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[2]), _mm_madd_epi16(xmm[6][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[3]), _mm_madd_epi16(xmm[6][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[3]), _mm_madd_epi16(xmm[6][11], cmm[3])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[0]), _mm_madd_epi16(xmm[7][15], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[0]), _mm_madd_epi16(xmm[7][8], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[1]), _mm_madd_epi16(xmm[7][14], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[1]), _mm_madd_epi16(xmm[7][9], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[2]), _mm_madd_epi16(xmm[7][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[2]), _mm_madd_epi16(xmm[7][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[3]), _mm_madd_epi16(xmm[7][12], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[3]), _mm_madd_epi16(xmm[7][11], cmm[3])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (4 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (4 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (4 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (4 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[12*line]
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[0][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[0][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[3]), _mm_madd_epi16(xmm[0][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[3]), _mm_madd_epi16(xmm[0][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[0]), _mm_madd_epi16(xmm[0][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[2]), _mm_madd_epi16(xmm[0][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[2]), _mm_madd_epi16(xmm[0][11], cmm[2])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[1]), _mm_madd_epi16(xmm[1][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[1]), _mm_madd_epi16(xmm[1][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[3]), _mm_madd_epi16(xmm[1][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[3]), _mm_madd_epi16(xmm[1][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[0]), _mm_madd_epi16(xmm[1][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[2]), _mm_madd_epi16(xmm[1][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[2]), _mm_madd_epi16(xmm[1][11], cmm[2])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[1]), _mm_madd_epi16(xmm[2][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[1]), _mm_madd_epi16(xmm[2][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[3]), _mm_madd_epi16(xmm[2][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[3]), _mm_madd_epi16(xmm[2][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[0]), _mm_madd_epi16(xmm[2][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[2]), _mm_madd_epi16(xmm[2][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[2]), _mm_madd_epi16(xmm[2][11], cmm[2])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[1]), _mm_madd_epi16(xmm[3][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[1]), _mm_madd_epi16(xmm[3][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[3]), _mm_madd_epi16(xmm[3][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[3]), _mm_madd_epi16(xmm[3][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[0]), _mm_madd_epi16(xmm[3][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[2]), _mm_madd_epi16(xmm[3][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[2]), _mm_madd_epi16(xmm[3][11], cmm[2])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[1]), _mm_madd_epi16(xmm[4][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[1]), _mm_madd_epi16(xmm[4][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[3]), _mm_madd_epi16(xmm[4][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[3]), _mm_madd_epi16(xmm[4][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[0]), _mm_madd_epi16(xmm[4][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[0]), _mm_madd_epi16(xmm[4][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[2]), _mm_madd_epi16(xmm[4][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[2]), _mm_madd_epi16(xmm[4][11], cmm[2])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[1]), _mm_madd_epi16(xmm[5][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[1]), _mm_madd_epi16(xmm[5][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[3]), _mm_madd_epi16(xmm[5][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[3]), _mm_madd_epi16(xmm[5][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[0]), _mm_madd_epi16(xmm[5][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[0]), _mm_madd_epi16(xmm[5][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[2]), _mm_madd_epi16(xmm[5][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[2]), _mm_madd_epi16(xmm[5][11], cmm[2])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[1]), _mm_madd_epi16(xmm[6][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[1]), _mm_madd_epi16(xmm[6][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[3]), _mm_madd_epi16(xmm[6][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[3]), _mm_madd_epi16(xmm[6][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[0]), _mm_madd_epi16(xmm[6][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[0]), _mm_madd_epi16(xmm[6][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[2]), _mm_madd_epi16(xmm[6][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[2]), _mm_madd_epi16(xmm[6][11], cmm[2])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[1]), _mm_madd_epi16(xmm[7][15], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[1]), _mm_madd_epi16(xmm[7][8], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[3]), _mm_madd_epi16(xmm[7][14], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[3]), _mm_madd_epi16(xmm[7][9], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[0]), _mm_madd_epi16(xmm[7][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[0]), _mm_madd_epi16(xmm[7][10], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[2]), _mm_madd_epi16(xmm[7][12], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[2]), _mm_madd_epi16(xmm[7][11], cmm[2])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (12 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (12 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (12 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (12 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[20*line]
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[2]), _mm_madd_epi16(xmm[0][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[2]), _mm_madd_epi16(xmm[0][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[0][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[0][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[3]), _mm_madd_epi16(xmm[0][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[3]), _mm_madd_epi16(xmm[0][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[0][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[0][11], cmm[1])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[2]), _mm_madd_epi16(xmm[1][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[2]), _mm_madd_epi16(xmm[1][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[0]), _mm_madd_epi16(xmm[1][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[0]), _mm_madd_epi16(xmm[1][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[3]), _mm_madd_epi16(xmm[1][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[3]), _mm_madd_epi16(xmm[1][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[1]), _mm_madd_epi16(xmm[1][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[1]), _mm_madd_epi16(xmm[1][11], cmm[1])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[2]), _mm_madd_epi16(xmm[2][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[2]), _mm_madd_epi16(xmm[2][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[0]), _mm_madd_epi16(xmm[2][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[0]), _mm_madd_epi16(xmm[2][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[3]), _mm_madd_epi16(xmm[2][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[3]), _mm_madd_epi16(xmm[2][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[1]), _mm_madd_epi16(xmm[2][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[1]), _mm_madd_epi16(xmm[2][11], cmm[1])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[2]), _mm_madd_epi16(xmm[3][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[2]), _mm_madd_epi16(xmm[3][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[0]), _mm_madd_epi16(xmm[3][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[0]), _mm_madd_epi16(xmm[3][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[3]), _mm_madd_epi16(xmm[3][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[3]), _mm_madd_epi16(xmm[3][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[1]), _mm_madd_epi16(xmm[3][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[1]), _mm_madd_epi16(xmm[3][11], cmm[1])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[2]), _mm_madd_epi16(xmm[4][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[2]), _mm_madd_epi16(xmm[4][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[0]), _mm_madd_epi16(xmm[4][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[0]), _mm_madd_epi16(xmm[4][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[3]), _mm_madd_epi16(xmm[4][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[3]), _mm_madd_epi16(xmm[4][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[1]), _mm_madd_epi16(xmm[4][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[1]), _mm_madd_epi16(xmm[4][11], cmm[1])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[2]), _mm_madd_epi16(xmm[5][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[2]), _mm_madd_epi16(xmm[5][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[0]), _mm_madd_epi16(xmm[5][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[0]), _mm_madd_epi16(xmm[5][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[3]), _mm_madd_epi16(xmm[5][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[3]), _mm_madd_epi16(xmm[5][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[1]), _mm_madd_epi16(xmm[5][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[1]), _mm_madd_epi16(xmm[5][11], cmm[1])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[2]), _mm_madd_epi16(xmm[6][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[2]), _mm_madd_epi16(xmm[6][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[0]), _mm_madd_epi16(xmm[6][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[0]), _mm_madd_epi16(xmm[6][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[3]), _mm_madd_epi16(xmm[6][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[3]), _mm_madd_epi16(xmm[6][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[1]), _mm_madd_epi16(xmm[6][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[1]), _mm_madd_epi16(xmm[6][11], cmm[1])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[2]), _mm_madd_epi16(xmm[7][15], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[2]), _mm_madd_epi16(xmm[7][8], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[0]), _mm_madd_epi16(xmm[7][14], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[0]), _mm_madd_epi16(xmm[7][9], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[3]), _mm_madd_epi16(xmm[7][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[3]), _mm_madd_epi16(xmm[7][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[1]), _mm_madd_epi16(xmm[7][12], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[1]), _mm_madd_epi16(xmm[7][11], cmm[1])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (20 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (20 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (20 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (20 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[28*line]
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[3]), _mm_madd_epi16(xmm[0][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[3]), _mm_madd_epi16(xmm[0][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[2]), _mm_madd_epi16(xmm[0][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[2]), _mm_madd_epi16(xmm[0][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[0][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[0][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[0][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[0][11], cmm[0])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[3]), _mm_madd_epi16(xmm[1][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[1][7], cmm[3]), _mm_madd_epi16(xmm[1][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][1], cmm[2]), _mm_madd_epi16(xmm[1][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[2]), _mm_madd_epi16(xmm[1][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[1]), _mm_madd_epi16(xmm[1][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[1][5], cmm[1]), _mm_madd_epi16(xmm[1][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][3], cmm[0]), _mm_madd_epi16(xmm[1][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[0]), _mm_madd_epi16(xmm[1][11], cmm[0])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[3]), _mm_madd_epi16(xmm[2][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], cmm[3]), _mm_madd_epi16(xmm[2][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][1], cmm[2]), _mm_madd_epi16(xmm[2][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[2]), _mm_madd_epi16(xmm[2][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[1]), _mm_madd_epi16(xmm[2][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], cmm[1]), _mm_madd_epi16(xmm[2][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][3], cmm[0]), _mm_madd_epi16(xmm[2][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[0]), _mm_madd_epi16(xmm[2][11], cmm[0])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[3]), _mm_madd_epi16(xmm[3][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[3][7], cmm[3]), _mm_madd_epi16(xmm[3][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][1], cmm[2]), _mm_madd_epi16(xmm[3][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[2]), _mm_madd_epi16(xmm[3][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[1]), _mm_madd_epi16(xmm[3][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[3][5], cmm[1]), _mm_madd_epi16(xmm[3][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][3], cmm[0]), _mm_madd_epi16(xmm[3][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[0]), _mm_madd_epi16(xmm[3][11], cmm[0])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[3]), _mm_madd_epi16(xmm[4][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[4][7], cmm[3]), _mm_madd_epi16(xmm[4][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], cmm[2]), _mm_madd_epi16(xmm[4][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[2]), _mm_madd_epi16(xmm[4][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[1]), _mm_madd_epi16(xmm[4][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[4][5], cmm[1]), _mm_madd_epi16(xmm[4][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], cmm[0]), _mm_madd_epi16(xmm[4][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[0]), _mm_madd_epi16(xmm[4][11], cmm[0])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[3]), _mm_madd_epi16(xmm[5][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[5][7], cmm[3]), _mm_madd_epi16(xmm[5][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][1], cmm[2]), _mm_madd_epi16(xmm[5][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[2]), _mm_madd_epi16(xmm[5][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[1]), _mm_madd_epi16(xmm[5][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[5][5], cmm[1]), _mm_madd_epi16(xmm[5][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][3], cmm[0]), _mm_madd_epi16(xmm[5][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[0]), _mm_madd_epi16(xmm[5][11], cmm[0])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[3]), _mm_madd_epi16(xmm[6][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], cmm[3]), _mm_madd_epi16(xmm[6][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][1], cmm[2]), _mm_madd_epi16(xmm[6][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[2]), _mm_madd_epi16(xmm[6][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[1]), _mm_madd_epi16(xmm[6][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], cmm[1]), _mm_madd_epi16(xmm[6][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][3], cmm[0]), _mm_madd_epi16(xmm[6][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[0]), _mm_madd_epi16(xmm[6][11], cmm[0])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[3]), _mm_madd_epi16(xmm[7][15], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[7][7], cmm[3]), _mm_madd_epi16(xmm[7][8], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][1], cmm[2]), _mm_madd_epi16(xmm[7][14], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[2]), _mm_madd_epi16(xmm[7][9], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[1]), _mm_madd_epi16(xmm[7][13], cmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[7][5], cmm[1]), _mm_madd_epi16(xmm[7][10], cmm[1])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][3], cmm[0]), _mm_madd_epi16(xmm[7][12], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[0]), _mm_madd_epi16(xmm[7][11], cmm[0])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (28 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (28 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (28 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (28 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]: Set the constands and compute "dst" --> dst(2), dst(6), ..., and dst(30)
  cmm[0] = _mm_set1_epi16(90); cmm[1] = _mm_set1_epi16(87); cmm[2] = _mm_set1_epi16(80); cmm[3] = _mm_set1_epi16(70);
  cmm[4] = _mm_set1_epi16(57); cmm[5] = _mm_set1_epi16(43); cmm[6] = _mm_set1_epi16(25); cmm[7] = _mm_set1_epi16(9);

  // [JDS]:  dst[2*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[0][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[2]), _mm_madd_epi16(xmm[0][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[3]), _mm_madd_epi16(xmm[0][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[4]), _mm_madd_epi16(xmm[0][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[5]), _mm_madd_epi16(xmm[0][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[6]), _mm_madd_epi16(xmm[0][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[7]), _mm_madd_epi16(xmm[0][8], cmm[7])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[1]), _mm_madd_epi16(xmm[1][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[2]), _mm_madd_epi16(xmm[1][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[3]), _mm_madd_epi16(xmm[1][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[4]), _mm_madd_epi16(xmm[1][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[5]), _mm_madd_epi16(xmm[1][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[6]), _mm_madd_epi16(xmm[1][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[7]), _mm_madd_epi16(xmm[1][8], cmm[7])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[1]), _mm_madd_epi16(xmm[2][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[2]), _mm_madd_epi16(xmm[2][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[3]), _mm_madd_epi16(xmm[2][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[4]), _mm_madd_epi16(xmm[2][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[5]), _mm_madd_epi16(xmm[2][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[6]), _mm_madd_epi16(xmm[2][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[7]), _mm_madd_epi16(xmm[2][8], cmm[7])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[1]), _mm_madd_epi16(xmm[3][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[2]), _mm_madd_epi16(xmm[3][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[3]), _mm_madd_epi16(xmm[3][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[4]), _mm_madd_epi16(xmm[3][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[5]), _mm_madd_epi16(xmm[3][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[6]), _mm_madd_epi16(xmm[3][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[7]), _mm_madd_epi16(xmm[3][8], cmm[7])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[0]), _mm_madd_epi16(xmm[4][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[1]), _mm_madd_epi16(xmm[4][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[2]), _mm_madd_epi16(xmm[4][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[3]), _mm_madd_epi16(xmm[4][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[4]), _mm_madd_epi16(xmm[4][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[5]), _mm_madd_epi16(xmm[4][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[6]), _mm_madd_epi16(xmm[4][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[7]), _mm_madd_epi16(xmm[4][8], cmm[7])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[0]), _mm_madd_epi16(xmm[5][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[1]), _mm_madd_epi16(xmm[5][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[2]), _mm_madd_epi16(xmm[5][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[3]), _mm_madd_epi16(xmm[5][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[4]), _mm_madd_epi16(xmm[5][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[5]), _mm_madd_epi16(xmm[5][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[6]), _mm_madd_epi16(xmm[5][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[7]), _mm_madd_epi16(xmm[5][8], cmm[7])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[0]), _mm_madd_epi16(xmm[6][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[1]), _mm_madd_epi16(xmm[6][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[2]), _mm_madd_epi16(xmm[6][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[3]), _mm_madd_epi16(xmm[6][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[4]), _mm_madd_epi16(xmm[6][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[5]), _mm_madd_epi16(xmm[6][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[6]), _mm_madd_epi16(xmm[6][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[7]), _mm_madd_epi16(xmm[6][8], cmm[7])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[0]), _mm_madd_epi16(xmm[7][15], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[1]), _mm_madd_epi16(xmm[7][14], cmm[1])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[2]), _mm_madd_epi16(xmm[7][13], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[3]), _mm_madd_epi16(xmm[7][12], cmm[3])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[4]), _mm_madd_epi16(xmm[7][11], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[5]), _mm_madd_epi16(xmm[7][10], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[6]), _mm_madd_epi16(xmm[7][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[7]), _mm_madd_epi16(xmm[7][8], cmm[7])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (2 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (2 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (2 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (2 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[6*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[0][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[4]), _mm_madd_epi16(xmm[0][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[7]), _mm_madd_epi16(xmm[0][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[5]), _mm_madd_epi16(xmm[0][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[2]), _mm_madd_epi16(xmm[0][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[0]), _mm_madd_epi16(xmm[0][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[3]), _mm_madd_epi16(xmm[0][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[6]), _mm_madd_epi16(xmm[0][8], cmm[6])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[1]), _mm_madd_epi16(xmm[1][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[4]), _mm_madd_epi16(xmm[1][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[7]), _mm_madd_epi16(xmm[1][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[5]), _mm_madd_epi16(xmm[1][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[2]), _mm_madd_epi16(xmm[1][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[0]), _mm_madd_epi16(xmm[1][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[3]), _mm_madd_epi16(xmm[1][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[6]), _mm_madd_epi16(xmm[1][8], cmm[6])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[1]), _mm_madd_epi16(xmm[2][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[4]), _mm_madd_epi16(xmm[2][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[7]), _mm_madd_epi16(xmm[2][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[5]), _mm_madd_epi16(xmm[2][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[2]), _mm_madd_epi16(xmm[2][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[0]), _mm_madd_epi16(xmm[2][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[3]), _mm_madd_epi16(xmm[2][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[6]), _mm_madd_epi16(xmm[2][8], cmm[6])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[1]), _mm_madd_epi16(xmm[3][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[4]), _mm_madd_epi16(xmm[3][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[7]), _mm_madd_epi16(xmm[3][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[5]), _mm_madd_epi16(xmm[3][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[2]), _mm_madd_epi16(xmm[3][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[0]), _mm_madd_epi16(xmm[3][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[3]), _mm_madd_epi16(xmm[3][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[6]), _mm_madd_epi16(xmm[3][8], cmm[6])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[1]), _mm_madd_epi16(xmm[4][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[4]), _mm_madd_epi16(xmm[4][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[7]), _mm_madd_epi16(xmm[4][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[5]), _mm_madd_epi16(xmm[4][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[2]), _mm_madd_epi16(xmm[4][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[0]), _mm_madd_epi16(xmm[4][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[3]), _mm_madd_epi16(xmm[4][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[6]), _mm_madd_epi16(xmm[4][8], cmm[6])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[1]), _mm_madd_epi16(xmm[5][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[4]), _mm_madd_epi16(xmm[5][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[7]), _mm_madd_epi16(xmm[5][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[5]), _mm_madd_epi16(xmm[5][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[2]), _mm_madd_epi16(xmm[5][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[0]), _mm_madd_epi16(xmm[5][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[3]), _mm_madd_epi16(xmm[5][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[6]), _mm_madd_epi16(xmm[5][8], cmm[6])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[1]), _mm_madd_epi16(xmm[6][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[4]), _mm_madd_epi16(xmm[6][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[7]), _mm_madd_epi16(xmm[6][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[5]), _mm_madd_epi16(xmm[6][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[2]), _mm_madd_epi16(xmm[6][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[0]), _mm_madd_epi16(xmm[6][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[3]), _mm_madd_epi16(xmm[6][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[6]), _mm_madd_epi16(xmm[6][8], cmm[6])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[1]), _mm_madd_epi16(xmm[7][15], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[4]), _mm_madd_epi16(xmm[7][14], cmm[4])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[7]), _mm_madd_epi16(xmm[7][13], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[5]), _mm_madd_epi16(xmm[7][12], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[2]), _mm_madd_epi16(xmm[7][11], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[0]), _mm_madd_epi16(xmm[7][10], cmm[0])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[3]), _mm_madd_epi16(xmm[7][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[6]), _mm_madd_epi16(xmm[7][8], cmm[6])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (6 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (6 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (6 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (6 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[10*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[2]), _mm_madd_epi16(xmm[0][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[7]), _mm_madd_epi16(xmm[0][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[3]), _mm_madd_epi16(xmm[0][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[0][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[6]), _mm_madd_epi16(xmm[0][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[4]), _mm_madd_epi16(xmm[0][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[0][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[5]), _mm_madd_epi16(xmm[0][8], cmm[5])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[2]), _mm_madd_epi16(xmm[1][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[7]), _mm_madd_epi16(xmm[1][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[3]), _mm_madd_epi16(xmm[1][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[1]), _mm_madd_epi16(xmm[1][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[6]), _mm_madd_epi16(xmm[1][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[4]), _mm_madd_epi16(xmm[1][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[0]), _mm_madd_epi16(xmm[1][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[5]), _mm_madd_epi16(xmm[1][8], cmm[5])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[2]), _mm_madd_epi16(xmm[2][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[7]), _mm_madd_epi16(xmm[2][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[3]), _mm_madd_epi16(xmm[2][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[1]), _mm_madd_epi16(xmm[2][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[6]), _mm_madd_epi16(xmm[2][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[4]), _mm_madd_epi16(xmm[2][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[0]), _mm_madd_epi16(xmm[2][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[5]), _mm_madd_epi16(xmm[2][8], cmm[5])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[2]), _mm_madd_epi16(xmm[3][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[7]), _mm_madd_epi16(xmm[3][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[3]), _mm_madd_epi16(xmm[3][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[1]), _mm_madd_epi16(xmm[3][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[6]), _mm_madd_epi16(xmm[3][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[4]), _mm_madd_epi16(xmm[3][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[0]), _mm_madd_epi16(xmm[3][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[5]), _mm_madd_epi16(xmm[3][8], cmm[5])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[2]), _mm_madd_epi16(xmm[4][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[7]), _mm_madd_epi16(xmm[4][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[3]), _mm_madd_epi16(xmm[4][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[1]), _mm_madd_epi16(xmm[4][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[6]), _mm_madd_epi16(xmm[4][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[4]), _mm_madd_epi16(xmm[4][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[0]), _mm_madd_epi16(xmm[4][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[5]), _mm_madd_epi16(xmm[4][8], cmm[5])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[2]), _mm_madd_epi16(xmm[5][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[7]), _mm_madd_epi16(xmm[5][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[3]), _mm_madd_epi16(xmm[5][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[1]), _mm_madd_epi16(xmm[5][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[6]), _mm_madd_epi16(xmm[5][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[4]), _mm_madd_epi16(xmm[5][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[0]), _mm_madd_epi16(xmm[5][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[5]), _mm_madd_epi16(xmm[5][8], cmm[5])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[2]), _mm_madd_epi16(xmm[6][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[7]), _mm_madd_epi16(xmm[6][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[3]), _mm_madd_epi16(xmm[6][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[1]), _mm_madd_epi16(xmm[6][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[6]), _mm_madd_epi16(xmm[6][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[4]), _mm_madd_epi16(xmm[6][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[0]), _mm_madd_epi16(xmm[6][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[5]), _mm_madd_epi16(xmm[6][8], cmm[5])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[2]), _mm_madd_epi16(xmm[7][15], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[7]), _mm_madd_epi16(xmm[7][14], cmm[7])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[3]), _mm_madd_epi16(xmm[7][13], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[1]), _mm_madd_epi16(xmm[7][12], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[6]), _mm_madd_epi16(xmm[7][11], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[4]), _mm_madd_epi16(xmm[7][10], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[0]), _mm_madd_epi16(xmm[7][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[5]), _mm_madd_epi16(xmm[7][8], cmm[5])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (10 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (10 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (10 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (10 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[14*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[3]), _mm_madd_epi16(xmm[0][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[5]), _mm_madd_epi16(xmm[0][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[0][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[7]), _mm_madd_epi16(xmm[0][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[0][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[6]), _mm_madd_epi16(xmm[0][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[2]), _mm_madd_epi16(xmm[0][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[4]), _mm_madd_epi16(xmm[0][8], cmm[4])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[3]), _mm_madd_epi16(xmm[1][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[5]), _mm_madd_epi16(xmm[1][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[1]), _mm_madd_epi16(xmm[1][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[7]), _mm_madd_epi16(xmm[1][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[0]), _mm_madd_epi16(xmm[1][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[6]), _mm_madd_epi16(xmm[1][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[2]), _mm_madd_epi16(xmm[1][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[4]), _mm_madd_epi16(xmm[1][8], cmm[4])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[3]), _mm_madd_epi16(xmm[2][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[5]), _mm_madd_epi16(xmm[2][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[1]), _mm_madd_epi16(xmm[2][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[7]), _mm_madd_epi16(xmm[2][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[0]), _mm_madd_epi16(xmm[2][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[6]), _mm_madd_epi16(xmm[2][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[2]), _mm_madd_epi16(xmm[2][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[4]), _mm_madd_epi16(xmm[2][8], cmm[4])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[3]), _mm_madd_epi16(xmm[3][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[5]), _mm_madd_epi16(xmm[3][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[1]), _mm_madd_epi16(xmm[3][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[7]), _mm_madd_epi16(xmm[3][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[0]), _mm_madd_epi16(xmm[3][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[6]), _mm_madd_epi16(xmm[3][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[2]), _mm_madd_epi16(xmm[3][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[4]), _mm_madd_epi16(xmm[3][8], cmm[4])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[3]), _mm_madd_epi16(xmm[4][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[5]), _mm_madd_epi16(xmm[4][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[1]), _mm_madd_epi16(xmm[4][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[7]), _mm_madd_epi16(xmm[4][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[0]), _mm_madd_epi16(xmm[4][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[6]), _mm_madd_epi16(xmm[4][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[2]), _mm_madd_epi16(xmm[4][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[4]), _mm_madd_epi16(xmm[4][8], cmm[4])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[3]), _mm_madd_epi16(xmm[5][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[5]), _mm_madd_epi16(xmm[5][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[1]), _mm_madd_epi16(xmm[5][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[7]), _mm_madd_epi16(xmm[5][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[0]), _mm_madd_epi16(xmm[5][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[6]), _mm_madd_epi16(xmm[5][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[2]), _mm_madd_epi16(xmm[5][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[4]), _mm_madd_epi16(xmm[5][8], cmm[4])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[3]), _mm_madd_epi16(xmm[6][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[5]), _mm_madd_epi16(xmm[6][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[1]), _mm_madd_epi16(xmm[6][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[7]), _mm_madd_epi16(xmm[6][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[0]), _mm_madd_epi16(xmm[6][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[6]), _mm_madd_epi16(xmm[6][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[2]), _mm_madd_epi16(xmm[6][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[4]), _mm_madd_epi16(xmm[6][8], cmm[4])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[3]), _mm_madd_epi16(xmm[7][15], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[5]), _mm_madd_epi16(xmm[7][14], cmm[5])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[1]), _mm_madd_epi16(xmm[7][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[7]), _mm_madd_epi16(xmm[7][12], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[0]), _mm_madd_epi16(xmm[7][11], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[6]), _mm_madd_epi16(xmm[7][10], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[2]), _mm_madd_epi16(xmm[7][9], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[4]), _mm_madd_epi16(xmm[7][8], cmm[4])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (14 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (14 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (14 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (14 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[18*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[4]), _mm_madd_epi16(xmm[0][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[2]), _mm_madd_epi16(xmm[0][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[6]), _mm_madd_epi16(xmm[0][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[0][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[7]), _mm_madd_epi16(xmm[0][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[0][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[5]), _mm_madd_epi16(xmm[0][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[3]), _mm_madd_epi16(xmm[0][8], cmm[3])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[4]), _mm_madd_epi16(xmm[1][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[2]), _mm_madd_epi16(xmm[1][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[6]), _mm_madd_epi16(xmm[1][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[0]), _mm_madd_epi16(xmm[1][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[7]), _mm_madd_epi16(xmm[1][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[1]), _mm_madd_epi16(xmm[1][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[5]), _mm_madd_epi16(xmm[1][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[3]), _mm_madd_epi16(xmm[1][8], cmm[3])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[4]), _mm_madd_epi16(xmm[2][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[2]), _mm_madd_epi16(xmm[2][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[6]), _mm_madd_epi16(xmm[2][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[0]), _mm_madd_epi16(xmm[2][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[7]), _mm_madd_epi16(xmm[2][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[1]), _mm_madd_epi16(xmm[2][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[5]), _mm_madd_epi16(xmm[2][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[3]), _mm_madd_epi16(xmm[2][8], cmm[3])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[4]), _mm_madd_epi16(xmm[3][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[2]), _mm_madd_epi16(xmm[3][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[6]), _mm_madd_epi16(xmm[3][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[0]), _mm_madd_epi16(xmm[3][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[7]), _mm_madd_epi16(xmm[3][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[1]), _mm_madd_epi16(xmm[3][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[5]), _mm_madd_epi16(xmm[3][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[3]), _mm_madd_epi16(xmm[3][8], cmm[3])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[4]), _mm_madd_epi16(xmm[4][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[2]), _mm_madd_epi16(xmm[4][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[6]), _mm_madd_epi16(xmm[4][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[0]), _mm_madd_epi16(xmm[4][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[7]), _mm_madd_epi16(xmm[4][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[1]), _mm_madd_epi16(xmm[4][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[5]), _mm_madd_epi16(xmm[4][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[3]), _mm_madd_epi16(xmm[4][8], cmm[3])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[4]), _mm_madd_epi16(xmm[5][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[2]), _mm_madd_epi16(xmm[5][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[6]), _mm_madd_epi16(xmm[5][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[0]), _mm_madd_epi16(xmm[5][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[7]), _mm_madd_epi16(xmm[5][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[1]), _mm_madd_epi16(xmm[5][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[5]), _mm_madd_epi16(xmm[5][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[3]), _mm_madd_epi16(xmm[5][8], cmm[3])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[4]), _mm_madd_epi16(xmm[6][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[2]), _mm_madd_epi16(xmm[6][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[6]), _mm_madd_epi16(xmm[6][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[0]), _mm_madd_epi16(xmm[6][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[7]), _mm_madd_epi16(xmm[6][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[1]), _mm_madd_epi16(xmm[6][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[5]), _mm_madd_epi16(xmm[6][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[3]), _mm_madd_epi16(xmm[6][8], cmm[3])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[4]), _mm_madd_epi16(xmm[7][15], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[2]), _mm_madd_epi16(xmm[7][14], cmm[2])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[6]), _mm_madd_epi16(xmm[7][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[0]), _mm_madd_epi16(xmm[7][12], cmm[0])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[7]), _mm_madd_epi16(xmm[7][11], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[1]), _mm_madd_epi16(xmm[7][10], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[5]), _mm_madd_epi16(xmm[7][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[3]), _mm_madd_epi16(xmm[7][8], cmm[3])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (18 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (18 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (18 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (18 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[22*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[5]), _mm_madd_epi16(xmm[0][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[0][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[4]), _mm_madd_epi16(xmm[0][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[6]), _mm_madd_epi16(xmm[0][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[0][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[3]), _mm_madd_epi16(xmm[0][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[7]), _mm_madd_epi16(xmm[0][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[2]), _mm_madd_epi16(xmm[0][8], cmm[2])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[5]), _mm_madd_epi16(xmm[1][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[0]), _mm_madd_epi16(xmm[1][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[4]), _mm_madd_epi16(xmm[1][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[6]), _mm_madd_epi16(xmm[1][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[1]), _mm_madd_epi16(xmm[1][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[3]), _mm_madd_epi16(xmm[1][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[7]), _mm_madd_epi16(xmm[1][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[2]), _mm_madd_epi16(xmm[1][8], cmm[2])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[5]), _mm_madd_epi16(xmm[2][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[0]), _mm_madd_epi16(xmm[2][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[4]), _mm_madd_epi16(xmm[2][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[6]), _mm_madd_epi16(xmm[2][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[1]), _mm_madd_epi16(xmm[2][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[3]), _mm_madd_epi16(xmm[2][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[7]), _mm_madd_epi16(xmm[2][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[2]), _mm_madd_epi16(xmm[2][8], cmm[2])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[5]), _mm_madd_epi16(xmm[3][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[0]), _mm_madd_epi16(xmm[3][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[4]), _mm_madd_epi16(xmm[3][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[6]), _mm_madd_epi16(xmm[3][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[1]), _mm_madd_epi16(xmm[3][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[3]), _mm_madd_epi16(xmm[3][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[7]), _mm_madd_epi16(xmm[3][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[2]), _mm_madd_epi16(xmm[3][8], cmm[2])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[5]), _mm_madd_epi16(xmm[4][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[0]), _mm_madd_epi16(xmm[4][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[4]), _mm_madd_epi16(xmm[4][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[6]), _mm_madd_epi16(xmm[4][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[1]), _mm_madd_epi16(xmm[4][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[3]), _mm_madd_epi16(xmm[4][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[7]), _mm_madd_epi16(xmm[4][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[2]), _mm_madd_epi16(xmm[4][8], cmm[2])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[5]), _mm_madd_epi16(xmm[5][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[0]), _mm_madd_epi16(xmm[5][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[4]), _mm_madd_epi16(xmm[5][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[6]), _mm_madd_epi16(xmm[5][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[1]), _mm_madd_epi16(xmm[5][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[3]), _mm_madd_epi16(xmm[5][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[7]), _mm_madd_epi16(xmm[5][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[2]), _mm_madd_epi16(xmm[5][8], cmm[2])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[5]), _mm_madd_epi16(xmm[6][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[0]), _mm_madd_epi16(xmm[6][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[4]), _mm_madd_epi16(xmm[6][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[6]), _mm_madd_epi16(xmm[6][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[1]), _mm_madd_epi16(xmm[6][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[3]), _mm_madd_epi16(xmm[6][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[7]), _mm_madd_epi16(xmm[6][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[2]), _mm_madd_epi16(xmm[6][8], cmm[2])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[5]), _mm_madd_epi16(xmm[7][15], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[0]), _mm_madd_epi16(xmm[7][14], cmm[0])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[4]), _mm_madd_epi16(xmm[7][13], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[6]), _mm_madd_epi16(xmm[7][12], cmm[6])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[1]), _mm_madd_epi16(xmm[7][11], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[3]), _mm_madd_epi16(xmm[7][10], cmm[3])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[7]), _mm_madd_epi16(xmm[7][9], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[2]), _mm_madd_epi16(xmm[7][8], cmm[2])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (22 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (22 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (22 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (22 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[26*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[6]), _mm_madd_epi16(xmm[0][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[3]), _mm_madd_epi16(xmm[0][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[2]), _mm_madd_epi16(xmm[0][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[5]), _mm_madd_epi16(xmm[0][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[7]), _mm_madd_epi16(xmm[0][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[4]), _mm_madd_epi16(xmm[0][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[0][8], cmm[1])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[6]), _mm_madd_epi16(xmm[1][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[3]), _mm_madd_epi16(xmm[1][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[2]), _mm_madd_epi16(xmm[1][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[5]), _mm_madd_epi16(xmm[1][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[7]), _mm_madd_epi16(xmm[1][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[4]), _mm_madd_epi16(xmm[1][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[1]), _mm_madd_epi16(xmm[1][8], cmm[1])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[6]), _mm_madd_epi16(xmm[2][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[3]), _mm_madd_epi16(xmm[2][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[2]), _mm_madd_epi16(xmm[2][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[5]), _mm_madd_epi16(xmm[2][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[7]), _mm_madd_epi16(xmm[2][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[4]), _mm_madd_epi16(xmm[2][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[1]), _mm_madd_epi16(xmm[2][8], cmm[1])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[6]), _mm_madd_epi16(xmm[3][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[3]), _mm_madd_epi16(xmm[3][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[2]), _mm_madd_epi16(xmm[3][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[5]), _mm_madd_epi16(xmm[3][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[7]), _mm_madd_epi16(xmm[3][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[4]), _mm_madd_epi16(xmm[3][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[1]), _mm_madd_epi16(xmm[3][8], cmm[1])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[6]), _mm_madd_epi16(xmm[4][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[3]), _mm_madd_epi16(xmm[4][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[0]), _mm_madd_epi16(xmm[4][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[2]), _mm_madd_epi16(xmm[4][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[5]), _mm_madd_epi16(xmm[4][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[7]), _mm_madd_epi16(xmm[4][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[4]), _mm_madd_epi16(xmm[4][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[1]), _mm_madd_epi16(xmm[4][8], cmm[1])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[6]), _mm_madd_epi16(xmm[5][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[3]), _mm_madd_epi16(xmm[5][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[0]), _mm_madd_epi16(xmm[5][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[2]), _mm_madd_epi16(xmm[5][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[5]), _mm_madd_epi16(xmm[5][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[7]), _mm_madd_epi16(xmm[5][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[4]), _mm_madd_epi16(xmm[5][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[1]), _mm_madd_epi16(xmm[5][8], cmm[1])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[6]), _mm_madd_epi16(xmm[6][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[3]), _mm_madd_epi16(xmm[6][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[0]), _mm_madd_epi16(xmm[6][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[2]), _mm_madd_epi16(xmm[6][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[5]), _mm_madd_epi16(xmm[6][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[7]), _mm_madd_epi16(xmm[6][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[4]), _mm_madd_epi16(xmm[6][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[1]), _mm_madd_epi16(xmm[6][8], cmm[1])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[6]), _mm_madd_epi16(xmm[7][15], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[3]), _mm_madd_epi16(xmm[7][14], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[0]), _mm_madd_epi16(xmm[7][13], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[2]), _mm_madd_epi16(xmm[7][12], cmm[2])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[5]), _mm_madd_epi16(xmm[7][11], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[7]), _mm_madd_epi16(xmm[7][10], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[4]), _mm_madd_epi16(xmm[7][9], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[1]), _mm_madd_epi16(xmm[7][8], cmm[1])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (26 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (26 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (26 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (26 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[30*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[7]), _mm_madd_epi16(xmm[0][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[6]), _mm_madd_epi16(xmm[0][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[5]), _mm_madd_epi16(xmm[0][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[4]), _mm_madd_epi16(xmm[0][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[3]), _mm_madd_epi16(xmm[0][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[2]), _mm_madd_epi16(xmm[0][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[0][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[0]), _mm_madd_epi16(xmm[0][8], cmm[0])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[7]), _mm_madd_epi16(xmm[1][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][1], cmm[6]), _mm_madd_epi16(xmm[1][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[5]), _mm_madd_epi16(xmm[1][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][3], cmm[4]), _mm_madd_epi16(xmm[1][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[3]), _mm_madd_epi16(xmm[1][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][5], cmm[2]), _mm_madd_epi16(xmm[1][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[1]), _mm_madd_epi16(xmm[1][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][7], cmm[0]), _mm_madd_epi16(xmm[1][8], cmm[0])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[7]), _mm_madd_epi16(xmm[2][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][1], cmm[6]), _mm_madd_epi16(xmm[2][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[5]), _mm_madd_epi16(xmm[2][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][3], cmm[4]), _mm_madd_epi16(xmm[2][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[3]), _mm_madd_epi16(xmm[2][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][5], cmm[2]), _mm_madd_epi16(xmm[2][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[1]), _mm_madd_epi16(xmm[2][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][7], cmm[0]), _mm_madd_epi16(xmm[2][8], cmm[0])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[7]), _mm_madd_epi16(xmm[3][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][1], cmm[6]), _mm_madd_epi16(xmm[3][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[5]), _mm_madd_epi16(xmm[3][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][3], cmm[4]), _mm_madd_epi16(xmm[3][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[3]), _mm_madd_epi16(xmm[3][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][5], cmm[2]), _mm_madd_epi16(xmm[3][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[1]), _mm_madd_epi16(xmm[3][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][7], cmm[0]), _mm_madd_epi16(xmm[3][8], cmm[0])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[7]), _mm_madd_epi16(xmm[4][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][1], cmm[6]), _mm_madd_epi16(xmm[4][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[5]), _mm_madd_epi16(xmm[4][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][3], cmm[4]), _mm_madd_epi16(xmm[4][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[3]), _mm_madd_epi16(xmm[4][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][5], cmm[2]), _mm_madd_epi16(xmm[4][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[1]), _mm_madd_epi16(xmm[4][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][7], cmm[0]), _mm_madd_epi16(xmm[4][8], cmm[0])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[7]), _mm_madd_epi16(xmm[5][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][1], cmm[6]), _mm_madd_epi16(xmm[5][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[5]), _mm_madd_epi16(xmm[5][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][3], cmm[4]), _mm_madd_epi16(xmm[5][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[3]), _mm_madd_epi16(xmm[5][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][5], cmm[2]), _mm_madd_epi16(xmm[5][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[1]), _mm_madd_epi16(xmm[5][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][7], cmm[0]), _mm_madd_epi16(xmm[5][8], cmm[0])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[7]), _mm_madd_epi16(xmm[6][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][1], cmm[6]), _mm_madd_epi16(xmm[6][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[5]), _mm_madd_epi16(xmm[6][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][3], cmm[4]), _mm_madd_epi16(xmm[6][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[3]), _mm_madd_epi16(xmm[6][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][5], cmm[2]), _mm_madd_epi16(xmm[6][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[1]), _mm_madd_epi16(xmm[6][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][7], cmm[0]), _mm_madd_epi16(xmm[6][8], cmm[0])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[7]), _mm_madd_epi16(xmm[7][15], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][1], cmm[6]), _mm_madd_epi16(xmm[7][14], cmm[6])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[5]), _mm_madd_epi16(xmm[7][13], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][3], cmm[4]), _mm_madd_epi16(xmm[7][12], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[3]), _mm_madd_epi16(xmm[7][11], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][5], cmm[2]), _mm_madd_epi16(xmm[7][10], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[1]), _mm_madd_epi16(xmm[7][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][7], cmm[0]), _mm_madd_epi16(xmm[7][8], cmm[0])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (30 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (30 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (30 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (30 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]: Set the constands and compute "dst" --> dst(1), dst(3), ... , and dst(31)
  cmm[0] = _mm_unpacklo_epi16(_mm_set1_epi16(90), _mm_set1_epi16(-90));  cmm[1] = _mm_unpacklo_epi16(_mm_set1_epi16(88), _mm_set1_epi16(-88));  cmm[2] = _mm_unpacklo_epi16(_mm_set1_epi16(85), _mm_set1_epi16(-85));
  cmm[3] = _mm_unpacklo_epi16(_mm_set1_epi16(82), _mm_set1_epi16(-82));  cmm[4] = _mm_unpacklo_epi16(_mm_set1_epi16(78), _mm_set1_epi16(-78));  cmm[5] = _mm_unpacklo_epi16(_mm_set1_epi16(73), _mm_set1_epi16(-73));
  cmm[6] = _mm_unpacklo_epi16(_mm_set1_epi16(67), _mm_set1_epi16(-67));  cmm[7] = _mm_unpacklo_epi16(_mm_set1_epi16(61), _mm_set1_epi16(-61));  cmm[8] = _mm_unpacklo_epi16(_mm_set1_epi16(54), _mm_set1_epi16(-54));
  cmm[9] = _mm_unpacklo_epi16(_mm_set1_epi16(46), _mm_set1_epi16(-46));  cmm[10] = _mm_unpacklo_epi16(_mm_set1_epi16(38), _mm_set1_epi16(-38));  cmm[11] = _mm_unpacklo_epi16(_mm_set1_epi16(31), _mm_set1_epi16(-31));
  cmm[12] = _mm_unpacklo_epi16(_mm_set1_epi16(22), _mm_set1_epi16(-22));  cmm[13] = _mm_unpacklo_epi16(_mm_set1_epi16(13), _mm_set1_epi16(-13));  cmm[14] = _mm_unpacklo_epi16(_mm_set1_epi16(4), _mm_set1_epi16(-4));

  // [JDS]:  dst[line]
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[0][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[3]), _mm_madd_epi16(xmm[0][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[5]), _mm_madd_epi16(xmm[0][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[7]), _mm_madd_epi16(xmm[0][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[9]), _mm_madd_epi16(xmm[0][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[11]), _mm_madd_epi16(xmm[0][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[13]), _mm_madd_epi16(xmm[0][15], cmm[14])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[1]), _mm_madd_epi16(xmm[1][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[3]), _mm_madd_epi16(xmm[1][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[5]), _mm_madd_epi16(xmm[1][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[7]), _mm_madd_epi16(xmm[1][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[9]), _mm_madd_epi16(xmm[1][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[11]), _mm_madd_epi16(xmm[1][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[13]), _mm_madd_epi16(xmm[1][15], cmm[14])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[1]), _mm_madd_epi16(xmm[2][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[3]), _mm_madd_epi16(xmm[2][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[5]), _mm_madd_epi16(xmm[2][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[7]), _mm_madd_epi16(xmm[2][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[9]), _mm_madd_epi16(xmm[2][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[11]), _mm_madd_epi16(xmm[2][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[13]), _mm_madd_epi16(xmm[2][15], cmm[14])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[1]), _mm_madd_epi16(xmm[3][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[3]), _mm_madd_epi16(xmm[3][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[5]), _mm_madd_epi16(xmm[3][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[7]), _mm_madd_epi16(xmm[3][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[9]), _mm_madd_epi16(xmm[3][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[11]), _mm_madd_epi16(xmm[3][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[13]), _mm_madd_epi16(xmm[3][15], cmm[14])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[0]), _mm_madd_epi16(xmm[4][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[1]), _mm_madd_epi16(xmm[4][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[3]), _mm_madd_epi16(xmm[4][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[5]), _mm_madd_epi16(xmm[4][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[7]), _mm_madd_epi16(xmm[4][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[9]), _mm_madd_epi16(xmm[4][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[11]), _mm_madd_epi16(xmm[4][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[13]), _mm_madd_epi16(xmm[4][15], cmm[14])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[0]), _mm_madd_epi16(xmm[5][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[1]), _mm_madd_epi16(xmm[5][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[3]), _mm_madd_epi16(xmm[5][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[5]), _mm_madd_epi16(xmm[5][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[7]), _mm_madd_epi16(xmm[5][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[9]), _mm_madd_epi16(xmm[5][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[11]), _mm_madd_epi16(xmm[5][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[13]), _mm_madd_epi16(xmm[5][15], cmm[14])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[0]), _mm_madd_epi16(xmm[6][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[1]), _mm_madd_epi16(xmm[6][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[3]), _mm_madd_epi16(xmm[6][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[5]), _mm_madd_epi16(xmm[6][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[7]), _mm_madd_epi16(xmm[6][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[9]), _mm_madd_epi16(xmm[6][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[11]), _mm_madd_epi16(xmm[6][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[13]), _mm_madd_epi16(xmm[6][15], cmm[14])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[0]), _mm_madd_epi16(xmm[7][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[1]), _mm_madd_epi16(xmm[7][3], cmm[2])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[3]), _mm_madd_epi16(xmm[7][5], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[5]), _mm_madd_epi16(xmm[7][7], cmm[6])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[7]), _mm_madd_epi16(xmm[7][9], cmm[8])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[9]), _mm_madd_epi16(xmm[7][11], cmm[10])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[11]), _mm_madd_epi16(xmm[7][13], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[13]), _mm_madd_epi16(xmm[7][15], cmm[14])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (line)), _mm_packs_epi32(rmm[0], rmm[1]));     _mm_storeu_si128((__m128i*)(dst + (line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));     _mm_storeu_si128((__m128i*)(dst + (line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[3*line]
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[0][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[6]), _mm_madd_epi16(xmm[0][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[12]), _mm_madd_epi16(xmm[0][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[11]), _mm_madd_epi16(xmm[0][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[5]), _mm_madd_epi16(xmm[0][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[0]), _mm_madd_epi16(xmm[0][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[4]), _mm_madd_epi16(xmm[0][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[10]), _mm_madd_epi16(xmm[0][15], cmm[13])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[0]), _mm_madd_epi16(xmm[1][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[6]), _mm_madd_epi16(xmm[1][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[12]), _mm_madd_epi16(xmm[1][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[11]), _mm_madd_epi16(xmm[1][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[5]), _mm_madd_epi16(xmm[1][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[0]), _mm_madd_epi16(xmm[1][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[4]), _mm_madd_epi16(xmm[1][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[10]), _mm_madd_epi16(xmm[1][15], cmm[13])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[0]), _mm_madd_epi16(xmm[2][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[6]), _mm_madd_epi16(xmm[2][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[12]), _mm_madd_epi16(xmm[2][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[11]), _mm_madd_epi16(xmm[2][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[5]), _mm_madd_epi16(xmm[2][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[0]), _mm_madd_epi16(xmm[2][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[4]), _mm_madd_epi16(xmm[2][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[10]), _mm_madd_epi16(xmm[2][15], cmm[13])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[0]), _mm_madd_epi16(xmm[3][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[6]), _mm_madd_epi16(xmm[3][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[12]), _mm_madd_epi16(xmm[3][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[11]), _mm_madd_epi16(xmm[3][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[5]), _mm_madd_epi16(xmm[3][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[0]), _mm_madd_epi16(xmm[3][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[4]), _mm_madd_epi16(xmm[3][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[10]), _mm_madd_epi16(xmm[3][15], cmm[13])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[0]), _mm_madd_epi16(xmm[4][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[6]), _mm_madd_epi16(xmm[4][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[12]), _mm_madd_epi16(xmm[4][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[11]), _mm_madd_epi16(xmm[4][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[5]), _mm_madd_epi16(xmm[4][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[0]), _mm_madd_epi16(xmm[4][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[4]), _mm_madd_epi16(xmm[4][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[10]), _mm_madd_epi16(xmm[4][15], cmm[13])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[0]), _mm_madd_epi16(xmm[5][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[6]), _mm_madd_epi16(xmm[5][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[12]), _mm_madd_epi16(xmm[5][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[11]), _mm_madd_epi16(xmm[5][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[5]), _mm_madd_epi16(xmm[5][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[0]), _mm_madd_epi16(xmm[5][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[4]), _mm_madd_epi16(xmm[5][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[10]), _mm_madd_epi16(xmm[5][15], cmm[13])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[0]), _mm_madd_epi16(xmm[6][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[6]), _mm_madd_epi16(xmm[6][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[12]), _mm_madd_epi16(xmm[6][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[11]), _mm_madd_epi16(xmm[6][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[5]), _mm_madd_epi16(xmm[6][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[0]), _mm_madd_epi16(xmm[6][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[4]), _mm_madd_epi16(xmm[6][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[10]), _mm_madd_epi16(xmm[6][15], cmm[13])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[0]), _mm_madd_epi16(xmm[7][1], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[6]), _mm_madd_epi16(xmm[7][3], cmm[9])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[12]), _mm_madd_epi16(xmm[7][5], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[11]), _mm_madd_epi16(xmm[7][7], cmm[8])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[5]), _mm_madd_epi16(xmm[7][9], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[0]), _mm_madd_epi16(xmm[7][11], cmm[1])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[4]), _mm_madd_epi16(xmm[7][13], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[10]), _mm_madd_epi16(xmm[7][15], cmm[13])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (3 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (3 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (3 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (3 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[5*line]
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[0][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[11]), _mm_madd_epi16(xmm[0][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[8]), _mm_madd_epi16(xmm[0][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[0][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[9]), _mm_madd_epi16(xmm[0][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[10]), _mm_madd_epi16(xmm[0][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[0]), _mm_madd_epi16(xmm[0][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[7]), _mm_madd_epi16(xmm[0][15], cmm[12])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[1]), _mm_madd_epi16(xmm[1][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[11]), _mm_madd_epi16(xmm[1][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[8]), _mm_madd_epi16(xmm[1][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[0]), _mm_madd_epi16(xmm[1][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[9]), _mm_madd_epi16(xmm[1][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[10]), _mm_madd_epi16(xmm[1][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[0]), _mm_madd_epi16(xmm[1][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[7]), _mm_madd_epi16(xmm[1][15], cmm[12])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[1]), _mm_madd_epi16(xmm[2][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[11]), _mm_madd_epi16(xmm[2][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[8]), _mm_madd_epi16(xmm[2][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[0]), _mm_madd_epi16(xmm[2][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[9]), _mm_madd_epi16(xmm[2][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[10]), _mm_madd_epi16(xmm[2][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[0]), _mm_madd_epi16(xmm[2][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[7]), _mm_madd_epi16(xmm[2][15], cmm[12])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[1]), _mm_madd_epi16(xmm[3][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[11]), _mm_madd_epi16(xmm[3][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[8]), _mm_madd_epi16(xmm[3][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[0]), _mm_madd_epi16(xmm[3][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[9]), _mm_madd_epi16(xmm[3][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[10]), _mm_madd_epi16(xmm[3][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[0]), _mm_madd_epi16(xmm[3][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[7]), _mm_madd_epi16(xmm[3][15], cmm[12])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[1]), _mm_madd_epi16(xmm[4][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[11]), _mm_madd_epi16(xmm[4][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[8]), _mm_madd_epi16(xmm[4][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[0]), _mm_madd_epi16(xmm[4][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[9]), _mm_madd_epi16(xmm[4][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[10]), _mm_madd_epi16(xmm[4][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[0]), _mm_madd_epi16(xmm[4][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[7]), _mm_madd_epi16(xmm[4][15], cmm[12])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[1]), _mm_madd_epi16(xmm[5][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[11]), _mm_madd_epi16(xmm[5][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[8]), _mm_madd_epi16(xmm[5][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[0]), _mm_madd_epi16(xmm[5][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[9]), _mm_madd_epi16(xmm[5][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[10]), _mm_madd_epi16(xmm[5][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[0]), _mm_madd_epi16(xmm[5][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[7]), _mm_madd_epi16(xmm[5][15], cmm[12])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[1]), _mm_madd_epi16(xmm[6][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[11]), _mm_madd_epi16(xmm[6][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[8]), _mm_madd_epi16(xmm[6][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[0]), _mm_madd_epi16(xmm[6][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[9]), _mm_madd_epi16(xmm[6][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[10]), _mm_madd_epi16(xmm[6][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[0]), _mm_madd_epi16(xmm[6][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[7]), _mm_madd_epi16(xmm[6][15], cmm[12])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[1]), _mm_madd_epi16(xmm[7][1], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[11]), _mm_madd_epi16(xmm[7][3], cmm[13])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[8]), _mm_madd_epi16(xmm[7][5], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[0]), _mm_madd_epi16(xmm[7][7], cmm[4])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[9]), _mm_madd_epi16(xmm[7][9], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[10]), _mm_madd_epi16(xmm[7][11], cmm[5])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[0]), _mm_madd_epi16(xmm[7][13], cmm[2])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[7]), _mm_madd_epi16(xmm[7][15], cmm[12])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (5 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (5 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (5 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (5 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[7*line]
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[2]), _mm_madd_epi16(xmm[0][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[13]), _mm_madd_epi16(xmm[0][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[0][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[12]), _mm_madd_epi16(xmm[0][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[3]), _mm_madd_epi16(xmm[0][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][10], cmm[8]), _mm_madd_epi16(xmm[0][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[7]), _mm_madd_epi16(xmm[0][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[4]), _mm_madd_epi16(xmm[0][15], cmm[11])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[2]), _mm_madd_epi16(xmm[1][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[13]), _mm_madd_epi16(xmm[1][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[0]), _mm_madd_epi16(xmm[1][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[12]), _mm_madd_epi16(xmm[1][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[3]), _mm_madd_epi16(xmm[1][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][10], cmm[8]), _mm_madd_epi16(xmm[1][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[7]), _mm_madd_epi16(xmm[1][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[4]), _mm_madd_epi16(xmm[1][15], cmm[11])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[2]), _mm_madd_epi16(xmm[2][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[13]), _mm_madd_epi16(xmm[2][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[0]), _mm_madd_epi16(xmm[2][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[12]), _mm_madd_epi16(xmm[2][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[3]), _mm_madd_epi16(xmm[2][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][10], cmm[8]), _mm_madd_epi16(xmm[2][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[7]), _mm_madd_epi16(xmm[2][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[4]), _mm_madd_epi16(xmm[2][15], cmm[11])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[2]), _mm_madd_epi16(xmm[3][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[13]), _mm_madd_epi16(xmm[3][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[0]), _mm_madd_epi16(xmm[3][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[12]), _mm_madd_epi16(xmm[3][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[3]), _mm_madd_epi16(xmm[3][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][10], cmm[8]), _mm_madd_epi16(xmm[3][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[7]), _mm_madd_epi16(xmm[3][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[4]), _mm_madd_epi16(xmm[3][15], cmm[11])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[2]), _mm_madd_epi16(xmm[4][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[13]), _mm_madd_epi16(xmm[4][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[0]), _mm_madd_epi16(xmm[4][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[12]), _mm_madd_epi16(xmm[4][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[3]), _mm_madd_epi16(xmm[4][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][10], cmm[8]), _mm_madd_epi16(xmm[4][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[7]), _mm_madd_epi16(xmm[4][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[4]), _mm_madd_epi16(xmm[4][15], cmm[11])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[2]), _mm_madd_epi16(xmm[5][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[13]), _mm_madd_epi16(xmm[5][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[0]), _mm_madd_epi16(xmm[5][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[12]), _mm_madd_epi16(xmm[5][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[3]), _mm_madd_epi16(xmm[5][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][10], cmm[8]), _mm_madd_epi16(xmm[5][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[7]), _mm_madd_epi16(xmm[5][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[4]), _mm_madd_epi16(xmm[5][15], cmm[11])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[2]), _mm_madd_epi16(xmm[6][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[13]), _mm_madd_epi16(xmm[6][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[0]), _mm_madd_epi16(xmm[6][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[12]), _mm_madd_epi16(xmm[6][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[3]), _mm_madd_epi16(xmm[6][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][10], cmm[8]), _mm_madd_epi16(xmm[6][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[7]), _mm_madd_epi16(xmm[6][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[4]), _mm_madd_epi16(xmm[6][15], cmm[11])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[2]), _mm_madd_epi16(xmm[7][1], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[13]), _mm_madd_epi16(xmm[7][3], cmm[6])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[0]), _mm_madd_epi16(xmm[7][5], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[12]), _mm_madd_epi16(xmm[7][7], cmm[10])));
  tmm[2] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[3]), _mm_madd_epi16(xmm[7][9], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][10], cmm[8]), _mm_madd_epi16(xmm[7][11], cmm[14])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[7]), _mm_madd_epi16(xmm[7][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[4]), _mm_madd_epi16(xmm[7][15], cmm[11])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (7 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (7 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (7 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (7 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[9*line]
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[3]), _mm_madd_epi16(xmm[0][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[8]), _mm_madd_epi16(xmm[0][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[7]), _mm_madd_epi16(xmm[0][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[4]), _mm_madd_epi16(xmm[0][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][8], cmm[11]), _mm_madd_epi16(xmm[0][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[0]), _mm_madd_epi16(xmm[0][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[14]), _mm_madd_epi16(xmm[0][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[1]), _mm_madd_epi16(xmm[0][15], cmm[10])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][0], cmm[3]), _mm_madd_epi16(xmm[1][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[8]), _mm_madd_epi16(xmm[1][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[7]), _mm_madd_epi16(xmm[1][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[4]), _mm_madd_epi16(xmm[1][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][8], cmm[11]), _mm_madd_epi16(xmm[1][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[0]), _mm_madd_epi16(xmm[1][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[14]), _mm_madd_epi16(xmm[1][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[1]), _mm_madd_epi16(xmm[1][15], cmm[10])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][0], cmm[3]), _mm_madd_epi16(xmm[2][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[8]), _mm_madd_epi16(xmm[2][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[7]), _mm_madd_epi16(xmm[2][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[4]), _mm_madd_epi16(xmm[2][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][8], cmm[11]), _mm_madd_epi16(xmm[2][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[0]), _mm_madd_epi16(xmm[2][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[14]), _mm_madd_epi16(xmm[2][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[1]), _mm_madd_epi16(xmm[2][15], cmm[10])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][0], cmm[3]), _mm_madd_epi16(xmm[3][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[8]), _mm_madd_epi16(xmm[3][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[7]), _mm_madd_epi16(xmm[3][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[4]), _mm_madd_epi16(xmm[3][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][8], cmm[11]), _mm_madd_epi16(xmm[3][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[0]), _mm_madd_epi16(xmm[3][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[14]), _mm_madd_epi16(xmm[3][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[1]), _mm_madd_epi16(xmm[3][15], cmm[10])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], cmm[3]), _mm_madd_epi16(xmm[4][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[8]), _mm_madd_epi16(xmm[4][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[7]), _mm_madd_epi16(xmm[4][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[4]), _mm_madd_epi16(xmm[4][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][8], cmm[11]), _mm_madd_epi16(xmm[4][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[0]), _mm_madd_epi16(xmm[4][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[14]), _mm_madd_epi16(xmm[4][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[1]), _mm_madd_epi16(xmm[4][15], cmm[10])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][0], cmm[3]), _mm_madd_epi16(xmm[5][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[8]), _mm_madd_epi16(xmm[5][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[7]), _mm_madd_epi16(xmm[5][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[4]), _mm_madd_epi16(xmm[5][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][8], cmm[11]), _mm_madd_epi16(xmm[5][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[0]), _mm_madd_epi16(xmm[5][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[14]), _mm_madd_epi16(xmm[5][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[1]), _mm_madd_epi16(xmm[5][15], cmm[10])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][0], cmm[3]), _mm_madd_epi16(xmm[6][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[8]), _mm_madd_epi16(xmm[6][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[7]), _mm_madd_epi16(xmm[6][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[4]), _mm_madd_epi16(xmm[6][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][8], cmm[11]), _mm_madd_epi16(xmm[6][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[0]), _mm_madd_epi16(xmm[6][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[14]), _mm_madd_epi16(xmm[6][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[1]), _mm_madd_epi16(xmm[6][15], cmm[10])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][0], cmm[3]), _mm_madd_epi16(xmm[7][1], cmm[12])), _mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[8]), _mm_madd_epi16(xmm[7][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[7]), _mm_madd_epi16(xmm[7][5], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[4]), _mm_madd_epi16(xmm[7][7], cmm[2])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][8], cmm[11]), _mm_madd_epi16(xmm[7][9], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[0]), _mm_madd_epi16(xmm[7][11], cmm[6])));
  tmm[3] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[14]), _mm_madd_epi16(xmm[7][13], cmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[1]), _mm_madd_epi16(xmm[7][15], cmm[10])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (9 * line)), _mm_packs_epi32(rmm[0], rmm[1]));    _mm_storeu_si128((__m128i*)(dst + (9 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (9 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));    _mm_storeu_si128((__m128i*)(dst + (9 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[11*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[4]), _mm_madd_epi16(xmm[0][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[3]), _mm_madd_epi16(xmm[0][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[13]), _mm_madd_epi16(xmm[0][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[6]), _mm_madd_epi16(xmm[0][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[1]), _mm_madd_epi16(xmm[0][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[11]), _mm_madd_epi16(xmm[0][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][12], cmm[8]), _mm_madd_epi16(xmm[0][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[0]), _mm_madd_epi16(xmm[0][15], cmm[9])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[4]), _mm_madd_epi16(xmm[1][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[3]), _mm_madd_epi16(xmm[1][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[13]), _mm_madd_epi16(xmm[1][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[6]), _mm_madd_epi16(xmm[1][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[1]), _mm_madd_epi16(xmm[1][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[11]), _mm_madd_epi16(xmm[1][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][12], cmm[8]), _mm_madd_epi16(xmm[1][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[0]), _mm_madd_epi16(xmm[1][15], cmm[9])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[4]), _mm_madd_epi16(xmm[2][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[3]), _mm_madd_epi16(xmm[2][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[13]), _mm_madd_epi16(xmm[2][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[6]), _mm_madd_epi16(xmm[2][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[1]), _mm_madd_epi16(xmm[2][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[11]), _mm_madd_epi16(xmm[2][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][12], cmm[8]), _mm_madd_epi16(xmm[2][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[0]), _mm_madd_epi16(xmm[2][15], cmm[9])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[4]), _mm_madd_epi16(xmm[3][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[3]), _mm_madd_epi16(xmm[3][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[13]), _mm_madd_epi16(xmm[3][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[6]), _mm_madd_epi16(xmm[3][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[1]), _mm_madd_epi16(xmm[3][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[11]), _mm_madd_epi16(xmm[3][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][12], cmm[8]), _mm_madd_epi16(xmm[3][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[0]), _mm_madd_epi16(xmm[3][15], cmm[9])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[4]), _mm_madd_epi16(xmm[4][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[3]), _mm_madd_epi16(xmm[4][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[13]), _mm_madd_epi16(xmm[4][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[6]), _mm_madd_epi16(xmm[4][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[1]), _mm_madd_epi16(xmm[4][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[11]), _mm_madd_epi16(xmm[4][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][12], cmm[8]), _mm_madd_epi16(xmm[4][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[0]), _mm_madd_epi16(xmm[4][15], cmm[9])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[4]), _mm_madd_epi16(xmm[5][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[3]), _mm_madd_epi16(xmm[5][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[13]), _mm_madd_epi16(xmm[5][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[6]), _mm_madd_epi16(xmm[5][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[1]), _mm_madd_epi16(xmm[5][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[11]), _mm_madd_epi16(xmm[5][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][12], cmm[8]), _mm_madd_epi16(xmm[5][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[0]), _mm_madd_epi16(xmm[5][15], cmm[9])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[4]), _mm_madd_epi16(xmm[6][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[3]), _mm_madd_epi16(xmm[6][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[13]), _mm_madd_epi16(xmm[6][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[6]), _mm_madd_epi16(xmm[6][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[1]), _mm_madd_epi16(xmm[6][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[11]), _mm_madd_epi16(xmm[6][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][12], cmm[8]), _mm_madd_epi16(xmm[6][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[0]), _mm_madd_epi16(xmm[6][15], cmm[9])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[4]), _mm_madd_epi16(xmm[7][1], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[3]), _mm_madd_epi16(xmm[7][3], cmm[5])));
  tmm[1] = _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[13]), _mm_madd_epi16(xmm[7][5], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[6]), _mm_madd_epi16(xmm[7][7], cmm[12])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[1]), _mm_madd_epi16(xmm[7][9], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[11]), _mm_madd_epi16(xmm[7][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][12], cmm[8]), _mm_madd_epi16(xmm[7][13], cmm[10])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[0]), _mm_madd_epi16(xmm[7][15], cmm[9])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (11 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (11 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (11 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (11 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[13*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[5]), _mm_madd_epi16(xmm[0][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[4]), _mm_madd_epi16(xmm[0][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[10]), _mm_madd_epi16(xmm[0][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][8], cmm[13]), _mm_madd_epi16(xmm[0][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][10], cmm[7]), _mm_madd_epi16(xmm[0][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[1]), _mm_madd_epi16(xmm[0][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[2]), _mm_madd_epi16(xmm[0][15], cmm[8])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[5]), _mm_madd_epi16(xmm[1][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[4]), _mm_madd_epi16(xmm[1][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[10]), _mm_madd_epi16(xmm[1][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][8], cmm[13]), _mm_madd_epi16(xmm[1][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][10], cmm[7]), _mm_madd_epi16(xmm[1][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[1]), _mm_madd_epi16(xmm[1][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[2]), _mm_madd_epi16(xmm[1][15], cmm[8])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[5]), _mm_madd_epi16(xmm[2][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[4]), _mm_madd_epi16(xmm[2][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[10]), _mm_madd_epi16(xmm[2][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][8], cmm[13]), _mm_madd_epi16(xmm[2][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][10], cmm[7]), _mm_madd_epi16(xmm[2][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[1]), _mm_madd_epi16(xmm[2][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[2]), _mm_madd_epi16(xmm[2][15], cmm[8])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[5]), _mm_madd_epi16(xmm[3][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[4]), _mm_madd_epi16(xmm[3][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[10]), _mm_madd_epi16(xmm[3][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][8], cmm[13]), _mm_madd_epi16(xmm[3][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][10], cmm[7]), _mm_madd_epi16(xmm[3][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[1]), _mm_madd_epi16(xmm[3][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[2]), _mm_madd_epi16(xmm[3][15], cmm[8])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[5]), _mm_madd_epi16(xmm[4][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[0]), _mm_madd_epi16(xmm[4][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[4]), _mm_madd_epi16(xmm[4][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[10]), _mm_madd_epi16(xmm[4][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][8], cmm[13]), _mm_madd_epi16(xmm[4][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][10], cmm[7]), _mm_madd_epi16(xmm[4][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[1]), _mm_madd_epi16(xmm[4][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[2]), _mm_madd_epi16(xmm[4][15], cmm[8])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[5]), _mm_madd_epi16(xmm[5][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[0]), _mm_madd_epi16(xmm[5][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[4]), _mm_madd_epi16(xmm[5][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[10]), _mm_madd_epi16(xmm[5][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][8], cmm[13]), _mm_madd_epi16(xmm[5][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][10], cmm[7]), _mm_madd_epi16(xmm[5][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[1]), _mm_madd_epi16(xmm[5][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[2]), _mm_madd_epi16(xmm[5][15], cmm[8])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[5]), _mm_madd_epi16(xmm[6][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[0]), _mm_madd_epi16(xmm[6][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[4]), _mm_madd_epi16(xmm[6][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[10]), _mm_madd_epi16(xmm[6][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][8], cmm[13]), _mm_madd_epi16(xmm[6][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][10], cmm[7]), _mm_madd_epi16(xmm[6][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[1]), _mm_madd_epi16(xmm[6][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[2]), _mm_madd_epi16(xmm[6][15], cmm[8])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[5]), _mm_madd_epi16(xmm[7][1], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[0]), _mm_madd_epi16(xmm[7][3], cmm[12])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[4]), _mm_madd_epi16(xmm[7][5], cmm[6])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[10]), _mm_madd_epi16(xmm[7][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][8], cmm[13]), _mm_madd_epi16(xmm[7][9], cmm[3])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][10], cmm[7]), _mm_madd_epi16(xmm[7][11], cmm[9])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[1]), _mm_madd_epi16(xmm[7][13], cmm[14])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[2]), _mm_madd_epi16(xmm[7][15], cmm[8])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (13 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (13 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (13 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (13 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[15*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[6]), _mm_madd_epi16(xmm[0][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[4]), _mm_madd_epi16(xmm[0][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[2]), _mm_madd_epi16(xmm[0][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[0][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[0]), _mm_madd_epi16(xmm[0][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[1]), _mm_madd_epi16(xmm[0][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[3]), _mm_madd_epi16(xmm[0][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[5]), _mm_madd_epi16(xmm[0][15], cmm[7])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[6]), _mm_madd_epi16(xmm[1][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[4]), _mm_madd_epi16(xmm[1][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[2]), _mm_madd_epi16(xmm[1][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[0]), _mm_madd_epi16(xmm[1][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[0]), _mm_madd_epi16(xmm[1][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[1]), _mm_madd_epi16(xmm[1][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[3]), _mm_madd_epi16(xmm[1][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[5]), _mm_madd_epi16(xmm[1][15], cmm[7])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[6]), _mm_madd_epi16(xmm[2][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[4]), _mm_madd_epi16(xmm[2][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[2]), _mm_madd_epi16(xmm[2][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[0]), _mm_madd_epi16(xmm[2][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[0]), _mm_madd_epi16(xmm[2][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[1]), _mm_madd_epi16(xmm[2][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[3]), _mm_madd_epi16(xmm[2][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[5]), _mm_madd_epi16(xmm[2][15], cmm[7])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[6]), _mm_madd_epi16(xmm[3][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[4]), _mm_madd_epi16(xmm[3][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[2]), _mm_madd_epi16(xmm[3][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[0]), _mm_madd_epi16(xmm[3][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[0]), _mm_madd_epi16(xmm[3][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[1]), _mm_madd_epi16(xmm[3][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[3]), _mm_madd_epi16(xmm[3][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[5]), _mm_madd_epi16(xmm[3][15], cmm[7])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[6]), _mm_madd_epi16(xmm[4][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[4]), _mm_madd_epi16(xmm[4][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[2]), _mm_madd_epi16(xmm[4][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[0]), _mm_madd_epi16(xmm[4][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[0]), _mm_madd_epi16(xmm[4][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[1]), _mm_madd_epi16(xmm[4][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[3]), _mm_madd_epi16(xmm[4][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[5]), _mm_madd_epi16(xmm[4][15], cmm[7])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[6]), _mm_madd_epi16(xmm[5][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[4]), _mm_madd_epi16(xmm[5][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[2]), _mm_madd_epi16(xmm[5][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[0]), _mm_madd_epi16(xmm[5][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[0]), _mm_madd_epi16(xmm[5][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[1]), _mm_madd_epi16(xmm[5][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[3]), _mm_madd_epi16(xmm[5][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[5]), _mm_madd_epi16(xmm[5][15], cmm[7])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[6]), _mm_madd_epi16(xmm[6][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[4]), _mm_madd_epi16(xmm[6][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[2]), _mm_madd_epi16(xmm[6][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[0]), _mm_madd_epi16(xmm[6][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[0]), _mm_madd_epi16(xmm[6][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[1]), _mm_madd_epi16(xmm[6][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[3]), _mm_madd_epi16(xmm[6][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[5]), _mm_madd_epi16(xmm[6][15], cmm[7])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[6]), _mm_madd_epi16(xmm[7][1], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[4]), _mm_madd_epi16(xmm[7][3], cmm[10])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[2]), _mm_madd_epi16(xmm[7][5], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[0]), _mm_madd_epi16(xmm[7][7], cmm[14])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[0]), _mm_madd_epi16(xmm[7][9], cmm[13])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[1]), _mm_madd_epi16(xmm[7][11], cmm[11])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[3]), _mm_madd_epi16(xmm[7][13], cmm[9])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[5]), _mm_madd_epi16(xmm[7][15], cmm[7])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (15 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (15 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (15 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (15 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[17*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[7]), _mm_madd_epi16(xmm[0][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[9]), _mm_madd_epi16(xmm[0][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[11]), _mm_madd_epi16(xmm[0][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[13]), _mm_madd_epi16(xmm[0][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[14]), _mm_madd_epi16(xmm[0][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[12]), _mm_madd_epi16(xmm[0][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[10]), _mm_madd_epi16(xmm[0][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[8]), _mm_madd_epi16(xmm[0][15], cmm[6])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[7]), _mm_madd_epi16(xmm[1][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[9]), _mm_madd_epi16(xmm[1][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[11]), _mm_madd_epi16(xmm[1][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[13]), _mm_madd_epi16(xmm[1][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[14]), _mm_madd_epi16(xmm[1][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[12]), _mm_madd_epi16(xmm[1][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[10]), _mm_madd_epi16(xmm[1][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[8]), _mm_madd_epi16(xmm[1][15], cmm[6])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[7]), _mm_madd_epi16(xmm[2][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[9]), _mm_madd_epi16(xmm[2][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[11]), _mm_madd_epi16(xmm[2][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[13]), _mm_madd_epi16(xmm[2][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[14]), _mm_madd_epi16(xmm[2][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[12]), _mm_madd_epi16(xmm[2][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[10]), _mm_madd_epi16(xmm[2][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[8]), _mm_madd_epi16(xmm[2][15], cmm[6])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[7]), _mm_madd_epi16(xmm[3][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[9]), _mm_madd_epi16(xmm[3][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[11]), _mm_madd_epi16(xmm[3][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[13]), _mm_madd_epi16(xmm[3][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[14]), _mm_madd_epi16(xmm[3][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[12]), _mm_madd_epi16(xmm[3][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[10]), _mm_madd_epi16(xmm[3][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[8]), _mm_madd_epi16(xmm[3][15], cmm[6])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[7]), _mm_madd_epi16(xmm[4][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[9]), _mm_madd_epi16(xmm[4][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[11]), _mm_madd_epi16(xmm[4][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[13]), _mm_madd_epi16(xmm[4][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[14]), _mm_madd_epi16(xmm[4][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[12]), _mm_madd_epi16(xmm[4][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[10]), _mm_madd_epi16(xmm[4][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[8]), _mm_madd_epi16(xmm[4][15], cmm[6])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[7]), _mm_madd_epi16(xmm[5][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[9]), _mm_madd_epi16(xmm[5][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[11]), _mm_madd_epi16(xmm[5][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[13]), _mm_madd_epi16(xmm[5][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[14]), _mm_madd_epi16(xmm[5][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[12]), _mm_madd_epi16(xmm[5][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[10]), _mm_madd_epi16(xmm[5][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[8]), _mm_madd_epi16(xmm[5][15], cmm[6])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[7]), _mm_madd_epi16(xmm[6][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[9]), _mm_madd_epi16(xmm[6][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[11]), _mm_madd_epi16(xmm[6][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[13]), _mm_madd_epi16(xmm[6][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[14]), _mm_madd_epi16(xmm[6][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[12]), _mm_madd_epi16(xmm[6][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[10]), _mm_madd_epi16(xmm[6][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[8]), _mm_madd_epi16(xmm[6][15], cmm[6])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[7]), _mm_madd_epi16(xmm[7][1], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[9]), _mm_madd_epi16(xmm[7][3], cmm[3])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[11]), _mm_madd_epi16(xmm[7][5], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[13]), _mm_madd_epi16(xmm[7][7], cmm[0])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[14]), _mm_madd_epi16(xmm[7][9], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[12]), _mm_madd_epi16(xmm[7][11], cmm[2])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[10]), _mm_madd_epi16(xmm[7][13], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[8]), _mm_madd_epi16(xmm[7][15], cmm[6])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (17 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (17 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (17 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (17 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[19*line]
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[8]), _mm_madd_epi16(xmm[0][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[14]), _mm_madd_epi16(xmm[0][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[9]), _mm_madd_epi16(xmm[0][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[3]), _mm_madd_epi16(xmm[0][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][8], cmm[0]), _mm_madd_epi16(xmm[0][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][10], cmm[6]), _mm_madd_epi16(xmm[0][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][12], cmm[12]), _mm_madd_epi16(xmm[0][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[11]), _mm_madd_epi16(xmm[0][15], cmm[5])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[8]), _mm_madd_epi16(xmm[1][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[14]), _mm_madd_epi16(xmm[1][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[9]), _mm_madd_epi16(xmm[1][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[3]), _mm_madd_epi16(xmm[1][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][8], cmm[0]), _mm_madd_epi16(xmm[1][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][10], cmm[6]), _mm_madd_epi16(xmm[1][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][12], cmm[12]), _mm_madd_epi16(xmm[1][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[11]), _mm_madd_epi16(xmm[1][15], cmm[5])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[8]), _mm_madd_epi16(xmm[2][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[14]), _mm_madd_epi16(xmm[2][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[9]), _mm_madd_epi16(xmm[2][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[3]), _mm_madd_epi16(xmm[2][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][8], cmm[0]), _mm_madd_epi16(xmm[2][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][10], cmm[6]), _mm_madd_epi16(xmm[2][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][12], cmm[12]), _mm_madd_epi16(xmm[2][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[11]), _mm_madd_epi16(xmm[2][15], cmm[5])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[8]), _mm_madd_epi16(xmm[3][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[14]), _mm_madd_epi16(xmm[3][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[9]), _mm_madd_epi16(xmm[3][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[3]), _mm_madd_epi16(xmm[3][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][8], cmm[0]), _mm_madd_epi16(xmm[3][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][10], cmm[6]), _mm_madd_epi16(xmm[3][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][12], cmm[12]), _mm_madd_epi16(xmm[3][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[11]), _mm_madd_epi16(xmm[3][15], cmm[5])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[8]), _mm_madd_epi16(xmm[4][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[14]), _mm_madd_epi16(xmm[4][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[9]), _mm_madd_epi16(xmm[4][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[3]), _mm_madd_epi16(xmm[4][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][8], cmm[0]), _mm_madd_epi16(xmm[4][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][10], cmm[6]), _mm_madd_epi16(xmm[4][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][12], cmm[12]), _mm_madd_epi16(xmm[4][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[11]), _mm_madd_epi16(xmm[4][15], cmm[5])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[8]), _mm_madd_epi16(xmm[5][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[14]), _mm_madd_epi16(xmm[5][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[9]), _mm_madd_epi16(xmm[5][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[3]), _mm_madd_epi16(xmm[5][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][8], cmm[0]), _mm_madd_epi16(xmm[5][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][10], cmm[6]), _mm_madd_epi16(xmm[5][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][12], cmm[12]), _mm_madd_epi16(xmm[5][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[11]), _mm_madd_epi16(xmm[5][15], cmm[5])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[8]), _mm_madd_epi16(xmm[6][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[14]), _mm_madd_epi16(xmm[6][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[9]), _mm_madd_epi16(xmm[6][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[3]), _mm_madd_epi16(xmm[6][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][8], cmm[0]), _mm_madd_epi16(xmm[6][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][10], cmm[6]), _mm_madd_epi16(xmm[6][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][12], cmm[12]), _mm_madd_epi16(xmm[6][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[11]), _mm_madd_epi16(xmm[6][15], cmm[5])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[8]), _mm_madd_epi16(xmm[7][1], cmm[2])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[14]), _mm_madd_epi16(xmm[7][3], cmm[1])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[9]), _mm_madd_epi16(xmm[7][5], cmm[7])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[3]), _mm_madd_epi16(xmm[7][7], cmm[13])));
  tmm[2] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][8], cmm[0]), _mm_madd_epi16(xmm[7][9], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][10], cmm[6]), _mm_madd_epi16(xmm[7][11], cmm[4])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][12], cmm[12]), _mm_madd_epi16(xmm[7][13], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[11]), _mm_madd_epi16(xmm[7][15], cmm[5])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (19 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (19 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (19 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (19 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[21*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[9]), _mm_madd_epi16(xmm[0][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[10]), _mm_madd_epi16(xmm[0][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[0][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[7]), _mm_madd_epi16(xmm[0][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[12]), _mm_madd_epi16(xmm[0][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][10], cmm[2]), _mm_madd_epi16(xmm[0][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][12], cmm[5]), _mm_madd_epi16(xmm[0][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[0][14], cmm[14]), _mm_madd_epi16(xmm[0][15], cmm[4])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[9]), _mm_madd_epi16(xmm[1][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][2], cmm[10]), _mm_madd_epi16(xmm[1][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[0]), _mm_madd_epi16(xmm[1][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[7]), _mm_madd_epi16(xmm[1][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[12]), _mm_madd_epi16(xmm[1][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][10], cmm[2]), _mm_madd_epi16(xmm[1][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][12], cmm[5]), _mm_madd_epi16(xmm[1][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[1][14], cmm[14]), _mm_madd_epi16(xmm[1][15], cmm[4])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[9]), _mm_madd_epi16(xmm[2][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], cmm[10]), _mm_madd_epi16(xmm[2][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[0]), _mm_madd_epi16(xmm[2][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[7]), _mm_madd_epi16(xmm[2][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[12]), _mm_madd_epi16(xmm[2][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][10], cmm[2]), _mm_madd_epi16(xmm[2][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][12], cmm[5]), _mm_madd_epi16(xmm[2][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[2][14], cmm[14]), _mm_madd_epi16(xmm[2][15], cmm[4])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[9]), _mm_madd_epi16(xmm[3][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][2], cmm[10]), _mm_madd_epi16(xmm[3][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[0]), _mm_madd_epi16(xmm[3][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[7]), _mm_madd_epi16(xmm[3][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[12]), _mm_madd_epi16(xmm[3][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][10], cmm[2]), _mm_madd_epi16(xmm[3][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][12], cmm[5]), _mm_madd_epi16(xmm[3][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[3][14], cmm[14]), _mm_madd_epi16(xmm[3][15], cmm[4])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[9]), _mm_madd_epi16(xmm[4][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][2], cmm[10]), _mm_madd_epi16(xmm[4][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[0]), _mm_madd_epi16(xmm[4][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[7]), _mm_madd_epi16(xmm[4][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[12]), _mm_madd_epi16(xmm[4][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][10], cmm[2]), _mm_madd_epi16(xmm[4][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][12], cmm[5]), _mm_madd_epi16(xmm[4][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[4][14], cmm[14]), _mm_madd_epi16(xmm[4][15], cmm[4])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[9]), _mm_madd_epi16(xmm[5][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][2], cmm[10]), _mm_madd_epi16(xmm[5][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[0]), _mm_madd_epi16(xmm[5][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[7]), _mm_madd_epi16(xmm[5][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[12]), _mm_madd_epi16(xmm[5][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][10], cmm[2]), _mm_madd_epi16(xmm[5][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][12], cmm[5]), _mm_madd_epi16(xmm[5][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[5][14], cmm[14]), _mm_madd_epi16(xmm[5][15], cmm[4])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[9]), _mm_madd_epi16(xmm[6][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], cmm[10]), _mm_madd_epi16(xmm[6][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[0]), _mm_madd_epi16(xmm[6][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[7]), _mm_madd_epi16(xmm[6][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[12]), _mm_madd_epi16(xmm[6][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][10], cmm[2]), _mm_madd_epi16(xmm[6][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][12], cmm[5]), _mm_madd_epi16(xmm[6][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[6][14], cmm[14]), _mm_madd_epi16(xmm[6][15], cmm[4])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[9]), _mm_madd_epi16(xmm[7][1], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][2], cmm[10]), _mm_madd_epi16(xmm[7][3], cmm[8])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[0]), _mm_madd_epi16(xmm[7][5], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[7]), _mm_madd_epi16(xmm[7][7], cmm[1])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[12]), _mm_madd_epi16(xmm[7][9], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][10], cmm[2]), _mm_madd_epi16(xmm[7][11], cmm[13])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][12], cmm[5]), _mm_madd_epi16(xmm[7][13], cmm[3])), _mm_add_epi32(_mm_madd_epi16(xmm[7][14], cmm[14]), _mm_madd_epi16(xmm[7][15], cmm[4])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (21 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (21 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (21 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (21 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[23*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[10]), _mm_madd_epi16(xmm[0][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[5]), _mm_madd_epi16(xmm[0][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[6]), _mm_madd_epi16(xmm[0][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[9]), _mm_madd_epi16(xmm[0][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][8], cmm[2]), _mm_madd_epi16(xmm[0][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[13]), _mm_madd_epi16(xmm[0][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][12], cmm[0]), _mm_madd_epi16(xmm[0][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][14], cmm[12]), _mm_madd_epi16(xmm[0][15], cmm[3])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[10]), _mm_madd_epi16(xmm[1][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[5]), _mm_madd_epi16(xmm[1][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[6]), _mm_madd_epi16(xmm[1][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[1][6], cmm[9]), _mm_madd_epi16(xmm[1][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][8], cmm[2]), _mm_madd_epi16(xmm[1][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[13]), _mm_madd_epi16(xmm[1][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][12], cmm[0]), _mm_madd_epi16(xmm[1][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][14], cmm[12]), _mm_madd_epi16(xmm[1][15], cmm[3])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[10]), _mm_madd_epi16(xmm[2][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[5]), _mm_madd_epi16(xmm[2][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[6]), _mm_madd_epi16(xmm[2][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], cmm[9]), _mm_madd_epi16(xmm[2][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][8], cmm[2]), _mm_madd_epi16(xmm[2][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[13]), _mm_madd_epi16(xmm[2][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][12], cmm[0]), _mm_madd_epi16(xmm[2][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][14], cmm[12]), _mm_madd_epi16(xmm[2][15], cmm[3])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[10]), _mm_madd_epi16(xmm[3][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[5]), _mm_madd_epi16(xmm[3][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[6]), _mm_madd_epi16(xmm[3][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[3][6], cmm[9]), _mm_madd_epi16(xmm[3][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][8], cmm[2]), _mm_madd_epi16(xmm[3][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[13]), _mm_madd_epi16(xmm[3][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][12], cmm[0]), _mm_madd_epi16(xmm[3][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][14], cmm[12]), _mm_madd_epi16(xmm[3][15], cmm[3])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[10]), _mm_madd_epi16(xmm[4][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[5]), _mm_madd_epi16(xmm[4][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[6]), _mm_madd_epi16(xmm[4][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[4][6], cmm[9]), _mm_madd_epi16(xmm[4][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][8], cmm[2]), _mm_madd_epi16(xmm[4][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[13]), _mm_madd_epi16(xmm[4][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][12], cmm[0]), _mm_madd_epi16(xmm[4][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][14], cmm[12]), _mm_madd_epi16(xmm[4][15], cmm[3])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[10]), _mm_madd_epi16(xmm[5][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[5]), _mm_madd_epi16(xmm[5][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[6]), _mm_madd_epi16(xmm[5][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[5][6], cmm[9]), _mm_madd_epi16(xmm[5][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][8], cmm[2]), _mm_madd_epi16(xmm[5][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[13]), _mm_madd_epi16(xmm[5][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][12], cmm[0]), _mm_madd_epi16(xmm[5][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][14], cmm[12]), _mm_madd_epi16(xmm[5][15], cmm[3])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[10]), _mm_madd_epi16(xmm[6][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[5]), _mm_madd_epi16(xmm[6][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[6]), _mm_madd_epi16(xmm[6][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], cmm[9]), _mm_madd_epi16(xmm[6][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][8], cmm[2]), _mm_madd_epi16(xmm[6][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[13]), _mm_madd_epi16(xmm[6][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][12], cmm[0]), _mm_madd_epi16(xmm[6][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][14], cmm[12]), _mm_madd_epi16(xmm[6][15], cmm[3])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[10]), _mm_madd_epi16(xmm[7][1], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[5]), _mm_madd_epi16(xmm[7][3], cmm[14])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[6]), _mm_madd_epi16(xmm[7][5], cmm[0])), _mm_add_epi32(_mm_madd_epi16(xmm[7][6], cmm[9]), _mm_madd_epi16(xmm[7][7], cmm[11])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][8], cmm[2]), _mm_madd_epi16(xmm[7][9], cmm[4])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[13]), _mm_madd_epi16(xmm[7][11], cmm[7])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][12], cmm[0]), _mm_madd_epi16(xmm[7][13], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][14], cmm[12]), _mm_madd_epi16(xmm[7][15], cmm[3])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (23 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (23 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (23 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (23 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[25*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[11]), _mm_madd_epi16(xmm[0][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[0][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[14]), _mm_madd_epi16(xmm[0][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[0][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][8], cmm[10]), _mm_madd_epi16(xmm[0][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][10], cmm[5]), _mm_madd_epi16(xmm[0][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][12], cmm[6]), _mm_madd_epi16(xmm[0][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][14], cmm[9]), _mm_madd_epi16(xmm[0][15], cmm[2])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[11]), _mm_madd_epi16(xmm[1][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[0]), _mm_madd_epi16(xmm[1][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][4], cmm[14]), _mm_madd_epi16(xmm[1][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[1]), _mm_madd_epi16(xmm[1][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][8], cmm[10]), _mm_madd_epi16(xmm[1][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][10], cmm[5]), _mm_madd_epi16(xmm[1][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][12], cmm[6]), _mm_madd_epi16(xmm[1][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][14], cmm[9]), _mm_madd_epi16(xmm[1][15], cmm[2])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[11]), _mm_madd_epi16(xmm[2][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[0]), _mm_madd_epi16(xmm[2][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][4], cmm[14]), _mm_madd_epi16(xmm[2][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[1]), _mm_madd_epi16(xmm[2][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][8], cmm[10]), _mm_madd_epi16(xmm[2][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][10], cmm[5]), _mm_madd_epi16(xmm[2][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][12], cmm[6]), _mm_madd_epi16(xmm[2][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][14], cmm[9]), _mm_madd_epi16(xmm[2][15], cmm[2])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[11]), _mm_madd_epi16(xmm[3][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[0]), _mm_madd_epi16(xmm[3][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][4], cmm[14]), _mm_madd_epi16(xmm[3][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[1]), _mm_madd_epi16(xmm[3][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][8], cmm[10]), _mm_madd_epi16(xmm[3][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][10], cmm[5]), _mm_madd_epi16(xmm[3][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][12], cmm[6]), _mm_madd_epi16(xmm[3][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][14], cmm[9]), _mm_madd_epi16(xmm[3][15], cmm[2])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[11]), _mm_madd_epi16(xmm[4][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[0]), _mm_madd_epi16(xmm[4][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], cmm[14]), _mm_madd_epi16(xmm[4][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[1]), _mm_madd_epi16(xmm[4][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][8], cmm[10]), _mm_madd_epi16(xmm[4][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][10], cmm[5]), _mm_madd_epi16(xmm[4][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][12], cmm[6]), _mm_madd_epi16(xmm[4][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][14], cmm[9]), _mm_madd_epi16(xmm[4][15], cmm[2])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[11]), _mm_madd_epi16(xmm[5][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[0]), _mm_madd_epi16(xmm[5][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][4], cmm[14]), _mm_madd_epi16(xmm[5][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[1]), _mm_madd_epi16(xmm[5][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][8], cmm[10]), _mm_madd_epi16(xmm[5][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][10], cmm[5]), _mm_madd_epi16(xmm[5][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][12], cmm[6]), _mm_madd_epi16(xmm[5][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][14], cmm[9]), _mm_madd_epi16(xmm[5][15], cmm[2])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[11]), _mm_madd_epi16(xmm[6][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[0]), _mm_madd_epi16(xmm[6][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][4], cmm[14]), _mm_madd_epi16(xmm[6][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[1]), _mm_madd_epi16(xmm[6][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][8], cmm[10]), _mm_madd_epi16(xmm[6][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][10], cmm[5]), _mm_madd_epi16(xmm[6][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][12], cmm[6]), _mm_madd_epi16(xmm[6][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][14], cmm[9]), _mm_madd_epi16(xmm[6][15], cmm[2])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[11]), _mm_madd_epi16(xmm[7][1], cmm[4])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[0]), _mm_madd_epi16(xmm[7][3], cmm[7])));
  tmm[1] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][4], cmm[14]), _mm_madd_epi16(xmm[7][5], cmm[8])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[1]), _mm_madd_epi16(xmm[7][7], cmm[3])));
  tmm[2] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][8], cmm[10]), _mm_madd_epi16(xmm[7][9], cmm[12])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][10], cmm[5]), _mm_madd_epi16(xmm[7][11], cmm[0])));
  tmm[3] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][12], cmm[6]), _mm_madd_epi16(xmm[7][13], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][14], cmm[9]), _mm_madd_epi16(xmm[7][15], cmm[2])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (25 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (25 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (25 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (25 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[27*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[12]), _mm_madd_epi16(xmm[0][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[2]), _mm_madd_epi16(xmm[0][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[5]), _mm_madd_epi16(xmm[0][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[14]), _mm_madd_epi16(xmm[0][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][8], cmm[4]), _mm_madd_epi16(xmm[0][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][10], cmm[3]), _mm_madd_epi16(xmm[0][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][12], cmm[13]), _mm_madd_epi16(xmm[0][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][14], cmm[6]), _mm_madd_epi16(xmm[0][15], cmm[1])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[12]), _mm_madd_epi16(xmm[1][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[2]), _mm_madd_epi16(xmm[1][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[5]), _mm_madd_epi16(xmm[1][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[14]), _mm_madd_epi16(xmm[1][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][8], cmm[4]), _mm_madd_epi16(xmm[1][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][10], cmm[3]), _mm_madd_epi16(xmm[1][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[1][12], cmm[13]), _mm_madd_epi16(xmm[1][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][14], cmm[6]), _mm_madd_epi16(xmm[1][15], cmm[1])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[12]), _mm_madd_epi16(xmm[2][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[2]), _mm_madd_epi16(xmm[2][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[5]), _mm_madd_epi16(xmm[2][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[14]), _mm_madd_epi16(xmm[2][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][8], cmm[4]), _mm_madd_epi16(xmm[2][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][10], cmm[3]), _mm_madd_epi16(xmm[2][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[2][12], cmm[13]), _mm_madd_epi16(xmm[2][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][14], cmm[6]), _mm_madd_epi16(xmm[2][15], cmm[1])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[12]), _mm_madd_epi16(xmm[3][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[2]), _mm_madd_epi16(xmm[3][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[5]), _mm_madd_epi16(xmm[3][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[14]), _mm_madd_epi16(xmm[3][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][8], cmm[4]), _mm_madd_epi16(xmm[3][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][10], cmm[3]), _mm_madd_epi16(xmm[3][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[3][12], cmm[13]), _mm_madd_epi16(xmm[3][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][14], cmm[6]), _mm_madd_epi16(xmm[3][15], cmm[1])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[12]), _mm_madd_epi16(xmm[4][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[2]), _mm_madd_epi16(xmm[4][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[5]), _mm_madd_epi16(xmm[4][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[14]), _mm_madd_epi16(xmm[4][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][8], cmm[4]), _mm_madd_epi16(xmm[4][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][10], cmm[3]), _mm_madd_epi16(xmm[4][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][12], cmm[13]), _mm_madd_epi16(xmm[4][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][14], cmm[6]), _mm_madd_epi16(xmm[4][15], cmm[1])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[12]), _mm_madd_epi16(xmm[5][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[2]), _mm_madd_epi16(xmm[5][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[5]), _mm_madd_epi16(xmm[5][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[14]), _mm_madd_epi16(xmm[5][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][8], cmm[4]), _mm_madd_epi16(xmm[5][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][10], cmm[3]), _mm_madd_epi16(xmm[5][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[5][12], cmm[13]), _mm_madd_epi16(xmm[5][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][14], cmm[6]), _mm_madd_epi16(xmm[5][15], cmm[1])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[12]), _mm_madd_epi16(xmm[6][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[2]), _mm_madd_epi16(xmm[6][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[5]), _mm_madd_epi16(xmm[6][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[14]), _mm_madd_epi16(xmm[6][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][8], cmm[4]), _mm_madd_epi16(xmm[6][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][10], cmm[3]), _mm_madd_epi16(xmm[6][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[6][12], cmm[13]), _mm_madd_epi16(xmm[6][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][14], cmm[6]), _mm_madd_epi16(xmm[6][15], cmm[1])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[12]), _mm_madd_epi16(xmm[7][1], cmm[7])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[2]), _mm_madd_epi16(xmm[7][3], cmm[0])));
  tmm[1] = _mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[5]), _mm_madd_epi16(xmm[7][5], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[14]), _mm_madd_epi16(xmm[7][7], cmm[9])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][8], cmm[4]), _mm_madd_epi16(xmm[7][9], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][10], cmm[3]), _mm_madd_epi16(xmm[7][11], cmm[8])));
  tmm[3] = _mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[7][12], cmm[13]), _mm_madd_epi16(xmm[7][13], cmm[11])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][14], cmm[6]), _mm_madd_epi16(xmm[7][15], cmm[1])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (27 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (27 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (27 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (27 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[29*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[13]), _mm_madd_epi16(xmm[0][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[7]), _mm_madd_epi16(xmm[0][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[0][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[2]), _mm_madd_epi16(xmm[0][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][8], cmm[8]), _mm_madd_epi16(xmm[0][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[0][10], cmm[14]), _mm_madd_epi16(xmm[0][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][12], cmm[9]), _mm_madd_epi16(xmm[0][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][14], cmm[3]), _mm_madd_epi16(xmm[0][15], cmm[0])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[13]), _mm_madd_epi16(xmm[1][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[7]), _mm_madd_epi16(xmm[1][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[1]), _mm_madd_epi16(xmm[1][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[2]), _mm_madd_epi16(xmm[1][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][8], cmm[8]), _mm_madd_epi16(xmm[1][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[1][10], cmm[14]), _mm_madd_epi16(xmm[1][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][12], cmm[9]), _mm_madd_epi16(xmm[1][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][14], cmm[3]), _mm_madd_epi16(xmm[1][15], cmm[0])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[13]), _mm_madd_epi16(xmm[2][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[7]), _mm_madd_epi16(xmm[2][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[1]), _mm_madd_epi16(xmm[2][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[2]), _mm_madd_epi16(xmm[2][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][8], cmm[8]), _mm_madd_epi16(xmm[2][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[2][10], cmm[14]), _mm_madd_epi16(xmm[2][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][12], cmm[9]), _mm_madd_epi16(xmm[2][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][14], cmm[3]), _mm_madd_epi16(xmm[2][15], cmm[0])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[13]), _mm_madd_epi16(xmm[3][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[7]), _mm_madd_epi16(xmm[3][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[1]), _mm_madd_epi16(xmm[3][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[2]), _mm_madd_epi16(xmm[3][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][8], cmm[8]), _mm_madd_epi16(xmm[3][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[3][10], cmm[14]), _mm_madd_epi16(xmm[3][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][12], cmm[9]), _mm_madd_epi16(xmm[3][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][14], cmm[3]), _mm_madd_epi16(xmm[3][15], cmm[0])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[13]), _mm_madd_epi16(xmm[4][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[7]), _mm_madd_epi16(xmm[4][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[1]), _mm_madd_epi16(xmm[4][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[2]), _mm_madd_epi16(xmm[4][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][8], cmm[8]), _mm_madd_epi16(xmm[4][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[4][10], cmm[14]), _mm_madd_epi16(xmm[4][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][12], cmm[9]), _mm_madd_epi16(xmm[4][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][14], cmm[3]), _mm_madd_epi16(xmm[4][15], cmm[0])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[13]), _mm_madd_epi16(xmm[5][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[7]), _mm_madd_epi16(xmm[5][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[1]), _mm_madd_epi16(xmm[5][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[2]), _mm_madd_epi16(xmm[5][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][8], cmm[8]), _mm_madd_epi16(xmm[5][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[5][10], cmm[14]), _mm_madd_epi16(xmm[5][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][12], cmm[9]), _mm_madd_epi16(xmm[5][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][14], cmm[3]), _mm_madd_epi16(xmm[5][15], cmm[0])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[13]), _mm_madd_epi16(xmm[6][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[7]), _mm_madd_epi16(xmm[6][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[1]), _mm_madd_epi16(xmm[6][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[2]), _mm_madd_epi16(xmm[6][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][8], cmm[8]), _mm_madd_epi16(xmm[6][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[6][10], cmm[14]), _mm_madd_epi16(xmm[6][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][12], cmm[9]), _mm_madd_epi16(xmm[6][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][14], cmm[3]), _mm_madd_epi16(xmm[6][15], cmm[0])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[13]), _mm_madd_epi16(xmm[7][1], cmm[10])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[7]), _mm_madd_epi16(xmm[7][3], cmm[4])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[1]), _mm_madd_epi16(xmm[7][5], cmm[0])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[2]), _mm_madd_epi16(xmm[7][7], cmm[5])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][8], cmm[8]), _mm_madd_epi16(xmm[7][9], cmm[11])), _mm_add_epi32(_mm_madd_epi16(xmm[7][10], cmm[14]), _mm_madd_epi16(xmm[7][11], cmm[12])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][12], cmm[9]), _mm_madd_epi16(xmm[7][13], cmm[6])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][14], cmm[3]), _mm_madd_epi16(xmm[7][15], cmm[0])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_sub_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (29 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (29 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (29 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (29 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));

  // [JDS]:  dst[31*line]
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[14]), _mm_madd_epi16(xmm[0][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[12]), _mm_madd_epi16(xmm[0][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[10]), _mm_madd_epi16(xmm[0][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[8]), _mm_madd_epi16(xmm[0][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][8], cmm[6]), _mm_madd_epi16(xmm[0][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][10], cmm[4]), _mm_madd_epi16(xmm[0][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][12], cmm[2]), _mm_madd_epi16(xmm[0][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[0][14], cmm[0]), _mm_madd_epi16(xmm[0][15], cmm[0])));
  rmm[0] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][0], cmm[14]), _mm_madd_epi16(xmm[1][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][2], cmm[12]), _mm_madd_epi16(xmm[1][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][4], cmm[10]), _mm_madd_epi16(xmm[1][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][6], cmm[8]), _mm_madd_epi16(xmm[1][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][8], cmm[6]), _mm_madd_epi16(xmm[1][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][10], cmm[4]), _mm_madd_epi16(xmm[1][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[1][12], cmm[2]), _mm_madd_epi16(xmm[1][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[1][14], cmm[0]), _mm_madd_epi16(xmm[1][15], cmm[0])));
  rmm[1] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][0], cmm[14]), _mm_madd_epi16(xmm[2][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][2], cmm[12]), _mm_madd_epi16(xmm[2][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][4], cmm[10]), _mm_madd_epi16(xmm[2][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][6], cmm[8]), _mm_madd_epi16(xmm[2][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][8], cmm[6]), _mm_madd_epi16(xmm[2][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][10], cmm[4]), _mm_madd_epi16(xmm[2][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[2][12], cmm[2]), _mm_madd_epi16(xmm[2][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[2][14], cmm[0]), _mm_madd_epi16(xmm[2][15], cmm[0])));
  rmm[2] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][0], cmm[14]), _mm_madd_epi16(xmm[3][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][2], cmm[12]), _mm_madd_epi16(xmm[3][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][4], cmm[10]), _mm_madd_epi16(xmm[3][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][6], cmm[8]), _mm_madd_epi16(xmm[3][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][8], cmm[6]), _mm_madd_epi16(xmm[3][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][10], cmm[4]), _mm_madd_epi16(xmm[3][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[3][12], cmm[2]), _mm_madd_epi16(xmm[3][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[3][14], cmm[0]), _mm_madd_epi16(xmm[3][15], cmm[0])));
  rmm[3] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][0], cmm[14]), _mm_madd_epi16(xmm[4][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][2], cmm[12]), _mm_madd_epi16(xmm[4][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][4], cmm[10]), _mm_madd_epi16(xmm[4][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][6], cmm[8]), _mm_madd_epi16(xmm[4][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][8], cmm[6]), _mm_madd_epi16(xmm[4][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][10], cmm[4]), _mm_madd_epi16(xmm[4][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[4][12], cmm[2]), _mm_madd_epi16(xmm[4][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[4][14], cmm[0]), _mm_madd_epi16(xmm[4][15], cmm[0])));
  rmm[4] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][0], cmm[14]), _mm_madd_epi16(xmm[5][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][2], cmm[12]), _mm_madd_epi16(xmm[5][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][4], cmm[10]), _mm_madd_epi16(xmm[5][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][6], cmm[8]), _mm_madd_epi16(xmm[5][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][8], cmm[6]), _mm_madd_epi16(xmm[5][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][10], cmm[4]), _mm_madd_epi16(xmm[5][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[5][12], cmm[2]), _mm_madd_epi16(xmm[5][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[5][14], cmm[0]), _mm_madd_epi16(xmm[5][15], cmm[0])));
  rmm[5] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][0], cmm[14]), _mm_madd_epi16(xmm[6][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][2], cmm[12]), _mm_madd_epi16(xmm[6][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][4], cmm[10]), _mm_madd_epi16(xmm[6][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][6], cmm[8]), _mm_madd_epi16(xmm[6][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][8], cmm[6]), _mm_madd_epi16(xmm[6][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][10], cmm[4]), _mm_madd_epi16(xmm[6][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[6][12], cmm[2]), _mm_madd_epi16(xmm[6][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[6][14], cmm[0]), _mm_madd_epi16(xmm[6][15], cmm[0])));
  rmm[6] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  tmm[0] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][0], cmm[14]), _mm_madd_epi16(xmm[7][1], cmm[13])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][2], cmm[12]), _mm_madd_epi16(xmm[7][3], cmm[11])));
  tmm[1] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][4], cmm[10]), _mm_madd_epi16(xmm[7][5], cmm[9])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][6], cmm[8]), _mm_madd_epi16(xmm[7][7], cmm[7])));
  tmm[2] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][8], cmm[6]), _mm_madd_epi16(xmm[7][9], cmm[5])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][10], cmm[4]), _mm_madd_epi16(xmm[7][11], cmm[3])));
  tmm[3] = _mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[7][12], cmm[2]), _mm_madd_epi16(xmm[7][13], cmm[1])), _mm_sub_epi32(_mm_madd_epi16(xmm[7][14], cmm[0]), _mm_madd_epi16(xmm[7][15], cmm[0])));
  rmm[7] = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(_mm_add_epi32(tmm[0], tmm[1]), _mm_add_epi32(tmm[2], tmm[3])), ADD), shift);
  _mm_storeu_si128((__m128i*)(dst + (31 * line)), _mm_packs_epi32(rmm[0], rmm[1]));   _mm_storeu_si128((__m128i*)(dst + (31 * line + 8)), _mm_packs_epi32(rmm[2], rmm[3]));
  _mm_storeu_si128((__m128i*)(dst + (31 * line + 16)), _mm_packs_epi32(rmm[4], rmm[5]));   _mm_storeu_si128((__m128i*)(dst + (31 * line + 24)), _mm_packs_epi32(rmm[6], rmm[7]));
#else

Int j, k;
Int E[16], O[16];
Int EE[8], EO[8];
Int EEE[4], EEO[4];
Int EEEE[2], EEEO[2];

  for (j=0; j<line; j++)
  {    
    /* E and O*/
    for (k=0;k<16;k++)
    {
      E[k] = src[k] + src[31-k];
      O[k] = src[k] - src[31-k];
    } 
    /* EE and EO */
    for (k=0;k<8;k++)
    {
      EE[k] = E[k] + E[15-k];
      EO[k] = E[k] - E[15-k];
    }
    /* EEE and EEO */
    for (k=0;k<4;k++)
    {
      EEE[k] = EE[k] + EE[7-k];
      EEO[k] = EE[k] - EE[7-k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];    
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    dst[ 0       ] = (g_aiT32[ 0][0]*EEEE[0] + g_aiT32[ 0][1]*EEEE[1] + add)>>shift;
    dst[ 16*line ] = (g_aiT32[16][0]*EEEE[0] + g_aiT32[16][1]*EEEE[1] + add)>>shift;
    dst[ 8*line  ] = (g_aiT32[ 8][0]*EEEO[0] + g_aiT32[ 8][1]*EEEO[1] + add)>>shift; 
    dst[ 24*line ] = (g_aiT32[24][0]*EEEO[0] + g_aiT32[24][1]*EEEO[1] + add)>>shift;
    for (k=4;k<32;k+=8)
    {
      dst[ k*line ] = (g_aiT32[k][0]*EEO[0] + g_aiT32[k][1]*EEO[1] + g_aiT32[k][2]*EEO[2] + g_aiT32[k][3]*EEO[3] + add)>>shift;
    }       
    for (k=2;k<32;k+=4)
    {
      dst[ k*line ] = (g_aiT32[k][0]*EO[0] + g_aiT32[k][1]*EO[1] + g_aiT32[k][2]*EO[2] + g_aiT32[k][3]*EO[3] + 
        g_aiT32[k][4]*EO[4] + g_aiT32[k][5]*EO[5] + g_aiT32[k][6]*EO[6] + g_aiT32[k][7]*EO[7] + add)>>shift;
    }       
    for (k=1;k<32;k+=2)
    {
      dst[ k*line ] = (g_aiT32[k][ 0]*O[ 0] + g_aiT32[k][ 1]*O[ 1] + g_aiT32[k][ 2]*O[ 2] + g_aiT32[k][ 3]*O[ 3] + 
        g_aiT32[k][ 4]*O[ 4] + g_aiT32[k][ 5]*O[ 5] + g_aiT32[k][ 6]*O[ 6] + g_aiT32[k][ 7]*O[ 7] +
        g_aiT32[k][ 8]*O[ 8] + g_aiT32[k][ 9]*O[ 9] + g_aiT32[k][10]*O[10] + g_aiT32[k][11]*O[11] + 
        g_aiT32[k][12]*O[12] + g_aiT32[k][13]*O[13] + g_aiT32[k][14]*O[14] + g_aiT32[k][15]*O[15] + add)>>shift;
    }
    src += 32;
    dst ++;
  }
#endif // #if ETRI_SIMD_TR
}


void partialButterflyInverse32(Short *src,Short *dst,Int shift, Int line)
{
  Int add = 1<<(shift-1);

#if ETRI_SIMD_TR
  // [JDS]: Variables for SIMD code
  __m128i xmm[8][8], omm[16][8], emm[16][8], cmm[16], wmm[8];
  short *Tmp1 = src + 8 * 32; short *Tmp2 = src + 8 * 64; short *Tmp3 = src + 8 * 96;

  // [JDS]: 32x16 data load and alignment for O[k] 
  omm[0][0] = _mm_loadu_si128((__m128i*)(src + 8 * 4));    omm[0][1] = _mm_loadu_si128((__m128i*)(src + 8 * 5));   omm[0][2] = _mm_loadu_si128((__m128i*)(src + 8 * 6));    omm[0][3] = _mm_loadu_si128((__m128i*)(src + 8 * 7));
  omm[0][4] = _mm_loadu_si128((__m128i*)(src + 8 * 12));    omm[0][5] = _mm_loadu_si128((__m128i*)(src + 8 * 13));  omm[0][6] = _mm_loadu_si128((__m128i*)(src + 8 * 14));    omm[0][7] = _mm_loadu_si128((__m128i*)(src + 8 * 15));
  omm[1][0] = _mm_loadu_si128((__m128i*)(src + 8 * 20));    omm[1][1] = _mm_loadu_si128((__m128i*)(src + 8 * 21));  omm[1][2] = _mm_loadu_si128((__m128i*)(src + 8 * 22));    omm[1][3] = _mm_loadu_si128((__m128i*)(src + 8 * 23));
  omm[1][4] = _mm_loadu_si128((__m128i*)(src + 8 * 28));    omm[1][5] = _mm_loadu_si128((__m128i*)(src + 8 * 29));  omm[1][6] = _mm_loadu_si128((__m128i*)(src + 8 * 30));    omm[1][7] = _mm_loadu_si128((__m128i*)(src + 8 * 31));
  omm[2][0] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 4));    omm[2][1] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 5));  omm[2][2] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 6));    omm[2][3] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 7));
  omm[2][4] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 12));   omm[2][5] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 13));  omm[2][6] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 14));   omm[2][7] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 15));
  omm[3][0] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 20));   omm[3][1] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 21));  omm[3][2] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 22));   omm[3][3] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 23));
  omm[3][4] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 28));   omm[3][5] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 29));  omm[3][6] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 30));   omm[3][7] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 31));
  omm[4][0] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 4));    omm[4][1] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 5));  omm[4][2] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 6));    omm[4][3] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 7));
  omm[4][4] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 12));   omm[4][5] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 13));  omm[4][6] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 14));   omm[4][7] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 15));
  omm[5][0] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 20));   omm[5][1] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 21));  omm[5][2] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 22));   omm[5][3] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 23));
  omm[5][4] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 28));   omm[5][5] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 29));  omm[5][6] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 30));   omm[5][7] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 31));
  omm[6][0] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 4));    omm[6][1] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 5));  omm[6][2] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 6));    omm[6][3] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 7));
  omm[6][4] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 12));   omm[6][5] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 13));  omm[6][6] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 14));   omm[6][7] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 15));
  omm[7][0] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 20));   omm[7][1] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 21));  omm[7][2] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 22));   omm[7][3] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 23));
  omm[7][4] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 28));   omm[7][5] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 29));  omm[7][6] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 30));   omm[7][7] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 31));
  xmm[0][0] = _mm_unpacklo_epi16(omm[0][0], omm[0][4]); xmm[0][1] = _mm_unpackhi_epi16(omm[0][0], omm[0][4]); xmm[0][2] = _mm_unpacklo_epi16(omm[0][1], omm[0][5]); xmm[0][3] = _mm_unpackhi_epi16(omm[0][1], omm[0][5]);
  xmm[0][4] = _mm_unpacklo_epi16(omm[0][2], omm[0][6]); xmm[0][5] = _mm_unpackhi_epi16(omm[0][2], omm[0][6]); xmm[0][6] = _mm_unpacklo_epi16(omm[0][3], omm[0][7]); xmm[0][7] = _mm_unpackhi_epi16(omm[0][3], omm[0][7]);
  xmm[1][0] = _mm_unpacklo_epi16(omm[1][0], omm[1][4]); xmm[1][1] = _mm_unpackhi_epi16(omm[1][0], omm[1][4]); xmm[1][2] = _mm_unpacklo_epi16(omm[1][1], omm[1][5]); xmm[1][3] = _mm_unpackhi_epi16(omm[1][1], omm[1][5]);
  xmm[1][4] = _mm_unpacklo_epi16(omm[1][2], omm[1][6]); xmm[1][5] = _mm_unpackhi_epi16(omm[1][2], omm[1][6]); xmm[1][6] = _mm_unpacklo_epi16(omm[1][3], omm[1][7]); xmm[1][7] = _mm_unpackhi_epi16(omm[1][3], omm[1][7]);
  xmm[2][0] = _mm_unpacklo_epi16(omm[2][0], omm[2][4]); xmm[2][1] = _mm_unpackhi_epi16(omm[2][0], omm[2][4]); xmm[2][2] = _mm_unpacklo_epi16(omm[2][1], omm[2][5]); xmm[2][3] = _mm_unpackhi_epi16(omm[2][1], omm[2][5]);
  xmm[2][4] = _mm_unpacklo_epi16(omm[2][2], omm[2][6]); xmm[2][5] = _mm_unpackhi_epi16(omm[2][2], omm[2][6]); xmm[2][6] = _mm_unpacklo_epi16(omm[2][3], omm[2][7]); xmm[2][7] = _mm_unpackhi_epi16(omm[2][3], omm[2][7]);
  xmm[3][0] = _mm_unpacklo_epi16(omm[3][0], omm[3][4]); xmm[3][1] = _mm_unpackhi_epi16(omm[3][0], omm[3][4]); xmm[3][2] = _mm_unpacklo_epi16(omm[3][1], omm[3][5]); xmm[3][3] = _mm_unpackhi_epi16(omm[3][1], omm[3][5]);
  xmm[3][4] = _mm_unpacklo_epi16(omm[3][2], omm[3][6]); xmm[3][5] = _mm_unpackhi_epi16(omm[3][2], omm[3][6]); xmm[3][6] = _mm_unpacklo_epi16(omm[3][3], omm[3][7]); xmm[3][7] = _mm_unpackhi_epi16(omm[3][3], omm[3][7]);
  xmm[4][0] = _mm_unpacklo_epi16(omm[4][0], omm[4][4]); xmm[4][1] = _mm_unpackhi_epi16(omm[4][0], omm[4][4]); xmm[4][2] = _mm_unpacklo_epi16(omm[4][1], omm[4][5]); xmm[4][3] = _mm_unpackhi_epi16(omm[4][1], omm[4][5]);
  xmm[4][4] = _mm_unpacklo_epi16(omm[4][2], omm[4][6]); xmm[4][5] = _mm_unpackhi_epi16(omm[4][2], omm[4][6]); xmm[4][6] = _mm_unpacklo_epi16(omm[4][3], omm[4][7]); xmm[4][7] = _mm_unpackhi_epi16(omm[4][3], omm[4][7]);
  xmm[5][0] = _mm_unpacklo_epi16(omm[5][0], omm[5][4]); xmm[5][1] = _mm_unpackhi_epi16(omm[5][0], omm[5][4]); xmm[5][2] = _mm_unpacklo_epi16(omm[5][1], omm[5][5]); xmm[5][3] = _mm_unpackhi_epi16(omm[5][1], omm[5][5]);
  xmm[5][4] = _mm_unpacklo_epi16(omm[5][2], omm[5][6]); xmm[5][5] = _mm_unpackhi_epi16(omm[5][2], omm[5][6]); xmm[5][6] = _mm_unpacklo_epi16(omm[5][3], omm[5][7]); xmm[5][7] = _mm_unpackhi_epi16(omm[5][3], omm[5][7]);
  xmm[6][0] = _mm_unpacklo_epi16(omm[6][0], omm[6][4]); xmm[6][1] = _mm_unpackhi_epi16(omm[6][0], omm[6][4]); xmm[6][2] = _mm_unpacklo_epi16(omm[6][1], omm[6][5]); xmm[6][3] = _mm_unpackhi_epi16(omm[6][1], omm[6][5]);
  xmm[6][4] = _mm_unpacklo_epi16(omm[6][2], omm[6][6]); xmm[6][5] = _mm_unpackhi_epi16(omm[6][2], omm[6][6]); xmm[6][6] = _mm_unpacklo_epi16(omm[6][3], omm[6][7]); xmm[6][7] = _mm_unpackhi_epi16(omm[6][3], omm[6][7]);
  xmm[7][0] = _mm_unpacklo_epi16(omm[7][0], omm[7][4]); xmm[7][1] = _mm_unpackhi_epi16(omm[7][0], omm[7][4]); xmm[7][2] = _mm_unpacklo_epi16(omm[7][1], omm[7][5]); xmm[7][3] = _mm_unpackhi_epi16(omm[7][1], omm[7][5]);
  xmm[7][4] = _mm_unpacklo_epi16(omm[7][2], omm[7][6]); xmm[7][5] = _mm_unpackhi_epi16(omm[7][2], omm[7][6]); xmm[7][6] = _mm_unpacklo_epi16(omm[7][3], omm[7][7]); xmm[7][7] = _mm_unpackhi_epi16(omm[7][3], omm[7][7]);

  // [JDS]: Set the constants and compute for O[k]
  cmm[0] = _mm_set1_epi16(90); cmm[1] = _mm_set1_epi16(88); cmm[2] = _mm_set1_epi16(85); cmm[3] = _mm_set1_epi16(82); cmm[4] = _mm_set1_epi16(78); cmm[5] = _mm_set1_epi16(73); cmm[6] = _mm_set1_epi16(67);
  cmm[7] = _mm_set1_epi16(61); cmm[8] = _mm_set1_epi16(54); cmm[9] = _mm_set1_epi16(46); cmm[10] = _mm_set1_epi16(38); cmm[11] = _mm_set1_epi16(31); cmm[12] = _mm_set1_epi16(22); cmm[13] = _mm_set1_epi16(13);
  cmm[14] = _mm_set1_epi16(4); wmm[1] = _mm_unpacklo_epi16(cmm[1], cmm[2]); wmm[2] = _mm_unpacklo_epi16(cmm[3], cmm[4]);  wmm[3] = _mm_unpacklo_epi16(cmm[5], cmm[6]); wmm[4] = _mm_unpacklo_epi16(cmm[7], cmm[8]);
  wmm[5] = _mm_unpacklo_epi16(cmm[9], cmm[10]); wmm[6] = _mm_unpacklo_epi16(cmm[11], cmm[12]); wmm[7] = _mm_unpacklo_epi16(cmm[13], cmm[14]);
  omm[0][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[0][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[0][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[0][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[0][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[0][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[0][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[0][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[0], cmm[3]); wmm[1] = _mm_unpacklo_epi16(cmm[6], cmm[9]); wmm[2] = _mm_unpacklo_epi16(cmm[12], _mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])));
  wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])), _mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8])));
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])), _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])));
  wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])), _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])));
  wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10])), _mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])));
  omm[1][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[1][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[1][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[1][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[1][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[1][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[1][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[1][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[1], cmm[6]); wmm[1] = _mm_unpacklo_epi16(cmm[11], _mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])));
  wmm[2] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8])), _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));
  wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), _mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])));
  wmm[5] = _mm_unpacklo_epi16(cmm[10], cmm[5]); wmm[6] = _mm_unpacklo_epi16(cmm[0], cmm[2]);  wmm[7] = _mm_unpacklo_epi16(cmm[7], cmm[12]);
  omm[2][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[2][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[2][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[2][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[2][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[2][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[2][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[2][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[2], cmm[9]);
  wmm[1] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])), _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])));
  wmm[2] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));
  wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12])), cmm[10]);
  wmm[4] = _mm_unpacklo_epi16(cmm[3], cmm[1]); wmm[5] = _mm_unpacklo_epi16(cmm[8], _mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])));
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])), _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])), _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  omm[3][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[3][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[3][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[3][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[3][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[3][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[3][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[3][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[3], cmm[12]); wmm[1] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8])), _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  wmm[2] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])), cmm[13]); wmm[3] = _mm_unpacklo_epi16(cmm[4], cmm[2]);
  wmm[4] = _mm_unpacklo_epi16(cmm[11], _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])));
  wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])));
  wmm[6] = _mm_unpacklo_epi16(cmm[14], cmm[5]); wmm[7] = _mm_unpacklo_epi16(cmm[1], cmm[10]);
  omm[4][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[4][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[4][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[4][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[4][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[4][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[4][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[4][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[4], _mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])));
  wmm[1] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])), _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));
  wmm[2] = _mm_unpacklo_epi16(cmm[13], cmm[2]); wmm[3] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12])));
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])), _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])));
  wmm[5] = _mm_unpacklo_epi16(cmm[11], cmm[0]); wmm[6] = _mm_unpacklo_epi16(cmm[8], _mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10])));
  wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])));
  omm[5][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[5][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[5][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[5][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[5][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[5][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[5][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[5][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  wmm[1] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), _mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12])));
  wmm[2] = _mm_unpacklo_epi16(cmm[4], cmm[6]); wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10])), _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])), cmm[3]); wmm[5] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])));
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])), _mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14]))); wmm[7] = _mm_unpacklo_epi16(cmm[2], cmm[8]);
  omm[6][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[6][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[6][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[6][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[6][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[6][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[6][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[6][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8])));  wmm[1] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])), cmm[10]);
  wmm[2] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12]))); wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), cmm[14]);
  wmm[4] = _mm_unpacklo_epi16(cmm[0], cmm[13]); wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])), _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  wmm[6] = _mm_unpacklo_epi16(cmm[3], cmm[9]);  wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])), _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])));
  omm[7][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[7][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[7][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[7][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[7][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[7][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[7][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[7][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5]))); wmm[1] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), cmm[3]);
  wmm[2] = _mm_unpacklo_epi16(cmm[11], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1]))); wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])), cmm[0]);
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])), _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0]))); wmm[5] = _mm_unpacklo_epi16(cmm[12], cmm[2]);
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10])), _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4]))); wmm[7] = _mm_unpacklo_epi16(cmm[8], cmm[6]);
  omm[8][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[8][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[8][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[8][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[8][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[8][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[8][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[8][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[8], _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2]))); wmm[1] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])), cmm[1]);
  wmm[2] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7]))); wmm[3] = _mm_unpacklo_epi16(cmm[3], cmm[13]);
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), cmm[10]); wmm[5] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12])), cmm[0]);
  wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])), _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));
  omm[9][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[9][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[9][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[9][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[9][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[9][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[9][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[9][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[9], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0]))); wmm[1] = _mm_unpacklo_epi16(cmm[10], cmm[8]);
  wmm[2] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), cmm[11]);  wmm[3] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  wmm[4] = _mm_unpacklo_epi16(cmm[12], cmm[6]); wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])), cmm[13]);
  wmm[6] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3]))); wmm[7] = _mm_unpacklo_epi16(cmm[14], cmm[4]);
  omm[10][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[10][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[10][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[10][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[10][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[10][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[10][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[10][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[10], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1]))); wmm[1] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])));
  wmm[2] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])), cmm[0]);
  wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  wmm[4] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4]))); wmm[5] = _mm_unpacklo_epi16(cmm[13], cmm[7]);
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])), cmm[8]);   wmm[7] = _mm_unpacklo_epi16(cmm[12], _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));
  omm[11][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[11][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[11][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[11][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[11][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[11][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[11][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[11][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[11], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4]))); wmm[1] = _mm_unpacklo_epi16(cmm[0], _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])));
  wmm[2] = _mm_unpacklo_epi16(cmm[14], cmm[8]); wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])), cmm[3]);
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10])), _mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12])));
  wmm[5] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0]))); wmm[6] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])));
  wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), cmm[2]);
  omm[12][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[12][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[12][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[12][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[12][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[12][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[12][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[12][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[12], _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])));  wmm[1] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  wmm[2] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10]))); wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])), cmm[9]);
  wmm[4] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])), cmm[0]);  wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])), cmm[8]);
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])), _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  wmm[7] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  omm[13][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[13][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[13][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[13][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[13][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[13][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[13][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[13][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[13], _mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10]))); wmm[1] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  wmm[2] = _mm_unpacklo_epi16(cmm[1], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));  wmm[3] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));
  wmm[4] = _mm_unpacklo_epi16(cmm[8], _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11]))); wmm[5] = _mm_unpacklo_epi16(cmm[14], cmm[12]);
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), cmm[6]);   wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])), cmm[0]);
  omm[14][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[14][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[14][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[14][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[14][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[14][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[14][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[14][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[14], _mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13]))); wmm[1] = _mm_unpacklo_epi16(cmm[12], _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  wmm[2] = _mm_unpacklo_epi16(cmm[10], _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])));  wmm[3] = _mm_unpacklo_epi16(cmm[8], _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])));
  wmm[4] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[5], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[5])));  wmm[5] = _mm_unpacklo_epi16(cmm[4], _mm_mullo_epi16(cmm[3], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[3])));
  wmm[6] = _mm_unpacklo_epi16(cmm[2], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));  wmm[7] = _mm_unpacklo_epi16(cmm[0], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  omm[15][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  omm[15][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  omm[15][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  omm[15][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  omm[15][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  omm[15][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  omm[15][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  omm[15][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  // [JDS]: 32x16 data load and alignment for E[k] 
  emm[0][0] = _mm_loadu_si128((__m128i*)(src));      emm[0][1] = _mm_loadu_si128((__m128i*)(src + 8));    emm[0][2] = _mm_loadu_si128((__m128i*)(src + 8 * 2));    emm[0][3] = _mm_loadu_si128((__m128i*)(src + 8 * 3));
  emm[0][4] = _mm_loadu_si128((__m128i*)(src + 8 * 8));    emm[0][5] = _mm_loadu_si128((__m128i*)(src + 8 * 9));   emm[0][6] = _mm_loadu_si128((__m128i*)(src + 8 * 10));    emm[0][7] = _mm_loadu_si128((__m128i*)(src + 8 * 11));
  emm[1][0] = _mm_loadu_si128((__m128i*)(src + 8 * 16));    emm[1][1] = _mm_loadu_si128((__m128i*)(src + 8 * 17));  emm[1][2] = _mm_loadu_si128((__m128i*)(src + 8 * 18));    emm[1][3] = _mm_loadu_si128((__m128i*)(src + 8 * 19));
  emm[1][4] = _mm_loadu_si128((__m128i*)(src + 8 * 24));    emm[1][5] = _mm_loadu_si128((__m128i*)(src + 8 * 25));  emm[1][6] = _mm_loadu_si128((__m128i*)(src + 8 * 26));    emm[1][7] = _mm_loadu_si128((__m128i*)(src + 8 * 27));
  emm[2][0] = _mm_loadu_si128((__m128i*)(Tmp1));      emm[2][1] = _mm_loadu_si128((__m128i*)(Tmp1 + 8));    emm[2][2] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 2));    emm[2][3] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 3));
  emm[2][4] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 8));    emm[2][5] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 9));  emm[2][6] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 10));   emm[2][7] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 11));
  emm[3][0] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 16));   emm[3][1] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 17));  emm[3][2] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 18));   emm[3][3] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 19));
  emm[3][4] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 24));   emm[3][5] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 25));  emm[3][6] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 26));   emm[3][7] = _mm_loadu_si128((__m128i*)(Tmp1 + 8 * 27));
  emm[4][0] = _mm_loadu_si128((__m128i*)(Tmp2));      emm[4][1] = _mm_loadu_si128((__m128i*)(Tmp2 + 8));    emm[4][2] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 2));    emm[4][3] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 3));
  emm[4][4] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 8));    emm[4][5] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 9));  emm[4][6] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 10));   emm[4][7] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 11));
  emm[5][0] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 16));   emm[5][1] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 17));  emm[5][2] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 18));   emm[5][3] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 19));
  emm[5][4] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 24));   emm[5][5] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 25));  emm[5][6] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 26));   emm[5][7] = _mm_loadu_si128((__m128i*)(Tmp2 + 8 * 27));
  emm[6][0] = _mm_loadu_si128((__m128i*)(Tmp3));      emm[6][1] = _mm_loadu_si128((__m128i*)(Tmp3 + 8));    emm[6][2] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 2));    emm[6][3] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 3));
  emm[6][4] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 8));    emm[6][5] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 9));  emm[6][6] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 10));   emm[6][7] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 11));
  emm[7][0] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 16));   emm[7][1] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 17));  emm[7][2] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 18));   emm[7][3] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 19));
  emm[7][4] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 24));   emm[7][5] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 25));  emm[7][6] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 26));   emm[7][7] = _mm_loadu_si128((__m128i*)(Tmp3 + 8 * 27));
  xmm[0][0] = _mm_unpacklo_epi16(emm[0][0], emm[4][0]); xmm[0][1] = _mm_unpackhi_epi16(emm[0][0], emm[4][0]); xmm[0][2] = _mm_unpacklo_epi16(emm[0][1], emm[4][1]); xmm[0][3] = _mm_unpackhi_epi16(emm[0][1], emm[4][1]);
  xmm[0][4] = _mm_unpacklo_epi16(emm[0][2], emm[4][2]); xmm[0][5] = _mm_unpackhi_epi16(emm[0][2], emm[4][2]); xmm[0][6] = _mm_unpacklo_epi16(emm[0][3], emm[4][3]); xmm[0][7] = _mm_unpackhi_epi16(emm[0][3], emm[4][3]);
  xmm[1][0] = _mm_unpacklo_epi16(emm[2][0], emm[6][0]); xmm[1][1] = _mm_unpackhi_epi16(emm[2][0], emm[6][0]); xmm[1][2] = _mm_unpacklo_epi16(emm[2][1], emm[6][1]); xmm[1][3] = _mm_unpackhi_epi16(emm[2][1], emm[6][1]);
  xmm[1][4] = _mm_unpacklo_epi16(emm[2][2], emm[6][2]); xmm[1][5] = _mm_unpackhi_epi16(emm[2][2], emm[6][2]); xmm[1][6] = _mm_unpacklo_epi16(emm[2][3], emm[6][3]); xmm[1][7] = _mm_unpackhi_epi16(emm[2][3], emm[6][3]);
  xmm[2][0] = _mm_unpacklo_epi16(emm[1][0], emm[3][0]); xmm[2][1] = _mm_unpackhi_epi16(emm[1][0], emm[3][0]); xmm[2][2] = _mm_unpacklo_epi16(emm[1][1], emm[3][1]); xmm[2][3] = _mm_unpackhi_epi16(emm[1][1], emm[3][1]);
  xmm[2][4] = _mm_unpacklo_epi16(emm[1][2], emm[3][2]); xmm[2][5] = _mm_unpackhi_epi16(emm[1][2], emm[3][2]); xmm[2][6] = _mm_unpacklo_epi16(emm[1][3], emm[3][3]); xmm[2][7] = _mm_unpackhi_epi16(emm[1][3], emm[3][3]);
  xmm[3][0] = _mm_unpacklo_epi16(emm[5][0], emm[7][0]); xmm[3][1] = _mm_unpackhi_epi16(emm[5][0], emm[7][0]); xmm[3][2] = _mm_unpacklo_epi16(emm[5][1], emm[7][1]); xmm[3][3] = _mm_unpackhi_epi16(emm[5][1], emm[7][1]);
  xmm[3][4] = _mm_unpacklo_epi16(emm[5][2], emm[7][2]); xmm[3][5] = _mm_unpackhi_epi16(emm[5][2], emm[7][2]); xmm[3][6] = _mm_unpacklo_epi16(emm[5][3], emm[7][3]); xmm[3][7] = _mm_unpackhi_epi16(emm[5][3], emm[7][3]);
  xmm[4][0] = _mm_unpacklo_epi16(emm[0][4], emm[1][4]); xmm[4][1] = _mm_unpackhi_epi16(emm[0][4], emm[1][4]); xmm[4][2] = _mm_unpacklo_epi16(emm[0][5], emm[1][5]); xmm[4][3] = _mm_unpackhi_epi16(emm[0][5], emm[1][5]);
  xmm[4][4] = _mm_unpacklo_epi16(emm[0][6], emm[1][6]); xmm[4][5] = _mm_unpackhi_epi16(emm[0][6], emm[1][6]); xmm[4][6] = _mm_unpacklo_epi16(emm[0][7], emm[1][7]); xmm[4][7] = _mm_unpackhi_epi16(emm[0][7], emm[1][7]);
  xmm[5][0] = _mm_unpacklo_epi16(emm[2][4], emm[3][4]); xmm[5][1] = _mm_unpackhi_epi16(emm[2][4], emm[3][4]); xmm[5][2] = _mm_unpacklo_epi16(emm[2][5], emm[3][5]); xmm[5][3] = _mm_unpackhi_epi16(emm[2][5], emm[3][5]);
  xmm[5][4] = _mm_unpacklo_epi16(emm[2][6], emm[3][6]); xmm[5][5] = _mm_unpackhi_epi16(emm[2][6], emm[3][6]); xmm[5][6] = _mm_unpacklo_epi16(emm[2][7], emm[3][7]); xmm[5][7] = _mm_unpackhi_epi16(emm[2][7], emm[3][7]);
  xmm[6][0] = _mm_unpacklo_epi16(emm[4][4], emm[5][4]); xmm[6][1] = _mm_unpackhi_epi16(emm[4][4], emm[5][4]); xmm[6][2] = _mm_unpacklo_epi16(emm[4][5], emm[5][5]); xmm[6][3] = _mm_unpackhi_epi16(emm[4][5], emm[5][5]);
  xmm[6][4] = _mm_unpacklo_epi16(emm[4][6], emm[5][6]); xmm[6][5] = _mm_unpackhi_epi16(emm[4][6], emm[5][6]); xmm[6][6] = _mm_unpacklo_epi16(emm[4][7], emm[5][7]); xmm[6][7] = _mm_unpackhi_epi16(emm[4][7], emm[5][7]);
  xmm[7][0] = _mm_unpacklo_epi16(emm[6][4], emm[7][4]); xmm[7][1] = _mm_unpackhi_epi16(emm[6][4], emm[7][4]); xmm[7][2] = _mm_unpacklo_epi16(emm[6][5], emm[7][5]); xmm[7][3] = _mm_unpackhi_epi16(emm[6][5], emm[7][5]);
  xmm[7][4] = _mm_unpacklo_epi16(emm[6][6], emm[7][6]); xmm[7][5] = _mm_unpackhi_epi16(emm[6][6], emm[7][6]); xmm[7][6] = _mm_unpacklo_epi16(emm[6][7], emm[7][7]); xmm[7][7] = _mm_unpackhi_epi16(emm[6][7], emm[7][7]);

  // [JDS]: Set the constants anc compute for E[k]
  cmm[1] = _mm_set1_epi16(64); cmm[2] = _mm_set1_epi16(83); cmm[3] = _mm_set1_epi16(36); cmm[4] = _mm_set1_epi16(89); cmm[5] = _mm_set1_epi16(75); cmm[6] = _mm_set1_epi16(50); cmm[7] = _mm_set1_epi16(18);
  cmm[8] = _mm_set1_epi16(87); cmm[9] = _mm_set1_epi16(80); cmm[10] = _mm_set1_epi16(70); cmm[11] = _mm_set1_epi16(57); cmm[12] = _mm_set1_epi16(43); cmm[13] = _mm_set1_epi16(25); cmm[14] = _mm_set1_epi16(9);
  wmm[1] = _mm_unpacklo_epi16(cmm[2], cmm[3]);  wmm[2] = _mm_unpacklo_epi16(cmm[4], cmm[5]);  wmm[3] = _mm_unpacklo_epi16(cmm[6], cmm[7]);  wmm[4] = _mm_unpacklo_epi16(cmm[0], cmm[8]);
  wmm[5] = _mm_unpacklo_epi16(cmm[9], cmm[10]); wmm[6] = _mm_unpacklo_epi16(cmm[11], cmm[12]); wmm[7] = _mm_unpacklo_epi16(cmm[13], cmm[14]);
  emm[0][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[0][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[0][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[0][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[0][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[0][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[0][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[0][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[15][0] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[15][1] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[15][2] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[15][3] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[15][4] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[15][5] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[15][6] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[15][7] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[1] = _mm_unpacklo_epi16(cmm[2], cmm[3]); wmm[2] = _mm_unpacklo_epi16(cmm[4], cmm[5]);
  wmm[3] = _mm_unpacklo_epi16(cmm[6], cmm[7]); wmm[4] = _mm_unpacklo_epi16(cmm[14], _mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])));
  wmm[5] = _mm_unpacklo_epi16(cmm[12], _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  wmm[6] = _mm_unpacklo_epi16(cmm[10], _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])));
  wmm[7] = _mm_unpacklo_epi16(cmm[8], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  emm[7][0] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[7][1] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[7][2] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[7][3] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[7][4] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[7][5] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[7][6] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[7][7] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[8][0] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[8][1] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[8][2] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[8][3] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[8][4] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[8][5] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[8][6] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[8][7] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[2] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])));  wmm[3] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  wmm[4] = _mm_unpacklo_epi16(cmm[10], _mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12]))); wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8])), cmm[14]);
  wmm[6] = _mm_unpacklo_epi16(cmm[0], cmm[13]); wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), _mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])));
  emm[3][0] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[3][1] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[3][2] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[3][3] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[3][4] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[3][5] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[3][6] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[3][7] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[12][0] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[12][1] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[12][2] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[12][3] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[12][4] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[12][5] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[12][6] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[12][7] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[2] = _mm_unpacklo_epi16(cmm[7], _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6]))); wmm[3] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));
  wmm[4] = _mm_unpacklo_epi16(cmm[11], _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9]))); wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])), cmm[0]);
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[14], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[14])), _mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8]))); wmm[7] = _mm_unpacklo_epi16(cmm[12], cmm[10]);
  emm[4][0] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[4][1] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[4][2] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[4][3] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[4][4] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[4][5] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[4][6] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[4][7] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[11][0] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], cmm[1]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[11][1] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], cmm[1]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[11][2] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], cmm[1]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[11][3] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], cmm[1]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[11][4] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], cmm[1]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[11][5] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], cmm[1]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[11][6] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], cmm[1]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[11][7] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], cmm[1]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[0] = _mm_unpacklo_epi16(cmm[1], _mm_mullo_epi16(cmm[1], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[1])));
  wmm[1] = _mm_unpacklo_epi16(cmm[3], _mm_mullo_epi16(cmm[2], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[2])));
  wmm[2] = _mm_unpacklo_epi16(cmm[5], _mm_mullo_epi16(cmm[7], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[7])));
  wmm[3] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])), _mm_mullo_epi16(cmm[6], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[6])));
  wmm[4] = _mm_unpacklo_epi16(cmm[8], cmm[11]); wmm[5] = _mm_unpacklo_epi16(cmm[14], _mm_mullo_epi16(cmm[12], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[12])));
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])), _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0])));
  wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10])), _mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])));
  emm[1][0] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[1][1] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[1][2] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[1][3] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[1][4] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[1][5] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[1][6] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[1][7] = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[14][0] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[14][1] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[14][2] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[14][3] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[14][4] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[14][5] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[14][6] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[14][7] = _mm_sub_epi32(_mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[4] = _mm_unpacklo_epi16(cmm[13], _mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10]))); wmm[5] = _mm_unpacklo_epi16(cmm[0], _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])));
  wmm[6] = _mm_unpacklo_epi16(cmm[12], cmm[14]); wmm[7] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[11], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[11])), cmm[8]);
  emm[6][0] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[6][1] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[6][2] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[6][3] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[6][4] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[6][5] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[6][6] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[6][7] = _mm_add_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[9][0] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[9][1] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[9][2] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[9][3] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[9][4] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[9][5] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[9][6] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[9][7] = _mm_sub_epi32(_mm_sub_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[2] = _mm_unpacklo_epi16(cmm[6], _mm_mullo_epi16(cmm[4], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[4])));   wmm[3] = _mm_unpacklo_epi16(cmm[7], cmm[5]);
  wmm[4] = _mm_unpacklo_epi16(cmm[9], cmm[14]); wmm[5] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[10], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[10])), _mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8])));
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[13], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[13])), cmm[11]); wmm[7] = _mm_unpacklo_epi16(cmm[0], cmm[12]);
  emm[2][0] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[2][1] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[2][2] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[2][3] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[2][4] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[2][5] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[2][6] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[2][7] = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[13][0] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[13][1] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[13][2] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[13][3] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[13][4] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[13][5] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[13][6] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[13][7] = _mm_sub_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  wmm[4] = _mm_unpacklo_epi16(cmm[12], _mm_mullo_epi16(cmm[0], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[0]))); wmm[5] = _mm_unpacklo_epi16(cmm[11], cmm[13]);
  wmm[6] = _mm_unpacklo_epi16(_mm_mullo_epi16(cmm[8], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[8])), cmm[10]);  wmm[7] = _mm_unpacklo_epi16(cmm[14], _mm_mullo_epi16(cmm[9], _mm_cmplt_epi16(_mm_setzero_si128(), cmm[9])));
  emm[5][0] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[5][1] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[5][2] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[5][3] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[5][4] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[5][5] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[5][6] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[5][7] = _mm_add_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));
  emm[10][0] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][0], wmm[0]), _mm_madd_epi16(xmm[1][0], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][0], wmm[2]), _mm_madd_epi16(xmm[3][0], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][0], wmm[4]), _mm_madd_epi16(xmm[5][0], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][0], wmm[6]), _mm_madd_epi16(xmm[7][0], wmm[7]))));
  emm[10][1] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][1], wmm[0]), _mm_madd_epi16(xmm[1][1], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][1], wmm[2]), _mm_madd_epi16(xmm[3][1], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][1], wmm[4]), _mm_madd_epi16(xmm[5][1], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][1], wmm[6]), _mm_madd_epi16(xmm[7][1], wmm[7]))));
  emm[10][2] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][2], wmm[0]), _mm_madd_epi16(xmm[1][2], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][2], wmm[2]), _mm_madd_epi16(xmm[3][2], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][2], wmm[4]), _mm_madd_epi16(xmm[5][2], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][2], wmm[6]), _mm_madd_epi16(xmm[7][2], wmm[7]))));
  emm[10][3] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][3], wmm[0]), _mm_madd_epi16(xmm[1][3], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][3], wmm[2]), _mm_madd_epi16(xmm[3][3], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][3], wmm[4]), _mm_madd_epi16(xmm[5][3], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][3], wmm[6]), _mm_madd_epi16(xmm[7][3], wmm[7]))));
  emm[10][4] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][4], wmm[0]), _mm_madd_epi16(xmm[1][4], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][4], wmm[2]), _mm_madd_epi16(xmm[3][4], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][4], wmm[4]), _mm_madd_epi16(xmm[5][4], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][4], wmm[6]), _mm_madd_epi16(xmm[7][4], wmm[7]))));
  emm[10][5] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][5], wmm[0]), _mm_madd_epi16(xmm[1][5], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][5], wmm[2]), _mm_madd_epi16(xmm[3][5], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][5], wmm[4]), _mm_madd_epi16(xmm[5][5], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][5], wmm[6]), _mm_madd_epi16(xmm[7][5], wmm[7]))));
  emm[10][6] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][6], wmm[0]), _mm_madd_epi16(xmm[1][6], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][6], wmm[2]), _mm_madd_epi16(xmm[3][6], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][6], wmm[4]), _mm_madd_epi16(xmm[5][6], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][6], wmm[6]), _mm_madd_epi16(xmm[7][6], wmm[7]))));
  emm[10][7] = _mm_sub_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_madd_epi16(xmm[0][7], wmm[0]), _mm_madd_epi16(xmm[1][7], wmm[1])), _mm_add_epi32(_mm_madd_epi16(xmm[2][7], wmm[2]), _mm_madd_epi16(xmm[3][7], wmm[3]))),
	  _mm_add_epi32(_mm_add_epi32(_mm_madd_epi16(xmm[4][7], wmm[4]), _mm_madd_epi16(xmm[5][7], wmm[5])), _mm_add_epi32(_mm_madd_epi16(xmm[6][7], wmm[6]), _mm_madd_epi16(xmm[7][7], wmm[7]))));

  // [JDS]: Compute "dst"
  wmm[0] = _mm_set1_epi32(add);
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][0], omm[0][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][1], omm[0][1]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][0], omm[1][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][1], omm[1][1]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][0], omm[2][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][1], omm[2][1]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][0], omm[3][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][1], omm[3][1]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][0], omm[4][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][1], omm[4][1]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][0], omm[5][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][1], omm[5][1]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][0], omm[6][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][1], omm[6][1]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][0], omm[7][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][1], omm[7][1]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + line), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 2 * line), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 3 * line), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 4 * line), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 5 * line), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 6 * line), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 7 * line), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][0], omm[0][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][1], omm[0][1]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][0], omm[1][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][1], omm[1][1]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][0], omm[2][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][1], omm[2][1]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][0], omm[3][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][1], omm[3][1]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][0], omm[4][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][1], omm[4][1]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][0], omm[5][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][1], omm[5][1]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][0], omm[6][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][1], omm[6][1]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][0], omm[7][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][1], omm[7][1]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 24), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + line + 24), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 2 * line + 24), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 3 * line + 24), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 4 * line + 24), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 5 * line + 24), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 6 * line + 24), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 7 * line + 24), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][0], omm[8][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][1], omm[8][1]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][0], omm[9][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][1], omm[9][1]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][0], omm[10][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][1], omm[10][1]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][0], omm[11][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][1], omm[11][1]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][0], omm[12][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][1], omm[12][1]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][0], omm[13][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][1], omm[13][1]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][0], omm[14][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][1], omm[14][1]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][0], omm[15][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][1], omm[15][1]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 8), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + line + 8), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 2 * line + 8), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 3 * line + 8), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 4 * line + 8), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 5 * line + 8), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 6 * line + 8), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 7 * line + 8), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][0], omm[8][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][1], omm[8][1]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][0], omm[9][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][1], omm[9][1]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][0], omm[10][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][1], omm[10][1]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][0], omm[11][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][1], omm[11][1]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][0], omm[12][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][1], omm[12][1]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][0], omm[13][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][1], omm[13][1]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][0], omm[14][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][1], omm[14][1]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][0], omm[15][0]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][1], omm[15][1]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 16), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + line + 16), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 2 * line + 16), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 3 * line + 16), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 4 * line + 16), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 5 * line + 16), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 6 * line + 16), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 7 * line + 16), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][2], omm[0][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][3], omm[0][3]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][2], omm[1][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][3], omm[1][3]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][2], omm[2][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][3], omm[2][3]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][2], omm[3][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][3], omm[3][3]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][2], omm[4][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][3], omm[4][3]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][2], omm[5][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][3], omm[5][3]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][2], omm[6][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][3], omm[6][3]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][2], omm[7][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][3], omm[7][3]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 8 * line), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 9 * line), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 10 * line), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 11 * line), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 12 * line), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 13 * line), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 14 * line), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 15 * line), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][2], omm[0][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][3], omm[0][3]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][2], omm[1][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][3], omm[1][3]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][2], omm[2][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][3], omm[2][3]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][2], omm[3][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][3], omm[3][3]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][2], omm[4][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][3], omm[4][3]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][2], omm[5][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][3], omm[5][3]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][2], omm[6][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][3], omm[6][3]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][2], omm[7][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][3], omm[7][3]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]);  xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]);  xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]);  xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]);  xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]);  xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]);  xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]);  xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]);  xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]);  xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]);  xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]);  xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]);  xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 8 * line + 24), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 9 * line + 24), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 10 * line + 24), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 11 * line + 24), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 12 * line + 24), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 13 * line + 24), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 14 * line + 24), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 15 * line + 24), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][2], omm[8][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][3], omm[8][3]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][2], omm[9][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][3], omm[9][3]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][2], omm[10][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][3], omm[10][3]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][2], omm[11][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][3], omm[11][3]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][2], omm[12][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][3], omm[12][3]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][2], omm[13][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][3], omm[13][3]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][2], omm[14][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][3], omm[14][3]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][2], omm[15][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][3], omm[15][3]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 8 * line + 8), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 9 * line + 8), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 10 * line + 8), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 11 * line + 8), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 12 * line + 8), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 13 * line + 8), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 14 * line + 8), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 15 * line + 8), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][2], omm[8][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][3], omm[8][3]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][2], omm[9][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][3], omm[9][3]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][2], omm[10][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][3], omm[10][3]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][2], omm[11][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][3], omm[11][3]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][2], omm[12][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][3], omm[12][3]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][2], omm[13][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][3], omm[13][3]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][2], omm[14][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][3], omm[14][3]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][2], omm[15][2]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][3], omm[15][3]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 8 * line + 16), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 9 * line + 16), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 10 * line + 16), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 11 * line + 16), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 12 * line + 16), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 13 * line + 16), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 14 * line + 16), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 15 * line + 16), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][4], omm[0][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][5], omm[0][5]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][4], omm[1][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][5], omm[1][5]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][4], omm[2][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][5], omm[2][5]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][4], omm[3][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][5], omm[3][5]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][4], omm[4][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][5], omm[4][5]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][4], omm[5][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][5], omm[5][5]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][4], omm[6][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][5], omm[6][5]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][4], omm[7][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][5], omm[7][5]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]);  xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]);  xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]);  xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]);  xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]);  xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]);  xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]);  xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]);  xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]);  xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]);  xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]);  xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]);  xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 16 * line), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 17 * line), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 18 * line), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 19 * line), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 20 * line), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 21 * line), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 22 * line), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 23 * line), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][4], omm[0][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][5], omm[0][5]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][4], omm[1][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][5], omm[1][5]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][4], omm[2][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][5], omm[2][5]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][4], omm[3][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][5], omm[3][5]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][4], omm[4][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][5], omm[4][5]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][4], omm[5][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][5], omm[5][5]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][4], omm[6][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][5], omm[6][5]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][4], omm[7][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][5], omm[7][5]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 16 * line + 24), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 17 * line + 24), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 18 * line + 24), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 19 * line + 24), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 20 * line + 24), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 21 * line + 24), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 22 * line + 24), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 23 * line + 24), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));


  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][4], omm[8][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][5], omm[8][5]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][4], omm[9][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][5], omm[9][5]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][4], omm[10][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][5], omm[10][5]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][4], omm[11][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][5], omm[11][5]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][4], omm[12][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][5], omm[12][5]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][4], omm[13][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][5], omm[13][5]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][4], omm[14][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][5], omm[14][5]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][4], omm[15][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][5], omm[15][5]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 16 * line + 8), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 17 * line + 8), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 18 * line + 8), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 19 * line + 8), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 20 * line + 8), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 21 * line + 8), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 22 * line + 8), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 23 * line + 8), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][4], omm[8][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][5], omm[8][5]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][4], omm[9][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][5], omm[9][5]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][4], omm[10][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][5], omm[10][5]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][4], omm[11][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][5], omm[11][5]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][4], omm[12][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][5], omm[12][5]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][4], omm[13][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][5], omm[13][5]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][4], omm[14][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][5], omm[14][5]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][4], omm[15][4]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][5], omm[15][5]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 16 * line + 16), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 17 * line + 16), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 18 * line + 16), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 19 * line + 16), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 20 * line + 16), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 21 * line + 16), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 22 * line + 16), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 23 * line + 16), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][6], omm[0][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[0][7], omm[0][7]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][6], omm[1][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[1][7], omm[1][7]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][6], omm[2][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[2][7], omm[2][7]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][6], omm[3][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[3][7], omm[3][7]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][6], omm[4][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[4][7], omm[4][7]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][6], omm[5][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[5][7], omm[5][7]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][6], omm[6][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[6][7], omm[6][7]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][6], omm[7][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[7][7], omm[7][7]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 24 * line), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 25 * line), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 26 * line), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 27 * line), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 28 * line), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 29 * line), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 30 * line), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 31 * line), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][6], omm[0][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[0][7], omm[0][7]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][6], omm[1][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[1][7], omm[1][7]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][6], omm[2][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[2][7], omm[2][7]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][6], omm[3][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[3][7], omm[3][7]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][6], omm[4][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[4][7], omm[4][7]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][6], omm[5][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[5][7], omm[5][7]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][6], omm[6][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[6][7], omm[6][7]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][6], omm[7][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[7][7], omm[7][7]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]);  xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]);  xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]);  xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]);  xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]);  xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]);  xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]);  xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]);  xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]);  xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]);  xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]);  xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]);  xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 24 * line + 24), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 25 * line + 24), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 26 * line + 24), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 27 * line + 24), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 28 * line + 24), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 29 * line + 24), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 30 * line + 24), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 31 * line + 24), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][6], omm[8][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[8][7], omm[8][7]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][6], omm[9][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[9][7], omm[9][7]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][6], omm[10][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[10][7], omm[10][7]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][6], omm[11][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[11][7], omm[11][7]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][6], omm[12][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[12][7], omm[12][7]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][6], omm[13][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[13][7], omm[13][7]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][6], omm[14][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[14][7], omm[14][7]), wmm[0]), shift));
  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][6], omm[15][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(emm[15][7], omm[15][7]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 24 * line + 8), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 25 * line + 8), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 26 * line + 8), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 27 * line + 8), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 28 * line + 8), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 29 * line + 8), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 30 * line + 8), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 31 * line + 8), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));

  xmm[0][7] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][6], omm[8][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[8][7], omm[8][7]), wmm[0]), shift));
  xmm[0][6] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][6], omm[9][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[9][7], omm[9][7]), wmm[0]), shift));
  xmm[0][5] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][6], omm[10][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[10][7], omm[10][7]), wmm[0]), shift));
  xmm[0][4] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][6], omm[11][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[11][7], omm[11][7]), wmm[0]), shift));
  xmm[0][3] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][6], omm[12][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[12][7], omm[12][7]), wmm[0]), shift));
  xmm[0][2] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][6], omm[13][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[13][7], omm[13][7]), wmm[0]), shift));
  xmm[0][1] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][6], omm[14][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[14][7], omm[14][7]), wmm[0]), shift));
  xmm[0][0] = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][6], omm[15][6]), wmm[0]), shift), _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(emm[15][7], omm[15][7]), wmm[0]), shift));
  xmm[1][0] = _mm_unpacklo_epi16(xmm[0][0], xmm[0][2]); xmm[1][1] = _mm_unpacklo_epi16(xmm[0][1], xmm[0][3]); xmm[1][2] = _mm_unpacklo_epi16(xmm[0][4], xmm[0][6]); xmm[1][3] = _mm_unpacklo_epi16(xmm[0][5], xmm[0][7]);
  xmm[1][4] = _mm_unpackhi_epi16(xmm[0][0], xmm[0][2]); xmm[1][5] = _mm_unpackhi_epi16(xmm[0][1], xmm[0][3]); xmm[1][6] = _mm_unpackhi_epi16(xmm[0][4], xmm[0][6]); xmm[1][7] = _mm_unpackhi_epi16(xmm[0][5], xmm[0][7]);
  xmm[0][0] = _mm_unpacklo_epi16(xmm[1][0], xmm[1][1]); xmm[0][1] = _mm_unpacklo_epi16(xmm[1][2], xmm[1][3]); xmm[0][2] = _mm_unpacklo_epi16(xmm[1][4], xmm[1][5]); xmm[0][3] = _mm_unpacklo_epi16(xmm[1][6], xmm[1][7]);
  xmm[0][4] = _mm_unpackhi_epi16(xmm[1][0], xmm[1][1]); xmm[0][5] = _mm_unpackhi_epi16(xmm[1][2], xmm[1][3]); xmm[0][6] = _mm_unpackhi_epi16(xmm[1][4], xmm[1][5]); xmm[0][7] = _mm_unpackhi_epi16(xmm[1][6], xmm[1][7]);
  _mm_storeu_si128((__m128i*)(dst + 24 * line + 16), _mm_unpacklo_epi64(xmm[0][0], xmm[0][1]));  _mm_storeu_si128((__m128i*)(dst + 25 * line + 16), _mm_unpackhi_epi64(xmm[0][0], xmm[0][1]));
  _mm_storeu_si128((__m128i*)(dst + 26 * line + 16), _mm_unpacklo_epi64(xmm[0][4], xmm[0][5]));  _mm_storeu_si128((__m128i*)(dst + 27 * line + 16), _mm_unpackhi_epi64(xmm[0][4], xmm[0][5]));
  _mm_storeu_si128((__m128i*)(dst + 28 * line + 16), _mm_unpacklo_epi64(xmm[0][2], xmm[0][3]));  _mm_storeu_si128((__m128i*)(dst + 29 * line + 16), _mm_unpackhi_epi64(xmm[0][2], xmm[0][3]));
  _mm_storeu_si128((__m128i*)(dst + 30 * line + 16), _mm_unpacklo_epi64(xmm[0][6], xmm[0][7]));  _mm_storeu_si128((__m128i*)(dst + 31 * line + 16), _mm_unpackhi_epi64(xmm[0][6], xmm[0][7]));
#else


Int j, k;
Int E[16], O[16];
Int EE[8], EO[8];
Int EEE[4], EEO[4];
Int EEEE[2], EEEO[2];

  for (j=0; j<line; j++)
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<16;k++)
    {
      O[k] = g_aiT32[ 1][k]*src[ line  ] + g_aiT32[ 3][k]*src[ 3*line  ] + g_aiT32[ 5][k]*src[ 5*line  ] + g_aiT32[ 7][k]*src[ 7*line  ] + 
        g_aiT32[ 9][k]*src[ 9*line  ] + g_aiT32[11][k]*src[ 11*line ] + g_aiT32[13][k]*src[ 13*line ] + g_aiT32[15][k]*src[ 15*line ] + 
        g_aiT32[17][k]*src[ 17*line ] + g_aiT32[19][k]*src[ 19*line ] + g_aiT32[21][k]*src[ 21*line ] + g_aiT32[23][k]*src[ 23*line ] + 
        g_aiT32[25][k]*src[ 25*line ] + g_aiT32[27][k]*src[ 27*line ] + g_aiT32[29][k]*src[ 29*line ] + g_aiT32[31][k]*src[ 31*line ];
    }
    for (k=0;k<8;k++)
    {
      EO[k] = g_aiT32[ 2][k]*src[ 2*line  ] + g_aiT32[ 6][k]*src[ 6*line  ] + g_aiT32[10][k]*src[ 10*line ] + g_aiT32[14][k]*src[ 14*line ] + 
        g_aiT32[18][k]*src[ 18*line ] + g_aiT32[22][k]*src[ 22*line ] + g_aiT32[26][k]*src[ 26*line ] + g_aiT32[30][k]*src[ 30*line ];
    }
    for (k=0;k<4;k++)
    {
      EEO[k] = g_aiT32[4][k]*src[ 4*line ] + g_aiT32[12][k]*src[ 12*line ] + g_aiT32[20][k]*src[ 20*line ] + g_aiT32[28][k]*src[ 28*line ];
    }
    EEEO[0] = g_aiT32[8][0]*src[ 8*line ] + g_aiT32[24][0]*src[ 24*line ];
    EEEO[1] = g_aiT32[8][1]*src[ 8*line ] + g_aiT32[24][1]*src[ 24*line ];
    EEEE[0] = g_aiT32[0][0]*src[ 0      ] + g_aiT32[16][0]*src[ 16*line ];    
    EEEE[1] = g_aiT32[0][1]*src[ 0      ] + g_aiT32[16][1]*src[ 16*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];    
    for (k=0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+4] = EEE[3-k] - EEO[3-k];
    }    
    for (k=0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+8] = EE[7-k] - EO[7-k];
    }    
    for (k=0;k<16;k++)
    {
      dst[k]    = Clip3( -32768, 32767, (E[k] + O[k] + add)>>shift );
      dst[k+16] = Clip3( -32768, 32767, (E[15-k] - O[15-k] + add)>>shift );
    }
    src ++;
    dst += 32;
  }
#endif // #if ETRI_SIMD_TR
}

/** MxN forward transform (2D)
*  \param block input data (residual)
*  \param coeff output data (transform coefficients)
*  \param iWidth input data (width of transform)
*  \param iHeight input data (height of transform)
*/
#if ETRI_SIMD_MATRIX_TRANSFORM
static void DCT4(short* plCoeff, Pel* pResidual, UInt uiDstStride, UInt uiSrcStride, Int shift)
{
    const __m128i xmm_add = _mm_set1_epi32(1 << (shift - 1));

    __m128i       xmm_s[2];
    __m128i       xmm_c[4];
    __m128i       xmm_t[2];

#if !_ETRI_WINDOWS_APPLICATION
    __m128i       xmm_zero = _mm_setzero_si128(); 
#endif 
    // load 16bit*16comp
    xmm_c[0] = _mm_loadl_epi64((const __m128i*) (pResidual));
    xmm_c[1] = _mm_loadl_epi64((const __m128i*) (pResidual + uiSrcStride));
    xmm_c[2] = _mm_loadl_epi64((const __m128i*) (pResidual + uiSrcStride * 2));
    xmm_c[3] = _mm_loadl_epi64((const __m128i*) (pResidual + uiSrcStride * 3));

    xmm_s[0] = _mm_unpacklo_epi64(xmm_c[0], xmm_c[1]);
    xmm_s[1] = _mm_unpacklo_epi64(xmm_c[2], xmm_c[3]);

    // load 16bit*16comp
    xmm_c[0] = _mm_loadl_epi64((const __m128i*) (g_aiT4[0]));
    xmm_c[1] = _mm_loadl_epi64((const __m128i*) (g_aiT4[1]));
    xmm_c[2] = _mm_loadl_epi64((const __m128i*) (g_aiT4[2]));
    xmm_c[3] = _mm_loadl_epi64((const __m128i*) (g_aiT4[3]));

    xmm_c[0] = _mm_unpacklo_epi64(xmm_c[0], xmm_c[0]);
    xmm_c[1] = _mm_unpacklo_epi64(xmm_c[1], xmm_c[1]);
    xmm_c[2] = _mm_unpacklo_epi64(xmm_c[2], xmm_c[2]);
    xmm_c[3] = _mm_unpacklo_epi64(xmm_c[3], xmm_c[3]);

    // calc
    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[0]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[0]);
    xmm_c[0] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[1]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[1]);
    xmm_c[1] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[2]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[2]);
    xmm_c[2] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[3]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[3]);
    xmm_c[3] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    // scale down
    xmm_c[0] = _mm_add_epi32(xmm_c[0], xmm_add);
    xmm_c[0] = _mm_srai_epi32(xmm_c[0], shift);

    xmm_c[1] = _mm_add_epi32(xmm_c[1], xmm_add);
    xmm_c[1] = _mm_srai_epi32(xmm_c[1], shift);

    xmm_c[2] = _mm_add_epi32(xmm_c[2], xmm_add);
    xmm_c[2] = _mm_srai_epi32(xmm_c[2], shift);

    xmm_c[3] = _mm_add_epi32(xmm_c[3], xmm_add);
    xmm_c[3] = _mm_srai_epi32(xmm_c[3], shift);

    // signed saturate 32bit->16bit
    xmm_s[0] = _mm_packs_epi32(xmm_c[0], xmm_c[1]);
    xmm_s[1] = _mm_packs_epi32(xmm_c[2], xmm_c[3]);

    // store
#if _ETRI_WINDOWS_APPLICATION
    *(int64_t *)(plCoeff) = xmm_s[0].m128i_i64[0];
    *(int64_t *)(plCoeff + uiDstStride) = xmm_s[0].m128i_i64[1];
    *(int64_t *)(plCoeff + uiDstStride * 2) = xmm_s[1].m128i_i64[0];
    *(int64_t *)(plCoeff + uiDstStride * 3) = xmm_s[1].m128i_i64[1];
#else 
    _mm_storel_epi64((__m128i*)(plCoeff), xmm_s[0]);
    xmm_t[0] = _mm_unpackhi_epi16(xmm_s[0], _mm_cmplt_epi16(xmm_s[0], xmm_zero));
    xmm_t[1] = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(plCoeff + uiDstStride), xmm_t[1]);

    _mm_storel_epi64((__m128i*)(plCoeff + uiDstStride * 2), xmm_s[1]);
    xmm_t[0] = _mm_unpackhi_epi16(xmm_s[1], _mm_cmplt_epi16(xmm_s[1], xmm_zero));
    xmm_t[1] = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(plCoeff + uiDstStride * 3), xmm_t[1]);
#endif 
}
static void DST4(short* plCoeff, Pel* pResidual, UInt uiDstStride, UInt uiSrcStride, Int shift)
{
    const __m128i xmm_add = _mm_set1_epi32(1 << (shift - 1));

    __m128i       xmm_s[2];
    __m128i       xmm_c[4];
    __m128i       xmm_t[2];

#if !_ETRI_WINDOWS_APPLICATION
    __m128i       xmm_zero = _mm_setzero_si128();
#endif 

    // load 16bit*16comp
    xmm_c[0] = _mm_loadl_epi64((const __m128i*) (pResidual));
    xmm_c[1] = _mm_loadl_epi64((const __m128i*) (pResidual + uiSrcStride));
    xmm_c[2] = _mm_loadl_epi64((const __m128i*) (pResidual + uiSrcStride * 2));
    xmm_c[3] = _mm_loadl_epi64((const __m128i*) (pResidual + uiSrcStride * 3));

    xmm_s[0] = _mm_unpacklo_epi64(xmm_c[0], xmm_c[1]);
    xmm_s[1] = _mm_unpacklo_epi64(xmm_c[2], xmm_c[3]);

    // load 16bit*16comp
    xmm_c[0] = _mm_loadl_epi64((const __m128i*) (g_as_DST_MAT_4[0]));
    xmm_c[1] = _mm_loadl_epi64((const __m128i*) (g_as_DST_MAT_4[1]));
    xmm_c[2] = _mm_loadl_epi64((const __m128i*) (g_as_DST_MAT_4[2]));
    xmm_c[3] = _mm_loadl_epi64((const __m128i*) (g_as_DST_MAT_4[3]));

    xmm_c[0] = _mm_unpacklo_epi64(xmm_c[0], xmm_c[0]);
    xmm_c[1] = _mm_unpacklo_epi64(xmm_c[1], xmm_c[1]);
    xmm_c[2] = _mm_unpacklo_epi64(xmm_c[2], xmm_c[2]);
    xmm_c[3] = _mm_unpacklo_epi64(xmm_c[3], xmm_c[3]);

    // calc
    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[0]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[0]);
    xmm_c[0] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[1]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[1]);
    xmm_c[1] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[2]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[2]);
    xmm_c[2] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c[3]);
    xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c[3]);
    xmm_c[3] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    // scale down
    xmm_c[0] = _mm_add_epi32(xmm_c[0], xmm_add);
    xmm_c[0] = _mm_srai_epi32(xmm_c[0], shift);

    xmm_c[1] = _mm_add_epi32(xmm_c[1], xmm_add);
    xmm_c[1] = _mm_srai_epi32(xmm_c[1], shift);

    xmm_c[2] = _mm_add_epi32(xmm_c[2], xmm_add);
    xmm_c[2] = _mm_srai_epi32(xmm_c[2], shift);

    xmm_c[3] = _mm_add_epi32(xmm_c[3], xmm_add);
    xmm_c[3] = _mm_srai_epi32(xmm_c[3], shift);

    // signed saturate 32bit->16bit
    xmm_s[0] = _mm_packs_epi32(xmm_c[0], xmm_c[1]);
    xmm_s[1] = _mm_packs_epi32(xmm_c[2], xmm_c[3]);

    // store
#if _ETRI_WINDOWS_APPLICATION
    *(int64_t *)(plCoeff) = xmm_s[0].m128i_i64[0];
    *(int64_t *)(plCoeff + uiDstStride) = xmm_s[0].m128i_i64[1];
    *(int64_t *)(plCoeff + uiDstStride * 2) = xmm_s[1].m128i_i64[0];
    *(int64_t *)(plCoeff + uiDstStride * 3) = xmm_s[1].m128i_i64[1];
#else 
    _mm_storel_epi64((__m128i*)(plCoeff), xmm_s[0]);
    xmm_t[0]  = _mm_unpackhi_epi16(xmm_s[0], _mm_cmplt_epi16(xmm_s[0], xmm_zero));
    xmm_t[1]  = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(plCoeff + uiDstStride), xmm_t[1]);

    _mm_storel_epi64((__m128i*)(plCoeff + uiDstStride * 2), xmm_s[1]);
    xmm_t[0] = _mm_unpackhi_epi16(xmm_s[1], _mm_cmplt_epi16(xmm_s[1], xmm_zero));
    xmm_t[1] = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(plCoeff + uiDstStride * 3), xmm_t[1]);
#endif 
}
static void DCT8(short* plCoeff, Pel* pResidual, UInt uiDstStride, UInt uiSrcStride, Int shift)
{
    Int j;

    const __m128i xmm_add = _mm_set1_epi32(1 << (shift - 1));

    __m128i       xmm_s[8];
    __m128i       xmm_t[8];
    __m128i       xmm_c;

    // load 16bit*64comp
    xmm_s[0] = _mm_loadu_si128((const __m128i *) (pResidual));
    xmm_s[1] = _mm_loadu_si128((const __m128i *) (pResidual + uiSrcStride));
    xmm_s[2] = _mm_loadu_si128((const __m128i *) (pResidual + uiSrcStride * 2));
    xmm_s[3] = _mm_loadu_si128((const __m128i *) (pResidual + uiSrcStride * 3));
    xmm_s[4] = _mm_loadu_si128((const __m128i *) (pResidual + uiSrcStride * 4));
    xmm_s[5] = _mm_loadu_si128((const __m128i *) (pResidual + uiSrcStride * 5));
    xmm_s[6] = _mm_loadu_si128((const __m128i *) (pResidual + uiSrcStride * 6));
    xmm_s[7] = _mm_loadu_si128((const __m128i *) (pResidual + uiSrcStride * 7));

    // calc
    for (j = 0; j < 8; j++)
    {
        // load 16bit*64comp
        xmm_c = _mm_loadu_si128((const __m128i *) g_aiT8[j]);

        xmm_t[0] = _mm_madd_epi16(xmm_s[0], xmm_c);
        xmm_t[1] = _mm_madd_epi16(xmm_s[1], xmm_c);
        xmm_t[2] = _mm_madd_epi16(xmm_s[2], xmm_c);
        xmm_t[3] = _mm_madd_epi16(xmm_s[3], xmm_c);
        xmm_t[4] = _mm_madd_epi16(xmm_s[4], xmm_c);
        xmm_t[5] = _mm_madd_epi16(xmm_s[5], xmm_c);
        xmm_t[6] = _mm_madd_epi16(xmm_s[6], xmm_c);
        xmm_t[7] = _mm_madd_epi16(xmm_s[7], xmm_c);

        xmm_t[0] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);
        xmm_t[1] = _mm_hadd_epi32(xmm_t[2], xmm_t[3]);
        xmm_t[2] = _mm_hadd_epi32(xmm_t[4], xmm_t[5]);
        xmm_t[3] = _mm_hadd_epi32(xmm_t[6], xmm_t[7]);

        xmm_t[0] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);
        xmm_t[1] = _mm_hadd_epi32(xmm_t[2], xmm_t[3]);

        // scale down
        xmm_t[0] = _mm_add_epi32(xmm_t[0], xmm_add);
        xmm_t[0] = _mm_srai_epi32(xmm_t[0], shift);

        xmm_t[1] = _mm_add_epi32(xmm_t[1], xmm_add);
        xmm_t[1] = _mm_srai_epi32(xmm_t[1], shift);

        // signed saturate 32bit->16bit
        xmm_t[0] = _mm_packs_epi32(xmm_t[0], xmm_t[1]);

        // store
        _mm_storeu_si128((__m128i*)(plCoeff), xmm_t[0]);

        plCoeff += uiDstStride;
    }
}
static void DCT16(const short *src, short *dst, UInt stride)
{
    // Const
    __m128i c_4 = _mm_set1_epi32(ADD_16T_1);
    __m128i c_512 = _mm_set1_epi32(ADD_16T_2);

    int i;

    ALIGN_VAR_32(int16_t, tmp[16 * 16]);

    __m128i T00A, T01A, T02A, T03A, T04A, T05A, T06A, T07A;
    __m128i T00B, T01B, T02B, T03B, T04B, T05B, T06B, T07B;
    __m128i T10, T11, T12, T13, T14, T15, T16, T17;
    __m128i T20, T21, T22, T23, T24, T25, T26, T27;
    __m128i T30, T31, T32, T33, T34, T35, T36, T37;
    __m128i T40, T41, T42, T43, T44, T45, T46, T47;
    __m128i T50, T51, T52, T53;
    __m128i T60, T61, T62, T63, T64, T65, T66, T67;
    __m128i T70;

    // DCT1
    for (i = 0; i < 16; i += 8)
    {

        T00A = _mm_loadu_si128((__m128i*)&src[(i + 0) * stride + 0]);    // [07 06 05 04 03 02 01 00]
        T00B = _mm_loadu_si128((__m128i*)&src[(i + 0) * stride + 8]);    // [0F 0E 0D 0C 0B 0A 09 08]
        T01A = _mm_loadu_si128((__m128i*)&src[(i + 1) * stride + 0]);    // [17 16 15 14 13 12 11 10]
        T01B = _mm_loadu_si128((__m128i*)&src[(i + 1) * stride + 8]);    // [1F 1E 1D 1C 1B 1A 19 18]
        T02A = _mm_loadu_si128((__m128i*)&src[(i + 2) * stride + 0]);    // [27 26 25 24 23 22 21 20]
        T02B = _mm_loadu_si128((__m128i*)&src[(i + 2) * stride + 8]);    // [2F 2E 2D 2C 2B 2A 29 28]
        T03A = _mm_loadu_si128((__m128i*)&src[(i + 3) * stride + 0]);    // [37 36 35 34 33 32 31 30]
        T03B = _mm_loadu_si128((__m128i*)&src[(i + 3) * stride + 8]);    // [3F 3E 3D 3C 3B 3A 39 38]
        T04A = _mm_loadu_si128((__m128i*)&src[(i + 4) * stride + 0]);    // [47 46 45 44 43 42 41 40]
        T04B = _mm_loadu_si128((__m128i*)&src[(i + 4) * stride + 8]);    // [4F 4E 4D 4C 4B 4A 49 48]
        T05A = _mm_loadu_si128((__m128i*)&src[(i + 5) * stride + 0]);    // [57 56 55 54 53 52 51 50]
        T05B = _mm_loadu_si128((__m128i*)&src[(i + 5) * stride + 8]);    // [5F 5E 5D 5C 5B 5A 59 58]
        T06A = _mm_loadu_si128((__m128i*)&src[(i + 6) * stride + 0]);    // [67 66 65 64 63 62 61 60]
        T06B = _mm_loadu_si128((__m128i*)&src[(i + 6) * stride + 8]);    // [6F 6E 6D 6C 6B 6A 69 68]
        T07A = _mm_loadu_si128((__m128i*)&src[(i + 7) * stride + 0]);    // [77 76 75 74 73 72 71 70]
        T07B = _mm_loadu_si128((__m128i*)&src[(i + 7) * stride + 8]);    // [7F 7E 7D 7C 7B 7A 79 78]

        T00B = _mm_shuffle_epi8(T00B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T01B = _mm_shuffle_epi8(T01B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T02B = _mm_shuffle_epi8(T02B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T03B = _mm_shuffle_epi8(T03B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T04B = _mm_shuffle_epi8(T04B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T05B = _mm_shuffle_epi8(T05B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T06B = _mm_shuffle_epi8(T06B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T07B = _mm_shuffle_epi8(T07B, _mm_load_si128((__m128i*)tab_dct_16_0[0]));

        T10 = _mm_add_epi16(T00A, T00B);
        T11 = _mm_add_epi16(T01A, T01B);
        T12 = _mm_add_epi16(T02A, T02B);
        T13 = _mm_add_epi16(T03A, T03B);
        T14 = _mm_add_epi16(T04A, T04B);
        T15 = _mm_add_epi16(T05A, T05B);
        T16 = _mm_add_epi16(T06A, T06B);
        T17 = _mm_add_epi16(T07A, T07B);

        T20 = _mm_sub_epi16(T00A, T00B);
        T21 = _mm_sub_epi16(T01A, T01B);
        T22 = _mm_sub_epi16(T02A, T02B);
        T23 = _mm_sub_epi16(T03A, T03B);
        T24 = _mm_sub_epi16(T04A, T04B);
        T25 = _mm_sub_epi16(T05A, T05B);
        T26 = _mm_sub_epi16(T06A, T06B);
        T27 = _mm_sub_epi16(T07A, T07B);

        T30 = _mm_shuffle_epi8(T10, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T31 = _mm_shuffle_epi8(T11, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T32 = _mm_shuffle_epi8(T12, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T33 = _mm_shuffle_epi8(T13, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T34 = _mm_shuffle_epi8(T14, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T35 = _mm_shuffle_epi8(T15, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T36 = _mm_shuffle_epi8(T16, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T37 = _mm_shuffle_epi8(T17, _mm_load_si128((__m128i*)tab_dct_16_0[1]));

        T40 = _mm_hadd_epi16(T30, T31);
        T41 = _mm_hadd_epi16(T32, T33);
        T42 = _mm_hadd_epi16(T34, T35);
        T43 = _mm_hadd_epi16(T36, T37);
        T44 = _mm_hsub_epi16(T30, T31);
        T45 = _mm_hsub_epi16(T32, T33);
        T46 = _mm_hsub_epi16(T34, T35);
        T47 = _mm_hsub_epi16(T36, T37);

        T50 = _mm_hadd_epi16(T40, T41);
        T51 = _mm_hadd_epi16(T42, T43);
        T52 = _mm_hsub_epi16(T40, T41);
        T53 = _mm_hsub_epi16(T42, T43);

        T60 = _mm_madd_epi16(T50, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T61 = _mm_madd_epi16(T51, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[0 * 16 + i], T70);

        T60 = _mm_madd_epi16(T50, _mm_load_si128((__m128i*)tab_dct_8[2]));
        T61 = _mm_madd_epi16(T51, _mm_load_si128((__m128i*)tab_dct_8[2]));
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[8 * 16 + i], T70);

        T60 = _mm_madd_epi16(T52, _mm_load_si128((__m128i*)tab_dct_8[3]));
        T61 = _mm_madd_epi16(T53, _mm_load_si128((__m128i*)tab_dct_8[3]));
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[4 * 16 + i], T70);

        T60 = _mm_madd_epi16(T52, _mm_load_si128((__m128i*)tab_dct_8[4]));
        T61 = _mm_madd_epi16(T53, _mm_load_si128((__m128i*)tab_dct_8[4]));
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[12 * 16 + i], T70);

        T60 = _mm_madd_epi16(T44, _mm_load_si128((__m128i*)tab_dct_8[5]));
        T61 = _mm_madd_epi16(T45, _mm_load_si128((__m128i*)tab_dct_8[5]));
        T62 = _mm_madd_epi16(T46, _mm_load_si128((__m128i*)tab_dct_8[5]));
        T63 = _mm_madd_epi16(T47, _mm_load_si128((__m128i*)tab_dct_8[5]));
        T60 = _mm_hadd_epi32(T60, T61);
        T61 = _mm_hadd_epi32(T62, T63);
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[2 * 16 + i], T70);

        T60 = _mm_madd_epi16(T44, _mm_load_si128((__m128i*)tab_dct_8[6]));
        T61 = _mm_madd_epi16(T45, _mm_load_si128((__m128i*)tab_dct_8[6]));
        T62 = _mm_madd_epi16(T46, _mm_load_si128((__m128i*)tab_dct_8[6]));
        T63 = _mm_madd_epi16(T47, _mm_load_si128((__m128i*)tab_dct_8[6]));
        T60 = _mm_hadd_epi32(T60, T61);
        T61 = _mm_hadd_epi32(T62, T63);
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[6 * 16 + i], T70);

        T60 = _mm_madd_epi16(T44, _mm_load_si128((__m128i*)tab_dct_8[7]));
        T61 = _mm_madd_epi16(T45, _mm_load_si128((__m128i*)tab_dct_8[7]));
        T62 = _mm_madd_epi16(T46, _mm_load_si128((__m128i*)tab_dct_8[7]));
        T63 = _mm_madd_epi16(T47, _mm_load_si128((__m128i*)tab_dct_8[7]));
        T60 = _mm_hadd_epi32(T60, T61);
        T61 = _mm_hadd_epi32(T62, T63);
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[10 * 16 + i], T70);

        T60 = _mm_madd_epi16(T44, _mm_load_si128((__m128i*)tab_dct_8[8]));
        T61 = _mm_madd_epi16(T45, _mm_load_si128((__m128i*)tab_dct_8[8]));
        T62 = _mm_madd_epi16(T46, _mm_load_si128((__m128i*)tab_dct_8[8]));
        T63 = _mm_madd_epi16(T47, _mm_load_si128((__m128i*)tab_dct_8[8]));
        T60 = _mm_hadd_epi32(T60, T61);
        T61 = _mm_hadd_epi32(T62, T63);
        T60 = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST);
        T61 = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST);
        T70 = _mm_packs_epi32(T60, T61);
        _mm_store_si128((__m128i*)&tmp[14 * 16 + i], T70);

#define MAKE_ODD(tab, dstPos) \
  T60  = _mm_madd_epi16(T20, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T61  = _mm_madd_epi16(T21, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T62  = _mm_madd_epi16(T22, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T63  = _mm_madd_epi16(T23, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T64  = _mm_madd_epi16(T24, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T65  = _mm_madd_epi16(T25, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T66  = _mm_madd_epi16(T26, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T67  = _mm_madd_epi16(T27, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T60  = _mm_hadd_epi32(T60, T61); \
  T61  = _mm_hadd_epi32(T62, T63); \
  T62  = _mm_hadd_epi32(T64, T65); \
  T63  = _mm_hadd_epi32(T66, T67); \
  T60  = _mm_hadd_epi32(T60, T61); \
  T61  = _mm_hadd_epi32(T62, T63); \
  T60  = _mm_srai_epi32(_mm_add_epi32(T60, c_4), SHIFT_16T_1ST); \
  T61  = _mm_srai_epi32(_mm_add_epi32(T61, c_4), SHIFT_16T_1ST); \
  T70  = _mm_packs_epi32(T60, T61); \
  _mm_store_si128((__m128i*)&tmp[(dstPos) * 16 + i], T70);

        MAKE_ODD(0, 1);
        MAKE_ODD(1, 3);
        MAKE_ODD(2, 5);
        MAKE_ODD(3, 7);
        MAKE_ODD(4, 9);
        MAKE_ODD(5, 11);
        MAKE_ODD(6, 13);
        MAKE_ODD(7, 15);
#undef MAKE_ODD
    }

    // DCT2
    for (i = 0; i < 16; i += 4)
    {
        T00A = _mm_load_si128((__m128i*)&tmp[(i + 0) * 16 + 0]);    // [07 06 05 04 03 02 01 00]
        T00B = _mm_load_si128((__m128i*)&tmp[(i + 0) * 16 + 8]);    // [0F 0E 0D 0C 0B 0A 09 08]
        T01A = _mm_load_si128((__m128i*)&tmp[(i + 1) * 16 + 0]);    // [17 16 15 14 13 12 11 10]
        T01B = _mm_load_si128((__m128i*)&tmp[(i + 1) * 16 + 8]);    // [1F 1E 1D 1C 1B 1A 19 18]
        T02A = _mm_load_si128((__m128i*)&tmp[(i + 2) * 16 + 0]);    // [27 26 25 24 23 22 21 20]
        T02B = _mm_load_si128((__m128i*)&tmp[(i + 2) * 16 + 8]);    // [2F 2E 2D 2C 2B 2A 29 28]
        T03A = _mm_load_si128((__m128i*)&tmp[(i + 3) * 16 + 0]);    // [37 36 35 34 33 32 31 30]
        T03B = _mm_load_si128((__m128i*)&tmp[(i + 3) * 16 + 8]);    // [3F 3E 3D 3C 3B 3A 39 38]

        T00A = _mm_shuffle_epi8(T00A, _mm_load_si128((__m128i*)tab_dct_16_0[2]));
        T00B = _mm_shuffle_epi8(T00B, _mm_load_si128((__m128i*)tab_dct_16_0[3]));
        T01A = _mm_shuffle_epi8(T01A, _mm_load_si128((__m128i*)tab_dct_16_0[2]));
        T01B = _mm_shuffle_epi8(T01B, _mm_load_si128((__m128i*)tab_dct_16_0[3]));
        T02A = _mm_shuffle_epi8(T02A, _mm_load_si128((__m128i*)tab_dct_16_0[2]));
        T02B = _mm_shuffle_epi8(T02B, _mm_load_si128((__m128i*)tab_dct_16_0[3]));
        T03A = _mm_shuffle_epi8(T03A, _mm_load_si128((__m128i*)tab_dct_16_0[2]));
        T03B = _mm_shuffle_epi8(T03B, _mm_load_si128((__m128i*)tab_dct_16_0[3]));

        T10 = _mm_unpacklo_epi16(T00A, T00B);
        T11 = _mm_unpackhi_epi16(T00A, T00B);
        T12 = _mm_unpacklo_epi16(T01A, T01B);
        T13 = _mm_unpackhi_epi16(T01A, T01B);
        T14 = _mm_unpacklo_epi16(T02A, T02B);
        T15 = _mm_unpackhi_epi16(T02A, T02B);
        T16 = _mm_unpacklo_epi16(T03A, T03B);
        T17 = _mm_unpackhi_epi16(T03A, T03B);

        T20 = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T21 = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T22 = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T23 = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T24 = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T25 = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T26 = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T27 = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_8[1]));

        T30 = _mm_add_epi32(T20, T21);
        T31 = _mm_add_epi32(T22, T23);
        T32 = _mm_add_epi32(T24, T25);
        T33 = _mm_add_epi32(T26, T27);

        T30 = _mm_hadd_epi32(T30, T31);
        T31 = _mm_hadd_epi32(T32, T33);

        T40 = _mm_hadd_epi32(T30, T31);
        T41 = _mm_hsub_epi32(T30, T31);
        T40 = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND);
        T41 = _mm_srai_epi32(_mm_add_epi32(T41, c_512), SHIFT_16T_2ND);
        T40 = _mm_packs_epi32(T40, T40);
        T41 = _mm_packs_epi32(T41, T41);
        _mm_storel_epi64((__m128i*)&dst[0 * 16 + i], T40);
        _mm_storel_epi64((__m128i*)&dst[8 * 16 + i], T41);

        T20 = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T21 = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T22 = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T23 = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T24 = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T25 = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T26 = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T27 = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_16_1[8]));

        T30 = _mm_add_epi32(T20, T21);
        T31 = _mm_add_epi32(T22, T23);
        T32 = _mm_add_epi32(T24, T25);
        T33 = _mm_add_epi32(T26, T27);

        T30 = _mm_hadd_epi32(T30, T31);
        T31 = _mm_hadd_epi32(T32, T33);

        T40 = _mm_hadd_epi32(T30, T31);
        T40 = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND);
        T40 = _mm_packs_epi32(T40, T40);
        _mm_storel_epi64((__m128i*)&dst[4 * 16 + i], T40);

        T20 = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T21 = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T22 = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T23 = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T24 = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T25 = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T26 = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T27 = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_16_1[9]));

        T30 = _mm_add_epi32(T20, T21);
        T31 = _mm_add_epi32(T22, T23);
        T32 = _mm_add_epi32(T24, T25);
        T33 = _mm_add_epi32(T26, T27);

        T30 = _mm_hadd_epi32(T30, T31);
        T31 = _mm_hadd_epi32(T32, T33);

        T40 = _mm_hadd_epi32(T30, T31);
        T40 = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND);
        T40 = _mm_packs_epi32(T40, T40);
        _mm_storel_epi64((__m128i*)&dst[12 * 16 + i], T40);

        T20 = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_16_1[10]));
        T21 = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_16_1[10]));
        T22 = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_16_1[10]));
        T23 = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_16_1[10]));
        T24 = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_16_1[10]));
        T25 = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_16_1[10]));
        T26 = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_16_1[10]));
        T27 = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_16_1[10]));

        T30 = _mm_sub_epi32(T20, T21);
        T31 = _mm_sub_epi32(T22, T23);
        T32 = _mm_sub_epi32(T24, T25);
        T33 = _mm_sub_epi32(T26, T27);

        T30 = _mm_hadd_epi32(T30, T31);
        T31 = _mm_hadd_epi32(T32, T33);

        T40 = _mm_hadd_epi32(T30, T31);
        T40 = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND);
        T40 = _mm_packs_epi32(T40, T40);
        _mm_storel_epi64((__m128i*)&dst[2 * 16 + i], T40);

        T20 = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_16_1[11]));
        T21 = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_16_1[11]));
        T22 = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_16_1[11]));
        T23 = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_16_1[11]));
        T24 = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_16_1[11]));
        T25 = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_16_1[11]));
        T26 = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_16_1[11]));
        T27 = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_16_1[11]));

        T30 = _mm_sub_epi32(T20, T21);
        T31 = _mm_sub_epi32(T22, T23);
        T32 = _mm_sub_epi32(T24, T25);
        T33 = _mm_sub_epi32(T26, T27);

        T30 = _mm_hadd_epi32(T30, T31);
        T31 = _mm_hadd_epi32(T32, T33);

        T40 = _mm_hadd_epi32(T30, T31);
        T40 = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND);
        T40 = _mm_packs_epi32(T40, T40);
        _mm_storel_epi64((__m128i*)&dst[6 * 16 + i], T40);

        T20 = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_16_1[12]));
        T21 = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_16_1[12]));
        T22 = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_16_1[12]));
        T23 = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_16_1[12]));
        T24 = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_16_1[12]));
        T25 = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_16_1[12]));
        T26 = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_16_1[12]));
        T27 = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_16_1[12]));

        T30 = _mm_sub_epi32(T20, T21);
        T31 = _mm_sub_epi32(T22, T23);
        T32 = _mm_sub_epi32(T24, T25);
        T33 = _mm_sub_epi32(T26, T27);

        T30 = _mm_hadd_epi32(T30, T31);
        T31 = _mm_hadd_epi32(T32, T33);

        T40 = _mm_hadd_epi32(T30, T31);
        T40 = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND);
        T40 = _mm_packs_epi32(T40, T40);
        _mm_storel_epi64((__m128i*)&dst[10 * 16 + i], T40);

        T20 = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_16_1[13]));
        T21 = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_16_1[13]));
        T22 = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_16_1[13]));
        T23 = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_16_1[13]));
        T24 = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_16_1[13]));
        T25 = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_16_1[13]));
        T26 = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_16_1[13]));
        T27 = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_16_1[13]));

        T30 = _mm_sub_epi32(T20, T21);
        T31 = _mm_sub_epi32(T22, T23);
        T32 = _mm_sub_epi32(T24, T25);
        T33 = _mm_sub_epi32(T26, T27);

        T30 = _mm_hadd_epi32(T30, T31);
        T31 = _mm_hadd_epi32(T32, T33);

        T40 = _mm_hadd_epi32(T30, T31);
        T40 = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND);
        T40 = _mm_packs_epi32(T40, T40);
        _mm_storel_epi64((__m128i*)&dst[14 * 16 + i], T40);

#define MAKE_ODD(tab, dstPos) \
  T20  = _mm_madd_epi16(T10, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)]));       /* [*O2_0 *O1_0 *O3_0 *O0_0] */ \
  T21  = _mm_madd_epi16(T11, _mm_load_si128((__m128i*)tab_dct_16_1[(tab) + 1]));   /* [*O5_0 *O6_0 *O4_0 *O7_0] */ \
  T22  = _mm_madd_epi16(T12, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T23  = _mm_madd_epi16(T13, _mm_load_si128((__m128i*)tab_dct_16_1[(tab) + 1])); \
  T24  = _mm_madd_epi16(T14, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T25  = _mm_madd_epi16(T15, _mm_load_si128((__m128i*)tab_dct_16_1[(tab) + 1])); \
  T26  = _mm_madd_epi16(T16, _mm_load_si128((__m128i*)tab_dct_16_1[(tab)])); \
  T27  = _mm_madd_epi16(T17, _mm_load_si128((__m128i*)tab_dct_16_1[(tab) + 1])); \
  \
  T30  = _mm_add_epi32(T20, T21); \
  T31  = _mm_add_epi32(T22, T23); \
  T32  = _mm_add_epi32(T24, T25); \
  T33  = _mm_add_epi32(T26, T27); \
  \
  T30  = _mm_hadd_epi32(T30, T31); \
  T31  = _mm_hadd_epi32(T32, T33); \
  \
  T40  = _mm_hadd_epi32(T30, T31); \
  T40  = _mm_srai_epi32(_mm_add_epi32(T40, c_512), SHIFT_16T_2ND); \
  T40  = _mm_packs_epi32(T40, T40); \
  _mm_storel_epi64((__m128i*)&dst[(dstPos) * 16 + i], T40);

        MAKE_ODD(14, 1);
        MAKE_ODD(16, 3);
        MAKE_ODD(18, 5);
        MAKE_ODD(20, 7);
        MAKE_ODD(22, 9);
        MAKE_ODD(24, 11);
        MAKE_ODD(26, 13);
        MAKE_ODD(28, 15);
#undef MAKE_ODD
    }
}
static void DCT32(const short *src, short *dst, UInt stride)
{

    // Const
    __m128i c_8 = _mm_set1_epi32(ADD_32T_1);
    __m128i c_1024 = _mm_set1_epi32(ADD_32T_2);

    int i;

    __m128i T00A, T01A, T02A, T03A, T04A, T05A, T06A, T07A;
    __m128i T00B, T01B, T02B, T03B, T04B, T05B, T06B, T07B;
    __m128i T00C, T01C, T02C, T03C, T04C, T05C, T06C, T07C;
    __m128i T00D, T01D, T02D, T03D, T04D, T05D, T06D, T07D;
    __m128i T10A, T11A, T12A, T13A, T14A, T15A, T16A, T17A;
    __m128i T10B, T11B, T12B, T13B, T14B, T15B, T16B, T17B;
    __m128i T20, T21, T22, T23, T24, T25, T26, T27;
    __m128i T30, T31, T32, T33, T34, T35, T36, T37;
    __m128i T40, T41, T42, T43, T44, T45, T46, T47;
    __m128i T50, T51, T52, T53;
    __m128i T60, T61, T62, T63, T64, T65, T66, T67;
    __m128i im[32][4];

    // DCT1
    for (i = 0; i < 32 / 8; i++)
    {
        T00A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 0) * stride + 0]);    // [07 06 05 04 03 02 01 00]
        T00B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 0) * stride + 8]);    // [15 14 13 12 11 10 09 08]
        T00C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 0) * stride + 16]);    // [23 22 21 20 19 18 17 16]
        T00D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 0) * stride + 24]);    // [31 30 29 28 27 26 25 24]
        T01A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 1) * stride + 0]);
        T01B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 1) * stride + 8]);
        T01C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 1) * stride + 16]);
        T01D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 1) * stride + 24]);
        T02A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 2) * stride + 0]);
        T02B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 2) * stride + 8]);
        T02C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 2) * stride + 16]);
        T02D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 2) * stride + 24]);
        T03A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 3) * stride + 0]);
        T03B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 3) * stride + 8]);
        T03C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 3) * stride + 16]);
        T03D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 3) * stride + 24]);
        T04A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 4) * stride + 0]);
        T04B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 4) * stride + 8]);
        T04C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 4) * stride + 16]);
        T04D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 4) * stride + 24]);
        T05A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 5) * stride + 0]);
        T05B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 5) * stride + 8]);
        T05C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 5) * stride + 16]);
        T05D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 5) * stride + 24]);
        T06A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 6) * stride + 0]);
        T06B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 6) * stride + 8]);
        T06C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 6) * stride + 16]);
        T06D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 6) * stride + 24]);
        T07A = _mm_loadu_si128((__m128i*)&src[(i * 8 + 7) * stride + 0]);
        T07B = _mm_loadu_si128((__m128i*)&src[(i * 8 + 7) * stride + 8]);
        T07C = _mm_loadu_si128((__m128i*)&src[(i * 8 + 7) * stride + 16]);
        T07D = _mm_loadu_si128((__m128i*)&src[(i * 8 + 7) * stride + 24]); 

        T00A = _mm_shuffle_epi8(T00A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));    // [05 02 06 01 04 03 07 00]
        T00B = _mm_shuffle_epi8(T00B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));    // [10 13 09 14 11 12 08 15]
        T00C = _mm_shuffle_epi8(T00C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));    // [21 18 22 17 20 19 23 16]
        T00D = _mm_shuffle_epi8(T00D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));    // [26 29 25 30 27 28 24 31]
        T01A = _mm_shuffle_epi8(T01A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T01B = _mm_shuffle_epi8(T01B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T01C = _mm_shuffle_epi8(T01C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T01D = _mm_shuffle_epi8(T01D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T02A = _mm_shuffle_epi8(T02A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T02B = _mm_shuffle_epi8(T02B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T02C = _mm_shuffle_epi8(T02C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T02D = _mm_shuffle_epi8(T02D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T03A = _mm_shuffle_epi8(T03A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T03B = _mm_shuffle_epi8(T03B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T03C = _mm_shuffle_epi8(T03C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T03D = _mm_shuffle_epi8(T03D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T04A = _mm_shuffle_epi8(T04A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T04B = _mm_shuffle_epi8(T04B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T04C = _mm_shuffle_epi8(T04C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T04D = _mm_shuffle_epi8(T04D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T05A = _mm_shuffle_epi8(T05A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T05B = _mm_shuffle_epi8(T05B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T05C = _mm_shuffle_epi8(T05C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T05D = _mm_shuffle_epi8(T05D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T06A = _mm_shuffle_epi8(T06A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T06B = _mm_shuffle_epi8(T06B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T06C = _mm_shuffle_epi8(T06C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T06D = _mm_shuffle_epi8(T06D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T07A = _mm_shuffle_epi8(T07A, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T07B = _mm_shuffle_epi8(T07B, _mm_load_si128((__m128i*)tab_dct_32_0[0]));
        T07C = _mm_shuffle_epi8(T07C, _mm_load_si128((__m128i*)tab_dct_16_0[1]));
        T07D = _mm_shuffle_epi8(T07D, _mm_load_si128((__m128i*)tab_dct_32_0[0]));

        T10A = _mm_add_epi16(T00A, T00D);   // [E05 E02 E06 E01 E04 E03 E07 E00]
        T10B = _mm_add_epi16(T00B, T00C);   // [E10 E13 E09 E14 E11 E12 E08 E15]
        T11A = _mm_add_epi16(T01A, T01D);
        T11B = _mm_add_epi16(T01B, T01C);
        T12A = _mm_add_epi16(T02A, T02D);
        T12B = _mm_add_epi16(T02B, T02C);
        T13A = _mm_add_epi16(T03A, T03D);
        T13B = _mm_add_epi16(T03B, T03C);
        T14A = _mm_add_epi16(T04A, T04D);
        T14B = _mm_add_epi16(T04B, T04C);
        T15A = _mm_add_epi16(T05A, T05D);
        T15B = _mm_add_epi16(T05B, T05C);
        T16A = _mm_add_epi16(T06A, T06D);
        T16B = _mm_add_epi16(T06B, T06C);
        T17A = _mm_add_epi16(T07A, T07D);
        T17B = _mm_add_epi16(T07B, T07C);

        T00A = _mm_sub_epi16(T00A, T00D);   // [O05 O02 O06 O01 O04 O03 O07 O00]
        T00B = _mm_sub_epi16(T00B, T00C);   // [O10 O13 O09 O14 O11 O12 O08 O15]
        T01A = _mm_sub_epi16(T01A, T01D);
        T01B = _mm_sub_epi16(T01B, T01C);
        T02A = _mm_sub_epi16(T02A, T02D);
        T02B = _mm_sub_epi16(T02B, T02C);
        T03A = _mm_sub_epi16(T03A, T03D);
        T03B = _mm_sub_epi16(T03B, T03C);
        T04A = _mm_sub_epi16(T04A, T04D);
        T04B = _mm_sub_epi16(T04B, T04C);
        T05A = _mm_sub_epi16(T05A, T05D);
        T05B = _mm_sub_epi16(T05B, T05C);
        T06A = _mm_sub_epi16(T06A, T06D);
        T06B = _mm_sub_epi16(T06B, T06C);
        T07A = _mm_sub_epi16(T07A, T07D);
        T07B = _mm_sub_epi16(T07B, T07C);

        T20 = _mm_add_epi16(T10A, T10B);   // [EE5 EE2 EE6 EE1 EE4 EE3 EE7 EE0]
        T21 = _mm_add_epi16(T11A, T11B);
        T22 = _mm_add_epi16(T12A, T12B);
        T23 = _mm_add_epi16(T13A, T13B);
        T24 = _mm_add_epi16(T14A, T14B);
        T25 = _mm_add_epi16(T15A, T15B);
        T26 = _mm_add_epi16(T16A, T16B);
        T27 = _mm_add_epi16(T17A, T17B);

        T30 = _mm_madd_epi16(T20, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T31 = _mm_madd_epi16(T21, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T32 = _mm_madd_epi16(T22, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T33 = _mm_madd_epi16(T23, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T34 = _mm_madd_epi16(T24, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T35 = _mm_madd_epi16(T25, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T36 = _mm_madd_epi16(T26, _mm_load_si128((__m128i*)tab_dct_8[1]));
        T37 = _mm_madd_epi16(T27, _mm_load_si128((__m128i*)tab_dct_8[1]));

        T40 = _mm_hadd_epi32(T30, T31);
        T41 = _mm_hadd_epi32(T32, T33);
        T42 = _mm_hadd_epi32(T34, T35);
        T43 = _mm_hadd_epi32(T36, T37);

        T50 = _mm_hadd_epi32(T40, T41);
        T51 = _mm_hadd_epi32(T42, T43);
        T50 = _mm_srai_epi32(_mm_add_epi32(T50, c_8), SHIFT_32T_1ST);
        T51 = _mm_srai_epi32(_mm_add_epi32(T51, c_8), SHIFT_32T_1ST);
        T60 = _mm_packs_epi32(T50, T51);
        im[0][i] = T60;

        T50 = _mm_hsub_epi32(T40, T41);
        T51 = _mm_hsub_epi32(T42, T43);
        T50 = _mm_srai_epi32(_mm_add_epi32(T50, c_8), SHIFT_32T_1ST);
        T51 = _mm_srai_epi32(_mm_add_epi32(T51, c_8), SHIFT_32T_1ST);
        T60 = _mm_packs_epi32(T50, T51);
        im[16][i] = T60;

        T30 = _mm_madd_epi16(T20, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T31 = _mm_madd_epi16(T21, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T32 = _mm_madd_epi16(T22, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T33 = _mm_madd_epi16(T23, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T34 = _mm_madd_epi16(T24, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T35 = _mm_madd_epi16(T25, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T36 = _mm_madd_epi16(T26, _mm_load_si128((__m128i*)tab_dct_16_1[8]));
        T37 = _mm_madd_epi16(T27, _mm_load_si128((__m128i*)tab_dct_16_1[8]));

        T40 = _mm_hadd_epi32(T30, T31);
        T41 = _mm_hadd_epi32(T32, T33);
        T42 = _mm_hadd_epi32(T34, T35);
        T43 = _mm_hadd_epi32(T36, T37);

        T50 = _mm_hadd_epi32(T40, T41);
        T51 = _mm_hadd_epi32(T42, T43);
        T50 = _mm_srai_epi32(_mm_add_epi32(T50, c_8), SHIFT_32T_1ST);
        T51 = _mm_srai_epi32(_mm_add_epi32(T51, c_8), SHIFT_32T_1ST);
        T60 = _mm_packs_epi32(T50, T51);
        im[8][i] = T60;

        T30 = _mm_madd_epi16(T20, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T31 = _mm_madd_epi16(T21, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T32 = _mm_madd_epi16(T22, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T33 = _mm_madd_epi16(T23, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T34 = _mm_madd_epi16(T24, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T35 = _mm_madd_epi16(T25, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T36 = _mm_madd_epi16(T26, _mm_load_si128((__m128i*)tab_dct_16_1[9]));
        T37 = _mm_madd_epi16(T27, _mm_load_si128((__m128i*)tab_dct_16_1[9]));

        T40 = _mm_hadd_epi32(T30, T31);
        T41 = _mm_hadd_epi32(T32, T33);
        T42 = _mm_hadd_epi32(T34, T35);
        T43 = _mm_hadd_epi32(T36, T37);

        T50 = _mm_hadd_epi32(T40, T41);
        T51 = _mm_hadd_epi32(T42, T43);
        T50 = _mm_srai_epi32(_mm_add_epi32(T50, c_8), SHIFT_32T_1ST);
        T51 = _mm_srai_epi32(_mm_add_epi32(T51, c_8), SHIFT_32T_1ST);
        T60 = _mm_packs_epi32(T50, T51);
        im[24][i] = T60;

#define MAKE_ODD(tab, dstPos) \
  T30  = _mm_madd_epi16(T20, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T31  = _mm_madd_epi16(T21, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T32  = _mm_madd_epi16(T22, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T33  = _mm_madd_epi16(T23, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T34  = _mm_madd_epi16(T24, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T35  = _mm_madd_epi16(T25, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T36  = _mm_madd_epi16(T26, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T37  = _mm_madd_epi16(T27, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  \
  T40  = _mm_hadd_epi32(T30, T31); \
  T41  = _mm_hadd_epi32(T32, T33); \
  T42  = _mm_hadd_epi32(T34, T35); \
  T43  = _mm_hadd_epi32(T36, T37); \
  \
  T50  = _mm_hadd_epi32(T40, T41); \
  T51  = _mm_hadd_epi32(T42, T43); \
  T50  = _mm_srai_epi32(_mm_add_epi32(T50, c_8), SHIFT_32T_1ST); \
  T51  = _mm_srai_epi32(_mm_add_epi32(T51, c_8), SHIFT_32T_1ST); \
  T60  = _mm_packs_epi32(T50, T51); \
  im[(dstPos)][i] = T60;

        MAKE_ODD(0, 4);
        MAKE_ODD(1, 12);
        MAKE_ODD(2, 20);
        MAKE_ODD(3, 28);

        T20 = _mm_sub_epi16(T10A, T10B);   // [EO5 EO2 EO6 EO1 EO4 EO3 EO7 EO0]
        T21 = _mm_sub_epi16(T11A, T11B);
        T22 = _mm_sub_epi16(T12A, T12B);
        T23 = _mm_sub_epi16(T13A, T13B);
        T24 = _mm_sub_epi16(T14A, T14B);
        T25 = _mm_sub_epi16(T15A, T15B);
        T26 = _mm_sub_epi16(T16A, T16B);
        T27 = _mm_sub_epi16(T17A, T17B);

        MAKE_ODD(4, 2);
        MAKE_ODD(5, 6);
        MAKE_ODD(6, 10);
        MAKE_ODD(7, 14);
        MAKE_ODD(8, 18);
        MAKE_ODD(9, 22);
        MAKE_ODD(10, 26);
        MAKE_ODD(11, 30);
#undef MAKE_ODD

#define MAKE_ODD(tab, dstPos) \
  T20  = _mm_madd_epi16(T00A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T21  = _mm_madd_epi16(T00B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  T22  = _mm_madd_epi16(T01A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T23  = _mm_madd_epi16(T01B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  T24  = _mm_madd_epi16(T02A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T25  = _mm_madd_epi16(T02B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  T26  = _mm_madd_epi16(T03A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T27  = _mm_madd_epi16(T03B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  T30  = _mm_madd_epi16(T04A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T31  = _mm_madd_epi16(T04B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  T32  = _mm_madd_epi16(T05A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T33  = _mm_madd_epi16(T05B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  T34  = _mm_madd_epi16(T06A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T35  = _mm_madd_epi16(T06B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  T36  = _mm_madd_epi16(T07A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab)])); \
  T37  = _mm_madd_epi16(T07B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab) + 1])); \
  \
  T40  = _mm_hadd_epi32(T20, T21); \
  T41  = _mm_hadd_epi32(T22, T23); \
  T42  = _mm_hadd_epi32(T24, T25); \
  T43  = _mm_hadd_epi32(T26, T27); \
  T44  = _mm_hadd_epi32(T30, T31); \
  T45  = _mm_hadd_epi32(T32, T33); \
  T46  = _mm_hadd_epi32(T34, T35); \
  T47  = _mm_hadd_epi32(T36, T37); \
  \
  T50  = _mm_hadd_epi32(T40, T41); \
  T51  = _mm_hadd_epi32(T42, T43); \
  T52  = _mm_hadd_epi32(T44, T45); \
  T53  = _mm_hadd_epi32(T46, T47); \
  \
  T50  = _mm_hadd_epi32(T50, T51); \
  T51  = _mm_hadd_epi32(T52, T53); \
  T50  = _mm_srai_epi32(_mm_add_epi32(T50, c_8), SHIFT_32T_1ST); \
  T51  = _mm_srai_epi32(_mm_add_epi32(T51, c_8), SHIFT_32T_1ST); \
  T60  = _mm_packs_epi32(T50, T51); \
  im[(dstPos)][i] = T60;

        MAKE_ODD(12, 1);
        MAKE_ODD(14, 3);
        MAKE_ODD(16, 5);
        MAKE_ODD(18, 7);
        MAKE_ODD(20, 9);
        MAKE_ODD(22, 11);
        MAKE_ODD(24, 13);
        MAKE_ODD(26, 15);
        MAKE_ODD(28, 17);
        MAKE_ODD(30, 19);
        MAKE_ODD(32, 21);
        MAKE_ODD(34, 23);
        MAKE_ODD(36, 25);
        MAKE_ODD(38, 27);
        MAKE_ODD(40, 29);
        MAKE_ODD(42, 31);

#undef MAKE_ODD
    }

    // DCT2
    for (i = 0; i < 32 / 4; i++)
    {
        // OPT_ME: to avoid register spill, I use matrix multiply, have other way?
        T00A = im[i * 4 + 0][0];    // [07 06 05 04 03 02 01 00]
        T00B = im[i * 4 + 0][1];    // [15 14 13 12 11 10 09 08]
        T00C = im[i * 4 + 0][2];    // [23 22 21 20 19 18 17 16]
        T00D = im[i * 4 + 0][3];    // [31 30 29 28 27 26 25 24]
        T01A = im[i * 4 + 1][0];
        T01B = im[i * 4 + 1][1];
        T01C = im[i * 4 + 1][2];
        T01D = im[i * 4 + 1][3];
        T02A = im[i * 4 + 2][0];
        T02B = im[i * 4 + 2][1];
        T02C = im[i * 4 + 2][2];
        T02D = im[i * 4 + 2][3];
        T03A = im[i * 4 + 3][0];
        T03B = im[i * 4 + 3][1];
        T03C = im[i * 4 + 3][2];
        T03D = im[i * 4 + 3][3];

        T00C = _mm_shuffle_epi8(T00C, _mm_load_si128((__m128i*)tab_dct_16_0[0]));    // [16 17 18 19 20 21 22 23]
        T00D = _mm_shuffle_epi8(T00D, _mm_load_si128((__m128i*)tab_dct_16_0[0]));    // [24 25 26 27 28 29 30 31]
        T01C = _mm_shuffle_epi8(T01C, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T01D = _mm_shuffle_epi8(T01D, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T02C = _mm_shuffle_epi8(T02C, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T02D = _mm_shuffle_epi8(T02D, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T03C = _mm_shuffle_epi8(T03C, _mm_load_si128((__m128i*)tab_dct_16_0[0]));
        T03D = _mm_shuffle_epi8(T03D, _mm_load_si128((__m128i*)tab_dct_16_0[0]));

        T10A = _mm_unpacklo_epi16(T00A, T00D);  // [28 03 29 02 30 01 31 00]
        T10B = _mm_unpackhi_epi16(T00A, T00D);  // [24 07 25 06 26 05 27 04]
        T00A = _mm_unpacklo_epi16(T00B, T00C);  // [20 11 21 10 22 09 23 08]
        T00B = _mm_unpackhi_epi16(T00B, T00C);  // [16 15 17 14 18 13 19 12]
        T11A = _mm_unpacklo_epi16(T01A, T01D);
        T11B = _mm_unpackhi_epi16(T01A, T01D);
        T01A = _mm_unpacklo_epi16(T01B, T01C);
        T01B = _mm_unpackhi_epi16(T01B, T01C);
        T12A = _mm_unpacklo_epi16(T02A, T02D);
        T12B = _mm_unpackhi_epi16(T02A, T02D);
        T02A = _mm_unpacklo_epi16(T02B, T02C);
        T02B = _mm_unpackhi_epi16(T02B, T02C);
        T13A = _mm_unpacklo_epi16(T03A, T03D);
        T13B = _mm_unpackhi_epi16(T03A, T03D);
        T03A = _mm_unpacklo_epi16(T03B, T03C);
        T03B = _mm_unpackhi_epi16(T03B, T03C);

#define MAKE_ODD(tab0, tab1, tab2, tab3, dstPos) \
  T20  = _mm_madd_epi16(T10A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab0)])); \
  T21  = _mm_madd_epi16(T10B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab1)])); \
  T22  = _mm_madd_epi16(T00A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab2)])); \
  T23  = _mm_madd_epi16(T00B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab3)])); \
  T24  = _mm_madd_epi16(T11A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab0)])); \
  T25  = _mm_madd_epi16(T11B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab1)])); \
  T26  = _mm_madd_epi16(T01A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab2)])); \
  T27  = _mm_madd_epi16(T01B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab3)])); \
  T30  = _mm_madd_epi16(T12A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab0)])); \
  T31  = _mm_madd_epi16(T12B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab1)])); \
  T32  = _mm_madd_epi16(T02A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab2)])); \
  T33  = _mm_madd_epi16(T02B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab3)])); \
  T34  = _mm_madd_epi16(T13A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab0)])); \
  T35  = _mm_madd_epi16(T13B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab1)])); \
  T36  = _mm_madd_epi16(T03A, _mm_load_si128((__m128i*)tab_dct_32_1[(tab2)])); \
  T37  = _mm_madd_epi16(T03B, _mm_load_si128((__m128i*)tab_dct_32_1[(tab3)])); \
  \
  T60  = _mm_hadd_epi32(T20, T21); \
  T61  = _mm_hadd_epi32(T22, T23); \
  T62  = _mm_hadd_epi32(T24, T25); \
  T63  = _mm_hadd_epi32(T26, T27); \
  T64  = _mm_hadd_epi32(T30, T31); \
  T65  = _mm_hadd_epi32(T32, T33); \
  T66  = _mm_hadd_epi32(T34, T35); \
  T67  = _mm_hadd_epi32(T36, T37); \
  \
  T60  = _mm_hadd_epi32(T60, T61); \
  T61  = _mm_hadd_epi32(T62, T63); \
  T62  = _mm_hadd_epi32(T64, T65); \
  T63  = _mm_hadd_epi32(T66, T67); \
  \
  T60  = _mm_hadd_epi32(T60, T61); \
  T61  = _mm_hadd_epi32(T62, T63); \
  \
  T60  = _mm_hadd_epi32(T60, T61); \
  \
  T60  = _mm_srai_epi32(_mm_add_epi32(T60, c_1024), SHIFT_32T_2ND); \
  T60  = _mm_packs_epi32(T60, T60); \
  _mm_storel_epi64((__m128i*)&dst[(dstPos) * 32 + (i * 4) + 0], T60); \

        MAKE_ODD(44, 44, 44, 44, 0);
        MAKE_ODD(45, 45, 45, 45, 16);
        MAKE_ODD(46, 47, 46, 47, 8);
        MAKE_ODD(48, 49, 48, 49, 24);

        MAKE_ODD(50, 51, 52, 53, 4);
        MAKE_ODD(54, 55, 56, 57, 12);
        MAKE_ODD(58, 59, 60, 61, 20);
        MAKE_ODD(62, 63, 64, 65, 28);

        MAKE_ODD(66, 67, 68, 69, 2);
        MAKE_ODD(70, 71, 72, 73, 6);
        MAKE_ODD(74, 75, 76, 77, 10);
        MAKE_ODD(78, 79, 80, 81, 14);

        MAKE_ODD(82, 83, 84, 85, 18);
        MAKE_ODD(86, 87, 88, 89, 22);
        MAKE_ODD(90, 91, 92, 93, 26);
        MAKE_ODD(94, 95, 96, 97, 30);

        MAKE_ODD(98, 99, 100, 101, 1);
        MAKE_ODD(102, 103, 104, 105, 3);
        MAKE_ODD(106, 107, 108, 109, 5);
        MAKE_ODD(110, 111, 112, 113, 7);
        MAKE_ODD(114, 115, 116, 117, 9);
        MAKE_ODD(118, 119, 120, 121, 11);
        MAKE_ODD(122, 123, 124, 125, 13);
        MAKE_ODD(126, 127, 128, 129, 15);
        MAKE_ODD(130, 131, 132, 133, 17);
        MAKE_ODD(134, 135, 136, 137, 19);
        MAKE_ODD(138, 139, 140, 141, 21);
        MAKE_ODD(142, 143, 144, 145, 23);
        MAKE_ODD(146, 147, 148, 149, 25);
        MAKE_ODD(150, 151, 152, 153, 27);
        MAKE_ODD(154, 155, 156, 157, 29);
        MAKE_ODD(158, 159, 160, 161, 31);
#undef MAKE_ODD
    }

}
static void IDCT4(Pel* pResidual, short* plCoeff, UInt uiDstStride, UInt uiSrcStride, Int shift)
{
    const __m128i xmm_add = _mm_set1_epi32(1 << (shift - 1));

    __m128i       xmm_s[4];
    __m128i       xmm_c[2];
    __m128i       xmm_t[2];

#if !_ETRI_WINDOWS_APPLICATION
    __m128i       xmm_zero = _mm_setzero_si128();
#endif 

    // load 16bit*16comp
    xmm_c[0] = _mm_loadu_si128((const __m128i*) g_aiIT4[0]);
    xmm_c[1] = _mm_loadu_si128((const __m128i*) g_aiIT4[2]);

    // load 16bit*16comp
    xmm_s[0] = _mm_loadl_epi64((const __m128i*) (plCoeff));
    xmm_s[1] = _mm_loadl_epi64((const __m128i*) (plCoeff + uiSrcStride));
    xmm_s[2] = _mm_loadl_epi64((const __m128i*) (plCoeff + uiSrcStride * 2));
    xmm_s[3] = _mm_loadl_epi64((const __m128i*) (plCoeff + uiSrcStride * 3));

    // trans pose
    xmm_t[0] = _mm_unpacklo_epi16(xmm_s[0], xmm_s[1]);
    xmm_t[1] = _mm_unpacklo_epi16(xmm_s[2], xmm_s[3]);
    xmm_s[2] = _mm_unpacklo_epi32(xmm_t[0], xmm_t[1]);
    xmm_s[3] = _mm_unpackhi_epi32(xmm_t[0], xmm_t[1]);

    xmm_s[0] = _mm_unpacklo_epi64(xmm_s[2], xmm_s[2]);
    xmm_s[1] = _mm_unpackhi_epi64(xmm_s[2], xmm_s[2]);
    xmm_s[2] = _mm_unpacklo_epi64(xmm_s[3], xmm_s[3]);
    xmm_s[3] = _mm_unpackhi_epi64(xmm_s[3], xmm_s[3]);

    // calc
    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[0]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[0]);
    xmm_s[0] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[1]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[1]);
    xmm_s[1] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[2]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[2]);
    xmm_s[2] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[3]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[3]);
    xmm_s[3] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    // scale down
    xmm_s[0] = _mm_add_epi32(xmm_s[0], xmm_add);
    xmm_s[0] = _mm_srai_epi32(xmm_s[0], shift);

    xmm_s[1] = _mm_add_epi32(xmm_s[1], xmm_add);
    xmm_s[1] = _mm_srai_epi32(xmm_s[1], shift);

    xmm_s[2] = _mm_add_epi32(xmm_s[2], xmm_add);
    xmm_s[2] = _mm_srai_epi32(xmm_s[2], shift);

    xmm_s[3] = _mm_add_epi32(xmm_s[3], xmm_add);
    xmm_s[3] = _mm_srai_epi32(xmm_s[3], shift);

    // signed saturate 32bit->16bit
    xmm_s[0] = _mm_packs_epi32(xmm_s[0], xmm_s[1]);
    xmm_s[1] = _mm_packs_epi32(xmm_s[2], xmm_s[3]);

    // store
#if _ETRI_WINDOWS_APPLICATION
    *(int64_t *)(pResidual) = xmm_s[0].m128i_i64[0];
    *(int64_t *)(pResidual + uiDstStride) = xmm_s[0].m128i_i64[1];
    *(int64_t *)(pResidual + uiDstStride * 2) = xmm_s[1].m128i_i64[0];
    *(int64_t *)(pResidual + uiDstStride * 3) = xmm_s[1].m128i_i64[1];
#else
    _mm_storel_epi64((__m128i*)(pResidual), xmm_s[0]);
    xmm_t[0] = _mm_unpackhi_epi16(xmm_s[0], _mm_cmplt_epi16(xmm_s[0], xmm_zero));
    xmm_t[1] = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(pResidual + uiDstStride), xmm_t[1]);

    _mm_storel_epi64((__m128i*)(pResidual + uiDstStride * 2), xmm_s[1]);
    xmm_t[0] = _mm_unpackhi_epi16(xmm_s[1], _mm_cmplt_epi16(xmm_s[1], xmm_zero));
    xmm_t[1] = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(pResidual + uiDstStride * 3), xmm_t[1]);
#endif 
}
static void IDST4(Pel* pResidual, Short* plCoeff, UInt uiDstStride, UInt uiSrcStride, Int shift)
{
    const __m128i xmm_add = _mm_set1_epi32(1 << (shift - 1));

    __m128i       xmm_s[4];
    __m128i       xmm_c[2];
    __m128i       xmm_t[2];

#if !_ETRI_WINDOWS_APPLICATION
    __m128i       xmm_zero = _mm_setzero_si128();
#endif 
    xmm_c[0] = _mm_loadu_si128((__m128i*) g_as_IDST_MAT_4[0]);
    xmm_c[1] = _mm_loadu_si128((__m128i*) g_as_IDST_MAT_4[2]);

    xmm_s[0] = _mm_loadl_epi64((__m128i*) (plCoeff));
    xmm_s[1] = _mm_loadl_epi64((__m128i*) (plCoeff + uiSrcStride));
    xmm_s[2] = _mm_loadl_epi64((__m128i*) (plCoeff + uiSrcStride * 2));
    xmm_s[3] = _mm_loadl_epi64((__m128i*) (plCoeff + uiSrcStride * 3));

    // trans pose
    xmm_t[0] = _mm_unpacklo_epi16(xmm_s[0], xmm_s[1]);
    xmm_t[1] = _mm_unpacklo_epi16(xmm_s[2], xmm_s[3]);
    xmm_s[2] = _mm_unpacklo_epi32(xmm_t[0], xmm_t[1]);
    xmm_s[3] = _mm_unpackhi_epi32(xmm_t[0], xmm_t[1]);

    xmm_s[0] = _mm_unpacklo_epi64(xmm_s[2], xmm_s[2]);
    xmm_s[1] = _mm_unpackhi_epi64(xmm_s[2], xmm_s[2]);
    xmm_s[2] = _mm_unpacklo_epi64(xmm_s[3], xmm_s[3]);
    xmm_s[3] = _mm_unpackhi_epi64(xmm_s[3], xmm_s[3]);

    // calc
    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[0]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[0]);
    xmm_s[0] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[1]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[1]);
    xmm_s[1] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[2]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[2]);
    xmm_s[2] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    xmm_t[0] = _mm_madd_epi16(xmm_c[0], xmm_s[3]);
    xmm_t[1] = _mm_madd_epi16(xmm_c[1], xmm_s[3]);
    xmm_s[3] = _mm_hadd_epi32(xmm_t[0], xmm_t[1]);

    // scale down
    xmm_s[0] = _mm_add_epi32(xmm_s[0], xmm_add);
    xmm_s[0] = _mm_srai_epi32(xmm_s[0], shift);

    xmm_s[1] = _mm_add_epi32(xmm_s[1], xmm_add);
    xmm_s[1] = _mm_srai_epi32(xmm_s[1], shift);

    xmm_s[2] = _mm_add_epi32(xmm_s[2], xmm_add);
    xmm_s[2] = _mm_srai_epi32(xmm_s[2], shift);

    xmm_s[3] = _mm_add_epi32(xmm_s[3], xmm_add);
    xmm_s[3] = _mm_srai_epi32(xmm_s[3], shift);

    // signed saturate 32bit->16bit
    xmm_s[0] = _mm_packs_epi32(xmm_s[0], xmm_s[1]);
    xmm_s[1] = _mm_packs_epi32(xmm_s[2], xmm_s[3]);

    // store
#if _ETRI_WINDOWS_APPLICATION
    *(int64_t *)(pResidual) = xmm_s[0].m128i_i64[0];
    *(int64_t *)(pResidual + uiDstStride) = xmm_s[0].m128i_i64[1];
    *(int64_t *)(pResidual + uiDstStride * 2) = xmm_s[1].m128i_i64[0];
    *(int64_t *)(pResidual + uiDstStride * 3) = xmm_s[1].m128i_i64[1];
#else 
    _mm_storel_epi64((__m128i*)(pResidual), xmm_s[0]);
    xmm_t[0] = _mm_unpackhi_epi16(xmm_s[0], _mm_cmplt_epi16(xmm_s[0], xmm_zero));
    xmm_t[1] = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(pResidual + uiDstStride), xmm_t[1]);

    _mm_storel_epi64((__m128i*)(pResidual + uiDstStride * 2), xmm_s[1]);
    xmm_t[0] = _mm_unpackhi_epi16(xmm_s[1], _mm_cmplt_epi16(xmm_s[1], xmm_zero));
    xmm_t[1] = _mm_packs_epi32(xmm_t[0], xmm_zero);
    _mm_storel_epi64((__m128i*)(pResidual + uiDstStride * 3), xmm_t[1]);
#endif 
}
static void IDCT8(const int16_t* src, int16_t* dst, intptr_t stride)
{
    __m128i m128iS0, m128iS1, m128iS2, m128iS3, m128iS4, m128iS5, m128iS6, m128iS7, m128iAdd, m128Tmp0, m128Tmp1, m128Tmp2, m128Tmp3, E0h, E1h, E2h, E3h, E0l, E1l, E2l, E3l, O0h, O1h, O2h, O3h, O0l, O1l, O2l, O3l, EE0l, EE1l, E00l, E01l, EE0h, EE1h, E00h, E01h;
    __m128i T00, T01, T02, T03, T04, T05, T06, T07;

    m128iAdd = _mm_set1_epi32(ADD_IT_1);

    m128iS1 = _mm_load_si128((__m128i*)&src[8 + 0]);
    m128iS3 = _mm_load_si128((__m128i*)&src[24 + 0]);
    m128Tmp0 = _mm_unpacklo_epi16(m128iS1, m128iS3);
    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[0])));
    m128Tmp1 = _mm_unpackhi_epi16(m128iS1, m128iS3);
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[0])));

    m128iS5 = _mm_load_si128((__m128i*)&src[40 + 0]);
    m128iS7 = _mm_load_si128((__m128i*)&src[56 + 0]);
    m128Tmp2 = _mm_unpacklo_epi16(m128iS5, m128iS7);
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[1])));
    m128Tmp3 = _mm_unpackhi_epi16(m128iS5, m128iS7);
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[1])));
    O0l = _mm_add_epi32(E1l, E2l);
    O0h = _mm_add_epi32(E1h, E2h);

    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[2])));
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[2])));
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[3])));
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[3])));

    O1l = _mm_add_epi32(E1l, E2l);
    O1h = _mm_add_epi32(E1h, E2h);

    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[4])));
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[4])));
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[5])));
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[5])));
    O2l = _mm_add_epi32(E1l, E2l);
    O2h = _mm_add_epi32(E1h, E2h);

    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[6])));
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[6])));
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[7])));
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[7])));
    O3h = _mm_add_epi32(E1h, E2h);
    O3l = _mm_add_epi32(E1l, E2l);

    /*    -------     */

    m128iS0 = _mm_load_si128((__m128i*)&src[0 + 0]);
    m128iS4 = _mm_load_si128((__m128i*)&src[32 + 0]);
    m128Tmp0 = _mm_unpacklo_epi16(m128iS0, m128iS4);
    EE0l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[8])));
    m128Tmp1 = _mm_unpackhi_epi16(m128iS0, m128iS4);
    EE0h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[8])));

    EE1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[9])));
    EE1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[9])));

    /*    -------     */

    m128iS2 = _mm_load_si128((__m128i*)&src[16 + 0]);
    m128iS6 = _mm_load_si128((__m128i*)&src[48 + 0]);
    m128Tmp0 = _mm_unpacklo_epi16(m128iS2, m128iS6);
    E00l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[10])));
    m128Tmp1 = _mm_unpackhi_epi16(m128iS2, m128iS6);
    E00h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[10])));
    E01l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[11])));
    E01h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[11])));
    E0l = _mm_add_epi32(EE0l, E00l);
    E0l = _mm_add_epi32(E0l, m128iAdd);
    E0h = _mm_add_epi32(EE0h, E00h);
    E0h = _mm_add_epi32(E0h, m128iAdd);
    E3l = _mm_sub_epi32(EE0l, E00l);
    E3l = _mm_add_epi32(E3l, m128iAdd);
    E3h = _mm_sub_epi32(EE0h, E00h);
    E3h = _mm_add_epi32(E3h, m128iAdd);

    E1l = _mm_add_epi32(EE1l, E01l);
    E1l = _mm_add_epi32(E1l, m128iAdd);
    E1h = _mm_add_epi32(EE1h, E01h);
    E1h = _mm_add_epi32(E1h, m128iAdd);
    E2l = _mm_sub_epi32(EE1l, E01l);
    E2l = _mm_add_epi32(E2l, m128iAdd);
    E2h = _mm_sub_epi32(EE1h, E01h);
    E2h = _mm_add_epi32(E2h, m128iAdd);
    m128iS0 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E0l, O0l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_add_epi32(E0h, O0h), SHIFT_IT_1ST));
    m128iS1 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E1l, O1l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_add_epi32(E1h, O1h), SHIFT_IT_1ST));
    m128iS2 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E2l, O2l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_add_epi32(E2h, O2h), SHIFT_IT_1ST));
    m128iS3 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E3l, O3l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_add_epi32(E3h, O3h), SHIFT_IT_1ST));
    m128iS4 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E3l, O3l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_sub_epi32(E3h, O3h), SHIFT_IT_1ST));
    m128iS5 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E2l, O2l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_sub_epi32(E2h, O2h), SHIFT_IT_1ST));
    m128iS6 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E1l, O1l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_sub_epi32(E1h, O1h), SHIFT_IT_1ST));
    m128iS7 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E0l, O0l), SHIFT_IT_1ST), _mm_srai_epi32(_mm_sub_epi32(E0h, O0h), SHIFT_IT_1ST));
    /*  Invers matrix   */

    E0l = _mm_unpacklo_epi16(m128iS0, m128iS4);
    E1l = _mm_unpacklo_epi16(m128iS1, m128iS5);
    E2l = _mm_unpacklo_epi16(m128iS2, m128iS6);
    E3l = _mm_unpacklo_epi16(m128iS3, m128iS7);
    O0l = _mm_unpackhi_epi16(m128iS0, m128iS4);
    O1l = _mm_unpackhi_epi16(m128iS1, m128iS5);
    O2l = _mm_unpackhi_epi16(m128iS2, m128iS6);
    O3l = _mm_unpackhi_epi16(m128iS3, m128iS7);
    m128Tmp0 = _mm_unpacklo_epi16(E0l, E2l);
    m128Tmp1 = _mm_unpacklo_epi16(E1l, E3l);
    m128iS0 = _mm_unpacklo_epi16(m128Tmp0, m128Tmp1);
    m128iS1 = _mm_unpackhi_epi16(m128Tmp0, m128Tmp1);
    m128Tmp2 = _mm_unpackhi_epi16(E0l, E2l);
    m128Tmp3 = _mm_unpackhi_epi16(E1l, E3l);
    m128iS2 = _mm_unpacklo_epi16(m128Tmp2, m128Tmp3);
    m128iS3 = _mm_unpackhi_epi16(m128Tmp2, m128Tmp3);
    m128Tmp0 = _mm_unpacklo_epi16(O0l, O2l);
    m128Tmp1 = _mm_unpacklo_epi16(O1l, O3l);
    m128iS4 = _mm_unpacklo_epi16(m128Tmp0, m128Tmp1);
    m128iS5 = _mm_unpackhi_epi16(m128Tmp0, m128Tmp1);
    m128Tmp2 = _mm_unpackhi_epi16(O0l, O2l);
    m128Tmp3 = _mm_unpackhi_epi16(O1l, O3l);
    m128iS6 = _mm_unpacklo_epi16(m128Tmp2, m128Tmp3);
    m128iS7 = _mm_unpackhi_epi16(m128Tmp2, m128Tmp3);

    m128iAdd = _mm_set1_epi32(ADD_IT_2);

    m128Tmp0 = _mm_unpacklo_epi16(m128iS1, m128iS3);
    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[0])));
    m128Tmp1 = _mm_unpackhi_epi16(m128iS1, m128iS3);
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[0])));
    m128Tmp2 = _mm_unpacklo_epi16(m128iS5, m128iS7);
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[1])));
    m128Tmp3 = _mm_unpackhi_epi16(m128iS5, m128iS7);
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[1])));
    O0l = _mm_add_epi32(E1l, E2l);
    O0h = _mm_add_epi32(E1h, E2h);
    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[2])));
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[2])));
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[3])));
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[3])));
    O1l = _mm_add_epi32(E1l, E2l);
    O1h = _mm_add_epi32(E1h, E2h);
    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[4])));
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[4])));
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[5])));
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[5])));
    O2l = _mm_add_epi32(E1l, E2l);
    O2h = _mm_add_epi32(E1h, E2h);
    E1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[6])));
    E1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[6])));
    E2l = _mm_madd_epi16(m128Tmp2, _mm_load_si128((__m128i*)(tab_idct_8x8[7])));
    E2h = _mm_madd_epi16(m128Tmp3, _mm_load_si128((__m128i*)(tab_idct_8x8[7])));
    O3h = _mm_add_epi32(E1h, E2h);
    O3l = _mm_add_epi32(E1l, E2l);

    m128Tmp0 = _mm_unpacklo_epi16(m128iS0, m128iS4);
    EE0l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[8])));
    m128Tmp1 = _mm_unpackhi_epi16(m128iS0, m128iS4);
    EE0h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[8])));
    EE1l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[9])));
    EE1h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[9])));

    m128Tmp0 = _mm_unpacklo_epi16(m128iS2, m128iS6);
    E00l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[10])));
    m128Tmp1 = _mm_unpackhi_epi16(m128iS2, m128iS6);
    E00h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[10])));
    E01l = _mm_madd_epi16(m128Tmp0, _mm_load_si128((__m128i*)(tab_idct_8x8[11])));
    E01h = _mm_madd_epi16(m128Tmp1, _mm_load_si128((__m128i*)(tab_idct_8x8[11])));
    E0l = _mm_add_epi32(EE0l, E00l);
    E0l = _mm_add_epi32(E0l, m128iAdd);
    E0h = _mm_add_epi32(EE0h, E00h);
    E0h = _mm_add_epi32(E0h, m128iAdd);
    E3l = _mm_sub_epi32(EE0l, E00l);
    E3l = _mm_add_epi32(E3l, m128iAdd);
    E3h = _mm_sub_epi32(EE0h, E00h);
    E3h = _mm_add_epi32(E3h, m128iAdd);
    E1l = _mm_add_epi32(EE1l, E01l);
    E1l = _mm_add_epi32(E1l, m128iAdd);
    E1h = _mm_add_epi32(EE1h, E01h);
    E1h = _mm_add_epi32(E1h, m128iAdd);
    E2l = _mm_sub_epi32(EE1l, E01l);
    E2l = _mm_add_epi32(E2l, m128iAdd);
    E2h = _mm_sub_epi32(EE1h, E01h);
    E2h = _mm_add_epi32(E2h, m128iAdd);

    m128iS0 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E0l, O0l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_add_epi32(E0h, O0h), SHIFT_IT_2ND));
    m128iS1 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E1l, O1l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_add_epi32(E1h, O1h), SHIFT_IT_2ND));
    m128iS2 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E2l, O2l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_add_epi32(E2h, O2h), SHIFT_IT_2ND));
    m128iS3 = _mm_packs_epi32(_mm_srai_epi32(_mm_add_epi32(E3l, O3l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_add_epi32(E3h, O3h), SHIFT_IT_2ND));
    m128iS4 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E3l, O3l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_sub_epi32(E3h, O3h), SHIFT_IT_2ND));
    m128iS5 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E2l, O2l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_sub_epi32(E2h, O2h), SHIFT_IT_2ND));
    m128iS6 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E1l, O1l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_sub_epi32(E1h, O1h), SHIFT_IT_2ND));
    m128iS7 = _mm_packs_epi32(_mm_srai_epi32(_mm_sub_epi32(E0l, O0l), SHIFT_IT_2ND), _mm_srai_epi32(_mm_sub_epi32(E0h, O0h), SHIFT_IT_2ND));

    // [07 06 05 04 03 02 01 00]
    // [17 16 15 14 13 12 11 10]
    // [27 26 25 24 23 22 21 20]
    // [37 36 35 34 33 32 31 30]
    // [47 46 45 44 43 42 41 40]
    // [57 56 55 54 53 52 51 50]
    // [67 66 65 64 63 62 61 60]
    // [77 76 75 74 73 72 71 70]

    T00 = _mm_unpacklo_epi16(m128iS0, m128iS1);     // [13 03 12 02 11 01 10 00]
    T01 = _mm_unpackhi_epi16(m128iS0, m128iS1);     // [17 07 16 06 15 05 14 04]
    T02 = _mm_unpacklo_epi16(m128iS2, m128iS3);     // [33 23 32 22 31 21 30 20]
    T03 = _mm_unpackhi_epi16(m128iS2, m128iS3);     // [37 27 36 26 35 25 34 24]
    T04 = _mm_unpacklo_epi16(m128iS4, m128iS5);     // [53 43 52 42 51 41 50 40]
    T05 = _mm_unpackhi_epi16(m128iS4, m128iS5);     // [57 47 56 46 55 45 54 44]
    T06 = _mm_unpacklo_epi16(m128iS6, m128iS7);     // [73 63 72 62 71 61 70 60]
    T07 = _mm_unpackhi_epi16(m128iS6, m128iS7);     // [77 67 76 66 75 65 74 64]

    __m128i T10, T11;
    T10 = _mm_unpacklo_epi32(T00, T02);                                     // [31 21 11 01 30 20 10 00]
    T11 = _mm_unpackhi_epi32(T00, T02);                                     // [33 23 13 03 32 22 12 02]
    _mm_storel_epi64((__m128i*)&dst[0 * stride + 0], T10);                   // [30 20 10 00]
    _mm_storeh_pi((__m64*)&dst[1 * stride + 0], _mm_castsi128_ps(T10));  // [31 21 11 01]
    _mm_storel_epi64((__m128i*)&dst[2 * stride + 0], T11);                   // [32 22 12 02]
    _mm_storeh_pi((__m64*)&dst[3 * stride + 0], _mm_castsi128_ps(T11));  // [33 23 13 03]

    T10 = _mm_unpacklo_epi32(T04, T06);                                     // [71 61 51 41 70 60 50 40]
    T11 = _mm_unpackhi_epi32(T04, T06);                                     // [73 63 53 43 72 62 52 42]
    _mm_storel_epi64((__m128i*)&dst[0 * stride + 4], T10);
    _mm_storeh_pi((__m64*)&dst[1 * stride + 4], _mm_castsi128_ps(T10));
    _mm_storel_epi64((__m128i*)&dst[2 * stride + 4], T11);
    _mm_storeh_pi((__m64*)&dst[3 * stride + 4], _mm_castsi128_ps(T11));

    T10 = _mm_unpacklo_epi32(T01, T03);                                     // [35 25 15 05 34 24 14 04]
    T11 = _mm_unpackhi_epi32(T01, T03);                                     // [37 27 17 07 36 26 16 06]
    _mm_storel_epi64((__m128i*)&dst[4 * stride + 0], T10);
    _mm_storeh_pi((__m64*)&dst[5 * stride + 0], _mm_castsi128_ps(T10));
    _mm_storel_epi64((__m128i*)&dst[6 * stride + 0], T11);
    _mm_storeh_pi((__m64*)&dst[7 * stride + 0], _mm_castsi128_ps(T11));

    T10 = _mm_unpacklo_epi32(T05, T07);                                     // [75 65 55 45 74 64 54 44]
    T11 = _mm_unpackhi_epi32(T05, T07);                                     // [77 67 57 47 76 56 46 36]
    _mm_storel_epi64((__m128i*)&dst[4 * stride + 4], T10);
    _mm_storeh_pi((__m64*)&dst[5 * stride + 4], _mm_castsi128_ps(T10));
    _mm_storel_epi64((__m128i*)&dst[6 * stride + 4], T11);
    _mm_storeh_pi((__m64*)&dst[7 * stride + 4], _mm_castsi128_ps(T11));

}
static void IDCT16(const short *src, short *dst, UInt stride)
{
#define READ_UNPACKHILO(offset)\
  const __m128i T_00_00A = _mm_unpacklo_epi16(*(__m128i*)&src[1 * 16 + offset], *(__m128i*)&src[3 * 16 + offset]);\
  const __m128i T_00_00B = _mm_unpackhi_epi16(*(__m128i*)&src[1 * 16 + offset], *(__m128i*)&src[3 * 16 + offset]);\
  const __m128i T_00_01A = _mm_unpacklo_epi16(*(__m128i*)&src[5 * 16 + offset], *(__m128i*)&src[7 * 16 + offset]);\
  const __m128i T_00_01B = _mm_unpackhi_epi16(*(__m128i*)&src[5 * 16 + offset], *(__m128i*)&src[7 * 16 + offset]);\
  const __m128i T_00_02A = _mm_unpacklo_epi16(*(__m128i*)&src[9 * 16 + offset], *(__m128i*)&src[11 * 16 + offset]);\
  const __m128i T_00_02B = _mm_unpackhi_epi16(*(__m128i*)&src[9 * 16 + offset], *(__m128i*)&src[11 * 16 + offset]);\
  const __m128i T_00_03A = _mm_unpacklo_epi16(*(__m128i*)&src[13 * 16 + offset], *(__m128i*)&src[15 * 16 + offset]);\
  const __m128i T_00_03B = _mm_unpackhi_epi16(*(__m128i*)&src[13 * 16 + offset], *(__m128i*)&src[15 * 16 + offset]);\
  const __m128i T_00_04A = _mm_unpacklo_epi16(*(__m128i*)&src[2 * 16 + offset], *(__m128i*)&src[6 * 16 + offset]);\
  const __m128i T_00_04B = _mm_unpackhi_epi16(*(__m128i*)&src[2 * 16 + offset], *(__m128i*)&src[6 * 16 + offset]);\
  const __m128i T_00_05A = _mm_unpacklo_epi16(*(__m128i*)&src[10 * 16 + offset], *(__m128i*)&src[14 * 16 + offset]);\
  const __m128i T_00_05B = _mm_unpackhi_epi16(*(__m128i*)&src[10 * 16 + offset], *(__m128i*)&src[14 * 16 + offset]);\
  const __m128i T_00_06A = _mm_unpacklo_epi16(*(__m128i*)&src[4 * 16 + offset], *(__m128i*)&src[12 * 16 + offset]);\
  const __m128i T_00_06B = _mm_unpackhi_epi16(*(__m128i*)&src[4 * 16 + offset], *(__m128i*)&src[12 * 16 + offset]);\
  const __m128i T_00_07A = _mm_unpacklo_epi16(*(__m128i*)&src[0 * 16 + offset], *(__m128i*)&src[8 * 16 + offset]);\
  const __m128i T_00_07B = _mm_unpackhi_epi16(*(__m128i*)&src[0 * 16 + offset], *(__m128i*)&src[8 * 16 + offset]);

#define UNPACKHILO(part) \
  const __m128i T_00_00A = _mm_unpacklo_epi16(in01[part], in03[part]);\
  const __m128i T_00_00B = _mm_unpackhi_epi16(in01[part], in03[part]);\
  const __m128i T_00_01A = _mm_unpacklo_epi16(in05[part], in07[part]);\
  const __m128i T_00_01B = _mm_unpackhi_epi16(in05[part], in07[part]);\
  const __m128i T_00_02A = _mm_unpacklo_epi16(in09[part], in11[part]);\
  const __m128i T_00_02B = _mm_unpackhi_epi16(in09[part], in11[part]);\
  const __m128i T_00_03A = _mm_unpacklo_epi16(in13[part], in15[part]);\
  const __m128i T_00_03B = _mm_unpackhi_epi16(in13[part], in15[part]);\
  const __m128i T_00_04A = _mm_unpacklo_epi16(in02[part], in06[part]);\
  const __m128i T_00_04B = _mm_unpackhi_epi16(in02[part], in06[part]);\
  const __m128i T_00_05A = _mm_unpacklo_epi16(in10[part], in14[part]);\
  const __m128i T_00_05B = _mm_unpackhi_epi16(in10[part], in14[part]);\
  const __m128i T_00_06A = _mm_unpacklo_epi16(in04[part], in12[part]);\
  const __m128i T_00_06B = _mm_unpackhi_epi16(in04[part], in12[part]);\
  const __m128i T_00_07A = _mm_unpacklo_epi16(in00[part], in08[part]);\
  const __m128i T_00_07B = _mm_unpackhi_epi16(in00[part], in08[part]);

#define COMPUTE_ROW_16(row0103, row0507, row0911, row1315, c0103, c0507, c0911, c1315, row) \
  T00 = _mm_add_epi32(_mm_madd_epi16(row0103, c0103), _mm_madd_epi16(row0507, c0507)); \
  T01 = _mm_add_epi32(_mm_madd_epi16(row0911, c0911), _mm_madd_epi16(row1315, c1315)); \
  row = _mm_add_epi32(T00, T01);

#define TRANSPOSE_8x8_16BIT(I0, I1, I2, I3, I4, I5, I6, I7, O0, O1, O2, O3, O4, O5, O6, O7) \
  tr0_0 = _mm_unpacklo_epi16(I0, I1); \
  tr0_1 = _mm_unpacklo_epi16(I2, I3); \
  tr0_2 = _mm_unpackhi_epi16(I0, I1); \
  tr0_3 = _mm_unpackhi_epi16(I2, I3); \
  tr0_4 = _mm_unpacklo_epi16(I4, I5); \
  tr0_5 = _mm_unpacklo_epi16(I6, I7); \
  tr0_6 = _mm_unpackhi_epi16(I4, I5); \
  tr0_7 = _mm_unpackhi_epi16(I6, I7); \
  tr1_0 = _mm_unpacklo_epi32(tr0_0, tr0_1); \
  tr1_1 = _mm_unpacklo_epi32(tr0_2, tr0_3); \
  tr1_2 = _mm_unpackhi_epi32(tr0_0, tr0_1); \
  tr1_3 = _mm_unpackhi_epi32(tr0_2, tr0_3); \
  tr1_4 = _mm_unpacklo_epi32(tr0_4, tr0_5); \
  tr1_5 = _mm_unpacklo_epi32(tr0_6, tr0_7); \
  tr1_6 = _mm_unpackhi_epi32(tr0_4, tr0_5); \
  tr1_7 = _mm_unpackhi_epi32(tr0_6, tr0_7); \
  O0 = _mm_unpacklo_epi64(tr1_0, tr1_4); \
  O1 = _mm_unpackhi_epi64(tr1_0, tr1_4); \
  O2 = _mm_unpacklo_epi64(tr1_2, tr1_6); \
  O3 = _mm_unpackhi_epi64(tr1_2, tr1_6); \
  O4 = _mm_unpacklo_epi64(tr1_1, tr1_5); \
  O5 = _mm_unpackhi_epi64(tr1_1, tr1_5); \
  O6 = _mm_unpacklo_epi64(tr1_3, tr1_7); \
  O7 = _mm_unpackhi_epi64(tr1_3, tr1_7);

#define PROCESS(part, rnd, shift) \
  __m128i c32_rnd = _mm_set1_epi32(rnd);\
  int nShift = shift;\
  \
  __m128i O0A, O1A, O2A, O3A, O4A, O5A, O6A, O7A;\
  __m128i O0B, O1B, O2B, O3B, O4B, O5B, O6B, O7B;\
  {\
  __m128i T00, T01;\
  \
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_p87_p90, c16_p70_p80, c16_p43_p57, c16_p09_p25, O0A)\
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_p57_p87, c16_n43_p09, c16_n90_n80, c16_n25_n70, O1A)\
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_p09_p80, c16_n87_n70, c16_p57_n25, c16_p43_p90, O2A)\
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_n43_p70, c16_p09_n87, c16_p25_p90, c16_n57_n80, O3A)\
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_n80_p57, c16_p90_n25, c16_n87_n09, c16_p70_p43, O4A)\
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_n90_p43, c16_p25_p57, c16_p70_n87, c16_n80_p09, O5A)\
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_n70_p25, c16_n80_p90, c16_p09_p43, c16_p87_n57, O6A)\
  COMPUTE_ROW_16(T_00_00A, T_00_01A, T_00_02A, T_00_03A, c16_n25_p09, c16_n57_p43, c16_n80_p70, c16_n90_p87, O7A)\
  \
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_p87_p90, c16_p70_p80, c16_p43_p57, c16_p09_p25, O0B)\
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_p57_p87, c16_n43_p09, c16_n90_n80, c16_n25_n70, O1B)\
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_p09_p80, c16_n87_n70, c16_p57_n25, c16_p43_p90, O2B)\
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_n43_p70, c16_p09_n87, c16_p25_p90, c16_n57_n80, O3B)\
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_n80_p57, c16_p90_n25, c16_n87_n09, c16_p70_p43, O4B)\
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_n90_p43, c16_p25_p57, c16_p70_n87, c16_n80_p09, O5B)\
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_n70_p25, c16_n80_p90, c16_p09_p43, c16_p87_n57, O6B)\
  COMPUTE_ROW_16(T_00_00B, T_00_01B, T_00_02B, T_00_03B, c16_n25_p09, c16_n57_p43, c16_n80_p70, c16_n90_p87, O7B)\
}\
  \
  __m128i EO0A, EO1A, EO2A, EO3A;\
  __m128i EO0B, EO1B, EO2B, EO3B;\
  EO0A = _mm_add_epi32(_mm_madd_epi16(T_00_04A, c16_p75_p89), _mm_madd_epi16(T_00_05A, c16_p18_p50));\
  EO0B = _mm_add_epi32(_mm_madd_epi16(T_00_04B, c16_p75_p89), _mm_madd_epi16(T_00_05B, c16_p18_p50));\
  EO1A = _mm_add_epi32(_mm_madd_epi16(T_00_04A, c16_n18_p75), _mm_madd_epi16(T_00_05A, c16_n50_n89));\
  EO1B = _mm_add_epi32(_mm_madd_epi16(T_00_04B, c16_n18_p75), _mm_madd_epi16(T_00_05B, c16_n50_n89));\
  EO2A = _mm_add_epi32(_mm_madd_epi16(T_00_04A, c16_n89_p50), _mm_madd_epi16(T_00_05A, c16_p75_p18));\
  EO2B = _mm_add_epi32(_mm_madd_epi16(T_00_04B, c16_n89_p50), _mm_madd_epi16(T_00_05B, c16_p75_p18));\
  EO3A = _mm_add_epi32(_mm_madd_epi16(T_00_04A, c16_n50_p18), _mm_madd_epi16(T_00_05A, c16_n89_p75));\
  EO3B = _mm_add_epi32(_mm_madd_epi16(T_00_04B, c16_n50_p18), _mm_madd_epi16(T_00_05B, c16_n89_p75));\
  \
  __m128i EEO0A, EEO1A;\
  __m128i EEO0B, EEO1B;\
  EEO0A = _mm_madd_epi16(T_00_06A, c16_p36_p83);\
  EEO0B = _mm_madd_epi16(T_00_06B, c16_p36_p83);\
  EEO1A = _mm_madd_epi16(T_00_06A, c16_n83_p36);\
  EEO1B = _mm_madd_epi16(T_00_06B, c16_n83_p36);\
  \
  __m128i EEE0A, EEE1A;\
  __m128i EEE0B, EEE1B;\
  EEE0A = _mm_madd_epi16(T_00_07A, c16_p64_p64);\
  EEE0B = _mm_madd_epi16(T_00_07B, c16_p64_p64);\
  EEE1A = _mm_madd_epi16(T_00_07A, c16_n64_p64);\
  EEE1B = _mm_madd_epi16(T_00_07B, c16_n64_p64);\
  \
  const __m128i EE0A = _mm_add_epi32(EEE0A, EEO0A);\
  const __m128i EE0B = _mm_add_epi32(EEE0B, EEO0B);\
  const __m128i EE1A = _mm_add_epi32(EEE1A, EEO1A);\
  const __m128i EE1B = _mm_add_epi32(EEE1B, EEO1B);\
  const __m128i EE3A = _mm_sub_epi32(EEE0A, EEO0A);\
  const __m128i EE3B = _mm_sub_epi32(EEE0B, EEO0B);\
  const __m128i EE2A = _mm_sub_epi32(EEE1A, EEO1A);\
  const __m128i EE2B = _mm_sub_epi32(EEE1B, EEO1B);\
  \
  const __m128i E0A = _mm_add_epi32(EE0A, EO0A);\
  const __m128i E0B = _mm_add_epi32(EE0B, EO0B);\
  const __m128i E1A = _mm_add_epi32(EE1A, EO1A);\
  const __m128i E1B = _mm_add_epi32(EE1B, EO1B);\
  const __m128i E2A = _mm_add_epi32(EE2A, EO2A);\
  const __m128i E2B = _mm_add_epi32(EE2B, EO2B);\
  const __m128i E3A = _mm_add_epi32(EE3A, EO3A);\
  const __m128i E3B = _mm_add_epi32(EE3B, EO3B);\
  const __m128i E7A = _mm_sub_epi32(EE0A, EO0A);\
  const __m128i E7B = _mm_sub_epi32(EE0B, EO0B);\
  const __m128i E6A = _mm_sub_epi32(EE1A, EO1A);\
  const __m128i E6B = _mm_sub_epi32(EE1B, EO1B);\
  const __m128i E5A = _mm_sub_epi32(EE2A, EO2A);\
  const __m128i E5B = _mm_sub_epi32(EE2B, EO2B);\
  const __m128i E4A = _mm_sub_epi32(EE3A, EO3A);\
  const __m128i E4B = _mm_sub_epi32(EE3B, EO3B);\
  \
  const __m128i T10A = _mm_add_epi32(E0A, c32_rnd);\
  const __m128i T10B = _mm_add_epi32(E0B, c32_rnd);\
  const __m128i T11A = _mm_add_epi32(E1A, c32_rnd);\
  const __m128i T11B = _mm_add_epi32(E1B, c32_rnd);\
  const __m128i T12A = _mm_add_epi32(E2A, c32_rnd);\
  const __m128i T12B = _mm_add_epi32(E2B, c32_rnd);\
  const __m128i T13A = _mm_add_epi32(E3A, c32_rnd);\
  const __m128i T13B = _mm_add_epi32(E3B, c32_rnd);\
  const __m128i T14A = _mm_add_epi32(E4A, c32_rnd);\
  const __m128i T14B = _mm_add_epi32(E4B, c32_rnd);\
  const __m128i T15A = _mm_add_epi32(E5A, c32_rnd);\
  const __m128i T15B = _mm_add_epi32(E5B, c32_rnd);\
  const __m128i T16A = _mm_add_epi32(E6A, c32_rnd);\
  const __m128i T16B = _mm_add_epi32(E6B, c32_rnd);\
  const __m128i T17A = _mm_add_epi32(E7A, c32_rnd);\
  const __m128i T17B = _mm_add_epi32(E7B, c32_rnd);\
  \
  const __m128i T20A = _mm_add_epi32(T10A, O0A);\
  const __m128i T20B = _mm_add_epi32(T10B, O0B);\
  const __m128i T21A = _mm_add_epi32(T11A, O1A);\
  const __m128i T21B = _mm_add_epi32(T11B, O1B);\
  const __m128i T22A = _mm_add_epi32(T12A, O2A);\
  const __m128i T22B = _mm_add_epi32(T12B, O2B);\
  const __m128i T23A = _mm_add_epi32(T13A, O3A);\
  const __m128i T23B = _mm_add_epi32(T13B, O3B);\
  const __m128i T24A = _mm_add_epi32(T14A, O4A);\
  const __m128i T24B = _mm_add_epi32(T14B, O4B);\
  const __m128i T25A = _mm_add_epi32(T15A, O5A);\
  const __m128i T25B = _mm_add_epi32(T15B, O5B);\
  const __m128i T26A = _mm_add_epi32(T16A, O6A);\
  const __m128i T26B = _mm_add_epi32(T16B, O6B);\
  const __m128i T27A = _mm_add_epi32(T17A, O7A);\
  const __m128i T27B = _mm_add_epi32(T17B, O7B);\
  const __m128i T2FA = _mm_sub_epi32(T10A, O0A);\
  const __m128i T2FB = _mm_sub_epi32(T10B, O0B);\
  const __m128i T2EA = _mm_sub_epi32(T11A, O1A);\
  const __m128i T2EB = _mm_sub_epi32(T11B, O1B);\
  const __m128i T2DA = _mm_sub_epi32(T12A, O2A);\
  const __m128i T2DB = _mm_sub_epi32(T12B, O2B);\
  const __m128i T2CA = _mm_sub_epi32(T13A, O3A);\
  const __m128i T2CB = _mm_sub_epi32(T13B, O3B);\
  const __m128i T2BA = _mm_sub_epi32(T14A, O4A);\
  const __m128i T2BB = _mm_sub_epi32(T14B, O4B);\
  const __m128i T2AA = _mm_sub_epi32(T15A, O5A);\
  const __m128i T2AB = _mm_sub_epi32(T15B, O5B);\
  const __m128i T29A = _mm_sub_epi32(T16A, O6A);\
  const __m128i T29B = _mm_sub_epi32(T16B, O6B);\
  const __m128i T28A = _mm_sub_epi32(T17A, O7A);\
  const __m128i T28B = _mm_sub_epi32(T17B, O7B);\
  \
  const __m128i T30A = _mm_srai_epi32(T20A, nShift);\
  const __m128i T30B = _mm_srai_epi32(T20B, nShift);\
  const __m128i T31A = _mm_srai_epi32(T21A, nShift);\
  const __m128i T31B = _mm_srai_epi32(T21B, nShift);\
  const __m128i T32A = _mm_srai_epi32(T22A, nShift);\
  const __m128i T32B = _mm_srai_epi32(T22B, nShift);\
  const __m128i T33A = _mm_srai_epi32(T23A, nShift);\
  const __m128i T33B = _mm_srai_epi32(T23B, nShift);\
  const __m128i T34A = _mm_srai_epi32(T24A, nShift);\
  const __m128i T34B = _mm_srai_epi32(T24B, nShift);\
  const __m128i T35A = _mm_srai_epi32(T25A, nShift);\
  const __m128i T35B = _mm_srai_epi32(T25B, nShift);\
  const __m128i T36A = _mm_srai_epi32(T26A, nShift);\
  const __m128i T36B = _mm_srai_epi32(T26B, nShift);\
  const __m128i T37A = _mm_srai_epi32(T27A, nShift);\
  const __m128i T37B = _mm_srai_epi32(T27B, nShift);\
  \
  const __m128i T38A = _mm_srai_epi32(T28A, nShift);\
  const __m128i T38B = _mm_srai_epi32(T28B, nShift);\
  const __m128i T39A = _mm_srai_epi32(T29A, nShift);\
  const __m128i T39B = _mm_srai_epi32(T29B, nShift);\
  const __m128i T3AA = _mm_srai_epi32(T2AA, nShift);\
  const __m128i T3AB = _mm_srai_epi32(T2AB, nShift);\
  const __m128i T3BA = _mm_srai_epi32(T2BA, nShift);\
  const __m128i T3BB = _mm_srai_epi32(T2BB, nShift);\
  const __m128i T3CA = _mm_srai_epi32(T2CA, nShift);\
  const __m128i T3CB = _mm_srai_epi32(T2CB, nShift);\
  const __m128i T3DA = _mm_srai_epi32(T2DA, nShift);\
  const __m128i T3DB = _mm_srai_epi32(T2DB, nShift);\
  const __m128i T3EA = _mm_srai_epi32(T2EA, nShift);\
  const __m128i T3EB = _mm_srai_epi32(T2EB, nShift);\
  const __m128i T3FA = _mm_srai_epi32(T2FA, nShift);\
  const __m128i T3FB = _mm_srai_epi32(T2FB, nShift);\
  \
  res00[part]  = _mm_packs_epi32(T30A, T30B);\
  res01[part]  = _mm_packs_epi32(T31A, T31B);\
  res02[part]  = _mm_packs_epi32(T32A, T32B);\
  res03[part]  = _mm_packs_epi32(T33A, T33B);\
  res04[part]  = _mm_packs_epi32(T34A, T34B);\
  res05[part]  = _mm_packs_epi32(T35A, T35B);\
  res06[part]  = _mm_packs_epi32(T36A, T36B);\
  res07[part]  = _mm_packs_epi32(T37A, T37B);\
  \
  res08[part]  = _mm_packs_epi32(T38A, T38B);\
  res09[part]  = _mm_packs_epi32(T39A, T39B);\
  res10[part]  = _mm_packs_epi32(T3AA, T3AB);\
  res11[part]  = _mm_packs_epi32(T3BA, T3BB);\
  res12[part]  = _mm_packs_epi32(T3CA, T3CB);\
  res13[part]  = _mm_packs_epi32(T3DA, T3DB);\
  res14[part]  = _mm_packs_epi32(T3EA, T3EB);\
  res15[part]  = _mm_packs_epi32(T3FA, T3FB);

    const __m128i c16_p87_p90 = _mm_set1_epi32(0x0057005A); //row0 87high - 90low address
    const __m128i c16_p70_p80 = _mm_set1_epi32(0x00460050);
    const __m128i c16_p43_p57 = _mm_set1_epi32(0x002B0039);
    const __m128i c16_p09_p25 = _mm_set1_epi32(0x00090019);
    const __m128i c16_p57_p87 = _mm_set1_epi32(0x00390057); //row1
    const __m128i c16_n43_p09 = _mm_set1_epi32(0xFFD50009);
    const __m128i c16_n90_n80 = _mm_set1_epi32(0xFFA6FFB0);
    const __m128i c16_n25_n70 = _mm_set1_epi32(0xFFE7FFBA);
    const __m128i c16_p09_p80 = _mm_set1_epi32(0x00090050); //row2
    const __m128i c16_n87_n70 = _mm_set1_epi32(0xFFA9FFBA);
    const __m128i c16_p57_n25 = _mm_set1_epi32(0x0039FFE7);
    const __m128i c16_p43_p90 = _mm_set1_epi32(0x002B005A);
    const __m128i c16_n43_p70 = _mm_set1_epi32(0xFFD50046); //row3
    const __m128i c16_p09_n87 = _mm_set1_epi32(0x0009FFA9);
    const __m128i c16_p25_p90 = _mm_set1_epi32(0x0019005A);
    const __m128i c16_n57_n80 = _mm_set1_epi32(0xFFC7FFB0);
    const __m128i c16_n80_p57 = _mm_set1_epi32(0xFFB00039); //row4
    const __m128i c16_p90_n25 = _mm_set1_epi32(0x005AFFE7);
    const __m128i c16_n87_n09 = _mm_set1_epi32(0xFFA9FFF7);
    const __m128i c16_p70_p43 = _mm_set1_epi32(0x0046002B);
    const __m128i c16_n90_p43 = _mm_set1_epi32(0xFFA6002B); //row5
    const __m128i c16_p25_p57 = _mm_set1_epi32(0x00190039);
    const __m128i c16_p70_n87 = _mm_set1_epi32(0x0046FFA9);
    const __m128i c16_n80_p09 = _mm_set1_epi32(0xFFB00009);
    const __m128i c16_n70_p25 = _mm_set1_epi32(0xFFBA0019); //row6
    const __m128i c16_n80_p90 = _mm_set1_epi32(0xFFB0005A);
    const __m128i c16_p09_p43 = _mm_set1_epi32(0x0009002B);
    const __m128i c16_p87_n57 = _mm_set1_epi32(0x0057FFC7);
    const __m128i c16_n25_p09 = _mm_set1_epi32(0xFFE70009); //row7
    const __m128i c16_n57_p43 = _mm_set1_epi32(0xFFC7002B);
    const __m128i c16_n80_p70 = _mm_set1_epi32(0xFFB00046);
    const __m128i c16_n90_p87 = _mm_set1_epi32(0xFFA60057);

    const __m128i c16_p75_p89 = _mm_set1_epi32(0x004B0059);
    const __m128i c16_p18_p50 = _mm_set1_epi32(0x00120032);
    const __m128i c16_n18_p75 = _mm_set1_epi32(0xFFEE004B);
    const __m128i c16_n50_n89 = _mm_set1_epi32(0xFFCEFFA7);
    const __m128i c16_n89_p50 = _mm_set1_epi32(0xFFA70032);
    const __m128i c16_p75_p18 = _mm_set1_epi32(0x004B0012);
    const __m128i c16_n50_p18 = _mm_set1_epi32(0xFFCE0012);
    const __m128i c16_n89_p75 = _mm_set1_epi32(0xFFA7004B);

    const __m128i c16_p36_p83 = _mm_set1_epi32(0x00240053);
    const __m128i c16_n83_p36 = _mm_set1_epi32(0xFFAD0024);

    const __m128i c16_n64_p64 = _mm_set1_epi32(0xFFC00040);
    const __m128i c16_p64_p64 = _mm_set1_epi32(0x00400040);

    // DCT1
    __m128i in00[2], in01[2], in02[2], in03[2], in04[2], in05[2], in06[2], in07[2];
    __m128i in08[2], in09[2], in10[2], in11[2], in12[2], in13[2], in14[2], in15[2];
    __m128i res00[2], res01[2], res02[2], res03[2], res04[2], res05[2], res06[2], res07[2];
    __m128i res08[2], res09[2], res10[2], res11[2], res12[2], res13[2], res14[2], res15[2];

    {
        READ_UNPACKHILO(0)
            PROCESS(0, ADD_IT_1, SHIFT_IT_1ST)
    }

  {
      READ_UNPACKHILO(8)
          PROCESS(1, ADD_IT_1, SHIFT_IT_1ST)
  }
  {
      __m128i tr0_0, tr0_1, tr0_2, tr0_3, tr0_4, tr0_5, tr0_6, tr0_7;
      __m128i tr1_0, tr1_1, tr1_2, tr1_3, tr1_4, tr1_5, tr1_6, tr1_7;
      TRANSPOSE_8x8_16BIT(res00[0], res01[0], res02[0], res03[0], res04[0], res05[0], res06[0], res07[0], in00[0], in01[0], in02[0], in03[0], in04[0], in05[0], in06[0], in07[0])
          TRANSPOSE_8x8_16BIT(res08[0], res09[0], res10[0], res11[0], res12[0], res13[0], res14[0], res15[0], in00[1], in01[1], in02[1], in03[1], in04[1], in05[1], in06[1], in07[1])
          TRANSPOSE_8x8_16BIT(res00[1], res01[1], res02[1], res03[1], res04[1], res05[1], res06[1], res07[1], in08[0], in09[0], in10[0], in11[0], in12[0], in13[0], in14[0], in15[0])
          TRANSPOSE_8x8_16BIT(res08[1], res09[1], res10[1], res11[1], res12[1], res13[1], res14[1], res15[1], in08[1], in09[1], in10[1], in11[1], in12[1], in13[1], in14[1], in15[1])
  }

  {
      UNPACKHILO(0)
          PROCESS(0, ADD_IT_2, SHIFT_IT_2ND)
  }
  {
      UNPACKHILO(1)
          PROCESS(1, ADD_IT_2, SHIFT_IT_2ND)
  }

  {
      __m128i tr0_0, tr0_1, tr0_2, tr0_3, tr0_4, tr0_5, tr0_6, tr0_7;
      __m128i tr1_0, tr1_1, tr1_2, tr1_3, tr1_4, tr1_5, tr1_6, tr1_7;
      TRANSPOSE_8x8_16BIT(res00[0], res01[0], res02[0], res03[0], res04[0], res05[0], res06[0], res07[0], in00[0], in01[0], in02[0], in03[0], in04[0], in05[0], in06[0], in07[0])
          _mm_store_si128((__m128i*)&dst[0 * stride + 0], in00[0]);
      _mm_store_si128((__m128i*)&dst[1 * stride + 0], in01[0]);
      _mm_store_si128((__m128i*)&dst[2 * stride + 0], in02[0]);
      _mm_store_si128((__m128i*)&dst[3 * stride + 0], in03[0]);
      _mm_store_si128((__m128i*)&dst[4 * stride + 0], in04[0]);
      _mm_store_si128((__m128i*)&dst[5 * stride + 0], in05[0]);
      _mm_store_si128((__m128i*)&dst[6 * stride + 0], in06[0]);
      _mm_store_si128((__m128i*)&dst[7 * stride + 0], in07[0]);
      TRANSPOSE_8x8_16BIT(res08[0], res09[0], res10[0], res11[0], res12[0], res13[0], res14[0], res15[0], in00[1], in01[1], in02[1], in03[1], in04[1], in05[1], in06[1], in07[1])
          _mm_store_si128((__m128i*)&dst[0 * stride + 8], in00[1]);
      _mm_store_si128((__m128i*)&dst[1 * stride + 8], in01[1]);
      _mm_store_si128((__m128i*)&dst[2 * stride + 8], in02[1]);
      _mm_store_si128((__m128i*)&dst[3 * stride + 8], in03[1]);
      _mm_store_si128((__m128i*)&dst[4 * stride + 8], in04[1]);
      _mm_store_si128((__m128i*)&dst[5 * stride + 8], in05[1]);
      _mm_store_si128((__m128i*)&dst[6 * stride + 8], in06[1]);
      _mm_store_si128((__m128i*)&dst[7 * stride + 8], in07[1]);
      TRANSPOSE_8x8_16BIT(res00[1], res01[1], res02[1], res03[1], res04[1], res05[1], res06[1], res07[1], in08[0], in09[0], in10[0], in11[0], in12[0], in13[0], in14[0], in15[0])
          _mm_store_si128((__m128i*)&dst[8 * stride + 0], in08[0]);
      _mm_store_si128((__m128i*)&dst[9 * stride + 0], in09[0]);
      _mm_store_si128((__m128i*)&dst[10 * stride + 0], in10[0]);
      _mm_store_si128((__m128i*)&dst[11 * stride + 0], in11[0]);
      _mm_store_si128((__m128i*)&dst[12 * stride + 0], in12[0]);
      _mm_store_si128((__m128i*)&dst[13 * stride + 0], in13[0]);
      _mm_store_si128((__m128i*)&dst[14 * stride + 0], in14[0]);
      _mm_store_si128((__m128i*)&dst[15 * stride + 0], in15[0]);
      TRANSPOSE_8x8_16BIT(res08[1], res09[1], res10[1], res11[1], res12[1], res13[1], res14[1], res15[1], in08[1], in09[1], in10[1], in11[1], in12[1], in13[1], in14[1], in15[1])
          _mm_store_si128((__m128i*)&dst[8 * stride + 8], in08[1]);
      _mm_store_si128((__m128i*)&dst[9 * stride + 8], in09[1]);
      _mm_store_si128((__m128i*)&dst[10 * stride + 8], in10[1]);
      _mm_store_si128((__m128i*)&dst[11 * stride + 8], in11[1]);
      _mm_store_si128((__m128i*)&dst[12 * stride + 8], in12[1]);
      _mm_store_si128((__m128i*)&dst[13 * stride + 8], in13[1]);
      _mm_store_si128((__m128i*)&dst[14 * stride + 8], in14[1]);
      _mm_store_si128((__m128i*)&dst[15 * stride + 8], in15[1]);
  }
}
static void IDCT32(const short *src, short *dst, UInt stride)
{

    //Odd
    const __m128i c16_p90_p90 = _mm_set1_epi32(0x005A005A); //column 0
    const __m128i c16_p85_p88 = _mm_set1_epi32(0x00550058);
    const __m128i c16_p78_p82 = _mm_set1_epi32(0x004E0052);
    const __m128i c16_p67_p73 = _mm_set1_epi32(0x00430049);
    const __m128i c16_p54_p61 = _mm_set1_epi32(0x0036003D);
    const __m128i c16_p38_p46 = _mm_set1_epi32(0x0026002E);
    const __m128i c16_p22_p31 = _mm_set1_epi32(0x0016001F);
    const __m128i c16_p04_p13 = _mm_set1_epi32(0x0004000D);
    const __m128i c16_p82_p90 = _mm_set1_epi32(0x0052005A); //column 1
    const __m128i c16_p46_p67 = _mm_set1_epi32(0x002E0043);
    const __m128i c16_n04_p22 = _mm_set1_epi32(0xFFFC0016);
    const __m128i c16_n54_n31 = _mm_set1_epi32(0xFFCAFFE1);
    const __m128i c16_n85_n73 = _mm_set1_epi32(0xFFABFFB7);
    const __m128i c16_n88_n90 = _mm_set1_epi32(0xFFA8FFA6);
    const __m128i c16_n61_n78 = _mm_set1_epi32(0xFFC3FFB2);
    const __m128i c16_n13_n38 = _mm_set1_epi32(0xFFF3FFDA);
    const __m128i c16_p67_p88 = _mm_set1_epi32(0x00430058); //column 2
    const __m128i c16_n13_p31 = _mm_set1_epi32(0xFFF3001F);
    const __m128i c16_n82_n54 = _mm_set1_epi32(0xFFAEFFCA);
    const __m128i c16_n78_n90 = _mm_set1_epi32(0xFFB2FFA6);
    const __m128i c16_n04_n46 = _mm_set1_epi32(0xFFFCFFD2);
    const __m128i c16_p73_p38 = _mm_set1_epi32(0x00490026);
    const __m128i c16_p85_p90 = _mm_set1_epi32(0x0055005A);
    const __m128i c16_p22_p61 = _mm_set1_epi32(0x0016003D);
    const __m128i c16_p46_p85 = _mm_set1_epi32(0x002E0055); //column 3
    const __m128i c16_n67_n13 = _mm_set1_epi32(0xFFBDFFF3);
    const __m128i c16_n73_n90 = _mm_set1_epi32(0xFFB7FFA6);
    const __m128i c16_p38_n22 = _mm_set1_epi32(0x0026FFEA);
    const __m128i c16_p88_p82 = _mm_set1_epi32(0x00580052);
    const __m128i c16_n04_p54 = _mm_set1_epi32(0xFFFC0036);
    const __m128i c16_n90_n61 = _mm_set1_epi32(0xFFA6FFC3);
    const __m128i c16_n31_n78 = _mm_set1_epi32(0xFFE1FFB2);
    const __m128i c16_p22_p82 = _mm_set1_epi32(0x00160052); //column 4
    const __m128i c16_n90_n54 = _mm_set1_epi32(0xFFA6FFCA);
    const __m128i c16_p13_n61 = _mm_set1_epi32(0x000DFFC3);
    const __m128i c16_p85_p78 = _mm_set1_epi32(0x0055004E);
    const __m128i c16_n46_p31 = _mm_set1_epi32(0xFFD2001F);
    const __m128i c16_n67_n90 = _mm_set1_epi32(0xFFBDFFA6);
    const __m128i c16_p73_p04 = _mm_set1_epi32(0x00490004);
    const __m128i c16_p38_p88 = _mm_set1_epi32(0x00260058);
    const __m128i c16_n04_p78 = _mm_set1_epi32(0xFFFC004E); //column 5
    const __m128i c16_n73_n82 = _mm_set1_epi32(0xFFB7FFAE);
    const __m128i c16_p85_p13 = _mm_set1_epi32(0x0055000D);
    const __m128i c16_n22_p67 = _mm_set1_epi32(0xFFEA0043);
    const __m128i c16_n61_n88 = _mm_set1_epi32(0xFFC3FFA8);
    const __m128i c16_p90_p31 = _mm_set1_epi32(0x005A001F);
    const __m128i c16_n38_p54 = _mm_set1_epi32(0xFFDA0036);
    const __m128i c16_n46_n90 = _mm_set1_epi32(0xFFD2FFA6);
    const __m128i c16_n31_p73 = _mm_set1_epi32(0xFFE10049); //column 6
    const __m128i c16_n22_n90 = _mm_set1_epi32(0xFFEAFFA6);
    const __m128i c16_p67_p78 = _mm_set1_epi32(0x0043004E);
    const __m128i c16_n90_n38 = _mm_set1_epi32(0xFFA6FFDA);
    const __m128i c16_p82_n13 = _mm_set1_epi32(0x0052FFF3);
    const __m128i c16_n46_p61 = _mm_set1_epi32(0xFFD2003D);
    const __m128i c16_n04_n88 = _mm_set1_epi32(0xFFFCFFA8);
    const __m128i c16_p54_p85 = _mm_set1_epi32(0x00360055);
    const __m128i c16_n54_p67 = _mm_set1_epi32(0xFFCA0043); //column 7
    const __m128i c16_p38_n78 = _mm_set1_epi32(0x0026FFB2);
    const __m128i c16_n22_p85 = _mm_set1_epi32(0xFFEA0055);
    const __m128i c16_p04_n90 = _mm_set1_epi32(0x0004FFA6);
    const __m128i c16_p13_p90 = _mm_set1_epi32(0x000D005A);
    const __m128i c16_n31_n88 = _mm_set1_epi32(0xFFE1FFA8);
    const __m128i c16_p46_p82 = _mm_set1_epi32(0x002E0052);
    const __m128i c16_n61_n73 = _mm_set1_epi32(0xFFC3FFB7);
    const __m128i c16_n73_p61 = _mm_set1_epi32(0xFFB7003D); //column 8
    const __m128i c16_p82_n46 = _mm_set1_epi32(0x0052FFD2);
    const __m128i c16_n88_p31 = _mm_set1_epi32(0xFFA8001F);
    const __m128i c16_p90_n13 = _mm_set1_epi32(0x005AFFF3);
    const __m128i c16_n90_n04 = _mm_set1_epi32(0xFFA6FFFC);
    const __m128i c16_p85_p22 = _mm_set1_epi32(0x00550016);
    const __m128i c16_n78_n38 = _mm_set1_epi32(0xFFB2FFDA);
    const __m128i c16_p67_p54 = _mm_set1_epi32(0x00430036);
    const __m128i c16_n85_p54 = _mm_set1_epi32(0xFFAB0036); //column 9
    const __m128i c16_p88_n04 = _mm_set1_epi32(0x0058FFFC);
    const __m128i c16_n61_n46 = _mm_set1_epi32(0xFFC3FFD2);
    const __m128i c16_p13_p82 = _mm_set1_epi32(0x000D0052);
    const __m128i c16_p38_n90 = _mm_set1_epi32(0x0026FFA6);
    const __m128i c16_n78_p67 = _mm_set1_epi32(0xFFB20043);
    const __m128i c16_p90_n22 = _mm_set1_epi32(0x005AFFEA);
    const __m128i c16_n73_n31 = _mm_set1_epi32(0xFFB7FFE1);
    const __m128i c16_n90_p46 = _mm_set1_epi32(0xFFA6002E); //column 10
    const __m128i c16_p54_p38 = _mm_set1_epi32(0x00360026);
    const __m128i c16_p31_n90 = _mm_set1_epi32(0x001FFFA6);
    const __m128i c16_n88_p61 = _mm_set1_epi32(0xFFA8003D);
    const __m128i c16_p67_p22 = _mm_set1_epi32(0x00430016);
    const __m128i c16_p13_n85 = _mm_set1_epi32(0x000DFFAB);
    const __m128i c16_n82_p73 = _mm_set1_epi32(0xFFAE0049);
    const __m128i c16_p78_p04 = _mm_set1_epi32(0x004E0004);
    const __m128i c16_n88_p38 = _mm_set1_epi32(0xFFA80026); //column 11
    const __m128i c16_n04_p73 = _mm_set1_epi32(0xFFFC0049);
    const __m128i c16_p90_n67 = _mm_set1_epi32(0x005AFFBD);
    const __m128i c16_n31_n46 = _mm_set1_epi32(0xFFE1FFD2);
    const __m128i c16_n78_p85 = _mm_set1_epi32(0xFFB20055);
    const __m128i c16_p61_p13 = _mm_set1_epi32(0x003D000D);
    const __m128i c16_p54_n90 = _mm_set1_epi32(0x0036FFA6);
    const __m128i c16_n82_p22 = _mm_set1_epi32(0xFFAE0016);
    const __m128i c16_n78_p31 = _mm_set1_epi32(0xFFB2001F); //column 12
    const __m128i c16_n61_p90 = _mm_set1_epi32(0xFFC3005A);
    const __m128i c16_p54_p04 = _mm_set1_epi32(0x00360004);
    const __m128i c16_p82_n88 = _mm_set1_epi32(0x0052FFA8);
    const __m128i c16_n22_n38 = _mm_set1_epi32(0xFFEAFFDA);
    const __m128i c16_n90_p73 = _mm_set1_epi32(0xFFA60049);
    const __m128i c16_n13_p67 = _mm_set1_epi32(0xFFF30043);
    const __m128i c16_p85_n46 = _mm_set1_epi32(0x0055FFD2);
    const __m128i c16_n61_p22 = _mm_set1_epi32(0xFFC30016); //column 13
    const __m128i c16_n90_p85 = _mm_set1_epi32(0xFFA60055);
    const __m128i c16_n38_p73 = _mm_set1_epi32(0xFFDA0049);
    const __m128i c16_p46_n04 = _mm_set1_epi32(0x002EFFFC);
    const __m128i c16_p90_n78 = _mm_set1_epi32(0x005AFFB2);
    const __m128i c16_p54_n82 = _mm_set1_epi32(0x0036FFAE);
    const __m128i c16_n31_n13 = _mm_set1_epi32(0xFFE1FFF3);
    const __m128i c16_n88_p67 = _mm_set1_epi32(0xFFA80043);
    const __m128i c16_n38_p13 = _mm_set1_epi32(0xFFDA000D); //column 14
    const __m128i c16_n78_p61 = _mm_set1_epi32(0xFFB2003D);
    const __m128i c16_n90_p88 = _mm_set1_epi32(0xFFA60058);
    const __m128i c16_n73_p85 = _mm_set1_epi32(0xFFB70055);
    const __m128i c16_n31_p54 = _mm_set1_epi32(0xFFE10036);
    const __m128i c16_p22_p04 = _mm_set1_epi32(0x00160004);
    const __m128i c16_p67_n46 = _mm_set1_epi32(0x0043FFD2);
    const __m128i c16_p90_n82 = _mm_set1_epi32(0x005AFFAE);
    const __m128i c16_n13_p04 = _mm_set1_epi32(0xFFF30004); //column 15
    const __m128i c16_n31_p22 = _mm_set1_epi32(0xFFE10016);
    const __m128i c16_n46_p38 = _mm_set1_epi32(0xFFD20026);
    const __m128i c16_n61_p54 = _mm_set1_epi32(0xFFC30036);
    const __m128i c16_n73_p67 = _mm_set1_epi32(0xFFB70043);
    const __m128i c16_n82_p78 = _mm_set1_epi32(0xFFAE004E);
    const __m128i c16_n88_p85 = _mm_set1_epi32(0xFFA80055);
    const __m128i c16_n90_p90 = _mm_set1_epi32(0xFFA6005A);

    //EO
    const __m128i c16_p87_p90 = _mm_set1_epi32(0x0057005A); //row0 87high - 90low address
    const __m128i c16_p70_p80 = _mm_set1_epi32(0x00460050);
    const __m128i c16_p43_p57 = _mm_set1_epi32(0x002B0039);
    const __m128i c16_p09_p25 = _mm_set1_epi32(0x00090019);
    const __m128i c16_p57_p87 = _mm_set1_epi32(0x00390057); //row1
    const __m128i c16_n43_p09 = _mm_set1_epi32(0xFFD50009);
    const __m128i c16_n90_n80 = _mm_set1_epi32(0xFFA6FFB0);
    const __m128i c16_n25_n70 = _mm_set1_epi32(0xFFE7FFBA);
    const __m128i c16_p09_p80 = _mm_set1_epi32(0x00090050); //row2
    const __m128i c16_n87_n70 = _mm_set1_epi32(0xFFA9FFBA);
    const __m128i c16_p57_n25 = _mm_set1_epi32(0x0039FFE7);
    const __m128i c16_p43_p90 = _mm_set1_epi32(0x002B005A);
    const __m128i c16_n43_p70 = _mm_set1_epi32(0xFFD50046); //row3
    const __m128i c16_p09_n87 = _mm_set1_epi32(0x0009FFA9);
    const __m128i c16_p25_p90 = _mm_set1_epi32(0x0019005A);
    const __m128i c16_n57_n80 = _mm_set1_epi32(0xFFC7FFB0);
    const __m128i c16_n80_p57 = _mm_set1_epi32(0xFFB00039); //row4
    const __m128i c16_p90_n25 = _mm_set1_epi32(0x005AFFE7);
    const __m128i c16_n87_n09 = _mm_set1_epi32(0xFFA9FFF7);
    const __m128i c16_p70_p43 = _mm_set1_epi32(0x0046002B);
    const __m128i c16_n90_p43 = _mm_set1_epi32(0xFFA6002B); //row5
    const __m128i c16_p25_p57 = _mm_set1_epi32(0x00190039);
    const __m128i c16_p70_n87 = _mm_set1_epi32(0x0046FFA9);
    const __m128i c16_n80_p09 = _mm_set1_epi32(0xFFB00009);
    const __m128i c16_n70_p25 = _mm_set1_epi32(0xFFBA0019); //row6
    const __m128i c16_n80_p90 = _mm_set1_epi32(0xFFB0005A);
    const __m128i c16_p09_p43 = _mm_set1_epi32(0x0009002B);
    const __m128i c16_p87_n57 = _mm_set1_epi32(0x0057FFC7);
    const __m128i c16_n25_p09 = _mm_set1_epi32(0xFFE70009); //row7
    const __m128i c16_n57_p43 = _mm_set1_epi32(0xFFC7002B);
    const __m128i c16_n80_p70 = _mm_set1_epi32(0xFFB00046);
    const __m128i c16_n90_p87 = _mm_set1_epi32(0xFFA60057);
    //EEO
    const __m128i c16_p75_p89 = _mm_set1_epi32(0x004B0059);
    const __m128i c16_p18_p50 = _mm_set1_epi32(0x00120032);
    const __m128i c16_n18_p75 = _mm_set1_epi32(0xFFEE004B);
    const __m128i c16_n50_n89 = _mm_set1_epi32(0xFFCEFFA7);
    const __m128i c16_n89_p50 = _mm_set1_epi32(0xFFA70032);
    const __m128i c16_p75_p18 = _mm_set1_epi32(0x004B0012);
    const __m128i c16_n50_p18 = _mm_set1_epi32(0xFFCE0012);
    const __m128i c16_n89_p75 = _mm_set1_epi32(0xFFA7004B);
    //EEEO
    const __m128i c16_p36_p83 = _mm_set1_epi32(0x00240053);
    const __m128i c16_n83_p36 = _mm_set1_epi32(0xFFAD0024);
    //EEEE
    const __m128i c16_n64_p64 = _mm_set1_epi32(0xFFC00040);
    const __m128i c16_p64_p64 = _mm_set1_epi32(0x00400040);
    __m128i c32_rnd = _mm_set1_epi32(ADD_IT_1);

    int nShift = SHIFT_IT_1ST;

    // DCT1
    __m128i in00[4], in01[4], in02[4], in03[4], in04[4], in05[4], in06[4], in07[4], in08[4], in09[4], in10[4], in11[4], in12[4], in13[4], in14[4], in15[4];
    __m128i in16[4], in17[4], in18[4], in19[4], in20[4], in21[4], in22[4], in23[4], in24[4], in25[4], in26[4], in27[4], in28[4], in29[4], in30[4], in31[4];
    __m128i res00[4], res01[4], res02[4], res03[4], res04[4], res05[4], res06[4], res07[4], res08[4], res09[4], res10[4], res11[4], res12[4], res13[4], res14[4], res15[4];
    __m128i res16[4], res17[4], res18[4], res19[4], res20[4], res21[4], res22[4], res23[4], res24[4], res25[4], res26[4], res27[4], res28[4], res29[4], res30[4], res31[4];

    for (int i = 0; i < 4; i++)
    {
        const int offset = (i << 3);
        in00[i] = _mm_loadu_si128((const __m128i*)&src[0 * 32 + offset]);
        in01[i] = _mm_loadu_si128((const __m128i*)&src[1 * 32 + offset]);
        in02[i] = _mm_loadu_si128((const __m128i*)&src[2 * 32 + offset]);
        in03[i] = _mm_loadu_si128((const __m128i*)&src[3 * 32 + offset]);
        in04[i] = _mm_loadu_si128((const __m128i*)&src[4 * 32 + offset]);
        in05[i] = _mm_loadu_si128((const __m128i*)&src[5 * 32 + offset]);
        in06[i] = _mm_loadu_si128((const __m128i*)&src[6 * 32 + offset]);
        in07[i] = _mm_loadu_si128((const __m128i*)&src[7 * 32 + offset]);
        in08[i] = _mm_loadu_si128((const __m128i*)&src[8 * 32 + offset]);
        in09[i] = _mm_loadu_si128((const __m128i*)&src[9 * 32 + offset]);
        in10[i] = _mm_loadu_si128((const __m128i*)&src[10 * 32 + offset]);
        in11[i] = _mm_loadu_si128((const __m128i*)&src[11 * 32 + offset]);
        in12[i] = _mm_loadu_si128((const __m128i*)&src[12 * 32 + offset]);
        in13[i] = _mm_loadu_si128((const __m128i*)&src[13 * 32 + offset]);
        in14[i] = _mm_loadu_si128((const __m128i*)&src[14 * 32 + offset]);
        in15[i] = _mm_loadu_si128((const __m128i*)&src[15 * 32 + offset]);
        in16[i] = _mm_loadu_si128((const __m128i*)&src[16 * 32 + offset]);
        in17[i] = _mm_loadu_si128((const __m128i*)&src[17 * 32 + offset]);
        in18[i] = _mm_loadu_si128((const __m128i*)&src[18 * 32 + offset]);
        in19[i] = _mm_loadu_si128((const __m128i*)&src[19 * 32 + offset]);
        in20[i] = _mm_loadu_si128((const __m128i*)&src[20 * 32 + offset]);
        in21[i] = _mm_loadu_si128((const __m128i*)&src[21 * 32 + offset]);
        in22[i] = _mm_loadu_si128((const __m128i*)&src[22 * 32 + offset]);
        in23[i] = _mm_loadu_si128((const __m128i*)&src[23 * 32 + offset]);
        in24[i] = _mm_loadu_si128((const __m128i*)&src[24 * 32 + offset]);
        in25[i] = _mm_loadu_si128((const __m128i*)&src[25 * 32 + offset]);
        in26[i] = _mm_loadu_si128((const __m128i*)&src[26 * 32 + offset]);
        in27[i] = _mm_loadu_si128((const __m128i*)&src[27 * 32 + offset]);
        in28[i] = _mm_loadu_si128((const __m128i*)&src[28 * 32 + offset]);
        in29[i] = _mm_loadu_si128((const __m128i*)&src[29 * 32 + offset]);
        in30[i] = _mm_loadu_si128((const __m128i*)&src[30 * 32 + offset]);
        in31[i] = _mm_loadu_si128((const __m128i*)&src[31 * 32 + offset]);
    }

    for (int pass = 0; pass < 2; pass++)
    {
        if (pass == 1)
        {
            c32_rnd = _mm_set1_epi32(ADD_IT_2);
            nShift = SHIFT_IT_2ND;
        }

        for (int part = 0; part < 4; part++)
        {
            const __m128i T_00_00A = _mm_unpacklo_epi16(in01[part], in03[part]);       // [33 13 32 12 31 11 30 10]
            const __m128i T_00_00B = _mm_unpackhi_epi16(in01[part], in03[part]);       // [37 17 36 16 35 15 34 14]
            const __m128i T_00_01A = _mm_unpacklo_epi16(in05[part], in07[part]);       // [ ]
            const __m128i T_00_01B = _mm_unpackhi_epi16(in05[part], in07[part]);       // [ ]
            const __m128i T_00_02A = _mm_unpacklo_epi16(in09[part], in11[part]);       // [ ]
            const __m128i T_00_02B = _mm_unpackhi_epi16(in09[part], in11[part]);       // [ ]
            const __m128i T_00_03A = _mm_unpacklo_epi16(in13[part], in15[part]);       // [ ]
            const __m128i T_00_03B = _mm_unpackhi_epi16(in13[part], in15[part]);       // [ ]
            const __m128i T_00_04A = _mm_unpacklo_epi16(in17[part], in19[part]);       // [ ]
            const __m128i T_00_04B = _mm_unpackhi_epi16(in17[part], in19[part]);       // [ ]
            const __m128i T_00_05A = _mm_unpacklo_epi16(in21[part], in23[part]);       // [ ]
            const __m128i T_00_05B = _mm_unpackhi_epi16(in21[part], in23[part]);       // [ ]
            const __m128i T_00_06A = _mm_unpacklo_epi16(in25[part], in27[part]);       // [ ]
            const __m128i T_00_06B = _mm_unpackhi_epi16(in25[part], in27[part]);       // [ ]
            const __m128i T_00_07A = _mm_unpacklo_epi16(in29[part], in31[part]);       //
            const __m128i T_00_07B = _mm_unpackhi_epi16(in29[part], in31[part]);       // [ ]

            const __m128i T_00_08A = _mm_unpacklo_epi16(in02[part], in06[part]);       // [ ]
            const __m128i T_00_08B = _mm_unpackhi_epi16(in02[part], in06[part]);       // [ ]
            const __m128i T_00_09A = _mm_unpacklo_epi16(in10[part], in14[part]);       // [ ]
            const __m128i T_00_09B = _mm_unpackhi_epi16(in10[part], in14[part]);       // [ ]
            const __m128i T_00_10A = _mm_unpacklo_epi16(in18[part], in22[part]);       // [ ]
            const __m128i T_00_10B = _mm_unpackhi_epi16(in18[part], in22[part]);       // [ ]
            const __m128i T_00_11A = _mm_unpacklo_epi16(in26[part], in30[part]);       // [ ]
            const __m128i T_00_11B = _mm_unpackhi_epi16(in26[part], in30[part]);       // [ ]

            const __m128i T_00_12A = _mm_unpacklo_epi16(in04[part], in12[part]);       // [ ]
            const __m128i T_00_12B = _mm_unpackhi_epi16(in04[part], in12[part]);       // [ ]
            const __m128i T_00_13A = _mm_unpacklo_epi16(in20[part], in28[part]);       // [ ]
            const __m128i T_00_13B = _mm_unpackhi_epi16(in20[part], in28[part]);       // [ ]

            const __m128i T_00_14A = _mm_unpacklo_epi16(in08[part], in24[part]);       //
            const __m128i T_00_14B = _mm_unpackhi_epi16(in08[part], in24[part]);       // [ ]
            const __m128i T_00_15A = _mm_unpacklo_epi16(in00[part], in16[part]);       //
            const __m128i T_00_15B = _mm_unpackhi_epi16(in00[part], in16[part]);       // [ ]

            __m128i O00A, O01A, O02A, O03A, O04A, O05A, O06A, O07A, O08A, O09A, O10A, O11A, O12A, O13A, O14A, O15A;
            __m128i O00B, O01B, O02B, O03B, O04B, O05B, O06B, O07B, O08B, O09B, O10B, O11B, O12B, O13B, O14B, O15B;
            {
                __m128i T00, T01, T02, T03;
#define COMPUTE_ROW(r0103, r0507, r0911, r1315, r1719, r2123, r2527, r2931, c0103, c0507, c0911, c1315, c1719, c2123, c2527, c2931, row) \
  T00 = _mm_add_epi32(_mm_madd_epi16(r0103, c0103), _mm_madd_epi16(r0507, c0507)); \
  T01 = _mm_add_epi32(_mm_madd_epi16(r0911, c0911), _mm_madd_epi16(r1315, c1315)); \
  T02 = _mm_add_epi32(_mm_madd_epi16(r1719, c1719), _mm_madd_epi16(r2123, c2123)); \
  T03 = _mm_add_epi32(_mm_madd_epi16(r2527, c2527), _mm_madd_epi16(r2931, c2931)); \
  row = _mm_add_epi32(_mm_add_epi32(T00, T01), _mm_add_epi32(T02, T03));

                COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_p90_p90, c16_p85_p88, c16_p78_p82, c16_p67_p73, c16_p54_p61, c16_p38_p46, c16_p22_p31, c16_p04_p13, O00A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_p82_p90, c16_p46_p67, c16_n04_p22, c16_n54_n31, c16_n85_n73, c16_n88_n90, c16_n61_n78, c16_n13_n38, O01A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_p67_p88, c16_n13_p31, c16_n82_n54, c16_n78_n90, c16_n04_n46, c16_p73_p38, c16_p85_p90, c16_p22_p61, O02A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_p46_p85, c16_n67_n13, c16_n73_n90, c16_p38_n22, c16_p88_p82, c16_n04_p54, c16_n90_n61, c16_n31_n78, O03A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_p22_p82, c16_n90_n54, c16_p13_n61, c16_p85_p78, c16_n46_p31, c16_n67_n90, c16_p73_p04, c16_p38_p88, O04A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n04_p78, c16_n73_n82, c16_p85_p13, c16_n22_p67, c16_n61_n88, c16_p90_p31, c16_n38_p54, c16_n46_n90, O05A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n31_p73, c16_n22_n90, c16_p67_p78, c16_n90_n38, c16_p82_n13, c16_n46_p61, c16_n04_n88, c16_p54_p85, O06A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n54_p67, c16_p38_n78, c16_n22_p85, c16_p04_n90, c16_p13_p90, c16_n31_n88, c16_p46_p82, c16_n61_n73, O07A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n73_p61, c16_p82_n46, c16_n88_p31, c16_p90_n13, c16_n90_n04, c16_p85_p22, c16_n78_n38, c16_p67_p54, O08A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n85_p54, c16_p88_n04, c16_n61_n46, c16_p13_p82, c16_p38_n90, c16_n78_p67, c16_p90_n22, c16_n73_n31, O09A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n90_p46, c16_p54_p38, c16_p31_n90, c16_n88_p61, c16_p67_p22, c16_p13_n85, c16_n82_p73, c16_p78_p04, O10A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n88_p38, c16_n04_p73, c16_p90_n67, c16_n31_n46, c16_n78_p85, c16_p61_p13, c16_p54_n90, c16_n82_p22, O11A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n78_p31, c16_n61_p90, c16_p54_p04, c16_p82_n88, c16_n22_n38, c16_n90_p73, c16_n13_p67, c16_p85_n46, O12A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n61_p22, c16_n90_p85, c16_n38_p73, c16_p46_n04, c16_p90_n78, c16_p54_n82, c16_n31_n13, c16_n88_p67, O13A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n38_p13, c16_n78_p61, c16_n90_p88, c16_n73_p85, c16_n31_p54, c16_p22_p04, c16_p67_n46, c16_p90_n82, O14A)
                    COMPUTE_ROW(T_00_00A, T_00_01A, T_00_02A, T_00_03A, T_00_04A, T_00_05A, T_00_06A, T_00_07A, \
                    c16_n13_p04, c16_n31_p22, c16_n46_p38, c16_n61_p54, c16_n73_p67, c16_n82_p78, c16_n88_p85, c16_n90_p90, O15A)

                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_p90_p90, c16_p85_p88, c16_p78_p82, c16_p67_p73, c16_p54_p61, c16_p38_p46, c16_p22_p31, c16_p04_p13, O00B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_p82_p90, c16_p46_p67, c16_n04_p22, c16_n54_n31, c16_n85_n73, c16_n88_n90, c16_n61_n78, c16_n13_n38, O01B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_p67_p88, c16_n13_p31, c16_n82_n54, c16_n78_n90, c16_n04_n46, c16_p73_p38, c16_p85_p90, c16_p22_p61, O02B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_p46_p85, c16_n67_n13, c16_n73_n90, c16_p38_n22, c16_p88_p82, c16_n04_p54, c16_n90_n61, c16_n31_n78, O03B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_p22_p82, c16_n90_n54, c16_p13_n61, c16_p85_p78, c16_n46_p31, c16_n67_n90, c16_p73_p04, c16_p38_p88, O04B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n04_p78, c16_n73_n82, c16_p85_p13, c16_n22_p67, c16_n61_n88, c16_p90_p31, c16_n38_p54, c16_n46_n90, O05B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n31_p73, c16_n22_n90, c16_p67_p78, c16_n90_n38, c16_p82_n13, c16_n46_p61, c16_n04_n88, c16_p54_p85, O06B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n54_p67, c16_p38_n78, c16_n22_p85, c16_p04_n90, c16_p13_p90, c16_n31_n88, c16_p46_p82, c16_n61_n73, O07B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n73_p61, c16_p82_n46, c16_n88_p31, c16_p90_n13, c16_n90_n04, c16_p85_p22, c16_n78_n38, c16_p67_p54, O08B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n85_p54, c16_p88_n04, c16_n61_n46, c16_p13_p82, c16_p38_n90, c16_n78_p67, c16_p90_n22, c16_n73_n31, O09B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n90_p46, c16_p54_p38, c16_p31_n90, c16_n88_p61, c16_p67_p22, c16_p13_n85, c16_n82_p73, c16_p78_p04, O10B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n88_p38, c16_n04_p73, c16_p90_n67, c16_n31_n46, c16_n78_p85, c16_p61_p13, c16_p54_n90, c16_n82_p22, O11B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n78_p31, c16_n61_p90, c16_p54_p04, c16_p82_n88, c16_n22_n38, c16_n90_p73, c16_n13_p67, c16_p85_n46, O12B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n61_p22, c16_n90_p85, c16_n38_p73, c16_p46_n04, c16_p90_n78, c16_p54_n82, c16_n31_n13, c16_n88_p67, O13B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n38_p13, c16_n78_p61, c16_n90_p88, c16_n73_p85, c16_n31_p54, c16_p22_p04, c16_p67_n46, c16_p90_n82, O14B)
                    COMPUTE_ROW(T_00_00B, T_00_01B, T_00_02B, T_00_03B, T_00_04B, T_00_05B, T_00_06B, T_00_07B, \
                    c16_n13_p04, c16_n31_p22, c16_n46_p38, c16_n61_p54, c16_n73_p67, c16_n82_p78, c16_n88_p85, c16_n90_p90, O15B)

#undef COMPUTE_ROW
            }

            __m128i EO0A, EO1A, EO2A, EO3A, EO4A, EO5A, EO6A, EO7A;
            __m128i EO0B, EO1B, EO2B, EO3B, EO4B, EO5B, EO6B, EO7B;
            {
                __m128i T00, T01;
#define COMPUTE_ROW(row0206, row1014, row1822, row2630, c0206, c1014, c1822, c2630, row) \
  T00 = _mm_add_epi32(_mm_madd_epi16(row0206, c0206), _mm_madd_epi16(row1014, c1014)); \
  T01 = _mm_add_epi32(_mm_madd_epi16(row1822, c1822), _mm_madd_epi16(row2630, c2630)); \
  row = _mm_add_epi32(T00, T01);

                COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_p87_p90, c16_p70_p80, c16_p43_p57, c16_p09_p25, EO0A)
                    COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_p57_p87, c16_n43_p09, c16_n90_n80, c16_n25_n70, EO1A)
                    COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_p09_p80, c16_n87_n70, c16_p57_n25, c16_p43_p90, EO2A)
                    COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_n43_p70, c16_p09_n87, c16_p25_p90, c16_n57_n80, EO3A)
                    COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_n80_p57, c16_p90_n25, c16_n87_n09, c16_p70_p43, EO4A)
                    COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_n90_p43, c16_p25_p57, c16_p70_n87, c16_n80_p09, EO5A)
                    COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_n70_p25, c16_n80_p90, c16_p09_p43, c16_p87_n57, EO6A)
                    COMPUTE_ROW(T_00_08A, T_00_09A, T_00_10A, T_00_11A, c16_n25_p09, c16_n57_p43, c16_n80_p70, c16_n90_p87, EO7A)

                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_p87_p90, c16_p70_p80, c16_p43_p57, c16_p09_p25, EO0B)
                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_p57_p87, c16_n43_p09, c16_n90_n80, c16_n25_n70, EO1B)
                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_p09_p80, c16_n87_n70, c16_p57_n25, c16_p43_p90, EO2B)
                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_n43_p70, c16_p09_n87, c16_p25_p90, c16_n57_n80, EO3B)
                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_n80_p57, c16_p90_n25, c16_n87_n09, c16_p70_p43, EO4B)
                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_n90_p43, c16_p25_p57, c16_p70_n87, c16_n80_p09, EO5B)
                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_n70_p25, c16_n80_p90, c16_p09_p43, c16_p87_n57, EO6B)
                    COMPUTE_ROW(T_00_08B, T_00_09B, T_00_10B, T_00_11B, c16_n25_p09, c16_n57_p43, c16_n80_p70, c16_n90_p87, EO7B)
#undef COMPUTE_ROW
            }

            const __m128i EEO0A = _mm_add_epi32(_mm_madd_epi16(T_00_12A, c16_p75_p89), _mm_madd_epi16(T_00_13A, c16_p18_p50)); // EEO0
            const __m128i EEO0B = _mm_add_epi32(_mm_madd_epi16(T_00_12B, c16_p75_p89), _mm_madd_epi16(T_00_13B, c16_p18_p50));
            const __m128i EEO1A = _mm_add_epi32(_mm_madd_epi16(T_00_12A, c16_n18_p75), _mm_madd_epi16(T_00_13A, c16_n50_n89)); // EEO1
            const __m128i EEO1B = _mm_add_epi32(_mm_madd_epi16(T_00_12B, c16_n18_p75), _mm_madd_epi16(T_00_13B, c16_n50_n89));
            const __m128i EEO2A = _mm_add_epi32(_mm_madd_epi16(T_00_12A, c16_n89_p50), _mm_madd_epi16(T_00_13A, c16_p75_p18)); // EEO2
            const __m128i EEO2B = _mm_add_epi32(_mm_madd_epi16(T_00_12B, c16_n89_p50), _mm_madd_epi16(T_00_13B, c16_p75_p18));
            const __m128i EEO3A = _mm_add_epi32(_mm_madd_epi16(T_00_12A, c16_n50_p18), _mm_madd_epi16(T_00_13A, c16_n89_p75)); // EEO3
            const __m128i EEO3B = _mm_add_epi32(_mm_madd_epi16(T_00_12B, c16_n50_p18), _mm_madd_epi16(T_00_13B, c16_n89_p75));

            const __m128i EEEO0A = _mm_madd_epi16(T_00_14A, c16_p36_p83);
            const __m128i EEEO0B = _mm_madd_epi16(T_00_14B, c16_p36_p83);
            const __m128i EEEO1A = _mm_madd_epi16(T_00_14A, c16_n83_p36);
            const __m128i EEEO1B = _mm_madd_epi16(T_00_14B, c16_n83_p36);

            const __m128i EEEE0A = _mm_madd_epi16(T_00_15A, c16_p64_p64);
            const __m128i EEEE0B = _mm_madd_epi16(T_00_15B, c16_p64_p64);
            const __m128i EEEE1A = _mm_madd_epi16(T_00_15A, c16_n64_p64);
            const __m128i EEEE1B = _mm_madd_epi16(T_00_15B, c16_n64_p64);

            const __m128i EEE0A = _mm_add_epi32(EEEE0A, EEEO0A);          // EEE0 = EEEE0 + EEEO0
            const __m128i EEE0B = _mm_add_epi32(EEEE0B, EEEO0B);
            const __m128i EEE1A = _mm_add_epi32(EEEE1A, EEEO1A);          // EEE1 = EEEE1 + EEEO1
            const __m128i EEE1B = _mm_add_epi32(EEEE1B, EEEO1B);
            const __m128i EEE3A = _mm_sub_epi32(EEEE0A, EEEO0A);          // EEE2 = EEEE0 - EEEO0
            const __m128i EEE3B = _mm_sub_epi32(EEEE0B, EEEO0B);
            const __m128i EEE2A = _mm_sub_epi32(EEEE1A, EEEO1A);          // EEE3 = EEEE1 - EEEO1
            const __m128i EEE2B = _mm_sub_epi32(EEEE1B, EEEO1B);

            const __m128i EE0A = _mm_add_epi32(EEE0A, EEO0A);          // EE0 = EEE0 + EEO0
            const __m128i EE0B = _mm_add_epi32(EEE0B, EEO0B);
            const __m128i EE1A = _mm_add_epi32(EEE1A, EEO1A);          // EE1 = EEE1 + EEO1
            const __m128i EE1B = _mm_add_epi32(EEE1B, EEO1B);
            const __m128i EE2A = _mm_add_epi32(EEE2A, EEO2A);          // EE2 = EEE0 + EEO0
            const __m128i EE2B = _mm_add_epi32(EEE2B, EEO2B);
            const __m128i EE3A = _mm_add_epi32(EEE3A, EEO3A);          // EE3 = EEE1 + EEO1
            const __m128i EE3B = _mm_add_epi32(EEE3B, EEO3B);
            const __m128i EE7A = _mm_sub_epi32(EEE0A, EEO0A);          // EE7 = EEE0 - EEO0
            const __m128i EE7B = _mm_sub_epi32(EEE0B, EEO0B);
            const __m128i EE6A = _mm_sub_epi32(EEE1A, EEO1A);          // EE6 = EEE1 - EEO1
            const __m128i EE6B = _mm_sub_epi32(EEE1B, EEO1B);
            const __m128i EE5A = _mm_sub_epi32(EEE2A, EEO2A);          // EE5 = EEE0 - EEO0
            const __m128i EE5B = _mm_sub_epi32(EEE2B, EEO2B);
            const __m128i EE4A = _mm_sub_epi32(EEE3A, EEO3A);          // EE4 = EEE1 - EEO1
            const __m128i EE4B = _mm_sub_epi32(EEE3B, EEO3B);

            const __m128i E0A = _mm_add_epi32(EE0A, EO0A);          // E0 = EE0 + EO0
            const __m128i E0B = _mm_add_epi32(EE0B, EO0B);
            const __m128i E1A = _mm_add_epi32(EE1A, EO1A);          // E1 = EE1 + EO1
            const __m128i E1B = _mm_add_epi32(EE1B, EO1B);
            const __m128i E2A = _mm_add_epi32(EE2A, EO2A);          // E2 = EE2 + EO2
            const __m128i E2B = _mm_add_epi32(EE2B, EO2B);
            const __m128i E3A = _mm_add_epi32(EE3A, EO3A);          // E3 = EE3 + EO3
            const __m128i E3B = _mm_add_epi32(EE3B, EO3B);
            const __m128i E4A = _mm_add_epi32(EE4A, EO4A);          // E4 =
            const __m128i E4B = _mm_add_epi32(EE4B, EO4B);
            const __m128i E5A = _mm_add_epi32(EE5A, EO5A);          // E5 =
            const __m128i E5B = _mm_add_epi32(EE5B, EO5B);
            const __m128i E6A = _mm_add_epi32(EE6A, EO6A);          // E6 =
            const __m128i E6B = _mm_add_epi32(EE6B, EO6B);
            const __m128i E7A = _mm_add_epi32(EE7A, EO7A);          // E7 =
            const __m128i E7B = _mm_add_epi32(EE7B, EO7B);
            const __m128i EFA = _mm_sub_epi32(EE0A, EO0A);          // EF = EE0 - EO0
            const __m128i EFB = _mm_sub_epi32(EE0B, EO0B);
            const __m128i EEA = _mm_sub_epi32(EE1A, EO1A);          // EE = EE1 - EO1
            const __m128i EEB = _mm_sub_epi32(EE1B, EO1B);
            const __m128i EDA = _mm_sub_epi32(EE2A, EO2A);          // ED = EE2 - EO2
            const __m128i EDB = _mm_sub_epi32(EE2B, EO2B);
            const __m128i ECA = _mm_sub_epi32(EE3A, EO3A);          // EC = EE3 - EO3
            const __m128i ECB = _mm_sub_epi32(EE3B, EO3B);
            const __m128i EBA = _mm_sub_epi32(EE4A, EO4A);          // EB =
            const __m128i EBB = _mm_sub_epi32(EE4B, EO4B);
            const __m128i EAA = _mm_sub_epi32(EE5A, EO5A);          // EA =
            const __m128i EAB = _mm_sub_epi32(EE5B, EO5B);
            const __m128i E9A = _mm_sub_epi32(EE6A, EO6A);          // E9 =
            const __m128i E9B = _mm_sub_epi32(EE6B, EO6B);
            const __m128i E8A = _mm_sub_epi32(EE7A, EO7A);          // E8 =
            const __m128i E8B = _mm_sub_epi32(EE7B, EO7B);

            const __m128i T10A = _mm_add_epi32(E0A, c32_rnd);         // E0 + rnd
            const __m128i T10B = _mm_add_epi32(E0B, c32_rnd);
            const __m128i T11A = _mm_add_epi32(E1A, c32_rnd);         // E1 + rnd
            const __m128i T11B = _mm_add_epi32(E1B, c32_rnd);
            const __m128i T12A = _mm_add_epi32(E2A, c32_rnd);         // E2 + rnd
            const __m128i T12B = _mm_add_epi32(E2B, c32_rnd);
            const __m128i T13A = _mm_add_epi32(E3A, c32_rnd);         // E3 + rnd
            const __m128i T13B = _mm_add_epi32(E3B, c32_rnd);
            const __m128i T14A = _mm_add_epi32(E4A, c32_rnd);         // E4 + rnd
            const __m128i T14B = _mm_add_epi32(E4B, c32_rnd);
            const __m128i T15A = _mm_add_epi32(E5A, c32_rnd);         // E5 + rnd
            const __m128i T15B = _mm_add_epi32(E5B, c32_rnd);
            const __m128i T16A = _mm_add_epi32(E6A, c32_rnd);         // E6 + rnd
            const __m128i T16B = _mm_add_epi32(E6B, c32_rnd);
            const __m128i T17A = _mm_add_epi32(E7A, c32_rnd);         // E7 + rnd
            const __m128i T17B = _mm_add_epi32(E7B, c32_rnd);
            const __m128i T18A = _mm_add_epi32(E8A, c32_rnd);         // E8 + rnd
            const __m128i T18B = _mm_add_epi32(E8B, c32_rnd);
            const __m128i T19A = _mm_add_epi32(E9A, c32_rnd);         // E9 + rnd
            const __m128i T19B = _mm_add_epi32(E9B, c32_rnd);
            const __m128i T1AA = _mm_add_epi32(EAA, c32_rnd);         // E10 + rnd
            const __m128i T1AB = _mm_add_epi32(EAB, c32_rnd);
            const __m128i T1BA = _mm_add_epi32(EBA, c32_rnd);         // E11 + rnd
            const __m128i T1BB = _mm_add_epi32(EBB, c32_rnd);
            const __m128i T1CA = _mm_add_epi32(ECA, c32_rnd);         // E12 + rnd
            const __m128i T1CB = _mm_add_epi32(ECB, c32_rnd);
            const __m128i T1DA = _mm_add_epi32(EDA, c32_rnd);         // E13 + rnd
            const __m128i T1DB = _mm_add_epi32(EDB, c32_rnd);
            const __m128i T1EA = _mm_add_epi32(EEA, c32_rnd);         // E14 + rnd
            const __m128i T1EB = _mm_add_epi32(EEB, c32_rnd);
            const __m128i T1FA = _mm_add_epi32(EFA, c32_rnd);         // E15 + rnd
            const __m128i T1FB = _mm_add_epi32(EFB, c32_rnd);

            const __m128i T2_00A = _mm_add_epi32(T10A, O00A);          // E0 + O0 + rnd
            const __m128i T2_00B = _mm_add_epi32(T10B, O00B);
            const __m128i T2_01A = _mm_add_epi32(T11A, O01A);          // E1 + O1 + rnd
            const __m128i T2_01B = _mm_add_epi32(T11B, O01B);
            const __m128i T2_02A = _mm_add_epi32(T12A, O02A);          // E2 + O2 + rnd
            const __m128i T2_02B = _mm_add_epi32(T12B, O02B);
            const __m128i T2_03A = _mm_add_epi32(T13A, O03A);          // E3 + O3 + rnd
            const __m128i T2_03B = _mm_add_epi32(T13B, O03B);
            const __m128i T2_04A = _mm_add_epi32(T14A, O04A);          // E4
            const __m128i T2_04B = _mm_add_epi32(T14B, O04B);
            const __m128i T2_05A = _mm_add_epi32(T15A, O05A);          // E5
            const __m128i T2_05B = _mm_add_epi32(T15B, O05B);
            const __m128i T2_06A = _mm_add_epi32(T16A, O06A);          // E6
            const __m128i T2_06B = _mm_add_epi32(T16B, O06B);
            const __m128i T2_07A = _mm_add_epi32(T17A, O07A);          // E7
            const __m128i T2_07B = _mm_add_epi32(T17B, O07B);
            const __m128i T2_08A = _mm_add_epi32(T18A, O08A);          // E8
            const __m128i T2_08B = _mm_add_epi32(T18B, O08B);
            const __m128i T2_09A = _mm_add_epi32(T19A, O09A);          // E9
            const __m128i T2_09B = _mm_add_epi32(T19B, O09B);
            const __m128i T2_10A = _mm_add_epi32(T1AA, O10A);          // E10
            const __m128i T2_10B = _mm_add_epi32(T1AB, O10B);
            const __m128i T2_11A = _mm_add_epi32(T1BA, O11A);          // E11
            const __m128i T2_11B = _mm_add_epi32(T1BB, O11B);
            const __m128i T2_12A = _mm_add_epi32(T1CA, O12A);          // E12
            const __m128i T2_12B = _mm_add_epi32(T1CB, O12B);
            const __m128i T2_13A = _mm_add_epi32(T1DA, O13A);          // E13
            const __m128i T2_13B = _mm_add_epi32(T1DB, O13B);
            const __m128i T2_14A = _mm_add_epi32(T1EA, O14A);          // E14
            const __m128i T2_14B = _mm_add_epi32(T1EB, O14B);
            const __m128i T2_15A = _mm_add_epi32(T1FA, O15A);          // E15
            const __m128i T2_15B = _mm_add_epi32(T1FB, O15B);
            const __m128i T2_31A = _mm_sub_epi32(T10A, O00A);          // E0 - O0 + rnd
            const __m128i T2_31B = _mm_sub_epi32(T10B, O00B);
            const __m128i T2_30A = _mm_sub_epi32(T11A, O01A);          // E1 - O1 + rnd
            const __m128i T2_30B = _mm_sub_epi32(T11B, O01B);
            const __m128i T2_29A = _mm_sub_epi32(T12A, O02A);          // E2 - O2 + rnd
            const __m128i T2_29B = _mm_sub_epi32(T12B, O02B);
            const __m128i T2_28A = _mm_sub_epi32(T13A, O03A);          // E3 - O3 + rnd
            const __m128i T2_28B = _mm_sub_epi32(T13B, O03B);
            const __m128i T2_27A = _mm_sub_epi32(T14A, O04A);          // E4
            const __m128i T2_27B = _mm_sub_epi32(T14B, O04B);
            const __m128i T2_26A = _mm_sub_epi32(T15A, O05A);          // E5
            const __m128i T2_26B = _mm_sub_epi32(T15B, O05B);
            const __m128i T2_25A = _mm_sub_epi32(T16A, O06A);          // E6
            const __m128i T2_25B = _mm_sub_epi32(T16B, O06B);
            const __m128i T2_24A = _mm_sub_epi32(T17A, O07A);          // E7
            const __m128i T2_24B = _mm_sub_epi32(T17B, O07B);
            const __m128i T2_23A = _mm_sub_epi32(T18A, O08A);          //
            const __m128i T2_23B = _mm_sub_epi32(T18B, O08B);
            const __m128i T2_22A = _mm_sub_epi32(T19A, O09A);          //
            const __m128i T2_22B = _mm_sub_epi32(T19B, O09B);
            const __m128i T2_21A = _mm_sub_epi32(T1AA, O10A);          //
            const __m128i T2_21B = _mm_sub_epi32(T1AB, O10B);
            const __m128i T2_20A = _mm_sub_epi32(T1BA, O11A);          //
            const __m128i T2_20B = _mm_sub_epi32(T1BB, O11B);
            const __m128i T2_19A = _mm_sub_epi32(T1CA, O12A);          //
            const __m128i T2_19B = _mm_sub_epi32(T1CB, O12B);
            const __m128i T2_18A = _mm_sub_epi32(T1DA, O13A);          //
            const __m128i T2_18B = _mm_sub_epi32(T1DB, O13B);
            const __m128i T2_17A = _mm_sub_epi32(T1EA, O14A);          //
            const __m128i T2_17B = _mm_sub_epi32(T1EB, O14B);
            const __m128i T2_16A = _mm_sub_epi32(T1FA, O15A);          //
            const __m128i T2_16B = _mm_sub_epi32(T1FB, O15B);

            const __m128i T3_00A = _mm_srai_epi32(T2_00A, nShift);             // [30 20 10 00]
            const __m128i T3_00B = _mm_srai_epi32(T2_00B, nShift);             // [70 60 50 40]
            const __m128i T3_01A = _mm_srai_epi32(T2_01A, nShift);             // [31 21 11 01]
            const __m128i T3_01B = _mm_srai_epi32(T2_01B, nShift);             // [71 61 51 41]
            const __m128i T3_02A = _mm_srai_epi32(T2_02A, nShift);             // [32 22 12 02]
            const __m128i T3_02B = _mm_srai_epi32(T2_02B, nShift);             // [72 62 52 42]
            const __m128i T3_03A = _mm_srai_epi32(T2_03A, nShift);             // [33 23 13 03]
            const __m128i T3_03B = _mm_srai_epi32(T2_03B, nShift);             // [73 63 53 43]
            const __m128i T3_04A = _mm_srai_epi32(T2_04A, nShift);             // [33 24 14 04]
            const __m128i T3_04B = _mm_srai_epi32(T2_04B, nShift);             // [74 64 54 44]
            const __m128i T3_05A = _mm_srai_epi32(T2_05A, nShift);             // [35 25 15 05]
            const __m128i T3_05B = _mm_srai_epi32(T2_05B, nShift);             // [75 65 55 45]
            const __m128i T3_06A = _mm_srai_epi32(T2_06A, nShift);             // [36 26 16 06]
            const __m128i T3_06B = _mm_srai_epi32(T2_06B, nShift);             // [76 66 56 46]
            const __m128i T3_07A = _mm_srai_epi32(T2_07A, nShift);             // [37 27 17 07]
            const __m128i T3_07B = _mm_srai_epi32(T2_07B, nShift);             // [77 67 57 47]
            const __m128i T3_08A = _mm_srai_epi32(T2_08A, nShift);             // [30 20 10 00] x8
            const __m128i T3_08B = _mm_srai_epi32(T2_08B, nShift);             // [70 60 50 40]
            const __m128i T3_09A = _mm_srai_epi32(T2_09A, nShift);             // [31 21 11 01] x9
            const __m128i T3_09B = _mm_srai_epi32(T2_09B, nShift);             // [71 61 51 41]
            const __m128i T3_10A = _mm_srai_epi32(T2_10A, nShift);             // [32 22 12 02] xA
            const __m128i T3_10B = _mm_srai_epi32(T2_10B, nShift);             // [72 62 52 42]
            const __m128i T3_11A = _mm_srai_epi32(T2_11A, nShift);             // [33 23 13 03] xB
            const __m128i T3_11B = _mm_srai_epi32(T2_11B, nShift);             // [73 63 53 43]
            const __m128i T3_12A = _mm_srai_epi32(T2_12A, nShift);             // [33 24 14 04] xC
            const __m128i T3_12B = _mm_srai_epi32(T2_12B, nShift);             // [74 64 54 44]
            const __m128i T3_13A = _mm_srai_epi32(T2_13A, nShift);             // [35 25 15 05] xD
            const __m128i T3_13B = _mm_srai_epi32(T2_13B, nShift);             // [75 65 55 45]
            const __m128i T3_14A = _mm_srai_epi32(T2_14A, nShift);             // [36 26 16 06] xE
            const __m128i T3_14B = _mm_srai_epi32(T2_14B, nShift);             // [76 66 56 46]
            const __m128i T3_15A = _mm_srai_epi32(T2_15A, nShift);             // [37 27 17 07] xF
            const __m128i T3_15B = _mm_srai_epi32(T2_15B, nShift);             // [77 67 57 47]

            const __m128i T3_16A = _mm_srai_epi32(T2_16A, nShift);             // [30 20 10 00]
            const __m128i T3_16B = _mm_srai_epi32(T2_16B, nShift);             // [70 60 50 40]
            const __m128i T3_17A = _mm_srai_epi32(T2_17A, nShift);             // [31 21 11 01]
            const __m128i T3_17B = _mm_srai_epi32(T2_17B, nShift);             // [71 61 51 41]
            const __m128i T3_18A = _mm_srai_epi32(T2_18A, nShift);             // [32 22 12 02]
            const __m128i T3_18B = _mm_srai_epi32(T2_18B, nShift);             // [72 62 52 42]
            const __m128i T3_19A = _mm_srai_epi32(T2_19A, nShift);             // [33 23 13 03]
            const __m128i T3_19B = _mm_srai_epi32(T2_19B, nShift);             // [73 63 53 43]
            const __m128i T3_20A = _mm_srai_epi32(T2_20A, nShift);             // [33 24 14 04]
            const __m128i T3_20B = _mm_srai_epi32(T2_20B, nShift);             // [74 64 54 44]
            const __m128i T3_21A = _mm_srai_epi32(T2_21A, nShift);             // [35 25 15 05]
            const __m128i T3_21B = _mm_srai_epi32(T2_21B, nShift);             // [75 65 55 45]
            const __m128i T3_22A = _mm_srai_epi32(T2_22A, nShift);             // [36 26 16 06]
            const __m128i T3_22B = _mm_srai_epi32(T2_22B, nShift);             // [76 66 56 46]
            const __m128i T3_23A = _mm_srai_epi32(T2_23A, nShift);             // [37 27 17 07]
            const __m128i T3_23B = _mm_srai_epi32(T2_23B, nShift);             // [77 67 57 47]
            const __m128i T3_24A = _mm_srai_epi32(T2_24A, nShift);             // [30 20 10 00] x8
            const __m128i T3_24B = _mm_srai_epi32(T2_24B, nShift);             // [70 60 50 40]
            const __m128i T3_25A = _mm_srai_epi32(T2_25A, nShift);             // [31 21 11 01] x9
            const __m128i T3_25B = _mm_srai_epi32(T2_25B, nShift);             // [71 61 51 41]
            const __m128i T3_26A = _mm_srai_epi32(T2_26A, nShift);             // [32 22 12 02] xA
            const __m128i T3_26B = _mm_srai_epi32(T2_26B, nShift);             // [72 62 52 42]
            const __m128i T3_27A = _mm_srai_epi32(T2_27A, nShift);             // [33 23 13 03] xB
            const __m128i T3_27B = _mm_srai_epi32(T2_27B, nShift);             // [73 63 53 43]
            const __m128i T3_28A = _mm_srai_epi32(T2_28A, nShift);             // [33 24 14 04] xC
            const __m128i T3_28B = _mm_srai_epi32(T2_28B, nShift);             // [74 64 54 44]
            const __m128i T3_29A = _mm_srai_epi32(T2_29A, nShift);             // [35 25 15 05] xD
            const __m128i T3_29B = _mm_srai_epi32(T2_29B, nShift);             // [75 65 55 45]
            const __m128i T3_30A = _mm_srai_epi32(T2_30A, nShift);             // [36 26 16 06] xE
            const __m128i T3_30B = _mm_srai_epi32(T2_30B, nShift);             // [76 66 56 46]
            const __m128i T3_31A = _mm_srai_epi32(T2_31A, nShift);             // [37 27 17 07] xF
            const __m128i T3_31B = _mm_srai_epi32(T2_31B, nShift);             // [77 67 57 47]

            res00[part] = _mm_packs_epi32(T3_00A, T3_00B);        // [70 60 50 40 30 20 10 00]
            res01[part] = _mm_packs_epi32(T3_01A, T3_01B);        // [71 61 51 41 31 21 11 01]
            res02[part] = _mm_packs_epi32(T3_02A, T3_02B);        // [72 62 52 42 32 22 12 02]
            res03[part] = _mm_packs_epi32(T3_03A, T3_03B);        // [73 63 53 43 33 23 13 03]
            res04[part] = _mm_packs_epi32(T3_04A, T3_04B);        // [74 64 54 44 34 24 14 04]
            res05[part] = _mm_packs_epi32(T3_05A, T3_05B);        // [75 65 55 45 35 25 15 05]
            res06[part] = _mm_packs_epi32(T3_06A, T3_06B);        // [76 66 56 46 36 26 16 06]
            res07[part] = _mm_packs_epi32(T3_07A, T3_07B);        // [77 67 57 47 37 27 17 07]
            res08[part] = _mm_packs_epi32(T3_08A, T3_08B);        // [A0 ... 80]
            res09[part] = _mm_packs_epi32(T3_09A, T3_09B);        // [A1 ... 81]
            res10[part] = _mm_packs_epi32(T3_10A, T3_10B);        // [A2 ... 82]
            res11[part] = _mm_packs_epi32(T3_11A, T3_11B);        // [A3 ... 83]
            res12[part] = _mm_packs_epi32(T3_12A, T3_12B);        // [A4 ... 84]
            res13[part] = _mm_packs_epi32(T3_13A, T3_13B);        // [A5 ... 85]
            res14[part] = _mm_packs_epi32(T3_14A, T3_14B);        // [A6 ... 86]
            res15[part] = _mm_packs_epi32(T3_15A, T3_15B);        // [A7 ... 87]
            res16[part] = _mm_packs_epi32(T3_16A, T3_16B);
            res17[part] = _mm_packs_epi32(T3_17A, T3_17B);
            res18[part] = _mm_packs_epi32(T3_18A, T3_18B);
            res19[part] = _mm_packs_epi32(T3_19A, T3_19B);
            res20[part] = _mm_packs_epi32(T3_20A, T3_20B);
            res21[part] = _mm_packs_epi32(T3_21A, T3_21B);
            res22[part] = _mm_packs_epi32(T3_22A, T3_22B);
            res23[part] = _mm_packs_epi32(T3_23A, T3_23B);
            res24[part] = _mm_packs_epi32(T3_24A, T3_24B);
            res25[part] = _mm_packs_epi32(T3_25A, T3_25B);
            res26[part] = _mm_packs_epi32(T3_26A, T3_26B);
            res27[part] = _mm_packs_epi32(T3_27A, T3_27B);
            res28[part] = _mm_packs_epi32(T3_28A, T3_28B);
            res29[part] = _mm_packs_epi32(T3_29A, T3_29B);
            res30[part] = _mm_packs_epi32(T3_30A, T3_30B);
            res31[part] = _mm_packs_epi32(T3_31A, T3_31B);
        }
        //transpose matrix 8x8 16bit.
        {
            __m128i tr0_0, tr0_1, tr0_2, tr0_3, tr0_4, tr0_5, tr0_6, tr0_7;
            __m128i tr1_0, tr1_1, tr1_2, tr1_3, tr1_4, tr1_5, tr1_6, tr1_7;
#define TRANSPOSE_8x8_16BIT(I0, I1, I2, I3, I4, I5, I6, I7, O0, O1, O2, O3, O4, O5, O6, O7) \
  tr0_0 = _mm_unpacklo_epi16(I0, I1); \
  tr0_1 = _mm_unpacklo_epi16(I2, I3); \
  tr0_2 = _mm_unpackhi_epi16(I0, I1); \
  tr0_3 = _mm_unpackhi_epi16(I2, I3); \
  tr0_4 = _mm_unpacklo_epi16(I4, I5); \
  tr0_5 = _mm_unpacklo_epi16(I6, I7); \
  tr0_6 = _mm_unpackhi_epi16(I4, I5); \
  tr0_7 = _mm_unpackhi_epi16(I6, I7); \
  tr1_0 = _mm_unpacklo_epi32(tr0_0, tr0_1); \
  tr1_1 = _mm_unpacklo_epi32(tr0_2, tr0_3); \
  tr1_2 = _mm_unpackhi_epi32(tr0_0, tr0_1); \
  tr1_3 = _mm_unpackhi_epi32(tr0_2, tr0_3); \
  tr1_4 = _mm_unpacklo_epi32(tr0_4, tr0_5); \
  tr1_5 = _mm_unpacklo_epi32(tr0_6, tr0_7); \
  tr1_6 = _mm_unpackhi_epi32(tr0_4, tr0_5); \
  tr1_7 = _mm_unpackhi_epi32(tr0_6, tr0_7); \
  O0 = _mm_unpacklo_epi64(tr1_0, tr1_4); \
  O1 = _mm_unpackhi_epi64(tr1_0, tr1_4); \
  O2 = _mm_unpacklo_epi64(tr1_2, tr1_6); \
  O3 = _mm_unpackhi_epi64(tr1_2, tr1_6); \
  O4 = _mm_unpacklo_epi64(tr1_1, tr1_5); \
  O5 = _mm_unpackhi_epi64(tr1_1, tr1_5); \
  O6 = _mm_unpacklo_epi64(tr1_3, tr1_7); \
  O7 = _mm_unpackhi_epi64(tr1_3, tr1_7); \

            TRANSPOSE_8x8_16BIT(res00[0], res01[0], res02[0], res03[0], res04[0], res05[0], res06[0], res07[0], in00[0], in01[0], in02[0], in03[0], in04[0], in05[0], in06[0], in07[0])
                TRANSPOSE_8x8_16BIT(res00[1], res01[1], res02[1], res03[1], res04[1], res05[1], res06[1], res07[1], in08[0], in09[0], in10[0], in11[0], in12[0], in13[0], in14[0], in15[0])
                TRANSPOSE_8x8_16BIT(res00[2], res01[2], res02[2], res03[2], res04[2], res05[2], res06[2], res07[2], in16[0], in17[0], in18[0], in19[0], in20[0], in21[0], in22[0], in23[0])
                TRANSPOSE_8x8_16BIT(res00[3], res01[3], res02[3], res03[3], res04[3], res05[3], res06[3], res07[3], in24[0], in25[0], in26[0], in27[0], in28[0], in29[0], in30[0], in31[0])

                TRANSPOSE_8x8_16BIT(res08[0], res09[0], res10[0], res11[0], res12[0], res13[0], res14[0], res15[0], in00[1], in01[1], in02[1], in03[1], in04[1], in05[1], in06[1], in07[1])
                TRANSPOSE_8x8_16BIT(res08[1], res09[1], res10[1], res11[1], res12[1], res13[1], res14[1], res15[1], in08[1], in09[1], in10[1], in11[1], in12[1], in13[1], in14[1], in15[1])
                TRANSPOSE_8x8_16BIT(res08[2], res09[2], res10[2], res11[2], res12[2], res13[2], res14[2], res15[2], in16[1], in17[1], in18[1], in19[1], in20[1], in21[1], in22[1], in23[1])
                TRANSPOSE_8x8_16BIT(res08[3], res09[3], res10[3], res11[3], res12[3], res13[3], res14[3], res15[3], in24[1], in25[1], in26[1], in27[1], in28[1], in29[1], in30[1], in31[1])

                TRANSPOSE_8x8_16BIT(res16[0], res17[0], res18[0], res19[0], res20[0], res21[0], res22[0], res23[0], in00[2], in01[2], in02[2], in03[2], in04[2], in05[2], in06[2], in07[2])
                TRANSPOSE_8x8_16BIT(res16[1], res17[1], res18[1], res19[1], res20[1], res21[1], res22[1], res23[1], in08[2], in09[2], in10[2], in11[2], in12[2], in13[2], in14[2], in15[2])
                TRANSPOSE_8x8_16BIT(res16[2], res17[2], res18[2], res19[2], res20[2], res21[2], res22[2], res23[2], in16[2], in17[2], in18[2], in19[2], in20[2], in21[2], in22[2], in23[2])
                TRANSPOSE_8x8_16BIT(res16[3], res17[3], res18[3], res19[3], res20[3], res21[3], res22[3], res23[3], in24[2], in25[2], in26[2], in27[2], in28[2], in29[2], in30[2], in31[2])

                TRANSPOSE_8x8_16BIT(res24[0], res25[0], res26[0], res27[0], res28[0], res29[0], res30[0], res31[0], in00[3], in01[3], in02[3], in03[3], in04[3], in05[3], in06[3], in07[3])
                TRANSPOSE_8x8_16BIT(res24[1], res25[1], res26[1], res27[1], res28[1], res29[1], res30[1], res31[1], in08[3], in09[3], in10[3], in11[3], in12[3], in13[3], in14[3], in15[3])
                TRANSPOSE_8x8_16BIT(res24[2], res25[2], res26[2], res27[2], res28[2], res29[2], res30[2], res31[2], in16[3], in17[3], in18[3], in19[3], in20[3], in21[3], in22[3], in23[3])
                TRANSPOSE_8x8_16BIT(res24[3], res25[3], res26[3], res27[3], res28[3], res29[3], res30[3], res31[3], in24[3], in25[3], in26[3], in27[3], in28[3], in29[3], in30[3], in31[3])

#undef TRANSPOSE_8x8_16BIT
        }
    }

    // Add
    for (int i = 0; i < 2; i++)
    {
#define STORE_LINE(L0, L1, L2, L3, L4, L5, L6, L7, H0, H1, H2, H3, H4, H5, H6, H7, offsetV, offsetH) \
  _mm_storeu_si128((__m128i*)&dst[(0 + (offsetV)) * stride + (offsetH) + 0], L0); \
  _mm_storeu_si128((__m128i*)&dst[(0 + (offsetV)) * stride + (offsetH) + 8], H0); \
  _mm_storeu_si128((__m128i*)&dst[(1 + (offsetV)) * stride + (offsetH) + 0], L1); \
  _mm_storeu_si128((__m128i*)&dst[(1 + (offsetV)) * stride + (offsetH) + 8], H1); \
  _mm_storeu_si128((__m128i*)&dst[(2 + (offsetV)) * stride + (offsetH) + 0], L2); \
  _mm_storeu_si128((__m128i*)&dst[(2 + (offsetV)) * stride + (offsetH) + 8], H2); \
  _mm_storeu_si128((__m128i*)&dst[(3 + (offsetV)) * stride + (offsetH) + 0], L3); \
  _mm_storeu_si128((__m128i*)&dst[(3 + (offsetV)) * stride + (offsetH) + 8], H3); \
  _mm_storeu_si128((__m128i*)&dst[(4 + (offsetV)) * stride + (offsetH) + 0], L4); \
  _mm_storeu_si128((__m128i*)&dst[(4 + (offsetV)) * stride + (offsetH) + 8], H4); \
  _mm_storeu_si128((__m128i*)&dst[(5 + (offsetV)) * stride + (offsetH) + 0], L5); \
  _mm_storeu_si128((__m128i*)&dst[(5 + (offsetV)) * stride + (offsetH) + 8], H5); \
  _mm_storeu_si128((__m128i*)&dst[(6 + (offsetV)) * stride + (offsetH) + 0], L6); \
  _mm_storeu_si128((__m128i*)&dst[(6 + (offsetV)) * stride + (offsetH) + 8], H6); \
  _mm_storeu_si128((__m128i*)&dst[(7 + (offsetV)) * stride + (offsetH) + 0], L7); \
  _mm_storeu_si128((__m128i*)&dst[(7 + (offsetV)) * stride + (offsetH) + 8], H7);

        const int k = i * 2;
        STORE_LINE(in00[k], in01[k], in02[k], in03[k], in04[k], in05[k], in06[k], in07[k], in00[k + 1], in01[k + 1], in02[k + 1], in03[k + 1], in04[k + 1], in05[k + 1], in06[k + 1], in07[k + 1], 0, i * 16)
            STORE_LINE(in08[k], in09[k], in10[k], in11[k], in12[k], in13[k], in14[k], in15[k], in08[k + 1], in09[k + 1], in10[k + 1], in11[k + 1], in12[k + 1], in13[k + 1], in14[k + 1], in15[k + 1], 8, i * 16)
            STORE_LINE(in16[k], in17[k], in18[k], in19[k], in20[k], in21[k], in22[k], in23[k], in16[k + 1], in17[k + 1], in18[k + 1], in19[k + 1], in20[k + 1], in21[k + 1], in22[k + 1], in23[k + 1], 16, i * 16)
            STORE_LINE(in24[k], in25[k], in26[k], in27[k], in28[k], in29[k], in30[k], in31[k], in24[k + 1], in25[k + 1], in26[k + 1], in27[k + 1], in28[k + 1], in29[k + 1], in30[k + 1], in31[k + 1], 24, i * 16)
#undef STORE_LINE
    }
}
#endif 
void xTrMxN(Int bitDepth, Short *block,Short *coeff, Int iWidth, Int iHeight, UInt uiMode)
{
  Int shift_1st = g_aucConvertToBit[iWidth]  + 1 + bitDepth-8; // log2(iWidth) - 1 + g_bitDepth - 8
  Int shift_2nd = g_aucConvertToBit[iHeight]  + 8;                   // log2(iHeight) + 6

  Short tmp[ 64 * 64 ];

#if ETRI_SIMD_MATRIX_TRANSFORM
  switch(iWidth)
  {
      case 32:
          DCT32(block, coeff, iWidth);
         break;

      case 16:
          DCT16(block, coeff, iWidth);
          break;

      case  8:
          DCT8(tmp, block, iWidth, iWidth, shift_1st);
          DCT8(coeff, tmp, iWidth, iWidth, shift_2nd);
          break;

      case  4:
          if (uiMode != REG_DCT)
          {
              DST4(tmp, block, iWidth, iWidth, shift_1st);
              DST4(coeff, tmp, iWidth, iWidth, shift_2nd);
          }
          else
          {
              DCT4(tmp, block, iWidth, iWidth, shift_1st);
              DCT4(coeff, tmp, iWidth, iWidth, shift_2nd);
          }
          break;
  }
#else 
  if( iWidth == 4 && iHeight == 4)
  {
    if (uiMode != REG_DCT)
    {
      fastForwardDst(block,tmp,shift_1st); // Forward DST BY FAST ALGORITHM, block input, tmp output
      fastForwardDst(tmp,coeff,shift_2nd); // Forward DST BY FAST ALGORITHM, tmp input, coeff output
    }
    else
    {
      partialButterfly4(block, tmp, shift_1st, iHeight);
      partialButterfly4(tmp, coeff, shift_2nd, iWidth);
    }

  }
  else if( iWidth == 8 && iHeight == 8)
  {
    partialButterfly8( block, tmp, shift_1st, iHeight );
    partialButterfly8( tmp, coeff, shift_2nd, iWidth );
  }
  else if( iWidth == 16 && iHeight == 16)
  {
    partialButterfly16( block, tmp, shift_1st, iHeight );
    partialButterfly16( tmp, coeff, shift_2nd, iWidth );
  }
  else if( iWidth == 32 && iHeight == 32)
  {
    partialButterfly32( block, tmp, shift_1st, iHeight );
    partialButterfly32( tmp, coeff, shift_2nd, iWidth );
  }
#endif 
}
/** MxN inverse transform (2D)
*  \param coeff input data (transform coefficients)
*  \param block output data (residual)
*  \param iWidth input data (width of transform)
*  \param iHeight input data (height of transform)
*/
void xITrMxN(Int bitDepth, Short *coeff,Short *block, Int iWidth, Int iHeight, UInt uiMode)
{
  Int shift_1st = SHIFT_INV_1ST;
  Int shift_2nd = SHIFT_INV_2ND - (bitDepth-8);

  Short tmp[ 64*64];
#if ETRI_SIMD_MATRIX_TRANSFORM
  switch (iWidth)
  {
      case 32:
          IDCT32(coeff, block, iWidth);
          break;

      case 16:
          IDCT16(coeff, block, iWidth);
          break;

      case  8:
          IDCT8(coeff, block, iWidth);
          break;

      case  4:
          if (uiMode != REG_DCT)
          {
              IDST4(tmp, coeff, iWidth, iWidth, shift_1st);
              IDST4(coeff, tmp, iWidth, iWidth, shift_2nd);
          }
          else
          {
              IDCT4( tmp, coeff, iWidth, iWidth, shift_1st );
              IDCT4( block, tmp, iWidth, iWidth, shift_2nd );
          }
          break;
  }
#else 
  if( iWidth == 4 && iHeight == 4)
  {
    if (uiMode != REG_DCT)
    {
      fastInverseDst(coeff,tmp,shift_1st);    // Inverse DST by FAST Algorithm, coeff input, tmp output
      fastInverseDst(tmp,block,shift_2nd); // Inverse DST by FAST Algorithm, tmp input, coeff output
    }
    else
    {
      partialButterflyInverse4(coeff,tmp,shift_1st,iWidth);
      partialButterflyInverse4(tmp,block,shift_2nd,iHeight);
    }
  }
  else if( iWidth == 8 && iHeight == 8)
  {
    partialButterflyInverse8(coeff,tmp,shift_1st,iWidth);
    partialButterflyInverse8(tmp,block,shift_2nd,iHeight);
  }
  else if( iWidth == 16 && iHeight == 16)
  {
    partialButterflyInverse16(coeff,tmp,shift_1st,iWidth);
    partialButterflyInverse16(tmp,block,shift_2nd,iHeight);
  }
  else if( iWidth == 32 && iHeight == 32)
  {
    partialButterflyInverse32(coeff,tmp,shift_1st,iWidth);
    partialButterflyInverse32(tmp,block,shift_2nd,iHeight);
  }
#endif 
}

#endif //MATRIX_MULT

// To minimize the distortion only. No rate is considered. 
Void TComTrQuant::signBitHidingHDQ( TCoeff* pQCoef, TCoeff* pCoef, UInt const *scan, Int* deltaU, Int width, Int height )
{
  Int lastCG = -1;
  Int absSum = 0 ;
  Int n ;

  for( Int subSet = (width*height-1) >> LOG2_SCAN_SET_SIZE; subSet >= 0; subSet-- )
  {
    Int  subPos     = subSet << LOG2_SCAN_SET_SIZE;
    Int  firstNZPosInCG=SCAN_SET_SIZE , lastNZPosInCG=-1 ;
    absSum = 0 ;

    for(n = SCAN_SET_SIZE-1; n >= 0; --n )
    {
      if( pQCoef[ scan[ n + subPos ]] )
      {
        lastNZPosInCG = n;
        break;
      }
    }

    for(n = 0; n <SCAN_SET_SIZE; n++ )
    {
      if( pQCoef[ scan[ n + subPos ]] )
      {
        firstNZPosInCG = n;
        break;
      }
    }

    for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
    {
      absSum += pQCoef[ scan[ n + subPos ]];
    }

    if(lastNZPosInCG>=0 && lastCG==-1) 
    {
      lastCG = 1 ; 
    }

    if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
    {
      UInt signbit = (pQCoef[scan[subPos+firstNZPosInCG]]>0?0:1) ;
      if( signbit!=(absSum&0x1) )  //compare signbit with sum_parity
      {
        Int minCostInc = MAX_INT,  minPos =-1, finalChange=0, curCost=MAX_INT, curChange=0;
        
        for( n = (lastCG==1?lastNZPosInCG:SCAN_SET_SIZE-1) ; n >= 0; --n )
        {
          UInt blkPos   = scan[ n+subPos ];
          if(pQCoef[ blkPos ] != 0 )
          {
            if(deltaU[blkPos]>0)
            {
              curCost = - deltaU[blkPos]; 
              curChange=1 ;
            }
            else 
            {
              //curChange =-1;
              if(n==firstNZPosInCG && abs(pQCoef[blkPos])==1)
              {
                curCost=MAX_INT ; 
              }
              else
              {
                curCost = deltaU[blkPos]; 
                curChange =-1;
              }
            }
          }
          else
          {
            if(n<firstNZPosInCG)
            {
              UInt thisSignBit = (pCoef[blkPos]>=0?0:1);
              if(thisSignBit != signbit )
              {
                curCost = MAX_INT;
              }
              else
              { 
                curCost = - (deltaU[blkPos])  ;
                curChange = 1 ;
              }
            }
            else
            {
              curCost = - (deltaU[blkPos])  ;
              curChange = 1 ;
            }
          }

          if( curCost<minCostInc)
          {
            minCostInc = curCost ;
            finalChange = curChange ;
            minPos = blkPos ;
          }
        } //CG loop

        if(pQCoef[minPos] == 32767 || pQCoef[minPos] == -32768)
        {
          finalChange = -1;
        }

        if(pCoef[minPos]>=0)
        {
          pQCoef[minPos] += finalChange ; 
        }
        else 
        { 
          pQCoef[minPos] -= finalChange ;
        }  
      } // Hide
    }
    if(lastCG==1) 
    {
      lastCG=0 ;
    }
  } // TU loop

  return;
}

Void TComTrQuant::xQuant( TComDataCU* pcCU, 
                          Int*        pSrc, 
                          TCoeff*     pDes, 
#if ADAPTIVE_QP_SELECTION
                          Int*&       pArlDes,
#endif
                          Int         iWidth, 
                          Int         iHeight, 
                          UInt&       uiAcSum, 
                          TextType    eTType, 
                          UInt        uiAbsPartIdx )
{
	Int*   piCoef    = pSrc;
	TCoeff* piQCoef   = pDes;
#if !ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION
#if ADAPTIVE_QP_SELECTION
	Int*   piArlCCoef = pArlDes;
#endif
#endif
	Int   iAdd = 0;

#if ETRI_TRANSFORM_SKIP_OPTIMIZATION
	Bool useRDOQ = true;
#else
	Bool useRDOQ = pcCU->getTransformSkip(uiAbsPartIdx,eTType) ? m_useRDOQTS:m_useRDOQ;
#endif

#if ETRI_RDOQ_ROUGH_ESTIMATION
	Bool bRDOQ = false;

	QpParam cQpBase;
	Int iQpBase = pcCU->getSlice()->getSliceQpBase();

	Int qpScaled;
	Int qpBDOffset = (eTType == TEXT_LUMA) ? pcCU->getSlice()->getSPS()->getQpBDOffsetY() : pcCU->getSlice()->getSPS()->getQpBDOffsetC();

	if (eTType == TEXT_LUMA)
	{
		qpScaled = iQpBase + qpBDOffset;
	}
	else
	{
		Int chromaQPOffset;
		if (eTType == TEXT_CHROMA_U)
		{
			chromaQPOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
		}
		else
		{
			chromaQPOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
		}
		iQpBase = iQpBase + chromaQPOffset;

		qpScaled = Clip3(-qpBDOffset, 57, iQpBase);
		if (qpScaled < 0)
		{
			qpScaled = qpScaled + qpBDOffset;
		}
		else
		{
			qpScaled = g_aucChromaScale[qpScaled] + qpBDOffset;
		}
	}
	cQpBase.setQpParam(qpScaled);

	UInt uiLog2TrSize = g_aucConvertToBit[iWidth] + 2;

#if ETRI_SCALING_LIST_OPTIMIZATION
	Int quantScales[6] = { 26214, 23302, 20560, 18396, 16384, 14564 };
	Int *piQuantCoeff = &quantScales[m_cQP.m_iRem];
#else    
	Int *piQuantCoeff = 0;
	Int scalingListType = (pcCU->isIntra(uiAbsPartIdx) ? 0 : 3) + g_eTTable[(Int)eTType];
	piQuantCoeff = getQuantCoeff(scalingListType, m_cQP.m_iRem, uiLog2TrSize - 2);
#endif 

	UInt uiBitDepth = eTType == TEXT_LUMA ? g_bitDepthY : g_bitDepthC;
	Int iTransformShift = MAX_TR_DYNAMIC_RANGE - uiBitDepth - uiLog2TrSize;  // Represents scaling through forward transform

	Int iQBits = QUANT_SHIFT + cQpBase.m_iPer + iTransformShift;
	iAdd = (pcCU->getSlice()->getSliceType() == I_SLICE ? 171 : 85) << (iQBits - 9);

	for (Int n = 0; n < 16; n++) // further optimization needed
	{
		Int iLevel;

#if ETRI_EM_OPERATION_OPTIMIZATION
		UInt uiBlockPos = (n & 0x03) + (iWidth * (n >> 2));
#else
		UInt uiBlockPos = (n % 4) + (iWidth * (n / 4));
#endif

		iLevel = piCoef[uiBlockPos];

#if ETRI_SCALING_LIST_OPTIMIZATION
		Int64 tmpLevel = (Int64)abs(iLevel) * *piQuantCoeff;
#else
		Int64 tmpLevel = (Int64)abs(iLevel) * piQuantCoeff[uiBlockPos];
#endif
		iLevel = (Int)((tmpLevel + iAdd) >> iQBits);

		if (iLevel)
		{
			bRDOQ = true;
			break;
		}
	}

	if (useRDOQ && (eTType == TEXT_LUMA || RDOQ_CHROMA) && bRDOQ)
#else
	if ( useRDOQ && (eTType == TEXT_LUMA || RDOQ_CHROMA))
#endif
	{
#if ADAPTIVE_QP_SELECTION
		xRateDistOptQuant( pcCU, piCoef, pDes, pArlDes, iWidth, iHeight, uiAcSum, eTType, uiAbsPartIdx );
#else
		xRateDistOptQuant( pcCU, piCoef, pDes, iWidth, iHeight, uiAcSum, eTType, uiAbsPartIdx );
#endif
	}
	else
	{
#if ETRI_RDOQ_ROUGH_ESTIMATION
		uiAcSum = 0;
		::memset(piQCoef, 0, sizeof(Int)*iWidth*iHeight);
#else
		Int*   piArlCCoef = pArlDes;
		const UInt   log2BlockSize   = g_aucConvertToBit[ iWidth ] + 2;

		UInt scanIdx = pcCU->getCoefScanIdx(uiAbsPartIdx, iWidth, eTType==TEXT_LUMA, pcCU->isIntra(uiAbsPartIdx));
		const UInt *scan = g_auiSigLastScan[ scanIdx ][ log2BlockSize - 1 ];

		Int deltaU[32*32] ;

#if ADAPTIVE_QP_SELECTION
		QpParam cQpBase;
		Int iQpBase = pcCU->getSlice()->getSliceQpBase();

		Int qpScaled;
		Int qpBDOffset = (eTType == TEXT_LUMA)? pcCU->getSlice()->getSPS()->getQpBDOffsetY() : pcCU->getSlice()->getSPS()->getQpBDOffsetC();

		if(eTType == TEXT_LUMA)
		{
			qpScaled = iQpBase + qpBDOffset;
		}
		else
		{
			Int chromaQPOffset;
			if(eTType == TEXT_CHROMA_U)
			{
				chromaQPOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
			}
			else
			{
				chromaQPOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
			}
			iQpBase = iQpBase + chromaQPOffset;

			qpScaled = Clip3( -qpBDOffset, 57, iQpBase);

			if(qpScaled < 0)
			{
				qpScaled = qpScaled +  qpBDOffset;
			}
			else
			{
				qpScaled = g_aucChromaScale[ qpScaled ] + qpBDOffset;
			}
		}
		cQpBase.setQpParam(qpScaled);
#endif

		UInt uiLog2TrSize = g_aucConvertToBit[ iWidth ] + 2;
		Int scalingListType = (pcCU->isIntra(uiAbsPartIdx) ? 0 : 3) + g_eTTable[(Int)eTType];
		assert(scalingListType < SCALING_LIST_NUM);
		Int *piQuantCoeff = 0;
		piQuantCoeff = getQuantCoeff(scalingListType,m_cQP.m_iRem,uiLog2TrSize-2);

		UInt uiBitDepth = eTType == TEXT_LUMA ? g_bitDepthY : g_bitDepthC;
		Int iTransformShift = MAX_TR_DYNAMIC_RANGE - uiBitDepth - uiLog2TrSize;  // Represents scaling through forward transform

#if ADAPTIVE_QP_SELECTION
		Int iQBits = QUANT_SHIFT + cQpBase.m_iPer + iTransformShift;
		iAdd = (pcCU->getSlice()->getSliceType()==I_SLICE ? 171 : 85) << (iQBits-9);
		Int iQBitsC = QUANT_SHIFT + cQpBase.m_iPer + iTransformShift - ARL_C_PRECISION;  
		Int iAddC   = 1 << (iQBitsC-1);
#else
		Int iQBits = QUANT_SHIFT + m_cQP.m_iPer + iTransformShift;                // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
		iAdd = (pcCU->getSlice()->getSliceType()==I_SLICE ? 171 : 85) << (iQBits-9);
#endif

		Int qBits8 = iQBits-8;
		for( Int n = 0; n < iWidth*iHeight; n++ )
		{
			Int iLevel;
			Int  iSign;
			UInt uiBlockPos = n;
			iLevel  = piCoef[uiBlockPos];
			iSign   = (iLevel < 0 ? -1: 1);      

#if ADAPTIVE_QP_SELECTION
			Int64 tmpLevel = (Int64)ETRI_sABS(iLevel) * piQuantCoeff[uiBlockPos];
			if( m_bUseAdaptQpSelect )
			{
				piArlCCoef[uiBlockPos] = (Int)((tmpLevel + iAddC ) >> iQBitsC);
			}
			iLevel = (Int)((tmpLevel + iAdd ) >> iQBits);
			deltaU[uiBlockPos] = (Int)((tmpLevel - (iLevel<<iQBits) )>> qBits8);
#else
			iLevel = ((Int64)abs(iLevel) * piQuantCoeff[uiBlockPos] + iAdd ) >> iQBits;
			deltaU[uiBlockPos] = (Int)( ((Int64)abs(piCoef[uiBlockPos]) * piQuantCoeff[uiBlockPos] - (iLevel<<iQBits) )>> qBits8 );
#endif
			uiAcSum += iLevel;
			iLevel *= iSign;        
			piQCoef[uiBlockPos] = Clip3( -32768, 32767, iLevel );
		} // for n


		if( pcCU->getSlice()->getPPS()->getSignHideFlag() )
		{
			if(uiAcSum>=2)
			{
				signBitHidingHDQ( piQCoef, piCoef, scan, deltaU, iWidth, iHeight ) ;
			}
		}
#endif
	} //if RDOQ
	//return;

}

#if ETRI_SIMD_DEQUANTIZATION
Void TComTrQuant::xDeQuant(Int bitDepth, const TCoeff* pSrc, Int* pDes, Int iWidth, Int iHeight, Int scalingListType)
{
	if (iWidth > (Int)m_uiMaxTrSize)
	{
		iWidth = m_uiMaxTrSize;
		iHeight = m_uiMaxTrSize;
	}

#if ETRI_SCALING_LIST_OPTIMIZATION
	Int iShift;
#else
	Int iShift, iAdd, iCoeffQ;
#endif
	UInt uiLog2TrSize = g_aucConvertToBit[iWidth] + 2;

	Int iTransformShift = MAX_TR_DYNAMIC_RANGE - bitDepth - uiLog2TrSize;
	iShift = QUANT_IQUANT_SHIFT - QUANT_SHIFT - iTransformShift;

#if !ETRI_SCALING_LIST_OPTIMIZATION
	TCoeff clipQCoef;
#endif

	int valueToAdd;
	int invQuantScales[6] = { 40, 45, 51, 57, 64, 72 };

#if !ETRI_SCALING_LIST_OPTIMIZATION
	if (getUseScalingList())
	{
		iShift += 4;
		Int *piDequantCoef = getDequantCoeff(scalingListType, m_cQP.m_iRem, uiLog2TrSize - 2);

		if (iShift > m_cQP.m_iPer)
		{
			iAdd = 1 << (iShift - m_cQP.m_iPer - 1);

			for (Int n = 0; n < iWidth*iHeight; n++)
			{
				clipQCoef = Clip3(-32768, 32767, pSrc[n]);
				iCoeffQ = ((clipQCoef * piDequantCoef[n]) + iAdd) >> (iShift - m_cQP.m_iPer);
				pDes[n] = Clip3(-32768, 32767, iCoeffQ);
			}
		}
		else
		{
			for (Int n = 0; n < iWidth*iHeight; n++)
			{
				clipQCoef = Clip3(-32768, 32767, pSrc[n]);
				iCoeffQ = Clip3(-32768, 32767, clipQCoef * piDequantCoef[n]); // Clip to avoid possible overflow in following shift left operation
				pDes[n] = Clip3(-32768, 32767, iCoeffQ << (m_cQP.m_iPer - iShift));
			}
		}
	}
	else
#endif
	{
		valueToAdd = 1 << (iShift - 1);
		int scale = invQuantScales[m_cQP.m_iRem] << m_cQP.m_iPer;

		__m128i vScale = _mm_set1_epi32(scale);
		__m128i vAdd = _mm_set1_epi32(valueToAdd);

		for (int n = 0; n < iWidth*iHeight; n = n + 8)
		{
			__m128i quantCoef1, quantCoef2, quantCoef12, sign;

			quantCoef1 = _mm_loadu_si128((__m128i*)(pSrc + n));
			quantCoef2 = _mm_loadu_si128((__m128i*)(pSrc + n + 4));

			quantCoef12 = _mm_packs_epi32(quantCoef1, quantCoef2);
			sign = _mm_srai_epi16(quantCoef12, 15);
			quantCoef1 = _mm_unpacklo_epi16(quantCoef12, sign);
			quantCoef2 = _mm_unpackhi_epi16(quantCoef12, sign);

			quantCoef1 = _mm_sra_epi32(_mm_add_epi32(_mm_mullo_epi32(quantCoef1, vScale), vAdd), _mm_cvtsi32_si128(iShift));
			quantCoef2 = _mm_sra_epi32(_mm_add_epi32(_mm_mullo_epi32(quantCoef2, vScale), vAdd), _mm_cvtsi32_si128(iShift));

			quantCoef12 = _mm_packs_epi32(quantCoef1, quantCoef2);
			sign = _mm_srai_epi16(quantCoef12, 15);
			quantCoef1 = _mm_unpacklo_epi16(quantCoef12, sign);
			_mm_storeu_si128((__m128i*)(pDes + n), quantCoef1);
			quantCoef2 = _mm_unpackhi_epi16(quantCoef12, sign);
			_mm_storeu_si128((__m128i*)(pDes + n + 4), quantCoef2);
		}
	}
}
#else
Void TComTrQuant::xDeQuant(Int bitDepth, const TCoeff* pSrc, Int* pDes, Int iWidth, Int iHeight, Int scalingListType )
{
  
  const TCoeff* piQCoef   = pSrc;
  Int*   piCoef    = pDes;
  
  if ( iWidth > (Int)m_uiMaxTrSize )
  {
    iWidth  = m_uiMaxTrSize;
    iHeight = m_uiMaxTrSize;
  }
  
  Int iShift,iAdd,iCoeffQ;
  UInt uiLog2TrSize = g_aucConvertToBit[ iWidth ] + 2;

  Int iTransformShift = MAX_TR_DYNAMIC_RANGE - bitDepth - uiLog2TrSize;

  iShift = QUANT_IQUANT_SHIFT - QUANT_SHIFT - iTransformShift;

  TCoeff clipQCoef;

  if(getUseScalingList())
  {
    iShift += 4;
    Int *piDequantCoef = getDequantCoeff(scalingListType,m_cQP.m_iRem,uiLog2TrSize-2);

    if(iShift > m_cQP.m_iPer)
    {
      iAdd = 1 << (iShift - m_cQP.m_iPer - 1);
      
      for( Int n = 0; n < iWidth*iHeight; n++ )
      {
        clipQCoef = Clip3( -32768, 32767, piQCoef[n] );
        iCoeffQ = ((clipQCoef * piDequantCoef[n]) + iAdd ) >> (iShift -  m_cQP.m_iPer);
        piCoef[n] = Clip3(-32768,32767,iCoeffQ);
      }
    }
    else
    {
      for( Int n = 0; n < iWidth*iHeight; n++ )
      {
        clipQCoef = Clip3( -32768, 32767, piQCoef[n] );
        iCoeffQ   = Clip3( -32768, 32767, clipQCoef * piDequantCoef[n] ); // Clip to avoid possible overflow in following shift left operation
        piCoef[n] = Clip3( -32768, 32767, iCoeffQ << ( m_cQP.m_iPer - iShift ) );
      }
    }
  }
  else
  {
    iAdd = 1 << (iShift-1);
    Int scale = g_invQuantScales[m_cQP.m_iRem] << m_cQP.m_iPer;

    for( Int n = 0; n < iWidth*iHeight; n++ )
    {
      clipQCoef = Clip3( -32768, 32767, piQCoef[n] );
      iCoeffQ = ( clipQCoef * scale + iAdd ) >> iShift;
      piCoef[n] = Clip3(-32768,32767,iCoeffQ);
    }
  }
}
#endif

Void TComTrQuant::init( UInt uiMaxTrSize,
                       Bool bUseRDOQ,  
                       Bool bUseRDOQTS,
                       Bool bEnc, Bool useTransformSkipFast
#if ADAPTIVE_QP_SELECTION
                       , Bool bUseAdaptQpSelect
#endif
                       )
{
  m_uiMaxTrSize  	= uiMaxTrSize;
  m_bEnc         	= bEnc;
  m_useRDOQ  		= bUseRDOQ;
  m_useRDOQTS  		= bUseRDOQTS;
#if ADAPTIVE_QP_SELECTION
  m_bUseAdaptQpSelect = bUseAdaptQpSelect;
#endif
  m_useTransformSkipFast = useTransformSkipFast;
}

Void TComTrQuant::transformNxN( TComDataCU* pcCU, 
                               Pel*        pcResidual, 
                               UInt        uiStride, 
                               TCoeff*     rpcCoeff, 
#if ADAPTIVE_QP_SELECTION
                               Int*&       rpcArlCoeff, 
#endif
                               UInt        uiWidth, 
                               UInt        uiHeight, 
                               UInt&       uiAbsSum, 
                               TextType    eTType, 
                               UInt        uiAbsPartIdx,
                               Bool        useTransformSkip
                               )
{
#if !ETRI_LOSSLESS_OPTIMIZATION
	if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
	{
	uiAbsSum=0;
	for (UInt k = 0; k<uiHeight; k++)
	{
	for (UInt j = 0; j<uiWidth; j++)
	{
	rpcCoeff[k*uiWidth+j]= pcResidual[k*uiStride+j];
	uiAbsSum += abs(pcResidual[k*uiStride+j]);
	}
	}
	return;
	}
#endif
	UInt uiMode;  //luma intra pred
	if(eTType == TEXT_LUMA && pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA )
	{
		uiMode = pcCU->getLumaIntraDir( uiAbsPartIdx );
	}
	else
	{
		uiMode = REG_DCT;
	}

	uiAbsSum = 0;
	assert( (pcCU->getSlice()->getSPS()->getMaxTrSize() >= uiWidth) );
	Int bitDepth = eTType == TEXT_LUMA ? g_bitDepthY : g_bitDepthC;

	if(useTransformSkip)
	{
		xTransformSkip(bitDepth, pcResidual, uiStride, m_plTempCoeff, uiWidth, uiHeight );
	}
	else
	{
		xT(bitDepth, uiMode, pcResidual, uiStride, m_plTempCoeff, uiWidth, uiHeight );
	}

	xQuant( pcCU, m_plTempCoeff, rpcCoeff,
#if ADAPTIVE_QP_SELECTION
	rpcArlCoeff,
#endif
	uiWidth, uiHeight, uiAbsSum, eTType, uiAbsPartIdx );
}

Void TComTrQuant::invtransformNxN( Bool transQuantBypass, TextType eText, UInt uiMode,Pel* rpcResidual, UInt uiStride, TCoeff*   pcCoeff, UInt uiWidth, UInt uiHeight,  Int scalingListType, Bool useTransformSkip )
{
  if(transQuantBypass)
  {
    for (UInt k = 0; k<uiHeight; k++)
    {
      for (UInt j = 0; j<uiWidth; j++)
      {
        rpcResidual[k*uiStride+j] = pcCoeff[k*uiWidth+j];
      }
    } 
    return;
  }
  Int bitDepth = eText == TEXT_LUMA ? g_bitDepthY : g_bitDepthC;
  xDeQuant(bitDepth, pcCoeff, m_plTempCoeff, uiWidth, uiHeight, scalingListType);
  if(useTransformSkip == true)
  {
    xITransformSkip(bitDepth, m_plTempCoeff, rpcResidual, uiStride, uiWidth, uiHeight );
  }
  else
  {
    xIT(bitDepth, uiMode, m_plTempCoeff, rpcResidual, uiStride, uiWidth, uiHeight );
  }
}

Void TComTrQuant::invRecurTransformNxN( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eTxt, Pel* rpcResidual, UInt uiAddr, UInt uiStride, UInt uiWidth, UInt uiHeight, UInt uiMaxTrMode, UInt uiTrMode, TCoeff* rpcCoeff )
{
  if( !pcCU->getCbf(uiAbsPartIdx, eTxt, uiTrMode) )
  {
    return;
  }  
  const UInt stopTrMode = pcCU->getTransformIdx( uiAbsPartIdx );
  
  if( uiTrMode == stopTrMode )
  {
    UInt uiDepth      = pcCU->getDepth( uiAbsPartIdx ) + uiTrMode;
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth ] + 2;
    if( eTxt != TEXT_LUMA && uiLog2TrSize == 2 )
    {
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( uiDepth - 1 ) << 1 );
      if( ( uiAbsPartIdx % uiQPDiv ) != 0 )
      {
        return;
      }
      uiWidth  <<= 1;
      uiHeight <<= 1;
    }
    Pel* pResi = rpcResidual + uiAddr;
    Int scalingListType = (pcCU->isIntra(uiAbsPartIdx) ? 0 : 3) + g_eTTable[(Int)eTxt];
    assert(scalingListType < SCALING_LIST_NUM);
#if ETRI_LOSSLESS_OPTIMIZATION
	invtransformNxN(false, eTxt, REG_DCT, pResi, uiStride, rpcCoeff, uiWidth, uiHeight, scalingListType, pcCU->getTransformSkip(uiAbsPartIdx, eTxt));
#else
    invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), eTxt, REG_DCT, pResi, uiStride, rpcCoeff, uiWidth, uiHeight, scalingListType, pcCU->getTransformSkip(uiAbsPartIdx, eTxt) );
#endif
  }
  else
  {
    uiTrMode++;
    uiWidth  >>= 1;
    uiHeight >>= 1;
    Int trWidth = uiWidth, trHeight = uiHeight;
    UInt uiAddrOffset = trHeight * uiStride;
    UInt uiCoefOffset = trWidth * trHeight;
    UInt uiPartOffset = pcCU->getTotalNumPart() >> ( uiTrMode << 1 );
    {
      invRecurTransformNxN( pcCU, uiAbsPartIdx, eTxt, rpcResidual, uiAddr                         , uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff ); rpcCoeff += uiCoefOffset; uiAbsPartIdx += uiPartOffset;
      invRecurTransformNxN( pcCU, uiAbsPartIdx, eTxt, rpcResidual, uiAddr + trWidth               , uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff ); rpcCoeff += uiCoefOffset; uiAbsPartIdx += uiPartOffset;
      invRecurTransformNxN( pcCU, uiAbsPartIdx, eTxt, rpcResidual, uiAddr + uiAddrOffset          , uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff ); rpcCoeff += uiCoefOffset; uiAbsPartIdx += uiPartOffset;
      invRecurTransformNxN( pcCU, uiAbsPartIdx, eTxt, rpcResidual, uiAddr + uiAddrOffset + trWidth, uiStride, uiWidth, uiHeight, uiMaxTrMode, uiTrMode, rpcCoeff );
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

/** Wrapper function between HM interface and core NxN forward transform (2D) 
 *  \param piBlkResi input data (residual)
 *  \param psCoeff output data (transform coefficients)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void TComTrQuant::xT(Int bitDepth, UInt uiMode, Pel* piBlkResi, UInt uiStride, Int* psCoeff, Int iWidth, Int iHeight )
{
#if MATRIX_MULT  
  Int iSize = iWidth;
  xTr(bitDepth, piBlkResi,psCoeff,uiStride,(UInt)iSize,uiMode);
#else
  Int j;
  Short block[ 32 * 32 ];
  Short coeff[ 32 * 32 ];
  for (j = 0; j < iHeight; j++)
  {    
    memcpy( block + j * iWidth, piBlkResi + j * uiStride, iWidth * sizeof( Short ) );
  }
  xTrMxN(bitDepth, block, coeff, iWidth, iHeight, uiMode );
#if ETRI_TRANSFORM_TYPE_CONVERSION_SIMD
  __m128i xmm;
  for (j = 0; j < iHeight*iWidth; j += 8)
  {
      xmm = _mm_loadu_si128((__m128i*)&coeff[j]);
      _mm_storeu_si128((__m128i*)&psCoeff[j], _mm_cvtepi16_epi32(xmm));
      _mm_storeu_si128((__m128i*)&psCoeff[j + 4], _mm_cvtepi16_epi32(_mm_shuffle_epi32(xmm, 78)));
}
#else 
  for ( j = 0; j < iHeight * iWidth; j++ )
  {    
    psCoeff[ j ] = coeff[ j ];
  }
#endif 
#endif  
}


/** Wrapper function between HM interface and core NxN inverse transform (2D) 
 *  \param plCoef input data (transform coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void TComTrQuant::xIT(Int bitDepth, UInt uiMode, Int* plCoef, Pel* pResidual, UInt uiStride, Int iWidth, Int iHeight )
{
#if MATRIX_MULT  
  Int iSize = iWidth;
  xITr(bitDepth, plCoef,pResidual,uiStride,(UInt)iSize,uiMode);
#else
  Int j;
  {
    Short block[ 32 * 32 ];
    Short coeff[ 32 * 32 ];
#if ETRI_TRANSFORM_TYPE_CONVERSION_SIMD
    __m128i xmm[2];
    for (j = 0; j < iHeight*iWidth; j += 8)
    {
        xmm[0] = _mm_loadu_si128((__m128i*)&plCoef[j]);
        xmm[1] = _mm_loadu_si128((__m128i*)&plCoef[j+4]);
        _mm_storeu_si128((__m128i *)&coeff[j], _mm_packs_epi32(xmm[0], xmm[1]));

    }
#else 
    for ( j = 0; j < iHeight * iWidth; j++ )
    {    
      coeff[j] = (Short)plCoef[j];
    }
#endif 
    xITrMxN(bitDepth, coeff, block, iWidth, iHeight, uiMode );
    {
      for ( j = 0; j < iHeight; j++ )
      {    
        memcpy( pResidual + j * uiStride, block + j * iWidth, iWidth * sizeof(Short) );
      }
    }
    return ;
  }
#endif  
}
 
/** Wrapper function between HM interface and core 4x4 transform skipping
 *  \param piBlkResi input data (residual)
 *  \param psCoeff output data (transform coefficients)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 */
Void TComTrQuant::xTransformSkip(Int bitDepth, Pel* piBlkResi, UInt uiStride, Int* psCoeff, Int width, Int height )
{
  assert( width == height );
  UInt uiLog2TrSize = g_aucConvertToBit[ width ] + 2;
  Int  shift = MAX_TR_DYNAMIC_RANGE - bitDepth - uiLog2TrSize;
  UInt transformSkipShift;
  Int  j,k;
  if(shift >= 0)
  {
    transformSkipShift = shift;
    for (j = 0; j < height; j++)
    {    
      for(k = 0; k < width; k ++)
      {
        psCoeff[j*height + k] = piBlkResi[j * uiStride + k] << transformSkipShift;      
      }
    }
  }
  else
  {
    //The case when uiBitDepth > 13
    Int offset;
    transformSkipShift = -shift;
    offset = (1 << (transformSkipShift - 1));
    for (j = 0; j < height; j++)
    {    
      for(k = 0; k < width; k ++)
      {
        psCoeff[j*height + k] = (piBlkResi[j * uiStride + k] + offset) >> transformSkipShift;      
      }
    }
  }
}

/** Wrapper function between HM interface and core NxN transform skipping 
 *  \param plCoef input data (coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 */
Void TComTrQuant::xITransformSkip(Int bitDepth, Int* plCoef, Pel* pResidual, UInt uiStride, Int width, Int height )
{
  assert( width == height );
  UInt uiLog2TrSize = g_aucConvertToBit[ width ] + 2;
  Int  shift = MAX_TR_DYNAMIC_RANGE - bitDepth - uiLog2TrSize;
  UInt transformSkipShift; 
  Int  j,k;
  if(shift > 0)
  {
    Int offset;
    transformSkipShift = shift;
    offset = (1 << (transformSkipShift -1));
    for ( j = 0; j < height; j++ )
    {    
      for(k = 0; k < width; k ++)
      {
        pResidual[j * uiStride + k] =  (plCoef[j*width+k] + offset) >> transformSkipShift;
      } 
    }
  }
  else
  {
    //The case when uiBitDepth >= 13
    transformSkipShift = - shift;
    for ( j = 0; j < height; j++ )
    {    
      for(k = 0; k < width; k ++)
      {
        pResidual[j * uiStride + k] =  plCoef[j*width+k] << transformSkipShift;
      }
    }
  }
}

/** RDOQ with CABAC
 * \param pcCU pointer to coding unit structure
 * \param plSrcCoeff pointer to input buffer
 * \param piDstCoeff reference to pointer to output buffer
 * \param uiWidth block width
 * \param uiHeight block height
 * \param uiAbsSum reference to absolute sum of quantized transform coefficient
 * \param eTType plane type / luminance or chrominance
 * \param uiAbsPartIdx absolute partition index
 * \returns Void
 * Rate distortion optimized quantization for entropy
 * coding engines using probability models like CABAC
 */
Void TComTrQuant::xRateDistOptQuant                 ( TComDataCU*                     pcCU,
                                                      Int*                            plSrcCoeff,
                                                      TCoeff*                         piDstCoeff,
#if ADAPTIVE_QP_SELECTION
                                                      Int*&                           piArlDstCoeff,
#endif
                                                      UInt                            uiWidth,
                                                      UInt                            uiHeight,
                                                      UInt&                           uiAbsSum,
                                                      TextType                        eTType,
                                                      UInt                            uiAbsPartIdx )
{
	UInt uiLog2TrSize = g_aucConvertToBit[ uiWidth ] + 2;

	UInt uiBitDepth = eTType == TEXT_LUMA ? g_bitDepthY : g_bitDepthC;
	Int iTransformShift = MAX_TR_DYNAMIC_RANGE - uiBitDepth - uiLog2TrSize;  // Represents scaling through forward transform
	UInt       uiGoRiceParam       = 0;
	Double     d64BlockUncodedCost = 0;
	const UInt uiLog2BlkSize       = g_aucConvertToBit[ uiWidth ] + 2;
	const UInt uiMaxNumCoeff       = uiWidth * uiHeight;
	Int scalingListType = (pcCU->isIntra(uiAbsPartIdx) ? 0 : 3) + g_eTTable[(Int)eTType];
	assert(scalingListType < SCALING_LIST_NUM);

	Int iQBits = QUANT_SHIFT + m_cQP.m_iPer + iTransformShift;                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
#if ETRI_SCALING_LIST_OPTIMIZATION
	Double errScale[4][6] = {
	{ 4.6567549848772173e-008, 5.8933682965265348e-008, 7.5701373222910273e-008, 9.4559066580977233e-008, 1.1920928955078125e-007, 1.5086504887537272e-007 },
	{ 1.8627019939508869e-007, 2.3573473186106139e-007, 3.0280549289164109e-007, 3.7823626632390893e-007, 4.7683715820312500e-007, 6.0346019550149088e-007 },
	{ 7.4508079758035477e-007, 9.4293892744424556e-007, 1.2112219715665644e-006, 1.5129450652956357e-006, 1.9073486328125000e-006, 2.4138407820059635e-006 },
	{ 2.9803231903214191e-006, 3.7717557097769823e-006, 4.8448878862662575e-006, 6.0517802611825429e-006, 7.6293945312500000e-006, 9.6553631280238541e-006 }
	};
	Double *pdErrScale = &errScale[uiLog2TrSize - 2][m_cQP.m_iRem];

	Int quantScales[6] = { 26214, 23302, 20560, 18396, 16384, 14564 };
	Int *piQCoef = &quantScales[m_cQP.m_iRem];
#else
	Double *pdErrScaleOrg = getErrScaleCoeff(scalingListType,uiLog2TrSize-2,m_cQP.m_iRem);
	Double *pdErrScale = pdErrScaleOrg;

	Int *piQCoefOrg = getQuantCoeff(scalingListType,m_cQP.m_iRem,uiLog2TrSize-2);
	Int *piQCoef = piQCoefOrg;
#endif
#if !ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION
#if ADAPTIVE_QP_SELECTION
	Int iQBitsC = iQBits - ARL_C_PRECISION;
	Int iAddC =  1 << (iQBitsC-1);
#endif
#endif	
	UInt uiScanIdx = pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, eTType==TEXT_LUMA, pcCU->isIntra(uiAbsPartIdx));

#if ADAPTIVE_QP_SELECTION
	//#if !ETRI_RDOQ_CODE_OPTIMIZATION
	//  memset(piArlDstCoeff, 0, sizeof(Int) *  uiMaxNumCoeff);				// by serdong 
	//#endif
#endif

	Double pdCostCoeff [ 32 * 32 ];
	Double pdCostSig   [ 32 * 32 ];
	Double pdCostCoeff0[ 32 * 32 ];
#if !ETRI_RDOQ_CODE_OPTIMIZATION
	::memset( pdCostCoeff, 0, sizeof(Double) *  uiMaxNumCoeff );
	::memset( pdCostSig,   0, sizeof(Double) *  uiMaxNumCoeff );
#endif
	Int rateIncUp   [ 32 * 32 ];
	Int rateIncDown [ 32 * 32 ];
	Int sigRateDelta[ 32 * 32 ];
	Int deltaU      [ 32 * 32 ];
#if !ETRI_RDOQ_CODE_OPTIMIZATION
	::memset( rateIncUp,    0, sizeof(Int) *  uiMaxNumCoeff );
	::memset( rateIncDown,  0, sizeof(Int) *  uiMaxNumCoeff );
#endif
	::memset( sigRateDelta, 0, sizeof(Int) *  uiMaxNumCoeff );
#if !ETRI_RDOQ_CODE_OPTIMIZATION
	::memset( deltaU,       0, sizeof(Int) *  uiMaxNumCoeff );
#endif  

	const UInt * scanCG;
	{
		scanCG = g_auiSigLastScan[ uiScanIdx ][ uiLog2BlkSize > 3 ? uiLog2BlkSize-2-1 : 0  ];
		if( uiLog2BlkSize == 3 )
		{
			scanCG = g_sigLastScan8x8[ uiScanIdx ];
		}
		else if( uiLog2BlkSize == 5 )
		{
			scanCG = g_sigLastScanCG32x32;
		}
	}
	const UInt uiCGSize = (1 << MLS_CG_SIZE);         // 16
	Double pdCostCoeffGroupSig[ MLS_GRP_NUM ];
	UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];
	UInt uiNumBlkSide = uiWidth / MLS_CG_SIZE;
	Int iCGLastScanPos = -1;

	UInt    uiCtxSet            = 0;
	Int     c1                  = 1;
	Int     c2                  = 0;
	Double  d64BaseCost         = 0;
	Int     iLastScanPos        = -1;

	UInt    c1Idx     = 0;
	UInt    c2Idx     = 0;
	Int     baseLevel;

	const UInt *scan = g_auiSigLastScan[ uiScanIdx ][ uiLog2BlkSize - 1 ];

	::memset( pdCostCoeffGroupSig,   0, sizeof(Double) * MLS_GRP_NUM );
	::memset( uiSigCoeffGroupFlag,   0, sizeof(UInt) * MLS_GRP_NUM );

	UInt uiCGNum = uiWidth * uiHeight >> MLS_CG_SIZE;
	Int iScanPos;
	coeffGroupRDStats rdStats;     

#if ETRI_RDOQ_SIMD_OPTIMIZATION
	Int elLevelDouble[1024];
	Int euiMaxAbsLevel[1024];

	__m128i xmm0, xmm1;
	__m128i xmm2 = _mm_set1_epi32(MAX_INT - (1 << (iQBits - 1)));
	__m128i xmm3 = _mm_set1_epi32((1 << (iQBits - 1)));
#if ETRI_SCALING_LIST_OPTIMIZATION
	__m128i xmm4 = _mm_set1_epi32(*piQCoef);
#endif

	for (Int i = 0; i < uiMaxNumCoeff; i += 4)
	{
		xmm0 = _mm_loadu_si128((__m128i *)&plSrcCoeff[i]);
#if ETRI_SCALING_LIST_OPTIMIZATION    
		xmm1 = _mm_mullo_epi32(_mm_abs_epi32(xmm0),xmm4);
#else
		xmm1 = _mm_mullo_epi32(_mm_abs_epi32(xmm0), _mm_loadu_si128((__m128i *)(piQCoef + i)));
#endif
		xmm0 = _mm_min_epi32(xmm1, xmm2);

		_mm_storeu_si128((__m128i *)&elLevelDouble[i], xmm0);
		_mm_storeu_si128((__m128i *)&euiMaxAbsLevel[i], _mm_srai_epi32(_mm_add_epi32(xmm0, xmm3), iQBits));
	}
#endif

#if ETRI_RDOQ_ZONAL_CODING
	::memset(piDstCoeff, 0, sizeof(TCoeff)*uiWidth*uiHeight);

#if ETRI_INTER_RDOQ_ZONAL_CODING
	if (!pcCU->isIntra(uiAbsPartIdx))
	{
		if (uiWidth == 8)
		uiCGNum = 2; // half region
		else if (uiWidth == 16)
		uiCGNum = 6; // 9/16 region
		else if (uiWidth == 32)
		uiCGNum = 10; // 35/64 region  
	}
#else 
	if (uiWidth == 8)
	uiCGNum = 3; // half region
	else if (uiWidth == 16)
	uiCGNum = 10; // 9/16 region
	else if (uiWidth == 32)
	uiCGNum = 36; // 35/64 region  
#endif 
#endif

	for (Int iCGScanPos = uiCGNum-1; iCGScanPos >= 0; iCGScanPos--)
	{
		UInt uiCGBlkPos = scanCG[ iCGScanPos ];
		UInt uiCGPosY   = uiCGBlkPos / uiNumBlkSide;
		UInt uiCGPosX   = uiCGBlkPos - (uiCGPosY * uiNumBlkSide);
		::memset( &rdStats, 0, sizeof (coeffGroupRDStats));

		const Int patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, uiWidth, uiHeight);
		for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)
		{
			iScanPos = iCGScanPos*uiCGSize + iScanPosinCG;

			//===== quantization =====
			UInt    uiBlkPos          = scan[iScanPos];
			// set coeff
#if ETRI_SCALING_LIST_OPTIMIZATION
			Double dTemp = *pdErrScale;
#else
			Double dTemp = pdErrScale[uiBlkPos];
#endif
			Int lLevelDouble = plSrcCoeff[ uiBlkPos ];
#if ETRI_RDOQ_SIMD_OPTIMIZATION
			lLevelDouble = elLevelDouble[uiBlkPos];
#else
#if ETRI_SCALING_LIST_OPTIMIZATION
			Int uiQ = *piQCoef;
#else
			Int uiQ  = piQCoef[uiBlkPos];
#endif
			lLevelDouble = (Int)min<Int64>((Int64)abs((Int)lLevelDouble) * uiQ , MAX_INT - (1 << (iQBits - 1)));
#endif
#if !ETRI_ADAPTIVE_QP_SELECTION_OPTIMIZATION
#if ADAPTIVE_QP_SELECTION
			if( m_bUseAdaptQpSelect )
			{
			//  piArlDstCoeff[uiBlkPos]   = (Int)(( lLevelDouble + iAddC) >> iQBitsC );		by serdong
			}
#endif
#endif		
#if ETRI_RDOQ_SIMD_OPTIMIZATION
			UInt uiMaxAbsLevel = euiMaxAbsLevel[uiBlkPos];
#else
			UInt uiMaxAbsLevel = (lLevelDouble + (1 << (iQBits - 1))) >> iQBits;
#endif
			Double dErr               = Double( lLevelDouble );
			pdCostCoeff0[ iScanPos ]  = dErr * dErr * dTemp;
			d64BlockUncodedCost      += pdCostCoeff0[ iScanPos ];
			piDstCoeff[ uiBlkPos ]    = uiMaxAbsLevel;

			if ( uiMaxAbsLevel > 0 && iLastScanPos < 0 )
			{
				iLastScanPos 		= iScanPos;
				uiCtxSet  			= (iScanPos < SCAN_SET_SIZE || eTType!=TEXT_LUMA) ? 0 : 2;
				iCGLastScanPos 	= iCGScanPos;
			}

			if ( iLastScanPos >= 0 )
			{
				//===== coefficient level estimation =====
				UInt  uiLevel;
				UInt  uiOneCtx         = 4 * uiCtxSet + c1;
				UInt  uiAbsCtx         = uiCtxSet + c2;

				if( iScanPos == iLastScanPos )
				{
					uiLevel = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ], 
				            	lLevelDouble, uiMaxAbsLevel, 0, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, iQBits, dTemp, 1 );
				}
				else
				{
#if ETRI_GETSIGCTXINC_OPTIMIZATION
					UShort uiCtxSig = getSigCtxInc(patternSigCtx, uiScanIdx, uiBlkPos, uiLog2BlkSize, eTType);
#else
					UInt   uiPosY        = uiBlkPos >> uiLog2BlkSize;
					UInt   uiPosX        = uiBlkPos - ( uiPosY << uiLog2BlkSize );
					UShort uiCtxSig      = getSigCtxInc( patternSigCtx, uiScanIdx, uiPosX, uiPosY, uiLog2BlkSize, eTType );
#endif
					uiLevel  = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
					            lLevelDouble, uiMaxAbsLevel, uiCtxSig, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, iQBits, dTemp, 0 );
					sigRateDelta[ uiBlkPos ] = m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 1 ] - m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 0 ];
				}
				deltaU[ uiBlkPos ] = (lLevelDouble - ((Int)uiLevel << iQBits)) >> (iQBits-8);
				if( uiLevel > 0 )
				{
					Int rateNow = xGetICRate( uiLevel, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx );
					rateIncUp   [ uiBlkPos ] = xGetICRate( uiLevel+1, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx ) - rateNow;
					rateIncDown [ uiBlkPos ] = xGetICRate( uiLevel-1, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx ) - rateNow;
				}
				else // uiLevel == 0
				{
					rateIncUp   [ uiBlkPos ] = m_pcEstBitsSbac->m_greaterOneBits[ uiOneCtx ][ 0 ];
				}
				piDstCoeff[ uiBlkPos ] = uiLevel;
				d64BaseCost           += pdCostCoeff [ iScanPos ];
				baseLevel = (c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;

#if ETRI_RDOQ_CODE_OPTIMIZATION
				if( uiLevel >= baseLevel )
				{
					uiGoRiceParam+= (uiGoRiceParam < 4 && uiLevel > (3 << uiGoRiceParam));
				}

				c1Idx -= (-(int32_t)uiLevel) >> 31;

				//===== update bin model =====
				if( uiLevel > 1 )
				{
					c1 = 0; 
					c2 += (c2 < 2);
					c2Idx ++;
				}
				else if( (c1 < 3) && (c1 > 0) && uiLevel)
				{
					c1++;
				}
#else  
				if (uiLevel >= baseLevel)
				{
					if (uiLevel  > 3 * (1 << uiGoRiceParam))
					uiGoRiceParam = min<UInt>(uiGoRiceParam + 1, 4);
				}
				if (uiLevel >= 1)
				{
					c1Idx++;
				}

				//===== update bin model =====
				if (uiLevel > 1)
				{
					c1 = 0;
					c2 += (c2 < 2);
					c2Idx++;
				}
				else if ((c1 < 3) && (c1 > 0) && uiLevel)
				{
					c1++;
				}
#endif

				//===== context set update =====
#if ETRI_EM_OPERATION_OPTIMIZATION
				if (((iScanPos & 0x0F) == 0) && (iScanPos > 0))
#else
				if( ( iScanPos % SCAN_SET_SIZE == 0 ) && ( iScanPos > 0 ) )
#endif
				{
					c2                = 0;
					uiGoRiceParam     = 0;

					c1Idx   = 0;
					c2Idx   = 0; 
					uiCtxSet          = (iScanPos == SCAN_SET_SIZE || eTType!=TEXT_LUMA) ? 0 : 2;
#if ETRI_RDOQ_CODE_OPTIMIZATION
					uiCtxSet -= ((int32_t)(c1 - 1) >> 31);
#else
					if( c1 == 0 )
					{
						uiCtxSet++;
					}
#endif
					c1 = 1;
				}
			}
			else
			{
				d64BaseCost    += pdCostCoeff0[ iScanPos ];
			}
			rdStats.d64SigCost += pdCostSig[ iScanPos ];
			if (iScanPosinCG == 0 )
			{
				rdStats.d64SigCost_0 = pdCostSig[ iScanPos ];
			}
			if (piDstCoeff[ uiBlkPos ] )
			{
				uiSigCoeffGroupFlag[ uiCGBlkPos ] = 1;
				rdStats.d64CodedLevelandDist += pdCostCoeff[ iScanPos ] - pdCostSig[ iScanPos ];
				rdStats.d64UncodedDist += pdCostCoeff0[ iScanPos ];
				if ( iScanPosinCG != 0 )
				{
					rdStats.iNNZbeforePos0++;
				}
			}
		} //end for (iScanPosinCG)


    
	if (iCGLastScanPos >= 0) 
	{
		if( iCGScanPos )
		{
			if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
			{
				UInt  uiCtxSig = getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, uiWidth, uiHeight);
				d64BaseCost += xGetRateSigCoeffGroup(0, uiCtxSig) - rdStats.d64SigCost;;  
				pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(0, uiCtxSig);  
			} 
			else
			{
				if (iCGScanPos < iCGLastScanPos) //skip the last coefficient group, which will be handled together with last position below.
				{
					if ( rdStats.iNNZbeforePos0 == 0 ) 
					{
						d64BaseCost -= rdStats.d64SigCost_0;
						rdStats.d64SigCost -= rdStats.d64SigCost_0;
					}
					// rd-cost if SigCoeffGroupFlag = 0, initialization
					Double d64CostZeroCG = d64BaseCost;

					// add SigCoeffGroupFlag cost to total cost
					UInt  uiCtxSig = getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, uiWidth, uiHeight);

					d64BaseCost  += xGetRateSigCoeffGroup(1, uiCtxSig); 
					d64CostZeroCG += xGetRateSigCoeffGroup(0, uiCtxSig);  
					pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(1, uiCtxSig); 


					// try to convert the current coeff group from non-zero to all-zero
					d64CostZeroCG += rdStats.d64UncodedDist;  // distortion for resetting non-zero levels to zero levels
					d64CostZeroCG -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
					d64CostZeroCG -= rdStats.d64SigCost;     // sig cost for all coeffs, including zero levels and non-zerl levels

					// if we can save cost, change this block to all-zero block
					if ( d64CostZeroCG < d64BaseCost )      
					{
						uiSigCoeffGroupFlag[ uiCGBlkPos ] = 0;
						d64BaseCost = d64CostZeroCG;
						pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(0, uiCtxSig); 

						// reset coeffs to 0 in this block                
						for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)
						{
							iScanPos      = iCGScanPos*uiCGSize + iScanPosinCG;
							UInt uiBlkPos = scan[ iScanPos ];

							if (piDstCoeff[ uiBlkPos ])
							{
								piDstCoeff [ uiBlkPos ] = 0;
								pdCostCoeff[ iScanPos ] = pdCostCoeff0[ iScanPos ];
								pdCostSig  [ iScanPos ] = 0;
							}
						}
					} // end if ( d64CostAllZeros < d64BaseCost )      
				}
			} // end if if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
		}
		else
		{
			uiSigCoeffGroupFlag[ uiCGBlkPos ] = 1;
		}
	}

  } //end for (iCGScanPos)
  
	//===== estimate last position =====
	if ( iLastScanPos < 0 )
	{
		return;
	}

	Double  d64BestCost         = 0;
	Int     ui16CtxCbf          = 0;
	Int     iBestLastIdxP1      = 0;
	if( !pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && pcCU->getTransformIdx( uiAbsPartIdx ) == 0 )
	{
		ui16CtxCbf   = 0;
		d64BestCost  = d64BlockUncodedCost + xGetICost( m_pcEstBitsSbac->blockRootCbpBits[ ui16CtxCbf ][ 0 ] );
		d64BaseCost += xGetICost( m_pcEstBitsSbac->blockRootCbpBits[ ui16CtxCbf ][ 1 ] );
	}
	else
	{
		ui16CtxCbf   = pcCU->getCtxQtCbf( eTType, pcCU->getTransformIdx( uiAbsPartIdx ) );
		ui16CtxCbf   = ( eTType ? TEXT_CHROMA : eTType ) * NUM_QT_CBF_CTX + ui16CtxCbf;
		d64BestCost  = d64BlockUncodedCost + xGetICost( m_pcEstBitsSbac->blockCbpBits[ ui16CtxCbf ][ 0 ] );
		d64BaseCost += xGetICost( m_pcEstBitsSbac->blockCbpBits[ ui16CtxCbf ][ 1 ] );
	}

	Bool bFoundLast = false;
	for (Int iCGScanPos = iCGLastScanPos; iCGScanPos >= 0; iCGScanPos--)
	{
		UInt uiCGBlkPos = scanCG[ iCGScanPos ];

		d64BaseCost -= pdCostCoeffGroupSig [ iCGScanPos ]; 
		if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
		{     
			for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)
			{
				iScanPos = iCGScanPos*uiCGSize + iScanPosinCG;
				if (iScanPos > iLastScanPos) continue;
				UInt   uiBlkPos     = scan[iScanPos];

				if( piDstCoeff[ uiBlkPos ] )
				{
					UInt   uiPosY       = uiBlkPos >> uiLog2BlkSize;
					UInt   uiPosX       = uiBlkPos - ( uiPosY << uiLog2BlkSize );

					Double d64CostLast= uiScanIdx == SCAN_VER ? xGetRateLast( uiPosY, uiPosX ) : xGetRateLast( uiPosX, uiPosY );
					Double totalCost = d64BaseCost + d64CostLast - pdCostSig[ iScanPos ];

					if( totalCost < d64BestCost )
					{
						iBestLastIdxP1  = iScanPos + 1;
						d64BestCost     = totalCost;
					}
					if( piDstCoeff[ uiBlkPos ] > 1 )
					{
						bFoundLast = true;
						break;
					}
					d64BaseCost      -= pdCostCoeff[ iScanPos ];
					d64BaseCost      += pdCostCoeff0[ iScanPos ];
				}
				else
				{
					d64BaseCost      -= pdCostSig[ iScanPos ];
				}
			} //end for 
			if (bFoundLast)
			{
				break;
			}
		} // end if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
	} // end for 

	for ( Int scanPos = 0; scanPos < iBestLastIdxP1; scanPos++ )
	{
		Int blkPos = scan[ scanPos ];
		Int level  = piDstCoeff[ blkPos ];
		uiAbsSum += level;
		piDstCoeff[ blkPos ] = ( plSrcCoeff[ blkPos ] < 0 ) ? -level : level;
	}

	//===== clean uncoded coefficients =====
	for ( Int scanPos = iBestLastIdxP1; scanPos <= iLastScanPos; scanPos++ )
	{
		piDstCoeff[ scan[ scanPos ] ] = 0;
	}

	//===== RD based Sign Hiding =====
	if( pcCU->getSlice()->getPPS()->getSignHideFlag() && uiAbsSum>=2)
	{
		Int64 rdFactor = (Int64) (
		g_invQuantScales[m_cQP.rem()] * g_invQuantScales[m_cQP.rem()] * (1<<(2*m_cQP.m_iPer))
		/ m_dLambda / 16 / (1<<DISTORTION_PRECISION_ADJUSTMENT(2*(uiBitDepth-8)))
		+ 0.5);

		Int lastCG = 1;
		Int absSum = 0 ;
		Int n ;

		for( Int subSet = (uiWidth*uiHeight-1) >> LOG2_SCAN_SET_SIZE; subSet >= 0; subSet-- )
		{
			Int  subPos     = subSet << LOG2_SCAN_SET_SIZE;
			Int  firstNZPosInCG=SCAN_SET_SIZE , lastNZPosInCG=-1 ;
			absSum = 0 ;

			for(n = SCAN_SET_SIZE-1; n >= 0; --n )
				if( piDstCoeff[ scan[ n + subPos ]] ){break;}
			if (n < 0)	{continue;}				
			lastNZPosInCG = n;

			for(n = 0; n <SCAN_SET_SIZE; n++ )
				if( piDstCoeff[ scan[ n + subPos ]] ){break;}
			firstNZPosInCG = n;

			if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
			{
				for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
					absSum += piDstCoeff[ scan[ n + subPos ]];

				UInt signbit = (piDstCoeff[scan[subPos+firstNZPosInCG]]>0?0:1);
				if( signbit!=(absSum&0x1) )  // hide but need tune
				{
					// calculate the cost 
					Int64 minCostInc = MAX_INT64, curCost=MAX_INT64;
					Int minPos =-1, finalChange=0, curChange=0;

					for( n = (lastCG==1?lastNZPosInCG:SCAN_SET_SIZE-1) ; n >= 0; --n )
					{
						UInt uiBlkPos   = scan[ n + subPos ];
						if(piDstCoeff[ uiBlkPos ] != 0 )
						{
							Bool bDstCoeffisOne = abs(piDstCoeff[uiBlkPos])==1;
							Int64 costUp   = rdFactor * ( - deltaU[uiBlkPos] ) + rateIncUp[uiBlkPos] ;
							Int64 costDown = rdFactor * (   deltaU[uiBlkPos] ) + rateIncDown[uiBlkPos] - (bDstCoeffisOne ? sigRateDelta[uiBlkPos] : 0);

							if(lastCG==1 && lastNZPosInCG==n && bDstCoeffisOne){costDown -= 131072;}  //131072 =(4<<15) 

							if(costUp<costDown)
							{  
								curCost = costUp;
								curChange =  1 ;
							}
							else               
							{
								curChange = -1 ;
								curCost = (n==firstNZPosInCG && bDstCoeffisOne)? MAX_INT64:costDown;
							}
						}
						else
						{
							curCost = rdFactor * ( - (abs(deltaU[uiBlkPos])) ) + (1<<15) + rateIncUp[uiBlkPos] + sigRateDelta[uiBlkPos] ; 
							curChange = 1 ;

							if(n<firstNZPosInCG)
							{
								UInt thissignbit = (plSrcCoeff[uiBlkPos]>=0?0:1);
								if(thissignbit != signbit )	{curCost = MAX_INT64;}
							}
						}

						if( curCost<minCostInc)
						{
							minCostInc = curCost ;
							finalChange = curChange ;
							minPos = uiBlkPos ;
						}
					}

					/* don't allow sign hiding to violate the SPEC range */
					if(piDstCoeff[minPos] == 32767 || piDstCoeff[minPos] == -32768){	finalChange = -1;}

					/* Update piDstCoeff at [minPos] */
					piDstCoeff[minPos] += (plSrcCoeff[minPos]>=0)? finalChange : -finalChange;


				}
			}

			lastCG=0 ;  

		}
	}

}

/** Pattern decision for context derivation process of significant_coeff_flag
 * \param sigCoeffGroupFlag pointer to prior coded significant coeff group
 * \param posXCG column of current coefficient group
 * \param posYCG row of current coefficient group
 * \param width width of the block
 * \param height height of the block
 * \returns pattern for current coefficient group
 */
Int  TComTrQuant::calcPatternSigCtx( const UInt* sigCoeffGroupFlag, UInt posXCG, UInt posYCG, Int width, Int height )
{
  if( width == 4 && height == 4 ) return -1;

  UInt sigRight = 0;
  UInt sigLower = 0;

  width >>= 2;
  height >>= 2;
  if( posXCG < width - 1 )
  {
    sigRight = (sigCoeffGroupFlag[ posYCG * width + posXCG + 1 ] != 0);
  }
  if (posYCG < height - 1 )
  {
    sigLower = (sigCoeffGroupFlag[ (posYCG  + 1 ) * width + posXCG ] != 0);
  }
  return sigRight + (sigLower<<1);
}

/** Context derivation process of coeff_abs_significant_flag
 * \param patternSigCtx pattern for current coefficient group
 * \param posX column of current scan position
 * \param posY row of current scan position
 * \param log2BlockSize log2 value of block size (square block)
 * \param width width of the block
 * \param height height of the block
 * \param textureType texture type (TEXT_LUMA...)
 * \returns ctxInc for current scan position
 */
#if ETRI_GETSIGCTXINC_OPTIMIZATION
Int TComTrQuant::getSigCtxInc(
	Int                             patternSigCtx,
	UInt                            scanIdx,
	UInt                            blkPos,
	Int                             log2BlockSize,
	TextType                        textureType
	)
{
	if (blkPos == 0) return 0;

	static const Int ctxIndMap[16] =
	{
		0, 1, 4, 5,
		2, 3, 4, 5,
		6, 6, 8, 8,
		7, 7, 8, 8
	};

	if (log2BlockSize == 2) return ctxIndMap[blkPos];

	const UInt posY = blkPos >> log2BlockSize;
	const UInt posX = blkPos - (posY << log2BlockSize);

	static uint8_t table_cnt[4][4][4] =
	{
		{
			{ 2, 1, 1, 0 },
			{ 1, 1, 0, 0 },
			{ 1, 0, 0, 0 },
			{ 0, 0, 0, 0 },
		},
		{
			{ 2, 1, 0, 0 },
			{ 2, 1, 0, 0 },
			{ 2, 1, 0, 0 },
			{ 2, 1, 0, 0 },
		},
		{
			{ 2, 2, 2, 2 },
			{ 1, 1, 1, 1 },
			{ 0, 0, 0, 0 },
			{ 0, 0, 0, 0 },
		},
		{
			{ 2, 2, 2, 2 },
			{ 2, 2, 2, 2 },
			{ 2, 2, 2, 2 },
			{ 2, 2, 2, 2 },
		}
	};

	UInt cnt = table_cnt[patternSigCtx][posX & 3][posY & 3];
	UInt offset = log2BlockSize == 3 ? (scanIdx == SCAN_DIAG ? 9 : 15) : (textureType == TEXT_LUMA ? 21 : 12);

	offset += cnt;

	return (textureType == TEXT_LUMA && (posX | posY) >= 4) ? 3 + offset : offset;
}
#else
Int TComTrQuant::getSigCtxInc    (
                                   Int                             patternSigCtx,
                                   UInt                            scanIdx,
                                   Int                             posX,
                                   Int                             posY,
                                   Int                             log2BlockSize,
                                   TextType                        textureType
                                  )
{
  const Int ctxIndMap[16] =
  {
    0, 1, 4, 5,
    2, 3, 4, 5,
    6, 6, 8, 8,
    7, 7, 8, 8
  };

  if( posX + posY == 0 )
  {
    return 0;
  }

  if ( log2BlockSize == 2 )
  {
    return ctxIndMap[ 4 * posY + posX ];
  }

  Int offset = log2BlockSize == 3 ? (scanIdx==SCAN_DIAG ? 9 : 15) : (textureType == TEXT_LUMA ? 21 : 12);

  Int posXinSubset = posX-((posX>>2)<<2);
  Int posYinSubset = posY-((posY>>2)<<2);
  Int cnt = 0;
  if(patternSigCtx==0)
  {
    cnt = posXinSubset+posYinSubset<=2 ? (posXinSubset+posYinSubset==0 ? 2 : 1) : 0;
  }
  else if(patternSigCtx==1)
  {
    cnt = posYinSubset<=1 ? (posYinSubset==0 ? 2 : 1) : 0;
  }
  else if(patternSigCtx==2)
  {
    cnt = posXinSubset<=1 ? (posXinSubset==0 ? 2 : 1) : 0;
  }
  else
  {
    cnt = 2;
  }

  return (( textureType == TEXT_LUMA && ((posX>>2) + (posY>>2)) > 0 ) ? 3 : 0) + offset + cnt;
}
#endif

/** Get the best level in RD sense
 * \param rd64CodedCost reference to coded cost
 * \param rd64CodedCost0 reference to cost when coefficient is 0
 * \param rd64CodedCostSig reference to cost of significant coefficient
 * \param lLevelDouble reference to unscaled quantized level
 * \param uiMaxAbsLevel scaled quantized level
 * \param ui16CtxNumSig current ctxInc for coeff_abs_significant_flag
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice current Rice parameter for coeff_abs_level_minus3
 * \param iQBits quantization step size
 * \param dTemp correction factor
 * \param bLast indicates if the coefficient is the last significant
 * \returns best quantized transform level for given scan position
 * This method calculates the best quantized transform level for a given scan position.
 */
 #if ETRI_FAST_xGetICRateFUNCTIONS
 __inline UInt TComTrQuant::xGetCodedLevel ( Double&						 rd64CodedCost,
											 Double&						 rd64CodedCost0,
											 Double&						 rd64CodedCostSig,
											 Int							 lLevelDouble,
											 UInt							 uiMaxAbsLevel,
											 UShort 						 ui16CtxNumSig,
											 UShort 						 ui16CtxNumOne,
											 UShort 						 ui16CtxNumAbs,
											 UShort 						 ui16AbsGoRice,
											 UInt							 c1Idx,
											 UInt							 c2Idx,
											 Int							 iQBits,
											 Double 						 dTemp,
											 Bool							 bLast		  ) const
 {
 //  if( uiMaxAbsLevel == 0 ){ return 0; }		 /// Test Code for Optimization of xGetCodedLevel 
 
	 if( !bLast && uiMaxAbsLevel < 3 )
	 {
		 rd64CodedCostSig	 = m_pcEstBitsSbac->significantBits[ ui16CtxNumSig ][0] * m_dLambda; /// rd64CodedCostSig	 = xGetRateSigCoef( 0, ui16CtxNumSig ); 
		 rd64CodedCost		 = rd64CodedCost0 + rd64CodedCostSig;
 
		 if( uiMaxAbsLevel == 0 ){ return 0; }		 /// return Value : uiBestAbsLevel = 0;
	 }
	 else
	 {
		 rd64CodedCost		 = MAX_DOUBLE;
	 }
 
	 UInt	 uiBestAbsLevel 	 = 0;
	 Double  dCurrCostSig	 = (!bLast)? (m_pcEstBitsSbac->significantBits[ ui16CtxNumSig ][1] * m_dLambda) : 0; 
 
	 Int uiAbsLevel  = uiMaxAbsLevel; 
 
	 /*
		 lLevelDouble 자체의 값도 10^6 ~ 10^7 수준이며 ( uiAbsLevel << iQBits ) 는 2^22 = 4 * 10^6 수준. Int 32 bit 는 4 * 10^9 
	 */
 
	 Double dErr		 = Double( lLevelDouble  - ( uiAbsLevel << iQBits ) );
	 Double dCurrCost	 = dErr * dErr * dTemp + xGetICRateCost( uiAbsLevel, ui16CtxNumOne, ui16CtxNumAbs, ui16AbsGoRice, c1Idx, c2Idx );
	 dCurrCost		 += dCurrCostSig;
 
	 if( dCurrCost < rd64CodedCost )
	 {
		 uiBestAbsLevel    = uiAbsLevel;
		 rd64CodedCost	   = dCurrCost;
		 rd64CodedCostSig  = dCurrCostSig;
	 }
 
	 if (uiMaxAbsLevel	==	1){return uiBestAbsLevel;}
 
	 uiAbsLevel--;		 /// 2015 5 28 by Seok : this value is 1 or 0
	 
	 dErr			 = Double( lLevelDouble  - ( uiAbsLevel << iQBits ) );
	 dCurrCost	 = dErr * dErr * dTemp + xGetICRateCost( uiAbsLevel, ui16CtxNumOne, ui16CtxNumAbs, ui16AbsGoRice, c1Idx, c2Idx );
	 dCurrCost	 += dCurrCostSig;
 
	 if( dCurrCost < rd64CodedCost )
	 {
		 uiBestAbsLevel    = uiAbsLevel;
		 rd64CodedCost	   = dCurrCost;
		 rd64CodedCostSig  = dCurrCostSig;
	 }
 
	 return uiBestAbsLevel;
 }
 
 /** Calculates the cost for specific absolute transform level
  * \param uiAbsLevel scaled quantized level
  * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
  * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
  * \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
  * \returns cost of given absolute transform level
  */

 /**
 ----------------------------------------------------------------------------------------------------------------------
	This function is used only for TComTrQuant::xGetCodedLevel
 ----------------------------------------------------------------------------------------------------------------------
 */
 __inline Double TComTrQuant::xGetICRateCost  ( UInt							uiAbsLevel,
												UShort							ui16CtxNumOne,
												UShort							ui16CtxNumAbs,
												UShort							ui16AbsGoRice
											 ,	UInt							c1Idx,
												UInt							c2Idx
												) const
 {
#if ETRI_DOUBLE_CONVERSION
	 int iRate =32768;
#else
	 Double iRate = xGetIEPRate();
#endif
	 UInt baseLevel  =	(c1Idx < C1FLAG_NUMBER)? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;
 
	 if ( uiAbsLevel >= baseLevel )
	 {	  
		 UInt symbol	 = uiAbsLevel - baseLevel;
		 UInt length	 = (symbol>>ui16AbsGoRice);
		 
		 if ( length <	COEF_REMAIN_BIN_REDUCTION)
		 {
			 iRate += (length+1+ui16AbsGoRice)<< 15;
		 }
		 else
		 {
			 length = 0;
			 symbol  = (symbol>>ui16AbsGoRice) - COEF_REMAIN_BIN_REDUCTION;
			 if (symbol)
			 {
#if _ETRI_WINDOWS_APPLICATION
				unsigned long ulidx; ETRI_CLZ(ulidx, symbol + 1);
				length = ulidx;
#else
				length = __builtin_clz(symbol + 1);
#endif
			 }
			 iRate += (COEF_REMAIN_BIN_REDUCTION + length + 1 + ui16AbsGoRice + length)<< 15;
		 }
 
		 if (c1Idx < C1FLAG_NUMBER)
		 {
			 iRate += (m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ] + ((c2Idx < C2FLAG_NUMBER)? m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 1 ] : 0));
		 }
 
	 }
	 else
	 {
		 iRate += ((m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ uiAbsLevel - 1 ] + ((1 -uiAbsLevel) &  m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 0 ])));
	 }
 
	 return ( iRate * m_dLambda);
  //  return xGetICost( iRate );
 }
 
 
 __inline Int TComTrQuant::xGetICRate  ( UInt							 uiAbsLevel,
										UShort							ui16CtxNumOne,
										UShort							ui16CtxNumAbs,
										UShort							ui16AbsGoRice
									  , UInt							c1Idx,
										UInt							c2Idx
										) const
 {
	// uiAbsLevel == 0 이면 iRate = 0 이므로 바로 끝내도 된다.
	if (uiAbsLevel == 0){ return 0; }

#if ETRI_DOUBLE_CONVERSION
	Int iRate = 32768;
#else
	Int iRate = Int(xGetIEPRate());
#endif

	UInt baseLevel  =  (c1Idx < C1FLAG_NUMBER)? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;

	if ( uiAbsLevel >= baseLevel )
	{    
		UInt symbol     = uiAbsLevel - baseLevel;
		UInt length;
		if (symbol < (COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice))
		{
			length = symbol>>ui16AbsGoRice;
			iRate += (length+1+ui16AbsGoRice)<< 15;
		}
		else
		{
			length = ui16AbsGoRice;
			symbol  = symbol - ( COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice);

			///아래 while문을 intrinsic으로 변경한 버전 @ 2016 1 31 by Seok	
			if (symbol > 0)
			{
#if _ETRI_WINDOWS_APPLICATION
				unsigned long	e_uiIndex; ETRI_CLZ(e_uiIndex, (symbol + (1<<length)));
				length = e_uiIndex;
#else
				length = __builtin_clz(symbol + (1<<length));
#endif
			}
			iRate += (COEF_REMAIN_BIN_REDUCTION+length+1-ui16AbsGoRice+length)<< 15;
		}

		if (c1Idx < C1FLAG_NUMBER)
		{
			iRate += (m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ] + ((c2Idx < C2FLAG_NUMBER)? m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 1 ] : 0));
		}
	}
	else
	{
		// uiAbsLevel > 3 인 경우는 없다.
		iRate += ((m_pcEstBitsSbac->m_greaterOneBits[ui16CtxNumOne][uiAbsLevel - 1] + ((1 - uiAbsLevel) &  m_pcEstBitsSbac->m_levelAbsBits[ui16CtxNumAbs][0])));
	}

  return iRate;
 }

 #else
__inline UInt TComTrQuant::xGetCodedLevel ( Double&                         rd64CodedCost,
                                            Double&                         rd64CodedCost0,
                                            Double&                         rd64CodedCostSig,
                                            Int                             lLevelDouble,
                                            UInt                            uiMaxAbsLevel,
                                            UShort                          ui16CtxNumSig,
                                            UShort                          ui16CtxNumOne,
                                            UShort                          ui16CtxNumAbs,
                                            UShort                          ui16AbsGoRice,
                                            UInt                            c1Idx,
                                            UInt                            c2Idx,
                                            Int                             iQBits,
                                            Double                          dTemp,
                                            Bool                            bLast        ) const
{
  Double dCurrCostSig   = 0; 
  UInt   uiBestAbsLevel = 0;
  
  if( !bLast && uiMaxAbsLevel < 3 )
  {
    rd64CodedCostSig    = xGetRateSigCoef( 0, ui16CtxNumSig ); 
    rd64CodedCost       = rd64CodedCost0 + rd64CodedCostSig;
    if( uiMaxAbsLevel == 0 )
    {
      return uiBestAbsLevel;
    }
  }
  else
  {
    rd64CodedCost       = MAX_DOUBLE;
  }

  if( !bLast )
  {
    dCurrCostSig        = xGetRateSigCoef( 1, ui16CtxNumSig );
  }

  UInt uiMinAbsLevel    = ( uiMaxAbsLevel > 1 ? uiMaxAbsLevel - 1 : 1 );
  for( Int uiAbsLevel  = uiMaxAbsLevel; uiAbsLevel >= uiMinAbsLevel ; uiAbsLevel-- )
  {
    Double dErr         = Double( lLevelDouble  - ( uiAbsLevel << iQBits ) );
    Double dCurrCost    = dErr * dErr * dTemp + xGetICost(xGetICRate( uiAbsLevel, ui16CtxNumOne, ui16CtxNumAbs, ui16AbsGoRice, c1Idx, c2Idx ));
    dCurrCost          += dCurrCostSig;

    if( dCurrCost < rd64CodedCost )
    {
      uiBestAbsLevel    = uiAbsLevel;
      rd64CodedCost     = dCurrCost;
      rd64CodedCostSig  = dCurrCostSig;
    }
  }

  return uiBestAbsLevel;
}

/** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
 * \returns cost of given absolute transform level
 */
__inline Int TComTrQuant::xGetICRate  ( UInt                            uiAbsLevel,
                                        UShort                          ui16CtxNumOne,
                                        UShort                          ui16CtxNumAbs,
                                        UShort                          ui16AbsGoRice
                                     ,  UInt                            c1Idx,
                                        UInt                            c2Idx
                                        ) const
{
  Int iRate = Int(xGetIEPRate());
  UInt baseLevel  =  (c1Idx < C1FLAG_NUMBER)? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;

  if ( uiAbsLevel >= baseLevel )
  {    
    UInt symbol     = uiAbsLevel - baseLevel;
    UInt length;
    if (symbol < (COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice))
    {
      length = symbol>>ui16AbsGoRice;
      iRate += (length+1+ui16AbsGoRice)<< 15;
    }
    else
    {
      length = ui16AbsGoRice;
      symbol  = symbol - ( COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice);
      while (symbol >= (1<<length))
      {
        symbol -=  (1<<(length++));    
      }
      iRate += (COEF_REMAIN_BIN_REDUCTION+length+1-ui16AbsGoRice+length)<< 15;
    }
    if (c1Idx < C1FLAG_NUMBER)
    {
      iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];

      if (c2Idx < C2FLAG_NUMBER)
      {
        iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 1 ];
      }
    }
  }
  else
  if( uiAbsLevel == 1 )
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 0 ];
  }
  else if( uiAbsLevel == 2 )
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];
    iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 0 ];
  }
  else
  {
    iRate = 0;
  }
  return iRate;
}


#endif	/// ENd of  #if ETRI_FAST_xGetICRateFUNCTIONS @ 2016 1 30 by Seok 

__inline Double TComTrQuant::xGetRateSigCoeffGroup  ( UShort                    uiSignificanceCoeffGroup,
                                                UShort                          ui16CtxNumSig ) const
{
  return xGetICost( m_pcEstBitsSbac->significantCoeffGroupBits[ ui16CtxNumSig ][ uiSignificanceCoeffGroup ] );
}

/** Calculates the cost of signaling the last significant coefficient in the block
 * \param uiPosX X coordinate of the last significant coefficient
 * \param uiPosY Y coordinate of the last significant coefficient
 * \returns cost of last significant coefficient
 */
/*
 * \param uiWidth width of the transform unit (TU)
*/
__inline Double TComTrQuant::xGetRateLast   ( const UInt                      uiPosX,
                                              const UInt                      uiPosY ) const
{
	UInt uiCtxX   = g_uiGroupIdx[uiPosX];
	UInt uiCtxY   = g_uiGroupIdx[uiPosY];

#if ETRI_DOUBLE_CONVERSION
	int uiCost = m_pcEstBitsSbac->lastXBits[ uiCtxX ] + m_pcEstBitsSbac->lastYBits[ uiCtxY ];
	if( uiCtxX > 3 )
	uiCost += ((uiCtxX-2)>>1) << 15;

	if( uiCtxY > 3 )
	uiCost += ((uiCtxY-2)>>1) << 15;
#else
	Double uiCost = m_pcEstBitsSbac->lastXBits[ uiCtxX ] + m_pcEstBitsSbac->lastYBits[ uiCtxY ];
	if( uiCtxX > 3 )
	{
		uiCost += xGetIEPRate() * ((uiCtxX-2)>>1);
	}
	if( uiCtxY > 3 )
	{
		uiCost += xGetIEPRate() * ((uiCtxY-2)>>1);
	}
#endif

	return xGetICost( uiCost );
}

 /** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxBase current global offset for coeff_abs_level_greater1 and coeff_abs_level_greater2
 * \returns cost of given absolute transform level
 */
__inline Double TComTrQuant::xGetRateSigCoef  ( UShort                          uiSignificance,
                                                UShort                          ui16CtxNumSig ) const
{
  return xGetICost( m_pcEstBitsSbac->significantBits[ ui16CtxNumSig ][ uiSignificance ] );
}

/** Get the cost for a specific rate
 * \param dRate rate of a bit
 * \returns cost at the specific rate
 */
__inline Double TComTrQuant::xGetICost        ( Double                          dRate         ) const
{
  return m_dLambda * dRate;
}

/** Get the cost of an equal probable bit
 * \returns cost of equal probable bit
 */
__inline Double TComTrQuant::xGetIEPRate      (                                               ) const
{
  return 32768;
}

/** Context derivation process of coeff_abs_significant_flag
 * \param uiSigCoeffGroupFlag significance map of L1
 * \param uiBlkX column of current scan position
 * \param uiBlkY row of current scan position
 * \param uiLog2BlkSize log2 value of block size
 * \returns ctxInc for current scan position
 */
UInt TComTrQuant::getSigCoeffGroupCtxInc  ( const UInt*               uiSigCoeffGroupFlag,
                                           const UInt                      uiCGPosX,
                                           const UInt                      uiCGPosY,
                                           Int width, Int height)
{
  UInt uiRight = 0;
  UInt uiLower = 0;

  width >>= 2;
  height >>= 2;
  if( uiCGPosX < width - 1 )
  {
    uiRight = (uiSigCoeffGroupFlag[ uiCGPosY * width + uiCGPosX + 1 ] != 0);
  }
  if (uiCGPosY < height - 1 )
  {
    uiLower = (uiSigCoeffGroupFlag[ (uiCGPosY  + 1 ) * width + uiCGPosX ] != 0);
  }
  return (uiRight || uiLower);

}
/** set quantized matrix coefficient for encode
 * \param scalingList quantaized matrix address
 */
Void TComTrQuant::setScalingList(TComScalingList *scalingList)
{
  UInt size,list;
  UInt qp;

  for(size=0;size<SCALING_LIST_SIZE_NUM;size++)
  {
    for(list = 0; list < g_scalingListNum[size]; list++)
    {
      for(qp=0;qp<SCALING_LIST_REM_NUM;qp++)
      {
        xSetScalingListEnc(scalingList,list,size,qp);
        xSetScalingListDec(scalingList,list,size,qp);
        setErrScaleCoeff(list,size,qp);
      }
    }
  }
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 */
Void TComTrQuant::setScalingListDec(TComScalingList *scalingList)
{
  UInt size,list;
  UInt qp;

  for(size=0;size<SCALING_LIST_SIZE_NUM;size++)
  {
    for(list = 0; list < g_scalingListNum[size]; list++)
    {
      for(qp=0;qp<SCALING_LIST_REM_NUM;qp++)
      {
        xSetScalingListDec(scalingList,list,size,qp);
      }
    }
  }
}
/** set error scale coefficients
 * \param list List ID
 * \param uiSize Size
 * \param uiQP Quantization parameter
 */
Void TComTrQuant::setErrScaleCoeff(UInt list,UInt size, UInt qp)
{

	UInt uiLog2TrSize = g_aucConvertToBit[ g_scalingListSizeX[size] ] + 2;
	Int bitDepth = (size < SCALING_LIST_32x32 && list != 0 && list != 3) ? g_bitDepthC : g_bitDepthY;
	Int iTransformShift = MAX_TR_DYNAMIC_RANGE - bitDepth - uiLog2TrSize;  // Represents scaling through forward transform

	UInt i,uiMaxNumCoeff = g_scalingListSize[size];
	Int *piQuantcoeff;
	Double *pdErrScale;
	piQuantcoeff   = getQuantCoeff(list, qp,size);
	pdErrScale     = getErrScaleCoeff(list, size, qp);

	Double dErrScale = (Double)(1<<SCALE_BITS);                              // Compensate for scaling of bitcount in Lagrange cost function
	dErrScale = dErrScale*pow(2.0,-2.0*iTransformShift);                     // Compensate for scaling through forward transform


	for(i=0;i<uiMaxNumCoeff;i++)
	{
	pdErrScale[i] = dErrScale / piQuantcoeff[i] / piQuantcoeff[i] / (1<<DISTORTION_PRECISION_ADJUSTMENT(2*(bitDepth-8)));
	}

}

/** set quantized matrix coefficient for encode
 * \param scalingList quantaized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param uiQP Quantization parameter
 */
Void TComTrQuant::xSetScalingListEnc(TComScalingList *scalingList, UInt listId, UInt sizeId, UInt qp)
{
  UInt width = g_scalingListSizeX[sizeId];
  UInt height = g_scalingListSizeX[sizeId];
  UInt ratio = g_scalingListSizeX[sizeId]/min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);
  Int *quantcoeff;
  Int *coeff = scalingList->getScalingListAddress(sizeId,listId);
  quantcoeff   = getQuantCoeff(listId, qp, sizeId);

  processScalingListEnc(coeff,quantcoeff,g_quantScales[qp]<<4,height,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 * \param list List index
 * \param size size index
 * \param uiQP Quantization parameter
 */
Void TComTrQuant::xSetScalingListDec(TComScalingList *scalingList, UInt listId, UInt sizeId, UInt qp)
{
  UInt width = g_scalingListSizeX[sizeId];
  UInt height = g_scalingListSizeX[sizeId];
  UInt ratio = g_scalingListSizeX[sizeId]/min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);
  Int *dequantcoeff;
  Int *coeff = scalingList->getScalingListAddress(sizeId,listId);

  dequantcoeff = getDequantCoeff(listId, qp, sizeId);
  processScalingListDec(coeff,dequantcoeff,g_invQuantScales[qp],height,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));
}

/** set flat matrix value to quantized coefficient
 */
Void TComTrQuant::setFlatScalingList()
{
  UInt size,list;
  UInt qp;

  for(size=0;size<SCALING_LIST_SIZE_NUM;size++)
  {
    for(list = 0; list <  g_scalingListNum[size]; list++)
    {
      for(qp=0;qp<SCALING_LIST_REM_NUM;qp++)
      {
        xsetFlatScalingList(list,size,qp);
        setErrScaleCoeff(list,size,qp);
      }
    }
  }
}

/** set flat matrix value to quantized coefficient
 * \param list List ID
 * \param uiQP Quantization parameter
 * \param uiSize Size
 */
Void TComTrQuant::xsetFlatScalingList(UInt list, UInt size, UInt qp)
{
	UInt i,num = g_scalingListSize[size];
	Int *quantcoeff;
	Int *dequantcoeff;
	Int quantScales = g_quantScales[qp];
	Int invQuantScales = g_invQuantScales[qp]<<4;

	quantcoeff   = getQuantCoeff(list, qp, size);
	dequantcoeff = getDequantCoeff(list, qp, size);

#if ETRI_SIMD_REMAIN_PART
	__m128i xmm0 = _mm_set1_epi32(quantScales);
	__m128i xmm1 = _mm_set1_epi32(invQuantScales);
	for(i=0;i<num;i+=4)
	{ 
		_mm_storeu_si128((__m128i*)quantcoeff,xmm0);
		_mm_storeu_si128((__m128i*)dequantcoeff,xmm1);

		quantcoeff+= 4;
		dequantcoeff+= 4;
	}
#else
	for(i=0;i<num;i++)
	{ 
		*quantcoeff++ = quantScales;
		*dequantcoeff++ = invQuantScales;
	}
#endif

}

/** set quantized matrix coefficient for encode
 * \param coeff quantaized matrix address
 * \param quantcoeff quantaized matrix address
 * \param quantScales Q(QP%6)
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TComTrQuant::processScalingListEnc( Int *coeff, Int *quantcoeff, Int quantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{
  Int nsqth = (height < width) ? 4: 1; //height ratio for NSQT
  Int nsqtw = (width < height) ? 4: 1; //width ratio for NSQT
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
      quantcoeff[j*width + i] = quantScales / coeff[sizuNum * (j * nsqth / ratio) + i * nsqtw /ratio];
    }
  }
  if(ratio > 1)
  {
    quantcoeff[0] = quantScales / dc;
  }
}
/** set quantized matrix coefficient for decode
 * \param coeff quantaized matrix address
 * \param dequantcoeff quantaized matrix address
 * \param invQuantScales IQ(QP%6))
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TComTrQuant::processScalingListDec( Int *coeff, Int *dequantcoeff, Int invQuantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratio) + i / ratio];
    }
  }
  if(ratio > 1)
  {
    dequantcoeff[0] = invQuantScales * dc;
  }
}

/** initialization process of scaling list array
 */
Void TComTrQuant::initScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < g_scalingListNum[sizeId]; listId++)
    {
      for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        m_quantCoef   [sizeId][listId][qp] = new Int [g_scalingListSize[sizeId]];
        m_dequantCoef [sizeId][listId][qp] = new Int [g_scalingListSize[sizeId]];
        m_errScale    [sizeId][listId][qp] = new Double [g_scalingListSize[sizeId]];
      }
    }
  }
  // alias list [1] as [3].
  for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
  {
    m_quantCoef   [SCALING_LIST_32x32][3][qp] = m_quantCoef   [SCALING_LIST_32x32][1][qp];
    m_dequantCoef [SCALING_LIST_32x32][3][qp] = m_dequantCoef [SCALING_LIST_32x32][1][qp];
    m_errScale    [SCALING_LIST_32x32][3][qp] = m_errScale    [SCALING_LIST_32x32][1][qp];
  }
}
/** destroy quantization matrix array
 */
Void TComTrQuant::destroyScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < g_scalingListNum[sizeId]; listId++)
    {
      for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        if(m_quantCoef   [sizeId][listId][qp]) delete [] m_quantCoef   [sizeId][listId][qp];
        if(m_dequantCoef [sizeId][listId][qp]) delete [] m_dequantCoef [sizeId][listId][qp];
        if(m_errScale    [sizeId][listId][qp]) delete [] m_errScale    [sizeId][listId][qp];
      }
    }
  }
}

//! \}
