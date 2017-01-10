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

/** \file     TVideoIOYuv.h
    \brief    YUV file I/O class (header)
*/

#ifndef __TVIDEOIOYUV__
#define __TVIDEOIOYUV__

#include <stdio.h>
#include <fstream>
#include <iostream>
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComPicYuv.h"
#include "TLibCommon/TypeDef.h"
#if (ETRI_DLL_INTERFACE)
#if (_ETRI_WINDOWS_APPLICATION)
#include <Windows.h>
#else
#include <unistd.h>
#include <string.h>
#endif
#endif

using namespace std;

// ====================================================================================================================
// DLL Interface
// ====================================================================================================================

#if (ETRI_DLL_INTERFACE)
#define	ETRI_memcpy	memcpy

/// DLL file I/O class
class ETRI_fstream
{
private:
	unsigned char *e_ptr;		//always *(ptr + ptridx)

	unsigned long	ptridx;
	unsigned long	ptrfinal;
	unsigned long	ptrbegin;

	Int64 	pseudoFilePointer;
	Int64	ptrfinal64;
	Int64	ptrbegin64;

	bool		bEOF;

public:
	ETRI_fstream()		{bEOF = false; e_ptr= NULL;}
	virtual ~ETRI_fstream()	{e_ptr= NULL;}
	
	void	open(unsigned char * ptrbuf, unsigned long _size)	//Instead of File Mode
	{
		if (ptrbuf == nullptr)
			{fprintf(stderr, "ETRI_fstream : open error \n"), ETRI_EXIT(0);}
		else{
			e_ptr	= (unsigned char *)ptrbuf;
			ptrbegin	= (unsigned long)ptrbuf;
			ptrbegin64	= (Int64)ptrbuf; 

			ptridx = 0;
			pseudoFilePointer = 0;

			ptrfinal	= _size;
			ptrfinal64	= _size;
		}
	}

	int 	Ftell	() 	{return ptridx;}
	bool	fail	()	{return false;}
	void	clear	()	{memset(e_ptr, 0, ptrfinal);}
	bool	eof 	()	{return bEOF;}
	void	close	()	
	{
#if (_ETRI_WINDOWS_APPLICATION)
		if ((e_ptr != nullptr) && !IsBadReadPtr(e_ptr, (UINT_PTR)ptrfinal)){
#else
		if (access((const char *)e_ptr, F_OK) == 0) {
#endif
			free(e_ptr); e_ptr=nullptr;
		}

		ptridx=0; ptrfinal=0;ptrbegin=0; pseudoFilePointer=0;ptrfinal64=0;ptrbegin64=0;
	}

	int	seekg(int off, ios_base::seekdir way)
	{	
		if (way == ios_base::beg)
			ptridx = off;
		else if (way == ios_base::cur)
			ptridx = ptridx + off;
		else if (way == ios_base::end)
			ptridx = ptrfinal + off;

		if ((ptridx < ptrbegin) || (ptridx > ptrfinal))
			return 0;
		return 1;
	;}

	void	read (void* s, unsigned int n)		{ETRI_memcpy(s, e_ptr+ptridx, n); ptridx += n; pseudoFilePointer += n;}
	void	write (char* s, unsigned int n)		{ETRI_memcpy(e_ptr+ptridx, s, n); ptridx += n; pseudoFilePointer += n;}
	Void	SetEOF (bool EOFvalue)				{bEOF = EOFvalue;}	

	unsigned char*	getPTR		(int off)	{return (e_ptr + ptridx + off);}
	unsigned char 	getCurByte	(int off)	{return *(e_ptr+ptridx+off);}

};
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// YUV file I/O class
class TVideoIOYuv
{
private:
#if (ETRI_DLL_INTERFACE)
	ETRI_fstream	m_cHandle, *m_pcHandle; 		// 2013 4 1 by Seok : Memory Hnadle For YUV Input/Output (readPlane/writePlane)
	ETRI_fstream	m_cOutput, *m_pcOutput;		// 2013 4 3 by Seok : Memory Hnadle For Output : HEVC Bitstream (writeAnnexB)
#else
	fstream			m_cHandle;                                      ///< file handle
#endif
  
  Int m_fileBitDepthY; ///< bitdepth of input/output video file luma component
  Int m_fileBitDepthC; ///< bitdepth of input/output video file chroma component
  Int m_bitDepthShiftY;  ///< number of bits to increase or decrease luma by before/after write/read
  Int m_bitDepthShiftC;  ///< number of bits to increase or decrease chroma by before/after write/read

  Int em_iETRI_ColorSpaceYV12; ///< Indicate of I420 (=0) or YV12 (=1)
  
public:
  TVideoIOYuv()
  {
#if (ETRI_DLL_INTERFACE)
	m_pcHandle =(ETRI_fstream*)&m_cHandle;	// 2013 4 2 by Seok
	m_pcOutput =(ETRI_fstream*)&m_cOutput;	// 2013 4 2 by Seok
#endif

	em_iETRI_ColorSpaceYV12 = 0;
   }

  virtual ~TVideoIOYuv()  {}
  
  Void  open  ( Char* pchFile, Bool bWriteMode, Int fileBitDepthY, Int fileBitDepthC, Int internalBitDepthY, Int internalBitDepthC ); ///< open or create file
  Void  close ();                                           ///< close file

  void skipFrames(UInt numFrames, UInt width, UInt height);
  
  Bool  read  ( TComPicYuv*   pPicYuv, Int aiPad[2] );     ///< read  one YUV frame with padding parameter
  Bool  write( TComPicYuv*    pPicYuv, Int confLeft=0, Int confRight=0, Int confTop=0, Int confBottom=0 );
  Bool  write( TComPicYuv*    pPicYuv, TComPicYuv*    pPicYuv2, Int confLeft=0, Int confRight=0, Int confTop=0, Int confBottom=0  , bool isTff=false); 
  
  Bool  isEof ();                                           ///< check for end-of-file
  Bool  isFail();                                           ///< check for failure

#if (ETRI_DLL_INTERFACE)
  ETRI_fstream*	GetptrETRIfstream(bool isInput){return (isInput)? m_pcHandle : m_pcOutput;}	
#endif    

  Void ETRI_setYV12Enable(Int iValue){em_iETRI_ColorSpaceYV12= iValue;}	//2015 07 12 by seok
};

#endif // __TVIDEOIOYUV__

