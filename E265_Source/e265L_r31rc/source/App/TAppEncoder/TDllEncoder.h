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
	\file   	TDllEncoder.h
	\brief    	TDllEncoder (header)
*/
#include "TLibCommon/TypeDef.h"

#if (_ETRI_WINDOWS_APPLICATION)
#include <tchar.h>
#endif
#include "DLLInterfaceType.h"

#define ETRI_STATIC_DLL		0

#if ETRI_STATIC_DLL

#ifndef ENCODERDLL_EXPORTS
#define ENCODERDLL_EXPORTS


#if (_ETRI_WINDOWS_APPLICATION)
#ifdef ENCODERDLL_EXPORTS
#define ENCODERDLL_API __declspec(dllexport) 
#else
#define ENCODERDLL_API __declspec(dllimport) 
#endif
#else
#define ENCODERDLL_API __attribute__((__visibility__("default")))
#endif

class EncoderMain
{
public:
	EncoderMain() : hTAppEncTop(NULL) {}
	~EncoderMain() {}

	ENCODERDLL_API void *EncoderConstruct(int argc, char *argv[]);
	ENCODERDLL_API bool EncoderInitilization();
	ENCODERDLL_API bool EncoderMainFunc();
	ENCODERDLL_API void EncoderDestroy();				
	ENCODERDLL_API void EncoderPrintSummary();

	ENCODERDLL_API ETRI_Interface *GetEncInterface();

private:
	void *hTAppEncTop;
};
#endif

#else

#ifndef _TDLLENCODER_H_
#define _TDLLENCODER_H_

#if (_ETRI_WINDOWS_APPLICATION)
#if (defined(TDLLENCODER_EXPORTS) || defined(TDLLMULTIENCODER_EXPORTS))
#define 	DLL_DECL 	__declspec(dllexport)
#else
#define 	DLL_DECL 	__declspec(dllimport)
#endif
#else
#define		DLL_DECL	__attribute__((visibility("default")))
#endif

#endif

extern "C" DLL_DECL void *ETRI_EncoderConstruct		(int argc, char *argv[]);
extern "C" DLL_DECL bool  ETRI_EncoderInitilization	(void *hTAppEncTop);
extern "C" DLL_DECL bool  ETRI_EncoderMainFunc		(void *hTAppEncTop);
extern "C" DLL_DECL void  ETRI_EncoderDestroy		(void **hTAppEncTop);
extern "C" DLL_DECL void  ETRI_printSummary			(void *hTAppEncTop);

extern "C" DLL_DECL ETRI_Interface *ETRI_GetEncInterface(void *hTAppEncTop);
#endif
