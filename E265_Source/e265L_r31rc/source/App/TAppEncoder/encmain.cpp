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

/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>

#include "TAppEncTop.h"
#include "TAppCommon/program_options_lite.h"

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
  TAppEncTop  cTAppEncTop;

#if ETRI_THREADPOOL_OPT
  fprintf(stderr, "ETRI_THREADPOOL_OPT \n");
//#if QURAM_ES_FILE_WRITING
//  fprintf(stderr, "QURAM : QURAM_ES_FILE_WRITING 1 \n");
//#endif
#if ETRI_TILE_THEAD_OPT
  fprintf(stderr, "ETRI_TILE_THEAD_OPT 1 \n");
#endif
#if ETRI_FRAME_THEAD_OPT
  fprintf(stderr, "ETRI_FRAME_THEAD_OPT 1 \n");
#endif
//#if QURAM_THREAD_NUM_MODIFY
//  fprintf(stderr, "QURAM : QURAM_THREAD_NUM_MODIFY 1 \n");
//#endif
#if ETRI_THREAD_LOAD_BALANCING
  fprintf(stderr, "ETRI_THREAD_LOAD_BALANCING 1 \n");
#endif
#if ETRI_COPYTOPIC_MULTITHREAD
  fprintf(stderr, "ETRI_COPYTOPIC_MULTITHREAD 1 \n");
#endif
#else
  fprintf(stderr, "ETRI : QURAM define off \n");
#endif

  // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "HM software: Encoder Version [%s]", NV_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
  fprintf( stdout, "\n" );

  // ETRI_DLL_INTERFACE off check
#if ETRI_DLL_INTERFACE
  printf("\n ERROR: ETRI_DLL_INTERFACE should be off \n");
#endif
  // create application encoder class
  cTAppEncTop.create();

  // parse configuration
  try
  {
    if(!cTAppEncTop.parseCfg( argc, argv ))
    {
      cTAppEncTop.destroy();
      return 1;
    }
  }
  catch (po::ParseFailure& e)
  {
    cerr << "Error parsing option \""<< e.arg <<"\" with argument \""<< e.val <<"\"." << endl;
    return 1;
  }

  // starting time
  double dResult;
#if (_ETRI_WINDOWS_APPLICATION)
  long lBefore = clock();
#else
  timespec lBefore;
  clock_gettime(CLOCK_MONOTONIC, &lBefore); // Works on Linux by yhee 2016.04.19
#endif

  // call encoding function
  cTAppEncTop.encode();

    // ending time
#if (_ETRI_WINDOWS_APPLICATION)
  dResult = (double)(clock() - lBefore) / CLOCKS_PER_SEC;
#else
  timespec iCurrTime;
  clock_gettime(CLOCK_MONOTONIC, &iCurrTime); // Works on Linux by yhee 2016.04.19
  dResult = (iCurrTime.tv_sec - lBefore.tv_sec);
  dResult += (iCurrTime.tv_nsec - lBefore.tv_nsec) / 1000000000.0;
#endif
  printf("\n Total Time: %12.3f sec.\n", dResult);

  // destroy application encoder class
  cTAppEncTop.destroy();

  return 0;
}

//! \}
