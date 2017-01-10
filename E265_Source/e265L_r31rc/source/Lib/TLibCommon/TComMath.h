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
	\file   	TComMath.h
   	\brief    	Tile encoder class (header)
   	\author 	Jinwuk Seok (in ETRI) 
   	\date 	2015 5 26 
*/

#ifndef __TCOMMATH__
#define __TCOMMATH__

// Include files
#include "TLibCommon/CommonDef.h"

#include <assert.h>
#include <stdlib.h>
#include <math.h>

#include <stdio.h>


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TComMath
{
private:

	Double*	X;	///< State : Assume N x 1 @ 2015 5 26 by Seok
	Double*	Y;	///< Input : Assume M x 1 @ 2015 5 26 by Seok
	Double*	Z;	///< Input : Assume N x 1 @ 2015 5 26 by Seok
	
	Double**	F;	///< System : N x N @ 2015 5 26 by Seok
	Double**	H;	///< System : M x N @ 2015 5 26 by Seok	
	Double**	P; 	///< Error Covariance N x N @ 2015 5 26 by Seok
	Double**	Q; 	///< System Noise Covariance N x N @ 2015 5 26 by Seok
	Double**	R; 	///< Observing Noise Covariance M x M @ 2015 5 26 by Seok
	Double**	K;	///< Kalman Gain N x M @ 2015 5 26 by Seok

	Double	L;	///< Simple IIR Filter Gain @ 2015 5 26 by Seok
	UInt		N;	///< Simple MA Tab @ 2015 5 26 by Seok

	Double*	Sg;	///< Sg_i := Xi + u_i , Sg_i+n := Xi - u_i  For Unscented Kalman Filter @ 2015 5 26 by Seok	
	Double*	W;	///< W_i := Xi + u_i , Kai_i+n := Xi - u_i  For Unscented Kalman Filter @ 2015 5 26 by Seok	
	Double	kp;	///< Covariance of Input @ 2015 5 26 by Seok
	Double	Xm;	///< Mean of Input @ 2015 5 26 by Seok

	UInt	em_uiStateDim;	
	UInt	em_uiObsvrDim;	
	
public:
	TComMath();
	virtual ~TComMath();

	// -------------------------------------------------------------------------------------------------------------------
	// Creation and Initilization 
	// -------------------------------------------------------------------------------------------------------------------
	Void	create					(UInt uiDimIn, UInt uiDImOut);
	Void	destroy 				(Bool bOperation);
	Void	init  					(Double dStateCov, Double dObserverCov, Bool bOperation);

	// -------------------------------------------------------------------------------------------------------------------
	// Auxiliary Function
	// -------------------------------------------------------------------------------------------------------------------
	Double	InnerProduct		(Double*   e_dx, Double*   e_dy);
	Double	WedgeProduct	(Double*   e_dx, Double*   e_dy);
	Double	MatrixProduct  	(Double** e_dX, Double** e_dY);

	// -------------------------------------------------------------------------------------------------------------------
	// member Access/Estimation Filter Setting
	// -------------------------------------------------------------------------------------------------------------------
	Void 	ETRI_SetKalmanFilter 	(Double dStateCov, Double dObserverCov, Double* e_dInitValue);
	Void 	ETRI_SetUKFEstimator 	(Double dStateCov, Double dObserverCov, Double* e_dInitValue);
	Void 	ETRI_SetIIREstimator 	(Double dLambda, Double* e_dInitValue);	
	Void 	ETRI_SetFIREstimator 	(UInt uiFIRTab, Double* e_dInitValue);	

	// -------------------------------------------------------------------------------------------------------------------
	// Estimation Filters
	// -------------------------------------------------------------------------------------------------------------------
	Double	ETRI_KalmanEstimator 	(Double dInput);
	Double	ETRI_UKFEstimator   	(Double dInput);
	Double	ETRI_SimpleIIREstimator	(Double dInput);
	Double	ETRI_SimpleFIREstimator(Double dInput);

};
//! \}
#endif
