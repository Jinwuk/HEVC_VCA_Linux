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
	\file   	TComMath.cpp
   	\brief    	TComMath class (source)
	\author 	Jinwuk Seok (in ETRI) 
	\date	2015 5 26 
*/


// Include files
#include "TComMath.h"

//! \ingroup TLibEncoder
//! \{

TComMath::TComMath()
{
	X = nullptr;
	Y = nullptr;
	Z = nullptr;

	F = nullptr;
	H = nullptr;
	P = nullptr;
	Q = nullptr;
	R = nullptr;
	K = nullptr;

	L = 0.0;
	N = 0;

	Sg = nullptr;
	W  = nullptr;
	kp = 0.0;
	Xm = 0.0;

}

TComMath::~TComMath()
{

}

// -------------------------------------------------------------------------------------------------------------------
// Creation and Initilization 
// For Observation : 	em_uiObsvrDim x  em_uiStateDim
// For State Update : 	em_uiStateDim  x em_uiStateDim 
// -------------------------------------------------------------------------------------------------------------------

Void	TComMath::create(UInt uiDimIn, UInt uiDImOut)
{
	em_uiStateDim = uiDimIn;	
	em_uiObsvrDim = uiDImOut;	

	X 	= new Double [em_uiStateDim];
	Z 	= new Double [em_uiStateDim];
	Y 	= new Double [em_uiObsvrDim];

	F	= new Double*[em_uiStateDim];	///< N x N 2015 5 26 by Seok
	P	= new Double*[em_uiStateDim];	///< N x N 2015 5 26 by Seok
	Q	= new Double*[em_uiStateDim]; 	///< N x N 2015 5 26 by Seok
	H 	= new Double*[em_uiObsvrDim];	///< M x N 2015 5 26 by Seok
	K 	= new Double*[em_uiStateDim];  	///< N x M 2015 5 26 by Seok

	for(Int k=0; k<em_uiStateDim; k++)
	{
		F[k] = new Double [em_uiStateDim];
		P[k] = new Double [em_uiStateDim];
		Q[k] = new Double [em_uiStateDim];
		H[k] = new Double [em_uiStateDim];
	}

	R 	= new Double*[em_uiObsvrDim];	///< M x M @ 2015 5 26 by Seok
	for(Int k=0; k<em_uiObsvrDim; k++)
	{
		K[k] = new Double [em_uiStateDim];
		R[k] = new Double [em_uiObsvrDim];
	}

	Sg 	= new Double [em_uiStateDim * 2 + 1];
	W 	= new Double [em_uiStateDim * 2 + 1];
	

}

Void	TComMath::destroy(Bool bOperation)
{
	DeleteParam(X);
	DeleteParam(Z);
	DeleteParam(Y);
	DeleteParam(Sg);
	DeleteParam(W);

	for (Int k=0; k< em_uiStateDim; k++)
	{
		DeleteDimParam(H[k]);
		DeleteDimParam(Q[k]);
		DeleteDimParam(P[k]);
		DeleteDimParam(F[k]);
	}
	DeleteDimParam(H);
	DeleteDimParam(Q);
	DeleteDimParam(P);
	DeleteDimParam(F);

	for (Int k=0; k< em_uiStateDim; k++)
	{
		DeleteDimParam(K[k]);
		DeleteDimParam(R[k]);
	}
	DeleteDimParam(K);
	DeleteDimParam(R);
}

Void	TComMath::init(Double dStateCov, Double dObserverCov, Bool bOperation)
{

	for(Int j =0; j < em_uiStateDim; j++){
		for(Int i =0; i < em_uiStateDim; i++){
			Q[j][i] = (i == j)? 	dStateCov : 0.0;
		}
	}
		
	for(Int j =0; j < em_uiStateDim; j++){
		for(Int i =0; i < em_uiStateDim; i++){
			R[j][i] = (i == j)? 	dObserverCov : 0.0;
		}
	}


}

// -------------------------------------------------------------------------------------------------------------------
// Auxiliary Function
// -------------------------------------------------------------------------------------------------------------------
Double	TComMath::InnerProduct(Double*   e_dx, Double*   e_dy, Int iDim)
{
	Double	rtValue = 0.0;

	for(Int k=0; k<iDim; k++)
	rtValue += (e_dx[k] * e_dy[k]);

	return rtValue;
}
Double	TComMath::WedgeProduct	(Double*   e_dx, Double*   e_dy)
{
	Double	rtValue = 0.0;

	return rtValue;
}
Double	TComMath::MatrixProduct  	(Double** e_dX, Double** e_dY)
{
	Double	rtValue = 0.0;

	return rtValue;
}

// -------------------------------------------------------------------------------------------------------------------
// member Access/Estimation Filter Setting
// -------------------------------------------------------------------------------------------------------------------
Void 	TComMath::ETRI_SetKalmanFilter 	(Double dStateCov, Double dObserverCov, Double* e_dInitValue)
{


}
Void 	TComMath::ETRI_SetUKFEstimator 	(Double dStateCov, Double dObserverCov, Double* e_dInitValue)
{


}
Void 	TComMath::ETRI_SetIIREstimator 	(Double dLambda, Double* e_dInitValue)
{


}
Void 	TComMath::ETRI_SetFIREstimator 	(UInt uiFIRTab, Double* e_dInitValue)
{


}

// -------------------------------------------------------------------------------------------------------------------
// Estimation Filters
// -------------------------------------------------------------------------------------------------------------------
Double	TComMath::ETRI_KalmanEstimator 	(Double dInput)
{
	Double	rtValue = 0.0;

	return rtValue;
}
Double	TComMath::ETRI_UKFEstimator   	(Double dInput)
{
	Double	rtValue = 0.0;

	return rtValue;
}
Double	TComMath::ETRI_SimpleIIREstimator	(Double dInput)
{
	Double	rtValue = 0.0;

	return rtValue;
}
Double	TComMath::ETRI_SimpleFIREstimator(Double dInput)
{
	Double	rtValue = 0.0;

	return rtValue;
}






//! \}
