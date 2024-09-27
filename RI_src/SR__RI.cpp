// Class SR__RI: Ramp Identification class
#include <iostream>		// cout, cin, endl
#include <math.h>
#include "SR__RI.h"
using namespace std;	// cout, cin, endl

// Creator --------------------------------------------------------------------
SR__RI::SR__RI()
{
	u = pX - 1;
	upN = u + RI_N;

	// Empty all internal arrays
	SRidx i;
	for (i = 0; i < RI_2NpXp1; i++) {
		_x2[i] = 0.0;
		_xu[i] = 0.0;
		_sumR_x_C[i] = 0.0;
		_sumR_xu_C[i] = 0.0;
	}

}

// Destructor -----------------------------------------------------------------
SR__RI::~SR__RI()
{

}
/*
This is a Plot of public SR__RI object methods:
	RI_set:		allocates, initializes inner memory

	RI_update:	Input: newest sensor data point
				Control data: incremental or sub-incremental precision of L?
							expected range of gradient D
							equidistant sensor points or not
				States:	not enough data, running
				Output: Confidence levels for: Ramp discovery, Corner increment, Gradient
						Ramp discovery, Corner increment, Gradient
Other methods:
	RI_FCD:		evaluate the goal function F at test corner C and test gradient D

Data structures:
	Data over the Observation Window is stored twice, allowing a quick roll back.
	_i		array containing increment numbers 
	_x		array containing sensor data
	_s		array containing path coordinate (for later)

	intC	integer value of C, floor(C)
	intCp1	intC + 1. Increments intCp1...iW represent the Ramp
	sum_x2	Sum of all x squared over all increments
	sumR_xi	Sum of (x*i) over Ramp increments
	sumR_x	Sum of (x) over Ramp increments
	sumR_1	Sum (1) over Ramp increments
	sumR_i	Sum (i) over Ramp increments
	sumR_i2 Sum(i^2) over Ramp increments

*/

//	-----------------------------------------------------------------------------------------------
//	RI_set
//	-----------------------------------------------------------------------------------------------
//	Read the Callers expectations about the observed Ramp Process and copy into SR__RI.
//	Write the expected Ramp Detection Length into the Callers struct memory.
//	Return	1	on success
//			0	on error
//
enum_RI_states SR__RI::RI_set(RI_expectations* _user_Expects) {
	RI_expectations* _in = _user_Expects;

	// Check RI_state
	if(RI_state == enum_RI_states::stopped)
		return enum_RI_states::error;

	// Check data, according to their definition
	if (!(_in->is_R0_GT_known == 0 || _in->is_R0_GT_known == 1)) 
		return enum_RI_states::error;
	if (!(_in->is_sig_GT_known == 0 || _in->is_sig_GT_known == 1)) 
		return enum_RI_states::error;
	if(	_in->sig_GT <0 
		|| (_in->is_sig_GT_known == 1 && _in->sig_GT==0 )) 
		return enum_RI_states::error;
	if (!(_in->C_resolution == 0 || _in->C_resolution == 1)) 
		return enum_RI_states::error;
	if(_in->i1S < 0 
		|| (_in->iS  && _in->iS <= _in->i1S)) 
		return enum_RI_states::error;
	if (_in->i1E < 0
		|| (_in->iE && _in->iE <= _in->i1E)
		|| _in->i1E < _in->i1S
		|| _in->iE > _in->iS
		|| (_in->i1E && _in->i1S && _in->iE < _in->iS + RI_N)) 
		return enum_RI_states::error;
	if(_in->D_min < 0.0 
		|| (_in->D_max && _in->D_max < _in->D_min)) 
		return enum_RI_states::error;
	if(!(_in->fd == 3 || _in->fd == 4)) 
		return enum_RI_states::error;
	if (_in->X_Break < 0.0)
		return enum_RI_states::error;

	// Expected Ramp Detection Length
	//	calculated only if 
	//	 - expectations on D_min and sigma_GT are given
	//	 - resulting scaled minimum gradient is greater than 1/16, i.e. gradient exponent b>-4
	//	otherwise returns : L_RD=0
	_in->L_RDm3 = 0.0;
	_in->L_RDm5 = 0.0;
	if (_in->D_min > 0.0 && _in->sig_GT > 0.0) {
		double d_GT_min = _in->D_min / _in->sig_GT;
		if (d_GT_min > 0.0625) {
			_in->L_RDm3 = ceil(a0_RDm3 / d_GT_min + c0_RDm3);
			_in->L_RDm5 = ceil(a0_RDm5 / d_GT_min + c0_RDm5);
		}
	}

	// Copy the struct into SR__RI object.
	Expects = *_in;

	// Set internal state
	RI_state = enum_RI_states::set;

	return RI_state;
}

//	-----------------------------------------------------------------------------------------------
//	RI_reset(RI_expectations*)
//	-----------------------------------------------------------------------------------------------
//	Set to expectations, empty internal array and set to initial RIstate
//	Return	reset	on success
//			error	on error
enum_RI_states SR__RI::RI_reset(RI_expectations* _user_Expects) {
	u = pX - 1;
	upN = u + RI_N;

	// Set data to _RIexpects
	if (RI_set(_user_Expects) != enum_RI_states::set) return enum_RI_states::error;

	// Empty internal arrays
	SRidx i;
	for (i = 0; i < RI_2NpXp1; i++) {
		_x2[i] = 0.0;
		_xu[i] = 0.0;
		_sumR_x_C[i] = 0.0;
		_sumR_xu_C[i] = 0.0;
	}

	// Set internal state
	RI_state = enum_RI_states::reset;

	return RI_state;
};


//	-----------------------------------------------------------------------------------------------
//	RI_update
//	-----------------------------------------------------------------------------------------------
/*	Do Ramp Identification on the newest Sensor Data.

	Input
		i		newest increment number used by the source
		X		newest Sensor Data, not yet grounded to Floor level
*/
enum_RI_states SR__RI::RI_update(SRidx i, double X, RI_results* _user_Results) {
	//	Calculation of the ML-filtered standard deviation
	SRidx uu, uupN, w;				// local iterator
	int stepintoEW = 0;				// Increment i is stepping into the Expectation Window
	double sum_w = 0;				// a sum built by iteration over w
	double R0_M_2;					// Square of R0_M
	double x;						// Sensor data grounded to Floor level R0_M, if known

	// Save inputs
	i_IN = i;
	X_IN = X;

	// Check RI_state
	if (RI_state == enum_RI_states::stopped
		|| RI_state == enum_RI_states::error) {
		return RI_state;
	}
	// Check stop condition
	if (i<Expects.i1S 
		|| (Expects.iS && i > Expects.iS)) {
		RI_state = enum_RI_states::stopped;
		return RI_state;
	}

	// Increment or roll back array indices prior to storing the incoming Sensor data.
	u++;
	upN++;
	if (u == RI_NpX) {
		u = pX;
		upN = RI_N + pX;
	}

	// Check the RI state
	switch (RI_state) {
	case enum_RI_states::created:
	case enum_RI_states::set:
	case enum_RI_states::reset:
		// RI_update was called for the first time after creation or reset. 
		RI_state = enum_RI_states::collects;
		break;

	case enum_RI_states::collects:		// Check if Observation Window is full for the first time.

		if (u == RI_NpX - 1) {
			// At least N sensor data were collected. The Window is full for the first time.
			//	RI state changes to
			//	 - i1E && i<i1E		Expectation Window no yet reached	==>	enum_RI_states::beforeEW
			//	 - !i1E || i>=i1E	Expectation Window. 				==>	enum_RI_states::noRamp
			//							Now calculate an freeze R0_ML or set it to Expects.R0_GT if known
			//							Substract R0_ML from all Sensor data
			//							Calculate and freeze the scale between u and i: diydu
			if (Expects.i1E && i < Expects.i1E) {
				RI_state = enum_RI_states::beforeEW;
			}
			else {
				RI_state = enum_RI_states::noRamp;
				stepintoEW = 1;
			}
		}
		break;

	case enum_RI_states::beforeEW:		// Check if Expectation Window has been reached.
		if (i >= Expects.i1E) {
			RI_state = enum_RI_states::noRamp;
			stepintoEW = 1;
		}
		break;
	}
	if (stepintoEW) {
		// Hint: stepintoEW = 0 reset is done a bit later.
		// If R0 is not known from external sources, it is time to calculate it from N Sensor data
		if (Expects.is_R0_GT_known) {
			R0_ML = Expects.R0_GT;
		}
		else {
			// Store the latest data
			_x_2N[u] = X - R0_ML;
			// Determine the Floor level from the average of N first sensor data
			R0_ML = fast_mean(_x_2N, pX, RI_NpX - 1);

			// Subtract the Floor level from all Sensor data arrays: _x_2N, _x2, _xu, x_im1
			uu = pX;
			uupN = RI_N + pX;
			R0_M_2 = R0_ML * R0_ML;
			for (; uu < RI_NpX; uu++, uupN++) {
				// Square of Sensor data: (X-R0)²
				_x2[uu] = _x2[uupN] = _x2[uu] - 2 * R0_ML * _x_2N[uu] + R0_M_2;
				// Sensor Data times increment number: (X-R0)*u
				_xu[uu] -= R0_ML * (double)uu;
				_xu[uupN] -= R0_ML * (double)uupN;
				// finally the Sensor data itself (X-R0)
				_x_2N[uu] = _x_2N[uupN] = _x_2N[uu] - R0_ML;
			}
		}
	}

	// Operation count: part_1.  Case: N=100, RI_state=collects				Done July 2023. No longer up to date.
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		10		3		0		1						1
	//	loop		100		6		6		0		9
	//	sum1		100		610		630		0		901	
	//	call		100		104		104		0		100							fast_mean

	// Do Signal Data Preprocessing
	// Ground Sensor data to Floor level
	x = X - R0_ML;
	// Collect sensor data in rotating arrays. 
	_i_2N[u] = _i_2N[upN] = i;
	_x_2N[u] = _x_2N[upN] = x;
	// Fill in derived Data arrays for x² and xu
	_x2[u] = _x2[upN] = x * x;
	_xu[u] = x * (double)u;
	_xu[upN] = x * (double)upN;

	// Set Window increment boundaries
	u1W = u+1;
	uW = upN;
	dbl_u1W = (double)u1W;
	dbl_uW = (double)uW;

	if (stepintoEW) {
		stepintoEW = 0;
		// Calculate the scale between u and i increments: di/du = (i(uW)-i(uW-1))/(u-(u-1))
		diydu = _i_2N[uW] - _i_2N[uW - 1];
	}
	// Sums over Signal data in the Observation Window: 
	RI_sums_of_x();

	// Only continue with RI if
	//	- the Observation Window is filled with Sensor data.
	//	- increment i falls into the Expectation Window, if it exists
	if (RI_state == enum_RI_states::collects
		|| RI_state == enum_RI_states::beforeEW)
		return RI_state;

	// Four-step Minimum search
	//	Step a):	Find minima over D of F(C,D), where C takes all integer values inside the Observation Window
	RI_M_D();

	//	Step b):	Find candidate intervals: 10 smallest M_D mean values over interval i...i+1
	rank_M_D_et_I();

	// For C_resolution==continuous==0: Find global continuous minimum
	if (!Expects.C_resolution) {
		//	Step c):	Find sub-incremental minima within candidate Intervals
		find_sub_M_D();
	}
	//	Step d):	Find the global minimum in u-coordinate increments
	find_M_C_D();
	
	// Derive more Ramp parameters from ML results 
	// Ramp Length in u-coordinates
	L_ML = uW - C_ML;
	// Ramp Height
	H_ML = L_ML * D_ML;
	// Ramp value
	R_ML = R0_ML + H_ML;

	//	Calculate the ML-filtered sensor noise for the samples in the Observation Window
	//	If minimum is of type_M=2, uC_M is smaller than u1W, cannot be used as index.
	for (w = u1W; w <= SR_max(u1W-1,(SRidx)C_ML); w++) {
		_MLnoise[w] = _x_2N[w];
	}
	for ( ; w <= uW; w++) {
		_MLnoise[w] = _x_2N[w] - ((double)(w)-C_ML) * D_ML;
	}
	sig_ML_W = fast_stdev_S(_MLnoise, u1W, uW);
	//	Set the ML-fitered sensor noise according to request
	if (Expects.is_sig_GT_known) sig_ML = Expects.sig_GT;
	else sig_ML = sig_ML_W;
	//	>>> if sig_GT and sig_ML_W differ too much, a warning could be generated

	// RI detection results and estimation of significance and uncertainties
	make_RI_results();

	// Copy RI_results into received memory
	*_user_Results = Results;

	// Set breakpoint here
	if (1) {
		SRidx nix = 0;
	}
	return RI_state;

	// Operation count.  Case: N=100, !Expects.C_resolution
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	part1		100		610		630		0		901	
	//	call		100		104		104		0		100								fast_mean
	//	part2		0		20		6		3								7		(see below)
	//	loop		100		1		2		0		2
	//	call		100		407		202		0		404								RI_sums_of_x
	//	call		100		924		4063	1		308								RI_M_D
	//	call		1150	6000	100		2000	10200							rank_M_D_et_I
	//	call		30		490		1350	31		131						10		find_sub_M_D
	//	call		0		14		0		3		7								find_M_C_D
	//	call		100		207		310		1		300		1						fast_stdev_S
	//	call		0		90		35		11		0		0		6				make_RI_results
	//	sum			1780	8867	6802	2050	12353	1		6		17
	//	rough estimation
	//				2k		9k		7k		2k		13k		20: sqrt,pow,fct calls
}

//  PRIVATE =======================================================================================
//	-----------------------------------------------------------------------------------------------
//	RI_reset()			Obsolete ?
//	-----------------------------------------------------------------------------------------------
//	Empty internal array and set to initial RIstate
//	Return	1	on success
//			0	on error
int SR__RI::RI_reset() {
	u = pX - 1;
	upN = u + RI_N;

	// Empty internal arrays
	SRidx i;
	for (i = 0; i < RI_2NpXp1; i++) {
		_x2[i] = 0.0;
		_xu[i] = 0.0;
		_sumR_x_C[i] = 0.0;
		_sumR_xu_C[i] = 0.0;
	}

	// Set internal state
	RI_state = enum_RI_states::reset;

	return 1;
};

//	-----------------------------------------------------------------------------------------------
//	RI_reset_all		Obsolete?
//	-----------------------------------------------------------------------------------------------
//	Set / Reset inital values prior to the first call to update();
int SR__RI::RI_reset_all() {

	u = pX - 1;
	upN = u + RI_N;

	// Reset external knowledge
	Expects.is_sig_GT_known = 0;
	Expects.sig_GT = 0;
	Expects.is_R0_GT_known = 0;
	Expects.R0_GT;
	R0_ML = 0;
	Expects.C_resolution = 0;

	SRidx i;
	for (i = 0; i < RI_2NpXp1; i++) {
		_x2[i] = 0.0;
		_xu[i] = 0.0;
		_sumR_x_C[i] = 0.0;
		_sumR_xu_C[i] = 0.0;
	}

	RI_state = enum_RI_states::reset;

	return true;
}

//	-----------------------------------------------------------------------------------------------
//	RI_reset_2known		Obsolete?
//	-----------------------------------------------------------------------------------------------
//	Set / Reset inital values prior to the first call to update();
int SR__RI::RI_reset_2known() {

	u = pX - 1;
	upN = u + RI_N;

	// Reset to external knowledge
	if (Expects.is_sig_GT_known) {
		sig_ML = Expects.sig_GT;
	}
	if (Expects.is_R0_GT_known) {
		R0_ML = Expects.R0_GT;
	}

	SRidx i;
	for (i = 0; i < RI_2NpXp1; i++) {
		_x2[i] = 0.0;
		_xu[i] = 0.0;
		_sumR_x_C[i] = 0.0;
		_sumR_xu_C[i] = 0.0;
	}

	RI_state = enum_RI_states::reset;

	return true;
}

//	-----------------------------------------------------------------------------------------------
//	RI_set_known		Obsolete?
//	-----------------------------------------------------------------------------------------------
//	Set the standard deviation 'sig' to a value, known from external sources, if provided.
//	Set the Floor level 'R0' to a value, known from external sources, if provided.
//	Values == zero is considered as: value is not known
int SR__RI::RI_set_known(int sig_is_known, double req_sig, int R0_is_known, double req_R0, int req_C_resolution) {

	if (sig_is_known) {
		Expects.is_sig_GT_known = 1;
		Expects.sig_GT = req_sig;
	}
	else {
		Expects.is_sig_GT_known = 0;
		Expects.sig_GT = 1;
	}
	sig_ML = Expects.sig_GT;

	if (R0_is_known) {
		Expects.is_R0_GT_known = 1;
		Expects.R0_GT = req_R0;
	}
	else {
		Expects.is_R0_GT_known = 0;
		Expects.R0_GT = 0;
	}

	Expects.C_resolution = req_C_resolution;

	return true;
}

//	-----------------------------------------------------------------------------------------------
//	RI_sums_of_x
//	-----------------------------------------------------------------------------------------------
//	Build sums of Signal data in the Observation Window for all possible Corner positions.
//		Observation Window includes array indices [u1W...uW]
//		a possible Corner position is uC. The lower Signal data array index: floor(uC).
//		Thus, floor(uC)+1 is the first array index on the Ramp. Sums built over the Ramp start here.
//		Refer to floor(uC)+1 by uCp1
int SR__RI::RI_sums_of_x() {
	SRidx uC, uCp1;				// floor(C) and floor(C)+1 with C: the Corner position to be tested.
	double sum_xC=0, sum_xuC=0;
	SRidx u1Wm2;

	// Sum of all x squared over all increments --------------------------------------------------
	//	The sum is updated every cycle by 
	//	  - adding the newest sensor data and 
	//    - subtracting the sensor data of cycle-N-1, i.e. uW-101
	//	
	//	sum_x2 does not depend on the tested Corner position.
	//		For homogenity with _x2, _sum_x_C and _sumR_xu_C, it starts with underscore.
	_sum_x2_uW = _sum_x2_uW - x2_u1Wm1 + _x2[uW];
	x2_u1Wm1= _x2[u1W];

	// _sumR_x_C:	Sum of (x) over Ramp increments for Corner candidates C=0...N-1 ----------------
	// _sumR_xu_C:	Sum of (x*u) over Ramp increments for Corner candidates C=0...N-1---------------
	//	Both sums depend on Signal data above the tested Corner position, thus are stored in arrays.
	//	The array index equals floor(C).
	//	Cells [u1W-1] contain sums needed to calculate the linear approximation over the entire Window
	_sumR_x_C[uW] = 0.0;
	_sumR_xu_C[uW] = 0.0;
	u1Wm2 = u1W - 2;
	for (uC=uW-1, uCp1=uW; uC > u1Wm2; uC--,uCp1--) {
		sum_xC += _x_2N[uCp1];
		sum_xuC += _xu[uCp1];
		_sumR_x_C[uC] = sum_xC;
		_sumR_xu_C[uC] = sum_xuC;
	}
	return 1;

	// Operation count.  Case: N=100
	//	operation	times	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		7		2		0		4
	//	loop		100		4		2		0		4
	//	sum			100		407		202		0		404
}

//	-----------------------------------------------------------------------------------------------
//	RI_FCD
//	-----------------------------------------------------------------------------------------------
//	Evaluates the goal function for a set of N equidistant sensor data with test values for C and D
//		Sensor data lies in array _x in fields u1W...uW.
//		Sums for x, x², x*u, u² have been updated previously
double SR__RI::RI_FCD(double C, double D) {
	SRidx uC = (int)floor(C);
	double F_0, F_D, F_D2, F_CD, F_CD2, F_C2D2;
	double FCD, D2 = D*D;
	double dbl_uC = (double)uC;

	//	Calculate coefficients F_D, F_D2, F_CD, F_CD2 and F_C2D2 for F(C,D) evaluation
	F_0 = _sum_x2_uW;
	F_D = -2 * _sumR_xu_C[uC];
	F_D2 = (dbl_uW * (dbl_uW + 1) * (2 * dbl_uW + 1) - dbl_uC * (dbl_uC + 1) * (2 * dbl_uC + 1)) / 6;
	F_CD = 2 * _sumR_x_C[uC];
	F_CD2 = -(dbl_uW * dbl_uW + dbl_uW - dbl_uC * dbl_uC - dbl_uC);
	F_C2D2 = dbl_uW - dbl_uC;

	// Evaluate the goal function F(C,D)
	FCD = F_0 + D * F_D + D2 * F_D2 + C * D * F_CD + C * D2 * F_CD2 + C * C * D2 * F_C2D2;

	return FCD;

	// Operation count.  Case: N=100
	//	operation	times	equals	Flops	if's	array[]	sqrt	pow		calls
	//	sum			0		10		34		0		1
}

//	-----------------------------------------------------------------------------------------------
//	RI_M_D
//	-----------------------------------------------------------------------------------------------
//	Step a) of Four-step Minimum search:	
//
/*	Find minima over D of F(C, D), where C takes all integer values inside the Observation Window
		Intern
					C			test Corner position
					uC			integer test Corner position
					uCp1		integer test Corner position plus 1
									this is the starting index for sums that calculate coefficients F_...

		Output		_D_M_4uC	Minimum of F(C,D) over gradient D at all integer Corner position in u1W...uW
					_M_D_4uC	Goal function minimum _M_D_4uC[ uC+1] over D for all integer uC in u1W...uW
*/
int SR__RI::RI_M_D() {
	SRidx uC;			//	C taking all integer values inside the Observation Window
						//		integer of C
	double dbl_uC;		//	double version of C
	double sum_uW2, sum_uW;
	double F_0, F_D, F_D2, F_CD, F_CD2, F_C2D2;
	double D_M;			//	D value where F(C,D) is minimum for a given C
	double D2_M;		//	D_M squared 
	// Calculation of C,D and F for linear regression
	double x_mean_, u_mean_, sum_u2_, sum_u_, sum_x_, sum_xu_, D2_linR;

	// Test every integer Corner position inside the Observation Window
	F_0 = _sum_x2_uW;
	sum_uW2 = dbl_uW * (dbl_uW + 1) * (2 * dbl_uW + 1);
	sum_uW = dbl_uW * dbl_uW + dbl_uW;
	for (uC = u1W; uC < uW; uC++) {
		dbl_uC = (double)uC;
		//	Calculate coefficients F_D, F_D2, F_CD, F_CD2 and F_C2D2
		F_D = -2 * _sumR_xu_C[uC];
		F_D2 = (sum_uW2 - dbl_uC * (dbl_uC + 1) * (2*dbl_uC + 1)) / 6;
		F_CD = 2 * _sumR_x_C[uC];
		F_CD2 = -(sum_uW - dbl_uC * dbl_uC - dbl_uC);
		F_C2D2 = dbl_uW - dbl_uC;

		// Find the minimum over D_M of the goal function F(C,D) for C being fixed to integer value uC
		D_M = -0.5 * (F_D + dbl_uC * F_CD) / (F_D2 + dbl_uC * F_CD2 + dbl_uC * dbl_uC * F_C2D2);
		_D_M_4uC[uC] = D_M;
		D2_M = D_M * D_M;

		// Evaluate the goal function at D_M and uC
		_M_D_4uC[ uC] = F_0 + D_M * F_D + D2_M * F_D2 + 
			dbl_uC * D_M * F_CD + dbl_uC * D2_M * F_CD2 + dbl_uC * dbl_uC * D2_M * F_C2D2;
	}
	// Values at u=uW can not exist.
	_D_M_4uC[uW] = 0.0;
	_M_D_4uC[uW] = 0.0;

	// Linear regression: test for Corner position that the Window has already passed over
	//	The required sums were stored by function 'RI_sums_of_x' under index u1W-1
	uC = u1W - 1;
	dbl_uC = (double)uC;
	F_0_ = _sum_x2_uW;
	F_D_ = -2 * _sumR_xu_C[uC];
	F_D2_ = (sum_uW2 - dbl_uC * (dbl_uC + 1) * (2 * dbl_uC + 1)) / 6;
	F_CD_ = 2 * _sumR_x_C[uC];
	F_CD2_ = -(sum_uW - dbl_uC * dbl_uC - dbl_uC);
	F_C2D2_ = dbl_uW - dbl_uC;
	// Calculate C,D and F for linear regression
	x_mean_ = F_CD_ / (2 * N);
	u_mean_ = -F_CD2_ / (2 * N);
	sum_u2_ = F_D2_;
	sum_u_ = -F_CD2_ / 2;
	sum_x_ = F_CD_ / 2;
	sum_xu_ = -F_D_ / 2;

	D_linR = (sum_xu_ - x_mean_ * sum_u_ - u_mean_ * sum_x_ + N * u_mean_ * x_mean_) /
		(sum_u2_ - 2 * u_mean_ * sum_u_ + N * u_mean_ * u_mean_);
	D2_linR = D_linR* D_linR;
	C_linR = u_mean_ - x_mean_ / D_linR;
	// For incremental resolution in C, D_linR must be determined again with floored C_linR. 
	if (Expects.C_resolution) {
		C_linR = floor(C_linR+0.5);
		D_linR = x_mean_ / (u_mean_ - C_linR);
		D2_linR = D_linR * D_linR;
	}
	F_linR = F_0_ + D_linR * F_D_ + D2_linR * F_D2_ +
		C_linR * D_linR * F_CD_ + C_linR * D2_linR * F_CD2_ + C_linR * C_linR * D2_linR * F_C2D2_;

	return true;

	// Operation count.  Case: N=100, !Expects.C_resolution
	//	operation	times	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		24		63		1		8						0
	//	loop		100		9		40		0		3
	//	sum			100		924		4063	1		308
}

//	-----------------------------------------------------------------------------------------------
//	rank_M_D_et_I
//	-----------------------------------------------------------------------------------------------
//	Step b):	Rank minima M_D:
//					Smallest F(uC,D) in the observation window for integer uC
//		and		Find candidate Intervals for sub-incremental minima: 
//					upper and lower index are within the 10 smallest M_D(uC) values
int SR__RI::rank_M_D_et_I() {
	SRidx u, uu, XRu;
	double min_M_D;
	double XM_Du;

	// Preset index list of M_D's. Exclude u=uW.
	for (u = u1W; u < uW; u++) {
		_u_RankBy_M_D[u] = u;
		_M_D_ranked[u] = _M_D_4uC[u];
	}

	// Find the Nranks=10 smallest M_D and put its index at the bottom: u1W...u1W+9
	//	uu	is the array index to be filled
	//	u	is the array index o be compared with uu
	min_M_D = _M_D_4uC[u1W];
	for (uu = u1W; uu < u1W + Nranks; uu++) {
		for (u = uu + 1; u < uW; u++) {
			if (_M_D_ranked[u] < _M_D_ranked[uu]) {
				// Put new smallest M_D in first place of _Rank_M_D and _M_D_ranked by exchange
				XRu = _u_RankBy_M_D[uu];
				_u_RankBy_M_D[uu] = _u_RankBy_M_D[u];
				_u_RankBy_M_D[u] = XRu;
				XM_Du = _M_D_ranked[uu];
				_M_D_ranked[uu] = _M_D_ranked[u];
				_M_D_ranked[u] = XM_Du;
			}
		}
	}

	// Find neighboring pairs amongst the ranked M_D's
	//	Rank the interval between these pairs. Indentify the interval by its lower index

	// Take the index u_lo with lowest M_D. Check presence of u_lo+1 in the Ranks_M_D array
	//	If u_lo+1 is present, keep u_lo in the array _Ranks_I. If not, discard.
	N_I = 0;
	for (uu = u1W; uu < u1W + Nranks; uu++) {
		for (u = u1W; u < u1W + Nranks; u++) {
			if (_u_RankBy_M_D[u] == _u_RankBy_M_D[uu] + 1) {
				_Ranks_I[N_I] = _u_RankBy_M_D[uu];
				N_I++;
				break;
			}
		}
	}
	return true;

	// Operation count.  Case: N=100
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		3	
	//	loop_1		100		2		0		0		3
	//	loop_2		10*95	6		0		2		10
	//	loop_3		10*10	1		1		1		4
	//	sum			1150	6000	100		2000	10200
}

//	-----------------------------------------------------------------------------------------------
//	find_sub_M_D
//	-----------------------------------------------------------------------------------------------
//	Step c):	Find sub-incremental minima within the candidate Intervals _Ranks_I
int SR__RI::find_sub_M_D() {
	SRidx u_I, u_lo, u_hi;
	SRidx uC;
	double F_0, F_D, F_D2, F_CD, F_CD2, F_C2D2;
	double C_1, C_3, C_5, D_1, D_3, D_5, M_1, M_3, M_5, C_M, MD_CM;
	double D2_3, D_M, D2_M, C2_M, M_D, invDet;

	double g_1,g_2;
	double h_11, h_12, h_21, h_22;
	double q_1, q_2;

	// Set number of sub-incremental minima found
	N_sIM = 0;		
	F_0 = _sum_x2_uW;

	// Loop over all intervals found previously
	for (u_I = 0; u_I < N_I; u_I++) {
		// Extract the integer Corner positions of the Interval
		u_lo = _Ranks_I[u_I];
		u_hi = _Ranks_I[u_I] + 1;
		// Determine the middle between value pairs at both Interval borders.
		C_1 = (double)u_lo;
		C_5 = (double)u_hi;
		C_3 = 0.5 * (C_1 + C_5);
		D_1 = _D_M_4uC[u_lo];
		D_5 = _D_M_4uC[u_hi];
		D_3 = 0.5 * (D_1 + D_5);
		M_1 = _M_D_4uC[u_lo];
		M_5 = _M_D_4uC[u_hi];

		// Evaluate the goal function at the middle point
		//	Replaces: M_3 = RI_FCD(C_3, D_3);
		uC = (SRidx)C_1;
		//	Calculate coefficients F_D, F_D2, F_CD, F_CD2 and F_C2D2
		F_D = -2 * _sumR_xu_C[uC];
		F_D2 = (dbl_uW * (dbl_uW + 1) * (2 * dbl_uW + 1) - C_1 * (C_1 + 1) * (2 * C_1 + 1)) / 6;
		F_CD = 2 * _sumR_x_C[uC];
		F_CD2 = -(dbl_uW * dbl_uW + dbl_uW - C_1 * C_1 - C_1);
		F_C2D2 = dbl_uW - C_1;

		// Evaluate the goal function at C_3 and D_3
		D2_3 = D_3 * D_3;
		M_3 = F_0 + D_3 * F_D + D2_3*F_D2 +
			C_3 * D_3 * F_CD + C_3 * D2_3 * F_CD2 + C_3 * C_3 * D2_3 * F_C2D2;

		// Find the minimum of parabola through {C_1,M_1}, {C_3,M_3}, {C_5,M_5}
		min3pts(&C_M, &MD_CM, C_1, C_3, C_5, M_1, M_3, M_5);

		// Check whether the minimum is valid
		if (C_M > C_1 && C_M < C_5) {
			// Linear interpolation of the Gradient D between C_1 and C_5 by C_M
			D_M = D_1 + (C_M - C_1) * (D_5 - D_1) / (C_5 - C_1);
			C2_M = C_M * C_M;
			D2_M = D_M * D_M;

			// Do one Newton step
			//	Gradient 
			g_1 = D2_M * F_CD2 + 2 * C_M * D2_M * F_C2D2 + D_M * F_CD;
			g_2 = 2 * D_M * F_D2 + 2 * C_M * D_M * F_CD2 + 2 * C2_M * D_M * F_C2D2 + F_D + C_M * F_CD;
			//	Hessian
			h_11 = 2 * D2_M * F_C2D2;
			h_12 = 2 * D_M * F_CD2 + 4 * C_M * D_M * F_C2D2 + F_CD;
			h_21 = h_12;
			h_22 = 2 * F_D2 + 2 * C_M * F_CD2 + 2 * C2_M * F_C2D2;
			//	Newton step
			invDet = 1.0 / (h_11 * h_22 - h_12 * h_21);
			q_1 = -(h_22 * g_1 - h_12 * g_2) * invDet; 
			q_2 = (h_21 * g_1 - h_11 * g_2) * invDet;

			// Minimum after one Newton step
			C_M += q_1;
			D_M += q_2;

			if (C_M > C_1 && C_M < C_5) {
				// Evaluate the goal function at C_M and D_M
				//	replaces: M_D = RI_FCD(_C_Mu[N_sIM], _D_Mu[N_sIM]);
				D2_M = D_M * D_M;
				M_D = F_0 + D_M * F_D + D2_M * F_D2 +
					C_M * D_M * F_CD + C_M * D2_M * F_CD2 + C_M * C_M * D2_M * F_C2D2;

				// Store the sub-incremental minimum
				_C_Mu[N_sIM] = C_M;
				_D_Mu[N_sIM] = D_M;

				// M_D from min3pts
				_M_CMu[N_sIM] = M_D;
				N_sIM++;
			}
		}
	}

	// Find the smallest amongst sub-incremental minima, if there exists any
	u_sIM = 0;
	if (N_sIM) {
		double M_sIM = _M_CMu[0];
		for (u_I = 0; u_I < N_sIM; u_I++) {
			if (_M_CMu[u_I] < M_sIM) {
				u_sIM = u_I;
				M_sIM = _M_CMu[u_I];
			}
		}
	}
	return true;

	// Operation count.  Case max: N_sIM==N_I==N_ranks==10
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		6		0		1		1
	//	loop_1		10		37		111		2		11						1		min3pts
	//	loop_2		10		2		0		1		2
	//	min3pts		10		10		24		0		0
	//	sum			30		490		1350	31		131						10
}

//	-----------------------------------------------------------------------------------------------
//	find_M_C_D
//	-----------------------------------------------------------------------------------------------
//	Step d):	Find the global minimum o ML
//				ML results C and D are given in u-coordinate increments
//
//	The minimum is amongst of the following cases
//		_M_D_ranked[u1W]	:	smallest M_D at integer increment border,
//								D is to be found in _D_M_4uC[u_M] at u_M = _u_RankBy_M_D[u1W]
//		_M_CMu[u_M]			:	smallest sub-incremental minimum. Exist only if N_sIM>0
//								Search for u_M 
int SR__RI::find_M_C_D() {
	SRidx u_M = 0;		// index of minimum local minimum

	// Start by setting the global minimum to the smallest incremental minimum
	u_M = _u_RankBy_M_D[u1W];
	C_ML = (double)u_M;
	D_ML = _D_M_4uC[u_M];
	F_ML = _M_D_ranked[u1W];
	type_M = 0;							// Type of the global minimum :	0	at increment border

	// If there exist any sub-incremental minima, try to find a smaller minimum
	//	except if incremental resolution is requested: C_resolution==1
	if (N_sIM && !Expects.C_resolution) {
		// Compare the smallest sub-incremental minimum to the smallest incremental minimum
		if (_M_CMu[u_sIM] < F_ML) {
			// Set the global minimum to the smallest sub-incremental minimum
			C_ML = _C_Mu[u_sIM];
			D_ML = _D_Mu[u_sIM];
			F_ML = _M_CMu[u_sIM];
			type_M = 1;
		}
	}

	// Compare the linear regression to the minimum found so far.
	//	Zero crossing of regression line must hold C_linR < u1W
	if (F_linR < F_ML && C_linR < dbl_u1W) {
		C_ML = C_linR;
		D_ML = D_linR;
		F_ML = F_linR;
		type_M = 2;
	}

	return true;

	// Operation count.  Case: N=100, !Expects.C_resolution
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		14		0		3		7
	//	sum			0		14		0		3		7
}

//	-----------------------------------------------------------------------------------------------
//	make_RI_results
//	-----------------------------------------------------------------------------------------------
//	Calculate RI results
//	Return	1	on success
//			0	on error
//	Results are:
//		Corner detection significance and history
//		RI estimation of significance and uncertainties
//
int SR__RI::make_RI_results() {

	// Transformation from u- to i-coordintes
	Results.uW = uW;
	Results.iW = i_IN;
	Results.diydu = diydu;

	//Q1: Ramp Detection
	//	Determine whether the ramp detection criterion is fullfilled
	//	Signal to Noise ratios m=3 and m=5 with Noise measure sig_ML
	S2N_ML = H_ML / sig_ML;
	Q1m3_ML = S2N_ML > c0_Q1m_FD3 ? 1 : 0;
	Q1m5_ML = S2N_ML > c0_Q1m_FD4 ? 1 : 0;
	// Set RI_state and substate
	if (Q1m5_ML) {
		RI_state = enum_RI_states::detected;
		RIsub_noRamp = enum_RI_substates::unknown;
		RIsub_detected = enum_RI_substates::CornerFD4;
	}
	else if (Q1m3_ML) {
		RI_state = enum_RI_states::detected;
		RIsub_noRamp = enum_RI_substates::unknown;
		RIsub_detected = enum_RI_substates::CornerFD3;
	}
	else {
		// Check whether Corner had been detected previously 
		if (RI_state == enum_RI_states::detected
			|| RI_state == enum_RI_states::identified) {
			RI_state = enum_RI_states::noRamp;
			RIsub_noRamp = enum_RI_substates::CornerLost;
		}
		else {
			RI_state = enum_RI_states::noRamp;
			RIsub_noRamp = enum_RI_substates::unknown;
		}
	}

	// Save ML results. ML results are in u-coordinates and scaled by sig_ML
	Results.sig_ML = sig_ML;		// ML-filtered standard deviation from GT or Window
	Results.C_ML = C_ML;			// Corner position in u-coordinates
	Results.L_ML = L_ML;			// Length
	Results.D_ML = D_ML;			// Gradient
	Results.H_ML = H_ML;			// Height
	Results.R0_ML = R0_ML;			// Floor level from GT or Window
	Results.R0_GT = Expects.R0_GT;	// GT Floor level
	Results.F_D_ML = F_ML;			// Minimization Function value at the minimum
	Results.Q1m3_ML = Q1m3_ML;		// Q1 criterion for m=3 using sig_ML
	Results.Q1m5_ML = Q1m5_ML;		// Q1 criterion for m=5 using sig_ML

	// Determine Bias and Noise model coefficients.
	L_aQt = 0;
	if (RI_state == enum_RI_states::detected) {
		double d_ML = D_ML / sig_ML;
		// aQt Length in u-coordiantes
		L_aQt = a0_aQt / d_ML + c0_aQt;

		// Check whether aQt-Length is reached
		if (L_ML >= L_aQt) {

			// Augment the RI_state to 'identified'
			RI_state = enum_RI_states::identified;

			if (Expects.is_R0_GT_known) {
				// ML-estimated standard deviation sig_ML
				mrs_nsig_RI = c0_mrs_nsig + c1_mrs_nsig * L_ML;
				srs_nsig_RI = c0_srs_nsig + c1_srs_nsig * L_ML;

				// ML-estimated Ramp Length
				mrs_L_uRI = c0_mrs_L;
				srs_L_uRI = c0_srs_L * pow(L_ML, a1_srs_L) * pow(d_ML, a2_srs_L);

				// ML-estimated Ramp Gradient D_ML
				mrs_nD_uRI = c0_mrs_nD * pow(L_ML, a0_mrs_nD);
				srs_nD_uRI = c0_srs_nD * pow(L_ML, a0_srs_nD);

				// ML-estimated Ramp Height H_ML
				mrs_nH_RI = c0_mrs_nH * pow(L_ML, a0_mrs_nH);
				srs_nH_RI = c0_srs_nH * pow(L_ML, a0_srs_nH);
			}
			else {
				// ML-estimated standard deviation sig_ML
				mrs_nsig_RI = c0_mrs_nsig_R0ukn + c1_mrs_nsig_R0ukn * L_ML;
				srs_nsig_RI = c0_srs_nsig_R0ukn + c1_srs_nsig_R0ukn * L_ML;
				// ML-estimated Ramp Length
				mrs_L_uRI = c0_mrs_L_R0ukn;
				srs_L_uRI = c0_srs_L_R0ukn * pow(L_ML, a1_srs_L_R0ukn) * pow(d_ML, a2_srs_L_R0ukn);
				// ML-estimated Ramp Gradient D_ML
				mrs_nD_uRI = c0_mrs_nD_R0ukn * pow(L_ML, a0_mrs_nD_R0ukn);
				srs_nD_uRI = c0_srs_nD_R0ukn * pow(L_ML, a0_srs_nD_R0ukn);
				// ML-estimated Ramp Height H_ML
				mrs_nH_RI = c0_mrs_nH_R0ukn * pow(L_ML, a0_mrs_nH_R0ukn);
				srs_nH_RI = c0_srs_nH_R0ukn * pow(L_ML, a0_srs_nH_R0ukn);
			}
		}
	}
	else 
	{
		L_aQt = RI_N;
		mrs_L_uRI = 0;
		srs_L_uRI = 0;
		mrs_nsig_RI = 0;
		srs_nsig_RI = 0;
		mrs_nD_uRI = 0;
		srs_nD_uRI = 0;
		mrs_nH_RI = 0;
		srs_nH_RI = 0;
	}

	// Operation count: part 1 ///////////////////////////////////

	// Copy into RIresults
	//	aQt'd Length
	Results.L_aQt_uRI = L_aQt;
	Results.mrs_L_uRI = mrs_L_uRI;
	Results.srs_L_uRI = srs_L_uRI;
	Results.mrs_nsig_RI = mrs_nsig_RI;
	Results.srs_nsig_RI = srs_nsig_RI;
	Results.mrs_nD_uRI = mrs_nD_uRI;
	Results.srs_nD_uRI = srs_nD_uRI;
	Results.mrs_nH_RI = mrs_nH_RI;
	Results.srs_nH_RI = srs_nH_RI;

	// Bias and Noise at original Signal Scale
	//	Length: no scaling necessary. Results are mrs_L_RI and srs_L_RI.
	//	Signal Noise sigma
	mrs_sig_RI = sig_ML * mrs_nsig_RI;
	srs_sig_RI = sig_ML * srs_nsig_RI;
	mrs_D_uRI = sig_ML * mrs_nD_uRI;
	srs_D_uRI = sig_ML * srs_nD_uRI;
	mrs_H_RI = sig_ML * mrs_nH_RI;
	srs_H_RI = sig_ML * srs_nH_RI;

	// Determin Noise-multipliers
	//	Confidence >99.73%, "fd3"
	m_sig_fd3 = 0.0;
	m_L_fd3 = 0.0;
	m_D_fd3 = 0.0;
	m_H_fd3 = 0.0;
	//	Confidence >99.99%, "fd4"
	m_sig_fd4 = 0.0;
	m_L_fd4 = 0.0;
	m_D_fd4 = 0.0;
	m_H_fd4 = 0.0;

	if (!Expects.is_R0_GT_known && !Expects.is_sig_GT_known) {
		// Noise-multipliers from case: GT_none
		m_sig_fd3 = m_sig_fd3_GT_none;
		m_L_fd3 = m_L_fd3_GT_none;
		m_D_fd3 = m_D_fd3_GT_none;
		m_H_fd3 = m_H_fd3_GT_none;
		m_sig_fd4 = m_sig_fd4_GT_none;
		m_L_fd4 = m_L_fd4_GT_none;
		m_D_fd4 = m_D_fd4_GT_none;
		m_H_fd4 = m_H_fd4_GT_none;
	}
	else if (!Expects.is_R0_GT_known && Expects.is_sig_GT_known) {
		// Noise-multipliers from case: GT_sig 
		m_sig_fd3 = m_sig_fd3_GT_sig;
		m_L_fd3 = m_L_fd3_GT_sig;
		m_D_fd3 = m_D_fd3_GT_sig;
		m_H_fd3 = m_H_fd3_GT_sig;
		m_sig_fd4 = m_sig_fd4_GT_sig;
		m_L_fd4 = m_L_fd4_GT_sig;
		m_D_fd4 = m_D_fd4_GT_sig;
		m_H_fd4 = m_H_fd4_GT_sig;
	}
	else if (Expects.is_R0_GT_known && !Expects.is_sig_GT_known) {
		// Take Noise-multipliers from case: GT_R0
		m_sig_fd3 = m_sig_fd3_GT_R0;
		m_L_fd3 = m_L_fd3_GT_R0;
		m_D_fd3 = m_D_fd3_GT_R0;
		m_H_fd3 = m_H_fd3_GT_R0;
		m_sig_fd4 = m_sig_fd4_GT_R0;
		m_L_fd4 = m_L_fd4_GT_R0;
		m_D_fd4 = m_D_fd4_GT_R0;
		m_H_fd4 = m_H_fd4_GT_R0;
	}
	else {  // if (Expects.is_R0_GT_known && Expects.is_sig_GT_known) {
		// Take Noise-multipliers from case: GT_R0_sig
		m_sig_fd3 = m_sig_fd3_GT_R0_sig;
		m_L_fd3 = m_L_fd3_GT_R0_sig;
		m_D_fd3 = m_D_fd3_GT_R0_sig;
		m_H_fd3 = m_H_fd3_GT_R0_sig;
		m_sig_fd4 = m_sig_fd4_GT_R0_sig;
		m_L_fd4 = m_L_fd4_GT_R0_sig;
		m_D_fd4 = m_D_fd4_GT_R0_sig;
		m_H_fd4 = m_H_fd4_GT_R0_sig;
	}

	// Fill results. RI results are scaled to i-coordinates.
	//	aQt'd Length
	Results.L_aQt_RI = L_aQt * diydu;

	// RI Ramp Parameters.
	// Scale back from u to i coordinates. Scaling factor is: di/du = (iW - i1W) / (uW-u1W)
	Results.sig_RI = sig_ML - mrs_sig_RI;
	Results.L_RI = (L_ML - mrs_L_uRI) * diydu;
	Results.i_C_RI = i_IN - Results.L_RI;	// in i-coordinates
	Results.D_RI = (D_ML - mrs_D_uRI) / diydu;
	Results.H_RI = H_ML - mrs_H_RI;

	// RI Uncertainties, scaled to i-coordinates.
	// Uncertainties for Confidence > 99.73%
	Results.U_sig_fd3 = m_sig_fd3 * srs_sig_RI;
	Results.U_L_fd3 = m_L_fd3 * srs_L_uRI * diydu;
	Results.U_D_fd3 = m_D_fd3 * srs_D_uRI / diydu;
	Results.U_H_fd3 = m_H_fd3 * srs_H_RI;
	// Uncertainties for Confidence > 99.99%
	Results.U_sig_fd4 = m_sig_fd4 * srs_sig_RI;
	Results.U_L_fd4 = m_L_fd4 * srs_L_uRI * diydu;
	Results.U_D_fd4 = m_D_fd4 * srs_D_uRI / diydu;
	Results.U_H_fd4 = m_H_fd4 * srs_H_RI;

	// Check for unexpected situations
	if (Expects.X_Break && X_IN > Expects.X_Break) {
		RIsub_unexpected = enum_RI_substates::XiGTXBreak;
	}
	else if ((Expects.D_min && Results.D_RI<Expects.D_min)
		|| Results.D_RI && Results.D_RI > Expects.D_max) {
		RIsub_unexpected = enum_RI_substates::unexpected_D;
	}
	else if ((Expects.i1E && Results.i_C_RI<Expects.i1E) 
		|| (Results.i_C_RI && Results.i_C_RI > Expects.iE)) {
		RIsub_unexpected = enum_RI_substates::unexpected_iC;
	}
	else RIsub_unexpected = enum_RI_substates::as_expected;

	// Transfer RI states and substates into Results
	Results.RI_state = RI_state;
	Results.RIsub_noRamp = RIsub_noRamp;
	Results.RIsub_detected = RIsub_detected;
	Results.RIsub_unexpected = RIsub_unexpected;

	return 1;

	// Operation count.  Case: N=100, L_ML < L_aQt
	//	operation	loops	equals	Flops	if's	array[]	sqrt()	pow()	calls
	//	part_1		0		40		13		5		0		0		6
	//	part_2		0		50		22		6
	//	sum			0		90		35		11		0		0		6
}


//	-----------------------------------------------------------------------------------------------
//	min3pts
//	-----------------------------------------------------------------------------------------------
//	Find the minimum of a parabola given by 3 equidistant points by ascending oder in u coordinate:
//		{u_I,v_I}, {u_III,v_III}, {u_V,v_V}
int SR__RI::min3pts(double* _u_M, double* _v_M,
	double u_I, double u_III, double u_V, double v_I, double v_III, double v_V) {

	double u_II, u_IV;
	double dv_II, dv_III, dv_IV;
	double ddv;
	double u_M, v_M;

	// mean u coordinates between I-III and III-V
	u_II = 0.5 * (u_I + u_III);
	u_IV = 0.5 * (u_III + u_V);
	// gradients at II and IV
	dv_II = (v_III - v_I) / (u_III - u_I);
	dv_IV = (v_V - v_III) / (u_V - u_III);
	dv_III = 0.5 * (dv_II + dv_IV);
	// second derivative
	ddv = (dv_IV - dv_II) / (u_IV - u_II);

	// location u of the minimum
	u_M = u_II - dv_II / ddv;

	// Minimum value in u_M from Taylor series around u_III
	v_M = 0.5 * ddv * SR_sqr(u_M - u_III) + dv_III * (u_M - u_III) + v_III;

	// transfer results
	*_u_M = u_M;
	*_v_M = v_M;

	return true;

	// Operation count.  Case: N=100
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		10		24		0
	//	sum			0		10		24		0		0
}

//	-----------------------------------------------------------------------------------------------
//	fast_mean
//	-----------------------------------------------------------------------------------------------
/*
	Mean value of N sample values _y[i1]..._y[iZ].
		stdev_S = ( 1/(N) sum(for i=i1 to iZ)(_y[i])
	Without any checks.
*/
double SR__RI::fast_mean(double* _y, SRidx i1, SRidx iZ) {
	SRidx Nm1 = iZ - i1;	// N-1: number of values
	double dblN = (double)(Nm1 + 1);
	SRidx i;
	double sum_y = 0;
	double y_mean;
	// Sum _y and _y² over the indice range
	for (i = i1; i <= iZ; i++) {
		sum_y += _y[i];
	}
	y_mean = sum_y / dblN;
	return y_mean;

	// Operation count.  Case: N=100
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		4		4		0		0
	//	loop		100		1		1		0		1
	//	sum			100		104		104		0		100
}

//	-----------------------------------------------------------------------------------------------
//	fast_stdev_S
//	-----------------------------------------------------------------------------------------------
/*
	Standard deviation of N sample values _y[i1]..._y[iZ].
		stdev_S = sqrt ( 1/(N-1) sum(for i=i1 to iZ)(_y[i]-mean_y)²
	Without any checks.
*/
double SR__RI::fast_stdev_S(double* _y, SRidx i1, SRidx iZ) {
	SRidx Nm1 = iZ - i1;	// N-1: number of values
	double dblN = (double)(Nm1 + 1);
	SRidx i;
	double sum_y2 = 0, sum_y = 0, sumOsums;
	double y_mean, stdev;
	// Sum _y and _y² over the indice range
	for (i = i1; i <= iZ; i++) {
		sum_y += _y[i];
		sum_y2 += _y[i] * _y[i];
	}
	y_mean = sum_y / dblN;
	sumOsums = sum_y2 - 2 * y_mean * sum_y + dblN * y_mean * y_mean;
	stdev = sqrt(sumOsums / (double)Nm1);
	if (0) {	// result to cout
		for (i = i1; i <= iZ; i++) {
			cout << _y[i] << endl;
		}
	}
	return stdev;

	// Operation count.  Case: N=100
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		7		10		1		0		1
	//	loop		100		2		3		0		3	
	//	sum			100		207		310		1		300		1
}

//	-----------------------------------------------------------------------------------------------
//	fast_stdev_N
//	-----------------------------------------------------------------------------------------------
/*
	Standard deviation of a population of N values _y[i1]..._y[iZ].
		stdev_N = sqrt ( 1/N sum(for i=i1 to iZ)(_y[i]-mean_y)²
	Without any checks.
*/
double SR__RI::fast_stdev_N(double* _y, SRidx i1, SRidx iZ) {
	SRidx N = iZ - i1 + 1;	// N-1: number of values
	double dblN = (double)(N);
	SRidx i;
	double sum_y2 = 0, sum_y = 0, sumOsums;
	double y_mean, stdev;
	// Sum _y and _y² over the indice range
	for (i = i1; i <= iZ; i++) {
		sum_y += _y[i];
		sum_y2 += _y[i] * _y[i];
	}
	y_mean = sum_y / dblN;
	sumOsums = sum_y2 - 2 * y_mean * sum_y + dblN * y_mean * y_mean;
	stdev = sqrt(sumOsums / (double)N);
	return stdev;

	// Operation count.  Case: N=100
	//	operation	loops	equals	Flops	if's	array[]	sqrt	pow		calls
	//	top			0		7		10		0		0		1
	//	loop		100		3		3		0		3		
	//	sum			100		307		310		0		300		1
}


//////
//////// From ChatGPT: Quicksort algorithm in C code.
//////#include <stdio.h>
//////
//////// Function to swap two elements
//////void swap(int* a, int* b) {
//////	int t = *a;
//////	*a = *b;
//////	*b = t;
//////}
//////
//////// Function to partition the array and return the pivot index
//////int partition(int arr[], int low, int high) {
//////	int pivot = arr[high]; // Choose the rightmost element as the pivot
//////	int i = (low - 1); // Index of smaller element
//////
//////	for (int j = low; j <= high - 1; j++) {
//////		// If current element is smaller than or equal to the pivot
//////		if (arr[j] <= pivot) {
//////			i++; // Increment index of smaller element
//////			swap(&arr[i], &arr[j]);
//////		}
//////	}
//////	swap(&arr[i + 1], &arr[high]);
//////	return (i + 1);
//////}
//////
//////// Function to perform the quicksort
//////void quicksort(int arr[], int low, int high) {
//////	if (low < high) {
//////		// Find pivot index
//////		int pi = partition(arr, low, high);
//////
//////		// Recursively sort elements before and after the pivot
//////		quicksort(arr, low, pi - 1);
//////		quicksort(arr, pi + 1, high);
//////	}
//////}
