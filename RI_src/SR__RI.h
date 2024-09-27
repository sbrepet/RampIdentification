#pragma once

#define RI_N 100			// Number of increments in the Observation Window
#define pX 2				// Array index starts at pX: 2 for Excel, 1 for Matlab, 0 for C
#define RI_NpX 102			// SR__RI_N + pX
#define RI_2NpXp1 203		// Size of cyclic arrays storing Sensor data. X: Excel shift +2. 
							//	Must equal (2 * SR__RI_N) + 1 + pX !
#define SRidx long long
#define newSRidx(var) SRidx var = 0
#define RI_Ncand 10			// number of candidates of global minimum of F(C,D)
#define SR_sqr(a) ((a)*(a))
#define SR_max(a,b) ((a)>(b)?(a):(b))

//	enumerated RI states
typedef enum class SR__RIstates {
	none = 0,				//	no state or unknown state
	created,				//	SR__RI object is created. Signal data buffer is empty.
	set,					//	Expectations are set or modified. Signal data buffer remains unchanged.
	reset,					//	Expectations are set. Signal data buffer is empty.
	collects,				//	less than N sensor data were collected
	beforeEW,				//	RI has N data, but i<i1E: waiting before Expectation Window opens
	noRamp,					//	RI is active but no Ramp detected
	detected,				//	Ramp detected : iC, sigma, L, D, H, F_CD,
	identified,				//	Ramp Parameters and Uncertainties are determined
	unexpected,				//	Ramp outside expectations: D out of limits, i>iE, X > X_Break
	stopped,				//	RI stopped. i<i1S, i>iS. Call RI_reset().
	error,/**/				//	Erroneous input data: **see below
	EndofStates				//	marks the end of this enum class
}enum_RI_states;

/*	RI_state equals enum_RI_states::error in one or more of the following conditions holds :
	
	Erroneous input data in struct RI_expectations:
	is_R0_GT_known			is neither 0 nor 1				must be 0 or 1
	is_sig_GT_known			is neither 0 nor 1				must be 0 or 1
	sig_GT					is <0							must be 0 or positive
	is_sig_GT_known==true but sig_GT==0						if known, sig_GT must be positive
	C_resolution			is neither 0 nor 1				must be 0 or 1
	i1S < 0					increment numbers must be positive
	iS <= i1S				increment numbers must be in growing order
	i1E < 0					increment numbers must be positive
	iE <= i1E				increment numbers must be in growing order
	i1E < i1S				Expectation window must lie within start and stop increment
	iE > iS					Expectation window must lie within start and stop increment
	D_min < 0.0				expected minimum Gradient must be zero:(not set) or positive
	D_max < D_min)			expected maximum Gradient must not be smaller than minimum
	fd == 3 || fd == 4		Confidence level must be set to 3:99.73% or 4:99.99%
	X_Break < 0.0			Breaking condition must be zero:(not set) or positive

 */

//	RI sub-states
typedef enum class SR__RI_substate {
	unknown = 0,			//	unknown substate

	error_divider,			//	Sub-states if RI_state==enum_RI_states::error
	// to be defined

	noRamp_divider,			//	Sub-states if RI_state==enum_RI_states::noRamp
	CornerNone,				//	No Corner discovered so far
	CornerLost,				//	Corner was detected but lost again: Signal to Noise ratio fell below 3.0

	detected_divider,		//	Sub-states if RI_state==enum_RI_states::detected
	// RIres_CornerFD1,		//	Corner discovery confidence level: >67%    = possible
	// RIres_CornerFD2,		//	Corner discovery confidence level: >95%    = likely
	CornerFD3,				//	Corner discovery confidence level: >99.8%  = very likely
	CornerFD4,				//	Corner discovery confidence level: >99.99% = almost beyond doubt
	CornerPast,				//	Corner took place in the Past, left of ObservWindow: uC_M < u1W

	unexpected_divider,		//	Sub-states if RI_state==enum_RI_states::unexpected.			Priority
	XiGTXBreak,				//	Signal greater than breaking condition						top priority
	unexpected_D,			//	Gradient D is out of expectations D_min < D < D_max			< XiGTXBreak
	unexpected_iC,			//	Corner Increment is outside expectations i1E < iC < iE		< unexpected_D
	as_expected,			//	everything goes according to expectations					lowest

	RIres_EndofStates		//	marks the end of this enum class
}enum_RI_substates;


// Funktion schreiben: RI in einem Datensatz mit 100 Inkrementen bei gegebenem R0. Statt update-Funktion.

//	-----------------------------------------------------------------------------------------------
//	RI_expectations
//	---------------
//	Struct containing expectations and requirements received by the caller.
typedef struct RI_expectations {
	// Process expectations: read-only to SR__RI
	int is_R0_GT_known = 0;		//	1: R_0,GT is known to the process and provided to SR_RI
	double R0_GT = 0;			//	the actual known Ground Truth of the Ramp Floor: R_0,GT
	int is_sig_GT_known = 0;	//	1: sigma_GT is known by the process and provided to SR_RI
	double sig_GT = 0;			//	the actual known Ground Truth of the Noise sig_GT
	int C_resolution = 0;		//	1: Process requires incremental resolution of C. Not recommended.
	SRidx i1S = 0;				//	i1S>0: When i >= i1S start Ramp Identification
								//	i1S==0: start Ramp Identification immediately
	SRidx iS = 0;				//	iS>0: When i > iS stop Ramp Identification
								//		If Ramp Identification needs to stop: Set iS = i-1
								//	iS=0: No stop condition on increments is set by the caller
	SRidx i1E = 0;				//	>0: first increment where Ramp Detection is expected to occur
	SRidx iE = 0;				//	>0: last increment where Ramp Detection is expected to occur
	double D_min = 0;			//	>0:	Minimum expected Ramp Gradients.
	double D_max = 0;			//	>0:	Maximum expected Ramp Gradients.
	int fd =3;					//	Confidence required: fd==3 > 99.73%, fd==4 > 99.99%
	double X_Break=0;			//	Breaking condition. If X(i)>X_Break, iS will be set equal to i.

	// Process expectations: write-only by SR__RI
	//	Ramp Detection Length: 
	//		>0: number of increments required to detect a Ramp 
	//			with smallest expected Gradient D_min
	//			at a given confidence level 
	//			for a given standard deviation sigma_GT of the signal
	//		==0: No Ramp Detection Length could be found.
	double L_RDm3 = 0;			//	Confidence level > 99.8%, signal to noise ratio is m=3
	double L_RDm5 = 0;			//	Confidence level > 99.99%, signal to noise ratio is m=5
}RI_expectations;

//	-----------------------------------------------------------------------------------------------
//	RI_results
//	----------
//	Struct containing all Ramp Identification results.
typedef struct RI_results {
	// State of the RI object
	enum_RI_states		RI_state;
	enum_RI_substates	RIsub_noRamp = enum_RI_substates::unknown;
	enum_RI_substates	RIsub_detected = enum_RI_substates::unknown;
	enum_RI_substates	RIsub_unexpected = enum_RI_substates::unknown;

	// Transformation from u- into i-coordinates: i[u] = iW + (u-uW)* di/du 
	SRidx diydu;				// Incremental step in i. Equals (delta(i)/delta(u))
	SRidx uW;					// Internal u-coodinate of the most recent data point: "u-noW"
	SRidx iW;					// Received i-coodinate of the most recent data point: "i-noW"

	// ML results for analysis
	double sig_ML_W = 0;		//	Standard deviation of ML in the Observation Window
	double sig_ML = 0;			// ML-filtered standard deviation from GT or Window
	double C_ML = 0;			// Corner position in u-coordinates
	double L_ML = 0;			// Length
	double D_ML = 0;			// Gradient
	double H_ML = 0;			// Height
	double R0_ML = 0;			// Floor level from GT or Window
	double R0_GT = 0;			// GT Floor level
	double F_D_ML = 0;			// Minimization Function value at the minimum
	// Q1: Ramp Detection confidence
	int Q1m3_ML = 0;			//	1: Ramp Detection with m=3. Confidence is > 99.8%
	int Q1m5_ML = 0;			//	1: Ramp Detection with m=5. Confidence is > 99.99%
	//////int Q1m3_ML = 0;			// Q1 criterion for m=3 using sig_ML
	//////int Q1m5_ML = 0;			// Q1 criterion for m=5 using sig_ML

	// L_aQt in u_coordinate scale
	double L_aQt_uRI = 0;		//	aQt'd Ramp Length >0: Bias and Noise are available.
	// Bias and Noise level. Signal X is scaled by sig_ML. Increments in u-coordinate scale.
	double L_aQt = 0;
	double mrs_L_uRI = 0;
	double srs_L_uRI = 0;
	double mrs_nsig_RI = 0;
	double srs_nsig_RI = 0;
	double mrs_nD_uRI = 0;
	double srs_nD_uRI = 0;
	double mrs_nH_RI = 0;
	double srs_nH_RI = 0;

	// L_aQt in i_coordinate scale
	double L_aQt_RI = 0;		//	aQt'd Ramp Length >0: Bias and Noise are available.
	// RI estimated Ramp Parameters 
	double sig_RI = 0;			//	Standard deviation: either by RI or Ground Truth by the caller
	double i_C_RI = 0;			//	Corner increment at subincremental resolution
	double L_RI = 0;			//	Length
	double D_RI = 0;			//	Gradient
	double H_RI = 0;			//	Height
	double R0_RI = 0;			//	Floor level: either by RI or Ground Truth by the caller
	// Uncertainties for Confidence > 99.73%
	double U_sig_fd3 = 0;		
	double U_L_fd3 = 0;
	double U_D_fd3 = 0;
	double U_H_fd3 = 0;
	// Uncertainties for Confidence > 99.99%
	double U_sig_fd4 = 0;
	double U_L_fd4 = 0;
	double U_D_fd4 = 0;
	double U_H_fd4 = 0;

} RI_results;

//	-----------------------------------------------------------------------------------------------
//	SR__RI
//	------
//	Class that implements Ramp Identification 
/*
	Description

	Source:	
		"Uncertainties of Ramp Identification in 100 Equidistant Noisy Data Points"
		by Sebastian Repetzki
		ResearchGate, 20Sep2023
		DOI

	Objectives:
		Provide iterative real-time Ramp Identification functionality for 
		one-dimensional sensor data with fixed cycle time and presumed white noise.

	Arrays bounds:
		Observation Window bounds are [u1W...uW] = [1...N].
		This is for compatibility with the documentation of RI algorithm.

		Incoming Sensor Data is stored in array with 2*N+2+1 fields.
		Index 0 and 2 are never used
		Data is stored twice: at index u and u+N. 
		When u reaches N+1 it is reset to 1.

					at init		at start			some input			Nth input = last before backcycling
			index	u			u	Window			u		Window		u		Window
			0		0
			1		
			2					u	1 <--										1 <--
			:							|											|
			a							|			u		1 <--					|
			a+1							|					2	|					|
			:							|						|					|
			N+1						N <--						|				N <--
			N+2					upN								|		u		1 <--
			:													|					|
			a+N-1											N <--					|
			a+N										u+N								|
			:																		|
			2*N+1														u+N		N <--
			2*N+2


	States:
		new

*/
class SR__RI
{
public:

	// States
	enum_RI_states		RI_state = enum_RI_states::created;
	enum_RI_substates	RIsub_noRamp = enum_RI_substates::unknown;
	enum_RI_substates	RIsub_detected = enum_RI_substates::unknown;
	enum_RI_substates	RIsub_unexpected = enum_RI_substates::unknown;

	// Expectations, received by the Caller and checked
	RI_expectations Expects;
	// Results to be returned to the Caller
	RI_results Results;

private:
	// Input Data:
	SRidx i_IN;			//	input increment number
	double X_IN;		//	actual input signal data
	//	u_IN	increment number for rotating arrays: 1...2*N
	//	w_IN	increment number in actual observation window

	// Input Data arrays for RI_update()
	//	These data arrays have size 2*N+pX+1.
	//	New data will be stored at array index u and upN=u+N simultaneously.
	//	The index u increments each time, new data bas been stored. Reset when u==N is reached.
	//	The observation Window that contains the N newest data can be found from index u to u+N-1.
	SRidx u = 0;		//	array index for new input data to be stored
	SRidx upN = 0;		//	u + N
	SRidx diydu = 0;	//	scale di/du

	SRidx* _i_2N 		//	Increment number array
		= new SRidx[RI_2NpXp1];
	double* _x_2N 		//	Sensor data array
		= new double[RI_2NpXp1];
	double* _s_2N 		//	Path data array;
		= new double[RI_2NpXp1];

	// Window increment boundaries
	newSRidx(u1W);
	newSRidx(uW);		//	u1W...uW contains N increments. uW = upN is the newest increment.
	double dbl_u1W = 0, dbl_uW = 0;	// same as double

	// Ramp parameters
	double C = 0;		//	Corner position
	double D = 0;		//	Ramp gradient

	// Sums over Signal data in the Observation Window
	double* _x2			//	x squared
		= new double[RI_2NpXp1];
	double _sum_x2_uW=0;//	Sum of all x² over all increments in the Window. Not an array.
	double x2_u1Wm1 = 0;//	Oldest x² in the actual _sum_x2. To be subtracted when updating.
	double* _xu 		//	x*u
		= new double[RI_2NpXp1];
	double* _sumR_x_C 	//	Sum of (x) over Ramp increments for Corner candidates Ct=1...N
		= new double[RI_2NpXp1];
	double* _sumR_xu_C 	//	Sum of (x*u) over Ramp increments for Corner candidates Ct=1...N
		= new double[RI_2NpXp1];

	// Data for goal function evaluation. Use locally.
	SRidx N = RI_N;		//	Number of increments to be analysed
	newSRidx(intC);		//	integer value of C, floor(C)
	newSRidx(intCp1);	//	intC + 1. Increments intCp1...iW represent the Ramp

	// Calculation of the ML-filtered standard deviation
	double *_MLnoise	//	Sensor data minus the identified Ramp.
		= new double[RI_2NpXp1];

	// Coefficients for linear regression: the Window has already passed the Corner
	double F_0_ = 0, F_D_ = 0, F_D2_ = 0, F_CD_ = 0, F_CD2_ = 0, F_C2D2_ = 0;
	double C_linR = 0, D_linR = 0, F_linR = 0;	// zero crossing, gradient, root(sum errors²)

	// Minimize over gradient D for fixed Corner position at integer values 
	//	Gradient at minimum _D_M_4uC[ uC+1] for C fixed at integer uC
	double* _D_M_4uC 
		= new double[RI_2NpXp1];

	//	Goal function minimum _M_D_4uC[ uC+1] over D for all integer uC in u1W...uW
	double* _M_D_4uC 
		= new double[RI_2NpXp1];

	//	Ranks of M_D by ascending order and ranked M_D values
	int Nranks = 10;		// Number of smallest M_D to be found. Nranks<<N for higher speed.
	SRidx* _u_RankBy_M_D 	// array of indices u by ranking order: _Ranks_M_D[u1W] hat smallest M_D
		= new SRidx[RI_2NpXp1];
	double* _M_D_ranked		// M_D's by ranking order _M_D_ranked[u1W] is smallest in ObservWindow
		= new double[RI_2NpXp1];
	newSRidx(N_I);			// Number of intervals found
	SRidx* _Ranks_I			// array indices by ranking of Intervals: _Rank_I[u1W] and the next form an Interval
		= new SRidx[RI_2NpXp1];
	newSRidx(N_sIM);		// Number of sub-incremental minima found
	newSRidx(u_sIM);		//	Index of smalles sub-incremental minimum
	double* _C_Mu			//	Sub-incremental minima: Corner position
		= new double[RI_Ncand];
	double* _D_Mu			//	Sub-incremental minima: Gradient
		= new double[RI_Ncand];
	double* _M_CMu			//	Sub-incremental minima: Goal function value
		= new double[RI_Ncand];

	// Q1 Ramp Detection Signal to Noise maultiplier for different confidence levels
	double c0_Q1m_FD3 = 3.0;	//	99.8% confidence level
	double c0_Q1m_FD4 = 5.0;	//	99.99% confidence level

	// RI-estimation formula coefficients: Q1 Ramp Detection
	//	Coefficients L_RD = ceil( a0_RD/d_GT + c0_RD)
	//	 for m=3 (confidence>99.8%)  and m=5  (confidence>99.99%)
	double a0_RDm3 = 4;
	double c0_RDm3 = 2;
	double a0_RDm5 = 6.5;
	double c0_RDm5 = 1.5;

	// RI-estimation formula coefficients: Q2a Bias and Noise Modelling
	//	Coefficients L_aQt = a0_aQt/d_ML + c0_aQt
	double a0_aQt = 4.4;
	double c0_aQt = 2.5;

	// RI-estimation formula coefficients: Q2b Bias and Noise Models
	// ML-estimated standard deviation sig_ML
	//	Coefficients mrs_nsig_RI = c0_mrs_nsig + c1_mrs_nsig * L_ML
	double c0_mrs_nsig = -0.0186;
	double c1_mrs_nsig = 0.0000657;
	//	Coefficients srs_nsig_RI = c0_srs_nsig + c1_srs_nsig * L_ML
	double c0_srs_nsig = 0.0731;
	double c1_srs_nsig = -0.00000557;

	// ML-estimated Ramp Length L_ML
	//  Coefficients mrs_L_RI = c0_mrs_L
	double c0_mrs_L = 0.0;
	//  Coefficients srs_L_RI = c0_srsL * L_ML^a1_srsL * d_ML^a2_srsL
	double c0_srs_L = 2.062;
	double a1_srs_L = -0.5058;
	double a2_srs_L = -1.017;

	// ML-estimated Ramp Gradient D_ML
	//	Coefficients mrs_nD_RI = c0_mrs_nD * L_ML^a0_mrs_nD 
	double c0_mrs_nD = 2.0;
	double a0_mrs_nD = -2.4;
	//	Coefficients srs_nD_RI = c0_srs_nD * L_ML^a0_srs_nD 
	double c0_srs_nD = 2.8;
	double a0_srs_nD = -1.45;

	// ML-estimated Ramp Height H_ML
	//	Coefficients mrs_nH_RI = c0_mrs_nH * L_ML^a0_mrs_nH
	double c0_mrs_nH = 0.6;
	double a0_mrs_nH = -0.8;
	//	Coefficients srs_nH_RI = c0_srs_nH * L_ML^a0_srs_nH
	double c0_srs_nH = 1.6;
	double a0_srs_nH = -0.445;

	// R0 = unknown: coefficients -----------------------------------------------------------------
	//	Coefficients mrs_nsig_RI = c0_mrs_nsig + c1_mrs_nsig * L_ML if R0 is unknown
	double c0_mrs_nsig_R0ukn = -0.0186;
	double c1_mrs_nsig_R0ukn = 0.0000657;
	//	Coefficients srs_nsig_RI = c0_srs_nsig + c1_srs_nsig * L_ML if R0 is unknown
	double c0_srs_nsig_R0ukn = 0.0731;
	double c1_srs_nsig_R0ukn = +0.00000557;

	//  Coefficients mrs_L_RI = c0_mrs_L if R0 is unknown
	double c0_mrs_L_R0ukn = 0.0;
	//  Coefficients srs_L_RI = c0_srsL * L_ML^a1_srsL * d_ML^a2_srsL if R0 is unknown
	double c0_srs_L_R0ukn = 1.89;
	double a1_srs_L_R0ukn = -0.466;
	double a2_srs_L_R0ukn = -1.02;

	// ML-estimated Ramp Gradient D_ML
	//	Coefficients mrs_nD_RI = c0_mrs_nD * L_ML^a0_mrs_nD  if R0 is unknown
	double c0_mrs_nD_R0ukn = 2.0;
	double a0_mrs_nD_R0ukn = -2.4;
	//	Coefficients srs_nD_RI = c0_srs_nD * L_ML^a0_srs_nD  if R0 is unknown
	double c0_srs_nD_R0ukn = 2.8;
	double a0_srs_nD_R0ukn = -1.45;

	// ML-estimated Ramp Height H_ML
	//	Coefficients mrs_nH_RI = c0_mrs_nH * L_ML^a0_mrs_nH  if R0 is unknown
	double c0_mrs_nH_R0ukn = 0.6;
	double a0_mrs_nH_R0ukn = -0.8;
	//	Coefficients srs_nH_RI = c0_srs_nH * L_ML^a0_srs_nH  if R0 is unknown
	double c0_srs_nH_R0ukn = 1.47;
	double a0_srs_nH_R0ukn = -0.406;


	// Noise-multiplier for Uncertainties gauged to Confidence levels - case: GT_none -------------
	//	GT_none:	Floor level R_0 is not given by caller.
	//				Signal Noise sig_GT is not given by caller.
	//				Aka: R_0 unknown
	//	Confidence >99.73%, "fd3"
	double m_sig_fd3_GT_none = 3.4;
	double m_L_fd3_GT_none = 3.4;
	double m_D_fd3_GT_none = 3.4;
	double m_H_fd3_GT_none = 3.2;
	//	Confidence >99.99%, "fd4"
	double m_sig_fd4_GT_none = 5.0;
	double m_L_fd4_GT_none = 4.8;
	double m_D_fd4_GT_none = 5.0;
	double m_H_fd4_GT_none = 4.3;

	// Noise-multiplier for Uncertainties gauged to Confidence levels - case: GT_sig -----------
	//	GT_sig:		Floor level R_0 is not given by caller.
	//				Signal Noise sig_GT is given by caller.
	//				Aka: sig_GT known
	//	Confidence >99.73%, "fd3"
	double m_sig_fd3_GT_sig = 0.0;
	double m_L_fd3_GT_sig = 3.4;
	double m_D_fd3_GT_sig = 3.4;
	double m_H_fd3_GT_sig = 3.2;
	//	Confidence >99.99%, "fd4"
	double m_sig_fd4_GT_sig = 0.0;
	double m_L_fd4_GT_sig = 4.8;
	double m_D_fd4_GT_sig = 5.0;
	double m_H_fd4_GT_sig = 4.3;

	// Noise-multiplier for Uncertainties gauged to Confidence levels - GT_R0 ---------------------
	//	GT_R0:		Floor level R_0 is given by caller.
	//				Signal Noise sig_GT is not given by caller.
	//				Aka: the Reference case.
	//	Confidence >99.73%, "fd3"
	double m_sig_fd3_GT_R0 = 3.4;
	double m_L_fd3_GT_R0 = 3.4;
	double m_D_fd3_GT_R0 = 3.4;
	double m_H_fd3_GT_R0 = 3.2;
	//	Confidence >99.99%, "fd4"
	double m_sig_fd4_GT_R0 = 5.0;
	double m_L_fd4_GT_R0 = 5.0;
	double m_D_fd4_GT_R0 = 5.0;
	double m_H_fd4_GT_R0 = 4.2;

	// Noise-multiplier for Uncertainties gauged to Confidence levels - case: GT_R0_sig -----------
	//	GT_R0_sig:	Floor level R_0 is given by caller.
	//				Signal Noise sig_GT is given by caller.
	//				Aka: sig_GT known
	//	Confidence >99.73%, "fd3"
	double m_sig_fd3_GT_R0_sig = 0.0;
	double m_L_fd3_GT_R0_sig = 3.3;
	double m_D_fd3_GT_R0_sig = 3.3;
	double m_H_fd3_GT_R0_sig = 3.1;
	//	Confidence >99.99%, "fd4"
	double m_sig_fd4_GT_R0_sig = 0.0;
	double m_L_fd4_GT_R0_sig = 4.8;
	double m_D_fd4_GT_R0_sig = 4.7;
	double m_H_fd4_GT_R0_sig = 4.1;

	// Minimum of the goal function
	int type_M = 0;			//	Type of the global minimum: 
							//		0	at increment border
							//		1	sub-incremental minimum
							//		2	from linear regression

	// ML Results:
	//	Standard deviations
	double sig_ML = 0;		//	ML-filtered standard deviation: equals sig_GT or sig_ML_W
	double sig_ML_W = 0;	//	ML-filtered standard deviation from the Observation Window
	// ML results in u-coordinates
	double C_ML = 0;		//	Corner position in u-coordinates
	double R0_ML = 0;		//	Floor level
	double F_ML = 0;		//	Goal function
	double L_ML = 0;		//	Ramp Length
	double D_ML = 0;		//	Gradient dX/du
	double H_ML = 0;		//	Ramp Height
	double R_ML = 0;		//	Ramp Signal estimation

	double S2N_ML = 0;		// Signal to Noise ratio with sig_ML as Noise measure
	int Q1m3_ML = 0;		//	Q1: Ramp detection criterion normed by sig_ML: L_ML*D_ML/sig_ML >? m=3	
	int Q1m5_ML = 0;		//	Q1: Ramp detection criterion normed by sig_ML: L_ML*D_ML/sig_ML >? m=5

	// aQt Length and Bias and Noise. Signal level scaled by sig_ML. Increments in u-coordinates
	double L_aQt = 0;
	double mrs_L_uRI = 0;
	double srs_L_uRI = 0;
	double mrs_nsig_RI = 0;
	double srs_nsig_RI = 0;
	double mrs_nD_uRI = 0;
	double srs_nD_uRI = 0;
	double mrs_nH_RI = 0;
	double srs_nH_RI = 0;

	// Bias and Noise at original Signal Scale
	//	Length: no sig_ML scaling necessary. Results are mrs_L_uRI and srs_L_uRI.
	double mrs_sig_RI = 0.0;
	double srs_sig_RI = 0.0;
	double mrs_D_uRI = 0.0;
	double srs_D_uRI = 0.0;
	double mrs_H_RI = 0.0;
	double srs_H_RI = 0.0;

	// Noise-multipliers
	//	Confidence >99.73%, "fd3"
	double m_sig_fd3 = 0.0;
	double m_L_fd3 = 0.0;
	double m_D_fd3 = 0.0;
	double m_H_fd3 = 0.0;
	//	Confidence >99.99%, "fd4"
	double m_sig_fd4 = 0.0;
	double m_L_fd4 = 0.0;
	double m_D_fd4 = 0.0;
	double m_H_fd4 = 0.0;

public:
	SR__RI();
	~SR__RI();
	enum_RI_states RI_set(RI_expectations* _user_Expects);
	enum_RI_states RI_reset(RI_expectations* _user_Expects);
	enum_RI_states RI_update(SRidx i, double x, RI_results* _user_Results);

private: 
	int RI_reset();				//// old
	int RI_reset_all();			//// old
	int RI_reset_2known();		//// old
	int RI_set_known(int sig_is_known, double req_sig, int R0_is_known, double req_R0, int req_C_resolution);		//// old
	int RI_sums_of_x();
	double RI_FCD(double C, double D);
	int RI_M_D();
	int rank_M_D_et_I();
	int find_sub_M_D();
	int find_M_C_D();
	int make_RI_results();
	int min3pts(double* _u_M, double* _v_M, double u_I, double u_III, double u_V, double v_I, double v_III, double v_V);
	double fast_mean(double* _x, SRidx i1, SRidx iZ);
	double fast_stdev_S(double* _x, SRidx i1, SRidx iZ);
	double fast_stdev_N(double* _x, SRidx i1, SRidx iZ);
};
