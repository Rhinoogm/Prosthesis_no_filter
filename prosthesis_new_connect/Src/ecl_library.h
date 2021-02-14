#include "stm32f3xx.h"
#define PI 3.141592653589793

//#define USE_DOUBLE
//--------------------------------------------------------------
// double형 data를 쓸 것인지, float 형 data를 쓸 것인지
// default는 float를 쓰는 것이지만 만약 double형 data를 쓰고자
// 한다면 Project->options->C/C++ compiler->Preprocessor에 가서
// defined symbols 란에 USE_DOUBLE을 추가하면 된다.
//--------------------------------------------------------------
#ifdef USE_DOUBLE
  #define real_T double
#else
  #define real_T float
#endif 


#define	BIT_MASK(x)			(1 << (x))
#define	ClearBit(val, bit)	((val) &= ~BIT_MASK(bit))
#define	ToggleBit(val, bit)	((val) ^=BIT_MASK(bit))
#define GetBit(val, bit)	(((val) & BIT_MASK(bit))>>(bit))
#define	SetBit(val, bit)	((val) |= BIT_MASK(bit))

#define SS_DOUBLE  0   //    64-bit double 형 data
#define SS_SINGLE  1   //    32-bit float 형 data
#define SS_INT8    2   // int8_T
#define SS_UINT8   3   // uint8_T
#define SS_INT16   4   // int16_T
#define SS_UINT16  5   // uint16_T
#define SS_INT32   6   // int32_T
#define SS_UINT32  7   // uint32_T
#define SS_BOOLEAN 8   // boolean_T

#define MAX_NUM_OF_TO_HOST_BLOCK 16
typedef struct {
    unsigned int ModNumber;
    unsigned int ModCntr;
    unsigned int NumOfToHostBlock;
    unsigned int NumOfDoubleData; 
    unsigned int UpdateDoneFlag;
    int SendHoldFlag; 
    char TypeAndChannel[MAX_NUM_OF_TO_HOST_BLOCK];
    void * DataPtr[MAX_NUM_OF_TO_HOST_BLOCK];
    uint8_t data[512];
    unsigned long indx;	
} ToHost;


typedef unsigned long u32;
typedef signed long s32;
typedef short int s16;
typedef unsigned short int u16;

//----------------------------
// S2D (S to D) block
//----------------------------
typedef struct {
  real_T input; // x input
  real_T X0;
  real_T X1;
  real_T X2;
  real_T Y0;
  real_T Y1;
  real_T Y2;
  real_T a0; 	// Parameter : (den) * s^2
  real_T a1; 	// Parameter : (den) * s^1
  real_T a2; 	// Parameter : (den) * 1
  real_T b0;  	// Parameter : (num) * s^2
  real_T b1;	// Parameter : (num) * s^1
  real_T b2;  	// Parameter : (num) * 1
  real_T K;  	// Parameter : K = T/2
  real_T x; 	// Input
  real_T x_p; 	// Input_past
  real_T x_pp;  // Input_past_past
  real_T y; 	// Output
  real_T y_p; 	// Output_past
  real_T y_pp;  // Output_past_past
  real_T T;  	// Sampling time
  void (*calc)(); // Pointer to LPF calculation function
  void (*calc_param)(); // Pointer to parameter calculation function
} S2D;

#define S2D_DEFAULTS { 0,    	/* x input */ 					\
					   0,										\
					   0,										\
					   0,										\
					   0,										\
					   0,										\
					   0,										\
					   0,    	/* Parameter : (den) * s^2 */ 	\
                       0,   	/* Parameter : (den) * s^1 */ 	\
                       0,  		/* Parameter : (den) * 1 */ 	\
                       0,   	/* Parameter : (num) * s^2 */ 	\
                       0,    	/* Parameter : (num) * s^1 */ 	\
                       0,    	/* Parameter : (num) * 1 */ 	\
                       0,    	/* Parameter : K = T/2 */ 		\
					   0,	  	/* Input */ 					\
					   0,    	/* Input_past */ 				\
					   0,    	/* Input_past_past */ 			\
					   0,    	/* Output */ 					\
					   0,    	/* Output_past */ 				\
					   0,	  	/* Output_past_past */ 			\
					   0,    	/* Sampling time */ 			\
                       (void (*)(u32))S2D_calc,  		/* Function pointer */ 		\
                       (void (*)(u32))S2D_calc_param 	/* Function pointer */  	\
}


//-------------------------------
// PID Control Block
//-------------------------------
typedef struct {
	float Ref; // input : Reference input
	float Fdb; // input : Feedback input
	float Err; // variable : Error
	float Err_past; // variable : Error_past
	float Ts;  // Parameter : Sampling time
	float Kp;  // Parameter : Proportional gain
	float Ki;  // Parameter : Integral gain
	float Kd;  // Parameter : Derivative gain
	float OutMax; // Parameter : Maximum output
	float OutMin; // Parameter : Minimum output
	float Intg;   // Variable : Integral value
	float Deriv;  // Variable : Derivative value
	float Out_tmp;   // Output : PID output
	float Out;    // Output : PID output after the saturation block
	float SatErr; // Variable : Saturated difference
	float Ka;   // Back calculation coefficient
	float N;    // Filter coefficient
	void (*calc)(); // Pointer to calculation function
} PID_CON;


#define PID_CON_DEFAULTS { 0,   /* Ref */  \
                           0,   /* Fdb */  \
                           0,   /* Err */  \
						   0,   /* Err_past */  \
                           0.004,  /* Ts */ \
                           0,   /* Kp */  \
                           0,   /* Ki */  \
                           0,   /* Kd */  \
                           0,  /* OutMax */ \
						   0,  /* OutMin */ \
                           0,   /* Intg */ \
                           0,   /* Deriv */ \
                           0,   /* Out_tmp */ \
                           0,   /* Out */\
                           0,   /* SatErr */ \
                           0,   /* Ka */ \
                           0,   /* N */ \
                          (void (*)(u32))pid_con_calc  /* Function pointer */ \
}


//----------------------------
// LPF(Low Pass Filter) block
//----------------------------
typedef struct {
  real_T k1; // Parameter
  real_T k2; // Parameter
  real_T Ts; // Parameter : Sampling time 
  real_T In;  // Input 
  real_T Out; // Output
  real_T State;  // State
  real_T Fc;  // Cutoff frequency
  void (*calc)(); // Pointer to LPF calculation function
  void (*calc_param)(); // Pointer to parameter calculation function
} LPF;

#define LPF_DEFAULTS { 0,    /* k1 */ \
                       0,    /* k2 */ \
                       0.004,  /* Ts */ \
                       0,    /* In */ \
                       0,    /* Out */ \
                       0,    /* State */ \
                       0,    /* Fc */ \
                       (void (*)(u32))LPF_calc,  /* Function pointer */ \
                       (void (*)(u32))LPF_calc_param /* Function pointer */  \
}

//-----------------------------
// Encoder Interface block
//-----------------------------
typedef struct {
  s32 Nc ; // Input : EncoderCounterCurrent
  s32 diff;  // variable
  real_T Ts; // sample time
  u16 n_bits; // parameter : Encoder counter bit number
  s32 h_rng; // parameter : half_range
  s32 f_rng; // parameter : full_range;
  s32  Na; // Variable : EncoderCountAccumulated
  s32  Np; // Variable : EncoderCountPast;
  u16 PPR;  // parameter
  real_T scale;  // parameter : scaling factor
  real_T theta; // Output : angle
  real_T dot_theta; // Output : angular velocity
  real_T dot_theta_past;
  real_T dotdot_theta;
  void (*calc)();  // Pointer to Encoder calculation function
  void (*calc_param)(); // Pointer to parameter calculation function
//s32 Nc ; // Input : EncoderCounterCurrent
//s32 diff;  // variable
//real_T Ts; // sample time
//u16 n_bits; // parameter : Encoder counter bit number
//s32 h_rng; // parameter : half_range
//s32 f_rng; // parameter : full_range;
//s32  Na; // Variable : EncoderCountAccumulated
//s32  Np; // Variable : EncoderCountPast;
//u16 PPR;  // parameter
//real_T scale;  // parameter : scaling factor
//real_T theta; // Output : angle
//real_T dot_theta; // Output : angular velocity
//void (*calc)();  // Pointer to Encoder calculation function
//void (*calc_param)(); // Pointer to parameter calculation function
} ENCODER;   

#define ENCODER_DEFAULTS {\
  0,  /* Nc */ \
  0,  /* diff */ \
  0.004, /* Ts */ \
  16, /* n_bits */ \
  0,  /* h_rng  */ \
  0,  /* f_rng */ \
  0,  /* Na */ \
  0,  /* Np */ \
  0,  /* PPR */ \
  0,  /* scale */ \
  0.0, /* theta */ \
  0.0, /* dot_theta */ \
  0.0, /* dot_theta_past */ \
  0.0, /* dotdot_theta */ \
  (void (*)(u32))ENCODER_calc, /* Function pointer */ \
  (void (*)(u32))ENCODER_calc_param, /* Function pointer */ \
}

typedef S2D *S2D_handle;
typedef PID_CON *PID_CON_handle;
typedef LPF *LPF_handle;
typedef ENCODER *ENCODER_handle;

void S2D_calc(S2D_handle);
void S2D_calc_param(S2D_handle);
void pid_con_calc(PID_CON_handle);
void LPF_calc(LPF_handle);
void LPF_calc_param(LPF_handle);
void ENCODER_calc(ENCODER_handle);
void ENCODER_calc_param(ENCODER_handle);


void connectData(ToHost *toHostVar, char type, char channel, void *ptr);
void ProcessToHostBlock();

