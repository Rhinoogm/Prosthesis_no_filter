/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<math.h>
#include<string.h>
#include "ecl_library.h"
#include "vector_matrix.h"

#define pi 3.14159265
#define ADC_ch1 0x00000040
#define ADC_ch2 0x00000080
#define DUTY_LIMIT 60
#define RATED_CURRENT 5.06
#define Fout_max 897

#define g  9.8
#define Nm 9425

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern ToHost toSimulink;

/* ADC */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Load cell initial setting */
real_T ADC_load_zero = 1960;//2145.7;	// load cell (x1)

/* FSR initial setting */
real_T ADC_Heel_zero 	= 1972;   //2009;
real_T ADC_Toe_zero 	= 2005;   //2022;

real_T ADC_Heel_ON = 600;
real_T ADC_Heel_OFF = 50;		//36.7

real_T ADC_Toe_ON = 100;
real_T ADC_Toe_OFF = 30;		//43.0

/////////////////////////////////////////////////
__IO uint16_t ADC_array_1;
__IO uint16_t ADC_array_2;
uint32_t ADC_channel[3] = {0, ADC_ch1, ADC_ch2};

/////////////////////////////////////////////////
real_T CURRENT_val = 0;
real_T LOAD_val = 0;
real_T LOAD_compare;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* SPI : Absolute encoder  */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t null[1]={0, };
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*Encoder*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* motor encoder : pin interrupt */
/////////////////////////////////////////////////
uint16_t count_1;
uint16_t count_2;
uint16_t count_3;
uint16_t count_4;

uint16_t dir;
uint16_t dir_2;
uint16_t dir_3;
uint16_t dir_4;

/* Absolute Encoder */
real_T angle_zero = 349;  	//	730;
real_T angle_ninety = 1585; // 	1166;             /// 0.0947
							//	117.66 = 1906     /// 0.0788
real_T angle_org =0 ;
real_T angle =0;
real_T angle_rad=0;
real_T angle_LPF = 0;

/* Motor Encoder */
real_T m_enc=0;
real_T thetam=0;
real_T thetam_rad=0;
real_T thetam_dot = 0;
real_T thetam_dotdot = 0;

/* Spring Encoder */
real_T S_enc=0;
real_T thetaS=0;
real_T thetaS_rad=0;
real_T thetaS_dot = 0;
real_T thetaS_dotdot = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* spring force*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
real_T xs = 0;
real_T Ks = 194489;
real_T F_spring = 0;
real_T F_spring_lpf = 0;
real_T F_spring_lpf1 = 0;
real_T F_spring_lpf2 = 0;
real_T T_motor = 0; // motor torque
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* actuator */
real_T xl = 0;
real_T xl_compare = 0;
real_T xl0 = 0 ;
real_T xl_lpf = 0;
/* time variable */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t start_time;
real_T dt    = 0.004;
real_T time1 = 0;
real_T time2 = 0;
real_T time=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* LPF */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int8_t flag_start = 0; // lpf state 받을 때 사용

/* Motor LPF */
real_T Motor_dot_lpf = 0;
real_T Motor_dotdot_lpf = 0;

/* Spring LPF */
real_T Spring_dot_lpf = 0;
real_T Spring_dotdot_lpf = 0;

/* LPF 결과 값 : force, heel, toe의 original */
real_T Force_org;
real_T Heel_org;
real_T Toe_org;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* controller input */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
real_T FF_INPUT = 0;
real_T CURRENT_INPUT = 0;
real_T CURRENT_INPUT1 = 0;
real_T DUTY_INPUT = 0;
real_T pwm_start = 99; // pwm trigger
real_T force_input = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* motor kalman */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double Jm = 7e-6;
double Bm = 1.8e-4;
double delta = 0;
double delta_test = 0;

double sig[4] = {1000,0,0,1000};

double R[4] = {0,0,0,0};
double K[4] = {0,0,0,0};
double u[2] = {0, 0};
double sig_hat[4] = {0,0,0,0};

double theta_hat = 0;
double theta_dot_hat = 0;

double M1[4] 	= {0,0,0,0};
double M11[4] 	= {0,0,0,0};
double M2[4] 	= {0,0,0,0};
double M22[4] 	= {0,0,0,0};
double M3[4] 	= {0,0,0,0};
double M33[4] 	= {0,0,0,0};
double M4[4] 	= {0,0,0,0};
double M44[4] 	= {0,0,0,0};
double M5[4] 	= {0,0,0,0};
double M55[4] 	= {0,0,0,0};
double M6[4] 	= {0,0,0,0};
double M66[4] 	= {0,0,0,0};


double G_T[4] 	= {0,0,0,0};
double V_T[4] 	= {0,0,0,0};
double H_T[4] 	= {0,0,0,0};

double M44_I[4] = {0,0,0,0};

real_T thetam_rad_kalman = 0;
real_T thetam_rad_speed_kalman = 0;

//double I[2][2] = {{1, 0}, {0, 1}};
double I[4] = {1.0, 0.0, 0.0, 1.0};

//Jacobian
double G[4] = {1.0, 0.0, 0.004, 1.0-((1.8e-4)/(7e-6))*0.004};
double V[4] = {0.0, 0.004/(7e-6), 0.0, 0.004/(7e-6)/9425};

//G[0] = {1.0, 0.0, dt, 1.0 - (Bm/Jm)*dt};
//V = {0.0, dt/Jm, 0.0, dt/Jm/Nm};

//measurement
//double H[2][2] = {{1, 0}, {0, 1}};
double H[4] = {1.0, 0.0, 0.0, 1.0};

//측정오차에 대한 공분산
double Q[4] = {1., 0.0, 0.0, 10.};

//control value에 대한 uncertainty를 나타냄
double sig_con[4] = {10.0, 0.0, 0.0, 1000.0};

//double sig_hat[2][2] = {{0, 0}, {0, 0}};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* spring kalman */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double R_S[4] = {0,0,0,0};

double S1[4] = {0,0,0,0};
double S11[4] = {0,0,0,0};
double S2[2] = {0,0};
double S22[4] = {0,0,0,0};
double S3[4] = {0,0,0,0};
double S33[4] = {0,0,0,0};
double S4[4] = {0,0,0,0};
double S44[4] = {0,0,0,0};
double S5[4] = {0,0,0,0};
double S55[4] = {0,0,0,0};
double S6[4] = {0,0,0,0};
double S66[4] = {0,0,0,0};


double G_T_S[4] = {0,0,0,0};
double V_T_S[2] = {0,0};
double H_T_S[4] = {0,0,0,0};

double S44_I[4] = {0,0,0,0};

double Ms = 1.6;
double Bs = 250;
double delta_S = 0;

double K_S[4] = {0,0,0,0};
double u_S[2] = {0, 0};
double sig_hat_S[4] = {0,0,0,0};

double theta_hat_S = 0;
double theta_dot_hat_S = 0;

real_T thetaS_rad_kalman = 0;
real_T thetaS_rad_speed_kalman = 0;

//real_T F_ref = 0;

real_T DOB_input_1 = 0;
real_T DOB_input_2 = 0;

//Jacobian
//double G[2][2] = {{1, 0.004}, {0, 1-(1.8*(10^(-4))/7*(10^(-6)))*0.004}};
//double V[2][2] = {{0, 0}, {0.004/7*(10^(-6)), 0.004/7*(10^(-6))/9425}};
double G_S[4] = {1.0, 0.004 * (-194489/1.6), 0.004, 1.0-(250/1.6)*0.004};
double V_S[2] = {0.0, (1.0/1.6)*0.004};

//G[0] = {1.0, 0.0, dt, 1.0 - (Bm/Jm)*dt};
//V = {0.0, dt/Jm, 0.0, dt/Jm/Nm};

//measurement
//double H[2][2] = {{1, 0}, {0, 1}};
double H_S[4] = {1.0, 0.0, 0.0, 1.0};

//측정오차에 대한 공분산
//double Q[2][2] = {{0.0001,0}, {0,0.1}};
//double Q[4] = {0.0001, 0.0, 0.0, 0.1};
double Q_S[4] = {0.001, 0.0, 0.0, 0.1};

//control value에 대한 uncertainty를 나타냄
//double sig_con[2][2] = {{10, 0}, {0, 1000}};
//double sig_con[4] = {10.0, 0.0, 0.0, 1000.0};
double sig_con_S[1] = {1000.0};

double sig_S[4] = {100,0,0,100};
//double sig_hat[2][2] = {{0, 0}, {0, 0}};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* recursive least squares */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double P[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
double h[3] = {0, 0, 0};
double Gamma = 0;


double x_measurement[3] = {0, 0, 0};
double x_measurement_T[3] = {0, 0, 0};
double K_rls[3] = {0, 0, 0};
double K_num[3] = {0, 0, 0};
double K_den[1] = {0};

double P_1[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double P_2[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

double theta_k_dot;
double theta_k_dotdot;
double r;

double Ja = 0;
double Ba = 0;
double m_estimated = 0;


double y_temp = 0;
double e_temp = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






/* locomotion variable */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float HeelState=0;
float ToeState=0;
/////////////////////////////////////////////////
int stateNum=4;
char state='N';
/////////////////////////////////////////////////
real_T time_ex;
real_T thetak_old;
real_T thetak_0;
/////////////////////////////////////////////////
////////////////////////////////initial value//////////////////////
real_T initial_flag=0;
real_T start_flag = 0;
/////////////////////////////////parameter//////////////////////////
////////////////////////////////impedance parameter/////////////////
////////////////////////////////Speed 2.2Km/h///////////////////////
////////////////////////////////Stance//////////////////////////////
// Stance
float k0 = 5.0;					// linear stiffness
float b0 = 0.1;				   // damping ratio
float angle_eq0  = 15;		   // equilibrium point

// Pre Swing
float k1 = 1.0;	//0.4				// linear stiffness
float b1 = 0.05;			        // damping ratio
float angle_eq1  = 25;//35			// equilibrium point

// Swing Flexion
float k2 = 0.3;	//0.3			// linear stiffness
float B2 = 0.01;				// damping ratio
float angle_eq2  = 55;	//65		// equilibrium point

// Swing Extension
float k3 = 0.1;//0.1				// linear stiffness
float b3 = 0.007; //0.01;	 	    // damping ratio
float angle_eq3  = 15;			// equilibrium point

//real_T L = 0.002;
//real_T L1 = 0.2742609;
//real_T L12 = 0.075219;
//real_T L2 = 0.0757042;
//real_T L22 =  0.0057311;
//real_T _2L12 = 0.0415254;
//real_T alpha0= 0.8868;				단위가 조금 다름 /// radian // meter 인듯

real_T L2 		= 0.0993448;
real_T L1 		= 0.3033562; // m

real_T alpha_0	= 0.4789;	// radian
real_T alpha	= 0;		// radian

real_T T_k = 0; //knee torque
real_T T_k_lpf = 0; //knee torque

real_T theta_k = 0;
real_T theta_k_rad = 0;
real_T theta_compare = 0;

/////////////////////////////////////////////////
real_T force=0;
real_T forceSum=0;
real_T forceAvr=0;
real_T forceOrg =0;

real_T torque=0;
real_T torque_kalman = 0;
real_T Heel;
real_T Toe;

real_T x0 = 0;
real_T x=0;
real_T x_ref=0;
real_T x_diff;
/////////////////////////////////////////////////
real_T thetak=0;
real_T thetak_rad=0;
real_T thetak_past=0;
real_T thetak_velocity=0;
real_T thetak_velocity_past=0;

real_T thetak_ref=0;
real_T thetak_ref_rad=0;

real_T thetak_lpf_rad;
real_T thetak_lpf;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

real_T thetam_ref=0;
real_T thetam_ref_rad=0;



/* test1~5 : 사용 안 함 ///// sig_1~3 : 사용 중 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float teat_block[4] = {10^(-4),10e-04,3,4};

float test1 = 0;
float test2 = 0;
float test3 = 0;
float test4 = 0;
float test5 = 0;

real_T sig_1 = 0;
real_T sig_2 = 0;
real_T sig_3 = 0;

int cnt = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double AA[4] = {11, 21, 12, 22};
double BB[4] = {1, 3, 2, 4};
double CC[4] = {0, 0, 0, 0};


void Process_Init();
void Initialize();
void Process_start();
void Process_stop();

void SPI_init();
void PWM_init();
void ENC_init();
void LPF_init();

void Kalman_init();
void Kalman_init_S();
void RLS();

void motor_generation();

void Start();

//void Start();

/* CLASS DECLARE */
	LPF Pos_LPF = LPF_DEFAULTS;
	LPF Ref_LPF = LPF_DEFAULTS;
	LPF Force_LPF = LPF_DEFAULTS;
	LPF Torque_LPF = LPF_DEFAULTS;
	LPF Heel_LPF = LPF_DEFAULTS;
	LPF Toe_LPF = LPF_DEFAULTS;
	LPF Ang_LPF = LPF_DEFAULTS;

	LPF Motor_dot_LPF = LPF_DEFAULTS;
	LPF Spring_dot_LPF = LPF_DEFAULTS;

//	LPF ThetaS_dot_LPF = LPF_DEFAULTS;
//	LPF ThetaS_dotdot_LPF = LPF_DEFAULTS;

	ENCODER Motor_Enc = ENCODER_DEFAULTS;
	ENCODER Spring_Enc = ENCODER_DEFAULTS;

//	PID_CON initial_pos_Control = PID_CON_DEFAULTS;
	PID_CON pos_Control = PID_CON_DEFAULTS;
	PID_CON for_Control = PID_CON_DEFAULTS;


	S2D QFF = S2D_DEFAULTS;
	S2D Qtd = S2D_DEFAULTS;
	S2D Qptd = S2D_DEFAULTS;


	S2D QFF1 = S2D_DEFAULTS;
	S2D Qtd1 = S2D_DEFAULTS;
	S2D Qptd1 = S2D_DEFAULTS;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Initialize();
   Process_Init();

 /*ADC_setting*/

#if 0
  		ADC1->CR &= ~ADC_CR_ADVREGEN;
  		ADC1->CR |= ADC_CR_ADVREGEN_0;
  		ADC2->CR &= ~ADC_CR_ADVREGEN;
  		ADC2->CR |= ADC_CR_ADVREGEN_0;

  		HAL_Delay(10);

  		ADC1->SQR1 &= 0x00000000;
  		ADC2->SQR1 &= 0x00000000;

  		ADC1->CR |= ADC_CR_ADEN; // Enable ADC1
  		ADC2->CR |= ADC_CR_ADEN; // Enable ADC2

  		while(!ADC1->ISR & ADC_ISR_ADRD); // wait for ADRDY
  		ADC1->CR |= ADC_CR_ADSTART;

  		while(!ADC2->ISR & ADC_ISR_ADRD); // wait for ADRDY
  		ADC2->CR |= ADC_CR_ADSTART;
#endif



 #if 1
#if 1
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR)); // DMA_InitStructure.DMA_PeripheralBaseAddr
	DMA1_Channel1->CMAR = (uint32_t)&(ADC_array_1); // DMA_InitStructure.DMA_MemoryBaseAddr
	DMA1_Channel1->CNDTR = 1; // DMA_InitStructure.DMA_BufferSize
	DMA1_Channel1->CCR |= DMA_CCR_EN; // DMA_Cmd(DMA1_Channel1, ENABLE);

	ADC1->SQR1 &= 0x00000000;
	ADC1->CR |= ADC_CR_ADEN; // Enable ADC1
	while(!ADC1->ISR & ADC_ISR_ADRD); // wait for ADRDY
	ADC1->CFGR |= ADC_CFGR_DMAEN; // DMA enable
	ADC1->CFGR |= ADC_CFGR_DMACFG; // DMA circular mode

	ADC1->CR |= ADC_CR_ADSTART; //ADC start of regular conversion
//////////////////////////////ADC2/////////////////////////////////////////////

	DMA1_Channel2->CPAR = (uint32_t)(&(ADC2->DR)); // DMA_InitStructure.DMA_PeripheralBaseAddr
	DMA1_Channel2->CMAR = (uint32_t)&(ADC_array_2); // DMA_InitStructure.DMA_MemoryBaseAddr
	DMA1_Channel2->CNDTR = 1; // DMA_InitStructure.DMA_BufferSize
	DMA1_Channel2->CCR |= DMA_CCR_EN; // DMA_Cmd(DMA1_Channel1, ENABLE);

//	ADC2->IER = ADC_IER_ADRDYIE;
//	HAL_ADC_Start_IT (&hadc2);
	ADC2->SQR1 &= 0x00000000;
	ADC2->CR |= ADC_CR_ADEN; // Enable ADC2
	while(!ADC2->ISR & ADC_ISR_ADRD); // wait for ADRDY
	ADC2->CFGR |= ADC_CFGR_DMAEN; // DMA enable
	ADC2->CFGR |= ADC_CFGR_DMACFG; // DMA circular mode

	ADC2->CR |= ADC_CR_ADSTART; //ADC start of regular conversion
#endif
 #endif


 /*SPI_setting*/

 #if 1
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,SET);
   HAL_Delay(7);
 #endif


 /*ENC_setting*/

 #if 1

   HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
   HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

 #endif

 #if 1

   HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);

 #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  start_time = HAL_GetTick();

	  /*  MATLAB_MONITORING  */
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,SET);
#if 1
	  SetBit(toSimulink.UpdateDoneFlag, 0);   // Channel 0 data가 update되었음을 표시
	  SetBit(toSimulink.UpdateDoneFlag, 1);
	  SetBit(toSimulink.UpdateDoneFlag, 2);
	  //SetBit(toSimulink.UpdateDoneFlag, 3);
	  //SetBit(toSimulink.UpdateDoneFlag, 4);
	  //SetBit(toSimulink.UpdateDoneFlag, 5);
	  ProcessToHostBlock();
#endif

//	    ENC_init();
//		initial_flag = 0;
//		start_flag = 1;

	  if(initial_flag == 1){


	  }
	  else{
		  	  SPI_init();
		  	  LPF_init();
	  }						// 아마도 LPF가 수렴하는 속도 때문에 적절한 초기 변위가 생성되지 못 한다.
	  	  	  	  	  	  	// 따라서 처음에 SPI만 계속 받아주자 --> 어차피 초기각도 생성시에만 사용하고 만다.

//	  start_flag = 1 ;

//	  if(test1 >= 1000){
////		  htim3.State = HAL_TIM_STATE_RESET;
//		  test2 = F_spring + test2;
//	  	  test1 = 0;}
//	  else{
//		  test1 = test1 + 1;
//	  }
//	  matMulMat(AA, BB, CC, 2, 2, 2);
//	  transpose(AA, BB, 2, 2);
//	  delta_test = AA[0]*AA[3] - AA[2]*AA[1];
//
//	  		if(delta_test == 0){
//	  			BB[0] = 0.0;
//	  			BB[3] = 0.0;
//	  			BB[2] = 0.0;
//	  			BB[1] = 0.0;
//	  		}
//	  		else{
//	  			BB[0] =  AA[3]/delta_test;
//	  			BB[3] =  AA[0]/delta_test;
//	  			BB[2] = -AA[2]/delta_test;
//	  			BB[1] = -AA[1]/delta_test;
//	  		}										// M44_I = 1/(H*sig_hat*H' + Q)


//	  test1 = (float) thetam_dot;
//	  test2 = (float) theta_dot_hat;

	  Start();
	  RLS();
//	  ADC_init();
//	  ENC_init();
//	  SPI_init();
//	  LPF_init();

	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,RESET);

	  while((HAL_GetTick() - start_time) < (dt*1000));
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_12BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 18;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */


void Process_Init()
{
	// __HAL_SPI_ENABLE(&hspi1);
	//-------------------------------------------------------------
	// ECL에서 개발한 Simulink 연동 monitoring library를 사용하고자 할 경우
	// 다음과 같은 초기 설정을 수행해 주어야 한다.
	//-------------------------------------------------------------
	connectData(&toSimulink, SS_SINGLE, 0, (void *)&x);
	connectData(&toSimulink, SS_SINGLE, 1, (void *)&Ang_LPF.Out);
	connectData(&toSimulink, SS_SINGLE, 2, (void *)&F_spring);
//	connectData(&toSimulink, SS_SINGLE, 1, (void *)&Torque_LPF.Out);
//	connectData(&toSimulink, SS_SINGLE, 5, (void *)&Ang_LPF.Out);

	//--------------------------------------------------------------
	// 기타 초기 설정
	//--------------------------------------------------------------
	toSimulink.ModNumber = 1;     // Mod Number. 1로 설정하면 무난함
	toSimulink.ModCntr = 0;       // ModCntr 초기화
	toSimulink.SendHoldFlag = 0;  // SendHoldFlag을 false로 초기화
}

//---------------------------------------------------------------------------//
//                             Initialize Function                           //
//---------------------------------------------------------------------------//

void Initialize()
{
//	time1 = 0;

	Motor_Enc.PPR = 1024; // 1024
	Motor_Enc.Ts = dt;
	Motor_Enc.n_bits = 16;
	Motor_Enc.calc_param(&Motor_Enc);

	Spring_Enc.PPR = 5000; // 5000
	Spring_Enc.Ts = dt;
	Spring_Enc.n_bits = 16;
	Spring_Enc.calc_param(&Spring_Enc);

	//-----------------------------------------------
	// Low Pass Filter - Reference data
	//-----------------------------------------------
	Heel_LPF.Fc = 2.0;//1.0;
	Heel_LPF.Ts = dt;
	Heel_LPF.calc_param(&Heel_LPF);

	Toe_LPF.Fc = 10.0;//1.0;
	Toe_LPF.Ts = dt;
	Toe_LPF.calc_param(&Toe_LPF);

	Ang_LPF.Fc = 5.0;//1.0;
	Ang_LPF.Ts = dt;
	Ang_LPF.calc_param(&Ang_LPF);

	Ref_LPF.Fc = 2.5;//1.0;
	Ref_LPF.Ts = dt;
	Ref_LPF.calc_param(&Ref_LPF);

	Force_LPF.Fc= 10.0;
	Force_LPF.Ts=dt;
	Force_LPF.calc_param(&Force_LPF);

	Torque_LPF.Fc=8.0;
	Torque_LPF.Ts=dt;
	Torque_LPF.calc_param(&Torque_LPF);

	//-----------------------------------------------
	// Low Pass Filter - Reference data
	//-----------------------------------------------

	Motor_dot_LPF.Fc= 10.0;
	Motor_dot_LPF.Ts=dt;
	Motor_dot_LPF.calc_param(&Motor_dot_LPF);

	Spring_dot_LPF.Fc= 10.0;
	Spring_dot_LPF.Ts=dt;
	Spring_dot_LPF.calc_param(&Spring_dot_LPF);

	//-------------------------------------------
	//Position PID controller Initialization
	//-------------------------------------------
//
//	initial_pos_Control.OutMax = 	3;
//	initial_pos_Control.OutMin =   -3;
//
//	initial_pos_Control.Kp = 0.5;	// 1.2
//	initial_pos_Control.Ki = 0;
//	initial_pos_Control.Kd = 0;
//	initial_pos_Control.Kb = 0;
//	initial_pos_Control.Ts = dt;


	pos_Control.OutMax = 	800;
	pos_Control.OutMin =   -800;

//	pos_Control.Kp = 0.7;	// 0.7
//	pos_Control.Ki = 2.0;	// 2.5
//	pos_Control.Kd = 0.02;
//	pos_Control.Kb = 0/0.7;
//	pos_Control.Ts = dt;

	pos_Control.Kp = 3.3;	// 05.0	// 3.7
	pos_Control.Ki = 10.0;	// 10.0	// 9.0
	pos_Control.Kd = 0.22;	// 0.10 // 0.25
//	pos_Control.Ka = 3.0/pos_Control.Kp;
	pos_Control.Ka = 20;
	pos_Control.Ts = dt;

	//-------------------------------------------
	//Force PID controller Initialization
	//-------------------------------------------

	for_Control.OutMax = 	500;
	for_Control.OutMin =   -500;

	for_Control.Kp = 0.02;	// 0.03
	for_Control.Ki = 0.00;
	for_Control.Kd = 0.00;
	for_Control.Ka = 0;
	for_Control.Ts = dt;


//	for_Control1.OutMax = 	500;
//	for_Control1.OutMin =   -500;
//
//	for_Control1.Kp = 0.02;	// 0.03
//	for_Control1.Ki = 0.00;
//	for_Control1.Kd = 0.00;
//	for_Control1.Kb = 0;
//	for_Control1.Ts = dt;
//	EvaRegs.CMPR1 = t1_prd/2;
//	EvaRegs.CMPR2 = t1_prd/2;



/////////////////////////////////////////////////////////////////////////////////
	QFF.T  = dt;

	QFF.b0 =  0;		//num
	QFF.b1 =  1./(2.*3.14*5.);		//num
	QFF.b2 =  1;		//num

	QFF.a0 = 1./(70.*70.);			//den
	QFF.a1 = 1.4142*(1./70.);		//den
	QFF.a2 = 1;		//den

	QFF.calc_param(&QFF);
/////////////////////////////////////////////////////////////////////////////////
	Qtd.T  = dt;

	Qtd.b0 =  0;		//num
	Qtd.b1 =  0;		//num
	Qtd.b2 =  1;		//num

	Qtd.a0 = 1./(60.*2.*PI*60.*2.*PI);			//den
	Qtd.a1 = 1.4142/(60.*2.*PI);		//den
	Qtd.a2 = 1;		//den

	Qtd.calc_param(&Qtd);
/////////////////////////////////////////////////////////////////////////////////
	Qptd.T  = dt;

	Qptd.b0 =  0;		//num
	Qptd.b1 =  1./(2.*3.14*5.);		//num
	Qptd.b2 =  1;		//num

	Qptd.a0 = 1./(2.*PI*30.*2.*PI*30.);			//den
	Qptd.a1 = 1.4142/(2.*PI*30.);		//den
	Qptd.a2 = 1;		//den

	Qptd.calc_param(&Qptd);

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

	QFF1.T  = dt;

	QFF1.b0 =  0;		//num
	QFF1.b1 =  1./(2.*3.14*5.);		//num
	QFF1.b2 =  1;		//num

	QFF1.a0 = 1./(70.*70.);			//den
	QFF1.a1 = 1.4142*(1./70.);		//den
	QFF1.a2 = 1;		//den

	QFF1.calc_param(&QFF1);
/////////////////////////////////////////////////////////////////////////////////
	Qtd1.T  = dt;

	Qtd1.b0 =  0;		//num
	Qtd1.b1 =  0;		//num
	Qtd1.b2 =  1;		//num

	Qtd1.a0 = 1./(60.*2.*PI*60.*2.*PI);			//den
	Qtd1.a1 = 1.4142/(60.*2.*PI);		//den
	Qtd1.a2 = 1;		//den

	Qtd1.calc_param(&Qtd1);
/////////////////////////////////////////////////////////////////////////////////
	Qptd1.T  = dt;

	Qptd1.b0 =  0;		//num
	Qptd1.b1 =  1./(2.*3.14*5.);		//num
	Qptd1.b2 =  1;		//num

	Qptd1.a0 = 1./(2.*PI*30.*2.*PI*30.);			//den
	Qptd1.a1 = 1.4142/(2.*PI*30.);		//den
	Qptd1.a2 = 1;		//den

	Qptd1.calc_param(&Qptd1);
}


void Process_start(){
////////////////////////Motor_start//////////////////////////////////////////////
	      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
////////////////////////////////////////////////////////////////////////////////
}


void Process_stop(){
////////////////////////Motor_stop//////////////////////////////////////////////
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
////////////////////////////////////////////////////////////////////////////////
}



void SPI_init(){
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,RESET);
	  HAL_SPI_Receive(&hspi1,null,1,1000);
	  angle_org = SPI1->DR;
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,SET);
	  angle = ((real_T)angle_org - angle_zero)*360/4096;
//	  angle_rad = angle*PI/180;
}


void PWM_init(){   // 여기에 내부 힘 제어기를 넣으면 될 것 같다.

#if 1


	QFF.input = force_input;
	QFF.calc(&QFF);

	DOB_input_1 = 0.4*(Qptd.y - Qtd.y);

	DOB_input_2 = QFF.y - DOB_input_1;

	Qtd.input = DOB_input_2;
	Qtd.calc(&Qtd);

	Qptd.input = Force_LPF.Out;
	Qptd.calc(&Qptd);
////////////////////////////////////////////////////////////////////////////
	for_Control.Ref = DOB_input_2;
//			for_Control.Ref = 20;
	for_Control.Fdb = Force_LPF.Out;
	for_Control.calc(&for_Control);
//////////////////////////////////////0//////////////////////////////////////
	FF_INPUT = for_Control.Ref * 5.06 / 897.24;  // Feedfoward input

	CURRENT_INPUT = (for_Control.Out + FF_INPUT);

	motor_generation();


#endif

}



void motor_generation()			// 여기에 힘 제어기의 결과가 들어간다. (PID + Feed Forward)
{

		if (CURRENT_INPUT >= RATED_CURRENT){
			CURRENT_INPUT = RATED_CURRENT;
		}
		else if(CURRENT_INPUT <= -RATED_CURRENT){
			CURRENT_INPUT = -RATED_CURRENT;
		}
////////////////////////////////////////////////////////////////////////////
		DUTY_INPUT = CURRENT_INPUT / 12 * 99;

		if (DUTY_INPUT >= DUTY_LIMIT){
			DUTY_INPUT = DUTY_LIMIT;
		}
		else if(DUTY_INPUT <= -DUTY_LIMIT){
			DUTY_INPUT = -DUTY_LIMIT;
		}

//		DUTY_INPUT = 2;
////////////////////////////////////////////////////////////////////////////
		if (DUTY_INPUT >= 0){
			TIM1->CCR1 = DUTY_INPUT;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,RESET);
		}
		else if (DUTY_INPUT < 0){
			TIM1->CCR1 = -DUTY_INPUT;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,SET);
		}

}



void ENC_init(){

	  Motor_Enc.Nc = TIM2->CNT;
	  m_enc = (real_T)Motor_Enc.Nc;
	  Motor_Enc.calc(&Motor_Enc);
	  thetam_rad = Motor_Enc.theta;				// radian
	  thetam = thetam_rad*360/(2.0*PI);				// rotation
	  thetam_dot = Motor_Enc.dot_theta;
	  thetam_dotdot = Motor_Enc.dotdot_theta;




	  Spring_Enc.Nc = -TIM3->CNT;                 // 스프링 엔코더의 측정 부호를 바꿔야한다.
	  S_enc = (real_T)Spring_Enc.Nc;
	  Spring_Enc.calc(&Spring_Enc);
	  thetaS_rad = Spring_Enc.theta;			// radian
	  thetaS = thetaS_rad*360/(2.0*PI);				// rotation
	  thetaS_dot = Spring_Enc.dot_theta;
	  thetaS_dotdot = Spring_Enc.dotdot_theta;

	  xs = 2 * thetaS_rad * 0.001;   			// mm  (pi * 2r * thetaS_rad / 2pi * 0.001)
	  F_spring = - Ks * xs - test2;						// N

	  T_motor = F_spring / Nm;

//	  xl = xs + thetam_rad/Nm; --> 이게 x로 이미 사용하고 있었다.

}



void ADC_init(){
#if 1

#if 1

	  ADC1->SQR1 = ADC_SQR1_SQ1_1;
	  Toe  = ((float)ADC_array_1-ADC_Toe_zero)*-1; // Get ADC1 converted data

	  ADC2->SQR1 = ADC_SQR1_SQ1_0;
	  Heel = ((float)ADC_array_2-ADC_Heel_zero)*-1; // Get ADC1 converted data


//	  sprintf(temp, "Heel = %d\r\n", Toe_val);

#endif


#if 0

////////////////////////////////////////////////////////////////////////
	  if(cnt == 0){
	  	  ADC1->SQR1 = ADC_SQR1_SQ1_1;
//	  	  ADC2->SQR1 = ADC_SQR1_SQ1_0; //PA4
//	  	  ADC2->SQR1 |= *(pDrvPattern+1);
	  	  Toe = -1*((real_T)ADC1->DR-ADC_Toe_zero);
	  	  cnt = 1;
	  }
	  else if(cnt == 1){
//	  	  ADC1->SQR1 |= ADC_SQR1_SQ1_1;
//	  	  ADC2->SQR1 = ADC_SQR1_SQ1_0|ADC_SQR1_SQ1_1;
		  ADC2->SQR1 = ADC_SQR1_SQ1_0;
		  //ADC2->SQR1 |= *(pDrvPattern+1);
	  	  Heel = -1*(ADC2->DR-ADC_Heel_zero);
	  	  cnt = 0;
	  }

#endif



	#if 0

	if(cnt == 0){
	Toe = (float)ADC_array_1;
	ADC1->SQR1 = ADC_SQR1_SQ1_0; // PA 0

	Heel = (float)ADC_array_2;
	ADC2->SQR1 = ADC_SQR1_SQ1_0; // PA 4

	cnt = 1;
	}
	else if(cnt == 1){
	test1 = (float)ADC_array_1;
	ADC1->SQR1 = ADC_SQR3_SQ10_1; // PA 1

	test2 = (float)ADC_array_2;
	ADC2->SQR1 = ADC_SQR3_SQ10_0; // PA 5

	cnt = 0;
	}

	#endif


	#if 0

	  if(cnt == 0){
//    high_data = ADC_array_1;
//	  ADC1->SQR1 = ADC_SQR1_SQ1_0; // PA 0


		  Toe = ((real_T)ADC_array_1-ADC_Toe_zero)*-1;
		  ADC1->SQR1 = ADC_SQR1_SQ1_1;


//		  LOAD_val = (real_T)ADC_array_2;
//		  ADC2->SQR1 = ADC_SQR1_SQ1_0; // PA 4

	  cnt = 1;
	  }

	  else if(cnt == 1){
//	  ADC1->SQR1 = ADC_SQR1_SQ1_1; // PA 1


		  Heel = ((real_T)ADC_array_2-ADC_Heel_zero)*-1; //HEEL
		  ADC2->SQR1 = ADC_SQR1_SQ1_1;

	  cnt = 0;
    }

	#endif

	#if 0

	if(cnt == 0){
//	  	high_data = ADC_array_1;
//	  	ADC1->SQR1 = ADC_SQR1_SQ1_0; // PA 0

		Toe = ((real_T)ADC_array_1-ADC_Toe_zero)*-1;
		ADC1->SQR1 = 0x00000040;


//	  	LOAD_val = (real_T)ADC_array_2;
//	  	ADC2->SQR1 = ADC_SQR1_SQ1_0; // PA 4

		cnt = 1;
  	  }

  	  else if(cnt == 1){
 //	  	ADC1->SQR1 = ADC_SQR1_SQ1_1; // PA 1

  		Heel = ((real_T)ADC_array_1-ADC_Heel_zero)*-1; //HEEL
  		ADC1->SQR1 = 0x00000080;


//		CURRENT_val = (real_T)ADC_array_2;
//		ADC2->SQR1 = ADC_SQR1_SQ1_1; // PA 5 : LOADcell

  		cnt = 0;
      }
	#endif

	#if 0
  	  ADC2->SQR1 = ADC_SQR1_SQ1_1; // PA 5 : LOADcell
  	  LOAD_val = (real_T)ADC_array_2;
  	  LOAD_compare = (LOAD_val - ADC_load_zero)*-1.4;
	#endif

#endif
}



void LPF_init(){

	  Ang_LPF.In = (real_T)angle;
	  Ang_LPF.calc(&Ang_LPF);
//	  angle_rad = Ang_LPF.Out*PI/180;

	  Toe_LPF.In = (real_T)Toe;
	  Toe_LPF.calc(&Toe_LPF);
	  Toe_org = Toe_LPF.Out;

	  Heel_LPF.In = (real_T)Heel;
	  Heel_LPF.calc(&Heel_LPF);
	  Heel_org = Heel_LPF.Out;


	  Motor_dot_LPF.In = (real_T)thetam_dot;
	  Motor_dot_LPF.calc(&Motor_dot_LPF);
	  Motor_dot_lpf = Motor_dot_LPF.Out;


	  Spring_dot_LPF.In = (real_T)thetaS_dot;
	  Spring_dot_LPF.calc(&Spring_dot_LPF);
	  Spring_dot_lpf = Spring_dot_LPF.Out;
}


void Kalman_init()
{


//////////////////////////////////////////////////////////
		theta_hat 		= theta_hat 		+ theta_dot_hat*((double) dt);
//        theta_dot_hat 	= theta_dot_hat 	+ (-(Bm/Jm)*theta_dot_hat + (1/Jm)*((double)T_motor) + (1/Jm/Nm)*((double)F_spring))*((double) dt);
//        theta_dot_hat 	= theta_dot_hat 	+ (-(Bm/Jm)*theta_dot_hat + (1/Jm)*((double)torque_kalman) - (1/Jm/Nm)*((double)F_spring))*((double)dt);
        theta_dot_hat 	= theta_dot_hat 	+ (-(Bm/Jm)*theta_dot_hat)*((double)dt);

//////////////////////////////////////////////////////////
		transpose(G, G_T, 2, 2);
		transpose(V, V_T, 2, 2);
		transpose(H, H_T, 2, 2);

		//Prediction
		matMulMat(G, sig, M1, 2, 2, 2);
		matMulMat(M1, G_T, M11, 2, 2, 2);		// M11 = G*sig*G'

		matMulMat(V, sig_con, M2, 2, 2, 2);
		matMulMat(M2, V_T, M22, 2, 2, 2);		// M22 = V*sig_con*V'

//		test1 = M22[3];

		R[0] = 1;
		R[1] = 0;
		R[2] = 0;
		R[3] = 50;

/////////////////////matrix sum///////////////////////////
//		for(int i = 0; i <= 3; i++){
//				sig_hat[i] = M11[i] + M22[i];   // sig_hat = G*sig*G' + V*sig_con*V'
//		}

		for(int i = 0; i <= 3; i++){
				sig_hat[i] = M11[i] + R[i];   // sig_hat = G*sig*G' + V*sig_con*V'
		}
//////////////////////////////////////////////////////////


        //Correction
//////////////////////////////////////////////////////////
		matMulMat(sig_hat, H_T, M4, 2, 2, 2);  // M4  = sig_hat*H'

		matMulMat(H, sig_hat, M3, 2, 2, 2);
		matMulMat(M3, H_T, M33, 2, 2, 2);      // M33 = H*sig_hat*H'
///////////////////////Add///////////////////////////////
		for(int i = 0; i <= 3; i++){
				M44[i] = M33[i] + Q[i];        // M44 = H*sig_hat*H' + Q
		}
//////////////////////Inversion/////////////////////////
		delta = M44[0]*M44[3] - M44[2]*M44[1];

		if(delta == 0){
			M44_I[0] = 0.0;
			M44_I[3] = 0.0;
			M44_I[2] = 0.0;
			M44_I[1] = 0.0;
		}
		else{
			M44_I[0] =  M44[3]/delta;
			M44_I[3] =  M44[0]/delta;
			M44_I[2] = -M44[2]/delta;
			M44_I[1] = -M44[1]/delta;
		}										// M44_I = 1/(H*sig_hat*H' + Q)
//////////////////////////////////////////////////////////
		matMulMat(M4, M44_I, K, 2, 2, 2);       // K = sig_hat*H'/(H*sig_hat*H' + Q)
//		K = M4/(M44); // 2X1
//      u = [theta_hat;theta_dot_hat;theta_dotdot_hat] + K*([th_e;th_e_dot]-H*[theta_hat;theta_dot_hat;theta_dotdot_hat]);
        u[0] = theta_hat 		+ K[0]*((double)thetam_rad - theta_hat) 	+ K[2]*((double)thetam_dot - theta_dot_hat);
        u[1] = theta_dot_hat 	+ K[1]*((double)thetam_rad - theta_hat) 	+ K[3]*((double)thetam_dot - theta_dot_hat);
//      [theta_hat;theta_dot_hat] + K*([thetam_rad; thetam_speed]-H*[theta_hat;theta_dot_hat]);
//////////////////////////////////////////////////////////
		matMulMat(K, H, M5, 2, 2, 2);			// M5  = K*H
///////////////////////Add///////////////////////////////
		for(int i = 0; i <= 3; i++){
				M55[i] = I[i] - M5[i];			// M55 = I - K*H
		}
//////////////////////////////////////////////////////////
		matMulMat(M55, sig_hat, sig, 2, 2, 2);  // sig = (I - K*H)*sig_hat

//////////////////////////////////////////////////////////
		theta_hat 			= u[0];
		theta_dot_hat	 	= u[1];

		thetam_rad_kalman 		= (real_T) u[0];
		thetam_rad_speed_kalman = (real_T) u[1];


		test3 = (thetam_rad_speed_kalman - test4)/0.004;
		test4 = thetam_rad_speed_kalman;

}

void Kalman_init_S()
{

		transpose(G_S, G_T_S, 2, 2);
		transpose(V_S, V_T_S, 2, 1);
		transpose(H_S, H_T_S, 2, 2);

//Prediction
		matMulMat(G_S, sig_S, S1, 2, 2, 2);
		matMulMat(S1, G_T_S, S11, 2, 2, 2);		// M11 = G*sig*G'    	==> 2X2

		matMulMat(V_S, sig_con_S, S2, 2, 1, 1); // 2*1 X 1*1
		matMulMat(S2, V_T_S, S22, 2, 1, 2);		// 2*1 X 1*2
												// M22 = V*sig_con*V'	==> 2X2
		S22[0] = 1;
		S22[1] = 0;
		S22[2] = 0;
		S22[3] = 1;   // 커질수록 측정값의 영향을 많이 받고, 변동이 심함 // 얘가 이렇게 설정되면 R_S의 역할을 한다.
//////////////////////////////////////////////////////////
		theta_hat_S 		= theta_hat_S 		+ theta_dot_hat_S*((double) dt);
        theta_dot_hat_S 	= theta_dot_hat_S 	+ (-(Bs/Ms)*theta_dot_hat_S - (((double)Ks)/Ms)*theta_hat_S + (1/Ms)*((double)-F_spring))*((double) dt);
/////////////////////matrix sum///////////////////////////
		for(int i = 0; i <= 3; i++){
				sig_hat_S[i] = S11[i] + S22[i];   // sig_hat = G*sig*G' + V*sig_con*V'
		}
//////////////////////////////////////////////////////////
//Correction
//////////////////////////////////////////////////////////
		matMulMat(sig_hat_S, H_T_S, S4, 2, 2, 2);  // M4  = sig_hat*H'

		matMulMat(H_S, sig_hat_S, S3, 2, 2, 2);
		matMulMat(S3, H_T_S, S33, 2, 2, 2);      // M33 = H*sig_hat*H'
///////////////////////Add///////////////////////////////
		for(int i = 0; i <= 3; i++){
				S44[i] = S33[i] + Q_S[i];        // M44 = H*sig_hat*H' + Q  // 커질수록 측정값의 영향을 적게 받고, 변동이 덜함
		}
//////////////////////Inversion/////////////////////////
		delta_S = S44[0]*S44[3] - S44[2]*S44[1];

		if(delta_S == 0){
			S44_I[0] = 0.0;
			S44_I[3] = 0.0;
			S44_I[2] = 0.0;
			S44_I[1] = 0.0;
		}
		else{
			S44_I[0] =  S44[3]/delta_S;
			S44_I[3] =  S44[0]/delta_S;
			S44_I[2] = -S44[2]/delta_S;
			S44_I[1] = -S44[1]/delta_S;
		}										// M44_I = 1/(H*sig_hat*H' + Q)
//////////////////////////////////////////////////////////
		matMulMat(S4, S44_I, K_S, 2, 2, 2);       // K = sig_hat*H'/(H*sig_hat*H' + Q)
//		K = M4/(M44); // 2X1
//      u = [theta_hat;theta_dot_hat;theta_dotdot_hat] + K*([th_e;th_e_dot]-H*[theta_hat;theta_dot_hat;theta_dotdot_hat]);
        u_S[0] = theta_hat_S 		+ K_S[0]*((double)thetaS_rad - theta_hat_S) 	+ K_S[2]*((double)thetaS_dot - theta_dot_hat_S);
        u_S[1] = theta_dot_hat_S 	+ K_S[1]*((double)thetaS_rad - theta_hat_S) 	+ K_S[3]*((double)thetaS_dot - theta_dot_hat_S);
//      [theta_hat;theta_dot_hat] + K*([thetam_rad; thetam_speed]-H*[theta_hat;theta_dot_hat]);
//////////////////////////////////////////////////////////
		matMulMat(K_S, H_S, S5, 2, 2, 2);			// M5  = K*H
///////////////////////Add///////////////////////////////
		for(int i = 0; i <= 3; i++){
				S55[i] = I[i] - S5[i];			// M55 = I - K*H
		}
//////////////////////////////////////////////////////////
		matMulMat(S55, sig_hat_S, sig_S, 2, 2, 2);  // sig = (I - K*H)*sig_hat

//////////////////////////////////////////////////////////
		theta_hat_S 		= u_S[0];
		theta_dot_hat_S 	= u_S[1];

		thetaS_rad_kalman 			= (real_T) u_S[0];
		thetaS_rad_speed_kalman 	= (real_T) u_S[1];
}



void RLS() {

//  real_T P[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//  real_T gamma = 0;
//  real_T h[3] = {0, 0, 0};
//
//  real_T x_measurement[3] = {0, 0, 0};
//  real_T x_measurement_T[3] = {0, 0, 0};
//  real_T K_num[3] = {0, 0, 0};
//  real_T K_den[1] = {0};
//
//  real_T P_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//  real_T P_2 = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//  real_T theta_k_dot;
//  real_T theta_k_dotdot;
//  real_T r;
//
//  real_T Ja = 0;
//  real_T Ba = 0;
//  real_T m_estimated = 0;
//
//
//  real_T y_temp = 0;
//  real_T e_temp = 0;
	double K_DEN = 0;


  h[0] = Ja;
  h[1] = Ba;
  h[2] = m_estimated;

  x_measurement[0] = theta_k_dotdot;
  x_measurement[1] = theta_k_dot;
  x_measurement[2] = g*r*cos(theta_k);

  y_temp = h[0]*theta_k_dotdot + h[1]*theta_k_dot + h[2] * g*r*cos(theta_k);

  e_temp = Torque_LPF.Out - y_temp;

  transpose(x_measurement, x_measurement_T, 3, 3);
  matMulMat(P, x_measurement, K_num, 3, 3, 1);
  matMulMat(x_measurement_T, K_num, K_den, 1, 3, 1);

  K_DEN = K_den[0];

  for(int i = 0; i <= 2 ; i++){

	  K_rls[i] = K_num[i] / K_DEN;

  }

  h[0] = h[0] + K_rls[0]*e_temp;
  h[1] = h[1] + K_rls[1]*e_temp;
  h[2] = h[2] + K_rls[2]*e_temp;

  matMulMat(K_rls, x_measurement_T, P_1, 3, 1, 3);
  matMulMat(P_1, P, P_2, 3, 3, 3);

  for(int i = 0; i <= 8 ; i++){

	  P[i] = (P[i] - P_2[i]) / Gamma; 	  // P = P / g;
	  	  	  	  	  	  	  	  	  //  P = g^(-1) * P - g^(-1)*K*x_measurement_T * P;
  }


  Ja = h[0];
  Ba = h[1];
  m_estimated = h[2];


}






void Start() { // T = 1.83ms

if(start_flag == 1) {

		real_T F_ref;											// 이건 내 모터에 맞게 인풋을 바꿔야한다.
																// 위치 제어기 내부의 힘 제어기로 입력된다.
		real_T tnext, tnext_temp1, tnext_temp2;

		SPI_init();
		ENC_init();												// 이 때 값을 읽어오기 시작한다.
		ADC_init();
		LPF_init();												// Heel, Toe, Angle(absolute)
																// Heel_org, Toe_org, Ang_LPF.Out
		Kalman_init();
/////////////SPI 있어야 함///////////////////////////////////
	if(initial_flag ==0)
	{
//		alpha = pi - (2.5139 + 0.1489 - angle_LPF*2*pi/360); 	// alpha_0 + theta
//		xl0 = sqrt(L1*L1 + L2*L2 - 2*L1*L2*cos(alpha));

		angle_rad = Ang_LPF.Out*PI/180;							// 이 부분에서 앱솔루트 엔코더의 값을 읽어오고

		alpha = pi - (2.5139 + 0.1489 - angle_rad);
		x0 = sqrt(L1*L1+L2*L2-2*L1*L2*cos(alpha));				// 초기 로드 길이도 받아온다.

		x = x0;													// 여기까지가 초기 단계

		initial_flag = 1;
	}
	else
	{
		x = x0 + xs + thetam_rad/Nm;							// 이후 부터는 3번식으로 바꿔준다.

	}



//	test3 = Ang_LPF.Out;`



	thetak_rad = acos((L1*L1+L2*L2-x*x)/(2*L1*L2))-alpha_0;		// 현재 무릎 각도를 계산해준다.
	thetak = thetak_rad*180/PI;									// 이건 무릎 각도를 degree로 바꿔준다.

	thetak_velocity = (thetak-thetak_past)/dt;					// 이건 무릎의 속도
	thetak_past=thetak;											// 무릎 속도를 계산하기 위해 무릎의 이전 값을 저장해준다.


////////////Spring으로 측정한 힘///////////////////////////////

	if(flag_start<=100)
	{
		Force_LPF.State = F_spring;								// 여기에 F_spring이 들어가야한다.
		flag_start++;
	}

	Force_LPF.In = F_spring;
	Force_LPF.calc(&Force_LPF);

//------------------------------------------------------------
//                      Calculate the Torque
//------------------------------------------------------------
	torque_kalman = F_spring*L1*L2*sin(alpha_0+thetak_rad)/sqrt(L1*L1+L2*L2-2*L1*L2*cos(alpha_0+thetak_rad));     // 여기에서 무릎 토크를 계산해 준다. // 위에서 T_k
	torque = Force_LPF.Out*L1*L2*sin(alpha_0+thetak_rad)/sqrt(L1*L1+L2*L2-2*L1*L2*cos(alpha_0+thetak_rad));     // 여기에서 무릎 토크를 계산해 준다. // 위에서 T_k

	if(flag_start<=100)
	{
		Torque_LPF.State = torque;								// 위에서 계산한 토크로 여기서 T_k_lpf
	}

	Torque_LPF.In = torque;
	Torque_LPF.calc(&Torque_LPF);

///////////Heel&Toe 필요함 -> interrupt 에서 받아 옴/////////////////////

		if(Heel_org > ADC_Heel_ON)	HeelState = 1;				// 뒷꿈치 센서
		else if(Heel_org < ADC_Heel_OFF)	HeelState = 0;

		if(Toe_org > ADC_Toe_ON) ToeState = 1;					// 앞꿈치 센서
		else if(Toe_org < ADC_Toe_OFF)	ToeState = 0;

//////////////////////////////////////////////////////////////////


	switch(stateNum){																// 얘는 4번 부터 시작한다.
		case 0:	// Stance Phase - (Stance Flexion + Stance Extension)

			thetak_ref = angle_eq0 - Torque_LPF.Out/k0+(thetak_0-angle_eq0+Torque_LPF.Out/k0)*exp(-k0/b0*(time-time_ex));		// spring+damper
			if( (HeelState==0) && (ToeState==1) ){
				stateNum = 1;
				thetak_old = thetak_ref;
				thetak_0=thetak_ref;
				time_ex=time;
			}
			break;


		case 1:	// Pre-Swing phase
//			k1 = 1.0;
//			b1 = 0.05;
			thetak_ref = angle_eq1 - Torque_LPF.Out/k1+(thetak_0-angle_eq1+Torque_LPF.Out/k1)*exp(-k1/b1*(time-time_ex));		// spring+damper

			if( (HeelState==0) && (ToeState==0) ){
				stateNum = 2;
				thetak_old = thetak_ref;
				thetak_0=thetak_ref;
				time_ex=time;
			}
			break;


		case 2: // Swing Flexion phaseh
			thetak_ref = angle_eq2 - torque/k2+(thetak_0-angle_eq2+torque/k2)*exp(-k2/B2*(time-time_ex));		// spring+damper

			if( (thetak_velocity_past>0) && (thetak_velocity<15) && (thetak>30)){//45  //vel 15
				stateNum = 3;
				thetak_old = thetak_ref;
				thetak_0=thetak_ref;
				time_ex=time;
			}
			break;


		case 3:	// Swing Extension phase
			b3 = 0.007; // 얘가 커질수록 빠지는 각도가 작아진다 --> 되돌아 오는 속도가 커진다.
			thetak_ref = thetak_old - torque/b3*dt;		// damper

			thetak_old = thetak_ref;

			if(thetak<20){
				stateNum = 4;
			}
			break;

		case 4: // Pre-Landing phase ; High Impedance
			thetak_ref = 10;											// 처음에 10도로 맞춰준다.

			if(HeelState==1){										// 뒷꿈치가 땅에 닿았을 때 -> 오른발을 앞으로 내딛고 뒷꿈치가 닿을 때
				test2 = F_spring + test2;

				stateNum=0;
				thetak_old = thetak_ref;
				thetak_0=thetak_ref;									// 여기서 old랑 0를 둘 다 10도로 맞춰준다.
				time_ex=time;
			}
			break;
		default:
			break;
	}
///////////////////////////////////////////////////////////////////////
	if(thetak_velocity != 0) thetak_velocity_past=thetak_velocity;

	if(thetak_ref>80){
		thetak_ref=80;
		thetak_old = thetak_ref;
	}
	else if(thetak_ref<8){
		thetak_ref =8;
		thetak_old = thetak_ref;
	}																// 무릎 각도의 최대 최소 범위를 나타낸다.
////////////////////////////////////////////////////////////////////////
//	if(test5 == 0){
//	thetak_ref = 50;
//	}
//	else{
//		thetak_ref = 15;
//	}
//	thetak_ref = 30;

//	thetak_ref = 13;
	thetak_ref_rad = thetak_ref*PI/180;							// 위에서 생성된 무릎 각도의 기준 값을 radian으로 바꿔준다.


	x_ref = sqrt(L1*L1+L2*L2-2*L1*L2*cos(alpha_0+thetak_ref_rad));	// 무릎 각도로 기준 로드의 길이를 역산한다.
	x_diff = x_ref-(x0+xs);

	thetam_ref = x_diff*500*3.;
	thetam_ref_rad = thetam_ref*2*PI;								// 기준 로드의 길이로 모터의 회전 각도 기준을 생성한다. (무릎 기준 각도 -> 로드 기준 길이 -> 모터 기준 회전 각도)

//	thetam_ref_rad = Nm*x_diff;
//	thetam_ref_rad = 200;
	thetam_ref_rad = 200*sin(time*((2.0*pi)*(0.1)));

///////////////////////////////////////////////////////////////////////

//------------------------------------------------------
// Low pass Filtering of Reference data (angle value)
//------------------------------------------------------
	Ref_LPF.In=thetam_ref_rad;
	Ref_LPF.calc(&Ref_LPF);
//---------------------------------------------------
// Low Pass Filtering of the angle 		(Feedback value)
//---------------------------------------------------
	Pos_LPF.In = thetam_rad;
	Pos_LPF.calc(&Pos_LPF);											// 현재 모터의 회전 각도와 위에서 생성된 모터의 기준 회전 각도를 모두 필터링 해준다.

//	thetak_lpf_rad = Ref_LPF.Out/1000/PI+x0;
//	thetak_lpf_rad =  acos((L1*L1+L2*L2-thetak_lpf_rad*thetak_lpf_rad)/(2*L1*L2))-alpha0;
	thetak_lpf = thetak_lpf_rad*180/PI;								// 모터의 회전 각도를 degree로 바꿔준다. -> 그냥 plot용
///////////////////////////////////////////////////////////////////////

//------------------------
// PID control computation
//------------------------

	pos_Control.Ref = Ref_LPF.Out;
	pos_Control.Fdb = thetam_rad;									// 위에서 필터링한 값을 안 쓰고 이걸 썼네? -> 이건 다시 해보자

	pos_Control.calc(&pos_Control);

//	test1 = pos_Control.Kp * pos_Control.Err;
//	test4 = pos_Control.Ka * pos_Control.SatErr;
//	test2 = pos_Control.Intg * pos_Control.Ki;
//	test3 = pos_Control.Deriv * pos_Control.Kd;
////////////////////DUTY///////////////////////////////////////////////

	F_ref = pos_Control.Out;

	force_input = F_ref;

	PWM_init();

////////////////////TIME///////////////////////////////////////////////

//---------------------------
// Time update
//---------------------------
time += dt;

	tnext_temp1 = ceil((time/dt)+1);
	tnext_temp2 = (time/dt) + 1;
	if((tnext_temp1 - tnext_temp2)>0.500)
	   	tnext = (tnext_temp1-1)*dt;
	else
	tnext = tnext_temp1*dt;
	time = tnext;
//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,RESET);
}


else if(start_flag==0){

	Initialize();

}



}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_1) // SW1
	{

		start_flag = 0;

		if (test5 <1){
			test5 += 1;
		}
		else{
			test5 = 0;
		}

		Process_stop();

	}
	else if(GPIO_Pin == GPIO_PIN_3) // SW2
	{

		initial_flag = 0;
		start_flag = 1;
		flag_start = 0;



		Process_start();
	}



}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
