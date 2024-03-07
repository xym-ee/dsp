#include "head.h"


unsigned long int msgh,msgl;
unsigned long int msgh_tx, msgl_tx,msg_tx_temp;
unsigned long int msgh_updata,msgl_updata,msg_updata_temp;


unsigned int can_ctrl=1;  //  1  can ctrl     0  ad  ctrl
unsigned int SwitchCNT=0;

unsigned int AutoUpdata = 0;
unsigned int usCNT = 0;

unsigned int motornum;
unsigned int RUN_MODE;
unsigned int CURRENT_CNT;
unsigned int LED_CNT=0;

unsigned int FPGA_ID=0;
unsigned int PowerState=0;
unsigned int PowerUpCNT=0;
unsigned int PowerDownCNT=0;

float u_c_offset=0;
float v_c_offset=0;
float bus_v_offset=0;

unsigned int DosCNT = 0;
unsigned int LosCNT = 0;
unsigned int PLLCNT = 0;




extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;  

int DAC_View;


/****************position loop*********************/
unsigned int MotorSpeedCNT=0;
float Motor_speed_sum=0;

float Motor_rdc=0;



float   Voltage_bus_fil=0;
float   Voltage_bus_last=0;



float Analog_cm = 0;
float Analog_cm_1 = 0;
int   Analog_cm_read = 0;


unsigned int Motor_rdc_correct_flag=0;
float Motor_rdc_zero,Motor_rdc_angle,Motor_rdc_angle_last,Delta_Motor_rdc_angle,Motor_Mposition,Motor_Eposition;
int Motor_rdc_sign,Motor_pair_poles;


/*************Positon Loop**********************/

int Position_counter;
int Position_Cycle,Position_Cycle_DIR,Position_Start_Point,Position_Relative,Position_Delta;
int Position_Out,Position_Speed_LIMIT;
float Position_Kp, Position_Kp_Set,Position_Ki,Position_Ki_Set;
int Position_Error , Position_Error_Set;

/*************speed loop**********************/

unsigned int SinTestFlag = 0;
float SinFreq=0;
unsigned int sin_p_time_cnt = 0;


float Speed_cmd,Speed_cmd_set;
int   Cycle_DIR;
float Speed_cmd_fil,Speed_motor,Speed_error;
float Speed_Kp,Speed_Ki,Speed_Kp_Set,Speed_Ki_Set;
float Speed_error_I,Speed_ctrl_P,Speed_ctrl_I,Speed_ctrl,Speed_ctrl_last,Speed_ctrl_lim;

float Motor_speed_fil_20ms;
float I_modify_index = 1.0;
float P_modify_index = 1.0;
/*************current loop**********************/




float Current_a=0;
float Current_b=0;

float Current_a_1=0; 
float Current_b_1=0;

float Sin_theta=0;
float Cos_theta=0;
float Current_alfa=0;
float Current_beta=0;
float Current_d=0;
float Current_q=0;


float Current_d_filter=0;
float Current_q_filter=0;
float Current_Is=0;
float Current_Is_SUM=0;
float Current_Is_AVR=0;



float  Currentd_error_sum=0;
float Currentq_error_sum=0;
float Currentd_error_avr=0;
float Currentq_error_avr=0;
unsigned int Current_error_cnt=0;
unsigned int Current_error_flag=0;
unsigned int Current_OL_flag=0;
unsigned int Current_OL_cnt=0;

float Current_cmd,Current_cmd_set,Current_cmd_filter;
float Currentd_cmd,Currentq_cmd;
float Currentd_max,Currentq_max;
float Currentd_error,Currentq_error;
float Currentd_ctrl_P,Currentq_ctrl_P;
float Currentd_error_I,Currentq_error_I;
float Currentd_Kp,Currentq_Kp;
float Currentd_ctrl_I,Currentq_ctrl_I;
float Currentd_Ki,Currentq_Ki;
float Currentd_ctrl,Currentq_ctrl;
float Current_alfa_ctrl,Current_beta_ctrl;
float Current_a_ctrl,Current_b_ctrl,Current_c_ctrl;
float pwmout_a,pwmout_b,pwmout_c,pwmout_max;
float cur_zero_drift;
int pwmcmp_a,pwmcmp_b,pwmcmp_c;
long int p_time_cnt;



int data_id;
unsigned int pc_command_id;
unsigned int motor_id;
int Soft_EN = 0;

float SPEED_STEP,SPEED_LIMIT,CURRENT_LIMIT;

float Position_Cycle_Window;

int  E2PROM_START;

unsigned int Error_state_low=0;
unsigned int Error_state_high=0;
unsigned int Error_state_send=0x0200;
unsigned int Error_Rec=0;

unsigned int  ERROR_State_temp = 0;
unsigned int  CAN_LOST_CNT=0;
unsigned int  MOTOR_OVERHEAT_CNT=0;
unsigned int  CURRQ_CHAOCHA_CNT=0;
unsigned int  CURRD_CHAOCHA_CNT=0;
unsigned int  SPD_CHAOCHA_CNT =0;
unsigned int  OVER_LOAD_CNT   =0;


float PWM_set;

unsigned int  Mask_Flag=ERR_MASK;


unsigned int HistoryErr=0;


/**************************  Resolver  ***********************************/
float final_pos=0;
float final_spd=0;
float final_spd_last=0;
float final_spd_fil=0;

unsigned int spd_rec_cnt=0;
float   spd_rec[10]={
				
			   0 , 0 , 0 , 0 , 0,
			   0 , 0 , 0 , 0 , 0,

};

float spd_val[20]={
               0 , 0 , 0 , 0 , 0,
			   0 , 0 , 0 , 0 , 0,
			   0 , 0 , 0 , 0 , 0,
			   0 , 0 , 0 , 0 , 0,
};

long int  SINE_TABLE[16] = {
	BASE,             BASE + SINE225,   BASE + SINE450,   BASE + SINE675,
	BASE + SINE900,   BASE + SINE675,   BASE + SINE450,   BASE + SINE225,
	BASE,             BASE - SINE225,   BASE - SINE450,   BASE - SINE675,
	BASE - SINE900,   BASE - SINE675,   BASE - SINE450,   BASE - SINE225,
};

float FIR_COEFF[17] = {
   -0.08303630352,  -0.08532184362, -0.07218718529, -0.04381770268,
   -0.004064163659,  0.04017074406,  0.08038958907,  0.1084507555,
    0.1185115501,    0.1084507555,   0.08038958907,  0.04017074406,
   -0.004064163659, -0.04381770268, -0.07218718529, -0.08532184362,
   -0.08303630352
  };

RESOLVER_INPUT rslvrIn;
RESOLVER_OUTPUT rslvrOut;

long int *sineTable = SINE_TABLE;
float  *firCoeff  = FIR_COEFF;

// analog input offset estimation variables
float offsetFc,    // offset filter corner freq (Hz)
      offsetWfT;    // offset filter constant, internally computed based on filter coefficients

// control loop parameters (can be replaced with MACROs)
float  errorFc,    // error filter corner freq (in rad/sec)
       Kp,         // pi controller P gain
       piconFz;    // PI controller zero

long int  testCntr = 0,         // local counter for ISR tick
		 testCntMax = 1000;    // max count value

float  testAngleMax = 0.8;   // max test angle for ref gen

long int //resMag20,                     // resolver magnitude in Q20
      resetAll = 0,                 // reset all error flags
      resMagMax = _IQ(-2.0),        // resolver magnitude max value
      resMagMin = _IQ(2.0),         // resolver magnitude min value
      DOS_ERROR = 0,                // degradation of signal (DOS) - error flag
      LOS_ERROR = 0,                // loss of signal (LOS) - error flag
      PLL_ERROR = 0,                // PLL loop error flag
      dos,                          // degradation of signal (DOS)
      dosLimit = _IQ(0.5),          // Limit value of DOS
      losLimit = _IQ(0.8),          // Limit value of LOS
      skipLosCnt  = 0,              // cnt value for los validation
      skipLosPrd  = 500,            // Prd value for los validation  100
      angleErrMax = _IQ(0.4);       // Limit value of PLL loop error

unsigned int ReslvoReadyFlag=0;

/********************************* DSP  AD  SAMPLE****************************/
unsigned int  SampleSumCNT;
unsigned int  UcurentSamp[15]; 
unsigned int  VcurentSamp[15]; 
unsigned int  BusVolSamp[15];
unsigned int  IpmTempSamp[15];
unsigned int  AcommSamp[15];
unsigned int  MotorTempSamp[15];

float  UcurentOS=0; 
float  VcurentOS=0; 
float  BusVolOS=0;
float  IpmTempOS=0;
float  AcommOS=0;
float  MotorTempOS=0;

float  OffsetS_CHK=0;
float  OffsetC_CHK=0;
float  OffsetU_CHK=0;
float  OffsetV_CHK=0;



