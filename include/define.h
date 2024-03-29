#ifndef DEFINE_H
#define DEFINE_H

#ifndef GLOBAL_Q
#define GLOBAL_Q       20
#endif


/********************************************/
//--------          PLL         ------------//
/*******************************************/

#define PLLMUL        10
#define PLLDIV         2

/********************************************/
//--------         ADC          ------------//
/*******************************************/

#define ADC_MODCLK 0x2// HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
#define ADC_CKPS   0x1   // ADC module clock = HSPCLK/1      = 25.5MHz/(1)   = 25.0 MHz
#define ADC_SHCLK  0x1   // S/H width in ADC module periods 
#define ADC_cal_ptr          (void (*) (void)) 0x380080
/********************************************/
//--------           PWM        ------------//
/*******************************************/




#define EPWM1_TIMER_TBPRD 7500
#define EPWM2_TIMER_TBPRD 7500
#define EPWM3_TIMER_TBPRD 7500


#define DB_PRD_UP         1500
#define DB_PRD_DOWN       1500

// CTRMODE bits
#define	TB_COUNT_UP		0x0
#define	TB_COUNT_DOWN	0x1
#define	TB_COUNT_UPDOWN	0x2
#define	TB_FREEZE		0x3
// PHSEN bit
#define	TB_DISABLE		0x0
#define	TB_ENABLE		0x1
// PRDLD bit
#define	TB_SHADOW		0x0
#define	TB_IMMEDIATE	0x1
// SYNCOSEL bits
#define	TB_SYNC_IN		0x0
#define	TB_CTR_ZERO		0x1
#define	TB_CTR_CMPB		0x2
#define	TB_SYNC_DISABLE	0x3
// HSPCLKDIV and CLKDIV bits
#define	TB_DIV1			0x0
#define	TB_DIV2			0x1
#define	TB_DIV4			0x2
// PHSDIR bit
#define	TB_DOWN			0x0
#define	TB_UP			0x1

// CMPCTL (Compare Control)
//==========================
// LOADAMODE and LOADBMODE bits
#define	CC_CTR_ZERO		0x0
#define	CC_CTR_PRD		0x1
#define	CC_CTR_ZERO_PRD	0x2
#define	CC_LD_DISABLE	0x3
// SHDWAMODE and SHDWBMODE bits
#define	CC_SHADOW		0x0
#define	CC_IMMEDIATE	0x1

// AQCTLA and AQCTLB (Action Qualifier Control)
//=============================================
// ZRO, PRD, CAU, CAD, CBU, CBD bits
#define	AQ_NO_ACTION	0x0
#define	AQ_CLEAR		0x1
#define	AQ_SET			0x2
#define	AQ_TOGGLE		0x3

// DBCTL (Dead-Band Control)
//==========================
// OUT MODE bits
#define	DB_DISABLE		0x0
#define	DBA_ENABLE		0x1
#define	DBB_ENABLE		0x2
#define	DB_FULL_ENABLE	0x3
// POLSEL bits
#define	DB_ACTV_HI		0x0
#define	DB_ACTV_LOC		0x1
#define	DB_ACTV_HIC		0x2
#define	DB_ACTV_LO		0x3
// IN MODE
#define DBA_ALL         0x0
#define DBB_RED_DBA_FED 0x1
#define DBA_RED_DBB_FED 0x2
#define DBB_ALL         0x3

// ETSEL (Event Trigger Select)
//=============================
#define	ET_CTR_ZERO		0x1
#define	ET_CTR_PRD		0x2
#define	ET_CTRU_CMPA	0x4
#define	ET_CTRD_CMPA	0x5
#define	ET_CTRU_CMPB	0x6
#define	ET_CTRD_CMPB	0x7

// ETPS (Event Trigger Pre-scale)
//===============================
// INTPRD, SOCAPRD, SOCBPRD bits
#define	ET_DISABLE		0x0
#define	ET_1ST			0x1
#define	ET_2ND			0x2
#define	ET_3RD			0x3



// ---------------------------------- RESOLVER  -----------------------------------------
#define  BASE             280 
#define  SINE900         (BASE*0.8)
#define  SINE225         (int)(SINE900*0.383)
#define  SINE450         (int)(SINE900*0.707)
#define  SINE675         (int)(SINE900*0.924)
#define  PI               3.14159265358979
#define  TWO_PI           (2*PI)
////////////////////////////////////////////////////////////////


/********************************************/
//--------        CAN MSGID      ------------/
/*******************************************/

/*
#define  CANID_PC_R                 0x15000000          //PC_R  540
#define  CANID_PC_T                 0x16000000          //PC_T  580
#define  CANID_PC_R1                0x17000000          //PC_R  5C0
#define  CANID_PC_T1                0x18000000
#define  CANID_PC_T2                0x19000000           //PC_T  600
*/

#define  CANID_DRV_R                 0x00000000         // 00000000
#define  CANID_DRV_T                 0x00040000         // 00000001
#define  CANID_DRV_T_1               0x00080000         // 00000002


#define  DSP28_ECANB                1



/********************************************/
//-----------PC COMAND ID  -----------------/
/*******************************************/
#define  COM_CAN_CTRL            0x0000
#define  DAT_CAN_CTRL            0x0000

#define  COM_AD_CTRL             0xc009
#define  DAT_AD_CTRL             0xaa12          

#define  COM_EN                  0xc003
#define  DAT_EN                  0xaa55 

#define  COM_DIS_EN              0xc004
#define  DAT_DIS_EN              0x55aa

#define  COM_BRAKE               0xc005
#define  BRAKE_ON                1
#define  BRAKE_OFF               0 


#define  COM_RUN_MODE            0x8021
#define  DAT_POS_MODE            0x0001
#define  DAT_VEL_MODE            0x0002
#define  DAT_CUR_MODE            0x0003 

#define  COM_COM                 0xc001  

#define  COMAND_SET_MOTORNUM     0xd000
#define  COMAND_POLEPAIRS        0xd001
#define  COMAND_DIR              0xd002
#define  COMAND_SET_KPC          0xd003
#define  COMAND_SET_KIC          0xd004
#define  COMAND_DRIFT_CUR        0xd005
#define  COMAND_LIM_CUR          0xd006
#define  COMAND_SET_KPV          0xd007
#define  COMAND_SET_KIV          0xd008
#define  COMAND_SR_VEL           0xd009
#define  COMAND_LIM_VEL          0xd00a
#define  COMAND_SET_KPP          0xd00b
#define  COMAND_SET_KIP          0xd00c
#define  COMAND_CLR_ERR          0xd00d

//20200422
#define  COMAND_TEST1            0x001b
#define  COMAND_TEST2            0x001c
#define  COMAND_TEST3            0x001d
#define  COMAND_TEST4            0x001e
#define  COMAND_TEST5            0x001f
#define  COMAND_TEST6            0x0020
// 20200422


#define  CHECK_VEL               0xe001
#define  CHECK_CURR              0xe002
#define  CHECK_POS               0xe003
#define  CHECK_BUS_VOL           0xe004
#define  CHECK_ERR_STATE         0xe005
#define  CHECK_IPM_TEMP          0xe006
#define  CHECK_PARA_SET          0xe007
#define  CHECK_UPDATA_ON         0xe008
#define  CHECK_UPDATA_OFF        0xe009   


#define  DAT_UPDATA              0x5a5a          

/********************************************/
//-------------E2PROM ADDRESS------- --------/
/*******************************************/

#define ADDRESS_INTIAL         0x00     //e2prom initial

#define ADDRESS_MOTORNUM       0x02        

#define ADDRESS_MODE           0x04  
    
#define ADDRESS_POLEPAIRS      0x06  

#define ADDRESS_DIR            0x08 


#define ADDRESS_KPC            0x0A                 
#define ADDRESS_KIC            0x0C
#define ADDRESS_DRIFT          0x0E                 
#define ADDRESS_LIM_CUR        0x10

#define ADDRESS_KPV            0x12
#define ADDRESS_KIV            0x14
#define ADDRESS_SRV            0x16
#define ADDRESS_LIM_VEL        0x18


#define ADDRESS_KPP            0x1A
#define ADDRESS_KIP            0x1C

#define ADDRESS_CAN_AD         0x1E

#define ADDRESS_ERROR_START    0x20
#define ADDRESS_ERROR_END      0x2a



#define ADDRESS_END            0x4E

#define E2PROM_INITIAL         0x5555
#define E2PROM_END             0xAAAA

#define E2PRPM_CORRECT           1
#define E2PRPM_FAULT             0





////////////////外部接口定义////////////////////

#define FPGA_ERR_CODE         *(Uint16 *)0x4001
#define FPGA_EX_IN1           *(Uint16 *)0x4002
#define FPGA_EX_IN2           *(Uint16 *)0x4003
#define FPGA_TEST_CODE        *(Uint16 *)0x400F


#define FPGA_ERR_IO           *(Uint16 *)0x4010
#define FPGA_RELAY            *(Uint16 *)0x4011
#define FPGA_FAN              *(Uint16 *)0x4012
#define FPGA_IPM_PW           *(Uint16 *)0x4013
#define FPGA_EX_OUT2          *(Uint16 *)0x4014

#define TEST_PORT        GpioDataRegs.GPBDAT.bit.GPIO49
#define FLIP_TOGGLE      GpioDataRegs.GPBTOGGLE.bit.GPIO51

#define LED0_TOGGLE      GpioDataRegs.GPBTOGGLE.bit.GPIO52
#define LED1_PORT        GpioDataRegs.GPBDAT.bit.GPIO53

#define EN_PORT          GpioDataRegs.GPBDAT.bit.GPIO54
#define FPGA_RST_PORT    GpioDataRegs.GPBDAT.bit.GPIO55

#define BOX_LED_PORT     GpioDataRegs.GPBDAT.bit.GPIO56
#define BOX_LED_TOGGLE   GpioDataRegs.GPBTOGGLE.bit.GPIO56
//////////////////////滤波系数//////////////////

#define  Current_dfil           100.0
#define  Current_qfil           100.0
#define  Current_cmdfil         58.0

#define  CUR_FIL_VAR            0.65
#define  SPD_FIL_VAR_OBS        0.6
#define  VOL_FIL_VAR            0.985
#define  TMP_FIL_VAR            0.75 
#define  AD_COM_FIL_VAR         0.99


//////////////////////驱动器参数//////////////////

#define  RATED_SPEED            2500.0
#define  MAX_CURRENT            5                 //    current limit value   (rms)
#define  RATED_CURRENT          2                 //     over    load  value   (rms)   rated     
#define  CURRENT_RATIO          546.1333          //     15A -- 0.625V -- 0.375V   8192 DIGI  
#define  DELTA_CURRENT_DIGI     4 * CURRENT_RATIO //      0.5A * CURRENT_RATIO


#define  FAN_ON_TEMP            30.0            //     40
#define  FAN_OFF_TEMP           25.0           //      35
#define  MOTOR_OVER_HEAT_TEMP   130.0

#define  RELAY_VOL_ON           2048           //     200V              800 V 4096

#define  RELAY_VOL_OFF          1792           //     350V            



//////////////////////  故障保护  //////////////////


//    Time
#define  CAN_LINKOFF_TIME             500           //    500ms
#define  MOTOR_OVERHEAT_TIME          10000          //   10s
#define  CURR_CHAOCHA_TIME            10            //    200ms   10*20ms
#define  SPD_CHAOCHA_TIME             5000          //    500ms
#define  OVER_LOAD_TIME               20            //    1s     20*50ms  
#define  OUTPUT_ERR_TIME              10            //    100ms   10*10ms
#define  RDC_JUMP_TIME                100           //    10ms
#define  RDC_ERR_CNT_VAL              300

#define  LOW_CURRENT                  0.05* MAX_CURRENT * CURRENT_RATIO         //    5% max current 
#define  CURR_CHAOCHA_FACTOR          0.3
#define  MID_SPD_CHAOCHA_VALUE        400
#define  MID_SPD                      400
#define  LOW_SPD_CHAOCHA_FACTOR       0.5
#define  LOW_SPD                      10
#define  OUTPUT_ERR_VALUE             1.0


//ERR CODE
#define  ERR_CODE_ADOFFSET        0x8000        //    bit 15
#define  ERR_CODE_RDC             0x4000        //    bit 14
#define  ERR_CODE_OVERLOAD        0x2000       //     bit 13
#define  ERR_CODE_SPEEDCHAOCHA    0x1000       //     bit 12
#define  ERR_CODE_CURRCHAOCHA     0x0800       //     bit 11
#define  ERR_CODE_CANLINKOFF      0x0400       //     bit 10
#define  ERR_CODE_POWERNOTREADY   0x0200       //     bit 09
#define  ERR_CODE_MOTOROVERHEAT   0x0100       //     bit 08



#define  POLE_POS_DETECT            0          //    1  ON       0  OFF
//#define  ERR_MASK                 0xff3b       //    1 no mask   0  mask
#define  ERR_MASK                 0xe73f 


//		bit 15   AD_CAL_ERR   bit 14   RDC  err     bit 13   over load    bit  12  speed err ov 
//		bit 11  curr err ov   bit 10   CAN  err     bit 09   power relay  bit  08  MOTOR OT
//      bit 07  IO EN        bit  06   DSP EN       bit 05   DSP FLY      bit  04  IPM ERR
//     bit 03   ov           bit  02   uv           bit 01    oc          bit  0   IPM OT 


#endif

