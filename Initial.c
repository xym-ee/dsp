#include "head.h"
#pragma CODE_SECTION(InitFlash,"ramfuncs");



void initial_para_self(void)
{


	 CURRENT_CNT = 0;
	 

	 /////////////////////////////////////////////

	 SinTestFlag = 0;
 	 SinFreq=0;	 

//	 Motor_rdc_angle = 0.0;
	 Motor_Mposition = 0.0;
	 Motor_Eposition = 0.0;

	 /****************Position loop ****************/

	 Position_counter = 0;

	 Position_Cycle= 0 ;


	 Position_Start_Point=0;
	 Position_Relative = 0;
	 Position_Delta = 0;
	 Position_Out = 0;
	 Position_Kp =0.0;
	 Position_Ki = 0.0;
	 Position_Error = 0;
	 
	 

    /*************** speed loop ************/	 
     
     Position_counter = 0;

	 Speed_cmd = 0;
	 Speed_cmd_set = 0;
	 Speed_cmd_fil = 0.0;
     Speed_motor = 0.0;
     Speed_error = 0.0;
     Speed_Kp = 0.0;
     Speed_Ki = 0.0;
     Speed_error_I = 0.0;
     Speed_ctrl_P = 0.0;
     Speed_ctrl_I = 0.0;
     Speed_ctrl = 0.0;
	 Speed_ctrl_last = 0.0;



	 /***************current loop************/
	 Current_cmd = 0.0;
	 Current_cmd_set = 0.0;
	 Current_cmd_filter = 0.0;
	

	 Currentd_cmd = 0.0;
	 Currentq_cmd = 0.0;
	 Currentd_max = 0.248;
	 Currentq_max = 0.9688;
	 Currentd_error = 0.0;
	 Currentq_error = 0.0;
     Currentd_ctrl_P = 0.0;
     Currentq_ctrl_P = 0.0;
	 Currentd_error_I = 0.0;
	 Currentq_error_I = 0.0;

   
     
     Currentd_ctrl_I = 0.0;
     Currentq_ctrl_I = 0.0;
     


	 Currentd_ctrl = 0.0;
	 Currentq_ctrl = 0.0;
	 Current_alfa_ctrl = 0.0;
	 Current_beta_ctrl = 0.0;
	 Current_a_ctrl = 0.0;
	 Current_b_ctrl = 0.0;
	 Current_c_ctrl = 0.0;
     pwmout_a = 0.0;
     pwmout_b = 0.0;
     pwmout_c = 0.0;
	 
	 sin_p_time_cnt = 0;
	 p_time_cnt = 0;



	 data_id = 0xffff;
	 pc_command_id = 0xffff;





	 Currentd_error_sum=0;
     Currentq_error_sum=0;
     Currentd_error_avr=0;
     Currentq_error_avr=0;
     Current_error_cnt=0;
	 Current_error_flag=0;

	 CAN_LOST_CNT = 0;
	 MOTOR_OVERHEAT_CNT = 0;
	 CURRQ_CHAOCHA_CNT = 0;
	 CURRD_CHAOCHA_CNT = 0;
	 SPD_CHAOCHA_CNT = 0;
	 OVER_LOAD_CNT = 0;

//	 Delta_RDC=0;
//     RDC_LAST=0;
//     RDC_JMP_CNT=0;




     Position_Cycle_Window = Position_Speed_LIMIT * 0.04;
	 




}

void  driver_parameter(void)
{

//================================================================
//========          Çý¶¯Æ÷²ÎÊý¿ØÖÆÌ¨³ÌÐò      ====================
//================================================================ 
                                

//µç»ú³õÊ¼ÁãÎ»
         Motor_rdc_zero = 14460.0;             //B-202-baozha  
                                            //B-202-wubaozha  6150.0      
                                          //  380V 8500RPM   115str10ab
                                        
                                         //    7525  15733  tanhuang motor
                                       
                                        //    JINBAO SM60  BLACK  152-SIMPLE  9020 4886  821 13099 
                                            
                                            
                                       //  4200 12340  AKM JINBAO SM80

                                     //  845   AKM  JINBAO SM60-1930 GRARY
         
                                    //  420 4520  8600 12723  JINBAO SM60  BLACK
                                   
                                    //  6439   405B  shenlan
									
                                   //  16236  49003     // 407  ÎÞ±§Õ¢   16bit

		                           //  27000   59767     // 407  ±§Õ¢     16bit

								   //    12276.0         // 407  ÎÞ±§Õ¢   14bit


//RDC Ðý×ª·½Ïò£¬ÈôRDCÎªË³Ê±Õë£¬ÔòÎª-1£¬ÄæÊ±ÕëÎª1
         Motor_rdc_sign = -1;  



		 Position_Speed_LIMIT = 100;




// ËÙ¶ÈÐý×ªÕý·½Ïò

//		 Cycle_DIR = 1;             //   +1 (li)  or  -1 (shun)  

		 Position_Cycle_DIR = -1;   //   +1  (li)   or  -1  (shun)	  



		 Position_Error_Set = 10;



		
}

void InitSysCtrl(void)
{
	// Disable the watchdog
   DisableDog();

   // Initialize the PLL control: PLLCR and DIVSEL
   // DSP28_PLLCR and DSP28_DIVSEL are defined in DSP2833x_Examples.h
   InitPll(PLLMUL,PLLDIV);

   // Initialize the peripheral clocks
   InitPeripheralClocks();

}

void DisableDog(void)
{
    EALLOW;
    SysCtrlRegs.WDCR= 0x0068;
    EDIS;
}

void InitPll(int val,int divsel)
{
	 // Make sure the PLL is not running in limp mode
   if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
   {
      // Missing external clock has been detected
      // Replace this line with a call to an appropriate
      // SystemShutdown(); function.
      asm("        ESTOP0");
   }

   // DIVSEL MUST be 0 before PLLCR can be changed from
   // 0x0000. It is set to 0 by an external reset XRSn
   // This puts us in 1/4
   if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
   {
       EALLOW;
       SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
       EDIS;
   }

   // Change the PLLCR
   if (SysCtrlRegs.PLLCR.bit.DIV != val)
   {

      EALLOW;
      // Before setting PLLCR turn off missing clock detect logic
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
      SysCtrlRegs.PLLCR.bit.DIV = val;
      EDIS;

      // Optional: Wait for PLL to lock.
      // During this time the CPU will switch to OSCCLK/2 until
      // the PLL is stable.  Once the PLL is stable the CPU will
      // switch to the new PLL value.
      //
      // This time-to-lock is monitored by a PLL lock counter.
      //
      // Code is not required to sit and wait for the PLL to lock.
      // However, if the code does anything that is timing critical,
      // and requires the correct clock be locked, then it is best to
      // wait until this switching has completed.

      // Wait for the PLL lock bit to be set.

      // The watchdog should be disabled before this loop, or fed within
      // the loop via ServiceDog().

	  // Uncomment to disable the watchdog
      DisableDog();

      while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
      {
	      // Uncomment to service the watchdog
          // ServiceDog();
      }

      EALLOW;
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
      EDIS;
    }

    // If switching to 1/2
	if((divsel == 1)||(divsel == 2))
	{
		EALLOW;
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = divsel;
	    EDIS;
	}

	// If switching to 1/1
	// * First go to 1/2 and let the power settle
	//   The time required will depend on the system, this is only an example
	// * Then switch to 1/1
	if(divsel == 3)
	{
		EALLOW;
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
	    delay(10);
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;
	    EDIS;
    }
}


void InitPeripheralClocks(void)
{
   EALLOW;

// HISPCP/LOSPCP prescale register settings, normally it will be set to default values
//HISPCLK = SYSCLKOUT / (HISPCP * 2)
//LOSPCLK = SYSCLKOUT / (LOSPCP * 2)
   SysCtrlRegs.HISPCP.all = 0x0001;
   SysCtrlRegs.LOSPCP.all = 0x0002;

// XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/4 SYSCLKOUT
   // XTIMCLK = SYSCLKOUT/2
   XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
   // XCLKOUT = XTIMCLK/2
   XintfRegs.XINTCNF2.bit.CLKMODE = 1;
   // Enable XCLKOUT
   XintfRegs.XINTCNF2.bit.CLKOFF = 0;

// Peripheral clock enables set for the selected peripherals.
// If you are not using a peripheral leave the clock off
// to save on power.
//
// Note: not all peripherals are available on all 2833x derivates.
// Refer to the datasheet for your particular device.
//
// This function is not written to be an example of efficient code.

   SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;    // ADC     ÓÃHSPCLK¼ÆÊ±

	delay(100);                              

      // *IMPORTANT*
   // The ADC_cal function, which  copies the ADC calibration values from TI reserved
   // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
   // Boot ROM. If the boot ROM code is bypassed during the debug process, the
   // following function MUST be called for the ADC to function according
   // to specification. The clocks to the ADC MUST be enabled before calling this
   // function.
   // See the device data manual and/or the ADC Reference
   // Manual for more information.

  
   (*ADC_cal_ptr)();                          // call ADC_cal() from OTP - ADCCLK must have been already enabled



   SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;   // I2C       ÓÃSYSCLKOUT¼ÆÊ±
   SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;   // SCI-A     ÓÃLSPCLKÆÊ±
   SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 0;   // SCI-B
   SysCtrlRegs.PCLKCR0.bit.SCICENCLK = 1;   // SCI-C
   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;   // SPI-A     ÓÃLSPCLK¼ÆÊ±
   SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 0; // McBSP-A   ÃLSPCLKÆÊ±
   SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 0; // McBSP-B
   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=1;    // eCAN-A    ÓÃSYSCLKOUT/2¼ÆÊ±
   SysCtrlRegs.PCLKCR0.bit.ECANBENCLK=1;    // eCAN-B

   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC  = 0;  // Disable TBCLK within the ePWM
   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1     ÓÃSYSCLKOUT¼ÆÊ±
   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
   SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
   SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1;  // ePWM4
   SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1;  // ePWM5
   SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1;  // ePWM6
  

   SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK = 0;  // eCAP3     ÓÃSYSCLKOUT¼ÆÊ±
   SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK = 0;  // eCAP4
   SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK = 0;  // eCAP5
   SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK = 0;  // eCAP6
   SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 0;  // eCAP1
   SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK = 0;  // eCAP2
   SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 0;  // eQEP1     ÓÃSYSCLKOUT¼ÆÊ±
   SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 0;  // eQEP2

   SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
   SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 0; // CPU Timer 1
   SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 0; // CPU Timer 2

   SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 0;       // DMA Clock
   SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;     // XTIMCLK
   SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;    // GPIO input clock

   EDIS;
}


void InitFlash(void)
{
   EALLOW;
   //Enable Flash Pipeline mode to improve performance
   //of code executed from Flash.
   FlashRegs.FOPT.bit.ENPIPE = 1;   //Ê¹ÄÜFLASHÁ÷Ë®ÏßÄ£Ê½

   //                CAUTION
   //Minimum waitstates required for the flash operating
   //at a given CPU rate must be characterized by TI.
   //Refer to the datasheet for the latest information.
//#if CPU_FRQ_150MHZ
   //Set the Paged Waitstate for the Flash
   FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;

   //Set the Random Waitstate for the Flash
   FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;

   //Set the Waitstate for the OTP
   FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;
//#endif

//#if CPU_FRQ_100MHZ
   //Set the Paged Waitstate for the Flash
 // FlashRegs.FBANKWAIT.bit.PAGEWAIT = 3;

   //Set the Random Waitstate for the Flash
 //  FlashRegs.FBANKWAIT.bit.RANDWAIT = 3;

   //Set the Waitstate for the OTP
 //  FlashRegs.FOTPWAIT.bit.OTPWAIT = 5;
//#endif
   //                CAUTION
   //ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
   FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;    //´ÓË¯Ãßµ½±¸ÓÃÌ¬µÈ´ý511¸öÖÜÆÚ
   FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;    //´Ó±¸ÓÃµ½¼¤»îÌ¬µÈ´ý511¸öÖÜÆÚ
   EDIS;

   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.

   asm(" RPT #7 || NOP");
}



void InitPieCtrl(void)
{
    // Disable Interrupts at the CPU level:
    DINT;

    // Disable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

	// Clear all PIEIER(Ê¹ÄÜ) registers:
	PieCtrlRegs.PIEIER1.all = 0;
	PieCtrlRegs.PIEIER2.all = 0;
	PieCtrlRegs.PIEIER3.all = 0;	
	PieCtrlRegs.PIEIER4.all = 0;
	PieCtrlRegs.PIEIER5.all = 0;
	PieCtrlRegs.PIEIER6.all = 0;
	PieCtrlRegs.PIEIER7.all = 0;
	PieCtrlRegs.PIEIER8.all = 0;
	PieCtrlRegs.PIEIER9.all = 0;
	PieCtrlRegs.PIEIER10.all = 0;
	PieCtrlRegs.PIEIER11.all = 0;
	PieCtrlRegs.PIEIER12.all = 0;

	// Clear all PIEIFR(±êÖ¾) registers:
	PieCtrlRegs.PIEIFR1.all = 0;
	PieCtrlRegs.PIEIFR2.all = 0;
	PieCtrlRegs.PIEIFR3.all = 0;	
	PieCtrlRegs.PIEIFR4.all = 0;
	PieCtrlRegs.PIEIFR5.all = 0;
	PieCtrlRegs.PIEIFR6.all = 0;
	PieCtrlRegs.PIEIFR7.all = 0;
	PieCtrlRegs.PIEIFR8.all = 0;
	PieCtrlRegs.PIEIFR9.all = 0;
	PieCtrlRegs.PIEIFR10.all = 0;
	PieCtrlRegs.PIEIFR11.all = 0;
	PieCtrlRegs.PIEIFR12.all = 0;


}



void Initial_INT(void)
{
	DINT;        //Disable interrupt

	InitPieCtrl();
	IER = 0x0000;
    IFR = 0x0000;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;// PIE_TABLE ENABLE
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM6_INT   = &current_isr; 
    PieVectTable.ADCINT      = &ad_sample_isr;   
	EDIS;
}
void InitCpuTimers(void)
{
    // CPU Timer 0
	// Initialize address pointers to respective timer registers:
	CpuTimer0.RegsAddr = &CpuTimer0Regs;
	// Initialize timer period to maximum:ÖÜÆÚ¼Ä´æÆ÷£¬³õÊ¼»¯¶¨Ê±Æ÷ÖÜÆÚ
	CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;
	// Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer0Regs.TPR.all  = 0;
	CpuTimer0Regs.TPRH.all = 0;
	// Make sure timer is stopped:¿ØÖÆ¼Ä´æÆ÷£¬Í£Ö¹¶¨Ê±Æ÷
	CpuTimer0Regs.TCR.bit.TSS = 1;
	// Reload all counter register with period value: ¶¨Ê±Æ÷ÖØÐÂ×°ÔØ
	CpuTimer0Regs.TCR.bit.TRB = 1;
	// Reset interrupt counters:
	CpuTimer0.InterruptCount = 0;


// CpuTimer 1 and CpuTimer2 are reserved for DSP BIOS & other RTOS
// Do not use these two timers if you ever plan on integrating
// DSP-BIOS or another realtime OS.
//
// Initialize address pointers to respective timer registers:
	CpuTimer1.RegsAddr = &CpuTimer1Regs;
	CpuTimer2.RegsAddr = &CpuTimer2Regs;
	// Initialize timer period to maximum:
	CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;
	CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer1Regs.TPR.all  = 0;
	CpuTimer1Regs.TPRH.all = 0;
	CpuTimer2Regs.TPR.all  = 0;
	CpuTimer2Regs.TPRH.all = 0;
    // Make sure timers are stopped:
	CpuTimer1Regs.TCR.bit.TSS = 1;
	CpuTimer2Regs.TCR.bit.TSS = 1;
	// Reload all counter register with period value:
	CpuTimer1Regs.TCR.bit.TRB = 1;
	CpuTimer2Regs.TCR.bit.TRB = 1;
	// Reset interrupt counters:
	CpuTimer1.InterruptCount = 0;
	CpuTimer2.InterruptCount = 0;

}

struct CPUTIMER_VARS CpuTimer0;
struct CPUTIMER_VARS CpuTimer1;
struct CPUTIMER_VARS CpuTimer2; 

void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period)
{
	Uint32 	temp;

	// Initialize timer period:
	Timer->CPUFreqInMHz = Freq;
	Timer->PeriodInUSec = Period;
	temp = (long) (Freq * Period);
	Timer->RegsAddr->PRD.all = temp;

	// Set pre-scale counter to divide by 1 (SYSCLKOUT):
	Timer->RegsAddr->TPR.all  = 0;
	Timer->RegsAddr->TPRH.all  = 0;

	// Initialize timer control register:
	Timer->RegsAddr->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer
	Timer->RegsAddr->TCR.bit.TRB = 1;      // 1 = reload timer
	Timer->RegsAddr->TCR.bit.SOFT = 0;
	Timer->RegsAddr->TCR.bit.FREE = 0;     // Timer Free Run Disabled
	Timer->RegsAddr->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt

	// Reset interrupt counter:
	Timer->InterruptCount = 0;

	CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0 0100 0000 0000 0001Æô¶¯¶¨Ê±Æ÷
}

void EN_INT(void)
{
   IER |= M_INT1;
   IER |= M_INT3; 
  
   PieCtrlRegs.PIEIER1.bit.INTx6 = 1; //ADÖÐ¶ÏÊ¹ÄÜ
   PieCtrlRegs.PIEIER3.bit.INTx6 = 1; //EPWM6ÖÐ¶ÏÊ¹ÄÜ

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
}

void DIS_INT(void)
{
	PieCtrlRegs.PIEIER1.bit.INTx6 = 0; //ADÖÐ¶Ï½ûÖ¹
    PieCtrlRegs.PIEIER3.bit.INTx6 = 0; //EPWM6ÖÐ¶Ï½ûÖ¹
	DINT;
	DRTM;




}
void InitEPwm1()
{


   /******ÉèÖÃTime_Base¼Ä´æÆ÷******/

   EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;              // ÉèÖÃ¼ÆÊýÆ÷ÖÜÆÚ
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;              // ÉèÖÃÏàÎ»Îª0
   EPwm1Regs.TBCTR = 0x0000;                         // ÇåÁã¼ÆÊýÆ÷
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;    // ²úÉú¶Ô³Æ²¨ÐÎµÄÄ£Ê½
   EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;            // ÏàÎ»ÔØÈë
   EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;            // Í¨÷·
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;       // È·¶¨PWMµÄÍ¬½Ê±ÖÊäÈë£¬CTR=Zero
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;          // ¸ßÙÊ±ÓÖµ
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;             // Ê±ÖÓ·ÖÆµTBCLK = SYSCLKOUT / (HSPCLKDIV*CLKDIV)

   /*****ÉèÖÃCounter-Compare¼Ä´æÆ÷*****/

   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;       // ´ÓÒõÓ°¼Ä´æ÷ØÈë±È½ÏÖµ
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;       // ´ÓÒõÓ°Ä´æÆ÷ÔØÈë±½ÏÖµ
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØë
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØÈë
   EPwm1Regs.CMPA.half.CMPA = 3750;
   
   /*****ÉèÖÃAction-Qualifier¼ÄæÆ÷****/
   
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;                // Ôö¼ÆÊýµ½COMPAÊ±ÖÃ¸ßµçÆ½
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;              // ¼õ¼ÆÊýµ½COMPAÊ±ÖÃµÍµçÆ½

   /*****ÉèÖÃDead_Band¼Ä´æÆ÷*****/

   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;    // Dead_Band¼Ä´æÆ÷Êä³öµÄPWM1A¡¢PWM1B¶¼ÓÐËÀÇø
   EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;            // Dead_Band¼Ä´æÆ÷ÊäÈëÊ±PWM1AÉÏÉýÑØºÍÏÂ½µÑØ¶¼ÓÐÑÓ
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;         // EPWM1BÄ£Ê½
 //  EPwm1Regs.DBFED = DB_PRD_UP;                       // Éè¶¨ÏÂµÑËÀÇøÊ±¼ä
 //  EPwm1Regs.DBRED = DB_PRD_DOWN;                     // Éè¶¨ÉÏÉýÑØËÀÇøÊ±¼ä

   EPwm1Regs.DBFED = 1000;                       // Éè¶¨ÏÂµÑËÀÇøÊ±¼ä
   EPwm1Regs.DBRED = 1000; 

   EALLOW;


   EPwm1Regs.TZCTL.bit.TZA = 2;
   EPwm1Regs.TZCTL.bit.TZB = 2;

   EPwm1Regs.TZFRC.bit.OST = 1;

   EDIS;


}


void InitEPwm2()
{  
   /******ÉèÖÃTime_Base¼Ä´æÆ÷******/

   EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;           // ÉèÖÃ¼ÆÊýÆ÷µÄÖÜÆÚ
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;            // ÉèÖÃÏàÎ»Îª0
   EPwm2Regs.TBCTR = 0x0000;                      // ÇåÁã¼ÆÊýÆ÷
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // ²úÉú¶Ô³Æ²¨ÐÎµÄÄ£Ê½
   EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;         // Í¨¹ýÒõÓ°¼Ä´æÆ÷·ÃÎÊ
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // È·¶¨EPWM2µÄÍ¬²½Ê±ÖÓÊäÈë£¬EPWM1SYNC
   EPwm2Regs.TBCTL.bit.PHSEN =    TB_ENABLE;      // Ê¹ÄÜÏàÎ»ÔØÈë
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // ¸ßËÙÊ±ÖÓ·ÖÆµ
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Ê±ÖÓ·ÖÆµTBCLK = SYSCLKOUT / (HSPCLKDIV*CLKDIV)
   
   /*****ÉèÖÃCounter-Compare¼Ä´æÆ÷*****/

   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;       // ´ÓÒõÓ°¼Ä´æÆ÷ÔØÈë±È½ÏÖµ
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;       // ´ÓÒõÓ°¼Ä´æÆ÷ÔØÈë±È½ÏÖµ
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØÈë
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØÈë
   EPwm2Regs.CMPA.half.CMPA = 3750;
  
    /*****ÉèÖÃAction-Qualifier¼Ä´æÆ÷*****/
   
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;                // Ôö¼ÆÊýµ½COMPAÊ±ÖÃ¸ßµçÆ½
   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;              // ¼õ¼ÆÊýµ½COMPAÊ±ÖÃµÍµçÆ½
   /*****ÉèÖÃDead_Band¼Ä´æÆ÷*****/

   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;    // Dead_Band¼Ä´æÆ÷Êä³öµÄPWM2A¡¢PWM2B¶¼ÓÐËÀÇø
   EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;            // Dead_Band¼Ä´æÆ÷ÊäÈëÊ±PWM2AÉÏÉýÑØºÍÏÂ½µÑØ¶¼ÓÐÑÓ³Ù
   EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;         // Ö÷¸ß»¥²¹Ä£Ê½
  // EPwm2Regs.DBFED = DB_PRD_UP;                             // Éè¶¨ÏÂ½µÑØËÀÇøÊ±¼ä
 //  EPwm2Regs.DBRED = DB_PRD_DOWN;                             // Éè¶¨ÉÏÉýÑØËÀÇøÊ±¼ä

   EPwm2Regs.DBFED = 1000;                             // Éè¶¨ÏÂ½µÑØËÀÇøÊ±¼ä
   EPwm2Regs.DBRED = 1000;                             // Éè¶¨ÉÏÉýÑØËÀÇøÊ±¼ä

   EALLOW;

   EPwm2Regs.TZCTL.bit.TZA = 2;
   EPwm2Regs.TZCTL.bit.TZB = 2;

   EPwm2Regs.TZFRC.bit.OST = 1;

   EDIS;


 
}

void InitEPwm3(void)
{

   
  /******ÉèÖÃTime_Base¼Ä´æÆ÷******/

   EPwm3Regs.TBPRD = EPWM3_TIMER_TBPRD;           // ÉèÖÃ¼ÆÊýÆ÷µÄÖÜÆÚ
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;            // ÉèÖÃÏàÎ»Îª0
   EPwm3Regs.TBCTR = 0x0000;                      // ÇåÁã¼ÆÊýÆ÷
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // ²úÉú¶Ô³Æ²¨ÐÎµÄÄ£Ê½
   EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;         // Í¨¹ýÒõÓ°¼Ä´æÆ÷·ÃÎÊ
   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // È·¶¨EPWM3µÄÍ¬²½Ê±ÖÓÊäÈë£¬EPWM2SYNC
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;         // Ê¹ÄÜÏàÎ»ÔØÈë
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // ¸ßËÙÊ±ÖÓ·ÖÆµ
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Ê±ÖÓ·ÖÆµTBCLK = SYSCLKOUT / (HSPCLKDIV*CLKDIV)

   /*****ÉèÖÃCounter-Compare¼Ä´æÆ÷*****/

   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;       // ´ÓÒõÓ°¼Ä´æÆ÷ÔØÈë±È½ÏÖµ
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;       // ´ÓÒõÓ°¼Ä´æÆ÷ÔØÈë±È½ÏÖµ
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØÈë
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØÈë
   EPwm3Regs.CMPA.half.CMPA = 3750;

  
    /*****ÉèÖÃAction-Qualifier¼Ä´æÆ÷*****/
   
   EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;                // Ôö¼ÆÊýµ½COMPAÊ±ÖÃ¸ßµçÆ½
   EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;              // ¼õ¼ÆÊýµ½COMPAÊ±ÖÃµÍµçÆ½

 
   /*****ÉèÖÃDead_Band¼Ä´æÆ÷*****/

   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;    // Dead_Band¼Ä´æÆ÷ÊäöµÄPWM3A¡¢PWM3B¶¼ÓÐËÀÇø
   EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;            // Dead_Band¼Ä´æÆ÷ÊäÈëÊ±PWM3AÉÏÉýÑØºÍÏÂ½µÑØ¶¼ÓÐÑÓ³Ù
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;         // ÇÐ»»EPWM3BÄ£Ê½
 //  EPwm3Regs.DBFED = DB_PRD_UP;                             // Éè¶¨Â½µÑØËÀÇøÊ±¼ä
 //  EPwm3Regs.DBRED = DB_PRD_DOWN;                             // Éè¶¨ÉÏÉýÑØËÀÇøÊ±¼ä

   EPwm3Regs.DBFED = 1000;                             // Éè¶¨Â½µÑØËÀÇøÊ±¼ä
   EPwm3Regs.DBRED = 1000;                             // Éè¶¨ÉÏÉýÑØËÀÇøÊ±¼ä

   EALLOW;

   EPwm3Regs.TZCTL.bit.TZA = 2;
   EPwm3Regs.TZCTL.bit.TZB = 2;

   EPwm3Regs.TZFRC.bit.OST = 1;

   EDIS;


  
}
void InitEPwm4(void)
{
	  /******ÉèÖÃTime_Base¼Ä´æÆ÷******/

   EPwm4Regs.TBPRD = 937;                         //  150000 / 937 = 160kHz  base
   EPwm4Regs.TBPHS.half.TBPHS = 0x0000;            // ÉèÖÃÏàÎ»Îª0
   EPwm4Regs.TBCTR = 0x0000;                      // ÇåÁã¼ÆÊýÆ÷
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Èý½Ç²¨
   EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;      
   EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     
   EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;        
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // ¸ßËÙÊ±ÖÓ·ÖÆµ
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Ê±ÖÓ·ÖÆµTBCLK = SYSCLKOUT / (HSPCLKDIV*CLKDIV)

   /*****ÉèÖÃCounter-Compare¼Ä´æÆ÷*****/

   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;      
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;    
   EPwm4Regs.CMPA.half.CMPA = 0;

   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;      
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;
   EPwm4Regs.CMPB = 0;
  
    /*****ÉèÖÃAction-Qualifier¼Ä´æÆ÷*****/
   
   EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;               
   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;    
   
   
            
}

void InitEPwm5(void)
{
	  /******ÉèÖÃTime_Base¼Ä´æÆ÷******/

   EPwm5Regs.TBPRD = EPWM3_TIMER_TBPRD;           // ÉèÖÃ¼ÆÊýÆ÷µÄÖÜÆÚ
   EPwm5Regs.TBPHS.half.TBPHS = 0x0000;            // ÉèÖÃÏàÎ»Îª0
   EPwm5Regs.TBCTR = 0x0000;                      // ÇåÁã¼ÆÊýÆ÷
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // ²úÉú¶Ô³Æ²¨ÐÎµÄÄ£Ê½
   EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;         // Í¨¹ýÒõÓ°¼Ä´æÆ÷·ÃÎÊ
   EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // È·¶¨EPWM3µÄÍ¬²½Ê±ÖÓÊäÈë£¬EPWM2SYNC
   EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Ê¹ÄÜÏàÎ»ÔØÈë
   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // ¸ßËÙÊ±ÖÓ·ÖÆµ
   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Ê±ÖÓ·ÖÆµTBCLK = SYSCLKOUT / (HSPCLKDIV*CLKDIV)

   /*****ÉèÖÃCounter-Compare¼Ä´æÆ÷*****/

   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;       // ´ÓÒõ°¼Ä´æÆ÷ÔØÈë±È½ÏÖµ
   EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;       // ´ÓÒõÓ°¼Ä´æÆ÷ÔØÈëÈ½ÏÖµ
   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØÈë
   EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;     // µ±CTR=ZeroÊ±ÔØÈë
   EPwm5Regs.CMPA.half.CMPA = EPWM3_TIMER_TBPRD;

  
    /*****ÉèÖÃAction-Qualifier¼Ä´æÆ÷*****/
   
   EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;                // Ôö¼ÆÊýµ½COMPAÊ±ÖÃ¸ßµçÆ½
   EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;              // ¼õ¼ÆÊýµ½COMPAÊ±ÖÃµÍµçÆ½

 

}

void InitEPwm6(void)
{
	  /******ÉèÖÃTime_Base¼Ä´æÆ÷******/

   EPwm6Regs.TBPRD = 937;                         //  150000 / 937 = 160kHz  base
   EPwm6Regs.TBPHS.half.TBPHS = 0x0000;            // ÉèÖÃÏàÎ»Îª0
   EPwm6Regs.TBCTR = 0x0000;                      // ÇåÁã¼ÆÊýÆ÷
   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Èý½Ç²¨
   EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;      
   EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     
   EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;        
   EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // ¸ßËÙÊ±ÖÓ·ÖÆµ
   EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Ê±ÖÓ·ÖÆµTBCLK = SYSCLKOUT / (HSPCLKDIV*CLKDIV)

   /*****ÉèÖÃCounter-Compare¼Ä´æÆ÷*****/

   EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;      
   EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; 
      
   EPwm6Regs.CMPA.half.CMPA = 600;

   EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;      
   EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;
   EPwm6Regs.CMPB = 0;
  
    /*****ÉèÖÃAction-Qualifier¼Ä´æÆ÷*****/
   
   EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;               
   EPwm6Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;    
   
   
   EPwm6Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;               
   EPwm6Regs.AQCTLB.bit.CAU = AQ_NO_ACTION;
   EPwm6Regs.AQCTLB.bit.PRD = AQ_NO_ACTION; 

 

	// Configure SOC for ADC with CMPA UP event of EPWM6
   EPwm6Regs.ETSEL.bit.SOCAEN  = 1;             /* Enable SOCA */
   EPwm6Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPA;  /* Enable CMPA UP event for SOCA */
   EPwm6Regs.ETPS.bit.SOCAPRD  = ET_1ST;        /* Generate SOCA on the 1st event */
   EPwm6Regs.ETCLR.bit.SOCA    = 1;             /* Clear SOCA flag */

//   Enable EPWM6 interrupt        //

   EPwm6Regs.ETSEL.bit.INTEN  = 1;             // Enable EPWM1INT generation 
   EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;    // Enable interrupt CNT_PRD event
   EPwm6Regs.ETPS.bit.INTPRD  = 1;             // Generate interrupt on the 1st event
   EPwm6Regs.ETCLR.bit.INT    = 1;             // Enable more interrupts

  
            
}



void Initial_PWM(void)
{
	EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1();    
    InitEPwm2();
    InitEPwm3();
    InitEPwm4();
	InitEPwm5();
	InitEPwm6();

	EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // open PWM clock
    EDIS;


}




void Initial_CAN(void)
{
   
   Initial_CANa();
// Initial_CANb();

}

void Initial_CANa(void)
{
	struct ECAN_REGS ECanaShadow;
	EALLOW;		// EALLOW enables access to protected bits

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/  
    //IO¿ØÖÆ¼Ä´æ
    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all; //£¬½«IO¿ÚÅäÖÃ³ÉCAN½Ó¿Ú

/* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */
									// HECC mode also enables time-stamping feature
   //Ö÷¿ØÖÆ¼Ä´æÆ÷
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.SCB = 1;  //Ñ¡ÔñeCANÄ£Ê½,¶ø²»ÊÇSCCÄ£Ê½
//	ECanaShadow.CANMC.bit.DBO=1;     //ÏÈ·¢µÍ×Ö½Ú
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

/* Initialize all bits of 'Master Control Field' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero
     //ÏûÏ¢¿ØÖÆ¼Ä´æÆ÷
    ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
//	as a matter of precaution.
    //·¢ËÍÓ¦´ð¼ÄæÆ÷
	ECanaRegs.CANTA.all	= 0xFFFFFFFF;	/* Clear all TAn bits */
    //½ÓÊÕûÏ¢¹ÒÆð¼Ä´æÆ÷
	ECanaRegs.CANRMP.all = 0xFFFFFFFF;	/* Clear all RMPn bits */
    //È«¾ÖÖÐ¶Ï±êÖ¾¼Ä´æÆ÷
	ECanaRegs.CANGIF0.all = 0xFFFFFFFF;	/* Clear all interrupt flag bits */
	ECanaRegs.CANGIF1.all = 0xFFFFFFFF;


/* Configure bit timing parameters for eCANA*/
   //Ö÷¿ØÖÆ¼Ä´æÆ÷
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1£¬ÖÃCANBTC
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
  
  
   //´íÎóºÍ×´Ì¬¼Ä´æÆ÷
    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    do
	{
	    ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );  		// Wait for CCE bit to be set..¿ÉÒÔÐ´Ä´æÆ÷
 
   
  //Î»Ê±äÅäÖ¼Ä´æÆ÷   
    ECanaShadow.CANBTC.all = 0;

                          // CPU_FRQ_100MHz is defined in DSP2833x_Examples.h
	/* The following block is only for 100 MHz SYSCLKOUT. Bit rate = 1Mbps */
	    ECanaShadow.CANBTC.bit.BRPREG = 4;
		ECanaShadow.CANBTC.bit.TSEG2REG = 2;
		ECanaShadow.CANBTC.bit.TSEG1REG = 10;
	//²¨ÌØÂÊ=150M/2/((4+1)*((2+1)+(10+1)+1))=1M

    ECanaShadow.CANBTC.bit.SAM = 1;    //Ã¿Î»Êý¾Ý²ÉÑù3´Î
    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0£¬Íê³ÉÅäÖÃCANBTC
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

   //´íÎóºÍ×´Ì¬¼Ä´æÆ÷
    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    do
    {
       ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 ); 		// Wait for CCE bit to be  cleared..½ûÖ¹Ð´¼Ä´æÆ÷

/* Disable all Mailboxes  */
 	ECanaRegs.CANME.all = 0;		///Required before writing the MSGIDs


    EDIS;
   
}
//#if (DSP28_ECANB)
void Initial_CANb(void)		// Initialize eCAN-B module
{
/* Create a shadow register structure for the CAN control registers. This is
 needed, since only 32-bit access is allowed to these registers. 16-bit access
 to these registers could potentially corrupt the register contents or return 
 false data. This is especially true while writing to/reading from a bit 
 (or group of bits) among bits 16 - 31 */

struct ECAN_REGS ECanbShadow;

   EALLOW;		// EALLOW enables access to protected bits

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/  

    ECanbShadow.CANTIOC.all = ECanbRegs.CANTIOC.all;
    ECanbShadow.CANTIOC.bit.TXFUNC = 1;
    ECanbRegs.CANTIOC.all = ECanbShadow.CANTIOC.all;

    ECanbShadow.CANRIOC.all = ECanbRegs.CANRIOC.all;
    ECanbShadow.CANRIOC.bit.RXFUNC = 1;
    ECanbRegs.CANRIOC.all = ECanbShadow.CANRIOC.all;

/* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */

	ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	ECanbShadow.CANMC.bit.SCB = 1;
//	ECanbShadow.CANMC.bit.DBO=1;     //ÏÈ·¢µÍ×Ö½Ú
	ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

/* Initialize all bits of 'Master Control Field' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

    ECanbMboxes.MBOX0.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX5.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX6.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX7.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX8.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX9.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX10.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX11.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX12.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX13.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX14.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX15.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX16.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX17.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX18.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX19.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX20.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX21.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX22.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX23.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX24.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX25.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX26.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX27.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX28.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX29.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX30.MSGCTRL.all = 0x00000000;
    ECanbMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
//	as a matter of precaution.

	ECanbRegs.CANTA.all	= 0xFFFFFFFF;	/* Clear all TAn bits */

	ECanbRegs.CANRMP.all = 0xFFFFFFFF;	/* Clear all RMPn bits */

	ECanbRegs.CANGIF0.all = 0xFFFFFFFF;	/* Clear all interrupt flag bits */
	ECanbRegs.CANGIF1.all = 0xFFFFFFFF;


/* Configure bit timing parameters for eCANB*/

	ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	ECanbShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

    ECanbShadow.CANES.all = ECanbRegs.CANES.all;

    do
	{
	    ECanbShadow.CANES.all = ECanbRegs.CANES.all;
	} while(ECanbShadow.CANES.bit.CCE != 1 ); 		// Wait for CCE bit to be  cleared..


    ECanbShadow.CANBTC.all = 0;

   
	/* The following block for all 150 MHz SYSCLKOUT - default. Bit rate = 1 Mbps */
		ECanbShadow.CANBTC.bit.BRPREG = 4;
		ECanbShadow.CANBTC.bit.TSEG2REG = 2;
		ECanbShadow.CANBTC.bit.TSEG1REG = 10;


    ECanbShadow.CANBTC.bit.SAM = 1;
    ECanbRegs.CANBTC.all = ECanbShadow.CANBTC.all;

    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	ECanbShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;

    ECanbShadow.CANES.all = ECanbRegs.CANES.all;

    do
    {
        ECanbShadow.CANES.all = ECanbRegs.CANES.all;
    } while(ECanbShadow.CANES.bit.CCE != 0 ); 		// Wait for CCE bit to be  cleared..


/* Disable all Mailboxes  */
 	ECanbRegs.CANME.all = 0;		// Required before writing the MSGIDs

    EDIS;
}
//#endif // if DSP28_ECANB


void I2CA_Init(void)
{
   // Initialize I2C
   I2caRegs.I2CMDR.all = 0x0000;	// Take I2C reset
   									// Stop I2C when suspended

   I2caRegs.I2CFFTX.all = 0x0000;	// Transmit FIFO Register£¬Disable FIFO mode and TXFIFO  
   I2caRegs.I2CFFRX.all = 0x0000;	//I2C Receive FIFO Register£¬ Disable RXFIFO, clear RXFFINT,  0x0040

   // For 150 MHz SYSCLKOUT
   I2caRegs.I2CPSC.all = 14;	    // Prescaler - need 7-12 Mhz on module clk (150/(14+1) = 10MHz)(0.1us)
 
   I2caRegs.I2CCLKL = 295;			//   Low_Time:0.1us*(295+5)=30us
   I2caRegs.I2CCLKH = 195;			//   High_Time:0.1us*(195+5)=20us Period:30us+20us=50us frequence: 20K
   I2caRegs.I2CIER.all = 0x0018;	    	//I2C Interrupt Enable Register, enable XRDY & RRDY interrupts essential

   I2caRegs.I2CMDR.all = 0x0020;	// Take I2C out of reset
   									// Stop I2C when suspended

//   I2caRegs.I2CFFTX.all = 0x7000;	// Enable FIFO mode and TXFIFO
//   I2caRegs.I2CFFRX.all = 0x3000;	// Enable RXFIFO, clear RXFFINT,

   return;
}



void Initial_SCIc(void)
{
    // Note: Clocks were turned on to the SCIC peripheral
    // in the InitSysCtrl() function

 	ScicRegs.SCICCR.all =0x0007;   //Í¨Ñ¶¿ØÖÆ¼Ä´æÆ÷ 0000 0000 0000 0111
 	                               //Ò»¸öÍ£Ö¹Î»£»ÆæÐ£Ñé£»ÆæÅ¼Ð£Ñé½ûÓÃ£»×Ô²âÊÔÄ£Ê½½ûÓÃ£»¿ÕÏÐÏßÄ£Ê½£»8×Ö½ÚÊý¾Ý£»
 	                               //1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScicRegs.SCICTL1.all =0x0003;  //¿ØÖÆ¼Ä´æÆ÷1 0000 0000 0000 0011 
								   //½ÓÊÕ´íÎóÖÐ¶Ï½ûÓÃ£»¢ËÍÌØÕ÷Î´Ñ¡Ôñ£»ÐÝÃßÄ£Ê½½ûÓÃ£Ê¹ÄÜ·¢ËÍºÍ½ÓÊÕ÷£»£»
	                               // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003;  //¿ØÖÆ¼Ä´æÆ÷2 0000 0000 0000 0011
	ScicRegs.SCICTL2.bit.TXINTENA =1;//Ê¹ÄÜTXRDYÖÐ¶Ï
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;//Ê¹ÄÜRXRDY/BRKDTÖÐ¶Ï
    ScicRegs.SCIHBAUD    =0x0000;//²¨ÌØÂÊÑ¡Ôñ¼Ä´æÆ÷ @LSPCLK = 37.5MHz 
    ScicRegs.SCILBAUD    =0x0012;//²¨ÌØÂÊ = 37.5M/8/(SCIBAUD+1) = 246710
	ScicRegs.SCICTL1.all =0x0023;     //¿ØÖÆ¼Ä´æÆ÷1 0000 0000 0010 0011
									//í¼þ¸´Î»Î»£¬µÍµçÆ½ÓÐÐ§£¬ÖÃ¸ß Relinquish SCI from Reset£»
// Initalize the SCI FIFO
    ScicRegs.SCIFFTX.bit.TXFIFOXRESET=0; //·¢ËÍFIFO¸´Î»
    ScicRegs.SCIFFRX.bit.RXFIFORESET=0;  //½ÓÊÕFIFO¸´Î»

    ScicRegs.SCIFFTX.all=0xE040;  // SCI FIFO·¢ËÍ¼Ä´æÆ÷ 1110 0000 0100 0000
    ScicRegs.SCIFFRX.all=0x204f;  // SCI FIFO½ÓÕ¼Ä´æÆ÷ 0010 0000 0100 1111
    ScicRegs.SCIFFCT.all=0x0;     //SCI FIFO¿ØÖÆ¼Ä´æÆ÷ ûÖ¹²¨ÌØÂÊ×Ô¶¯µ÷Õû£»FIFO´«ËÍÑÓ³ÙÎª0
}


void InitXintf(void)
{
    // This shows how to write to the XINTF registers.  The
    // values used here are the default state after reset.
    // Different hardware will require a different configuration.

    // For an example of an XINTF configuration used with the
    // F28335 eZdsp, refer to the examples/run_from_xintf project.

    // Any changes to XINTF timing should only be made by code
    // running outside of the XINTF.

    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = 1/2 SYSCLKOUT
    EALLOW;
    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;//ÅäÖÃ¼Ä´æÆ÷  XTIMCLK = 1/2 SYSCLKOUT
    
    XintfRegs.XINTCNF2.bit.WRBUFF = 0;//0¸öÐ´»º³åÉî¶È£¬¿ÉÅäÖÃÎª0-3
    
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;//Ê¹ÄÜXCLKOUT
    
    XintfRegs.XINTCNF2.bit.CLKMODE = 1;// XCLKOUT = XTIMCLK/2


    // Zone 0------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
   
    XintfRegs.XTIMING0.bit.XWRLEAD = 3;//Ð´·ÃÎÊ½¨Á¢µÈ´ý3¸öXTIMCLKÖÜÆÚ
    XintfRegs.XTIMING0.bit.XWRACTIVE = 4;//Ð´·ÃÎÊ¼¤»îµÈ´ý4¸öXTIMCLKÖÜÆÚ
    XintfRegs.XTIMING0.bit.XWRTRAIL = 3;//Ð´·ÃÎÊ¸ú×ÙµÈ´ý3¸öXTIMCLKÖÜÆÚ
   
    XintfRegs.XTIMING0.bit.XRDLEAD = 3;//¶Á·ÃÎÊ½¨Á¢µÈ´ý3¸öXTIMCLKÖÜÆÚ
    XintfRegs.XTIMING0.bit.XRDACTIVE = 4;//Á·ÃÎÊ¼¤»îµÈ´4¸öXTIMCLKÖÜÆÚ
    XintfRegs.XTIMING0.bit.XRDTRAIL = 3;//¶Á·ÃÎÊ¸ú×ÙµÈ´ý3¸öXTIMCLKÖÜÆÚ

    XintfRegs.XTIMING0.bit.X2TIMING = 0;//0/1:Ò»/Á½±¶½¨Á¢¡¢¼¤»îºÍ¸ú×ÙµÄÊ±¼ä

    XintfRegs.XTIMING0.bit.USEREADY = 1;//²ÉÑùXREADYÐÅºÅ 
    XintfRegs.XTIMING0.bit.READYMODE = 1;  // XREADYÐÅºÅÒì²½²ÉÑù·½Ê½

    XintfRegs.XTIMING0.bit.XSIZE = 3;//16Î»Êý¾ÝÏß£¬1±íÊ¾32Î»ý¾ÝÏß

    // Zone 6------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    
    XintfRegs.XTIMING6.bit.XWRLEAD = 3;
    XintfRegs.XTIMING6.bit.XWRACTIVE = 4;
    XintfRegs.XTIMING6.bit.XWRTRAIL = 3;
    
    XintfRegs.XTIMING6.bit.XRDLEAD = 3;
    XintfRegs.XTIMING6.bit.XRDACTIVE = 4;
    XintfRegs.XTIMING6.bit.XRDTRAIL = 3;

    XintfRegs.XTIMING6.bit.X2TIMING = 0;

    XintfRegs.XTIMING6.bit.USEREADY = 1;
    XintfRegs.XTIMING6.bit.READYMODE = 1; 

    XintfRegs.XTIMING6.bit.XSIZE = 3;

    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
 
    XintfRegs.XTIMING7.bit.XWRLEAD = 3;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 4;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 3;
    
    XintfRegs.XTIMING7.bit.XRDLEAD = 3;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 4;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 3;

    XintfRegs.XTIMING7.bit.X2TIMING = 0;

    XintfRegs.XTIMING7.bit.USEREADY = 1;
    XintfRegs.XTIMING7.bit.READYMODE = 1; 

    XintfRegs.XTIMING7.bit.XSIZE = 3;

    // Bank switching
    // Assume Zone 7 is slow, so add additional BCYC cycles
    // when ever switching from Zone 6 to another Zone.
    // This will help avoid bus contention.
    // Bank switching
    // Assume Zone 6 is slow, so add additional BCYC cycles
    // when ever switching from Zone 6 to another Zone.
    // This will help avoid bus contention.
   // XintfRegs.XBANK.bit.BANK = 6;//Ê¹ÄÜ´æ´¢Æ÷ÇøÓòÇÐ»»¹¦ÄÜ
   // XintfRegs.XBANK.bit.BCYC = 7;//Á¬Ðø·ÃÎÊ²Ù×÷Ê±ÖÐ¼äµÈ´ý6¸öXTIMCLKÖÜÆÚ
   // XintfRegs.XBANK.bit.BANK = 7;
//    XintfRegs.XBANK.bit.BCYC = 7;
    EDIS;
   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.


   asm(" RPT #7 || NOP");

}

void Initial_Gpio(void)//(ÒÑ¸Ä)
{
	 EALLOW;

//GPIO7,10,17,52,53,58-61Î´Ê¹ÓÃ

//GPIO0-GPIO5ÉèÖÃÎªÁùÂ·PWMÊä³ö
     GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
     GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)   
   
     GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
     GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

     GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
     GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)

     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

     GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
     GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)

     GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
     GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

     GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // DIS pull-up on GPIO6 (EPWM4A)
     GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // DIS pull-up on GPIO7 (EPWM4B)

     GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A

     GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;   // Configure GPIO7 as GPIO
	 GpioCtrlRegs.GPADIR.bit.GPIO7  = 1;
 	 GpioDataRegs.GPADAT.bit.GPIO7 = 0;

	 GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // DIS pull-up on GPIO8 (EPWM5A)
     GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // DIS pull-up on GPIO9 (EPWM5B)

     GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
     GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B


	 GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // DIS pull-up on GPIO10 (EPWM6A)
     GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // DIS pull-up on GPIO11 (EPWM6B)

     GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
     GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B



//  GPIO12~GPIO13  Î´Ê¹ÓÃ 


//GPIO14ºÍGPIO15ÉèÖÃÉXHOLDºÍXHOLDA	
	 GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;     //XHOLD
	 GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;     //XHOLDA 

//GPIO16 GPIO17Î´Ê¹ÓÃ   

//GPIO18ºÍGPIO19ÉèÖÃ³ÉCANRXAºÍCANTXA

	 GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;     //Enable pullup on
	 GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;     //Enable pullup on

	 GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;   //ÊäÈëÒì²½£¬ÓÃÓÚSCI£¬SPI£¬eCAN£¬I2C 
	 GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;     //CANRXA
	 GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;     //CANTXA  


//GPIO20 ~ GPIO27Î´Ê¹ÓÃ 

//GPIO32-GPIO33ÅäÖÃSDAAºÍSCLA
     GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pullup on GPIO32
     GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // GPIO32 = SDAA
     GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // ÊäÈëÒì²½£¬ÓÃÓÚSCI£¬SPI£¬eCAN£¬I2C
     GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pullup on GPIO33
     GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // ÊäÈëÒì²½£¬ÓÃÓÚSCI£¬SPI£¬eCAN£¬I2C
     GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // GPIO33 = SCLA


	 //  GPIO48 Î´Ê¹ÓÃ

	GpioCtrlRegs.GPBPUD.bit.GPIO49  = 0;                      // test port
	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO49  = 1;
	GpioDataRegs.GPBDAT.bit.GPIO49  = 0;
	
      
	GpioCtrlRegs.GPBPUD.bit.GPIO51  = 1;                      // CAP 
	GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO51  = 1;
	GpioDataRegs.GPBDAT.bit.GPIO51  = 0;


	GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;   //Enalbe pullup        LED0
	GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;  //GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;   //GPIO = output
	GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;


	GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;   //Enalbe pullup       LED1
	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;  //GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;   //GPIO = output
	GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;

	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;   //Enalbe pullup       EN
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;  //GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1;   //GPIO = output
	GpioDataRegs.GPBDAT.bit.GPIO54 = 1;

	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;   //Enalbe pullup       FPGA_RST
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;  //GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;   //GPIO = output
	GpioDataRegs.GPBDAT.bit.GPIO55 = 1;

	GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;   //Enalbe pullup       BOX_LED
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;  //GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1;   //GPIO = output
	GpioDataRegs.GPBDAT.bit.GPIO56 = 1;

// GPIO57--GPIO63 Î´Ê¹ÓÃ


//GPIO28-GPIO31,GPIO34-GPIO47,GPIO64-GPIO87ÅäÖÃÍâ²¿½Ó¿Ú£¨XINTF£©

	 GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;  // XD15
     GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;  // XD14
     GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;  // XD13
     GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3;  // XD12
     GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3;  // XD11
     GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3;  // XD10
     GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3;  // XD19
     GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3;  // XD8
     GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3;  // XD7
     GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;  // XD6
     GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3;  // XD5
     GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3;  // XD4
     GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3;  // XD3
     GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3;  // XD2
     GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3;  // XD1
     GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 3;  // XD0

     GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 3;  // XA0/XWE1n
     GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 3;  // XA1
     GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;  // XA2
     GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;  // XA3
     GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 3;  // XA4
     GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 3;  // XA5
     GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 3;  // XA6
     GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 3;  // XA7

	 /*

     GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 3;  // XA8
     GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 3;  // XA9
     GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 3;  // XA10
     GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 3;  // XA11
     GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 3;  // XA12
     GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 3;  // XA13
     GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 3;  // XA14
     GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 3;  // XA15

     GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 3;  // XA16
     GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;  // XA17
     GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;  // XA18
     GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // XA19

	 */

     GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 3;  // XREADY
	 GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 3;  // XRNW
     GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 3;  // XWE0

     GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 3;  // XZCS0
 //    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;  // XZCS7
 //    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // XZCS6



	 
	      
     EDIS;
}





void Initial_ADC(void)
{
	EALLOW;

	
	AdcRegs.ADCTRL3.all = 0x00E0;            // ADC power up
	

	delay(1000);                              //about  30 ms


    AdcRegs.ADCTRL3.bit.ADCCLKPS = 1; // ADC CLOCK = HSPCLK / 6 = 12.5MHZ 
    AdcRegs.ADCTRL1.bit.CPS = 1;

    AdcRegs.ADCTRL1.bit.ACQ_PS = 6 ;

	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0;
	AdcRegs.ADCTRL2.bit.RST_SEQ1     = 1;
	AdcRegs.ADCTRL2.bit.RST_SEQ2     = 1;      
    
      
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        //¼¶ÁªÄ£Ê½
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;    //  sequencial  sampling mode
	AdcRegs.ADCREFSEL.bit.REF_SEL = 0;    //  INNER-REF
    AdcRegs.ADCMAXCONV.all = 0x0007;      //  16Í¨µÀ


	//////////// setup sample sequence////////////
	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x4;   // A4    SIN       R0
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x6;   // A6    COS       R1

	AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x0;   // A0    U_C      R2  
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x2;   // A2    V_C      R3 


	AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x8;   // B0    BUS_V      R4
	AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0xA;   // B2    IPM_TEMP   R5
	AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0xC;   // B4    ACOM_IN    R6 
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0xE;   // B6    MOTORTEMP  R7   




	delay(1000);


    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; 
	AdcRegs.ADCST.bit.INT_SEQ1_CLR   = 1;         
 
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0x1;   // enable SOC from EPWMA trigger
 
 	EDIS;
 	
 	
 	   	
}



void ResolverParaInit(void)
{
	float  kiTsBy2;
	
	rslvrIn.SAMPLING_TIME = 1.0 / 10000;
	rslvrIn.TABLE_LENGTH  = 16;
	rslvrIn.TUNING        = 0;
	rslvrIn.FIR32         = 0;
	rslvrIn.testAngle     = 0.0;
	rslvrIn.rpsMax        = 200.0;    // max motor speed in freq
    rslvrIn.firLag        = 3;        // fbk sine and cosine peaks occur when firLag = 3

	offsetFc = 500.0 / TWO_PI;        // offset filter corner frequency (Hz)          500/2pi
	errorFc  = 500.0;                // error filter corner frequency (Hz)           1000
    piconFz  = 200.0;                 // pi controller - ZERO frequency               200
    Kp = 1500.0;                      // pi controller - prop gain                    5000


    rslvrIn.errorWfT  =  errorFc  * TWO_PI * rslvrIn.SAMPLING_TIME;

	kiTsBy2   =  Kp * (piconFz * TWO_PI) * rslvrIn.SAMPLING_TIME / 2.0;
	rslvrIn.picon_K0  =  Kp + kiTsBy2;
	rslvrIn.picon_K1  = -Kp + kiTsBy2;

	init_resolver_Float();


}


