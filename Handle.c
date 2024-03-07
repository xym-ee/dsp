#include "head.h"


#pragma CODE_SECTION(ad_sample_isr, "ramfuncs");
interrupt void ad_sample_isr(void)
{
	
	TEST_PORT  = 1;

	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;         // clear pending ADC SEQ1 INT flag 
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;           // Reset ADC SEQ1
//	AdcRegs.ADCTRL2.bit.RST_SEQ2 = 1;           // Reset ADC SEQ2


	// *** Set up next sampling instant and update excitation sine wave ***
	rslvrOut.sineIndex = (rslvrOut.sineIndex-1) & 0x000f;  
    EPwm4Regs.CMPA.half.CMPA = sineTable[rslvrOut.sineIndex];

	    // *** Sample sine and cosine feedbacks from ADC ***
	rslvrOut.sin_input = ((float)AdcRegs.ADCRESULT0*(3.0/65536.0)) - rslvrIn.offsetS;  // remove opamp bias    3/65536 = 0.0000457763671875
	rslvrOut.cos_input = ((float)AdcRegs.ADCRESULT1*(3.0/65536.0)) - rslvrIn.offsetC;  // remove opamp bias
	

	if(resolver_algo_Float())
	{
		
        SampleSumCNT=0;

		ReslvoReadyFlag=1;

	
	}
	else
	{
		


		UcurentSamp[SampleSumCNT]     = AdcRegs.ADCRESULT2;
	    VcurentSamp[SampleSumCNT]     = AdcRegs.ADCRESULT3;
	    BusVolSamp[SampleSumCNT]      = AdcRegs.ADCRESULT4;
//	    IpmTempSamp[SampleSumCNT]     = AdcRegs.ADCRESULT5;
//	    AcommSamp[SampleSumCNT]       = AdcRegs.ADCRESULT6;
//      MotorTempSamp[SampleSumCNT]   = AdcRegs.ADCRESULT7;

		SampleSumCNT++;

		if((ReslvoReadyFlag==1)&&(SampleSumCNT==1))
		{
			final_pos = rslvrOut.angleOut;   //   -1 ~  1
			
			spd_rec[spd_rec_cnt] = -rslvrOut.rpsObs;

			spd_rec_cnt++;

            if(spd_rec_cnt == 10)
			{
				spd_rec_cnt = 0;
			}
			
			EPwm6Regs.ETCLR.bit.INT = 1;                // clear EPwm6 INT flag
		}
        


	}
    
   

	PieCtrlRegs.PIEACK.all  = PIEACK_GROUP1;    // ADC ISR ACK

	TEST_PORT  = 0;
	
}



#pragma CODE_SECTION(DataProcess, "ramfuncs");
void  DataProcess(void)
{
		unsigned int tempcnt=0;

		UcurentOS=0; 
	    VcurentOS=0; 
	    BusVolOS=0;
//	    IpmTempOS=0;
//	    AcommOS=0;
//	    MotorTempOS=0;

		for(tempcnt=5;tempcnt<15;tempcnt++)
		{
			UcurentOS   += (float)UcurentSamp[tempcnt];
			VcurentOS   += (float)VcurentSamp[tempcnt];
			BusVolOS    += (float)BusVolSamp[tempcnt];
//			IpmTempOS   += (float)IpmTempSamp[tempcnt];
//			AcommOS     += (float)AcommSamp[tempcnt];
//			MotorTempOS += (float)MotorTempSamp[tempcnt];

		}
	    UcurentOS   *= 0.1;
		VcurentOS   *= 0.1;
		BusVolOS    *= 0.1;
//		IpmTempOS   *= 0.1;
//		AcommOS     *= 0.1;
//		MotorTempOS *= 0.1;
	    

		UcurentOS -= u_c_offset;
		Current_a  = UcurentOS * (1- CUR_FIL_VAR) + Current_a_1 * CUR_FIL_VAR;
		Current_a_1 = Current_a;

		
		VcurentOS -= v_c_offset;
		Current_b = VcurentOS * (1- CUR_FIL_VAR) + Current_b_1 * CUR_FIL_VAR;
		Current_b_1 = Current_b;


		BusVolOS = 0.07332  * (BusVolOS - bus_v_offset); //      200V --15949 DIGI * 0.1284 -- 2048 DIGI 



        Motor_rdc = ((final_pos + 1)*8192);  // 32768/4
		
		
		/***************   计算电机的机械和电器角度  ********************/

        if(POLE_POS_DETECT)   
		{
			Motor_rdc_angle = Motor_rdc;
		}
		else
		{
			Motor_rdc_angle = Motor_rdc - Motor_rdc_zero;
		}
		


		if(Motor_rdc_angle<0)
		{
			Motor_rdc_angle += 16383.0;
		}


		Motor_Mposition = Motor_rdc_sign * Motor_rdc_angle;
		Motor_Eposition = Motor_pair_poles * Motor_Mposition;



		Current_alfa = 1.5 * Current_a;
		Current_beta = 0.866 * (Current_a + 2 * Current_b);

		/******************* sin cos**************************/
		Sin_theta =  sin((Motor_Eposition) / 16384.0 * (2 * PI));
		Cos_theta =  cos((Motor_Eposition) / 16384.0 * (2 * PI));

		/***************   alfa-beta -> dq   ****************/
		
        Current_d = (Current_alfa * Cos_theta) + (Current_beta * Sin_theta);
			
		Current_q = (Current_beta * Cos_theta) - (Current_alfa * Sin_theta);

		/***********   CURRENT (DQ) FILTER   *************/		
		
		Current_d_filter = (Current_d_filter * (256.0 - Current_dfil) + Current_d * Current_dfil) / 256.0;
		Current_q_filter = (Current_q_filter * (256.0 - Current_qfil) + Current_q * Current_qfil) / 256.0;

//		Current_q_filter = Current_q_filter -  cur_zero_drift; 

		Current_Is  = Current_q_filter * 0.5;
		Current_Is /= CURRENT_RATIO; 

		Current_Is_SUM += fabs(Current_Is);

		Current_OL_cnt++;

		if(Current_OL_cnt == 500)  //    50ms
		{
			Current_Is_AVR = Current_Is_SUM * 0.002;
			
			Current_Is_SUM = 0;

			Current_OL_flag = 1;

			Current_OL_cnt  = 0;
		}




	    usCNT++;

	    if(usCNT == 10)
	    {
		    usCNT = 0;


			final_spd = 0;

			for(MotorSpeedCNT=0;MotorSpeedCNT<10;MotorSpeedCNT++)
			{
				final_spd += spd_rec[MotorSpeedCNT];
			}
            

			final_spd = final_spd * 6.0;            //   * 0.1 * 60  

			

		//	final_spd  =  rslvrOut.rpsObs * 60.0;

			final_spd_fil = final_spd * (1-SPD_FIL_VAR_OBS) + final_spd_last * SPD_FIL_VAR_OBS;
			final_spd_last = final_spd_fil;
            

			Motor_speed_sum=0;

            for(MotorSpeedCNT=19;MotorSpeedCNT>0;MotorSpeedCNT--)
			{
				spd_val[MotorSpeedCNT] = spd_val[MotorSpeedCNT-1];
				Motor_speed_sum +=  spd_val[MotorSpeedCNT];
			}

			spd_val[0] = final_spd_fil;

			Motor_speed_sum += spd_val[0];

			Motor_speed_fil_20ms = Motor_speed_sum * 0.05;
			



			/*
			Analog_cm   = AcommOS * (1- AD_COM_FIL_VAR) + Analog_cm_1 * AD_COM_FIL_VAR;     
			Analog_cm_1 = Analog_cm;
			*/


			Voltage_bus_fil   =  Voltage_bus_last * VOL_FIL_VAR + BusVolOS * (1- VOL_FIL_VAR);    

			Voltage_bus_last  =  Voltage_bus_fil;


            
			if(PowerState == 0)
			{
				Power_Up();

				Error_state_high |= ERR_CODE_POWERNOTREADY;
			}
			else if (PowerState == 1)
			{
				Power_Down();

				Error_state_high &= (ERR_CODE_POWERNOTREADY ^ 0xffff);
			}
			

			Error_state_high &= (ERR_CODE_POWERNOTREADY ^ 0xffff);   //  20201015
			

			Err_Handle();
			


			if(AutoUpdata == 1)
			{
		

				msgh_updata = 0;
				msgl_updata = 0;
				msg_updata_temp = 0;

			
				msgh_updata  = (int)(Motor_speed_fil_20ms * 8);

			//	msgh_updata  = (int)(final_spd_fil * 8);
					
				msgh_updata &= 0x0000ffff;
				msgh_updata  = msgh_updata << 16;

				msg_updata_temp = (int) (Current_Is * 256);       //rms   
				
			    



				msg_updata_temp &= 0x0000ffff;
				msgh_updata  |= msg_updata_temp;


				msgl_updata  = (unsigned int)( Motor_rdc_angle);

//                msgl_updata  = HistoryErr; 


				msgl_updata &= 0x0000ffff;
				msgl_updata  = msgl_updata << 16;


				msg_updata_temp = (unsigned int)(Voltage_bus_fil);
				msg_updata_temp &= 0x0000ffff;
				msgl_updata  |= msg_updata_temp;


				CAN_A_AUTO_UPDATA(msgh_updata,msgl_updata);
				

			}
		

	    }



}

/******************************ERR CODE *******************************/

/*
  
bit 15   Reserve     bit 14  rdc  err      bit 13   over load    bit  12  speed err ov

bit 11  curr err ov   bit 10   CAN  err     bit 09   power relay  bit  08  motor over heat

bit 07  IO EN        bit  06   DSP EN       bit 05   DSP FLY      bit  04  IPM ERR

bit 03   ov           bit  02   uv           bit 01    oc          bit  0   IPM OT 
 
*/

#pragma CODE_SECTION(Err_Handle, "ramfuncs"); 
void Err_Handle(void)
{
		
	float spdcmdtmp=0;
		


		Error_state_low    =  FPGA_ERR_CODE;
		Error_state_low   &=  0x00ff;

		/****************** Resolver signal quality tests ***********************/
		   
	    if (rslvrOut.resMag20 > resMagMax)
	  	  resMagMax = rslvrOut.resMag20;
	    else if (rslvrOut.resMag20 < resMagMin)
	  	  resMagMin = rslvrOut.resMag20;

	    // Degradation Of Signal (DOS) identification
	    dos = resMagMax - resMagMin;
	    if (dos > dosLimit)
		{
			DosCNT++;

			if(DosCNT > RDC_ERR_CNT_VAL)
			{
				DOS_ERROR = 1;
			}
		}
		else
		{
			DosCNT = 0;
		}
	  	  

	    // Loss Of Signal (LOS) identification
	    if (skipLosCnt < skipLosPrd)
	    	skipLosCnt++;
	    else if (rslvrOut.resMag20 < losLimit)
		{
			LosCNT++;

			if(LosCNT > RDC_ERR_CNT_VAL)
			{
				LOS_ERROR = 1;
			}

			
		}
		else
		{
			LosCNT = 0;
		}
	  	  

	    // Phase Lock Loop (PLL) Error identification
	    // abs(error angle mag) = 0 <---> _IQ(0.5)
	    if (abs(rslvrOut.errorNew20) > angleErrMax)
		{
			PLLCNT++;

			if(PLLCNT > RDC_ERR_CNT_VAL)
			{
				PLL_ERROR = 1;
			}
		}
		else
		{
			PLLCNT = 0;
			
		}

	    	

	    // reseting all error flags
	    if (resetAll == 1) {
	  	  DOS_ERROR  = 0;
	  	  LOS_ERROR  = 0;
	  	  PLL_ERROR  = 0;
	  	  resetAll   = 0;
	  	  skipLosCnt = 0;
	  	  resMagMax  = _IQ(-2.0);
	  	  resMagMin  = _IQ(2.0);

		  DosCNT = 0;
		  LosCNT = 0;
		  PLLCNT = 0;

	    }


		
		if(( DOS_ERROR == 1)||( LOS_ERROR == 1)||( PLL_ERROR == 1))
		{
			Error_state_high  |= ERR_CODE_RDC;
		}

		

        /************************************************/

		if(Soft_EN ==1)
		{
			
		  if(!POLE_POS_DETECT)
          {
          	
          	/****************** CURR CHAOCHA ***********************/

			if(Current_error_flag == 1)  // 20ms once
			{
				Current_error_flag = 0;
				
				if(   (CURRQ_CHAOCHA_CNT < CURR_CHAOCHA_TIME)  &&  (CURRD_CHAOCHA_CNT < CURR_CHAOCHA_TIME) )
				{
					if( fabs(Current_cmd) > LOW_CURRENT )
					{
						if  ( fabs(Currentq_error_avr) >  LOW_CURRENT)  
						{
							CURRQ_CHAOCHA_CNT ++;
						}
						else
						{
							CURRQ_CHAOCHA_CNT = 0;
						}

						if  ( fabs(Currentd_error_avr) >  LOW_CURRENT)  
						{
							CURRD_CHAOCHA_CNT ++;
						}
						else
						{
							CURRD_CHAOCHA_CNT = 0;
						}
					}
					else
					{
						CURRQ_CHAOCHA_CNT = 0;
						CURRD_CHAOCHA_CNT = 0;
					}
					
					
				}
				else
				{
					Error_state_high  |= ERR_CODE_CURRCHAOCHA;
				}
			


			}
			
			
			/****************** SPD CHAOCHA ***********************/

			
			

			if (SPD_CHAOCHA_CNT < SPD_CHAOCHA_TIME )
			{
				spdcmdtmp = fabs(Speed_cmd_fil);
				if( spdcmdtmp > MID_SPD )
				{
					if(  fabs(Speed_error) >  MID_SPD_CHAOCHA_VALUE )
					{
						SPD_CHAOCHA_CNT ++;
					}
					else
					{
						SPD_CHAOCHA_CNT = 0;
					}
				}
				
		//		else if(  (spdcmdtmp>LOW_SPD) && (spdcmdtmp <=MID_SPD) )
		//		{
		//			if(  fabs(Speed_error) >  LOW_SPD_CHAOCHA_FACTOR * spdcmdtmp )
		//			{
		//				SPD_CHAOCHA_CNT ++;
		//			}
		//			else
		//			{
		//				SPD_CHAOCHA_CNT = 0;
		//			}
		//		}
				
				else
				{
					SPD_CHAOCHA_CNT = 0;
				}
			}
			else
			{
				Error_state_high  |= ERR_CODE_SPEEDCHAOCHA;
			}

			
			/****************** OVER LOAD ***********************/

			if(Current_OL_flag == 1)      //   50ms once
			{
				Current_OL_flag = 0;

				if(OVER_LOAD_CNT < OVER_LOAD_TIME)
				{
					if(Current_Is_AVR > (RATED_CURRENT * 1.1) )
					{
						OVER_LOAD_CNT ++;
					}
					else
					{
						OVER_LOAD_CNT = 0;
					}
				}
				else
				{
					Error_state_high  |= ERR_CODE_OVERLOAD;
				}
			}

          	
          	/****************** CAN link off ***********************/

			/*


			if(CAN_LOST_CNT < CAN_LINKOFF_TIME)
			{
				CAN_LOST_CNT++;
			}
			else
			{
				Error_state_high  |= ERR_CODE_CANLINKOFF;
			}

			*/
			
			
			
			/****************** MOTOR OVER HEAT ***********************/

			/*

			if (MOTOR_OVERHEAT_CNT < MOTOR_OVERHEAT_TIME )
			{
				if(Motor_Real_Temp >MOTOR_OVER_HEAT_TEMP )
				{
					MOTOR_OVERHEAT_CNT++;

				}
				else
				{
					MOTOR_OVERHEAT_CNT = 0;
				}
			}	
			else
			{
					Error_state_high  |= ERR_CODE_MOTOROVERHEAT;
			}

			*/

			

          }
		
		

		} //open enable


		Error_state_high   &=  0xff00;	


		Error_state_send = (Error_state_high | Error_state_low);


		ERROR_State_temp = Error_state_send;

		ERROR_State_temp &= 0xff3f;

		

		if ( (ERROR_State_temp & Mask_Flag) != 0 )
		{
			if(Soft_EN ==1)
			{
				HistoryErr = ERROR_State_temp;
				DIS_EN();
			}
			
			
			FPGA_ERR_IO = 1;
		}
		else
		{
			FPGA_ERR_IO = 0;
		}
		
		



}
