#include "head.h"
#pragma CODE_SECTION(current_isr, "ramfuncs");
interrupt void current_isr(void)
{
        
	    float SpeedVsVol;

		IER |= M_INT1;

		EINT;

		DataProcess();

//	    if(Soft_EN == 1) // 开始能

		if ( (Soft_EN == 1) && ( (Error_state_send & Mask_Flag) == 0) ) // 开始能

		{
			       
			CURRENT_CNT++; 

			/************************ Position Loop *******************************/

			if(RUN_MODE == 1)
			{
				
				if(Position_counter == 2)
				{
				
					  Position_counter = 0;
				
				}
				

			}

			/************************ Velocity Loop *******************************/

			if ( (RUN_MODE == 1) ||  (RUN_MODE == 2)  )
			{

				if(CURRENT_CNT == 10)
				{
					CURRENT_CNT = 0;
					Position_counter ++;

					if (RUN_MODE == 1)
					{
						Speed_cmd = Position_Out;
					}
					else if (RUN_MODE == 2)
					{
						
					//	Speed_cmd = Analog_cm / 4096.0 * RATED_SPEED;
						
					
						
						if(can_ctrl == 0)
						{
							Speed_cmd = Analog_cm / 4096.0 * RATED_SPEED;
						}
						else
						{

						
							Speed_cmd = Speed_cmd_set;
							
						}
						
					}
					else
					{
						Speed_cmd = 0;
					}



					Speed_cmd = Speed_cmd * Cycle_DIR;

					

					if ( fabs(Speed_cmd - Speed_cmd_fil) >= SPEED_STEP)
					{
						
						if( Speed_cmd_fil < Speed_cmd )
						{
							Speed_cmd_fil += SPEED_STEP;
						}
						else if (Speed_cmd_fil > Speed_cmd)
						{
							Speed_cmd_fil -= SPEED_STEP;
						}


					}
					else
					{
						Speed_cmd_fil = Speed_cmd;
					}


					if(Speed_cmd_fil > SPEED_LIMIT)
					{
						Speed_cmd_fil = SPEED_LIMIT;
					}
					else if(Speed_cmd_fil < (-1 * SPEED_LIMIT) )
					{
						Speed_cmd_fil = (-1 * SPEED_LIMIT);

					}
				

	

				    Speed_motor = final_spd_fil;

					if(Speed_motor > 10000.0)
					{
						Speed_motor = 10000.0;
					}
					else if(Speed_motor < -10000.0)
					{
						Speed_motor = -10000.0;
					}

					Speed_error = Speed_cmd_fil - Speed_motor;

					Speed_Kp = Speed_Kp_Set;
					Speed_Ki = Speed_Ki_Set;
		

		            /*
							
					if(abs(Speed_error) > 500.0) 
					{
						Speed_Kp = Speed_Kp_Set;
						Speed_Ki = 0;
						Speed_error_I = 0;	
					}	
					else if((abs(Speed_error)<=500.0)&&(abs(Speed_error)>400.0))
					{
						Speed_Kp = Speed_Kp_Set;
						Speed_Ki = Speed_Ki_Set;
					}
					else if((abs(Speed_error)<=400.0)&&(abs(Speed_error)>300.0))
					{				
						Speed_Kp = Speed_Kp_Set * 1.05;
						Speed_Ki = Speed_Ki_Set * 1.05;
					}
					else if((abs(Speed_error)<=300.0)&&(abs(Speed_error)>200.0))
					{

						Speed_Kp = Speed_Kp_Set * 1.11;
						Speed_Ki = Speed_Ki_Set * 1.11;				
					}
					else if((abs(Speed_error)<=200.0)&&(abs(Speed_error)>100.0))
					{
						
						Speed_Kp = Speed_Kp_Set * 1.17;
						Speed_Ki = Speed_Ki_Set * 1.17;					
					}
					else if((abs(Speed_error)<=100.0)&&(abs(Speed_error)>0))
					{
						
						Speed_Kp = Speed_Kp_Set * 1.23;
						Speed_Ki = Speed_Ki_Set * 1.23;	

					}

					*/

					P_modify_index = 2 - 0.01 * fabs(Speed_cmd_fil);

					if(P_modify_index < 1.0)
					{
						P_modify_index = 1.0;
					}

				//	P_modify_index = 1.0;
								
					Speed_ctrl_P = 	Speed_Kp * 	Speed_error * P_modify_index;


					I_modify_index = 6 - 0.05 * fabs(Speed_cmd_fil);

					if(I_modify_index < 1.0)
					{
						I_modify_index = 1.0;
					}
				//	I_modify_index = 1.0;

					Speed_error_I = Speed_error_I + I_modify_index * Speed_error;


				//	Speed_error_I = Speed_error_I + Speed_error;

					if(Speed_error_I > 120000.0)
					{
						Speed_error_I = 120000.0;
					}	
					else if(Speed_error_I < -120000.0)
					{
						Speed_error_I = -120000.0;
					}
						
					Speed_ctrl_I = Speed_Ki * Speed_error_I;

					if(Speed_ctrl_I > Speed_ctrl_lim)  
					{
					   Speed_ctrl_I = Speed_ctrl_lim;	
					}
					else if(Speed_ctrl_I < (-1 * Speed_ctrl_lim) )
					{
					   Speed_ctrl_I = (-1 * Speed_ctrl_lim);	
					}
						

					Speed_ctrl = Speed_ctrl_P + Speed_ctrl_I;

/*
					if (Speed_error > 0 )
					{
						Speed_ctrl = Speed_ctrl + cur_zero_drift;
					}
					else if(Speed_error < 0 )
					{
						Speed_ctrl = Speed_ctrl - cur_zero_drift;
					}
					
*/


					if (  (Speed_ctrl - Speed_ctrl_last) > DELTA_CURRENT_DIGI  )
					{
						   Speed_ctrl = Speed_ctrl_last + DELTA_CURRENT_DIGI ;
					}
					else if((Speed_ctrl - Speed_ctrl_last) < (-DELTA_CURRENT_DIGI))
					{
						   Speed_ctrl = Speed_ctrl_last - DELTA_CURRENT_DIGI ;
					}
					
					Speed_ctrl_last = Speed_ctrl;


					if(Speed_ctrl > Speed_ctrl_lim)
					{
					   Speed_ctrl = Speed_ctrl_lim;	
					}
					else if(Speed_ctrl < (-1 * Speed_ctrl_lim) )
					{
					   Speed_ctrl = (-1 * Speed_ctrl_lim) ;
					}
						
				
				}



			}/*  Velocity Loop Over  */

			
			/************************ Current Loop *******************************/
			

			if  (RUN_MODE == 3) 
			{
				if(can_ctrl == 0)
				{
					Current_cmd = Analog_cm / 4096.0 * MAX_CURRENT * 2 * CURRENT_RATIO;
				}
				else
				{
					Current_cmd = Current_cmd_set;
				}
			}
			else
			{
				Current_cmd = Speed_ctrl;	
			}
			

	    	//	Current_cmd = Speed_cmd_set;              ////////////////////////////////


			Current_cmd_filter = (Current_cmd_filter * (256.0 - Current_cmdfil) + Current_cmd * Current_cmdfil) / 256.0;


			/***********   CURRENT DQ COMMAND   *************/	

		//	Currentd_cmd = 0;                                     //0
	  	//	Currentq_cmd = Current_cmd_filter ;

	    	if(SwitchCNT == 1)
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.06976); //4  
				Currentq_cmd = Current_cmd_filter * 0.997564;				
			}

			else if(SwitchCNT == 2)
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.207912); //12
				Currentq_cmd = Current_cmd_filter * 0.978148;
			}
			else if(SwitchCNT == 3)
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.275637); //16
				Currentq_cmd = Current_cmd_filter * 0.961262;	
			}
            else if(SwitchCNT == 4)
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.34202); //20
				Currentq_cmd = Current_cmd_filter * 0.939693;
			}
			else if(SwitchCNT == 5)
			{
				
				Currentd_cmd = abs(Current_cmd_filter) * (-0.406737); //24
				Currentq_cmd = Current_cmd_filter * 0.913545;
			}
			else if(SwitchCNT == 6)
			{
				
				Currentd_cmd = abs(Current_cmd_filter) * (-0.469472); //28
				Currentq_cmd = Current_cmd_filter * 0.882948;

			}


			else if(SwitchCNT == 7)
			{
				
				Currentd_cmd = abs(Current_cmd_filter) * (-0.529919); //32
				Currentq_cmd = Current_cmd_filter * 0.848048;
			}
			else if(SwitchCNT == 8)
			{
				
				Currentd_cmd = abs(Current_cmd_filter) * (-0.587785); //36     
				Currentq_cmd = Current_cmd_filter * 0.809017;

			}	
			
			else if(SwitchCNT == 9)
			{
				
				Currentd_cmd = abs(Current_cmd_filter) * (-0.64278); //40    
				Currentq_cmd = Current_cmd_filter * 0.766044;

			}	
			

			else
			{
				
				Currentd_cmd = 0;                                     //0
				Currentq_cmd = Current_cmd_filter ;
			
			} 


		/*	

			if(Voltage_bus_fil != 0)
			{
				SpeedVsVol = Speed_cmd_fil / ( (float)(Voltage_bus_fil * 0.09766) );   //  0.09766   4096--400V
			}
			else
			{
				SpeedVsVol = 0;
			}
				

			if (SpeedVsVol < 0)
			{
				SpeedVsVol *= -1;
			}


			if(SpeedVsVol<6.7)
			{
				Currentd_cmd = 0;                                     //0
				Currentq_cmd = Current_cmd_filter ;
			}
		
			else if (  (SpeedVsVol>=6.7) &&  (SpeedVsVol<6.9)  )
			{

				Currentd_cmd = abs(Current_cmd_filter) * (-0.139173); //8
				Currentq_cmd = Current_cmd_filter * 0.990268;
			}
			else if (    (SpeedVsVol>=6.9) &&  (SpeedVsVol<7.1)  )
			{
				
						
				Currentd_cmd = abs(Current_cmd_filter) * (-0.207912); //12
				Currentq_cmd = Current_cmd_filter * 0.978148;

			}
			else if (  (SpeedVsVol>=7.1) &&  (SpeedVsVol<7.3)  )
			{
								
				Currentd_cmd = abs(Current_cmd_filter) * (-0.275637); //16
				Currentq_cmd = Current_cmd_filter * 0.961262;

			}
			else if (  (SpeedVsVol>=7.3) &&  (SpeedVsVol<7.5)   )
			{


				Currentd_cmd = abs(Current_cmd_filter) * (-0.34202); //20
				Currentq_cmd = Current_cmd_filter * 0.939693;

			}
			else if( (SpeedVsVol>=7.5) &&  (SpeedVsVol<7.7)  )
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.406737); //24
				Currentq_cmd = Current_cmd_filter * 0.913545;
			}
			else if ( (SpeedVsVol>=7.7) &&  (SpeedVsVol<7.9) )
			{
					Currentd_cmd = abs(Current_cmd_filter) * (-0.469472); //28
					Currentq_cmd = Current_cmd_filter * 0.882948;
			}
			else if(  (SpeedVsVol>=7.9) &&  (SpeedVsVol<8.1)   )
			{
					Currentd_cmd = abs(Current_cmd_filter) * (-0.529919); //32
					Currentq_cmd = Current_cmd_filter * 0.848048; 
			}
			else if(  (SpeedVsVol>=8.1) &&  (SpeedVsVol<8.3)   )
			{
					Currentd_cmd = abs(Current_cmd_filter) * (-0.587785); //36     
					Currentq_cmd = Current_cmd_filter * 0.809017;
			}
			else if(   (SpeedVsVol>=8.3) &&  (SpeedVsVol<8.5)    )
			{
					Currentd_cmd = abs(Current_cmd_filter) * (-0.642788); //40    
					Currentq_cmd = Current_cmd_filter * 0.766044;
			}
			else if ( (SpeedVsVol>=8.5) &&  (SpeedVsVol<8.7)   )
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.694658); //44    
				Currentq_cmd = Current_cmd_filter * 0.719340;
			}
			else if (  (SpeedVsVol>=8.7) &&  (SpeedVsVol<8.9)    )
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.743145); //48
				Currentq_cmd = Current_cmd_filter * 0.669131;		
			}
			else if (  (SpeedVsVol>=8.9) &&  (SpeedVsVol<9.1)   )
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.788011); //52
				Currentq_cmd = Current_cmd_filter * 0.615661;
			}
			else if (  (SpeedVsVol>=9.1) &&  (SpeedVsVol<9.5)   )
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.829038); //56
				Currentq_cmd = Current_cmd_filter * 0.559193;
			}
			else 
			{
				Currentd_cmd = abs(Current_cmd_filter) * (-0.866025); //60
				Currentq_cmd = Current_cmd_filter * 0.5; 

			//	Currentd_cmd = abs(Current_cmd_filter) * (-0.882948); //62
			//	Currentq_cmd = Current_cmd_filter * 0.469472; 
			}
			
            */





			Currentd_error = Currentd_cmd - Current_d_filter;
			Currentq_error = Currentq_cmd - Current_q_filter;


			Currentd_error_sum += Currentd_error;
			Currentq_error_sum += Currentq_error;
			
			Current_error_cnt++;

			if (Current_error_cnt == 200)
			{
				Currentd_error_avr = Currentd_error_sum * 0.005;
				Currentq_error_avr = Currentq_error_sum * 0.005;

				Currentd_error_sum = 0;
				Currentq_error_sum = 0;
				Current_error_cnt = 0;
				Current_error_flag = 1;

			}



			Currentd_ctrl_P = Currentd_Kp * Currentd_error; 
			Currentq_ctrl_P = Currentq_Kp * Currentq_error; 

	        Currentd_error_I = Currentd_error_I + Currentd_error;

			if (Currentd_error_I > 500000.0) //50000  20160914 改500000
			{
			   Currentd_error_I = 500000.0;	
			}	         
			else if(Currentd_error_I < - 500000.0)
			{
			   Currentd_error_I = - 500000.0;	
			}
				  

			Currentq_error_I = Currentq_error_I + Currentq_error;

			if (Currentq_error_I > 500000.0)   //50000
			{
				Currentq_error_I = 500000.0; 
			}
			else if(Currentq_error_I < - 500000.0)
			{
				Currentq_error_I = - 500000.0;
			}
			
			Currentd_ctrl_I =  Currentd_Ki * Currentd_error_I;
					
			if (Currentd_ctrl_I > 32767.0)
			{
				Currentd_ctrl_I = 32767.0;         //Currentd_ctrlI_lim = 32767.0
			}	
			else if(Currentd_ctrl_I < - 32767.0)
			{
				Currentd_ctrl_I = - 32767.0;
			}
				  
				 
			Currentq_ctrl_I =  Currentq_Ki * Currentq_error_I;
				
			if (Currentq_ctrl_I > 32767.0)
			{
				Currentq_ctrl_I = 32767.0;         //Currentq_ctrlI_lim = 32767.0
			}	
			else if(Currentq_ctrl_I < - 32767.0)
			{
				Currentq_ctrl_I = - 32767.0;
			}
				     


			Currentd_ctrl = Currentd_ctrl_P + Currentd_ctrl_I;

			if (Currentd_ctrl > 32767.0)
			{
				Currentd_ctrl = 32767.0;         //Currentd_ctrl_lim = 32767.0
			}	
			else if(Currentd_ctrl < - 32767.0)
			{
				Currentd_ctrl = - 32767.0; 
			}	

	        Currentq_ctrl = Currentq_ctrl_P + Currentq_ctrl_I;

			if (Currentq_ctrl > 32767.0)
			{
				Currentq_ctrl = 32767.0;         //Currentq_ctrl_lim = 32767.0
			}
			else if(Currentq_ctrl < - 32767.0)
			{
				Currentq_ctrl = - 32767.0;
			}

			/*******************   dq -> alfa-beta   *****************/
		
	        Current_alfa_ctrl = Currentd_ctrl * Cos_theta - Currentq_ctrl * Sin_theta;
			Current_beta_ctrl = Currentd_ctrl * Sin_theta + Currentq_ctrl * Cos_theta;

			Current_a_ctrl = Current_alfa_ctrl* 2.0 / 3.0;
	 
			if (Current_a_ctrl > 32767.0)    //32767
			{
			   Current_a_ctrl = 32767.0;	
			}
			else if(Current_a_ctrl < - 32767.0)
			{
			   Current_a_ctrl = - 32767.0;	
			}

			Current_b_ctrl =  0.577350 * Current_beta_ctrl -  Current_alfa_ctrl / 3.0; 

			if (Current_b_ctrl > 32767.0)   //32767
			{
				Current_b_ctrl = 32767.0;
			}
			else if(Current_b_ctrl < - 32767.0)
			{
				Current_b_ctrl = - 32767.0;
			}

			Current_c_ctrl = -1 * (Current_a_ctrl + Current_b_ctrl);

			if (Current_c_ctrl > 32767.0)    //32767
			{
				Current_c_ctrl = 32767.0;
			}		         
			else if(Current_c_ctrl < - 32767.0)
			{
				Current_c_ctrl = - 32767.0; 
			}


			/*******************   PWM CONTROL   *****************/




			if(POLE_POS_DETECT)   
			{
				/*****************测试初始零位 上230V电压************/
		
					PWM_set = Speed_cmd_set;
					                                                                 //  参考值 1000                                                                  
		 			if(PWM_set > 2000)                                                                                        
		 			{                                                                                                         
		 				PWM_set = 2000;                                                                                       
		 			}                                                                                                         
		 			else if (PWM_set < 0)                                                                                     
		 			{                                                                                                         
		 				PWM_set = 0;                                                                                          
		 			}
							
					pwmout_a = PWM_set;  
					pwmout_b = 0;
					pwmout_c = 0;
							
		
			}
			else
			{
			    

					pwmout_a = Current_a_ctrl;
					pwmout_b = Current_b_ctrl;
					pwmout_c = Current_c_ctrl;
               
				
				/*
					pwmout_a = (600.0 * sin( ( p_time_cnt) * PI / 20000.0 * 10) );  //5000 对应 999
					pwmout_b = (600.0 * sin( ( p_time_cnt) * PI / 20000.0 * 10 + 2 * PI / 3.0 ) );
					pwmout_c = (600.0 * sin( ( p_time_cnt) * PI / 20000.0 * 10 + 4 * PI / 3.0 ) );	

					p_time_cnt = p_time_cnt+1;
					if(p_time_cnt > 3999)
					{
						p_time_cnt = 0;
					}

				 */
				

			}

		
			pwmout_max = 4055.0;     //对应55A

			if (pwmout_a > pwmout_max)
			{
				pwmout_a = pwmout_max;	
			}
			else if(pwmout_a < -pwmout_max)
			{
				pwmout_a = -pwmout_max;
			} 
				

			if (pwmout_b > pwmout_max)
			{
				pwmout_b = pwmout_max;
			}	
			else if(pwmout_b < -pwmout_max)
			{
				pwmout_b = -pwmout_max;
			} 
			
			if (pwmout_c > pwmout_max)
			{
				pwmout_c = pwmout_max;
			}	
			else if(pwmout_c < -pwmout_max)
			{
				pwmout_c = -pwmout_max;
			} 
				


			pwmcmp_a = (pwmout_a / pwmout_max) * 3750.0 + 3750.0;     //pwmout_max 对应 EPWM1_TIMER_TBPRD - 0.5 * DB_PRD_UP = 7000
			pwmcmp_b = (pwmout_b / pwmout_max) * 3750.0 + 3750.0;     //-pwmout_max 对应  0.5 * DB_PRD_DOWN = 500
			pwmcmp_c = (pwmout_c / pwmout_max) * 3750.0 + 3750.0;



			EPwm1Regs.CMPA.half.CMPA = (7500 - pwmcmp_a);
   	    	EPwm2Regs.CMPA.half.CMPA = (7500 - pwmcmp_b);
	  	    EPwm3Regs.CMPA.half.CMPA = (7500 - pwmcmp_c);




		}




		/*******************    LED  & CAP    **********************/

		

		LED_CNT++;
		if (LED_CNT > 4000)
		{
			LED_CNT = 0;
					
			LED0_TOGGLE = 1;  //BOARD LED

			if(Soft_EN == 1)
			{
				BOX_LED_TOGGLE  = 1;  //BOX LED
			
			}
			else
			{
				BOX_LED_PORT   = 1;  // BOX LED
			}


		}

		FLIP_TOGGLE = 1;      //  CAP  toggle


		ReslvoReadyFlag=0;

    
        // Acknowledge this interrupt to receive more interrupts from group 1
        
        PieCtrlRegs.PIEACK.all  = PIEACK_GROUP3;    // EPWM3 ISR ACK  

		
  		
		 


}

