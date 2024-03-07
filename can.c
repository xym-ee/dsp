#include "head.h"

void Config_CAN(void)
{
  
  
  
   struct ECAN_REGS ECanaShadow;
//   struct ECAN_REGS ECanbShadow;   

    /* 设置CANA  */
	

/* Write to the MSGID field  */

   ECanaMboxes.MBOX0.MSGID.all = CANID_DRV_R;   // Extended Identifier 0x00000000
   ECanaMboxes.MBOX1.MSGID.all = CANID_DRV_T;   // Extended Identifier 0x00000001
   ECanaMboxes.MBOX2.MSGID.all = CANID_DRV_T_1; // Extended Identifier 0x00000002
/* Configure Mailbox under test as a Transmit mailbox */

   ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
   ECanaShadow.CANMD.bit.MD0 = 1;
   ECanaShadow.CANMD.bit.MD1 = 0;
   ECanaShadow.CANMD.bit.MD2 = 0;
   ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

/* Enable Mailbox under test */

   ECanaShadow.CANME.all = ECanaRegs.CANME.all;
   ECanaShadow.CANME.bit.ME0 = 1;
   ECanaShadow.CANME.bit.ME1 = 1;
   ECanaShadow.CANME.bit.ME2 = 1;
   ECanaRegs.CANME.all = ECanaShadow.CANME.all;

/* Write to DLC field in Master Control reg */

   ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 7;
   ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 7;
   ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;




   /* 设置CANB  */
 
	
   /*

   ECanbMboxes.MBOX0.MSGID.all = CANID_DRV_MRST;        
   ECanbMboxes.MBOX1.MSGID.all = CANID_DRV_MTSR;           
  
   
   ECanbShadow.CANMD.all = ECanbRegs.CANMD.all; // 邮箱方向寄存器RECEIVE OR TRANSMIT
   ECanbShadow.CANMD.bit.MD0   = 1;               // 邮箱方向寄存器：接收RECEIVE
   ECanbShadow.CANMD.bit.MD1   = 0;               // 邮箱方向寄存器：发送TRANSMIT
   ECanbRegs.CANMD.all = ECanbShadow.CANMD.all;

   ECanbShadow.CANME.all = ECanbRegs.CANME.all;///邮箱激活寄存器enable mbx
   ECanbShadow.CANME.bit.ME0 = 1;
   ECanbShadow.CANME.bit.ME1 = 1;
   ECanbRegs.CANME.all = ECanbShadow.CANME.all;// FRAME LENTH

   ECanbMboxes.MBOX0.MSGCTRL.bit.DLC = 8;      //8字节
   ECanbMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
   */




}

#pragma CODE_SECTION(CAN_A_RX,"ramfuncs");
void CAN_A_RX(void)                                              //    接收 和 发送 7 byte 都是左对齐
{
	struct ECAN_REGS ECanaShadow;
	unsigned int data_temp,data_temp1,para_temp;
	float speedsteptemp;

	 if(ECanaRegs.CANRMP.bit.RMP0 == 1 )//邮箱中包含一个接收到的消息
	  {

		CAN_LOST_CNT = 0;
		msgh = ECanaMboxes.MBOX0.MDL.all ;//低字节数据；0-7位0字节；8-15位1字节；16-23位2字节；24-31位3字节；
		msgl = ECanaMboxes.MBOX0.MDH.all ;
		ECanaShadow.CANRMP.bit.RMP0 = 1;//写1清除RMP0位
		ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;//写1清除RMP0位

		motor_id = ( (msgh & 0xff000000) >> 24 );

		pc_command_id =( (msgh & 0x00ffff00) >> 8 );

		data_temp    =  msgh & 0x000000ff ;
		data_temp  <<=  8;

		data_temp1   =  ( (msgl & 0xff000000)>>24 );
		data_id      =  (data_temp | data_temp1);

		if ( (motor_id == motornum) || (motor_id == 0) )
		{
			switch (pc_command_id)
			{
			    	case   COM_CAN_CTRL         :   if(Soft_EN == 0)
				                                    {
				                                    	if(data_id == DAT_CAN_CTRL)
														{
															can_ctrl=1;

                                                            I2C_Write(ADDRESS_CAN_AD,1); 

														//	CAN_A_TX(0x01000000,0x00aa5100);

															CAN_MSG_MERGE(motornum,COM_CAN_CTRL,0,0xaa51);   

													    	CAN_A_TX(msgh_tx,msgl_tx);
														}
				                                    }    
				                                    break;

			    	case   COM_AD_CTRL          :   if(Soft_EN == 0)
				                                    {
				                                    	if(data_id == DAT_AD_CTRL)
														{
															can_ctrl=0;

															I2C_Write(ADDRESS_CAN_AD,0);

														//	CAN_A_TX(0x01c00900,0x00876400);

															CAN_MSG_MERGE(motornum,COM_AD_CTRL,0,0x8764);   

													    	CAN_A_TX(msgh_tx,msgl_tx);

															
														}
				                                    }    
				                                    break;

			    	case  COM_EN                :   if(data_id == DAT_EN)
			                                        {
			                                    	 
			                                    	 	if( ( (Error_state_send & 0x7f3f) & Mask_Flag) == 0)
														{
															FPGA_FAN = BRAKE_ON;

															delay(10);
															
															OPEN_EN();
														}

				                                    	 	
													//	 CAN_A_TX(0x01c00300,0x0040a500);

														CAN_MSG_MERGE(motornum,COM_EN,0,0x40a5);   

												    	CAN_A_TX(msgh_tx,msgl_tx);

													 }
												
													break;

			    	case  COM_DIS_EN            :   if(data_id == DAT_DIS_EN)
			                                        {
			                                    	 
													     DIS_EN();


														 initial_para_self();
														 
													//	 CAN_A_TX(0x01c00400,0x00c53500);
													
														CAN_MSG_MERGE(motornum,COM_DIS_EN,0,0xc535);   

												    	CAN_A_TX(msgh_tx,msgl_tx);														 
													
											         }
				                                    
													break;
													 
					case  COM_BRAKE            :    if(data_id == BRAKE_ON)
													{
														  FPGA_FAN = BRAKE_ON;

														  CAN_MSG_MERGE(motornum,COM_BRAKE,BRAKE_ON,0x0000);   

												    	  CAN_A_TX(msgh_tx,msgl_tx);
													}
													else if(data_id == BRAKE_OFF)
													{
														  FPGA_FAN = BRAKE_OFF;

														  CAN_MSG_MERGE(motornum,COM_BRAKE,BRAKE_OFF,0x0000);

														  CAN_A_TX(msgh_tx,msgl_tx);
													}

					                                break;


					case  COM_RUN_MODE         :  //  if(Soft_EN == 0)
				                                  //  {
												        /*
				                                    	if(data_id == DAT_POS_MODE)
														{
															 
															 
															 RUN_MODE  = 1;

                                                             I2C_Write(ADDRESS_MODE,RUN_MODE);

															// CAN_A_TX(0x01802100,0x01d6be00);

														  	 CAN_MSG_MERGE(motornum,COM_RUN_MODE,1,0xd6be);   

													    	 CAN_A_TX(msgh_tx,msgl_tx);

														}
														else
														*/
														if(data_id == DAT_VEL_MODE)
														{
															 if(Soft_EN == 0)
															 {
															 	RUN_MODE  = 2;

															 	I2C_Write(ADDRESS_MODE,RUN_MODE);
															 }
															 else if(Soft_EN == 1)
															 {
															 	 DIS_EN();

																 RUN_MODE  = 2;

															 	 I2C_Write(ADDRESS_MODE,RUN_MODE);

																 delay(10);

																 OPEN_EN();
															 }
													

															 CAN_MSG_MERGE(motornum,COM_RUN_MODE,2,0xe6dd);   

													    	 CAN_A_TX(msgh_tx,msgl_tx);

														}
														else if(data_id == DAT_CUR_MODE)
														{
															 
															 if(Soft_EN == 0)
															 {
															 	RUN_MODE  = 3;

														//	 	I2C_Write(ADDRESS_MODE,RUN_MODE);
															 }
															 else if(Soft_EN == 1)
															 {
															 	DIS_EN();

																RUN_MODE  = 3;

														//	 	I2C_Write(ADDRESS_MODE,RUN_MODE);

																delay(10);

																OPEN_EN();
															 }
															 

															 CAN_MSG_MERGE(motornum,COM_RUN_MODE,3,0xf6fc);   

													    	 CAN_A_TX(msgh_tx,msgl_tx);

														}



				                                   // }
													break; 

				     	case COM_COM             :  if(Soft_EN == 1)
													{
														if(RUN_MODE == 1)
					                                    {
					                                    	Position_Cycle =  ((data_id & 0x7f80)>>7); // bit15 dir  bit 7 ~ bit 14 cycle   bit 0~ bit6 position
															Position_Delta =    data_id & 0x007f;
															Position_Cycle_DIR = data_id & 0x8000;     //  0 or 0x8000

															if(Position_Cycle_DIR == 0)
															{
																Position_Cycle_DIR = 1;
															}
															else
															{
																Position_Cycle_DIR = -1;
															}

															CAN_A_TX(msgh,msgl);

					                                    }
														else if(RUN_MODE == 2)
														{
														    if(fabs(data_id)<=27648)
															{
																SinTestFlag = 0;
																SinFreq = 0;
																Speed_cmd_set = data_id * RATED_SPEED / 27648.0;                      // 0x6c00  full speed  scale 
															}
															

															else
															{
																SinTestFlag = 0;
																Speed_cmd_set = 0;
																SinFreq = 0;

															}

														
															
														    

															CAN_A_TX(msgh,msgl);

														}
														else if(RUN_MODE == 3)
														{
														    Current_cmd_set = data_id * MAX_CURRENT * 2 * CURRENT_RATIO / 27648.0;  // 0x6c00  full current scale 

															CAN_A_TX(msgh,msgl);
														}


													}	
													break;

					case  COMAND_SET_MOTORNUM  :      if (Soft_EN == 0)
													  {
													  	 if( (data_id>0) && (data_id<=255)  )
														 {
														 	 motornum = data_id;

															 I2C_Write(ADDRESS_MOTORNUM,motornum);

														 }

														 CAN_MSG_MERGE(motornum,COMAND_SET_MOTORNUM,motornum,0);   

													     CAN_A_TX(msgh_tx,msgl_tx);

													  }

													  break;

				    case  COMAND_POLEPAIRS     :      if (Soft_EN == 0)
					                                  {
					                                  	 if( (data_id>0) && (data_id<33)  )
					                                     {
						                                  
						                                    Motor_pair_poles = data_id;

															I2C_Write(ADDRESS_POLEPAIRS,Motor_pair_poles);

															
					                                     }
					                                     
					                                     CAN_A_TX(msgh,msgl);					                               
					                                  
					                                  } 
													  break;


				    case  COMAND_DIR           :      if (Soft_EN == 0)
					                                  {
					                                  	  
						                                  if(data_id == 0)
						                                  {
						                                  	Cycle_DIR = 1;
						                                  }
														  else
														  {
														  	Cycle_DIR = -1;
														  }

                                                          I2C_Write(ADDRESS_DIR,data_id);

														  CAN_A_TX(msgh,msgl);
					                                  }

													  break;


					case  COMAND_SET_KPC          :   if (Soft_EN == 0)
					                                  {
					                                  		para_temp = data_id;
					                                  		
															if(para_temp<32000)
															{
																 I2C_Write(ADDRESS_KPC,para_temp);

							                               		 Currentd_Kp = para_temp/ 512.0;
							                               		 Currentq_Kp = Currentd_Kp;

																 
															}

															CAN_A_TX(msgh,msgl);

							                                
							                                  
					                                  }
					                                  break;

					case  COMAND_SET_KIC          :   if (Soft_EN == 0)
					                                  {
					                                  		para_temp = data_id;

															if(para_temp<32000)
															{
															    I2C_Write(ADDRESS_KIC,para_temp);

								                          		Currentd_Ki = para_temp/ 1024.0;
								                            	Currentq_Ki = Currentd_Ki;

																	
															}

															CAN_A_TX(msgh,msgl);

							                                
							                                  	
					                                  }				                                  
					                                  break;

					case  COMAND_DRIFT_CUR        :   if (Soft_EN == 0)
					                                  {
					                                  		if  ( (data_id>-10240)&&(data_id<10240)  )
															{
																 I2C_Write(ADDRESS_DRIFT,data_id);

																 SwitchCNT  =   	(unsigned int)(data_id/1024.0);

							                                    // cur_zero_drift = data_id * CURRENT_RATIO / 1024.0;            //  /1024 * (4055  /  55A)
															//	cur_zero_drift = ((float)data_id) * 2.0 * CURRENT_RATIO / 1024.0;            //  /1024 * (4055  /  55A)

														         	
															}

															CAN_A_TX(msgh,msgl);

							                                  	
					                                  }
					                                  break;
					
					case  COMAND_LIM_CUR          :   if (Soft_EN == 0)
					                                  {
					                                  		
                                                             if  ( (data_id>=0)&&(data_id<=100)  )
															 {
															 	I2C_Write(ADDRESS_LIM_CUR,data_id);
							                                 	Speed_ctrl_lim =  data_id * MAX_CURRENT * 2 * CURRENT_RATIO / 100.0;          //  lim *  ( 4055 / 55 )

															 	
															 }

															 CAN_A_TX(msgh,msgl);

							                                 
							                                  
					                                  }
					                                  break;


					case  COMAND_SET_KPV         :    if (Soft_EN == 0)
					                                  {
					                                  		  para_temp = data_id;

															  if(para_temp<32000)
															  {
																  I2C_Write(ADDRESS_KPV,para_temp);

								                                  Speed_Kp_Set =  para_temp/ 512.0;

															      
															  }

															  CAN_A_TX(msgh,msgl);

								                                  
							                                  
					                                  }
													  break;
					                                  

                    case  COMAND_SET_KIV         :    if(Soft_EN == 0)
					                                  {
					                                  		   para_temp = data_id;
                                                               
															   if(para_temp<32000)
															   {
															   		I2C_Write(ADDRESS_KIV,para_temp);
			                                                   		Speed_Ki_Set =  para_temp/ 1024.0;
								                              		
															   }

															   CAN_A_TX(msgh,msgl); 

							                                    
							                                  
					                                  }
                                                      
													  break;
                                                      

				    case  COMAND_SR_VEL          :    if (Soft_EN == 0)
					                                  {
					                                  		// 
						                                      speedsteptemp = data_id / 512.0;

							                                  if(speedsteptemp >= 0)
															  {
															  	 
																  	if(speedsteptemp < 0.001)
																	{
																		speedsteptemp = 0.001;
																	}
																	else if (speedsteptemp > 30)
																	{
																		speedsteptemp = 30.0;
																	}

																	I2C_Write(ADDRESS_SRV,(unsigned int)(speedsteptemp * 512.0));
																	
																	SPEED_STEP = RATED_SPEED * 0.001 / speedsteptemp;

															  }
															  CAN_A_TX(msgh,msgl); 
					                                  }
				                                      
					                                  break;

				    case  COMAND_LIM_VEL         :    if (Soft_EN == 0)
					                                  {
					                                  	  if( (data_id>=0) && (data_id<=100) )
						                                  {
						                                  	I2C_Write(ADDRESS_LIM_VEL,data_id);
					                                        SPEED_LIMIT =   data_id * RATED_SPEED / 100.0; 
						                                  }
														  CAN_A_TX(msgh,msgl); 

					                                  }
				                                      
					                                  break;

					case  COMAND_SET_KPP         :    if (Soft_EN == 0)
					                                  {
					                                  	   para_temp = data_id;	

														   if(para_temp<32000)
														   {
														   	 I2C_Write(ADDRESS_KPP,para_temp);
								                             Position_Kp_Set =  para_temp/ 512.0;
								                            
								                              	
														   }

														   CAN_A_TX(msgh,msgl);
							                                     
							                                  
					                                  }
					                                  
													  break;
					                                  

                    case  COMAND_SET_KIP         :    if (Soft_EN == 0)
					                                  {
					                                  	 	 para_temp = data_id;

															 if(para_temp<32000)
															 {
															 	I2C_Write(ADDRESS_KIP,para_temp);
			                                                 	Position_Ki_Set =  para_temp/ 1024.0;
															 }
							                                 

															  CAN_A_TX(msgh,msgl); 
								                                  
							                                  
					                                  }
                                                      
													  break;
													  
					case COMAND_CLR_ERR             : if (Soft_EN == 0)
					                                  {
					                                  	 	  if(data_id == 0)
															  {
															  	
															  	FPGA_Reset();

																Error_state_high &= 0x0200;
																CAN_LOST_CNT = 0;
																MOTOR_OVERHEAT_CNT = 0;
																CURRQ_CHAOCHA_CNT = 0;
																CURRD_CHAOCHA_CNT = 0;
																Currentd_error_sum = 0;
															    Currentq_error_sum = 0;
																Current_error_cnt = 0;
																Currentq_error_avr = 0;
																Currentd_error_avr = 0;
																SPD_CHAOCHA_CNT =0;
																OVER_LOAD_CNT = 0;
																Current_OL_cnt=0;
																Current_Is_SUM=0;
																Current_Is_AVR=0;

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

															  	CAN_A_TX(msgh,msgl);
															  }
							                                 
        
							                                  
					                                  }
                                                      
													  break;





			        case   CHECK_VEL                :  if(data_id == DAT_UPDATA)
				                                       {
				                                  		    CAN_MSG_MERGE(motornum,CHECK_VEL,(int)(Motor_speed_fil_20ms * 8),0);               //   real speed * 8

													    	CAN_A_TX(msgh_tx,msgl_tx);
				                                       }

													   break;

					case   CHECK_CURR              :  if(data_id == DAT_UPDATA)
				                                      {
				                                  		    CAN_MSG_MERGE(motornum,CHECK_CURR,(int) (Current_Is * 256),0);   //  real current * 256

													    	CAN_A_TX(msgh_tx,msgl_tx);
				                                      }

													   break;

				    case   CHECK_POS                : if(data_id == DAT_UPDATA)
				                                      {
				                                  		    CAN_MSG_MERGE(motornum,CHECK_POS,(unsigned int)( Motor_rdc_angle),0);   

													    	CAN_A_TX(msgh_tx,msgl_tx);
				                                      }

													  break;

				    case  CHECK_BUS_VOL            :  if(data_id == DAT_UPDATA)
				                                      {
				                                  		    CAN_MSG_MERGE(motornum,CHECK_BUS_VOL,(unsigned int)(Voltage_bus_fil),0);   

													    	CAN_A_TX(msgh_tx,msgl_tx);
				                                      }

													  break;


				    case  CHECK_ERR_STATE          :  if(data_id == DAT_UPDATA)
				                                      {
				                                  		    CAN_MSG_MERGE(motornum,CHECK_ERR_STATE,Error_state_send,0);   

													    	CAN_A_TX(msgh_tx,msgl_tx);
				                                      }

													  break;
													  
					case  CHECK_IPM_TEMP           : if(data_id == DAT_UPDATA)
				                                     {
 														
															CAN_MSG_MERGE(motornum,CHECK_IPM_TEMP,0,0);   

													    	CAN_A_TX(msgh_tx,msgl_tx);

				                                     }	
				                                     break;			

                    case  CHECK_PARA_SET           : if(data_id == DAT_UPDATA)
				                                     {
 															if (Soft_EN == 0)
													  		{
																	PARA_SET_UPDATA();
															}

				                                     }

													  break;

					case  CHECK_UPDATA_ON          :  if(data_id == DAT_UPDATA)
													  {
													  		AutoUpdata = 1;

														
													  }
													  break;

				    case  CHECK_UPDATA_OFF          : if(data_id == DAT_UPDATA)
													  {
													  		AutoUpdata = 0;

															

															if (Soft_EN == 0)
															{
																FPGA_Reset();
																Error_state_high &= 0x0200;
																CAN_LOST_CNT = 0;
																MOTOR_OVERHEAT_CNT = 0;
																CURRQ_CHAOCHA_CNT = 0;
																CURRD_CHAOCHA_CNT = 0;
																Currentd_error_sum = 0;
															    Currentq_error_sum = 0;
																Current_error_cnt = 0;
																Currentq_error_avr = 0;
																Currentd_error_avr = 0;
																SPD_CHAOCHA_CNT =0;
																OVER_LOAD_CNT = 0;
																Current_OL_cnt=0;
																Current_Is_SUM=0;
																Current_Is_AVR=0;

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
															
													  }
													  break;


			        default                        :  break; 



			}
		}

		


	  }

}


#pragma CODE_SECTION(CAN_A_TX,"ramfuncs");
void CAN_A_TX(unsigned long int mailh,unsigned long int maill)    //     mailh 11 22 33 44  maill 55 66 77 88   则发送 11 22 33 44 55 66 77
{
	struct ECAN_REGS ECanaShadow;
	
    ECanaMboxes.MBOX1.MDL.all = mailh;
    ECanaMboxes.MBOX1.MDH.all = maill;
	ECanaShadow.CANTRS.all = 0;
	ECanaShadow.CANTRS.bit.TRS1 = 1;             // Set TRS for mailbox under test
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
}

#pragma CODE_SECTION(CAN_A_AUTO_UPDATA,"ramfuncs");
void CAN_A_AUTO_UPDATA(unsigned long int mailh,unsigned long int maill)
{
	struct ECAN_REGS ECanaShadow;
	
	ECanaMboxes.MBOX2.MDL.all = mailh;
    ECanaMboxes.MBOX2.MDH.all = maill;
	ECanaShadow.CANTRS.all = 0;
	ECanaShadow.CANTRS.bit.TRS2 = 1;             // Set TRS for mailbox under test
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
}





#pragma CODE_SECTION(CAN_MSG_MERGE,"ramfuncs");
void CAN_MSG_MERGE(unsigned int id,unsigned int code,unsigned int data,unsigned int dataplus)
{
	msg_tx_temp  = id;
	msg_tx_temp  = msg_tx_temp << 16;
	msg_tx_temp |= code;
	msg_tx_temp  = msg_tx_temp << 8;
	msg_tx_temp |= (data>>8);

    msgh_tx = msg_tx_temp;


	msg_tx_temp = data;
	msg_tx_temp  = msg_tx_temp << 16;
	msg_tx_temp |= dataplus;
	msg_tx_temp  = msg_tx_temp << 8;

	msgl_tx = msg_tx_temp;



}


void PARA_SET_UPDATA(void)
{
	unsigned int readtemp;


	CAN_MSG_MERGE(motornum,COM_CAN_CTRL,can_ctrl,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 


	CAN_MSG_MERGE(motornum,COM_RUN_MODE,RUN_MODE,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500);



	CAN_MSG_MERGE(motornum,COMAND_SET_MOTORNUM,motornum,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500);


	CAN_MSG_MERGE(motornum,COMAND_POLEPAIRS,Motor_pair_poles,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 



	readtemp = I2C_Read(ADDRESS_DIR);

	CAN_MSG_MERGE(motornum,COMAND_DIR,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 


	readtemp = I2C_Read(ADDRESS_KPC);

	CAN_MSG_MERGE(motornum,COMAND_SET_KPC,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 



	readtemp = I2C_Read(ADDRESS_KIC);

	CAN_MSG_MERGE(motornum,COMAND_SET_KIC,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 



	readtemp = I2C_Read(ADDRESS_DRIFT);

	CAN_MSG_MERGE(motornum,COMAND_DRIFT_CUR,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 



	readtemp = I2C_Read(ADDRESS_LIM_CUR);

	CAN_MSG_MERGE(motornum,COMAND_LIM_CUR,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 	



	readtemp = I2C_Read(ADDRESS_KPV);

	CAN_MSG_MERGE(motornum,COMAND_SET_KPV,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 



	readtemp = I2C_Read(ADDRESS_KIV);

	CAN_MSG_MERGE(motornum,COMAND_SET_KIV,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 



	readtemp = I2C_Read(ADDRESS_SRV);

	CAN_MSG_MERGE(motornum,COMAND_SR_VEL,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500);  



	readtemp = I2C_Read(ADDRESS_LIM_VEL);

	CAN_MSG_MERGE(motornum,COMAND_LIM_VEL,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500);  



	readtemp = I2C_Read(ADDRESS_KPP);

	CAN_MSG_MERGE(motornum,COMAND_SET_KPP,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500);  


	readtemp = I2C_Read(ADDRESS_KIP);

	CAN_MSG_MERGE(motornum,COMAND_SET_KIP,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500);  

	//20200422
	readtemp = I2C_Read(ADDRESS_ERROR_START);

	CAN_MSG_MERGE(motornum,COMAND_TEST1,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 


	readtemp = I2C_Read(ADDRESS_ERROR_START + 2);

	CAN_MSG_MERGE(motornum,COMAND_TEST2,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 


	readtemp = I2C_Read(ADDRESS_ERROR_START + 4);

	CAN_MSG_MERGE(motornum,COMAND_TEST3,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 


	readtemp = I2C_Read(ADDRESS_ERROR_START + 6);

	CAN_MSG_MERGE(motornum,COMAND_TEST4,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 


	readtemp = I2C_Read(ADDRESS_ERROR_START + 8);

	CAN_MSG_MERGE(motornum,COMAND_TEST5,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 



	readtemp = I2C_Read(ADDRESS_ERROR_START + 0x0a);

	CAN_MSG_MERGE(motornum,COMAND_TEST6,readtemp,0);

	CAN_A_TX(msgh_tx,msgl_tx);

	delay(500); 





}









