#include "head.h"

#pragma CODE_SECTION(OPEN_EN, "ramfuncs");
void OPEN_EN(void)
{
	if(Soft_EN == 0)
    {
              
       //      FPGA_FAN = 1;

	        
             
			 delay(2000);  
    	 
           	 Soft_EN = 1;   // Soft_EN ==1  open   

			 EN_PORT = 0;
		  
			 EALLOW;

			 /*

			 EPwm1Regs.TZCTL.bit.TZA = 2;
			 EPwm1Regs.TZCTL.bit.TZB = 1;

			 EPwm2Regs.TZCTL.bit.TZA = 2;
			 EPwm2Regs.TZCTL.bit.TZB = 1;

			 EPwm3Regs.TZCTL.bit.TZA = 2;
			 EPwm3Regs.TZCTL.bit.TZB = 1;

			 EPwm1Regs.TZFRC.bit.OST = 1;
			 EPwm2Regs.TZFRC.bit.OST = 1;
			 EPwm3Regs.TZFRC.bit.OST = 1;

			 delay(20000);                      //  about 50ms

			 EPwm1Regs.TZCTL.bit.TZA = 2;
			 EPwm1Regs.TZCTL.bit.TZB = 2;

			 EPwm2Regs.TZCTL.bit.TZA = 2;
			 EPwm2Regs.TZCTL.bit.TZB = 2;

			 EPwm3Regs.TZCTL.bit.TZA = 2;
			 EPwm3Regs.TZCTL.bit.TZB = 2;

			 EPwm1Regs.TZFRC.bit.OST = 1;
			 EPwm2Regs.TZFRC.bit.OST = 1;
			 EPwm3Regs.TZFRC.bit.OST = 1;
			 

			 delay(10);

			 */

			 EPwm1Regs.TZCLR.bit.OST = 1;
			 EPwm2Regs.TZCLR.bit.OST = 1;
			 EPwm3Regs.TZCLR.bit.OST = 1;


			 EPwm1Regs.TBCTR = 0x0000;
			 EPwm2Regs.TBCTR = 0x0000;  
			 EPwm3Regs.TBCTR = 0x0000;


			 EDIS;

			 EPwm1Regs.CMPA.half.CMPA = 3750;
			 EPwm2Regs.CMPA.half.CMPA = 3750;
			 EPwm3Regs.CMPA.half.CMPA = 3750;
					
					
				
					
      
    }
}

#pragma CODE_SECTION(DIS_EN, "ramfuncs");
void DIS_EN(void)
{
	unsigned char addr_temp_start,addr_temp_read,addr_temp_write,dat_tepm; //20200421
	
	if(Soft_EN == 1)
    {
        
    	 
			 EN_PORT = 1;

			 Soft_EN = 0;   // Soft_EN ==0  closed

			 EALLOW;

			 EPwm1Regs.TZCTL.bit.TZA = 2;
			 EPwm1Regs.TZCTL.bit.TZB = 2;

			 EPwm2Regs.TZCTL.bit.TZA = 2;
			 EPwm2Regs.TZCTL.bit.TZB = 2;

			 EPwm3Regs.TZCTL.bit.TZA = 2;
			 EPwm3Regs.TZCTL.bit.TZB = 2;

			 EPwm1Regs.TZFRC.bit.OST = 1;
			 EPwm2Regs.TZFRC.bit.OST = 1;
			 EPwm3Regs.TZFRC.bit.OST = 1;
			 

			 EDIS;

			 EPwm1Regs.CMPA.half.CMPA = 3750;
			 EPwm2Regs.CMPA.half.CMPA = 3750;
			 EPwm3Regs.CMPA.half.CMPA = 3750;  


			 //20200421

			 Error_Rec = (Error_state_send & 0xfd3f);
	         if(Error_Rec  != 0)
			 {
				addr_temp_start = ADDRESS_ERROR_START;
				addr_temp_read = ADDRESS_ERROR_END;
				addr_temp_write = ADDRESS_ERROR_END;

				while( addr_temp_write > addr_temp_start)
				{
					addr_temp_read -= 2;
					dat_tepm = I2C_Read(addr_temp_read);

					short_delay();

					I2C_Write(addr_temp_write , dat_tepm);
					addr_temp_write -= 2;

				}

				I2C_Write(addr_temp_start , Error_Rec);
			 }


		 //20200421

			 initial_para_self();

		//	 FPGA_FAN = 0;
		
		    Error_state_high   &=  0xdfff;	// clear over load


		    Error_state_send   &=  0xdfff;    // clear over load

		   
      
    }
}

void Power_Up(void)
{
	
	if( Voltage_bus_fil > RELAY_VOL_ON )
	{
		PowerUpCNT++;

		if(PowerUpCNT > 1000)
		{
			FPGA_RELAY = 1;

			PowerState=1;

			PowerUpCNT = 0;

			delay(15000);

			FPGA_Reset();
		}

		
	}
	else
	{
		PowerUpCNT = 0;
	}

	

}

void Power_Down(void)
{
	if(Voltage_bus_fil < RELAY_VOL_OFF)
	{
		PowerDownCNT++;

		if(PowerDownCNT > 1000)
		{
			FPGA_RELAY = 0;

			PowerState=0;

			PowerDownCNT = 0;
		}
	}
	else
	{
		PowerDownCNT = 0;
	}
}



void FPGA_Reset(void)
{
	 
	 FPGA_RST_PORT = 1;
	 delay(100);
	 FPGA_RST_PORT = 0;
	 delay(100);
	 FPGA_RST_PORT = 1;


}



void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    { 
       *DestAddr++ = *SourceAddr++;
    }
    return;
}




void delay(unsigned int t)
{
	int i,j;
	for(i=0;i<t;i++)
	  {
	  	 for (j=0;j<10;j++)
		 	{   asm("   NOP") ;    }
	  }
}


void short_delay(void)             //   delay 300ns
{
		asm(" NOP");
		asm(" NOP");
		asm(" NOP");
		asm(" NOP");
		asm(" NOP");
	 	asm(" NOP");	
		asm(" NOP");
		asm(" NOP");
		asm(" NOP");
	 	asm(" NOP");


}


void SCIc_T(int a)
{
    //while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScicRegs.SCITXBUF=a;

}



void SCIc_R(void)
{
	Uint16 ReceivedChar;
	while(ScicRegs.SCIFFRX.bit.RXFFST != 0)
    {
    	ReceivedChar = ScicRegs.SCIRXBUF.all;
		SCIc_T(ReceivedChar); 
    }


}




void AD_OFFSET_CAL(void)
{
	unsigned int sample_times;

    /*
	rslvrIn.offsetS   = 1.5218;
	rslvrIn.offsetC   = 1.5224;
	u_c_offset        = 32298.8;
	v_c_offset        = 32253.8;
	bus_v_offset      = 1304.8;

    */

	rslvrIn.offsetS   = 0;
	rslvrIn.offsetC   = 0;
	u_c_offset        = 0;
	v_c_offset        = 0;
	bus_v_offset      = 1304.8;


	offsetWfT =  offsetFc * TWO_PI * rslvrIn.SAMPLING_TIME;

	EALLOW;

	for(sample_times=0;sample_times<32768;sample_times++)
	{
		AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;     // force an SOC
		while(!AdcRegs.ADCST.bit.INT_SEQ1)
		{;}
		

		rslvrIn.offsetS += ((float)AdcRegs.ADCRESULT0 - rslvrIn.offsetS) * offsetWfT;
		rslvrIn.offsetC += ((float)AdcRegs.ADCRESULT1 - rslvrIn.offsetC) * offsetWfT;
	
		u_c_offset      += ((float)AdcRegs.ADCRESULT2 - u_c_offset) * offsetWfT;
		v_c_offset      += ((float)AdcRegs.ADCRESULT3 - v_c_offset) * offsetWfT;
	//	bus_v_offset    += ((float)AdcRegs.ADCRESULT4 - bus_v_offset) * offsetWfT;	
		
	
		AdcRegs.ADCTRL2.bit.RST_SEQ1   = 1;           // Reset SEQ1
		AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
		
		

	}

	rslvrIn.offsetS = rslvrIn.offsetS * 3.0/65536.0;       // ADC result is left aligned
	rslvrIn.offsetC = rslvrIn.offsetC * 3.0/65536.0; 


	EDIS;



	OffsetS_CHK =  fabs(rslvrIn.offsetS - 1.5)/1.5;
    OffsetC_CHK =  fabs(rslvrIn.offsetC - 1.5)/1.5;
    OffsetU_CHK =  fabs(u_c_offset - 32767)/32768.0;
    OffsetV_CHK =  fabs(v_c_offset - 32767)/32768.0;

	if(OffsetS_CHK > 0.2)
	{
		Error_state_high  |= ERR_CODE_ADOFFSET;
	}
	if(OffsetC_CHK > 0.2)
	{
		Error_state_high  |= ERR_CODE_ADOFFSET;
	}
	if(OffsetU_CHK > 0.2)
	{
		Error_state_high  |= ERR_CODE_ADOFFSET;
	}
	if(OffsetV_CHK > 0.2)
	{
		Error_state_high  |= ERR_CODE_ADOFFSET;
	}


	
	
}



void FPGA_CHECK(void)
{
	FPGA_ID = FPGA_TEST_CODE;

	if(FPGA_ID != 0x5a5a)
	{
		while(1)
		{
			LED1_PORT = 1;

			delay(35000);

			LED1_PORT = 0;

			delay(35000);
		}
	}
}

