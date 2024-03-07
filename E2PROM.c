#include "head.h"

void E2PROM(void)
{
	unsigned int E2PROM_START,E2PROM_LAST;
	int temp_value;
	float slopetime;

	LED1_PORT = 1;

	E2PROM_START = I2C_Read(ADDRESS_INTIAL);
    E2PROM_LAST   = I2C_Read(ADDRESS_END);

	if ( (E2PROM_START != E2PROM_INITIAL) || ( E2PROM_LAST != E2PROM_END ) )
    {
    
		
			 delay(500);

			 I2C_Write(ADDRESS_INTIAL,E2PROM_INITIAL);

			 delay(500);
			 
			 motornum = 1; 

			 I2C_Write(ADDRESS_MOTORNUM,0x0001);

			 delay(500);
			 
			 RUN_MODE  = 2; 

			 I2C_Write(ADDRESS_MODE,0x0002);    //  velocity loop

			 delay(500); 

			 Motor_pair_poles = 2; 

			 I2C_Write(ADDRESS_POLEPAIRS,0x0002); 

			 delay(500); 

             Cycle_DIR = 1;  

			 I2C_Write(ADDRESS_DIR,0x0000);  

			 delay(500); 

			 Currentd_Kp = 3.5;
             Currentq_Kp = 3.5;

			 I2C_Write(ADDRESS_KPC,1792);  

			 delay(500); 

		     Currentd_Ki = 0.02344;
             Currentq_Ki = 0.02344;

			 I2C_Write(ADDRESS_KIC,24);  

			 delay(500);  
 
             cur_zero_drift = 0.0; 

			 I2C_Write(ADDRESS_DRIFT,0);  

			 delay(500); 
			 
			 Speed_ctrl_lim = 30.0 *  MAX_CURRENT * 2  * CURRENT_RATIO / 100.0; ;
			 
			 I2C_Write(ADDRESS_LIM_CUR, 30 );  

			 delay(500);
			 
			 Speed_Kp_Set = 6.0; 

			 I2C_Write(ADDRESS_KPV,3072);  

			 delay(500);

			 Speed_Ki_Set = 0.070312;

			 I2C_Write(ADDRESS_KIV,72);  

			 delay(500); 

			 SPEED_STEP = RATED_SPEED * 0.0002;     //  5 s  

			 I2C_Write(ADDRESS_SRV,2560);  

			 delay(500);
			 
			 SPEED_LIMIT = RATED_SPEED; 
			 
			 I2C_Write(ADDRESS_LIM_VEL,100);  

			 delay(500); 

			 Position_Kp_Set = 0.03;

			 I2C_Write(ADDRESS_KPP,16);  

			 delay(500); 

			 Position_Ki_Set = 0.0;

			 I2C_Write(ADDRESS_KIP,0);  

			 delay(500); 

			 can_ctrl= 1;

			 I2C_Write(ADDRESS_CAN_AD,1);  

			 delay(500); 



			 I2C_Write(ADDRESS_END,E2PROM_END);



		
    }
	else
	{
		  motornum          =        I2C_Read(ADDRESS_MOTORNUM);

		  short_delay();

		  RUN_MODE          =        I2C_Read(ADDRESS_MODE);

		  short_delay();

		  Motor_pair_poles  =        I2C_Read(ADDRESS_POLEPAIRS);

		  short_delay();
		  
		  temp_value        =        I2C_Read(ADDRESS_DIR);
		  if(temp_value == 0)
          {
          	Cycle_DIR = 1;
          }
		  else
		  {
		  	Cycle_DIR = -1;
		  }


		  temp_value        =        I2C_Read(ADDRESS_KPC);
          Currentd_Kp       =        ((float)temp_value)/512.0;
		  Currentq_Kp       =         Currentd_Kp;

		  temp_value        =        I2C_Read(ADDRESS_KIC);
          Currentd_Ki       =        ((float)temp_value)/1024.0;
		  Currentq_Ki       =        Currentd_Ki;

//		  temp_value        =        I2C_Read(ADDRESS_DRIFT);
//        cur_zero_drift    =        ((float)temp_value) * 2.0 *  CURRENT_RATIO/1024.0 ;

		  temp_value        =        I2C_Read(ADDRESS_DRIFT);
		  SwitchCNT         =   	(((float)temp_value)/1024.0);


		  temp_value        =        I2C_Read(ADDRESS_LIM_CUR);
		  Speed_ctrl_lim    =        temp_value *  MAX_CURRENT * 2 * CURRENT_RATIO / 100.0 ;

		  if (Speed_ctrl_lim > (MAX_CURRENT * 2 * CURRENT_RATIO) )
	      {
		      Speed_ctrl_lim = MAX_CURRENT * 2 * CURRENT_RATIO;
	      }
		  else if (Speed_ctrl_lim < CURRENT_RATIO )
		  {
		  	  Speed_ctrl_lim = CURRENT_RATIO;
		  }
          

		  temp_value        =        I2C_Read(ADDRESS_KPV);
          Speed_Kp_Set      =       ((float)temp_value)/512.0;

		  temp_value        =        I2C_Read(ADDRESS_KIV);
          Speed_Ki_Set      =        ((float)temp_value)/1024.0;


		  temp_value        =        I2C_Read(ADDRESS_LIM_VEL);
		  SPEED_LIMIT       =        temp_value *  RATED_SPEED / 100.0; 
		  if(SPEED_LIMIT > RATED_SPEED)
		  {
		  	 SPEED_LIMIT = RATED_SPEED;
		  }
		  else if(SPEED_LIMIT < 0.01 * RATED_SPEED)
		  {
		  	SPEED_LIMIT = 0.01 * RATED_SPEED;
		  }

		  short_delay();
           

		  temp_value        =        I2C_Read(ADDRESS_SRV);
          slopetime         =        temp_value / 512.0;

          if(slopetime >= 0)
		  {
		  	 
			  	if(slopetime < 0.001)
				{
					slopetime = 0.001;
				}
				else if (slopetime > 30)
				{
					slopetime = 30.0;
				}
				
				SPEED_STEP = RATED_SPEED * 0.001 / slopetime;

		  }



		  temp_value        =        I2C_Read(ADDRESS_KPP);
          Position_Kp_Set   =        temp_value / 512.0;


		  temp_value        =        I2C_Read(ADDRESS_KIP);
          Position_Ki_Set   =         temp_value / 1024.0;

		  temp_value        =        I2C_Read(ADDRESS_CAN_AD);
		  can_ctrl          =        temp_value;

	}


	LED1_PORT = 0;

}

#pragma CODE_SECTION(I2CA_WriteData,"ramfuncs");
void I2CA_WriteData(unsigned char addr,unsigned char dat)
{
  
   
   // Wait until the STP bit is cleared from any previous master communication.
   // Clearing of this bit by the module is delayed until after the SCD bit is
   // set. If this bit is not checked prior to initiating a new message, the
   // I2C could get confused.
/*   if (I2caRegs.I2CMDR.bit.STP == 1)
   {
      return 0;
   }
*/
   // Setup slave address
   I2caRegs.I2CSAR = 0x50;

   // Check if bus busy
/*   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
      return 0;
   }
*/
   // Setup number of bytes to send
   // MsgBuffer + Address
   I2caRegs.I2CCNT =0x02;
//   I2caRegs.I2CFFTX.all = 0x6300;	
   // Setup data to send
   while(I2caRegs.I2CSTR.bit.BB)
   {;}
   I2caRegs.I2CMDR.all = 0x6e20;
   while(!(I2caRegs.I2CSTR.all&0x0010))
   {;}
   I2caRegs.I2CDXR = addr;
   while(!(I2caRegs.I2CSTR.all&0x0010))
   {;}
   I2caRegs.I2CDXR = dat;
   delay(5000);
   while(!(I2caRegs.I2CSTR.all&0x0010))
   {;}
 
 
 



//  I2caRegs.I2CDXR = 0x0000;
// for (i=0; i<msg->NumOfBytes-2; i++)
   
  // I2caRegs.I2CDXR = i;
   

   // Send start as master transmitter
//   I2caRegs.I2CMDR.all = 0x6E20;
  
  
  




   while(I2caRegs.I2CMDR.bit.STP)
   		{;}


   
}

unsigned char I2CA_ReadData(unsigned char addr)
{  unsigned char i;
   
   I2caRegs.I2CSAR = 0x50;

   I2caRegs.I2CCNT =0x01;
//   I2caRegs.I2CFFTX.all = 0x6300;	
   // Setup data to send
   while(I2caRegs.I2CSTR.bit.BB)
   {;}
   I2caRegs.I2CMDR.all = 0x6620;
   while(!(I2caRegs.I2CSTR.all&0x0010))
   {;}
   I2caRegs.I2CDXR = addr;
   
   while(!(I2caRegs.I2CSTR.all&0x0010))
   {;}
	
//      I2caRegs.I2CSAR = 0x50;
	   I2caRegs.I2CCNT = 0x01;
	   I2caRegs.I2CMDR.all=0x6c20;

	   while(!(I2caRegs.I2CSTR.all&0x0008))
	   {;}
       i=I2caRegs.I2CDRR;
   	   while(I2caRegs.I2CMDR.bit.STP)
   	   {;}

   return i;
}

#pragma CODE_SECTION(I2C_Write,"ramfuncs");
void I2C_Write(unsigned char addr , unsigned int dat)
{
	unsigned char temp;
	temp = ( dat & 0x00ff);
	I2CA_WriteData(addr ,temp);
	dat = (dat >> 8);
	temp = ( dat & 0x00ff);
	addr +=1;
    I2CA_WriteData(addr ,temp);

}
unsigned int I2C_Read(unsigned char addr)
{
	unsigned int temp,temp1;
	temp = I2CA_ReadData((addr+1));
	temp = (temp << 8);
    temp1= I2CA_ReadData(addr);
	temp1 &= 0x00ff;
	temp |= temp1;
	return temp;
}

