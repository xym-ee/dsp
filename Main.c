#include "head.h"

void main(void)

{
	

	InitSysCtrl();
    
    Initial_Gpio();

 
	Initial_INT();
	
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

	InitFlash();
    
	InitXintf();

	Initial_PWM();

	Initial_ADC();

	Initial_CAN();

	Config_CAN();

	I2CA_Init();

   	E2PROM();	

	FPGA_Reset();

	delay(10000);

	FPGA_CHECK();

	ResolverParaInit();

	driver_parameter();

	initial_para_self();

	AD_OFFSET_CAL();

	EN_INT();


	while(1)
	{


        

		CAN_A_RX();



		
      
	}



}
