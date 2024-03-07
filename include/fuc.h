#ifndef FUC_H
#define FUC_H

extern void InitSysCtrl(void);
extern void DisableDog(void);
extern void InitPll(int val,int divsel);
extern void InitPeripheralClocks(void);
extern void delay(unsigned int t);
extern void short_delay(void);
extern void ADC_cal (void);
extern void Initial_SCIc(void);
extern void SCIc_T(int a);
extern void SCIc_R(void);
extern void InitXintf(void);
extern void Initial_ADC(void);
extern void FPGA_Reset(void);
extern void Power_Up(void);
extern void Power_Down(void);

extern void initial_para_self(void);
extern void driver_parameter(void);
extern void InitFlash(void);
extern void Initial_Gpio(void);
extern void InitPieCtrl(void);
extern void Initial_INT(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period);
extern void EN_INT(void);
extern void DIS_INT(void);
interrupt void current_isr(void); 
extern void InitEPwm1(void);
extern void InitEPwm2(void);
extern void InitEPwm3(void);
extern void InitEPwm4(void);
extern void InitEPwm5(void);
extern void InitEPwm6(void);

extern void Initial_PWM(void);


extern void Initial_CAN(void);
extern void Initial_CANa(void);
extern void Initial_CANb(void);
extern void I2CA_Init(void);
extern void Config_CAN(void);
extern void CAN_A_RX(void);
extern void CAN_A_TX(unsigned long int mailh,unsigned long int maill);
extern void CAN_A_AUTO_UPDATA(unsigned long int mailh,unsigned long int maill);
extern void CAN_MSG_MERGE(unsigned int id,unsigned int code,unsigned int data,unsigned int dataplus);
extern void PARA_SET_UPDATA(void);



extern void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);



extern void I2CA_WriteData(unsigned char addr,unsigned char dat);
extern unsigned char I2CA_ReadData(unsigned char addr);
extern void I2C_Write(unsigned char addr , unsigned int dat);
extern unsigned int I2C_Read(unsigned char addr);
extern void E2PROM(void);
extern void AD_OFFSET_CAL(void);


interrupt void ad_sample_isr(void);




extern void OPEN_EN(void);
extern void DIS_EN(void);
extern void Err_Handle(void);

extern void FPGA_CHECK(void);

extern void DataProcess(void);

extern void ResolverParaInit(void);

#endif
