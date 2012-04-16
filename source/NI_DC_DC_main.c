#include "DSP2802x_Device.h"
#include "DSP2802x_Examples.h"
#include "easy2802x_sci_v7.3.h"
#include "DSP2802x_GlobalPrototypes.h"
#include "Piccolo_PWM.h"
#include "IQmathLib.h"

#define VIN_SCALE	430		//Scaling for Q15 Format
#define VIN_OFFSET	10
#define VBUS_SCALE	667		//Scaling for Q15 Format
#define	I_SCALE		149		//Scaling for Q15 Format
#define I_OFFSET	60


#define GLOBAL_Q	15

#define HV_REFERENCE 100

#define MPPT_UPDATE_PERIOD_MS	500
#define MAX_POWER_SAMPLES		20
#define INITIAL_POWER_SAMPLES	10

#define MPPT_STEP_SIZE	_IQ15(0.4)
#define HICCUP_CURRENT_LIMIT	6
#define MAX_VOLTAGE		50
#define MAX_OPERATING_VOLTAGE	48

#define TSW		1370

#define OUT_MAX 0x599A
#define DELAY_MAX 45876 //(+1.4)
#define DELAY_MIN -45876 //(-1.4)

#pragma CODE_SECTION(pwm_int, "ramfuncs");
#pragma CODE_SECTION(mppt_int, "ramfuncs");
#pragma CODE_SECTION(ms_delay, "ramfuncs");
#pragma CODE_SECTION(__IQmpy, "ramfuncs");

//ADC Scaling/Offset Variables
int input_voltage_prescale;

//MPPT Variables
unsigned int step_direction;
unsigned int startup_flag;
long int Min_Startup_Voltage_Q15;
long int Max_Voltage_Q15;
long int Min_Operating_Voltage_Q15;
long int Max_Operating_Voltage_Q15;
long int Previous_Power_Q15;
long int MPPT_Step_Size_Q15;
long int Input_Power_Q15;
long int Hiccup_Current_Limit_Q15;
long int Current_Limit_Q15;
int step_dir;
int input_current_prescale;

int b_coeff;

//System Status Variables
unsigned int Power_Good;
unsigned int Output_Over_Voltage;

interrupt void pwm_int(void);
interrupt void mppt_int(void);

void pwm_setup(void);

unsigned int x;
volatile unsigned int duty;
unsigned int period;
unsigned int overlap;
unsigned int rising_edge_delay;
unsigned int falling_edge_delay;
volatile unsigned int first_run;
int y;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

volatile long int duty_output;
volatile long int err_delay1;
volatile long int err_delay2;
volatile long int out_delay1;
volatile long int Vin_reference_Q15;
volatile long long int v_comp_out;
volatile long int Vin_err_Q15;

volatile long long int v_temporary;
long int High_Voltage_Reference_Q15;
volatile long int duty_comp;

unsigned int deadtime_after_Q1_off;
unsigned int deadtime_after_Q2_off;
unsigned int deadtime_after_Q1_off_hi_res;
unsigned int deadtime_after_Q2_off_hi_res;
unsigned int early_turn_on_Q2;

long int Power_Samples_Q15[MAX_POWER_SAMPLES];

unsigned int Num_Power_Samples;
unsigned int Power_Sample_Counter;
long int Power_Sample_Sum_Q15;
long int Num_Power_Samples_Q15;
unsigned int delay_flag;
unsigned int duty_knee;

int i_sense_v_gain;
int i_sense_v_shift;

int32 Temp_Variable;

unsigned int i;
unsigned int Sample_Advance;

int32 Input_Voltage_Q15;
int32 Bus_Voltage_Q15;
int32 Input_Current_Q15;
extern void DSP28x_usDelay(Uint32);
void ms_delay(unsigned int);
void SetupAdc(void);
void initVariables(void);
void initialize_mppt_timer(void);


void main()
{
	initVariables();
	
	DINT;
	ms_delay(1);
	InitAdc();
	SetupAdc();
	InitSysCtrl();
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitPieVectTable();
	easyDSP_SCI_Init();

	InitEPwm4();
	pwm_setup();
	initialize_mppt_timer();
	EALLOW;
	PieVectTable.TINT2 = &mppt_int;
	PieVectTable.EPWM4_INT = &pwm_int;
	PieCtrlRegs.PIEIER3.bit.INTx4 = 0x1;
	EDIS;
	IER |= M_INT3;
	IER |= M_INT14;
	EINT;
	ms_delay(1);

	InitEPwm4Gpio();
	ERTM;
	y = 5;
	for(;;)
	{
		if(delay_flag)
		{
			ms_delay(10000);
			delay_flag = 0;
		}
	}
}

interrupt void mppt_int()
{
	GpioDataRegs.GPASET.bit.GPIO3 = 1;
	Power_Sample_Sum_Q15 = 0;
//	for (i = 0; i < Num_Power_Samples; i++)
//	{
//		Power_Sample_Sum_Q15 += Power_Samples_Q15[i];
//	}
//	Num_Power_Samples_Q15 = _IQ15(Num_Power_Samples);
//	Input_Power_Q15 = _IQ15div(Power_Sample_Sum_Q15, Num_Power_Samples_Q15);
//	if (delay_flag)
//	{
//		Vin_reference_Q15 = High_Voltage_Reference_Q15; //100V
//		startup_flag = 0;
//	}
//	else if (!startup_flag && Input_Voltage_Q15 > Min_Startup_Voltage_Q15)
//	{
//		Vin_reference_Q15 = _IQmpy(Input_Voltage_Q15, _IQ15(0.9));
//		step_direction = 0;
//		startup_flag = 1;
//	}
//	else if (startup_flag && (Input_Voltage_Q15 <= Min_Operating_Voltage_Q15))
//	{
//		Vin_reference_Q15 = Min_Operating_Voltage_Q15 + MPPT_Step_Size_Q15;
//		step_direction = 1;
//	}
//	else if (startup_flag && (Input_Voltage_Q15 >= Max_Operating_Voltage_Q15))
//	{
//		Vin_reference_Q15 = Max_Operating_Voltage_Q15 - MPPT_Step_Size_Q15;
//		step_direction = 0;
//	}
//	else
//	{
//		if (startup_flag)
//		{
//			if (Input_Power_Q15 > Previous_Power_Q15)
//			{
//				if (step_direction)
//				{
//					Vin_reference_Q15 += MPPT_Step_Size_Q15;
//				}
//				else
//				{
//					Vin_reference_Q15 -= MPPT_Step_Size_Q15;
//				}
//			}
//			else
//			{
//				if (step_direction)
//				{
//					step_direction = !step_direction;
//					Vin_reference_Q15 -= MPPT_Step_Size_Q15;
//				}
//				else
//				{
//					step_direction = !step_direction;
//					Vin_reference_Q15 += MPPT_Step_Size_Q15;
//				}
//			}
//		}
//		else
//		{
//			Vin_reference_Q15 = 3276800; //100V
//		}
//	}
//	Previous_Power_Q15 = Input_Power_Q15;
//	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
//	return;
}

interrupt void pwm_int()
{
	DINT;
	input_voltage_prescale = (((int) AdcResult.ADCRESULT0 - VIN_OFFSET) >> 1) << 1;
	Input_Voltage_Q15 = ((long int) input_voltage_prescale*VIN_SCALE);
	input_current_prescale = ((int) AdcResult.ADCRESULT1 - I_OFFSET) - ((input_voltage_prescale*i_sense_v_gain) >> i_sense_v_shift);
	Input_Current_Q15 = ((long int) (input_current_prescale )*I_SCALE);

	EPwm4Regs.TBPRD = period;

	EPwm4Regs.CMPA.half.CMPA = duty;

	EINT;
	EPwm4Regs.ETCLR.bit.INT = 0x1;  			//Clear the Interrupt Flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  	//Acknowledge the interrupt
	return;
}

void pwm_setup()
{

	EPwm4Regs.ETSEL.bit.INTEN = 0x1;
	EPwm4Regs.ETSEL.bit.INTSEL = 0x1;
	EPwm4Regs.ETPS.bit.INTPRD = 0x1;
	EPwm4Regs.ETSEL.bit.SOCAEN = 1;
	EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTRD_CMPA;
	EPwm4Regs.ETPS.bit.SOCAPRD = 0x1;
}

void ms_delay(unsigned int wait_time)
{
	volatile unsigned int i;
	CpuTimer1Regs.PRD.all = 0x0000EA60;
	for (i=0; i < wait_time; i++)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;
		CpuTimer1Regs.TCR.bit.TRB = 1;
		CpuTimer1Regs.TCR.bit.TSS = 0;
		while (CpuTimer1Regs.TCR.bit.TIF == 0)
		{	
		}
	}
}

void InitAdc(void)
{
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
        (*Device_cal)();
        EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2802x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
    EDIS;

    ms_delay(10);         // Delay before converting ADC channels
}

void SetupAdc(void)
{
	EALLOW;
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 0;
	//Input Voltage Sampling on SOC0
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x07;
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0xE;
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 0x6;	
	//Output Voltage Sampling on SOC1
//	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x00;
//	AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x0;
//	AdcRegs.ADCSOC1CTL.bit.ACQPS = 0x6;
//	//Input Current Sampling on SOC4
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x07;
	AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x2;
	AdcRegs.ADCSOC1CTL.bit.ACQPS = 0x6;
	EDIS;
}

void initialize_mppt_timer(void)
{
	//Load the pre-scaler to 60000 clock cycles (1ms)
	CpuTimer2Regs.TPRH.all = 0x00EA;
	CpuTimer2Regs.TPR.all = 0x0060;
	CpuTimer2Regs.PRD.all = MPPT_UPDATE_PERIOD_MS;
	CpuTimer2Regs.TCR.bit.TRB = 1;
	CpuTimer2Regs.TCR.all = 0x4000; //Initialize to Interrupt Enabled and Free-Running
}

void initVariables (void)
{
	period = TSW;
	duty = 0;
}
