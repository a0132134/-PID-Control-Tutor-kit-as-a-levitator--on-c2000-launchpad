//#############################################################################
//
//  File:   PID_Tutor.c
//
//  Title:  PID_Tutor
//
//  Target Device:  TMS320F2802x
//
//!   This code demonstrates the software controlled levitation experiment in
//!   Picollo device, which is setup with a linear hall-effect sensor feeding
//!   analog input, and LMD18201 driving the electromagnetic coil to hold the
//!   magnetic object in mid-air.
//!
//!   It configures the ADCINA4 to take sensor inputs and EPWM1A to drive the
//!   magnitude signal (pin 5) for LMD18201.
//!
//!   ADC, PWM, GPIO, CLOCK and TIMER are initialized at the start of the code.
//!   The system clock runs at 50 MHz. TIMER is configured to generate interrupts
//!   at every 250us. GPIO1 is toggled at every interrupt and brought out at J6 pin02
//!   to tune the frequency of operation for convenience. ADC Samples and PWM changes
//!   are done in the ISR.
//
//
//  (C) Copyright 2012, Texas Instruments, Inc.
//#############################################################################
// $TI Release: f2802x Support Library v200 $
// $Release Date: Thu Mar 21 10:01:39 CDT 2013 $
//#############################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/pwm.h"
#include "f2802x_common/include/timer.h"



// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
void delay_loop(void);
void Gpio_select(void);
void Adc_Config(void);
void InitEPwmTimer(void);

// Configure the period for the timer
#define PWM1_TIMER_TBPRD   0x7A00


// Global variables used in this example:

unsigned long timer0IntCount;
uint16_t LoopCount;
int  setpoint = 533;
int  error = 0;
int net_error = 0;
int old_error =0;
int Kc = 3;
int ki = 7;
int kp = 4;
int kd = 2;
int kfactor = 13;
int Pterm,Iterm,Dterm;
int P_read,I_read,D_read;
int correction;
int result;
int factor;
int factor_prev = 50;
int MAX = 72;
int cond1 = 64;
int MIN = 0;
int count1 =0;

ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
PWM_Handle myPwm;
TIMER_Handle myTimer0;


void main(void)
{
    CPU_Handle myCpu;
    PLL_Handle myPll;
    
    // Initialize all the handles needed for this application  
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myPwm = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    myTimer0 = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
    timer0IntCount = 0;



    // Perform basic system initialization    
    CLK_enableAdcClock(myClk);
    (*Device_cal)();
    
    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);
    
    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 6 / 4
    PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);
    
    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

    // If running from flash copy RAM only functions to RAM
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    // For this example use the following configuration:
    Gpio_select();

    // Initalize GPIO
    GPIO_setPullUp(myGpio, GPIO_Number_0, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_EPWM1A);

    // Enable XCLOCKOUT to allow monitoring of oscillator 1
     GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);
     CLK_setClkOutPreScaler(myClk, CLK_ClkOutPreScaler_SysClkOut_by_1);

    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);

    // Register interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_7, (intVec_t)&cpu_timer0_isr);
    
    //  Initialize the Device Peripheral. This function can be found in F2802x_CpuTimers.c
    TIMER_stop(myTimer0);
    TIMER_setPeriod(myTimer0, 50*250); // // Configure CPU-Timer 0 to interrupt every second: 50MHz CPU Freq, 1 second Period (in uSeconds)
    TIMER_setPreScaler(myTimer0, 0);
    TIMER_reload(myTimer0);
    TIMER_setEmulationMode(myTimer0, TIMER_EmulationMode_StopAfterNextDecrement);
    TIMER_enableInt(myTimer0);
    
    // Initialize the ADC
    ADC_enableBandGap(myAdc);
    ADC_enableRefBuffers(myAdc);
    ADC_powerUp(myAdc);
    ADC_enable(myAdc);
    ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

    // Configure ADC
    //Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A4);    //set SOC0 channel select to ADCINA4
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A4);    //set SOC0 channel select to ADCINA4
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //Set SOC0 acquisition period to 7 ADCCLK
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC1);                 //Connect ADCINT1 to EOC1
    ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enable ADCINT1

    InitEPwmTimer();

    LoopCount = 0;

    // Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_1);
    
    // To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
    // of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in F2802x_CpuTimers.h), the
    // below settings must also be updated.

    //   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    TIMER_start(myTimer0);
 
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    CPU_enableInt(myCpu, CPU_IntNumber_1);

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PIE_enableTimer0Int(myPie);

    // Enable global Interrupts and higher priority real-time debug events
    CPU_enableGlobalInts(myCpu);
    CPU_enableDebugInt(myCpu);
    
    // Set the flash OTP wait-states to minimum.
    FLASH_setup(myFlash);

    // Toggle I/Os using SET/CLEAR registers.
    // Set pins to a known state

    ((GPIO_Obj *)myGpio)->GPASET = 0xAAAAAAAA;
    ((GPIO_Obj *)myGpio)->GPACLEAR = 0x55555555;

    // IDLE loop. Just sit and loop forever (optional):
    //Main program loop
    for(;;)
    {
      LoopCount++;
    }

}


interrupt void cpu_timer0_isr(void)
{
    timer0IntCount++;

    ((GPIO_Obj *)myGpio)->GPATOGGLE = 0xFFFFFFFF;
    delay_loop();

    //Force start of conversion on SOC0 and SOC1
    ADC_forceConversion(myAdc, ADC_SocNumber_0);
    ADC_forceConversion(myAdc, ADC_SocNumber_1);

    //Wait for end of conversion.
    while(ADC_getIntStatus(myAdc, ADC_IntNumber_1) == 0) {}

    // Clear ADCINT1
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);

    //discard ADCRESULT0 as part of the workaround to the 1st sample errata for rev0

    result=  ADC_readResult(myAdc, ADC_ResultNumber_1);
    result = result >> 2;//Configuring result as a 10 bit ADC Output

	//PID Control Loop
    error  = setpoint  - result;  //position from top(0) of sensor region to bottom (650)
	net_error += error;
	if(net_error > 1023) net_error = 1023;
	if(net_error < 0) net_error = 0;
	Iterm = net_error/ki;
	Dterm = (error-old_error)*kd;
	Pterm = error/kp;
	correction = Kc*(Pterm + Iterm + Dterm);

	factor = (factor + ((setpoint - correction)/kfactor));
    old_error = error;

    // Making sure that the PWM duty cycle doesn't change drastically
	if (((factor - factor_prev) > 10) || ((factor_prev - factor) > 10) )
	{
	factor = factor_prev;
    }
	factor_prev = factor;

    //Limit the output
	if(factor > cond1)
	{
		factor = MAX;
		if(count1 == 10)
		{
			factor = MAX + 2;
			count1 =0;
		}count1++;
	}
	else if(factor < MIN)
	{
	     factor = MIN;
	}

    // update PWM
	PWM_setCmpA(myPwm, (PWM1_TIMER_TBPRD/100)*(100-factor));

    // Acknowledge this interrupt to receive more interrupts from group 1
	//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}

void delay_loop()
{
    short      i;
    for (i = 0; i < 1000; i++) {}
}

void Gpio_select(void)
{

    EALLOW;
    ((GPIO_Obj *)myGpio)->GPAMUX1 = 0x00000000;
    ((GPIO_Obj *)myGpio)->GPAMUX2 = 0x00000000;
    ((GPIO_Obj *)myGpio)->GPBMUX1 = 0x00000000;
    ((GPIO_Obj *)myGpio)->GPADIR = 0xFFFFFFFF;
    ((GPIO_Obj *)myGpio)->GPBDIR = 0x0000000F;
    EDIS;

}

void InitEPwmTimer()
{

    // Stop all the TB clocks
    CLK_disableTbClockSync(myClk);
    // Enable the PWM
    CLK_enablePwmClock(myClk, PWM_Number_1);

    // Disable Sync
    PWM_setSyncMode(myPwm, PWM_SyncMode_CounterEqualZero);

    // Initally disable Free/Soft Bits
    PWM_setRunMode(myPwm, PWM_RunMode_SoftStopAfterIncr);

    PWM_setPeriod(myPwm, PWM1_TIMER_TBPRD);                        // Set up PWM1 Period
    PWM_setCounterMode(myPwm, PWM_CounterMode_Up);                 // Count up mode
    PWM_setCmpA(myPwm, PWM1_TIMER_TBPRD/2);                        // CompareA event at half of period
    PWM_setActionQual_Period_PwmA(myPwm, PWM_ActionQual_Clear);    //
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm, PWM_ActionQual_Set);  // Action-qualifiers, Set on CMPA, Clear on PRD

    // Start all the timers synced
    CLK_enableTbClockSync(myClk);

}
}


//===========================================================================
// No more.
//===========================================================================
