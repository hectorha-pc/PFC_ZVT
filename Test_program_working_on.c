/*
Ha Phu Cuong (PFC_ZVT)
11_4_2024
email: phucuong5197@gmail.com
 */
//###########################################################################

// Included Files
#include "F28x_Project.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Defines
#define FREQ_SW             (300e3)
#define FREQ_SAMPLING       (20e3)                 // 1/10 FREQ_SW
#define SAMPLING_TIME       (1.0/FREQ_SAMPLING)      //  = 1/FREQ_SAMPLING
#define FREQ_GRID           (50.0)
#define CNT_5SEC            (FREQ_SAMPLING * 5.0)
#define PI                  (3.1415926535897932)
#define TWOPI               (2.0*PI)
#define Ls                  (1.5e-6)
#define Coss_Sm             (140e-12)
#define t_re                (PI/2*sqrt(Ls*Coss_Sm))
#define CPU_SPD              200E6  // error for CPU_SPD undified so need to add (check!)

#define ePWM_1to4_BUFF_ON       (GpioDataRegs.GPCSET.bit.GPIO91 = 1)    // epwm1, epwm2, epwm3, epwm4
#define ePWM_1to4_BUFF_OFF      (GpioDataRegs.GPCCLEAR.bit.GPIO91 = 1)
#define ePWM_5_12_BUFF_ON       (GpioDataRegs.GPCSET.bit.GPIO93 = 1)        // epwm5, epwm12
#define ePWM_5_12_BUFF_OFF       (GpioDataRegs.GPCCLEAR.bit.GPIO93 = 1)        // epwm5, epwm12

Uint16 ADC_interrupt_cnt;
//Uint16 PWM1_interrupt_cnt;

// ADC variables
//Uint16 A3_adc, B0_adc, C4_adc, D3_adc;
Uint16 A3_adc, B0_adc, D2_adc, A4_adc;
float32 offset_IC_I_low = 2.5, offset_Opam_I_low = 0, offset_Comp_I_low = -0.000112000009, R1_I_low = 5.6, R2_I_low = 3.3;
float32 offset_IC_I_high = 2.5, offset_Opam_I_high = 0, offset_Comp_I_high = -0.0106880702, R1_I_high = 2.2, R2_I_high = 2.2;
float32 offset_IC_V_low = 0, offset_Opam_V_low = 1.65, offset_Comp_V_low = 0.148684651, R1_V_low = 5.1, R2_V_low = 3.9, Rsen1_V_low = 510*4, Rsen2_V_low = 1;
float32 offset_IC_V_high = 0, offset_Opam_V_high = 0, offset_Comp_V_high = 0.0040480094, R1_V_high = 3.3, R2_V_high = 5.1, Rsen1_V_high = 510*9, Rsen2_V_high = 1;
float32 ADC_I_low, real_I_low, I_low;
float32 ADC_I_high, real_I_high, I_high;
float32 ADC_V_low, real_V_low, V_low;
float32 ADC_V_high, real_V_high, V_high_preBSF, V_high; // need an extra variable for V_high due to BSFs
float32 Max_V_low = 500, Max_I_low = 40, Max_V_high = 1200, Max_I_high = 100;

// other variables
float32 tSs, tdead;
float32 DS1, DS2, DS3, DS4, DSs, Dead, Duty;
Uint16 RUN = 0;
float32 V_high_ref = 850;   //reference
float32 V_low_mag = 339.4;  //expected magnitude (240*sqrt(2))
float32 V_low_Peak;         //to check magnitude

// variable for AC PWM testing
float32 ramp;
float32 V_low_gen, V_low_gen_pk = 339.41;
float32 I_low_gen, I_low_gen_pk = 64.82;
float32 V_high_gen = 850;
float32 I_high_gen = 12.94;
float32 temp_angle;

struct STATIONARY
{

    float32 alpha;
    float32 beta;

};

struct SYNCHRONOUS
{
    float32 D;
    float32 Q;
};

struct Z_1ST
{
    float32 in;
    float32 out;
};

struct Z_2ND
{
    float32 in;
    float32 out;
    struct Z_1ST Zn1;
};

struct LF_PI
{
    float32 Kp;
    float32 Ki;
};

struct COEF_1ST
{
    float32 a0;
    float32 a1;
    float32 b0;
    float32 b1;
};

struct COEF_2ND
{
    float32 a0;
    float32 a1;
    float32 a2;
    float32 b0;
    float32 b1;
    float32 b2;
};

struct SRF
{
    struct COEF_1ST         coeff_LF;   // Loop filter based on PI controller.
    float32                 v_q[2];     // input Q
    float32                 ylf[2];     // output LF
    float32                 fo;         // output frequency of PLL
    float32                 fn;         //nominal frequency
    float32                 theta[2];   // output of PLL
    float32                 delta_T;    // Sampling time
};

struct PFC_ZVT_CONTROL
{
    float32 errorV;       //V_high_ref - V_high
    struct Z_1ST errorV1;      //[n-1] values of Vloop
    float32 PI_Vloop;     //output of PI_Vloop
    float32 PI_Vloop_lim; //limited output of PI_Vloop
    float32 V_low_rep;    //replicated with theta
    float32 I_L_ref;
    float32 errorI;       //abs(I_L_ref) - abs(I_high)
    struct Z_1ST errorI1;      //[n-1] values of Iloop
    float32 PI_Iloop;     //output of PI_Iloop
    float32 PI_Iloop_lim; //limited output of PI_Iloop
    float32 DFF;          //Duty feed-forward
    float32 Duty_pre_lim; //Duty pre-limited
    float32 Duty;         //final Duty
} CTL;

volatile struct EPWM_REGS *ePWM_Regs[] =
{
    0,      //  0 address dummy!!
    &EPwm1Regs,
    &EPwm2Regs,
    &EPwm3Regs,
    &EPwm4Regs,
    &EPwm5Regs,
    &EPwm6Regs,
    &EPwm7Regs,
    &EPwm8Regs,
    &EPwm9Regs,
    &EPwm10Regs,
    &EPwm11Regs,
    &EPwm12Regs
};

enum TESTCASE
{
    SYSTEM_STANDBY,  //0
    START_UP,        //1
    FAULT,           //2
    RESET,           //3
    TEST_DC_IN_NOZVT_POS, //4
    TEST_DC_IN_NOZVT_NEG, //5
    TEST_DC_IN_POS,
    TEST_DC_IN_NEG,
    TEST_DC_IN,      //
    CLOSEDLOOP,      //
    CALIB_ADC        //
} ACDC_TEST = SYSTEM_STANDBY;

enum DAC_ID
{
    DACA,
    DACB,
    DACC
};

enum Mode
{
    AC,
    DC
};


typedef enum {
    D3,
    D4,
    D5,
    D8,
    D9,
    D10,
    NUM_LEDS  // Special value to represent the number of LEDs
} LED_ID;

typedef struct {
    LED_ID id;
    Uint16 cnt;
} LED_Control;

// Create LED_Control instances for each LED
LED_Control led3 = {D3, 0};
LED_Control led4 = {D4, 0};
LED_Control led5 = {D5, 0};
LED_Control led8 = {D8, 0};
LED_Control led9 = {D9, 0};
LED_Control led10 = {D10, 0};

// Pointers to GPIO registers controlling leds
volatile Uint32 *gpioSet = &GpioDataRegs.GPBSET.all;    // Pointer to the entire GPBSET register
volatile Uint32 *gpioClear = &GpioDataRegs.GPBCLEAR.all; // Pointer to the entire GPBCLEAR register
volatile Uint32 *gpioToggle = &GpioDataRegs.GPBTOGGLE.all; // Pointer to the entire GPBTOGGLE register

// Array of GPIO bit masks for each LED
Uint32 gpioBitMask[NUM_LEDS] = {
    ((Uint32)1 << 21),  // D3 is controlled by GPIO53 represented by bit 21 in the register GPBSET, GPBCLEAR, GPBTOGGLE
    ((Uint32)1 << 20),  // D4 is controlled by GPIO52 represented by bit 20 in the register
    ((Uint32)1 << 17),  // D5 is controlled by GPIO49 represented by bit 17 in the register
    ((Uint32)1 << 19),  // D8 is controlled by GPIO49 represented by bit 17 in the register
    ((Uint32)1 << 18),  // D9 is controlled by GPIO50 represented by bit 18 in the register
    ((Uint32)1 << 9)    // D10 is controlled by GPIO41 represented by bit 9 in the register
};

enum channel_ID
{
    A,
    B
}; // PWMA/B

enum state
{
    ON,
    OFF,
    NONE
}; // PWMA/B


union CHECK_FUALT
{
    Uint16      all;
    struct  CHECK_FUALT_BIT
    {
        Uint16  overVin:1;
        Uint16  overIin:1;
        Uint16  overVout:1;
        Uint16  overIout:1;
        Uint16  rsed:12;
    } bit;
} checkFault;

// PLL and control variables
struct LF_PI PI_PLL = {0.444, 25.3331};
struct LF_PI PI_Iloop = {0.02, 100}; //{0.000833 4.167} {0.008 500)
struct LF_PI PI_Vloop = {0.15, 4.5}; //{0.2 6}          {0.0005 0.175}
struct COEF_1ST coeff_LF_PLL, coeff_LF_Vloop, coeff_LF_Iloop; // loop filter
struct COEF_1ST coef1stLPF; // filter for ADC
struct Z_1ST V_high1, I_high1, V_low1, I_low1; //[n-1] values for LPF
struct COEF_2ND coef2ndBSF; // filter 2nd harmonic for Vhigh feedback
struct Z_2ND V_high2; //[n-2] and [n-1] values for LPF
struct STATIONARY V_low_albe, V_low_albe1;
struct SYNCHRONOUS V_low_DQ;
struct SRF srf_pll;
float32 theta_out;
float32 pre_error_V_low_PLL;
float32 error_V_low_PLL;
float32 refsin;
float32 refcos;
Uint16 POS, NEG;

//calib variables
unsigned long cnt_Calib = 0;

void ConfigureDAC(void)
{
    EALLOW;
    // DACA <=> ADC_A0
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1; // enable DAC_A - clock
    DacaRegs.DACCTL.bit.DACREFSEL = 1; // use ADC VREFHI instead of an external signal
    DacaRegs.DACCTL.bit.LOADMODE = 0; // load on SysClk not PWMSYNC (PWM not yet enabled)
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // enable output
    DacaRegs.DACVALS.all = 0; // output = 0

    //====================================================

    // DACB <=> ADC_A1
    CpuSysRegs.PCLKCR16.bit.DAC_B = 1; // enable DAC_B - clock
    DacbRegs.DACCTL.bit.DACREFSEL = 1; // use ADC VREFHI instead of an external signal
    DacbRegs.DACCTL.bit.LOADMODE = 0; // load on SysClk not PWMSYNC (PWM not yet enabled)
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // enable output
    DacbRegs.DACVALS.all = 0; // output = 0

    DELAY_US(10); // Delay for buffered DAC to power up

    //====================================================

    //DACC <=> ADC_B0 is used already
//    CpuSysRegs.PCLKCR16.bit.DAC_C = 1; // enable DAC_C - clock
//    DaccRegs.DACCTL.bit.DACREFSEL = 1; // use ADC VREFHI instead of an external signal
//    DaccRegs.DACCTL.bit.LOADMODE = 0; // load on SysClk not PWMSYNC (PWM not yet enabled)
//    DaccRegs.DACOUTEN.bit.DACOUTEN = 1; // enable output
//    DaccRegs.DACVALS.all = 0; // output = 0

    DELAY_US(10); // Delay for buffered DAC to power up

    EDIS;
}

void ConfigureADC(void)  // Write ADC configurations and power up the ADC
{
    EALLOW;
    // Write configurations
    // ADC A SETUP/CONFIG
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    // ADC B SETUP/CONFIG
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    // ADC C SETUP/CONFIG
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    // ADC D SETUP/CONFIG
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // ADC A SETUP
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1; // ADC B SETUP
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1; // ADC C SETUP
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1; // ADC D SETUP

    // Power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;// ADC A SETUP
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;// ADC B SETUP
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;// ADC C SETUP
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;// ADC D SETUP

    //Delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    EDIS;
}

void SetupADCEpwm(void) // SetupADCEpwm - Setup ADC EPWM acquisition window and select channels, SOC
{
    Uint16 acqps;
    Uint16 ADC_window;

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 25; // old 14
        ADC_window = acqps;
            //14 for ~75ns
            //25 for ~125ns
            //50 for ~250ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //Select channels to convert and end of conversion flag
    //======ADC_MULTI ADCs CHANNELS=====================================
    EALLOW;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;      //A3 - SOC0 - I_low
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADC_window;     //23 for 120ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    // ePWM1, ADCSOCA

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;      //B0 - SOC0 - I_high
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = ADC_window;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

//    AdccRegs.ADCSOC0CTL.bit.CHSEL = 4;      //C4 - SOC0 - V_low
//    AdccRegs.ADCSOC0CTL.bit.ACQPS = ADC_window;
//    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 2;      //D2 - SOC0 - V_low
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADC_window;
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

//    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 3;      //D3 - SOC0 - V_high
//    AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADC_window;
//    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;      //A4 - SOC1 - V_high
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = ADC_window;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

    //==================================================================

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of C_SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //======ADC_MULTI ADCs CHANNELS=====================================
    EDIS;
}

void InitEPWM(Uint16 ch, float32 freq)  // count up
{
    // Set switching frequency
    ePWM_Regs[ch]->TBPRD = (float32)(CPU_SPD/freq/2.0-1);    // UP Counter

    // Set actions
    if(ch == 2)
    {
        //PWM2A is for Ss, PWM2B is for S2
        ePWM_Regs[ch]->AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on zero
        ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event A
        ePWM_Regs[ch]->AQCTLB.bit.CAU = AQ_SET;            // Set PWM2B on event A
        ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM2B on event B
    }

    else if(ch == 3)
    {
        //PWM3A is for S1, PWM3B is for Sb1
        ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM3A on event A
        ePWM_Regs[ch]->AQCTLA.bit.PRD = AQ_CLEAR;          // Clear PWM3A on TBPRD
        //test
        ePWM_Regs[ch]->AQCTLB.bit.ZRO = AQ_SET;            // Set PWM3A on event A
        ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM3A on TBPRD
        //test

//        ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
    }

    else if(ch == 4)
    {
        //PWM4A is for S4
        ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM4A on event A
        ePWM_Regs[ch]->AQCTLA.bit.CBU = AQ_CLEAR;          // Clear PWM4A on event B
        ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
    }

    else if(ch == 5)
    {
        //PWM5A is for S3, PWM5B is for Sb2
        ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM5A on event A
        ePWM_Regs[ch]->AQCTLA.bit.PRD = AQ_CLEAR;          // Clear PWM5A on TBPRD
        //test
        ePWM_Regs[ch]->AQCTLB.bit.ZRO = AQ_SET;            // Set PWM3A on event A
        ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM3A on TBPRD
        //test

//        ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
    }

    // set phase, channel 1 is MASTER phase
    if(ch == 1)
    {
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_DISABLE;        // Phase Reg. DISABLE
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync Output Select  = 01
                                                            // 00 = EPWMxSYNCI
                                                            // 01 = CTR = 0  --> counter of EPWM1 is selected as the reference phase angle
                                                            // 10 = CTR = CMPB
                                                            // 11 = disable SyncOut
        ePWM_Regs[ch]->TBCTR = 0x0000;                      // Clear counter
    }

    else if(ch != 1)
    {
        ePWM_Regs[ch]->TBCTL.bit.PHSEN = TB_ENABLE;         // TBPHS on EPWMxSYNCI signal
        ePWM_Regs[ch]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;     // Sync Output Select  = 00
                                                            // 00 = EPWMxSYNCI  --> phase of other EPWM channel refer to EPWM1
                                                            // 01 = CTR = 0
                                                            // 10 = CTR = CMPB
                                                            // 11 = disable SyncOut
        ePWM_Regs[ch]->TBCTR = 0x0000;                      // Clear counter
    }

    // Setup TBCLK, counter mode
    ePWM_Regs[ch]->TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    ePWM_Regs[ch]->TBCTL.bit.PRDLD = TB_SHADOW;     // Period Shadow Load:  0 --> Load on CTR, 1 --> Load immediately (check if neccessary)
    ePWM_Regs[ch]->TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock Pre-scale:  000 = /1, 001 = /2, 010 = /4, ...
    ePWM_Regs[ch]->TBCTL.bit.CLKDIV = TB_DIV1;

    // Set Counter Compare Control Register, see page 1776 of spruhm8g.pdf (TMS320F2837xD Dual-Core Delfino)
    ePWM_Regs[ch]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO ????
    ePWM_Regs[ch]->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    ePWM_Regs[ch]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    ePWM_Regs[ch]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
}

void ePWM_SOCSEL(Uint16 ch,  Uint16 SOCASEL, Uint16 Number_envent)
{
    ePWM_Regs[ch]->ETSEL.bit.SOCAEN = 1;                    // Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse
    ePWM_Regs[ch]->ETSEL.bit.SOCASEL = SOCASEL;             // EET_CTR_ZERO = 0x1, enable event time-base counter equal to zero. (TBCTR = 0x00)
    ePWM_Regs[ch]->ETPS.bit.SOCPSSEL = 1;                   // Pre-Scale selection (interrupt once every 0-15 events)
    ePWM_Regs[ch]->ETSOCPS.bit.SOCAPRD2 = Number_envent;    // Generate ADC_INT on nth event
}

void updateDutyA(Uint16 ch, float32 duty)
{
    ePWM_Regs[ch]->CMPA.bit.CMPA = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * duty);
}

void updateDutyB(Uint16 ch, float32 duty)
{
    ePWM_Regs[ch]->CMPB.bit.CMPB = (Uint32)((float32)ePWM_Regs[ch]->TBPRD * duty);
}

void FORCE_PWM(Uint16 ch, enum state st, enum channel_ID channel)
{
    if (st == OFF)
    {
        if (channel == A) ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x1;//0x1 means forces a continuous low on ePWMA
        else if(channel == B) ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x1;//0x1 means forces a continuous low on ePWMB
    }
    else if (st == ON)
    {
        if(channel == A) ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x2;//0x1 means forces a continuous high on ePWMA
        else if(channel == B) ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x2;//0x1 means forces a continuous high on ePWMB
    }
    else if (st == NONE)
    {
        if(channel == A) ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x0;//0x1 Software forcing is disabled and has no effect
        else if(channel == B) ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x0;//Software forcing is disabled and has no effect
    }
}

void ForceOFFPWM(Uint16 ch, Uint16 off)
{
    if (off == 1)
    {
        ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x1;  // 0x1 means forces a continuous low on output X
        ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x1;
    }                                           // 0x2 means forces a continuous high on output X
    else if (off == 0)
    {
        ePWM_Regs[ch]->AQCSFRC.bit.CSFA = 0x0;  // Software forcing is disabled and has no effect
        ePWM_Regs[ch]->AQCSFRC.bit.CSFB = 0x0;
    }
}

float32 scaled_sensor_VOL(Uint16 ADC_Result, float32 offset_IC, float32 offset_ref,
                        float32 offset_comp, float32 R1, float32 R2, float32 Rsen1, float32 Rsen2)
{
    float32 out;
    float32 Vadc = ADC_Result *  3.3/4095.0;
    float32 Out_Sensor = (Vadc - offset_ref - offset_comp) * R1 / R2; // offset 1.65V or 0
    float32 Out_Sensor_zero = Out_Sensor - offset_IC; // internal Sensing IC offset
    float32 In_Sensor_zero = Out_Sensor_zero/8.2; // IC gain
    out = In_Sensor_zero * (Rsen1 + Rsen2)/Rsen2;
   return out;
 }

float32 scaled_sensor_CURR(Uint16 ADC_Result, float32 offset_IC, float32 offset_ref, float32 offset_comp, float32 R1, float32 R2)
{
    float32 out;
    float32 Vadc = ADC_Result * 3.3/4095.0;
    float32 Out_Sensor = (Vadc - offset_ref - offset_comp) * R1 / R2; // offset 1.65V or 0 (0 for used IC since IC offset already 2.5)
    float32 Out_Sensor_zero = Out_Sensor - offset_IC; // internal Sensing IC offset
    out = Out_Sensor_zero * 40; // 25mV/A
    return out;
 }

float32 get_offset(float32 in, float32 signal)
{
    float32 out;
    if(signal > 0)  out = in + 0.000004;
    if(signal < 0)  out = in - 0.000004;
    return out;
}

struct COEF_1ST get1stLPFcoef(float32 cutoff, float32 freqSampling)
{
    float32 w_cut;
    struct COEF_1ST coef;
    w_cut = TWOPI * cutoff;
    coef.a1 = ( w_cut - 2.0f * freqSampling)/( 2.0f * freqSampling + w_cut);
    coef.b0 = w_cut / ( 2.0f * freqSampling + w_cut);
    coef.b1 = coef.b0;
    coef.a0 = 0;
    return coef;
}

//PI tustin form
struct COEF_1ST getcoeff_PI(float32 Kp, float32 Ki, float32 Ts)
{
    struct COEF_1ST out;
    out.b0 = (2.0 * Kp + Ki * Ts) / 2.0;
    out.b1 = -(2.0 * Kp - Ki * Ts) / 2.0;
    out.a1 = -1;
    out.a0 = 1;
    return out;
}

float32 zTF_1st(float32 in, struct Z_1ST *Zn1, struct COEF_1ST coef)
{
    float32 out;
    out = coef.b0 * in + coef.b1 * Zn1->in + (-coef.a1 * Zn1->out);
    Zn1->in = in;
    Zn1->out = out;
    return out;
}

// Band stop coef
struct COEF_2ND get2ndBSFcoef(float32 freqCenter, float32 gain, float32 stopBand, float32 freqSampling)
{
    float32 B, w_0, Ts, denum;
    struct COEF_2ND coef3;
    w_0 = TWOPI * freqCenter;
    B = TWOPI * stopBand;
    Ts = 1 / freqSampling;
    denum = 1 + B * Ts / 2 + w_0 * w_0 * Ts * Ts / 4;
    coef3.b0 = (gain + gain * (w_0 * w_0 * Ts * Ts / 4)) / denum;
    coef3.b1 = (-2 * gain + gain * (w_0 * w_0 * Ts * Ts / 2)) / denum;
    coef3.b2 = (gain + gain * (w_0 * w_0 * Ts * Ts / 4)) / denum;
    coef3.a0 = 1;
    coef3.a1 = (-2 + w_0 * w_0 * Ts * Ts) / denum;
    coef3.a2 = (1 - 2 * B * Ts + (w_0 * w_0 * Ts * Ts / 4)) / denum;
    return coef3;
}

float32 zTF_2nd(float32 in, struct Z_2ND *Zn2, struct COEF_2ND coef)
{
    float32 out;
    out = -coef.a1 * Zn2->Zn1.out
          + -coef.a2 * Zn2->out
          +  coef.b0 * in
          +  coef.b1 * Zn2->Zn1.in
          +  coef.b2 * Zn2->in;
    Zn2->in = Zn2->Zn1.in;
    Zn2->Zn1.in = in;
    Zn2->out = Zn2->Zn1.out;
    Zn2->Zn1.out = out;
    return out;
}

void PLL_INIT(float32 grid_freq, float32 sampling_time, struct COEF_1ST lf_para, struct SRF *PLL)
{
    // initialization
    PLL->fn = grid_freq;
    PLL->fo = 0.0;
    //
    PLL->theta[0] = 0.0;
    PLL->theta[1] = 0.0;
    //
    PLL->v_q[0] = 0.0;
    PLL->v_q[1] = 0.0;
    //
    PLL->ylf[0] = 0.0;
    PLL->ylf[1] = 0.0;
    //
    PLL->delta_T = sampling_time;
    PLL->coeff_LF = lf_para;
}

struct STATIONARY Vgrid1phase_to_albe(float32 pre_error_VgPLL,float32 error_VgPLL,float32 Vgrid1phase, float32 sampl_time, float32 K,struct STATIONARY albe2)
{
    struct STATIONARY out;
    pre_error_VgPLL = Vgrid1phase - albe2.alpha;
    error_VgPLL = pre_error_VgPLL*K - albe2.beta ;
    out.alpha = albe2.alpha + error_VgPLL*sampl_time*50*2*PI;  // 314.159 = 2*PI*fgrid
    out.beta = albe2.beta + out.alpha*sampl_time*50*2*PI;
    return out;
}

struct SYNCHRONOUS albe2dq(struct STATIONARY in, float32 refsin, float32 refcos)
{
    struct SYNCHRONOUS out;
    out.D = refcos * in.alpha + refsin * in.beta;
    out.Q = -refsin * in.alpha + refcos * in.beta;
    return out;
}


void SRF_PLL_FUNC(struct SRF *pll)
{
    pll->ylf[0] = (pll->coeff_LF.b0 * pll->v_q[0]) + \
                  (pll->coeff_LF.b1 * pll->v_q[1])+ pll->ylf[1] ;
    pll->ylf[1] = pll->ylf[0];
    pll->v_q[1] = pll->v_q[0];
    pll->fo = pll->fn + pll->ylf[0];
    pll->theta[0] = pll->theta[1] + ((pll->fo * pll->delta_T) * (float32)(2 * PI));

    if (pll->theta[0] > (float32)(2 * PI))
    {
        pll->theta[0] = pll->theta[0] - (float32)(2 * PI);
//        pll->theta[0] = 0;
    }

    pll->theta[1] = pll->theta[0];
}

// Ramp generation, ramp output -> [0,1]
float32 Ramp_gen(float grid_freq, float interrupt_freq, float *temp_angle)
{
    float angle;
    float stepangle;
    float out;

    stepangle =((float32) 1.0/interrupt_freq);
    angle = (*temp_angle) + (stepangle*grid_freq);
    if (angle >= (1.0))
        angle -= (1.0);
    out = angle;
    (*temp_angle) = angle;

    return out;
}

void SetDAC(enum DAC_ID DAC, enum Mode mode, float32 value, float32 peak)
{
    if (mode == DC)
        if (DAC == DACA) DacaRegs.DACVALS.all = value/peak*4095;  // Set value for DAC A
        else if (DAC == DACB) DacbRegs.DACVALS.all = value/peak*4095;  // Set value for DAC B
        else if (DAC == DACC) DaccRegs.DACVALS.all = value/peak*4095;  // Set value for DAC C
    if (mode == AC)
        if (DAC == DACA) DacaRegs.DACVALS.all = value/peak*2047.5 + 2047.5;  // Set value for DAC A
        else if (DAC == DACB) DacbRegs.DACVALS.all = value/peak*2047.5 + 2047.5;  // Set value for DAC B
        else if (DAC == DACC) DaccRegs.DACVALS.all = value/peak*2047.5 + 2047.5;  // Set value for DAC C
    DELAY_US(2);
}

float32 Limiter(float32 in, float32 upper, float32 lower)
{
    if (in > upper)             return upper;
    else if (in < lower)        return lower;
    else                        return in;
}

void LED(LED_ID led, Uint16 state)  // LED function to set or clear an LED
{
    if (led >= NUM_LEDS)
    {
        // Handle invalid LED ID if needed
        return;
    }

    if (state == 1) {
        *gpioSet |= gpioBitMask[led];  // Set the corresponding GPIO bit
    } else if (state == 0) {
        *gpioClear |= gpioBitMask[led];  // Clear the corresponding GPIO bit
    }
}

// LED blink function to toggle an LED
void LED_BLINK(LED_Control *led_control, float32 ADC_int_frq, float32 blink_time)
{
    if (led_control->id >= NUM_LEDS) {
        // Handle invalid LED ID if needed
        return;
    }

    led_control->cnt++;  // Increment the counter for this specific LED

    float32 current_time = (float32)led_control->cnt / ADC_int_frq;
    if (current_time >= blink_time)
    {
        *gpioToggle |= gpioBitMask[led_control->id];  // Toggle the corresponding GPIO bit
        led_control->cnt = 0;  // Reset the counter after toggling
    }
}

void InitGPIOs(void)
{
    EALLOW;
    //GPIO58 is for ADCinterrupt time check
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0; //configure as GPIO dung tam
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;  //configure as output dung tam

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0; //configure as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;  //configure as output
    //GPIO53 is for LED_D3
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;
    //GPIO53 is for LED_D4
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;
    //GPIO53 is for LED_D5
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;
    //GPIO53 is for LED_D8
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;
    //GPIO53 is for LED_D9
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;
    //GPIO41 is for LED_D10
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;
    //GPIO91 is for ePWM_1to4_BUFF
    GpioCtrlRegs.GPCMUX2.bit.GPIO91 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO91 = 1;
    //GPIO93 is for ePWM_5_12_BUFF
    GpioCtrlRegs.GPCMUX2.bit.GPIO93 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO93 = 1;

    EDIS;
}


__interrupt void adca1_isr(void)  // #MAIN $ADC interruptcos
{
//    GpioDataRegs.GPBSET.bit.GPIO58 = 1; //check ADC interrupt time
    GpioDataRegs.GPBSET.bit.GPIO61 = 1; //check ADC interrupt time dung tam
    ADC_interrupt_cnt++;    // CHECKING COMPUTATION TIME

    //test
    // generate inputs
//    ramp = Ramp_gen(FREQ_GRID, FREQ_SAMPLING, &temp_angle); //ramp output -> [0,1]
//    V_low_gen = V_low_gen_pk*cos(ramp*2*PI);
//    I_low_gen = I_low_gen_pk*cos(ramp*2*PI);
//
//    I_low = I_low_gen;
//    I_high = I_high_gen;
//    V_low = V_low_gen;
//    V_high = V_high_gen;
    //test

    A3_adc =  AdcaResultRegs.ADCRESULT0;// SOC0
    B0_adc =  AdcbResultRegs.ADCRESULT0;// SOC0
//    C4_adc =  AdccResultRegs.ADCRESULT0;// SOC0
//    D3_adc =  AdcdResultRegs.ADCRESULT0;// SOC0
    D2_adc = AdcdResultRegs.ADCRESULT0;// SOC0
    A4_adc =  AdcaResultRegs.ADCRESULT1;// SOC1

    ADC_I_low = A3_adc;
    ADC_I_high = B0_adc;
//    ADC_V_low = C4_adc;
//    ADC_V_high = D3_adc;
    ADC_V_low = D2_adc;
    ADC_V_high = A4_adc;

    real_I_low = scaled_sensor_CURR(ADC_I_low, offset_IC_I_low, offset_Opam_I_low, offset_Comp_I_low, R1_I_low, R2_I_low);
    real_I_high = scaled_sensor_CURR(ADC_I_high, offset_IC_I_high, offset_Opam_I_high, offset_Comp_I_high, R1_I_high, R2_I_high);

    real_V_low = scaled_sensor_VOL(ADC_V_low, offset_IC_V_low, offset_Opam_V_low, offset_Comp_V_low, R1_V_low, R2_V_low, Rsen1_V_low, Rsen2_V_low);
    real_V_high = scaled_sensor_VOL(ADC_V_high, offset_IC_V_high, offset_Opam_V_high, offset_Comp_V_high, R1_V_high, R2_V_high, Rsen1_V_high, Rsen2_V_high);

    I_low = - zTF_1st(real_I_low, &I_low1, coef1stLPF);     // reversed because of layout reversal of input & output
    I_high = - zTF_1st(real_I_high, &I_high1, coef1stLPF);  // reversed because of layout reversal of input & output
    V_low = zTF_1st(real_V_low, &V_low1, coef1stLPF);
    V_high_preBSF = zTF_1st(real_V_high, &V_high1, coef1stLPF);
    V_high = zTF_2nd(V_high_preBSF, &V_high2, coef2ndBSF);
    V_high = zTF_1st(real_V_high, &V_high1, coef1stLPF);
//    V_high = 0;

    V_low_albe1 = Vgrid1phase_to_albe(pre_error_V_low_PLL, error_V_low_PLL, V_low, SAMPLING_TIME, 2.1, V_low_albe);
    V_low_albe = V_low_albe1;
    V_low_DQ = albe2dq(V_low_albe, refsin, refcos); // alpha- beta to dq
    srf_pll.v_q[0] = V_low_DQ.Q;
    SRF_PLL_FUNC(&srf_pll);
    theta_out = srf_pll.theta[1];

    // Update refsin and refcos variable to implement DQ transformation
    refsin = sin(theta_out);
    refcos = cos(theta_out);

    // Detect positive and negative cycles
    POS =(theta_out >= 0.03) && (theta_out <= 3.11);
    NEG =(theta_out >= 3.18) && (theta_out <= 6.25);

    // Protection
//    if (fabs(V_low) > Max_V_low)
//    {
//        ACDC_TEST = FAULT;
//        checkFault.bit.overVin = 1;
//    }
//    else if (fabs(I_low) > Max_I_low)
//    {
//        ACDC_TEST = FAULT;
//        checkFault.bit.overIin = 1;
//    }
//
//    else if (fabs(V_high) > Max_V_high)
//    {
//        ACDC_TEST = FAULT;
//        checkFault.bit.overVout = 1;
//    }
//
//    else if (fabs(I_high) > Max_I_high)
//    {
//        ACDC_TEST = FAULT;
//        checkFault.bit.overIout = 1;
//    }

    // Test Case
    switch(ACDC_TEST)
    {
        case SYSTEM_STANDBY:
            LED(D3,1);
            LED(D4,0);
            LED(D5,0);
            LED(D8,0);
            LED(D9,0);
            LED(D10,0);
            // Disable PWM Buffer
            ePWM_1to4_BUFF_OFF;
            ePWM_5_12_BUFF_OFF;
            RUN = 0;
            break;

        case START_UP:
            // Enable PWM Buffer
            ePWM_1to4_BUFF_ON;
            ePWM_5_12_BUFF_ON;
            // need more
            break;

        case FAULT:
            // Disable PWM Buffer
            ePWM_1to4_BUFF_OFF;
            ePWM_5_12_BUFF_OFF;
            // Enable ForceOFF PWM
            ForceOFFPWM(1,1);
            ForceOFFPWM(2,1);
            ForceOFFPWM(3,1);
            ForceOFFPWM(4,1);
            // Turn on LEDs
            LED(D3,1);
            LED(D4,1);
            LED(D5,1);
            LED(D8,1);
            LED(D9,1);
            LED(D10,1);
            break;

        case RESET:
            checkFault.all = 0;

            RUN = 0;
            memset(&CTL, 0, sizeof(CTL)); //Reset all values to 0

            ACDC_TEST = SYSTEM_STANDBY;
            break;

        case TEST_DC_IN_NOZVT_POS:
            LED(D3,0);
            LED(D4,0);
            LED_BLINK(&led5, FREQ_SAMPLING, 0.25);
            LED(D8,0);
            LED(D9,0);
            LED(D10,0);
            if(RUN == 1)
            {
                // Enable PWM Buffer
                ePWM_1to4_BUFF_ON;
                ePWM_5_12_BUFF_ON;
                // Adjust duty and snubber switch duty
                Duty = 0.5;
//                tSs = 200e-9;
//                DSs = tSs*FREQ_SW;
//                DSs = 0.05*Duty;  //  3%
                Dead = 0.06*Duty; //  3%
//                tSs = DSs/FREQ_SW;
                tdead = Dead/FREQ_SW;
            }
            // Set Duty for switches
            //PWM2A is for Ss, PWM2B is for S2
            //PWM3A is for S1, PWM3B is for Sb1
            //PWM4A is for S4
            //PWM5A is for S3, PWM5B is for Sb2
//            updateDutyA(2,DSs);
            FORCE_PWM(2,OFF,A);              //Ss OFF
            FORCE_PWM(3,OFF,B);              //Sb1 OFF
            FORCE_PWM(5,OFF,B);             //Sb2 OFF
            updateDutyA(3,Duty + Dead);     //S1 switch
            updateDutyB(2,Duty);            //S2 switch (main)
            FORCE_PWM(5,OFF,A);             //S3 OFF
            FORCE_PWM(4,ON,A);              //S4 ON
            break;


//
//                if(ch == 2)
//                   {
//                       //PWM2A is for Ss, PWM2B is for S2
//                       ePWM_Regs[ch]->AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on zero
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event A
//                       ePWM_Regs[ch]->AQCTLB.bit.CAU = AQ_SET;            // Set PWM2B on event A
//                       ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM2B on event B
//                   }
//
//                   else if(ch == 3)
//                   {
//                       //PWM3A is for S1, PWM3B is for Sb1
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM3A on event A
//                       ePWM_Regs[ch]->AQCTLA.bit.PRD = AQ_CLEAR;          // Clear PWM3A on TBPRD
//                       ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
//                   }
//
//                   else if(ch == 4)
//                   {
//                       //PWM4A is for S4
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM4A on event A
//                       ePWM_Regs[ch]->AQCTLA.bit.CBU = AQ_CLEAR;          // Clear PWM4A on event B
//                       ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
//                   }
//
//                   else if(ch == 5)
//                   {
//                       //PWM5A is for S3, PWM5B is for Sb2
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM5A on event A
//                       ePWM_Regs[ch]->AQCTLA.bit.PRD = AQ_CLEAR;          // Clear PWM5A on TBPRD
//                       ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
//                   }
//
                //PWM2A is for Ss, PWM2B is for S2
                //PWM3A is for S1, PWM3B is for Sb1
                //PWM4A is for S4
                //PWM5A is for S3, PWM5B is for Sb2


        case TEST_DC_IN_NOZVT_NEG:
            if(RUN == 1)
            {
                // Enable PWM Buffer
                ePWM_1to4_BUFF_ON;
                ePWM_5_12_BUFF_ON;
                // Adjust duty and snubber switch duty
                Duty = 0.5;
//                tSs = 200e-9;
//                DSs = tSs*FREQ_SW;
//                DSs = 0.05*Duty;  //  3%
                Dead = 0.06*Duty; //  3%
//                tSs = DSs/FREQ_SW;
                tdead = Dead/FREQ_SW;
            }
            FORCE_PWM(2,OFF,A);              //Ss OFF
            FORCE_PWM(3,OFF,B);             //Sb1 OFF
            FORCE_PWM(5,OFF,B);              //Sb2 OFF
            FORCE_PWM(3,OFF,A);             //S1 OFF
            FORCE_PWM(2,ON,B);              //S2 ON
            updateDutyA(5,Duty + Dead);     //S3 switch

            updateDutyA(4,DSs);             //S4 switch (main)
            updateDutyB(4,Duty);
            break;

        case TEST_DC_IN_POS:
            LED(D3,0);
            LED(D4,0);
            LED_BLINK(&led5, FREQ_SAMPLING, 0.25);
            LED(D8,0);
            LED(D9,0);
            LED(D10,0);
            if(RUN == 1)
            {
                // Enable PWM Buffer
                ePWM_1to4_BUFF_ON;
                ePWM_5_12_BUFF_ON;
                // Adjust duty and snubber switch duty
                Duty = 0.5;
//                tSs = 200e-9;
//                DSs = tSs*FREQ_SW;
                DSs = 0.05;  //  3%
                Dead = 0.03; //  3%
                tSs = DSs/FREQ_SW;
                tdead = Dead/FREQ_SW;
            }
            // Set Duty for switches
            //PWM2A is for Ss, PWM2B is for S2
            //PWM3A is for S1, PWM3B is for Sb1
            //PWM4A is for S4
            //PWM5A is for S3, PWM5B is for Sb2
            updateDutyA(2,DSs);
//            V_low = 10; // test
//            FORCE_PWM(3,ON,B);              //Sb1 ON
            //test
            updateDutyB(3,DSs*3);
            //test
            FORCE_PWM(5,OFF,B);             //Sb2 OFF
            updateDutyA(3,Duty + Dead);     //S1 switch
            updateDutyB(2,Duty);            //S2 switch (main)
            FORCE_PWM(5,OFF,A);             //S3 OFF
            FORCE_PWM(4,ON,A);              //S4 ON
            break;


//
//                if(ch == 2)
//                   {
//                       //PWM2A is for Ss, PWM2B is for S2
//                       ePWM_Regs[ch]->AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on zero
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event A
//                       ePWM_Regs[ch]->AQCTLB.bit.CAU = AQ_SET;            // Set PWM2B on event A
//                       ePWM_Regs[ch]->AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM2B on event B
//                   }
//
//                   else if(ch == 3)
//                   {
//                       //PWM3A is for S1, PWM3B is for Sb1
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM3A on event A
//                       ePWM_Regs[ch]->AQCTLA.bit.PRD = AQ_CLEAR;          // Clear PWM3A on TBPRD
//                       ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
//                   }
//
//                   else if(ch == 4)
//                   {
//                       //PWM4A is for S4
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM4A on event A
//                       ePWM_Regs[ch]->AQCTLA.bit.CBU = AQ_CLEAR;          // Clear PWM4A on event B
//                       ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
//                   }
//
//                   else if(ch == 5)
//                   {
//                       //PWM5A is for S3, PWM5B is for Sb2
//                       ePWM_Regs[ch]->AQCTLA.bit.CAU = AQ_SET;            // Set PWM5A on event A
//                       ePWM_Regs[ch]->AQCTLA.bit.PRD = AQ_CLEAR;          // Clear PWM5A on TBPRD
//                       ePWM_Regs[ch]->AQCTLB.all = 0;                     // Disable Action Qualifier Control
//                   }
//
                //PWM2A is for Ss, PWM2B is for S2
                //PWM3A is for S1, PWM3B is for Sb1
                //PWM4A is for S4
                //PWM5A is for S3, PWM5B is for Sb2
        case TEST_DC_IN_NEG:
            LED(D3,0);
            LED(D4,0);
            LED_BLINK(&led5, FREQ_SAMPLING, 0.25);
            LED(D8,0);
            LED(D9,0);
            LED(D10,0);
            if(RUN == 1)
            {
                // Enable PWM Buffer
                ePWM_1to4_BUFF_ON;
                ePWM_5_12_BUFF_ON;
                // Adjust duty and snubber switch duty
                Duty = 0.5;
//                tSs = 200e-9;
//                DSs = tSs*FREQ_SW;
                DSs = 0.05*Duty;  //  3%
                Dead = 0.03*Duty; //  3%
                tSs = DSs/FREQ_SW;
                tdead = Dead/FREQ_SW;
            }
            updateDutyA(2,DSs);
            FORCE_PWM(3,OFF,B);             //Sb1 OFF
//            FORCE_PWM(5,ON,B);              //Sb2 ON
            //test
            updateDutyB(5,DSs*3);
            //test
            FORCE_PWM(3,OFF,A);             //S1 OFF
            FORCE_PWM(2,ON,B);              //S2 ON
            updateDutyA(5,Duty + Dead);     //S3 switch

            updateDutyA(4,DSs);             //S4 switch (main)
            updateDutyB(4,Duty);
            break;

        case CLOSEDLOOP:
            LED(D3,0);
            LED_BLINK(&led5, FREQ_SAMPLING, 0.25);
            LED_BLINK(&led8, FREQ_SAMPLING, 0.25);
            LED_BLINK(&led4, FREQ_SAMPLING, 0.25);
            LED(D9,0);
            LED(D10,0);
            if((theta_out >= 0.0) && (theta_out <= 0.1)) RUN = 1;
            if(RUN == 1)
            {
                // Enable PWM Buffer
                ePWM_1to4_BUFF_ON;
                ePWM_5_12_BUFF_ON;
                // Duty calculation
//                CTL.errorV = V_high_ref - V_high;
//                CTL.PI_Vloop = zTF_1st(CTL.errorV, &CTL.errorV1, coeff_LF_Vloop);
//                CTL.PI_Vloop_lim = Limiter(CTL.PI_Vloop, 90, 0);
//                CTL.I_L_ref = CTL.PI_Vloop_lim*refcos;
//                CTL.errorI = abs(CTL.I_L_ref) - abs(I_low);
//                CTL.PI_Iloop = zTF_1st(CTL.errorI, &CTL.errorI1, coeff_LF_Iloop);
//                CTL.PI_Iloop_lim = Limiter(CTL.PI_Iloop, 0.5, -0.5);
//                CTL.V_low_rep = refcos*V_low_mag;
//                CTL.DFF = (V_high - abs(CTL.V_low_rep)) / V_high;
//                CTL.Duty_pre_lim = CTL.PI_Iloop_lim + CTL.DFF;
//                CTL.Duty = Limiter(CTL.Duty_pre_lim, 1, 0);
                // Adjust duty and snubber switch duty
//                Duty = CTL.Duty;

                //test
                Duty = (V_high - abs(V_low_gen)) / V_high;
                //test

                tSs = 1.3 * abs(I_low_gen) * Ls / V_high_ref + t_re;
                DSs = tSs*FREQ_SW;
                Dead = 0.03; // 3%
                // Set Duty for switches
                //PWM2A is for Ss, PWM2B is for S2
                //PWM3A is for S1, PWM3B is for Sb1
                //PWM4A is for S4
                //PWM5A is for S3, PWM5B is for Sb2
                //Ss Sb1 Sb2 ok,
                updateDutyA(2,DSs);
//                if(V_low>=0)
                if(POS)
                {
                    FORCE_PWM(3,ON,B);              //Sb1 ON
                    FORCE_PWM(5,OFF,B);             //Sb2 OFF

                    FORCE_PWM(3,NONE,A);             //disable force
                    updateDutyA(3,Duty + Dead);     //S1 switch
                    FORCE_PWM(2,NONE,B);             //disable force
                    updateDutyB(2,Duty);            //S2 switch (main)

                    FORCE_PWM(5,OFF,A);             //S3 OFF
                    FORCE_PWM(4,ON,A);              //S4 ON
                }
                else if(NEG)
//                if(NEG)
//                if(V_low<0)
                {
                    FORCE_PWM(3,OFF,B);             //Sb1 OFF
                    FORCE_PWM(5,ON,B);              //Sb2 ON
                    FORCE_PWM(3,OFF,A);             //S1 OFF
                    FORCE_PWM(2,ON,B);              //S2 ON

                    FORCE_PWM(5,NONE,A);            //disable force
                    updateDutyA(5,Duty + Dead);     //S3 switch

                    FORCE_PWM(4,NONE,A);            //disable force
                    updateDutyA(4,DSs);             //S4 switch (main)
                    updateDutyB(4,Duty);
                }
//                    // Update phase (check if this is necessary)
//                    updatePhase(1,0);
//                    updatePhase(2,0);
//                    updatePhase(3,0);
//                    updatePhase(4,0);
            }
            break;

        case CALIB_ADC:
            LED(D3,0);
            LED(D4,0);
            LED(D5,0);
            LED(D8,0);
            LED_BLINK(&led10, FREQ_SAMPLING, 0.25);
            LED_BLINK(&led9, FREQ_SAMPLING, 0.25);
            cnt_Calib++;
            // Disable PWM Buffer
            ePWM_1to4_BUFF_OFF;
            ePWM_5_12_BUFF_OFF;
            // Calculate offsets
            offset_Comp_I_low = get_offset(offset_Comp_I_low, real_I_low);
            offset_Comp_I_high = get_offset(offset_Comp_I_high, real_I_high);
            offset_Comp_V_low = get_offset(offset_Comp_V_low, real_V_low);
            offset_Comp_V_high = get_offset(offset_Comp_V_high, real_V_high);
            // check condition
            if (cnt_Calib >= CNT_5SEC)
            {
                cnt_Calib = 0;
                ACDC_TEST = SYSTEM_STANDBY;
            }
            break;

        default:
            // Disable PWM Buffer
            ePWM_1to4_BUFF_OFF;
            ePWM_5_12_BUFF_OFF;
            // Enable ForceOFF PWM
            ForceOFFPWM(1,1);
            ForceOFFPWM(2,1);
            ForceOFFPWM(3,1);
            ForceOFFPWM(4,1);
            break;
    }

    //test DAC
//    SetDAC(DACA, DC, ramp, 1);
    SetDAC(DACB, DC, theta_out, TWOPI);
    SetDAC(DACA, DC, Duty, 1);

//    GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1; // check adc interrupt time
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1; // check adc interrupt time dung tam
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // acknowledge interrupt to receive more interrupts from group 1
}

// epwm1_isr - EPWM1 ISR
//__interrupt void epwm1_isr(void)
//{
//    PWM1_interrupt_cnt++; // Testing interrupt
//
//    EPwm1Regs.ETCLR.bit.INT = 1; // Clear INT flag for this timer
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge interrupt
//}

void main(void)
{
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xD_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    InitGpio();

    // Init GPIO pins for ePWM1, ePWM2, ePWM3, ePWM4
    // These functions are in the F2837xD_EPwm.c file
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();

    ePWM_SOCSEL(1, ET_CTR_ZERO, 15);    // FREQ_SW/FREQ_SAMPLING = 250/50=5, 5 PWM trigger 1 ADC interrupt

    // Init PWM
    InitEPWM(1, FREQ_SW);   // to get a master (master is delayed with other slaves so we don't use the master)
    InitEPWM(2, FREQ_SW);   //Ss and S2
    InitEPWM(3, FREQ_SW);   //S1 and Sb1
    InitEPWM(4, FREQ_SW);   //S4
    InitEPWM(5, FREQ_SW);   //S3 and Sb2

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    // This will populate the entire table, even if the interrupt is not used in this example. This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Assign ISRs to interrupt vectors (Interrupts that are used in this file are re-mapped to ISR functions found within this file)
    EALLOW;  // Enable EALLOW protected register access
    PieVectTable.ADCA1_INT = &adca1_isr;   // Connect to ADC ISR (Point to interrupt function)
//        PieVectTable.EPWM1_INT = &epwm1_isr;   // Connect to PWM ISR (Point to interrupt function)
    EDIS;    // Disable EALLOW protected register access

    // Configure the ADC and power it up
    ConfigureADC();

    ConfigureDAC();

//      For this example, only initialize the ePWM
        EALLOW;
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
        EDIS;

        EALLOW;
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
        EDIS;
    //

    //Setup the ADC for ePWM triggered conversions
    SetupADCEpwm();

    // Step 4. User specific code, enable interrupts:
    IER |= M_INT1; // Enable group 1 interrupts (for ADC)
//        IER |= M_INT3; // Enable group 3 interrupts (for PWM)

    // Enable PIE (peripheral interrupt expansion) for ADC
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable PIE for PWM1
//        PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    InitGPIOs(); // Init used GPIOs

    // Calculate coefficients
    coeff_LF_PLL = getcoeff_PI(PI_PLL.Kp, PI_PLL.Ki, SAMPLING_TIME);        // coeff of loop filter in PLL (PI)
    PLL_INIT(FREQ_GRID, SAMPLING_TIME, coeff_LF_PLL ,&srf_pll);             // init values for PLL algorithm
    coef1stLPF =  get1stLPFcoef(5000.0, FREQ_SAMPLING);                     //coeff for LPF
    // cutoff frequency 5000, neu nho qua thi bi tre pha va giam gia tri, cao qua thi ko loc tot
    coef2ndBSF = get2ndBSFcoef(2*FREQ_GRID, 1, 20, FREQ_SAMPLING); //gain = 1, stop band 20Hz
    coeff_LF_Vloop = getcoeff_PI(PI_Vloop.Kp, PI_Vloop.Ki, SAMPLING_TIME);  // coeff of loop filter in Vloop (PI)
    coeff_LF_Iloop = getcoeff_PI(PI_Iloop.Kp, PI_Iloop.Ki, SAMPLING_TIME);  // coeff of loop filter in Iloop (PI)
//    Dead = tdead/SAMPLING_TIME; // Duty deadtime;

    // Step 5. IDLE loop. Just sit and loop forever (optional):
    for(;;)
    {

    }
}
