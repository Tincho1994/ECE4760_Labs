/*********************************************************************
 *
 *                  SPI to  MCP4822 dual channel 12-bit DAC
 *
 *********************************************************************
 * Bruce Land Cornell University
 * Spet 2015
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include <stdlib.h>
#include <plib.h>
#include <xc.h> // need for pps
#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "tft_master.h"
#include "tft_gfx.h"

// threading library
#include <plib.h>
#include "pt_cornell_1_2_1.h"

// Configuration Bit settings
// SYSCLK = 40 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//                       8MHZ                          4MHz               80MHz            40      <---    40MHz
//#pragma config FNOSC = FRCPLL, POSCMOD = OFF, FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
//#pragma config FWDTEN = OFF
//#pragma config FSOSCEN = OFF, JTAGEN = OFF, DEBUG = OFF

// core frequency we're running at // peripherals at 40 MHz
#define	SYS_FREQ 40000000

/* ====== MCP4822 control word ==============
bit 15 A/B: DACA or DACB Selection bit
1 = Write to DACB
0 = Write to DACA
bit 14 ? Don?t Care
bit 13 GA: Output Gain Selection bit
1 = 1x (VOUT = VREF * D/4096)
0 = 2x (VOUT = 2 * VREF * D/4096), where internal VREF = 2.048V.
bit 12 SHDN: Output Shutdown Control bit
1 = Active mode operation. VOUT is available. ?
0 = Shutdown the selected DAC channel. Analog output is not available at the channel that was shut down.
VOUT pin is connected to 500 k???typical)?
bit 11-0 D11:D0: DAC Input Data bits. Bit x is ignored.
*/
////////////////////////////////////
// pullup/down macros for keypad
// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;

#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;
////////////////////////////////////

////////////////////////////////////
// some precise, fixed, short delays
// to use for extending pulse durations on the keypad
// if behavior is erratic
#define NOP asm("nop");
// 1/2 microsec
#define wait20 NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// one microsec
#define wait40 wait20;wait20;


// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
// string buffer
char buffer[60];
//photothreads
static struct pt pt_timer, pt_adc, pt_print, pt_key;


// note that UART input and output are threads
static struct pt pt_cmd, pt_time, pt_input, pt_output, pt_DMA_output;

//system timer
int sys_time_seconds;

//The actual period of the wave
int generate_period = 40000;
//int pwm_on_time = 500;
//print state variable
//int printing=0;

//KEYPAD STATEMACHINE
int pushState = 0;

//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data, motor;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2;	// the SPI channel to use
volatile int spiClkDiv = 2; // 20 MHz max speed for this DAC

//adc variables for reading adc and storing values
//adc 5 is the menu control pot and adc 1 is the angle reading pot
static int adc_1;
static int adc_5;

//=======Timer 3 ISR===========================================
static float glob_angle = 670; //global angle variable
volatile float accu_err = 0; //accumulated error
volatile float err; //error
//PID coefficients (proportinal, differential, integral)
static float kp = 148, kd = 16000, ki = 0.0975;


void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void)
{
	//both adc channels are read
    adc_1 = ReadADC10(0);
    adc_5 = ReadADC10(1);
     
	//setting DAC data to read adc value 
	//halved to represent 0-180 degrees 
    DAC_data = adc_1<<2;
    
	//variable for last error and control variables
    volatile float last_err;
    volatile float prop_cntl, diff_cntl, int_cntl;
    volatile int v_cntl;
	//recently calculated error is last error
    last_err = err;
	//err is the previous angle minus the new angle  
    err = glob_angle - adc_1;
	//accumulated error
    accu_err = accu_err+err;
	//control variables are updated using PID coefficients
    prop_cntl = kp*err;
    diff_cntl = kd*(err-last_err);
	
	//cuts accu_error to 95% to eliminate shaking around desired angle
    if((err>0&&last_err<0)||(err<0&&last_err>0)){
        accu_err = accu_err*0.95;
    }
    //integral control variable updated 
    int_cntl = ki*accu_err;
    //pwm control variable calculated
    v_cntl = (int)(prop_cntl + diff_cntl + int_cntl);
    //no negative values
    if(v_cntl<0){
        v_cntl = 0;
    }
	//if over max put one under max period time
    else if (v_cntl > generate_period-1){
        v_cntl = generate_period-1;
    }
    //set PWM 
    SetDCOC3PWM(v_cntl);
    
    
    //===POT DAC======
       // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
    // test for ready
     while (TxBufFullSPI2());
     // write to spi2 the DAC DATA
     DAC_data = (DAC_data+1 & 0xfff);
     WriteSPI2(DAC_config_chan_A | DAC_data);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_4); // end transaction
     
    mPORTBClearBits(BIT_4); // start transaction
    // test for ready
     while (TxBufFullSPI2());
     // write to spi2 and control motor 
     //motor = ((int)v_cntl+1 & 0xfff);
     motor = (v_cntl+motor)>>3;
     //motor = motor + ((v_cntl - motor)>>4);
     
     WriteSPI2(DAC_config_chan_B | motor>>4);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_4); // end transaction
    
    mT3ClearIntFlag();
    //==================
}
//========================================

//menu select varaible for moving selector 
int menuSel = 0; 

//=====Print Thread==========================
static PT_THREAD (protothread_print(struct pt *pt))
{
    PT_BEGIN(pt);
      while(1) {
            // print every 200 mSec
            PT_YIELD_TIME_msec(200);
			//menu selector circle 
            int circlePos;
			//angle calculation from adc unit to degrees
            float angle1 = (float)(glob_angle-670)*0.3516;
            // erase
            tft_fillRoundRect(0,0, 250, 300, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            // draw selection circle and display menuSel in right location 
            circlePos = menuSel*50+50;
            tft_fillCircle(10, circlePos, 5, ILI9340_YELLOW);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            // sets the cursor for kp coefficient 
            tft_setCursor(15, 50);
            //sets and writes the proportional coefficient on tft
            sprintf(buffer," kp: %.0f", kp);
            tft_writeString(buffer);
            // sets the cursor for the kd coefficient
            tft_setCursor(15, 100);
            ////sets and writes the differential coefficient on tft
            sprintf(buffer," kd: %.0f",kd);
            tft_writeString(buffer); 
            // sets the cursor for the ki coefficient
            tft_setCursor(15, 150);
            //sets and writes the integral coefficient on the tft
            sprintf(buffer," ki: %.5f",ki);
            tft_writeString(buffer); 
            // sets the cursor for the angle display on tft 
            tft_setCursor(15, 200);
            //sets and writes the angle string on the tft
            sprintf(buffer," Angle: %.0f %.0f",glob_angle, angle1);
            tft_writeString(buffer);
      } // END WHILE(1)
    PT_END(pt);
}
//==============================================
//Thread ussed for testing adc values
// static PT_THREAD (protothread_timer(struct pt *pt))
// {
    // PT_BEGIN(pt);
//    tft_setCursor(0, 0);
//    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
//    tft_writeString("Time elapsed:\n");
      // while(1) {
//        yield time .1 second
        // PT_YIELD_TIME_msec(100);
        // tft_fillRoundRect(0, 15, 235, 40, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        // tft_setCursor(0, 15);
        // tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
        // sprintf(buffer,"adcval1 = %d  adcval5 = %d", adc_1, adc_5);
        // tft_writeString(buffer);
        // }
  // PT_END(pt);
// } 
//================================================

//=======Key THREAD===============================
static PT_THREAD (protothread_key(struct pt *pt))
{
    PT_BEGIN(pt);
	//Sets ports as input
    mPORTBSetPinsDigitalIn(BIT_7| BIT_13);
    //turns on pull-down on inputs
    EnablePullDownB(BIT_7 | BIT_13);

      while(1) {
          PT_YIELD_TIME_msec(100);
		  //reads if button is pressed
          if(mPORTBReadBits(BIT_7)){ 
			  //moves menu selector by incrementing 	
              menuSel++;
			  //cylces back to top of menu
              if(menuSel > 3){
                  menuSel=0;
              }
          }
          
          //====updating kp&kd&ki=====
		  
		  //kp slot
          if (menuSel == 0){
			  //allow for data entry while second button is pushed
              while(mPORTBReadBits(BIT_13)){
				  //read value from value setting pot thru adc
                  adc_5 = ReadADC10(1);
				  //scales kp
                  kp = (float)adc_5/1023*600;
              }
          }
		  //kd slot
          if (menuSel == 1){
			  //second button allows for edits
              while(mPORTBReadBits(BIT_13)){
				  //reads set value
                  adc_5 = ReadADC10(1);
				  //kd is scaled
                  kd = (float)adc_5/1023*30000;
              }
          }
		  //ki slot
          if (menuSel == 2){
			  //second button allows for edits
              while(mPORTBReadBits(BIT_13)){
				  //reads set value from pot
                  adc_5 = ReadADC10(1);
				  //ki scaled
                  ki = (float)adc_5/1023*0.25;
              }
          }
		  //angle slot
          if (menuSel == 3){
			  //second button allows for edits
              while(mPORTBReadBits(BIT_13)){
				  //reads set value from pot
                  adc_5 = ReadADC10(1);
				  //edits gobal angle 
                  glob_angle = (float)adc_5;
              }
          }
          
      }   // END WHILE(1)
  PT_END(pt);
} // ====================================        

//=======ADC THREAD===========================
static PT_THREAD (protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);
    static int adc_1;
    static float V;
    static float angle;

    while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(60);
        
        // read the ADC AN11 
        // read the first buffer position
        adc_1 = ReadADC10(0);   // read the result of channel 9 conversion from the idle buffer
        adc_5 = ReadADC10(1);
        AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below

        // draw adc and voltage
        tft_fillRoundRect(15, 225, 100, 50, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(15, 250);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        // convert to voltage
        V = (float)adc_1 * 3.3 / 1023.0 ; // Vref*adc/1023'
        angle = (float)(adc_1-670)*0.3516;
        // convert to fixed voltage
        //Vfix = multfix16(int2fix16(adc_1), ADC_scale) ;
        
//        sprintf(buffer,"%d %6.3f", adc_1, angle);
//        sprintf(buffer,"%d %3.0f %6.3f", sys_time_seconds, glob_angle, angle);
        sprintf(buffer,"%6.3f, %d",angle,adc_1);
        tft_writeString(buffer);
        
     
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} 
//=======================================

// === One second Thread ======================================
// update a 1 second tick counter
static PT_THREAD (protothread_time(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1) {
            // yield time 1 second
            PT_YIELD_TIME_msec(1000) ;
            sys_time_seconds++ ;
            // NEVER exit while
      } // END WHILE(1)

  PT_END(pt);
} // thread ==================================================

//========COMMAND THREAD======================================
//thread for running desired lab demo of cycling thru angles
static PT_THREAD (protothread_cmd(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
        PT_YIELD_TIME_msec(100);
        //initial angle at 0 degrees
        if (sys_time_seconds<5){
            glob_angle = 0*1023/360+670;
        }
        //second angle at +30 degrees
        else if (sys_time_seconds<10){
            glob_angle = 30*1023/360+670;
        }
        //3rd angle at -30 degrees
        else if (sys_time_seconds<15){ 
            glob_angle = -30*1023/360+670;
        }
		//final angle back at 0 degrees
        else if (sys_time_seconds == 15){
            glob_angle = 0*1023/360+670;
        }
    }
    PT_END(pt);
}
//=================================================================


// ========================================================================
int	main(void)
{
      ANSELA = 0; ANSELB = 0; 
	// Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states, RAM
	// wait state and enable prefetch cache but will not change the PBDIV.
	// The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

        // timer interrupt //////////////////////////
        // Set up timer3 on,  interrupts, internal clock, prescalar 1, toggle rate
        //same as generate_period for timer
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 40000);
		//set up the timer interrupt with a priority of 2
        ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
        mT3ClearIntFlag(); // and clear the interrupt flag
		
        // set pulse to go high at 1/4 of the timer period and drop again at 1/2 the timer period
        OpenOC3(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE , generate_period, generate_period);
        // OC3 is PPS group 4, map to RPB9 (pin )
        PPSOutput(4, RPB9, OC3);

 /// SPI setup //////////////////////////////////////////
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    PPSOutput(4, RPB10, SS2);
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);        
    

        // setup system wide interrupts  ///
        INTEnableSystemMultiVectoredInt();

        //====ADC Setup=======
    // configure and enable the ADC
 CloseADC10(); // ensure the ADC is off before setting the configuration

// define setup parameters for OpenADC10
// Turn module on | ouput in integer | trigger mode auto | enable autosample
// ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
// ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
// ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
#define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

// define setup parameters for OpenADC10
// ADC ref external  | disable offset test | disable scan mode | do 2 sample | use single buf | alternate mode on
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_ON
        //
// Define setup parameters for OpenADC10
// use peripherial bus clock | set sample time | set ADC clock divider
// ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
// ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
// SLOW it down a little
#define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

// define setup parameters for OpenADC10
// set AN11 and  as analog inputs
#define PARAM4 ENABLE_AN1_ANA | ENABLE_AN5_ANA // 

// define setup parameters for OpenADC10
// do not assign channels to scan
#define PARAM5 SKIP_SCAN_ALL //|SKIP_SCAN_AN5 //SKIP_SCAN_AN1 |SKIP_SCAN_AN5  //SKIP_SCAN_ALL
 
// // configure to sample AN5 and AN1 on MUX A and B
SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 );
    
OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

EnableADC10(); // Enable the ADC

// TO READ the two channels
// adc_1 = ReadADC10(0);
// adc_5 = ReadADC10(1); 
    
     // init the threads
    //PT_INIT(&pt_timer);
    PT_INIT(&pt_adc);
    PT_INIT(&pt_cmd);
    PT_INIT(&pt_time);
    PT_INIT(&pt_print);
    PT_INIT(&pt_key);

    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
    
    
    
    PT_setup();
	while(1)
	{
           // toggle a bit for ISR perfromance measure
            //mPORTBToggleBits(BIT_1);
        //PT_SCHEDULE(protothread_timer(&pt_timer));
        PT_SCHEDULE(protothread_adc(&pt_adc));
        PT_SCHEDULE(protothread_print(&pt_print));
        PT_SCHEDULE(protothread_key(&pt_key));
        PT_SCHEDULE(protothread_cmd(&pt_cmd));
        PT_SCHEDULE(protothread_time(&pt_time));
 	}

}
