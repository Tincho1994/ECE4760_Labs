/*********************************************************************
 *********************************************************************
 * Kelsey Nedd, Martin Herrera, Jonah Mittler Cornell University
 * Nov 2017
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
// core frequency we're running at // peripherals at 40 MHz
#define	SYS_FREQ 40000000 //need  to decide on a frequency


//===========================DAC SETUP FOR MOTOR COTROL================================
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
// pullup/down macros for button
// PORT B
//#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
//
//#define DisablePullDownB(bits) CNPDBCLR=bits;
//#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
//#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;
////////////////////////////////////

// string buffer
char buffer[60];

//threads
static struct pt pt_adc, pt_button, pt_gui, pt_time, pt_feedback;

//system timer
int sys_time_seconds=0;

//PWM period
int generate_period = 40000;

//key variables
int threshold = 0; //flip threshold
int threshold_2 = 0; //finish threshold
int menuSel = 0; //menu selection
int start = 0; //start motor feedback
int duty = 0; //duty for PWM
static float temp;

//adc variables for reading adc and storing values
//adc 1 is the thermistor
//adc_2 is the slider 
static int adc_1;
static int adc_2;
static float temp_2 = 0.0;

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    mT2ClearIntFlag();
//    set PWM for motor 1
//    SetDCOC3PWM(generate_period/4); 
    
    //set PWM for flip motor 
    SetDCOC2PWM(duty);
}

//=======ADC THREAD===========================
static PT_THREAD (protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);
    //these are the differenct constants for our adc to temperature conversion
    static float p1, p2, p3, p4;
    p1 = 1.456*0.0000001;
    p2 = 4.596*0.00001;
    p3 = 0.0334746;
    p4 = 55.78; 
    //variables used to average the temp readings and steady the printed output
    static int adcF1 = 0;
    static int adcF2 = 0;
    static int adcF3 = 0;
    while(1) {
           // yield time 1 second
           PT_YIELD_TIME_msec(1000);

           // read the ADC AN1 
           adc_1 = ReadADC10(0); //read the thermistor
           adc_2 = ReadADC10(1); //read the slide potentiometer
           
           //Manual control of height motor
           if(adc_2 < 10){
               mPORTASetBits(BIT_3);//moves motor up
               while(adc_2 < 10) adc_2 = ReadADC10(1);//checks for when pot slid back into stop position 
               mPORTAClearBits(BIT_3); //stop motor
           }
           else if(adc_2 > 1010){
               mPORTBSetBits(BIT_7);//moves motor down   
               while(adc_2 > 1010) adc_2 = ReadADC10(1); //checks for pot slid back to stop position
               mPORTBClearBits(BIT_7); //stop motor
           }   
           
           adcF1 = adcF2;
           adcF2 = adcF3;
           adcF3 = adc_1;
           //averaging of recent adc readings
           if (adcF1 > 0 & adcF2 > 0) adc_1 = (adcF1+adcF2+adcF3)/3;
           
           //calculating degrees Fahrenheit 
           temp = (p1*(adc_1*adc_1*adc_1))-(p2*(adc_1*adc_1))+(p3*adc_1)+p4; 
           
           //displaying adc value and temperature
           tft_fillRoundRect(15, 225, 150, 50, 1, ILI9340_BLACK);// x,y,w,h,radius,color
           tft_setCursor(15, 250);
           tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
           sprintf(buffer,"%d %6.3f", adc_1, temp); 
           tft_writeString(buffer);
           } // END WHILE(1)
  PT_END(pt);
} 

//==============================================================================
// === One second Thread ======================================
// update a 1 second tick counter
static PT_THREAD (protothread_time(struct pt *pt))
{
    PT_BEGIN(pt);
      
      while(1) {
            // yield time 1 second
            tft_fillRoundRect(130, 10, 55, 50, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(15, 10);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            //draws cook time on tft
            sprintf(buffer,"cook time: %d", sys_time_seconds); 
            tft_writeString(buffer);
            //yield for one second
            PT_YIELD_TIME_msec(1000);
            sys_time_seconds++;      
            // NEVER exit while
      } // END WHILE(1)

  PT_END(pt);
} 
// thread ======================================================================

//=======BUTTON THREAD=============================================================
static PT_THREAD (protothread_button(struct pt *pt))
{
    PT_BEGIN(pt);
	//Sets ports as input
    mPORTASetPinsDigitalIn(BIT_2);
    //turns on pull-down on inputs
    EnablePullDownA(BIT_2);

      while(1) {
          PT_YIELD_TIME_msec(200);
		  //reads if button is pressed
          if(mPORTAReadBits(BIT_2)){ 
			  //moves menu selector by incrementing 	
              menuSel++;
			  //cylces back to top of menu
              if(menuSel > 3){
                menuSel=0;
              }
          }
//===============Setup height controls==========================================          

          //====how do you want your meat cooked=================
		  
		  //RARE
          if (threshold < 200){
            if (menuSel == 0){
              threshold = 100;
              threshold_2 = 120; //125-130 internal temp rare beef
            }
             //Medium Rare
            if (menuSel == 1){
                    threshold = 100; //130-140 internal temp medium beef
                    threshold_2 = 130;
            }
            //Medium
            if (menuSel == 2){
                    threshold = 100; //140-150 internal temp medium beef
                    threshold_2 = 140;
            }
            //Medium Well
            if (menuSel == 3){
                    threshold = 100; // 150-155 internal temp medium beef
                   threshold_2 = 145; 
            }
  //          Well Done
  //          if (menuSel == 4){
  //                  threshold = 100; // 150-155 internal temp medium beef
  //                 threshold_2 = 150; 
  //          }
            } 
      }
    PT_END(pt);
}    
// =============================================================================
// ==========MENU DISPLAY THREAD================================================
static PT_THREAD (protothread_gui(struct pt *pt))
{
    PT_BEGIN(pt);
      while(1) {
            // print every 200 mSec
            PT_YIELD_TIME_msec(200);
			//menu selector circle 
            int circlePos;
            // erase
            tft_fillRoundRect(5, 45, 10, 205, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            //draw selection circle and display menu select in correct location 
            circlePos = menuSel*50+50;
            tft_fillCircle(10, circlePos, 5, ILI9340_YELLOW);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            // sets the cursor for Rare 
            tft_setCursor(15, 50);
            //writes selection on User interface
            sprintf(buffer," Rare");
            tft_writeString(buffer);
            //sets cursor for Medium Rare
             tft_setCursor(15, 100);
            //writes selection on User interface
            sprintf(buffer," Medium Rare");
            tft_writeString(buffer);
            // sets the cursor for Medium
            tft_setCursor(15, 150);
            //writes selection on User interface
            sprintf(buffer," Medium");
            tft_writeString(buffer); 
            // sets the cursor for the Well done 
            tft_setCursor(15, 200);
            //writes selection on User interface
            sprintf(buffer," Medium Well");
            tft_writeString(buffer); 
//            tft_setCursor(15, 250);
//            //writes selection on User interface
//            sprintf(buffer,"Well Done");
//            tft_writeString(buffer); 
      } // END WHILE(1)
    PT_END(pt);
}
//==============================================================================
//=================MOTOR CONTROL/FEEDBACK LOOP==================================
static PT_THREAD (protothread_feedback(struct pt *pt))  
{                                                       
    PT_BEGIN(pt);
      while(1) {
           if (temp_2==0) {
              //makes sure to set both bits low in beginning for motor to be still  
              mPORTBClearBits(BIT_7);
              mPORTAClearBits(BIT_3);
           }
           //temp_2 is temporary variable for actual temperature reading 
           temp_2 = temp;
           //if temp crossed initial threshold, initiate flip 
           if (temp_2 > threshold){
                mPORTASetBits(BIT_3); //moves up 
                PT_YIELD_TIME_msec(30000); //wait 30 seconds of ascension  
                mPORTAClearBits(BIT_3); //stop motor 
                PT_YIELD_TIME_msec(100); //wait0.1 sec 
                duty = 8000; //initiate PWM at 8000 duty cycle
                PT_YIELD_TIME_msec(2140); //Yield for time of 180 degree turn
                duty = 0; //set duty back to 0 to stop motor
                PT_YIELD_TIME_msec(100); //wait 0.1 sec
                mPORTBSetBits(BIT_7); //descend motor
                PT_YIELD_TIME_msec(30000); //wait 30 seconds of descension
                mPORTBClearBits(BIT_7); //stop motor
                threshold = 300; //threshold unreachable to stop repeat activation of process 
           }
           //checks for temperature crossing finish threshold
           if (temp_2 > threshold_2){
               mPORTASetBits(BIT_3); //move up
                PT_YIELD_TIME_msec(30000); //ascend 30 seconds
                mPORTAClearBits(BIT_3); //stop motor
                threshold_2 = 300; //unreachable threshold
           }
          
          PT_YIELD_TIME_msec(1000);
      } // END WHILE(1)
    PT_END(pt);
}
//================================================================================

int main(void)
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
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 40000);
		//set up the timer interrupt with a priority of 2
        ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
        mT2ClearIntFlag(); // and clear the interrupt flag
		
    
    mPORTBSetPinsDigitalOut(BIT_7); //set pin for hbridge motor control
     mPORTASetPinsDigitalOut(BIT_3); //sets pin for hbridge motor control
   
        // setup system wide interrupts  ///
        INTEnableSystemMultiVectoredInt();
   //output compare setup for PWM 
    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , generate_period, generate_period);
    // OC2 is PPS group 2, map to RPB5 (pin 14)
    PPSOutput(2, RPB5, OC2);
    
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
   #define PARAM4 ENABLE_AN1_ANA | ENABLE_AN11_ANA

   // define setup parameters for OpenADC10
   // do not assign channels to scan
   #define PARAM5 SKIP_SCAN_ALL //|SKIP_SCAN_AN5 //SKIP_SCAN_AN1 |SKIP_SCAN_AN5  //SKIP_SCAN_ALL
    
     SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN11); // configure to sample AN11 
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC

    PT_INIT(&pt_time);
    PT_INIT(&pt_adc);
    PT_INIT(&pt_feedback);
    PT_INIT(&pt_gui);
    PT_INIT(&pt_button);
    
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
     //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
    
     PT_setup();
	while(1)
	{
        PT_SCHEDULE(protothread_adc(&pt_adc));
        PT_SCHEDULE(protothread_button(&pt_button));
        PT_SCHEDULE(protothread_gui(&pt_gui));
        PT_SCHEDULE(protothread_feedback(&pt_feedback));
        PT_SCHEDULE(protothread_time(&pt_time));
 	}
}
