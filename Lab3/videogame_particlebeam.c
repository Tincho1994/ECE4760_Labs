#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "tft_master.h"
#include "tft_gfx.h"

// threading library
#include <plib.h>
#include "pt_cornell_1_2_1.h"

// config.h sets 40 MHz
#define	SYS_FREQ 40000000
#define sine_table_size 256
#define dmaChn 0
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000


static struct pt pt_dma; 

volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
//volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC

// string buffer
char buffer[60];
char strng[60];

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_color, pt_anim, pt_adc, pt_ball;

// system 1 second interval tick + scoring count
int sys_time_seconds, score=0, framerate, balls;

int begin_time;
int frame_time;
int begin_test;
int test_time=0;
int end_test;
int b_on;

// === the fixed point macros ========================================
typedef signed int fix16 ;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)


static PT_THREAD (protothread_dma(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
	    // enable dma channel to start sending wave to DAC
		DmaChnEnable(dmaChn);
		// wait 1 second and then disable
		PT_YIELD_TIME_msec(1000);
        DmaChnAbortTxfer(dmaChn);
		DmaChnDisable(dmaChn);
        PT_YIELD_TIME_msec(1000);
	}
	PT_END(pt);
}

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
//     tft_setCursor(0, 0);
//     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
//     tft_writeString("Time elapsed:\n");
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(400);
        tft_fillRoundRect(0, 10, 50, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
        sprintf(buffer,"%d %d", score, b_on);
        tft_writeString(buffer);
        
        tft_fillRoundRect(0, 20, 20, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 20);
        tft_setTextColor(ILI9340_BLUE); tft_setTextSize(1);
        sprintf(strng,"%d %d", frame_time, test_time);
        tft_writeString(strng);
        // draw sys_time
//        tft_fillRoundRect(0, 0, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 10);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
//        sprintf(buffer,"time elapsed:, %d", sys_time_seconds);
//        tft_writeString(buffer);
//         tft_fillRoundRect(0, 0, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//         
//        tft_setCursor(20, 0);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
//        sprintf(buffer,"score: %d", score);
//        tft_writeString(buffer);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Animation Thread =============================================
// update a 1 second tick counter
//static fix16 xc1=int2fix16(10), yc1=int2fix16(150), vxc1=int2fix16(10), vyc1=0;
//static fix16 xc2=int2fix16(200), yc2=int2fix16(150), vxc2=int2fix16(-10), vyc2=0;
//static fix16 xc[4]={int2fix16(25),int2fix16(75),int2fix16(50),int2fix16(110)}; 
//static fix16 yc[4]={int2fix16(150),int2fix16(150),int2fix16(10),int2fix16(62)};
//static fix16 vxc[4]={int2fix16(2),int2fix16(-2),int2fix16(2),int2fix16(2)};
//static fix16 vyc[4]={int2fix16(2),int2fix16(2),int2fix16(-2),int2fix16(2)};

#define NUM_BALLS 60

static fix16 xc[NUM_BALLS];
static fix16 yc[NUM_BALLS];
static fix16 vxc[NUM_BALLS];
static fix16 vyc[NUM_BALLS];
static int on[NUM_BALLS]; 
static int color_index[NUM_BALLS];

int k;
static int adc_9;

static void drawCirc (int x, int y, int color)
{
    tft_drawPixel(x-2,y,color);
    tft_drawPixel(x+2,y,color);
    tft_drawPixel(x-1,y+1,color);
    tft_drawPixel(x+1,y+1,color);
    tft_drawPixel(x,y-2,color);
    tft_drawPixel(x,y+2,color);
    tft_drawPixel(x-1,y-1,color);
    tft_drawPixel(x+1,y-1,color);
}
//static fix16 xc[2]={int2fix16(25),int2fix16(75)}; 
//static fix16 yc[2]={int2fix16(150),int2fix16(150)};
//static fix16 vxc[2]={int2fix16(1),int2fix16(-1)};
//static fix16 vyc[2]={int2fix16(1),int2fix16(1)};
static fix16 g = float2fix16(0.1), drag = float2fix16(0.00001);

static fix16 r12x, r12y, v12x, v12y, r12_dot_v12, dvx, dvy, mag_r12sq, overmag;

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);
    for (k=0; k<NUM_BALLS; k++){
    xc[k]=int2fix16(120);
    yc[k]=int2fix16(10);
    vxc[k] = int2fix16((rand()& 0xf)-10);
    vyc[k] = int2fix16(rand()& 0xf);
    color_index[k] = rand()& 0xf;
    }
    
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(32);
        begin_time = PT_GET_TIME();
        int i,j;
        int hit_count=0;
        for(i=0;i<NUM_BALLS;i++){
            // erase disk 
            if(on[i]==1){
                //tft_fillCircle(fix2int16(xc[i]), fix2int16(yc[i]), 4, ILI9340_BLACK); //x, y, radius, color
                drawCirc(fix2int16(xc[i]), fix2int16(yc[i]), ILI9340_BLACK);
                // update vel
                vxc[i] = vxc[i] - multfix16(vxc[i], drag);

                // update pos
                xc[i] = xc[i] + vxc[i];
                yc[i] = yc[i] + vyc[i];

                // walls
                if (xc[i]<int2fix16(10) || xc[i]>int2fix16(230)) vxc[i] = -vxc[i]; 
                if (yc[i]<int2fix16(5)) vyc[i] = -vyc[i];
                if (yc[i]>int2fix16(315)){
                    on[i] = 0;
                    b_on--;
                    xc[i]=int2fix16(120);
                    yc[i]=int2fix16(10);
                    vxc[i] = int2fix16(rand()%20-10);
                    vyc[i] = int2fix16(rand()%10);
                    score--;
                }
                // paddle
                if (absfix16(yc[i]-int2fix16(300))<int2fix16(4)){
                    if ((absfix16(yc[i]-int2fix16(300))<int2fix16(4)) &&
                        (xc[i]>int2fix16(adc_9*190>>10) && 
                            xc[i]<int2fix16((adc_9*190>>10)+50))){
                        vyc[i] = -vyc[i];
                    }
                }
                
                
                // new wall
                if (absfix16(yc[i]-int2fix16(200))<int2fix16(2)){
                    if ((absfix16(yc[i]-int2fix16(200))<int2fix16(1)) &&
                        (absfix16(xc[i]-int2fix16(75))<int2fix16(1)) &&
                        (absfix16(xc[i]-int2fix16(165))<int2fix16(1))){
                        vxc[i] = -vxc[i];
                    }
                    if (xc[i]<int2fix16(75) || xc[i]>int2fix16(165)){
                        if (vyc[i] > int2fix16(0)) vyc[i] = -vyc[i];//could clip thru
                        else{
                            on[i] = 0;
                            b_on--;
                            xc[i]=int2fix16(120);
                            yc[i]=int2fix16(10);
                            vxc[i] = int2fix16(rand()%20-10);
                            vyc[i] = int2fix16(rand()%10);
                            score++;
                        }
                    }
                }
                
                

            //NEST FOR LOOP HERE
                for (j=0; j<NUM_BALLS; j++){
                    if(on[j]==1){
                        if (j==i) ;//do nothing 
                        else if (absfix16(xc[i]-xc[j])<= int2fix16(4) && absfix16(yc[i]-yc[j])<= int2fix16(4)){                
            //                tft_fillRoundRect(0,20, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            //                tft_setCursor(0, 20);
            //                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            //                sprintf(strng,"enters loop");
            //                tft_writeString(strng);

                            r12x = xc[i]-xc[j];
                            r12y = yc[i]-yc[j];
                            mag_r12sq = multfix16(r12x, r12x)+ multfix16(r12y, r12y);
                            if (0 < mag_r12sq <= int2fix16(2)){  
                                fix16 ix = vxc[i];
                                fix16 iy = vyc[i];
                                vxc[i] = vxc[j];
                                vxc[j] = ix;
                                vyc[i] = vyc[j];
                                vyc[j] = iy;
                            }
                            else if(0< mag_r12sq < int2fix16(16) && hit_count==0){
                                begin_test = PT_GET_TIME();
                                overmag = divfix16(int2fix16(1),mag_r12sq);
                                v12x = vxc[i]-vxc[j];
                                v12y = vyc[i]-vyc[j];
                                r12_dot_v12 = multfix16(r12x, v12x)+ multfix16(r12y, v12y);
        //                        dvx = divfix16(multfix16(-r12x, r12_dot_v12), mag_r12sq); //bit shift by 4 instead possibly?
        //                        dvy = divfix16(multfix16(-r12y, r12_dot_v12), mag_r12sq);
                                dvx = multfix16(multfix16(-r12x, r12_dot_v12),overmag); //bit shift by 4 instead possibly?
                                dvy = multfix16(multfix16(-r12y, r12_dot_v12),overmag);

                                vxc[i] = vxc[i]+dvx;
                                vxc[j] = vxc[j]-dvx;
                                vyc[i] = vyc[i]+dvy;
                                vyc[j] = vyc[j]-dvy;
                                hit_count = 10;
                                end_test = PT_GET_TIME()-begin_test;
                                if(test_time<end_test){
                                    test_time=end_test;
                                };
                            }
                            else if (hit_count>0){
                                hit_count--;
                            }
                        }
                    }
                }
            
            //  draw disk
            
            
                if(color_index[i]==0){
                    drawCirc(fix2int16(xc[i]), fix2int16(yc[i]), ILI9340_GREEN); //x, y, radius, color
                }
                else if(color_index[i]==1){
                    drawCirc(fix2int16(xc[i]), fix2int16(yc[i]), ILI9340_RED); //x, y, radius, color
                }
                else if (color_index[i]==2){
                    drawCirc(fix2int16(xc[i]), fix2int16(yc[i]), ILI9340_YELLOW); //x, y, radius, color
                }
                else{
                    drawCirc(fix2int16(xc[i]), fix2int16(yc[i]), ILI9340_BLUE); //x, y, radius, color
                }
               
            }
            tft_fillRoundRect(0,200, 75, 2, 1, ILI9340_YELLOW);// x,y,w,h,radius,color
            tft_fillRoundRect(165, 200, 75, 2, 1, ILI9340_YELLOW);
            tft_fillRoundRect(adc_9*190>>10,300, 50, 2, 1, ILI9340_BLACK); //190 for width of paddle
            adc_9 = ReadADC10(0);
            tft_fillRoundRect(adc_9*190>>10,300, 50, 2, 1, ILI9340_BLUE);// x,y,w,h,radius,color
            
//            tft_fillRoundRect(adc_9*190>>10,300,50,2,1);
        }
        frame_time = PT_GET_TIME()-begin_time;
//        // erase disk
//         tft_fillCircle(fix2int16(xc1), fix2int16(yc1), 4, ILI9340_BLACK); //x, y, radius, color
//         tft_fillCircle(fix2int16(xc2), fix2int16(yc2), 4, ILI9340_BLACK); //x, y, radius, color
        // compute new velocities
//         vyc = vyc + g - multfix16(vyc, drag) ;
//         vxc1 = vxc1 - multfix16(vxc1, drag);
//         vxc2 = vxc2 - multfix16(vxc2, drag);
         
//         xc1 = xc1 + vxc1;
//         yc1 = yc1 + vyc1;
//         
//         xc2 = xc2 + vxc2;
//         yc2 = yc2 + vyc2;
//         
//         if (xc1<int2fix16(5) || xc1>int2fix16(235)) vxc1 = -vxc1; 
//         if (yc1>int2fix16(315)) vyc1 = -vyc1; 
//
//         if (xc2<int2fix16(5) || xc2>int2fix16(235)) vxc2 = -vxc2; 
//         if (yc2>int2fix16(315)) vyc2 = -vyc2;          
         //  draw disk
//         tft_fillCircle(fix2int16(xc1), fix2int16(yc1), 4, ILI9340_GREEN); //x, y, radius, color
//         tft_fillCircle(fix2int16(xc2), fix2int16(yc2), 4, ILI9340_RED); //x, y, radius, color 
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // animation thread

//=======BALL THREAD======================
static PT_THREAD (protothread_ball(struct pt *pt)){
    PT_BEGIN(pt);
    int m;
    int n;
    
    while(1){ 
        PT_YIELD_TIME_msec(300);
        if (frame_time<=67){
            for(m=0; m<NUM_BALLS; m++){
                if(on[m]==1);
                else{
                    on[m]=1;
                    b_on++;
                    PT_YIELD_TIME_msec(300);   
                }
            } 
        }
    }
    
    PT_END(pt);
}



//=========================================
//=======ADC THREAD=======================

static PT_THREAD (protothread_adc(struct pt *pt)){
       PT_BEGIN(pt);
    static int adc_9;
    //static float V;
    //static fix16 Vfix, ADC_scale ;
    
    //ADC_scale = float2fix16(3.3/1023.0); //Vref/(full scale)
            
    while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(60);
        
        // read the ADC AN11 
        // read the first buffer position
        adc_9 = ReadADC10(0);   // read the result of channel 9 conversion from the idle buffer

        // draw adc and voltage
//        tft_fillRoundRect(0,100, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 100);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        // convert to voltage
        //V = (float)adc_9 * 3.3 / 1023.0 ; // Vref*adc/1023
        // convert to fixed voltage
        //Vfix = multfix16(int2fix16(adc_9), ADC_scale) ;
        
        // print raw ADC, floating voltage, fixed voltage
        //sprintf(buffer,"%d %d.%03d", adc_9, fix2int16(Vfix), fix2int16(Vfix*1000)-fix2int16(Vfix)*1000);
        //tft_writeString(buffer);
        
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
}

void main(void) {
	SYSTEMConfigPerformance(PBCLK);

	ANSELA = 0; ANSELB = 0; CM1CON = 0; CM2CON = 0;

	// Create sin table, including DAC channel in entry for DMA
	static unsigned int sin_table[sine_table_size];
	int s;
    int i;
	for (i = 0; i < sine_table_size; i++){
		s = ((2047 * sin((float)i*6.283/(float)sine_table_size))) + 2048;
		sin_table[i] = DAC_config_chan_B | (short)s;
	}

	// Set up timer3 on,  interrupts, internal clock, prescalar 1, toggle rate
	// 5000 is 8 ksamples/sec at 40 MHz clock

	OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 5000);
//	// set up the timer interrupt with a priority of 2
    //ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
 	mT3ClearIntFlag(); // and clear the interrupt flag
//
//
//	// setup SPI
//	// SCK2 is pin 26 
//	// SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
	PPSOutput(2, RPB5, SDO2);
//
//
//	// setup spi channel
//	// clk divider set to 2 for 20 MHz
	SpiChnOpen(SPI_CHANNEL2, 
	        SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV | SPICON_FRMEN | SPICON_FRMPOL,
	        2);
//	// SS1 to RPB4 for FRAMED SPI
    PPSOutput(4, RPB10, SS2);
//
//	// Open the desired DMA channel.
//	// We enable the AUTO option, we'll keep repeating the sam transfer over and over.
	DmaChnOpen(dmaChn, 0, DMA_OPEN_AUTO);
//
//	// set the transfer parameters: source & destination address, source & destination size, number of bytes per event
//	// Setting the last parameter to one makes the DMA output one byte/interrupt
	DmaChnSetTxfer(dmaChn, sin_table, (void*)&SPI2BUF, sine_table_size*4, 4, 4);
//
//	// set the transfer event control: what event is to start the DMA transfer
//	// In this case, timer3/
	DmaChnSetEventControl(dmaChn, DMA_EV_START_IRQ(_TIMER_3_IRQ));

//====ADC Setup=======
    // configure and enable the ADC
    CloseADC10();	// ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | output in integer | trigger mode auto | enable autosample
    // ADC_FORMAT_INTG16 -- allows for the ADC to output in 16 bit integer values
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON 

    // ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
    // ADC_VREF_AVDD_AVSS -- Sets the reference voltages for the ADC with Vref+ at Vdd and Vref- at Vss
    //ADC_OFFSET_CAL_DISABLE -- disables the Offset calibrations which measures offset errors and subtracts it from the conversions. Not really necessary in our lab.
    //ADC_SCAN_OFF -- This setting scans through a selected vector of inputs. We disable it because we only need to scan for the potentiometer.
    //ADC_SAMPLES_PER_INT_1 -- This parameter sets up the ADC to perform one sample every time an interrupt is fired.
    //ADC_ALT_BUF_OFF -- Alternating buffer fills.
    //ADC_ALT_INPUT_OFF -- This is used by the ADC module to alternate between the two input multiplexers. We turned this off seeing as we?re only getting one sample and only need one input.    
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    //
    // use peripheral bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_PB -- This is the peripheral bus clock used to trigger conversions 
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 sets the sample time using the adc clock. 
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2


    // set AN11 and  as analog inputs
    #define PARAM4	ENABLE_AN11_ANA // pin 24

    // do not assign channels to scan
    #define PARAM5	SKIP_SCAN_ALL

    // use ground as neg ref for A and use AN11 for input A     
    // configured to sample AN11 
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11 ); // configure to sample AN11 
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC
    
    //==========================
    
     // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_color);
  PT_INIT(&pt_anim);
//  PT_INIT(&pt_adc);
  PT_INIT(&pt_ball);
  PT_INIT(&pt_dma);
  
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  srand(1);

	// Set up prototthreads
	PT_setup();
    
	
	while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_anim(&pt_anim));
	  PT_SCHEDULE(protothread_dma(&pt_dma));
//      PT_SCHEDULE(protothread_adc(&pt_adc));
      PT_SCHEDULE(protothread_ball(&pt_ball));
	}
}
