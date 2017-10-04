#include "config.h"
#include <stdlib.h>
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
static struct pt pt_timer, pt_color, pt_anim ;

// system 1 second interval tick
int sys_time_seconds ;

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


//static PT_THREAD (protothread_dma(struct pt *pt))
//{
//    PT_BEGIN(pt);
//    while(1){
//	    // enable dma channel to start sending wave to DAC
//		DmaChnEnable(dmaChn);
//		// wait 1 second and then disable
//		PT_YIELD_TIME_msec(1000); 
//		DmaChnDisable(dmaChn);
//	}
//	PT_END(pt);
//}

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     tft_setCursor(0, 0);
     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
     tft_writeString("Time in seconds since boot\n");
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
        
        // draw sys_time
        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", sys_time_seconds);
        tft_writeString(buffer);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Animation Thread =============================================
// update a 1 second tick counter
//static fix16 xc1=int2fix16(10), yc1=int2fix16(150), vxc1=int2fix16(10), vyc1=0;
//static fix16 xc2=int2fix16(200), yc2=int2fix16(150), vxc2=int2fix16(-10), vyc2=0;
static fix16 xc[2]={int2fix16(25),int2fix16(75)}; 
static fix16 yc[2]={int2fix16(150),int2fix16(150)};
static fix16 vxc[2]={int2fix16(3),int2fix16(-3)};
static fix16 vyc[2]={int2fix16(3),int2fix16(3)};
static fix16 g = float2fix16(0.1), drag = float2fix16(0.00001);

static fix16 r12x, r12y, v12x, v12y, r12_dot_v12, dvx, dvy, mag_r12sq;

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);
    
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(32);
        int i;
        int hit_count=0;
        for(i=0;i<2;i++){
            // erase disk
            
                    
            tft_fillCircle(fix2int16(xc[i]), fix2int16(yc[i]), 4, ILI9340_BLACK); //x, y, radius, color
            
            // update vel
            vxc[i] = vxc[i] - multfix16(vxc[i], drag);
            
            // update pos
            xc[i] = xc[i] + vxc[i];
            yc[i] = yc[i] + vyc[i];
            
            // walls
            if (xc[i]<int2fix16(5) || xc[i]>int2fix16(235)) vxc[i] = -vxc[i]; 
            if (yc[i]<int2fix16(5) || yc[i]>int2fix16(315)) vyc[i] = -vyc[i]; 
            
                
        //NEST FOR LOOP HERE    
            if (absfix16(xc[0]-xc[1])<= int2fix16(4) && absfix16(yc[0]-yc[1])<= int2fix16(4)){                
                tft_fillRoundRect(0,20, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
                tft_setCursor(0, 20);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                sprintf(strng,"enters loop");
                tft_writeString(strng);
                
                r12x = xc[0]-xc[1];
                r12y = yc[0]-yc[1];
                mag_r12sq = multfix16(r12x, r12x)+ multfix16(r12y, r12y);
//                if (mag_r12sq <= int2fix16(4)){
//                    fix16 ix = vxc[0];
//                    fix16 iy = vyc[0];
//                    vxc[0] = vxc[1];
//                    vxc[1] = ix;
//                    vyc[0] = vyc[1];
//                    vyc[1] = iy;
//                }
                if(mag_r12sq < int2fix16(64) && hit_count==0){
                    v12x = vxc[0]-vxc[1];
                    v12y = vyc[0]-vyc[1];
                    r12_dot_v12 = multfix16(r12x, v12x)+ multfix16(v12y, v12y);
                    dvx = divfix16(multfix16(-r12x, r12_dot_v12), mag_r12sq); //bit shift by 4 instead possibly?
                    dvy = divfix16(multfix16(-r12y, r12_dot_v12), mag_r12sq);
                    
                    vxc[0] = vxc[0]+dvx;
                    vxc[1] = vxc[1]-dvx;
                    hit_count = 10;
                }
                else if (hit_count>0){
                    hit_count--;
                }
            }
            
            //  draw disk
            if(i==0){
                tft_fillCircle(fix2int16(xc[i]), fix2int16(yc[i]), 4, ILI9340_GREEN); //x, y, radius, color
            }
            else{
                tft_fillCircle(fix2int16(xc[i]), fix2int16(yc[i]), 4, ILI9340_RED); //x, y, radius, color
            }
            
            
        }
        
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

void main(void) {
	SYSTEMConfigPerformance(PBCLK);

	ANSELA = 0; ANSELB = 0; CM1CON = 0; CM2CON = 0;

	// Create sin table, including DAC channel in entry for DMA
//	static unsigned int sin_table[sine_table_size];
//	int s;
//    int i;
//	for (i = 0; i < sine_table_size; i++){
//		s = ((2047 * sin((float)i*6.283/(float)sine_table_size))) + 2048;
//		sin_table[i] = DAC_config_chan_B | (short)s;
//	}

	// Set up timer3 on,  interrupts, internal clock, prescalar 1, toggle rate
	// 5000 is 8 ksamples/sec at 40 MHz clock

//	OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 5000);
//	// set up the timer interrupt with a priority of 2
//	// ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
//	mT3ClearIntFlag(); // and clear the interrupt flag
//
//
//	// setup SPI
//	// SCK2 is pin 26 
//	// SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
//	PPSOutput(2, RPB5, SDO2);
//
//
//	// setup spi channel
//	// clk divider set to 2 for 20 MHz
//	SpiChnOpen(SPI_CHANNEL2, 
//	        SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV | SPICON_FRMEN | SPICON_FRMPOL,
//	        2);
//	// SS1 to RPB4 for FRAMED SPI
//	PPSOutput(4, RPB10, SS2);
//
//	// Open the desired DMA channel.
//	// We enable the AUTO option, we'll keep repeating the sam transfer over and over.
//	DmaChnOpen(dmaChn, 0, DMA_OPEN_AUTO);
//
//	// set the transfer parameters: source & destination address, source & destination size, number of bytes per event
//	// Setting the last parameter to one makes the DMA output one byte/interrupt
//	DmaChnSetTxfer(dmaChn, sin_table, (void*)&SPI2BUF, sine_table_size*4, 4, 4);
//
//	// set the transfer event control: what event is to start the DMA transfer
//	// In this case, timer3
//	DmaChnSetEventControl(dmaChn, DMA_EV_START_IRQ(_TIMER_3_IRQ));

     // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_color);
  PT_INIT(&pt_anim);

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
	//PT_INIT(&pt_dma);
	while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_anim(&pt_anim));
		//PT_SCHEDULE(protothread_dma(&pt_dma));
	}
}
