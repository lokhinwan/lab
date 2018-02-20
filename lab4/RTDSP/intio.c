/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		  LAB 3: Interrupt I/O

 				            ********* I N T I O. C **********

  Demonstrates inputing and outputing data from the DSK's audio port using interrupts. 

 *************************************************************************************
 				Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
				Updated for CCS V4 Sept 10
 ************************************************************************************/
/*
 *	You should modify the code so that interrupts are used to service the 
 *  audio port.
 */
/**************************** Pre-processor statements ******************************/

#include <stdlib.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

//FIR coef
//#include "filename.txt"

//define a 12 element delay filter
#define N 250
short x[N];

/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio 
   interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
			 /**********************************************************************/
			 /*   REGISTER	            FUNCTION			      SETTINGS         */ 
			 /**********************************************************************/\
    0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
    0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
    0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
    0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
    0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
    0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
    0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
    0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control             8 KHZ                 */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};


// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;


/* Sampling frequency in HZ. Must only be set to 8000, 16000, 24000
32000, 44100 (CD standard), 48000 or 96000  */ 
float sampling_freq = 8000;

//INdex variable to keep track of the buffer position
int bufferIndex =0;


//FIR filter coefficient
double b[] = {   -1.8442833427486215e-03,   -4.1190441269031482e-03,   -2.0535873301344900e-03,   -1.3703337256430993e-03,    1.5790804473041996e-03,    3.2681033644419424e-03,    3.8489975928330590e-03,    2.5928873391771741e-03,    7.3935746553212343e-04,   -6.2977102622475696e-04,   -7.4022462422824781e-04,    1.0838837561731961e-05,    6.2201321650751323e-04,    2.6185292433271890e-04,   -9.8730303173071824e-04,   -2.2339066002165777e-03,   -2.5294527602002987e-03,   -1.6842480784886296e-03,   -4.2016236615805653e-04,    2.5804777578250976e-04,   -5.8087123504999503e-05,   -8.2110499392385199e-04,   -1.0138712540317635e-03,   -7.2462256245359687e-05,    1.5627338477842627e-03,    2.7731479416253199e-03,    2.7080617757320475e-03,    1.5299945897527159e-03,    2.9075333515620941e-04,    1.5404209169847384e-05,    7.8386710505807481e-04,    1.6284321951581570e-03,    1.3797792157293361e-03,   -2.5618521765966815e-04,   -2.3500948838336417e-03,   -3.4788598846532308e-03,   -2.9380959028741413e-03,   -1.3703061861778118e-03,   -2.2792940558228288e-04,   -4.9434851786305423e-04,   -1.7755467135105361e-03,   -2.5945983821700925e-03,   -1.6902711450564462e-03,    7.9565808913531586e-04,    3.3223856450177798e-03,    4.1997688463235074e-03,    3.0395347122889166e-03,    1.1215680893159529e-03,    2.9752002191501240e-04,    1.3341691375657206e-03,    3.1602890161298654e-03,    3.7417961603346418e-03,    1.8849793136964889e-03,   -1.6009389969865399e-03,   -4.4527074585415391e-03,   -4.8293802358949922e-03,   -2.9132207353678133e-03,   -7.8850406841229101e-04,   -6.2422045457231391e-04,   -2.7015401776614998e-03,   -5.0424933113524615e-03,   -5.0614770815655000e-03,   -1.8844015659051040e-03,    2.7175907964107565e-03,    5.6873054221566891e-03,    5.2510734704918210e-03,    2.4858008311992343e-03,    4.2982376587346661e-04,    1.3983170023313476e-03,    4.8185763803073470e-03,    7.5484357920380368e-03,    6.5370280269128854e-03,    1.5974940089143791e-03,   -4.1920011112126692e-03,   -6.9553964006178132e-03,   -5.3222239632348270e-03,   -1.6833671816724866e-03,   -1.6143678337881398e-04,   -2.9210596866010114e-03,   -8.0344442825560289e-03,   -1.0900480481149437e-02,   -8.1827848213134342e-03,   -9.0883142596030330e-04,    6.0966318821970462e-03,    8.1701258372188154e-03,    4.8496443049298159e-03,    4.1097124292751501e-04,    1.9261748447799249e-04,    5.7400707531912228e-03,    1.3031422928139412e-02,    1.5600386573684497e-02,    1.0120491742875957e-02,   -3.6398549700533179e-04,   -8.6154694364738577e-03,   -9.2393938920502447e-03,   -3.4913914122513493e-03,    1.5371957503511142e-03,   -9.5799325703711661e-04,   -1.1126447229789867e-02,   -2.1547712424146097e-02,   -2.3081317200719258e-02,   -1.2831352513823386e-02,    2.6694425875886112e-03,    1.2374349063449303e-02,    1.0080975202653719e-02,    3.2000937833540327e-04,   -4.8584066984498743e-03,    3.7319205875693418e-03,    2.3366682886717321e-02,    3.9990474677983653e-02,    3.9088852968988910e-02,    1.8511365301182438e-02,   -7.9253063604147259e-03,   -2.0525827170037838e-02,   -1.0613484503543108e-02,    9.6001638619096132e-03,    1.4005289209935474e-02,   -1.6623772517081796e-02,   -7.5176947268951036e-02,   -1.2649037212917455e-01,   -1.2785005010011607e-01,   -5.9386966030034079e-02,    5.7144488726887425e-02,    1.6633881280165075e-01,    2.1079847965781059e-01,    1.6633881280165075e-01,    5.7144488726887425e-02,   -5.9386966030034079e-02,   -1.2785005010011607e-01,   -1.2649037212917455e-01,   -7.5176947268951036e-02,   -1.6623772517081796e-02,    1.4005289209935474e-02,    9.6001638619096132e-03,   -1.0613484503543108e-02,   -2.0525827170037838e-02,   -7.9253063604147259e-03,    1.8511365301182438e-02,    3.9088852968988910e-02,    3.9990474677983653e-02,    2.3366682886717321e-02,    3.7319205875693418e-03,   -4.8584066984498743e-03,    3.2000937833540327e-04,    1.0080975202653719e-02,    1.2374349063449303e-02,    2.6694425875886112e-03,   -1.2831352513823386e-02,   -2.3081317200719258e-02,   -2.1547712424146097e-02,   -1.1126447229789867e-02,   -9.5799325703711661e-04,    1.5371957503511142e-03,   -3.4913914122513493e-03,   -9.2393938920502447e-03,   -8.6154694364738577e-03,   -3.6398549700533179e-04,    1.0120491742875957e-02,    1.5600386573684497e-02,    1.3031422928139412e-02,    5.7400707531912228e-03,    1.9261748447799249e-04,    4.1097124292751501e-04,    4.8496443049298159e-03,    8.1701258372188154e-03,    6.0966318821970462e-03,   -9.0883142596030330e-04,   -8.1827848213134342e-03,   -1.0900480481149437e-02,   -8.0344442825560289e-03,   -2.9210596866010114e-03,   -1.6143678337881398e-04,   -1.6833671816724866e-03,   -5.3222239632348270e-03,   -6.9553964006178132e-03,   -4.1920011112126692e-03,    1.5974940089143791e-03,    6.5370280269128854e-03,    7.5484357920380368e-03,    4.8185763803073470e-03,    1.3983170023313476e-03,    4.2982376587346661e-04,    2.4858008311992343e-03,    5.2510734704918210e-03,    5.6873054221566891e-03,    2.7175907964107565e-03,   -1.8844015659051040e-03,   -5.0614770815655000e-03,   -5.0424933113524615e-03,   -2.7015401776614998e-03,   -6.2422045457231391e-04,   -7.8850406841229101e-04,   -2.9132207353678133e-03,   -4.8293802358949922e-03,   -4.4527074585415391e-03,   -1.6009389969865399e-03,    1.8849793136964889e-03,    3.7417961603346418e-03,    3.1602890161298654e-03,    1.3341691375657206e-03,    2.9752002191501240e-04,    1.1215680893159529e-03,    3.0395347122889166e-03,    4.1997688463235074e-03,    3.3223856450177798e-03,    7.9565808913531586e-04,   -1.6902711450564462e-03,   -2.5945983821700925e-03,   -1.7755467135105361e-03,   -4.9434851786305423e-04,   -2.2792940558228288e-04,   -1.3703061861778118e-03,   -2.9380959028741413e-03,   -3.4788598846532308e-03,   -2.3500948838336417e-03,   -2.5618521765966815e-04,    1.3797792157293361e-03,    1.6284321951581570e-03,    7.8386710505807481e-04,    1.5404209169847384e-05,    2.9075333515620941e-04,    1.5299945897527159e-03,    2.7080617757320475e-03,    2.7731479416253199e-03,    1.5627338477842627e-03,   -7.2462256245359687e-05,   -1.0138712540317635e-03,   -8.2110499392385199e-04,   -5.8087123504999503e-05,    2.5804777578250976e-04,   -4.2016236615805653e-04,   -1.6842480784886296e-03,   -2.5294527602002987e-03,   -2.2339066002165777e-03,   -9.8730303173071824e-04,    2.6185292433271890e-04,    6.2201321650751323e-04,    1.0838837561731961e-05,   -7.4022462422824781e-04,   -6.2977102622475696e-04,    7.3935746553212343e-04,    2.5928873391771741e-03,    3.8489975928330590e-03,    3.2681033644419424e-03,    1.5790804473041996e-03,   -1.3703337256430993e-03,   -2.0535873301344900e-03,   -4.1190441269031482e-03,   -1.8442833427486215e-03};

 /******************************* Function prototypes ********************************/
void init_hardware(void);     
void init_HWI(void);

void rect(void);
float convolution(void);
void shiftBuffer(short newX);  
void wrapAroundBuffer(short newX);
short accessWrapAroundBuffer(int p);              
/********************************** Main routine ************************************/
void main(){      

	// initialize board and the audio port
  init_hardware();
	
  /* initialize hardware interrupts */
  init_HWI();
  	 		
  /* loop indefinitely, waiting for interrupts */  					
  while(1) 
  {};
  
}
        
/********************************** init_hardware() **********************************/  
void init_hardware()
{
    // Initialize the board support library, must be called first 
    DSK6713_init();
    
    // Start the AIC23 codec using the settings defined above in config 
    H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Function below sets the number of bits in word used by MSBSP (serial port) for 
	receives from AIC23 (audio port). We are using a 32 bit packet containing two 
	16 bit numbers hence 32BIT is set for  receive */
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);	

	/* Configures interrupt to activate on each consecutive available 32 bits 
	from Audio port hence an interrupt is generated for each L & R sample pair */	
	MCBSP_FSETS(SPCR1, RINTM, FRM);

	/* These commands do the same thing as above but applied to data transfers to  
	the audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);	
	MCBSP_FSETS(SPCR1, XINTM, FRM);	
	

}

/********************************** init_HWI() **************************************/  
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	IRQ_map(IRQ_EVT_RINT1,4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

} 

/******************** WRITE YOUR INTERRUPT SERVICE ROUTINE HERE***********************/  
void rect(){
	float output;
	
	wrapAroundBuffer(mono_read_16Bit());
	output = convolution();
	mono_write_16Bit((short)output);
}
 
void shiftBuffer(short newX){		//the lazy method, shift buffer
	int i;
	for (i= N-1 ; i>0; i--){
		x[i]=x[i-1];
	}
	x[0] = newX;
}

void inline wrapAroundBuffer(short newX){
	
	x[bufferIndex] = newX;
	//bufferIndex=(bufferIndex+1)%N;  2875 cycle
	bufferIndex++;	//2857
	if (bufferIndex >=N){
		bufferIndex-=N;
	}
}
short inline accessWrapAroundBuffer(int p){
	
	int realIndex = (bufferIndex - p);
	if (realIndex<0){
		realIndex+=N;	
	}
	return x[realIndex];
}	
	

float convolution(){
	float y=0;
	int i;
	for (i =0; i<N; i++){
		y += b[i] * accessWrapAroundBuffer(i);
	}
	
	return y;
}


  
