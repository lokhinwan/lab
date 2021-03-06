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


// PI defined here for use in your code 
#define PI 3.141592653589793

// DEFINE SINE_TABLE_SIZE
#define SINE_TABLE_SIZE 256

//define a 12 element delay filter
#define N 328
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


//FIR filter coef
double b[] = { -9.2607627986484458e-05,    2.8054697371736136e-04,    6.2337076411693600e-04,    9.3637166738587624e-04,    9.6139798874727450e-04,    5.7174442787877734e-04,   -1.0950024218979503e-04,   -7.3781208265950241e-04,   -9.4784546784412727e-04,   -6.0617457062478041e-04,    6.9272681852199563e-05,    6.3381011018659819e-04,    7.1506597819814941e-04,    2.7614466315278088e-04,   -3.5870893313119190e-04,   -7.4647027053111287e-04,   -6.5865703537273403e-04,   -2.4270181087930823e-04,    1.1495062283196203e-04,    1.1845916558900110e-04,   -1.8362365865911008e-04,   -4.3873208012712520e-04,   -2.9324626821878035e-04,    2.8566402711993643e-04,    9.5122812319208352e-04,    1.2414836434335172e-03,    9.5813308696171605e-04,    3.3780332124250804e-04,   -1.3932829846656608e-04,   -1.5543431852337698e-04,    1.6862946045894468e-04,    3.7254001373284774e-04,    6.2879231908431429e-05,   -7.1526030833675555e-04,   -1.4706242866027149e-03,   -1.6527287851697667e-03,   -1.1163205994157576e-03,   -2.6391321280830256e-04,    2.7732470995725251e-04,    1.8572032464916359e-04,   -2.7096595484217296e-04,   -4.5909023937577983e-04,    5.9209382426111945e-05,    1.1042405705781583e-03,    1.9775452706149829e-03,    2.0264868726922831e-03,    1.2057260555816062e-03,    1.4377471743033448e-04,   -3.7098270624732842e-04,   -6.4550878215339016e-05,    5.7142262246744025e-04,    6.9395411812609429e-04,   -1.4147031366293757e-04,   -1.5391492417359519e-03,   -2.5379675703354329e-03,   -2.4015763223436768e-03,   -1.2599140573832971e-03,   -3.0800638129844203e-05,    3.4604837480397524e-04,   -2.7470967875609841e-04,   -1.0924944955736078e-03,   -1.0409150185909772e-03,    2.5715887376112341e-04,    2.0878308274903086e-03,    3.1830601237029864e-03,    2.7755476892175400e-03,    1.2738880819966162e-03,   -5.3007225740377064e-05,   -1.5348649373476622e-04,    8.7710121233643937e-04,    1.8395569933381409e-03,    1.4573884535753110e-03,   -4.6837950988671282e-04,   -2.7884921633339358e-03,   -3.9046450830627416e-03,   -3.1109163586624899e-03,   -1.2233049736378252e-03,    8.7737498583126015e-05,   -2.6315620817877442e-04,   -1.7944614715803000e-03,   -2.8200784028272453e-03,   -1.9030751702413908e-03,    8.2601992910322371e-04,    3.6557275214372967e-03,    4.6654128628663425e-03,    3.3492059601176794e-03,    1.0814613988352842e-03,   -3.7672444921633269e-05,    9.8347362364853506e-04,    3.0964739270098037e-03,    4.0491218228337865e-03,    2.3403099721592178e-03,   -1.3731686025702985e-03,   -4.6860623253079476e-03,   -5.4049141189215794e-03,   -3.4172020423273611e-03,   -8.2745104761433510e-04,   -1.6120514194503359e-04,   -2.1271677098171037e-03,   -4.8884795061139175e-03,   -5.5619796596913847e-03,   -2.7375120011514692e-03,    2.1504474546048136e-03,    5.8676496266323840e-03,    6.0498178086595081e-03,    3.2373831897870497e-03,    4.6033594081454815e-04,    6.2205653505044813e-04,    3.8796347264273873e-03,    7.3332495257062945e-03,    7.4219056983785541e-03,    3.0590565424873091e-03,   -3.2158386240831458e-03,   -7.1935133139948027e-03,   -6.5087674700269084e-03,   -2.7094531155895836e-03,    1.0569941282067956e-05,   -1.5300077868801787e-03,   -6.5519748926856063e-03,   -1.0725581223138839e-02,   -9.7735188078980680e-03,   -3.2763536405284403e-03,    4.6638662459683698e-03,    8.6753487515183088e-03,    6.6551251962086487e-03,    1.6777597410497238e-03,   -5.5847873849957622e-04,    3.2428971100458728e-03,    1.0774911791690510e-02,    1.5707075516297721e-02,    1.2975385315800288e-02,    3.3628974778828575e-03,   -6.7227109277475904e-03,   -1.0413914144569280e-02,   -6.2812601585947418e-03,    1.8766015552820390e-04,    1.1505774967162491e-03,   -6.5801013151701154e-03,   -1.8113838354078170e-02,   -2.3984669624559127e-02,   -1.8070631351923536e-02,   -3.3062874299442466e-03,    1.0059615652337340e-02,    1.2795345771773162e-02,    4.8481599477362865e-03,   -3.9070541757795464e-03,   -1.7411077677811813e-03,    1.4268638191820809e-02,    3.4173298460928085e-02,    4.2082787051654538e-02,    2.9335610579047034e-02,    3.0993752486122260e-03,   -1.7788372842490650e-02,   -1.7905911218350695e-02,    2.9613879197320538e-04,    1.5362687485183784e-02,    2.2886484282692498e-03,   -4.5606132814961307e-02,   -1.0541287703609159e-01,   -1.3499812715442053e-01,   -1.0050992691877435e-01,   -2.7525886786966547e-03,    1.1710863041118000e-01,    1.9882533570069880e-01,    1.9882533570069880e-01,    1.1710863041118000e-01,   -2.7525886786966547e-03,   -1.0050992691877435e-01,   -1.3499812715442053e-01,   -1.0541287703609159e-01,   -4.5606132814961307e-02,    2.2886484282692498e-03,    1.5362687485183784e-02,    2.9613879197320538e-04,   -1.7905911218350695e-02,   -1.7788372842490650e-02,    3.0993752486122260e-03,    2.9335610579047034e-02,    4.2082787051654538e-02,    3.4173298460928085e-02,    1.4268638191820809e-02,   -1.7411077677811813e-03,   -3.9070541757795464e-03,    4.8481599477362865e-03,    1.2795345771773162e-02,    1.0059615652337340e-02,   -3.3062874299442466e-03,   -1.8070631351923536e-02,   -2.3984669624559127e-02,   -1.8113838354078170e-02,   -6.5801013151701154e-03,    1.1505774967162491e-03,    1.8766015552820390e-04,   -6.2812601585947418e-03,   -1.0413914144569280e-02,   -6.7227109277475904e-03,    3.3628974778828575e-03,    1.2975385315800288e-02,    1.5707075516297721e-02,    1.0774911791690510e-02,    3.2428971100458728e-03,   -5.5847873849957622e-04,    1.6777597410497238e-03,    6.6551251962086487e-03,    8.6753487515183088e-03,    4.6638662459683698e-03,   -3.2763536405284403e-03,   -9.7735188078980680e-03,   -1.0725581223138839e-02,   -6.5519748926856063e-03,   -1.5300077868801787e-03,    1.0569941282067956e-05,   -2.7094531155895836e-03,   -6.5087674700269084e-03,   -7.1935133139948027e-03,   -3.2158386240831458e-03,    3.0590565424873091e-03,    7.4219056983785541e-03,    7.3332495257062945e-03,    3.8796347264273873e-03,    6.2205653505044813e-04,    4.6033594081454815e-04,    3.2373831897870497e-03,    6.0498178086595081e-03,    5.8676496266323840e-03,    2.1504474546048136e-03,   -2.7375120011514692e-03,   -5.5619796596913847e-03,   -4.8884795061139175e-03,   -2.1271677098171037e-03,   -1.6120514194503359e-04,   -8.2745104761433510e-04,   -3.4172020423273611e-03,   -5.4049141189215794e-03,   -4.6860623253079476e-03,   -1.3731686025702985e-03,    2.3403099721592178e-03,    4.0491218228337865e-03,    3.0964739270098037e-03,    9.8347362364853506e-04,   -3.7672444921633269e-05,    1.0814613988352842e-03,    3.3492059601176794e-03,    4.6654128628663425e-03,    3.6557275214372967e-03,    8.2601992910322371e-04,   -1.9030751702413908e-03,   -2.8200784028272453e-03,   -1.7944614715803000e-03,   -2.6315620817877442e-04,    8.7737498583126015e-05,   -1.2233049736378252e-03,   -3.1109163586624899e-03,   -3.9046450830627416e-03,   -2.7884921633339358e-03,   -4.6837950988671282e-04,    1.4573884535753110e-03,    1.8395569933381409e-03,    8.7710121233643937e-04,   -1.5348649373476622e-04,   -5.3007225740377064e-05,    1.2738880819966162e-03,    2.7755476892175400e-03,    3.1830601237029864e-03,    2.0878308274903086e-03,    2.5715887376112341e-04,   -1.0409150185909772e-03,   -1.0924944955736078e-03,   -2.7470967875609841e-04,    3.4604837480397524e-04,   -3.0800638129844203e-05,   -1.2599140573832971e-03,   -2.4015763223436768e-03,   -2.5379675703354329e-03,   -1.5391492417359519e-03,   -1.4147031366293757e-04,    6.9395411812609429e-04,    5.7142262246744025e-04,   -6.4550878215339016e-05,   -3.7098270624732842e-04,    1.4377471743033448e-04,    1.2057260555816062e-03,    2.0264868726922831e-03,    1.9775452706149829e-03,    1.1042405705781583e-03,    5.9209382426111945e-05,   -4.5909023937577983e-04,   -2.7096595484217296e-04,    1.8572032464916359e-04,    2.7732470995725251e-04,   -2.6391321280830256e-04,   -1.1163205994157576e-03,   -1.6527287851697667e-03,   -1.4706242866027149e-03,   -7.1526030833675555e-04,    6.2879231908431429e-05,    3.7254001373284774e-04,    1.6862946045894468e-04,   -1.5543431852337698e-04,   -1.3932829846656608e-04,    3.3780332124250804e-04,    9.5813308696171605e-04,    1.2414836434335172e-03,    9.5122812319208352e-04,    2.8566402711993643e-04,   -2.9324626821878035e-04,   -4.3873208012712520e-04,   -1.8362365865911008e-04,    1.1845916558900110e-04,    1.1495062283196203e-04,   -2.4270181087930823e-04,   -6.5865703537273403e-04,   -7.4647027053111287e-04,   -3.5870893313119190e-04,    2.7614466315278088e-04,    7.1506597819814941e-04,    6.3381011018659819e-04,    6.9272681852199563e-05,   -6.0617457062478041e-04,   -9.4784546784412727e-04,   -7.3781208265950241e-04,   -1.0950024218979503e-04,    5.7174442787877734e-04,    9.6139798874727450e-04,    9.3637166738587624e-04,    6.2337076411693600e-04,    2.8054697371736136e-04,   -9.2607627986484458e-05};


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
 
void shiftBuffer(short newX){
	int i;
	for (i= N-1 ; i>0; i--){
		x[i]=x[i-1];
	}
	x[0] = newX;
}

void inline wrapAroundBuffer(short newX){
	

	x[bufferIndex] = newX;
	bufferIndex= (bufferIndex + 1)%N;
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
	
	return(y);
}


  
