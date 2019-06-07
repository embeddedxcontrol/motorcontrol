/* 
 *
 * 
 *
 */

#include <asf.h>

void InitPwmController(void);	//Top gate initialization
void InitPioController(void);	//Bottom gate initialization
void InitTimer(void);

void InitQuadratureDecoder(void); //Setup the timer channel to enable the quadrature decoder

//Toggle function for closed loop control
void TogglePwm(int index);

void pin_edge_handler(const uint32_t id, const uint32_t index);

uint32_t pin25Val = 0x0;
uint32_t pin26Val = 0x0;
uint32_t pin27Val = 0x0;

volatile int i = 0;
//    HIGH   LOW
//    |G4|  |G1| -> |Pin9|   |Pin3|
//    |G5|  |G2| -> |Pin8|   |Pin4|
//    |G6|  |G3| -> |Pin7|   |Pin5|

int sixstep[6][2] =
{
//   L, H
	{26,4}, 
	{25,4},  
	{25,5},  
	{28,5},  
	{28,6},
	{26,6}   
};

//void TC0_Handler()
//{
	//TogglePwm(&i);
	////PWM->PWM_CH_NUM[6].PWM_CDTYUPD = 2100;
	//uint32_t read_n_clear = TC0->TC_CHANNEL[0].TC_SR;
//}


void InitPwmController()
{
	//////////////////
	//PIO SETUP//////
	/////////////////
	//Enabling PIOC peripheral clock
	PMC->PMC_PCER0 = (1<<ID_PIOC);
	
	//Disable the general purpose PIO for PWM pins
	PIOC->PIO_PDR = PIO_PC23 | PIO_PC22 | PIO_PC21;
		
	//Select peripheral B from AB select register
	PIOC->PIO_ABSR = PIO_PC23 | PIO_PC22 | PIO_PC21;
	
	//Enable output on PIO
	PIOC->PIO_OER = PIO_PC23 | PIO_PC22 | PIO_PC21;
	
	//Disable pull-up resistor on pin
	PIOC->PIO_PUDR = PIO_PC23 | PIO_PC22 | PIO_PC21;
	
	/////////////////////////////////////////////
	//PWM SETUP FOR DIGITAL PINS 7, 8, 9/////////
	/////////////////////////////////////////////
	//Enable the PWM clock
	PMC->PMC_PCER1 = (1<<(ID_PWM - 32));
	
	//Pass through clock without dividing time
	PWM->PWM_CLK = 0;
	
	//Channel Pre-scaler
	PWM->PWM_CH_NUM[6].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_4;
	PWM->PWM_CH_NUM[5].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_4;
	PWM->PWM_CH_NUM[4].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_4;

	//Set period register
	PWM->PWM_CH_NUM[6].PWM_CPRD = 2100;
	PWM->PWM_CH_NUM[5].PWM_CPRD = 2100;
	PWM->PWM_CH_NUM[4].PWM_CPRD = 2100;
	
	
	//Set duty cycle register
	PWM->PWM_CH_NUM[6].PWM_CDTY = 1050;
	PWM->PWM_CH_NUM[5].PWM_CDTY = 1050;
	PWM->PWM_CH_NUM[4].PWM_CDTY = 1050;	
	
	//Enable PWM
	PWM->PWM_ENA = (1<<6) | (1<<5) | (1<<4);
}

void InitPioController()
{
	///////////////////////////////////////////
	//PIO SETUP FOR DIGITAL PINS 3, 4, 5///////
	///////////////////////////////////////////
	
	//Enable the general purpose PIO for PC23 (Digital Pins 3, Digital Pin 4, Digital Pin 5)
	PIOC->PIO_PER = PIO_PC28 | PIO_PC26 | PIO_PC25;
		
	//Enable output on PIO
	PIOC->PIO_OER = PIO_PC28 | PIO_PC26 | PIO_PC25;
		
	//Disable pull-up resistor on pin
	PIOC->PIO_PUDR = PIO_PC28 | PIO_PC26 | PIO_PC25;
		
	//Enable PIO D register and set as input digital pins for capturing motor encoder output signals - Lines 0,1,2 of PIO D bank set as gpio input pins - arduino due pins 25,26,27	
	PMC->PMC_PCER0 |= (1<<ID_PIOD);//enable clock for piod
	pio_set_input(PIOD, PIO_PD0 | PIO_PD1 | PIO_PD2, PIO_PULLUP); //set pins as input
	pio_handler_set(PIOD, ID_PIOD, PIO_PD0 | PIO_PD1 | PIO_PD2, PIO_IT_EDGE, pin_edge_handler);//set up interrupt handler to hit on any edge interrupt
	pio_enable_interrupt(PIOD, PIO_PD0 | PIO_PD1 | PIO_PD2);//enable interrupt generation for the pins
	PIOD->PIO_IFER = 0x7;//enable glitch filter for the three pioD pins.
	NVIC_EnableIRQ(PIOD_IRQn);//setup NVIC to enable PIOD interrupts 
	
	//Setup Timer 0 for PHA signal edge counting
	//Disable the general purpose PIO for PWM pins
	PIOB->PIO_PDR = PIO_PB25;
		
	//Select peripheral B from AB select register
	PIOB->PIO_ABSR = PIO_PB25;
		
	//Enable output on PIO
	PIOB->PIO_ODR = PIO_PB25;
		
	//Disable pull-up resistor on pin
	PIOB->PIO_PUDR = PIO_PB25;
}


void pin_edge_handler(const uint32_t id, const uint32_t index)
{
    
    uint32_t read_n_clear = PIOD->PIO_ISR;//this reads the interrupt status register and resets it
    	
	//Check to make sure the PIOD bank caused the pin level change interrupt (i.e an interrupt was caused by level edge change - falling or rising edge)
	if(id == ID_PIOD)
	{
		//let's now read the PIOD bank pin level from PDSR register - the pin data status register
		uint32_t pinlevels = PIOD->PIO_PDSR;
		
		//Break down the above pinlevels variable value to the three pins of interest - i.e. the first three pins 0,1,2 - digital pins 25,26,27.
		pin25Val = (pinlevels & 0x1);
		pin26Val = (pinlevels & 0x2);
		pin27Val = (pinlevels & 0x4);
		
		//As a test, toggle the PC28 pin line - i.e. Digital Pin 3
		if(pin25Val && pin26Val && !pin27Val)
		{
			TogglePwm(5);
		}
		else if(pin26Val && !pin25Val && !pin27Val)
		{
			TogglePwm(0);
		}
		else if(pin26Val && pin27Val && !pin25Val)
		{
			TogglePwm(1);
		}
		else if(pin27Val && !pin25Val && !pin26Val)
		{
			TogglePwm(2);
		}
		else if(pin27Val && pin25Val && !pin26Val)
		{
			TogglePwm(3);
		}
		else if(pin25Val && !pin26Val && !pin27Val)
		{
			TogglePwm(4);
		}
	}
}


//////////////////////////////////////////////////////////////////////////
//Setup the quadrature decoder for the timer channel for the BLY17 BLDC motor
//////////////////////////////////////////////////////////////////////////
void InitQuadratureDecoder()
{
	//Enable the quadrature decoder
	//TC0->TC_BMR = 1010000000001101100000010000
	TC0->TC_BMR = 0xA00D810;
	//TC0->TC_CMR = 10100000110;
	TC0->TC_CHANNEL[0].TC_CMR = 0x506;

}

void InitTimer()
{
	//Enable clock for Timer 0 
	PMC->PMC_PCER0 |= (1<<ID_TC0);
	
	//Setup Channel Mode Register for TC0
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE |				    //Wave Mode
								TC_CMR_TCCLKS_TIMER_CLOCK2 |	//MCK / 8
								TC_CMR_WAVSEL_UP |				//No automatic trigger on RC compare
								TC_CMR_ACPC_TOGGLE |			//RC Compare Effect on
								TC_CMR_CPCTRG;					//Compare RC Trigger (resets value)
	
	TC0->TC_CHANNEL[0].TC_RC = 1050000;
	
	//PWM Interrupt Enable Register for RC Compare (so we interrupt on RC Compare)
	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	
	//Enable interrupts for TC0 in NVIC
	NVIC_EnableIRQ(TC0_IRQn);
	
	//Enable TC0 channel
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;					
}


void TogglePwm(int index)
{
	//Disable top gates
	PWM->PWM_DIS = (1<<sixstep[index][1]);

	//Disable bottom gate
	PIOC->PIO_CODR = (1<<sixstep[index][0]);

	//Advance index
	if(index == 5)
	{
		index = 0;
	}
	else 
	{
		index = index + 1;
	}

	//Enable next bottom gate
	PIOC->PIO_SODR = (1<<sixstep[index][0]);

	//Enable next bottom gate
	PWM->PWM_ENA = (1<<sixstep[index][1]);
}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	SystemInit();
	InitPwmController();
	InitPioController();
	InitTimer();
	
	
	/* Insert application code here, after the board has been initialized. */
	while(1)
	{
		//TEST THIS LATER
		//Read the Timer 0 Channel 0 Counter Value every 100 ms
		uint32_t value = TC0->TC_CHANNEL[0].TC_CV;	
		
	    delay_ms(100);
		//TEST THIS LATER
	}
}