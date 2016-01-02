/*
 * Tiny_MPPT.c
 *
 * Created: 28.11.2015 13:11:27
 *  Author: Basti
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 8000000UL

#ifndef F_CPU
#warning "F_CPU is not defined, define 1000000 MHz"
#define F_CPU 1000000UL
#endif

#include <util/delay.h>

//precaluclatet compare values -> use ref voltage and voltage devider -> 1,1 Vref
#define DEFINE_5V_VALUE			202		//10 bit max ( 100 Ohm / ( 2k2 Ohm + 100 Ohm) )
#define DEFINE_4V2_VALUE		680		//10 bit max ( 47k Ohm / ( 47k Ohm + 220k Ohm) )

#define MIN_POWER_TO_SLEEP		2000	//20 bit max
#define CNT_UNDERVOLTAGE_REF	60000	//16 bit max

//can't be changed at this tiny13 device
#define DEFINE_MAX_TIMER_PER	255		//8 bit max


// use also internal ref
#define ADCMUX_CELL_VOLTAGE		(0x01 | (1<<REFS0))
#define ADCMUX_CELL_CURRENT		(0x02 | (1<<REFS0)) // 0.27 Ohm no devider
#define ADCMUX_ACCU_VOLTAGE		(0x03 | (1<<REFS0))

uint16_t accu_voltage = 0;
uint16_t cell_voltage = 0;
uint16_t cell_current = 0;

volatile uint8_t adc_ready = false;

// ADC ready interrupt
ISR(ADC_vect) 
{
	
	switch(ADMUX)
	{
		case ADCMUX_CELL_VOLTAGE:
			ADCSRB = 0; //change conversation to free running;
			cell_voltage = ADCW;
			ADMUX = ADCMUX_CELL_CURRENT;
		break;
		
		case ADCMUX_CELL_CURRENT:
			cell_current = ADCW;
			ADMUX = ADCMUX_ACCU_VOLTAGE;
		break;
		
		case ADCMUX_ACCU_VOLTAGE:
			accu_voltage = ADCW;
			ADMUX = ADCMUX_CELL_VOLTAGE;
			adc_ready = true;
			ADCSRB = (1<<ADTS2); //trigger adc read with timer in sync
		break;
	}
	
	//start adc conversation
	ADCSRA |= (1<<ADSC);

}

ISR(TIM0_OVF_vect) {
	//do we need this?
}

ISR(WDT_vect) {
	//use watchdog just to wake up (interrupt required!)
}


int main(void)
{
	// enable watchdog for sleep
	// prescale timer to 4s
	WDTCR |= (1<<WDP3);
	
	// Enable watchdog timer interrupts not for system reset
	WDTCR |= (1<<WDTIE);
	
	// Use the power down sleep mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	//set outputs
	DDRB = 0b00000011;
	
	//Timer with cpu frequency
	TCCR0B = 0b00000001;

	ADMUX = ADCMUX_CELL_VOLTAGE;                 // select start channel (mux)
		
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2) | (1<<ADIE); // frequency divider 64, ADC activate with interrupt
	
	DIDR0 = (1<<ADC1D) | (1<<ADC2D) | (1<<ADC3D); //save power on adc input pin...
	
	ADCSRB = (1<<ADTS2); //trigger adc read with timer in sync
		
	//global interrupt enable
	sei();


	OCR0A = 255;

	//Timer channel A as PWM
	TCCR0A |= (1<<COM0A0) | (1<<COM0A1);
	//pahse correct PWM start
	TCCR0A |= (1<<WGM00) | (1<<WGM01);
	//timer interrupt (BOTTOM) enable
	//TIMSK0 |= (1<<TOIE0);
	
	//start adc conversation
	ADCSRA |= (1<<ADSC);
	
	uint32_t power_old = 0;
	uint16_t c_voltage_old = 0;
	
	uint16_t cnt_undervoltage = 0;
		
    while(1)
    {
		
		//wait for new data
		while(!adc_ready);
		
		adc_ready = false;
		
        cli();
        uint16_t c_voltage =	cell_voltage;
        uint16_t c_current =	cell_current;
        uint16_t a_voltage =	accu_voltage;
        sei();
        

        uint32_t power_act = c_voltage * c_current;
        
        if(c_voltage < DEFINE_5V_VALUE || a_voltage > DEFINE_4V2_VALUE) {
	        if(OCR0A < DEFINE_MAX_TIMER_PER) 
				OCR0A++;
	    } 
		else 
		{
	        if(power_act > power_old) {
		        if(c_voltage > c_voltage_old) {
			        if(OCR0A > 0)
						OCR0A--;
		        }
		        else
		        {
			        if(OCR0A < DEFINE_MAX_TIMER_PER)
						OCR0A++;
		        }
	        }
	        else
	        {
		        if(c_voltage <= c_voltage_old) 
				{
			        if(OCR0A > 0)
						OCR0A--;
		        }
		        else
		        {
			        if(OCR0A < DEFINE_MAX_TIMER_PER)
						OCR0A++;
		        }
	        }
        }
        
        power_old = power_act;
        c_voltage_old = cell_voltage;

		//manage sleep mode if power is to less for a while (~ 6 sec)
		if(power_act < MIN_POWER_TO_SLEEP) {
			cnt_undervoltage++;
			if(cnt_undervoltage > CNT_UNDERVOLTAGE_REF) {
				cnt_undervoltage = 0;
				
				TCCR0A = 0; //disable timer = shutdown high side mosfet
				
				do {
					//disable interrupt
					ADCSRA &= ~(1<<ADIE);
										
					//reset pending interrupt
					ADCSRA |= (1<<ADIF);

					ADCSRB = 0; //change conversation to free running;
										
					ADMUX = ADCMUX_CELL_VOLTAGE; //setup mux to cell voltage
										
					//SLEEP!!
					sleep_mode();
		
					//start adc conversation
					ADCSRA |= (1<<ADSC);
					
					//wait for voltage reading
					while(!(ADCSRA & (1<<ADIF)));
			
				//wait until the open circuit voltage is greater then 5 V
				} while(ADCW < DEFINE_5V_VALUE);
				
				ADCSRB |= (1<<ADTS2); //trigger adc read with timer in sync
				
				//reset pending interrupt
				ADCSRA |= (1<<ADIF);
				
				//enable timer after sleep
				OCR0A = 255;
				
				//Timer channel A as PWM, phase correct PWM start
				TCCR0A |= (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);
				
				//enable interrupt
				ADCSRA |= (1<<ADIE);
				
				//start adc conversation
				ADCSRA |= (1<<ADSC);
					
			}
		} else {
			//restart if more power is detected
			cnt_undervoltage = 0;
		}
    }
}