/*
 * Tiny25_MPPT.c
 *
 * Created: 13.01.2016 13:11:27
 *  Author: Sebastian Foerster
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
#define DEFINE_WAKEUP_VALUE		200		//10 bit max ( 100 Ohm / ( 2k2 Ohm + 100 Ohm) )
#define DEFINE_4V2_VALUE		680		//10 bit max ( 47k Ohm / ( 47k Ohm + 220k Ohm) )

#define MIN_POWER_TO_SLEEP		2000	//20 bit max
#define CNT_UNDERVOLTAGE_REF	60000	//16 bit max

//can't be changed at this tiny device
#define DEFINE_MAX_TIMER_PER	255		//8 bit max

// use also internal ref
#define ADCMUX_CELL_VOLTAGE		(0x01 | (1<<REFS1))
#define ADCMUX_CELL_CURRENT		(0x02 | (1<<REFS1)) // 0.27 Ohm no devider
#define ADCMUX_ACCU_VOLTAGE		(0x03 | (1<<REFS1))

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
			//ADCSRB = 0; //change conversation to free running;
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
			//ADCSRB = (1<<ADTS2); //trigger adc read with timer in sync
		break;
	}
	
	//start adc conversation
	ADCSRA |= (1<<ADSC);

}

ISR(WDT_vect) {
	//use watchdog just to wake up (interrupt required!)
}


int main(void)
{
	
	//clock a 8 MHz source to about 10 MHz -> give us a ~ 312.5 kHz timer... results in a smaller inductor
	//could tune this without problem, because we don't try to write Flash or EEPROM
	OSCCAL = 0x7F; 

	// enable watchdog for sleep
	// prescale timer to 4s
	WDTCR |= (1<<WDP3);
	
	// Enable watchdog timer interrupts not for system reset
	WDTCR |= (1<<WDIE);
	
	// Use the power down sleep mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	//set outputs
	DDRB = 0b00000011;
		
	volatile uint8_t *pwm = &OCR1A;
	
	PLLCSR |= (1<<PLLE); //don't care about start timing of pll?
		
	_delay_ms(1);
		
	PLLCSR |= (1<<PCKE);

	ADMUX = ADCMUX_CELL_VOLTAGE;                 // select start channel (mux)
		
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2) | (1<<ADIE); // frequency divider 64, ADC activate with interrupt
	
	DIDR0 = (1<<ADC1D) | (1<<ADC2D) | (1<<ADC3D); //save power on adc input pin...
	
	ADCSRB = (1<<ADTS2); //trigger adc read with timer in sync
		
	//global interrupt enable
	sei();
	

	(*pwm) = 255;
	
	/*for(uint8_t i = 0;i < 100; i++) {
		_delay_ms(100);
	}*/
	
	
	//use timer 0 here just to trigger the adc read with pwm (timer 1) in sync...
	//because timer 1 with pll has no trigger to the adc
	//but timer 0 runs only 8 times slower... gets the same position
	//start Timer 0 with cpu frequency
	TCCR0B = 0b00000001;

	//Timer channel A as PWM, phase correct PWM start
	TCCR0A |= (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);
	
	//Timer with cpu * 8 frequency (pll)
	TCCR1 = 0b01110001;
	
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
        
		//if it in the range we can charge a battery?
        if(c_voltage < DEFINE_5V_VALUE || a_voltage > DEFINE_4V2_VALUE) {
			//no charging
	        if((*pwm) < DEFINE_MAX_TIMER_PER) 
				(*pwm)++;
	    } 
		else 
		{
	        if(power_act > power_old) {
		        if(c_voltage > c_voltage_old) {
			        if((*pwm) > 0)
						(*pwm)--;
		        }
		        else
		        {
			        if((*pwm) < DEFINE_MAX_TIMER_PER)
						(*pwm)++;
		        }
	        }
	        else
	        {
		        if(c_voltage <= c_voltage_old) 
				{
			        if((*pwm) > 0)
						(*pwm)--;
		        }
		        else
		        {
			        if((*pwm) < DEFINE_MAX_TIMER_PER)
						(*pwm)++;
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
				
				TCCR1 = 0; //disable timer = shutdown high side mosfet
				
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
				} while(ADCW < DEFINE_WAKEUP_VALUE);
				
				ADCSRB |= (1<<ADTS2); //trigger adc read with timer in sync
				
				//reset pending interrupt
				ADCSRA |= (1<<ADIF);
				
				PLLCSR |= (1<<PLLE); 
					
				_delay_ms(1);
					
				PLLCSR |= (1<<PCKE);
				
				//enable timer after sleep
				(*pwm) = 255;
				
				//sync timers
				TCNT0 = 0;
				TCNT1 = 0;
				
				//Timer channel A as PWM, phase correct PWM start
				TCCR0A |= (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);
				//Timer with cpu * 8 frequency (pll)
				TCCR1 = 0b01110001;
				
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