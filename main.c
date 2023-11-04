/*
 * Exam May 2019.c
 *
 * Created: 5/9/2019 1:39:13 PM
 * Author : KennetGunnar
 */ 

#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "lcd.h" // Enable the LCD (lcd_init(), lcd_clear(), lcd_gotoxy())				(lcd.c)

//FUNCTION PROTOTYPES
void usart_init(void);
void usart_send(unsigned char data);
void init_interrupt();
void initialize();
void gear_state();

//VARIABLES
volatile int RPM = 1000;
unsigned char current_gear = 1;
uint8_t wear_counter;
unsigned int addr = 2;

int main(void)
{
	usart_init();		//Calls the USART initialization code
	lcd_init();			// Initializing the LCD.
    initialize();		// Initializing the LEDs and the Buttons
	init_interrupt();	// Initializing the interrupt
	
	wear_counter = eeprom_read_byte((uint8_t *)addr); // Reading the value for wear_counter on the EEPROM
	usart_send(wear_counter); // Initially send the value of wear_counter, to check how much wear has the car seen.
	
    while (1) 
    {
		gear_state(); // Function for changing gear and turning on LEDs 
		
		lcd_gotoxy(1,1);
		printf("RPM: %d", RPM);
		
		lcd_gotoxy(2,1);
		printf("Gear: %d", current_gear);
		
		if (RPM > 7000) // RED Zone
		{
			wear_counter++;
			eeprom_write_byte((uint8_t *)addr, wear_counter); // Updating value for wear_counter on EEPROM
			PORTD = 0xF0;
			lcd_clear();
			printf("RED");
			_delay_ms(1000);
			
			current_gear = 1;
			RPM = 1000;
		}
    }
}



void init_interrupt(){
	
	TCCR0A |= (1<<WGM01); //Sets the Timer Mode to CTC
	
	OCR0A = 0x7C; // 124 = 2ms
	
	TIMSK0 |= (1 << OCIE0A); //Set ISR COMPA vect
	sei();
	TCCR0B |= (1<<CS02); // Prescaler of 256
	
}

ISR (TIMER0_COMPA_vect){
	
	if (PINC == 0x3F)
	{
		RPM = RPM-5;
	}
	if (PINC == 0x3E)
	{
		RPM = RPM+5;
	}
	
	if (PINC == 0x3D)
	{
		RPM = RPM-50;
	}
	
}

void initialize()
{
	DDRC = 0xF0; // Buttons input
	PORTC = 0x3F;
	DDRD = 0xFF; // LED output
	PORTD = 0x00; // LED off
}

void gear_state(void){
	
	if ((current_gear == 1) & (RPM < 1000))
	{
		RPM = 1000;
	}
	
	else if ((current_gear == 1) & (RPM >2000)){
		current_gear++;
		RPM = 1000;
	}
	
	else if ((current_gear == 2) & (RPM >2000)){
		current_gear++;
		RPM = 1000;
	}
	
	else if ((current_gear == 3) & (RPM >2000)){
		current_gear++;
		RPM = 1000;
	}
	
	
	else if ((current_gear == 2) & (RPM < 1000)){
		current_gear--;
		RPM = 2000;
	}
	
	else if ((current_gear == 3) & (RPM < 1000)){
		current_gear--;
		RPM = 2000;
	}
	
	else if ((current_gear == 4) & (RPM < 1000)){
		current_gear--;
		RPM = 2000;
	}
	
	
	
	if (current_gear == 1)
	{
		PORTD = 0x10;
	}
	
	else if (current_gear == 2)
	{
		PORTD = 0x20;
	}
	
	else if (current_gear == 3)
	{
		PORTD = 0x40;
	}
	
	else if (current_gear == 4)
	{
		PORTD = 0x80;
		
	}

}

void usart_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
}

void usart_send( unsigned char data){
	while(!(UCSR0A & (1<<UDRE0))); //wait for transmit buffer
	UDR0 = data; //data to be sent
}