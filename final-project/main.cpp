#define  F_CPU 16000000  // 16 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//TWI constants
#define TWI_START 0xA4 //(enable=1, start=1, interrupt flag=1)
#define TWI_STOP 0x94 //(enable=1, stop=1, interrupt flag=1)
#define TWI_SEND 0x84 //(enable=1, start=0, interrupt flag=1)

//LCD Signals
//D7 D6 D5 D4 BL E RW RS
#define LCD_ADDRESS 0x4E	//device address (0x27) in 7 MSB + W (0) in LSB
#define LCD_DISABLE 0x04
#define LCD_ENABLE 0x00
#define LCD_WRITE 0x00
#define LCD_READ 0x01
#define LCD_RS 0x01
#define LCD_BL 0x08

//LCD Commands (D0-D7 in Datasheet)
#define CLEAR_DISPLAY 0x01		//need to wait 2ms after this call (per datasheet)
#define CURSOR_HOME 0x02		//need to wait 2ms after this call (per datasheet)
#define SET_ADDRESS 0x80

volatile uint8_t rising_edges = 0x00;
volatile uint8_t falling_edges =0x00;
volatile uint8_t switch_states =0x00;

//declare buffer array
const uint8_t kBufferLength = 8;
volatile uint8_t switch_array[kBufferLength]={0};

//stepper state machine variable
int8_t stepperState=0;
int8_t stepperDirection=0;

//Sample switches at particular interval
void softwareDebounce(uint8_t sampleData){
	static uint8_t sample_index= 0;			//Index used to store switch samples in array
	uint8_t stable_high = 0xFF;				//Initialize temporary stable_high all high
	uint8_t stable_low = 0;					//Initialize temporary stable_low all low
	switch_array[sample_index] = sampleData;	//Store switch sample from SPI new_data in array
	
	//Loop through all historical switch samples to check for stable highs and lows
	for (uint8_t i=0;i<kBufferLength;i++){
		//"And" for stable high (all 1's will produce "1" for stable high)
		stable_high &= switch_array[i];
		//"Or" for stable low (all 0's will produce "0" for stable low)
		stable_low |= switch_array[i];
	}
	rising_edges = (~switch_states)&stable_high;					//Detect Rising Edges
	falling_edges = switch_states&(~stable_low);					//Detect Falling Edges
	switch_states = stable_high|(switch_states&stable_low);			//Update switch states
	
	//Update sample index and wrap if necessary
	if(++sample_index>=kBufferLength)
	sample_index = 0;//wrap
}

void TWI(unsigned char address, unsigned char data)
{
	TWCR = TWI_START;				//send start condition
	while(!(TWCR & (1<<TWINT))){}	//wait for start condition to transmit
	TWDR = address;					//send address to get device attention
	TWCR = TWI_SEND;				//Set TWINT to send address
	while(!(TWCR & (1<<TWINT))){}	//wait for address to go out
	TWDR = data;					//send data to address
	TWCR = TWI_SEND;				//Set TWINT to send address
	while(!(TWCR & (1<<TWINT))){}	//wait for data byte to transmit
	TWCR = TWI_STOP;				//finish transaction
}

void LCD_Display(unsigned char data)
{
	// Put in character data upper bits while keeping enable bit high
	TWI(LCD_ADDRESS,(data & 0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL|LCD_RS);
	
	// Pull enable bit low to make LCD display the data
	TWI(LCD_ADDRESS,(data & 0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL|LCD_RS);

	// Put in character data lower bits while keeping enable bit high
	TWI(LCD_ADDRESS,((data<<4) & 0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL|LCD_RS);
	
	// Pull enable bit low to make LCD display the data
	TWI(LCD_ADDRESS,((data<<4) & 0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL|LCD_RS);
}

void LCD_Command(unsigned char command)
{
	// Put in command data upper bits while keeping enable bit high
	TWI(LCD_ADDRESS,(command &0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL);
	
	// Pull enable bit low to make LCD process the command
	TWI(LCD_ADDRESS,(command &0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL);
	
	// Put in command data lower bits while keeping enable bit high
	TWI(LCD_ADDRESS,((command<<4) & 0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL);
	
	// Pull enable bit low to make LCD process the command
	TWI(LCD_ADDRESS,((command<<4) & 0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL);
}

void stepperStateMachine(){
	//STATE MACHINE FOR UNL2003 FULL STEP MODE
	switch(stepperState){
		//STEP AB
		case 0:
		PORTC=0x03;
		stepperState=stepperState+stepperDirection;
		break;
		//STEP BC
		case 1:
		PORTC=0x06;
		stepperState=stepperState+stepperDirection;
		break;
		//STEP CD
		case 2:
		PORTC=0x0C;
		stepperState=stepperState+stepperDirection;
		break;
		//STEP DA
		case 3:
		PORTC=0x09;
		stepperState=stepperState+stepperDirection;
		break;
	}
	
	//CHECK STEPPER STATE OVERFLOW
	if (stepperState>3){
		stepperState=0;
	}
	else if (stepperState<0){
		stepperState=3;
	}
}

// Constants for lock states
const int8_t UNLOCK_CLEAR_STATE = 0;
const int8_t UNLOCK_1_STATE = 1;
const int8_t UNLOCK_12_STATE = 2;
const int8_t UNLOCK_123_STATE = 3;
const int8_t LOCK_CLEAR_STATE = 4;
const int8_t LOCK_3_STATE = 5;
const int8_t LOCK_32_STATE = 6;
const int8_t LOCK_321_STATE = 7;
// lock state intialized to unlocked & no input
int8_t lockState= UNLOCK_CLEAR_STATE;

// Small method for printing lock state to screen
void printLockState() {
	LCD_Command(CLEAR_DISPLAY);
	_delay_ms(2);
	// First line is Lock/Unlock based on state
	LCD_Command(SET_ADDRESS|0x00);
	if (lockState == UNLOCK_CLEAR_STATE || lockState == UNLOCK_1_STATE || lockState == UNLOCK_12_STATE || lockState == UNLOCK_123_STATE) {
		LCD_Display('U');
		LCD_Display('N');
		LCD_Display('L');
		LCD_Display('O');
		LCD_Display('C');
		LCD_Display('K');
		} else {
		LCD_Display('L');
		LCD_Display('O');
		LCD_Display('C');
		LCD_Display('K');
	}
	// Second line shows current code state
	LCD_Command(SET_ADDRESS|0x40);
	LCD_Display('C');
	LCD_Display('O');
	LCD_Display('D');
	LCD_Display('E');
	LCD_Display(':');
	switch (lockState) {
		case UNLOCK_1_STATE:
		LCD_Display('1');
		break;
		case UNLOCK_12_STATE:
		LCD_Display('1');
		LCD_Display('2');
		break;
		case UNLOCK_123_STATE:
		LCD_Display('1');
		LCD_Display('2');
		LCD_Display('3');
		break;
		case LOCK_3_STATE:
		LCD_Display('3');
		break;
		case LOCK_32_STATE:
		LCD_Display('3');
		LCD_Display('2');
		break;
		case LOCK_321_STATE:
		LCD_Display('3');
		LCD_Display('2');
		LCD_Display('1');
		break;
	}
}

void lockStateMachine(){
	// Update lock state based on button pressed
	switch (lockState) {
		case UNLOCK_CLEAR_STATE:
		if (falling_edges&0x04) {
			lockState = UNLOCK_1_STATE;
		}
		break;
		case UNLOCK_1_STATE:
		if (falling_edges&0x02) {
			lockState = UNLOCK_12_STATE;
			} else {
			lockState = UNLOCK_CLEAR_STATE;
		}
		break;
		case UNLOCK_12_STATE:
		if (falling_edges&0x01) {
			lockState = UNLOCK_123_STATE;
			} else {
			lockState = UNLOCK_CLEAR_STATE;
		}
		break;
		case LOCK_CLEAR_STATE:
		if (falling_edges&0x01) {
			lockState = LOCK_3_STATE;
		}
		break;
		case LOCK_3_STATE:
		if (falling_edges&0x02) {
			lockState = LOCK_32_STATE;
			} else {
			lockState = LOCK_CLEAR_STATE;
		}
		break;
		case LOCK_32_STATE:
		if (falling_edges&0x04) {
			lockState = LOCK_321_STATE;
			} else {
			lockState = LOCK_CLEAR_STATE;
		}
		break;
	}
	// Update LCD
	printLockState();
	// If lock/unlock combo pressed, turn motor, update state, and update LCD
	if (lockState == UNLOCK_123_STATE) {
		//unlock
		stepperDirection = -1;
		for (int i = 0; i < 1024; i++) {
			stepperStateMachine();
			_delay_ms(10);
		}
		lockState = LOCK_CLEAR_STATE;
		printLockState();
		} else if (lockState == LOCK_321_STATE) {
		//lock
		stepperDirection = 1;
		for (int i = 0; i < 1024; i++) {
			stepperStateMachine();
			_delay_ms(10);
		}
		lockState = UNLOCK_CLEAR_STATE;
		printLockState();
	}
}



ISR(TIMER1_COMPA_vect){
	// Debounce push buttons
	softwareDebounce(PIND);
	if (falling_edges&0x07) {
		// Button pressed, update lock state
		lockStateMachine();
	}
}


int main(void)
{
	//CONFIGURE IO
	DDRD = (0<<PIND0)|(0<<PIND1)|(0<<PIND2); // push buttons
	DDRC = (1<<PINC0)|(1<<PINC1)|(1<<PINC2)|(1<<PINC3); // motor controller
	//Configure Bit Rate (TWBR and TWSR)
	TWBR = 18;	//TWBR=18
	TWSR = (0<<TWPS1)|(1<<TWPS0);	//PRESCALER = 1
	
	//Configure TWI Interface (TWCR)
	TWCR = (1<<TWEN);
	
	//INITIALIZE LCD
	TWI(LCD_ADDRESS,0x30|LCD_DISABLE|LCD_WRITE|LCD_BL);	// (data length of 8, number of lines=2, 5x8 digit space, load data)
	TWI(LCD_ADDRESS,0x30|LCD_ENABLE|LCD_WRITE|LCD_BL);	// (clock in data)
	_delay_ms(15);
	TWI(LCD_ADDRESS,0x30|LCD_DISABLE|LCD_WRITE|LCD_BL);	// (data length of 8, number of lines=2, 5x8 digit space, load data)
	TWI(LCD_ADDRESS,0x30|LCD_ENABLE|LCD_WRITE|LCD_BL);	// (clock in data)
	_delay_ms(4.1);
	TWI(LCD_ADDRESS,0x30|LCD_DISABLE|LCD_WRITE|LCD_BL);	// (data length of 8, number of lines=2, 5x8 digit space, load data)
	TWI(LCD_ADDRESS,0x30|LCD_ENABLE|LCD_WRITE|LCD_BL);	// (clock in data)
	_delay_ms(4.1);
	TWI(LCD_ADDRESS,0x20|LCD_DISABLE|LCD_WRITE|LCD_BL);	// (data length of 4, number of lines=2, 5x8 digit space, load data)
	TWI(LCD_ADDRESS,0x20|LCD_ENABLE|LCD_WRITE|LCD_BL);	// (clock in data)
	TWI(LCD_ADDRESS,0x20|LCD_DISABLE|LCD_WRITE|LCD_BL);	// (load data)
	TWI(LCD_ADDRESS,0x20|LCD_ENABLE|LCD_WRITE|LCD_BL);	// (clock in data)
	
	//HOME CURSOR
	TWI(LCD_ADDRESS,0x80|LCD_DISABLE|LCD_WRITE|LCD_BL);	// (load data)
	TWI(LCD_ADDRESS,0x80|LCD_ENABLE|LCD_WRITE|LCD_BL);	// (clock in data)
	
	//CLEAR DISPLAY AND WAIT TO FINSIH
	LCD_Command(CLEAR_DISPLAY);
	_delay_ms(2);
	
	LCD_Command(0x0C); //Display no cursor
	LCD_Command(0x06); //Automatic Increment
	
	LCD_Command(SET_ADDRESS|0x00);
	LCD_Display('U');
	LCD_Display('N');
	LCD_Display('L');
	LCD_Display('O');
	LCD_Display('C');
	LCD_Display('K');
	LCD_Command(SET_ADDRESS|0x40);
	LCD_Display('C');
	LCD_Display('O');
	LCD_Display('D');
	LCD_Display('E');
	LCD_Display(':');
	
	//Configure Timer 1
	TCCR1A = (0<<WGM10)|(0<<WGM11);
	TCCR1B = (1<<WGM12)|(0<<WGM13)|(0<<CS12)|(0<<CS11)|(1<<CS10);
	TIMSK1 = (1<<OCIE1A);
	OCR1A = 15999;
	
	sei();
	
	/* Replace with your application code */
	while (1)
	{
	}
}
