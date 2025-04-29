#define  F_CPU 16000000  // 16 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

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

// Pinout
#define BUZZER_PIN PIND6
#define GREEN_LED_PIN PIND5
#define RED_LED_PIN PIND4

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

// Constants for alarm states
const int8_t DISARM_CLEAR_STATE = 0;
const int8_t DISARM_1_STATE = 1;
const int8_t DISARM_12_STATE = 2;
const int8_t DISARM_123_STATE = 3;
const int8_t ARM_CLEAR_STATE = 4;
const int8_t ARM_3_STATE = 5;
const int8_t ARM_32_STATE = 6;
const int8_t ARM_321_STATE = 7;

// Alarm code constants
const uint8_t BUTTON_1 = 0x04;
const uint8_t BUTTON_2 = 0x02;
const uint8_t BUTTON_3 = 0x01;
const uint8_t DISARM_CODE_1 = BUTTON_2;
const char DISARM_CODE_1_CHAR = '2';
const uint8_t DISARM_CODE_2 = BUTTON_1;
const char DISARM_CODE_2_CHAR = '1';
const uint8_t DISARM_CODE_3 = BUTTON_3;
const char DISARM_CODE_3_CHAR = '3';
const uint8_t ARM_CODE_1 = BUTTON_2;
const char ARM_CODE_1_CHAR = '2';
const uint8_t ARM_CODE_2 = BUTTON_3;
const char ARM_CODE_2_CHAR = '3';
const uint8_t ARM_CODE_3 = BUTTON_1;
const char ARM_CODE_3_CHAR = '1';
// alarm state intialized to disarmed & no input
int8_t alarmState= DISARM_CLEAR_STATE;
const char* ARMED_TEXT = "ARMED";
const char* DISARMED_TEXT = "DISARMED";
const char* ALARM_TEXT = "ALARM";
// alarm static variables
bool alarm_tripped = false;

void clearLCD(uint8_t start_address, uint8_t end_address) {
	int curr_address = start_address;
	LCD_Command(SET_ADDRESS|start_address);
	while (curr_address <= end_address) {
		LCD_Display(' ');
		curr_address++;
	}
}

void printArmState() {
	clearLCD(0x00, 0x07);
	LCD_Command(SET_ADDRESS|0x00);
	if (alarmState == DISARM_CLEAR_STATE || alarmState == DISARM_1_STATE || alarmState == DISARM_12_STATE || alarmState == DISARM_123_STATE) {
		for (unsigned int i = 0; i < strlen(DISARMED_TEXT); i++) {
			LCD_Display(DISARMED_TEXT[i]);
		}
	} else if (alarm_tripped) {
		for (unsigned int i = 0; i < strlen(ALARM_TEXT); i++) {
			LCD_Display(ALARM_TEXT[i]);
		}
	} else {
		for (unsigned int i = 0; i < strlen(ARMED_TEXT); i++) {
			LCD_Display(ARMED_TEXT[i]);
		}
	}
}

void printCodeState() {
	clearLCD(0x40, 0x42);
	LCD_Command(SET_ADDRESS|0x40);
	switch (alarmState) {
		case DISARM_1_STATE:
		LCD_Display(DISARM_CODE_1_CHAR);
		break;
		case DISARM_12_STATE:
		LCD_Display(DISARM_CODE_1_CHAR);
		LCD_Display(DISARM_CODE_2_CHAR);
		break;
		case DISARM_123_STATE:
		LCD_Display(DISARM_CODE_1_CHAR);
		LCD_Display(DISARM_CODE_2_CHAR);
		LCD_Display(DISARM_CODE_3_CHAR);
		break;
		case ARM_3_STATE:
		LCD_Display(ARM_CODE_1_CHAR);
		break;
		case ARM_32_STATE:
		LCD_Display(ARM_CODE_1_CHAR);
		LCD_Display(ARM_CODE_2_CHAR);
		break;
		case ARM_321_STATE:
		LCD_Display(ARM_CODE_1_CHAR);
		LCD_Display(ARM_CODE_2_CHAR);
		LCD_Display(ARM_CODE_3_CHAR);
		break;
	}
}

// For time multiplexing LED when alarm tripped
uint8_t time_since_blink = 0;
void updateLED() {
	switch (alarmState) {
		case DISARM_CLEAR_STATE:
		case DISARM_1_STATE:
		case DISARM_12_STATE:
		case DISARM_123_STATE:
			PORTD |= (1<<GREEN_LED_PIN); // Turn green on
			PORTD &= ~(1<<RED_LED_PIN); // Turn red off
			PORTD &= ~(1<<BUZZER_PIN); // Turn buzzer off	
			time_since_blink = 0;
			break;
		case ARM_CLEAR_STATE:
		case ARM_3_STATE:
		case ARM_32_STATE:
		case ARM_321_STATE:
			if (!alarm_tripped) {
				PORTD |= (1<<RED_LED_PIN); // Turn red on
				PORTD &= ~(1<<GREEN_LED_PIN); // Turn green off
				PORTD &= ~(1<<BUZZER_PIN); // Turn buzzer off
			} else {
				PORTD &= ~(1<<GREEN_LED_PIN); // Ensure green is off
				PORTD |= (1<<BUZZER_PIN); // Turn buzzer on
				if (time_since_blink >= 25) {
					PORTD ^= (1<<RED_LED_PIN); // toggle red
					time_since_blink = 0;
				} else {
					time_since_blink++;
				}
			}
			break;
	}	
}

void tripAlarm(bool tripped) {
	alarm_tripped = tripped;
	printArmState();
	updateLED();
}

int16_t arm_count = 1000;
void armAlarm() {
	stepperDirection = -1;
	for (int i = 0; i < 512; i++) {
		stepperStateMachine();
		_delay_ms(10);
	}
	alarmState = ARM_CLEAR_STATE;
	printArmState();
	printCodeState();
}

void disarmAlarm() {
	tripAlarm(false);
	stepperDirection = 1;
	for (int i = 0; i < 512; i++) {
		stepperStateMachine();
		_delay_ms(10);
	}
	alarmState = DISARM_CLEAR_STATE;
	printCodeState();
}

char prev_tens = ' ';
char prev_ones = ' ';
void printCountdown() {
	if (alarmState == DISARM_123_STATE || alarmState == ARM_321_STATE) {
		uint8_t count_seconds = (arm_count / 100) + 1;
		char tens_place = '0' + ((count_seconds / 10) % 10);
		tens_place = tens_place == '0' ? ' ' : tens_place;
		char ones_place = '0' + (count_seconds % 10);
		//ones_place = ones_place == '0' ? ' ' : ones_place;
		if (tens_place != prev_tens || ones_place != prev_ones) {
			LCD_Command(SET_ADDRESS|0x4E);
			LCD_Display(tens_place);
			LCD_Display(ones_place);
		}
	} else {
		clearLCD(0x4E, 0x4F);
	}
}

void alarmStateMachine(){
	// Update alarm state based on button pressed
	switch (alarmState) {
		case DISARM_CLEAR_STATE:
		printArmState();
		if (falling_edges&DISARM_CODE_1) {
			alarmState = DISARM_1_STATE;
		}
		break;
		case DISARM_1_STATE:
		if (falling_edges&DISARM_CODE_2) {
			alarmState = DISARM_12_STATE;
		} else {
			alarmState = DISARM_CLEAR_STATE;
		}
		break;
		case DISARM_12_STATE:
		if (falling_edges&DISARM_CODE_3) {
			alarmState = DISARM_123_STATE;
			arm_count = 1000;	
		} else {
			alarmState = DISARM_CLEAR_STATE;
		}
		break;
		case ARM_CLEAR_STATE:
		printArmState();
		if (falling_edges&ARM_CODE_1) {
			alarmState = ARM_3_STATE;
		} else {
			tripAlarm(true);
		}
		break;
		case ARM_3_STATE:
		if (falling_edges&ARM_CODE_2) {
			alarmState = ARM_32_STATE;
		} else {
			alarmState = ARM_CLEAR_STATE;
			tripAlarm(true);
		}
		break;
		case ARM_32_STATE:
		if (falling_edges&ARM_CODE_3) {
			alarmState = ARM_321_STATE;
			arm_count = 1000;
		} else {
			alarmState = ARM_CLEAR_STATE;
			tripAlarm(true);
		}
		break;
	}
	// Update LCD
	printCodeState();
}

ISR(TIMER1_COMPA_vect){
	// Debounce push buttons
	softwareDebounce(PINB);
	if (falling_edges&0x07) {
		// Button pressed, update alarm state
		alarmStateMachine();
	}
	// If arm/disarm combo pressed, turn motor, update state, and update LCD
	if (alarmState == DISARM_123_STATE) {
		//arming the alarm
		if (--arm_count <= 0) { // waited 10s
			armAlarm();
		}
		printCountdown();
	} else if (alarmState == ARM_321_STATE) {
		//disarming the alarm
		disarmAlarm();
		printCountdown();
	}
	updateLED();
}

const uint8_t HELP_TX_PACKET = 0b11110000; // indicates to other board we've "requested help"
const uint8_t HELP_RX_PACKET = 0b00001111; // message that should be sent back by other board

#define TRANSMIT true
#define RECEIVE false
bool usart_mode;
void usart_config(bool tx) {
	usart_mode = tx;
	if (tx == TRANSMIT) {
		UCSR0B=(1<<TXCIE0)|(1<<TXEN0)|(0<<RXCIE0)|(0<<RXEN0);
	} else {
		UCSR0B=(0<<TXCIE0)|(0<<TXEN0)|(1<<RXCIE0)|(1<<RXEN0);
	}
}

ISR(USART_RX_vect){
	// Help packet finished transmitting
	if (usart_mode == RECEIVE && UDR0 == HELP_RX_PACKET) {
		// received correct response, display OK for 2 seconds then return to business as usual
		LCD_Command(CLEAR_DISPLAY);
		_delay_ms(2);
		LCD_Display('O');
		LCD_Display('K');
		_delay_ms(2000);
		printArmState(); // redisplay state
		printCodeState(); // redisplay code
		usart_config(TRANSMIT);
	}
}


ISR(USART_TX_vect){
	// Help packet finished transmitting, switch to rx mode
	if (usart_mode == TRANSMIT) {
		usart_config(RECEIVE);	
	}
}

// Triggered on help button falling edge
ISR(INT1_vect) {
	//start USART transmission
	if (usart_mode == TRANSMIT && (UCSR0A & (1<<UDRE0))){ //check to see if transmit buffer is clear and we are in transmit mode
		UDR0 = HELP_TX_PACKET; //set data register to tx packet
	}
}

int main(void)
{
	//CONFIGURE IO
	DDRD = (0<<PIND3)|(1<<RED_LED_PIN)|(1<<GREEN_LED_PIN)|(1<<BUZZER_PIN); // D3 - Help Push Button
	DDRB = (0<<PINB0)|(0<<PINB1)|(0<<PINB2);
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
	
	printArmState();
	printCodeState();
	
	//Configure Timer 1
	// CTC, Prescalar of 8
	TCCR1A = (0<<WGM10)|(0<<WGM11);
	TCCR1B = (1<<WGM12)|(0<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10);
	TIMSK1 = (1<<OCIE1A);
	OCR1A = 19999; // 10 ms
	
	// Configure INT1 Interrupt
	EIMSK = (1<<INT1);
	EICRA = (1<<ISC11)|(0<ISC10);
	
	// Configure USART
	UCSR0A=(0<<U2X0)|(1<<MPCM0); //enable multiprocessor mode
	usart_config(TRANSMIT); // transmit mode, tx interrupt
	UCSR0C=(1<<UCSZ01)|(1<<UCSZ00); // 8-bit characters
	UBRR0=3; //same baud rate for both Tx and Rx
	DDRD &= ~(1 << PIND0); // Set RX as input
	DDRD |= (1 << PIND1);  // Set TX as output
	
	sei();
	
	/* Replace with your application code */
	while (1) {}
}
