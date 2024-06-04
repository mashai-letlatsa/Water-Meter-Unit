/*
 * CODEWater.cpp
 *
 * Created: 2023/04/27 22:33:48
 * Author : 219024684
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>


#define LCDDDDR DDRB
#define LCDCDDR DDRD
#define LCDDATA PORTB
#define LCDCONTR PORTD
#define BUZZER_PIN PC2 // Change this to the pin connected to your buzzer
//**********************************************************************************
#define BAUDRATE 9600   // baud rate for serial communication
#define UBRR_VALUE ((F_CPU / (16UL * BAUDRATE)) - 1)
//**********************************************************************************
#define EEPROM_VOLUME_ADDR 0x00
#define EEPROM_BALANCE_ADDR 0x04
//**********************************************************************************
#define EEPROM_ADDR_START 0x0000
#define NUM_VALUES_TO_STORE 5
#define LM35_SENSOR_CHANNEL 0 // Define the ADC channel connected to the LM35 sensor
#define RTC_ADDRESS 0xD0 // DS1307 I2C address
//**********************************************************************************
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define RS 4
#define RW 5
#define EN 6
#define RL 3
#define WV 7

int countSec=0, valveState=0, flow_frequency, timer_seconds, pulseCount, pulseCounter, pulseCnt;
//*************************************
double FlowRate=0, Volume_w=0, balance=1000.00;

volatile uint32_t counter = 0;
float errorPercentage=0;
char volB[10],volM[10] ;
uint8_t i = 0;

double temperature;
char tempStr[16]; // Buffer to hold the temperature as a string
uint8_t seconds, minutes, hours;
uint8_t day, month;
uint16_t year;



void lcdCommand(char cmnd){
LCDDATA = cmnd;
LCDCONTR &= ~(1<<RS); // RS = 0 to indicate that we send command to lcd;
LCDCONTR |= (1<<EN);  // pulse enable;
_delay_ms(5);
LCDCONTR &= ~(1<<EN);
_delay_ms(5);
return;
}
void lcdInit(){
LCDDDDR=0xFF;//PORTB as digital output
LCDCDDR|=(1<<EN)|(1<<RS)|(1<<RW);
_delay_ms(5);
lcdCommand(0x38); // 8 bits mode initialization
lcdCommand(0x0c);// display cursor ON and OFF
lcdCommand(0x06); // shift cursor to the right
lcdCommand(0x01); // clear display
_delay_ms(5);
return;
}
void lcdData(char data){
LCDDATA = data;
LCDCONTR |= (1<<RS); // RS = 1 to indicate that we sending a data to the lcd;
LCDCONTR  |= (1<<EN);  // pulse enable;
_delay_ms(5);
LCDCONTR  &= ~(1<<EN);
_delay_ms(5);
return;
}
void lcdClear(){
lcdCommand(0x01); // clear display
return;
}
void lcdString(char* str){
int i;
for(i=0; str[i]!=0;i++){
lcdData(str[i]);
}
return;
}
void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
 char address = 0;
 if (row == 0) {
	 address = 0x00 + pos;
	 } else if (row == 1) {
	 address = 0x40 + pos;
	 } else if (row == 2) {
	 address = 0x10 + pos;
	 } else if (row == 3) {
	 address = 0x50 + pos;
 }
 lcdCommand(address | 0x80);
 lcdString(str);
}
void ADC_Init() {
	ADMUX = (1 << REFS0); // Reference voltage at AVCC (5V)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, Prescaler = 64
}

// Function to read ADC value from the given channel
uint16_t ADC_Read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
	ADCSRA |= (1 << ADSC); // Start conversion
	while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
	return ADC; // Return the ADC value
}
////*************************************************************************************************
// I2C initialization function
void i2c_init() {
	TWBR = 32; // Set bit rate register for 100kHz I2C clock
	TWSR &= ~(1 << TWPS0) & ~(1 << TWPS1); // Prescaler = 1
}

// I2C start condition function
void i2c_start() {
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

// I2C stop condition function
void i2c_stop() {
	TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
	while (TWCR & (1 << TWSTO));
}

// I2C write function
void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

// I2C read function with acknowledgment
uint8_t i2c_read_ack() {
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// I2C read function without acknowledgment
uint8_t i2c_read_nack() {
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// Initialize the DS1307 RTC
void rtc_init() {
	i2c_init(); // Initialize I2C communication

	// Initialize DS1307 with appropriate settings if needed
}
uint8_t bcdToDecimal(uint8_t bcd) {
	return ((bcd >> 4) * 10) + (bcd & 0x0F);
}


// Read time and date from the DS1307 RTC
void rtc_read(uint8_t *seconds, uint8_t *minutes, uint8_t *hours,
uint8_t *day, uint8_t *month, uint16_t *year) {
	i2c_start();
	i2c_write(RTC_ADDRESS); // Write address
	i2c_write(0x00); // Start reading from address 0x00 (seconds register)
	i2c_start();
	i2c_write(RTC_ADDRESS | 1); // Read address

	*seconds = bcdToDecimal(i2c_read_ack() & 0x7F);
	*minutes = bcdToDecimal(i2c_read_ack());
	*hours = bcdToDecimal(i2c_read_ack());
	i2c_read_ack(); // Read day (unused)
	*day = bcdToDecimal(i2c_read_ack());
	*month = bcdToDecimal(i2c_read_ack());
	*year = bcdToDecimal(i2c_read_nack());

	i2c_stop();
}

void DisplayTime(){
	
	rtc_read(&seconds, &minutes, &hours, &day, &month, &year);
	 // Display Time
	 char timeStr[9];
	 sprintf(timeStr, "Time:%02d:%02d:%02d", hours, minutes, seconds);
	 LCD_String_xy(2, 0, timeStr);

	 // Display Date
	 char dateStr[11];
	 sprintf(dateStr, "Date:%02d/%02d/%02d", day, month, year);
	 LCD_String_xy(3, 0, dateStr);
}
//**************************************************************************
void wmuInit(){
 // Initialize Timer1 and set external interrupt on INT1
    GICR |= (1 << INT1);
 // Set interrupt on rising edge of INT1 pin
    MCUCR |= (1 << ISC11) | (1 << ISC10);
	DDRD|=(1<<WV);// water valve control pin
	PORTD &= ~(1<<WV);
	TCNT1 = 57723;   // for 1 sec at 8 MHz
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10) | (1<<CS12);  // Timer mode with 1024 pre-scaler
	TIMSK = (1<<TOIE1);   // Enable timer1 overflow interrupt(TOIE1)
	GICR|=(1<<INT0); // enable int0 interrupt
	MCUCR|= (0<<ISC01)|(1<<ISC00);   //Rising edge on INT0 triggers interrupt.
	sei();//enable global interrupt
	return;
}
double flowRate(){
	
	return FlowRate;
	
}
double volume_calc(){
	return Volume_w;
}
void valveControl(){
	_delay_us(10);
	if(balance==0 || errorPercentage > 10.0){
		PORTD&=~(1<<WV);
		}else{
		PORTD|=(1<<WV);
	}
	_delay_us(10);
	return;
}
//******************************************************************************************************
void buzzer(){
	DDRC |= (1 << BUZZER_PIN); // Set buzzer pin as output
	
		if (errorPercentage > 10.0)
		{
			PORTC |= (1 << BUZZER_PIN); // Turn on buzzer by setting pin to high
			_delay_ms(100); // Delay for 0.1 second to allow buzzer to sound
			PORTC &= ~(1 << BUZZER_PIN); // Turn off buzzer by setting pin to low
		}
		
}
//******************************************************************************************************
void comparePulse(){
	errorPercentage=(abs((float)(pulseCount -  pulseCnt)) / (float)pulseCnt)* 100.0;
	if (pulseCnt>pulseCount && errorPercentage>10.0) {
		LCD_String_xy(3,0,"Leakage!"); // Display the leakage status on the LCD 
		buzzer();
		} else {
		LCD_String_xy(3,0,"No Leakage!"); // Display the leakage status on the LCD 
	}
}
//*****************************************************************************************************
void USART_Init(void) {
	// set baud rate
	UBRRH = (uint8_t)(UBRR_VALUE >> 8);
	UBRRL = (uint8_t)(UBRR_VALUE);
	// enable receiver and transmitter
	UCSRB = (1 << RXEN) | (1 << TXEN);
	// set frame format: 8 data bits, no parity, 1 stop bit
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
}

void USART_Transmit(unsigned char data) {
	// wait for empty transmit buffer
	while (!(UCSRA & (1 << UDRE)));
	// put data into buffer, send data
	UDR = data;
}

void USART_Print(char* string) {
	while (*string) {
		USART_Transmit(*string++);
	}
}
//*****************************************************************************************************
void TempControl(){
	// Control the heater and fan based on the temperature
	if (temperature <= 22) {
		LCD_String_xy(1, 0, "               "); // Clear the status area on the LCD
		LCD_String_xy(1, 0, "Water Cold");
		} else if (temperature >= 32) {
		LCD_String_xy(1, 0, "               "); // Clear the status area on the LCD
		LCD_String_xy(1, 0, "Water Hot");

		} else {
		_delay_ms(10);
		LCD_String_xy(1, 0, "               "); // Clear the status area on the LCD
		LCD_String_xy(1, 0, "Water Normal");

	}
}
//************************************************************************************************
void TempFunction(){
	uint16_t adcValue = ADC_Read(LM35_SENSOR_CHANNEL); // Read ADC value from LM35 sensor

	// Convert ADC value to temperature in degrees Celsius
	double voltage = (adcValue * 5.0) / 1024.0; // Calculate the voltage at ADC pin (LM35 output is 0-5V)
	temperature = voltage * 100; // LM35 sensor provides 10 mV/°C, so 1V = 100°C

	// Convert temperature to a string with one decimal place
	dtostrf(temperature, 4, 1, tempStr);

	// Display temperature on the LCD
	//lcdClear(); // Clear the LCD display
	LCD_String_xy(0, 0, "Tp:");
	LCD_String_xy(0, 3,tempStr);
	_delay_ms(50); // Delay for stable readings
	TempControl();
	
}
//****************************************************************************************************
void Turbidity() {
	uint16_t TurbValue = ADC_Read(1);
	double turbidity = (TurbValue * 5.0) / 1023.0;
	
	// Convert double to char array
	char turbidity_str[10];
	dtostrf(turbidity, 2, 1, turbidity_str);

	// Display turbidity value on LCD
	LCD_String_xy(0, 10, "Td:");
	LCD_String_xy(0, 13, turbidity_str);

}

//**************************************************************************************************
void wmuUpdate(){
	valveControl();
	//lcdClear();
	char buffer[20];
	strcpy(buffer,"FR=");
	dtostrf(flowRate(),4,2,&buffer[3]);
	strcat(buffer,"L/m");
	LCD_String_xy(0,0,buffer);
	
	char buff[20];
	strcpy(buff,"Vol=");
	dtostrf(Volume_w,4,2,volM);
	strcat(buff,volM);
	strcat(buff,"L");
	LCD_String_xy(1,0,buff);
	//
	char k[20];
	char h[20];
	strcpy(h,"BAL=");
	LCD_String_xy(2,0,h);
	dtostrf(balance,5,2,k);
	strcat(k,volB);
	strcat(k,"L");
	LCD_String_xy(2,4,k);
	comparePulse();
}

ISR(INT0_vect){
	cli();
	flow_frequency++;
	pulseCounter++;
	pulseCnt=pulseCounter/2;
	sei();
}
ISR(INT1_vect){
		cli();
	pulseCount++; // Increment pulse count on rising edge interrupt
	sei();
}

ISR(TIMER1_OVF_vect){
	cli();
	countSec=flow_frequency/2;
	FlowRate=(countSec/7.5)/60.0; //Flow rate in Liters/Second
	Volume_w+=((FlowRate)*60); //Volume calculation in Liters (The volume accumulate per minute)
	if (balance >= (FlowRate)*(60.0))
	{
		balance-=((FlowRate)*(60.0)); // Update user account balance
		} else{
		balance = 0;
	}
	
	flow_frequency=0;
	TCNT1 = 57723;   // for 1u sec at 8 MHz
	sei();
}

int main(void){
	lcdInit();
	ADC_Init();
    wmuInit();
    while (1)
{
wmuUpdate();
_delay_ms(1000);
lcdClear();
TempFunction();
Turbidity();
DisplayTime();
_delay_ms(1000);
lcdClear();
}
}
