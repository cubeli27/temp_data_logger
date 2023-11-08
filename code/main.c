/**
	@author Jure Cubelic
	@version 0.1
	@file main.c
	@brief Lab 5 - Main programm for temperature data logger
*/

const char MtrNum[] __attribute__((__progmem__)) = "24038";

/**
	@brief The CPU speed in Hz
*/
#define F_CPU 8000000UL


#define TEMPSENSOR_OFFSET 1265 // TODO done

/**
	@brief I2C Address of the DS1307
*/
#define DS1307_I2C_ADR 0xD0 //TODO: Enter the Address of the DS1307 done

/******************************************************************************/
/* INCLUDED FILES                                                             */
/******************************************************************************/
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "i2c_master.h"
#include "init.h"
#include "lcd.h"
#include "stdio.h"
/******************************************************************************/


/******************************************************************************/
/* GLOBAL MEMORY                                                              */
/******************************************************************************/
char* dayofweek[8] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun", "Err"};

// Global Time memory
uint8_t second;
uint8_t minute;
uint8_t hour;
uint8_t day;
uint8_t month;
uint8_t year;
uint8_t weekday;

volatile uint8_t E=0;

/******************************************************************************/

void init(void){
// Digital input
DDRB &= ~(1 << DDB0);		// PB0 as Input
DDRB &= ~(1 << DDB1);		// PB1 as Input
PORTB |= ( 1 << PB0);		// Pullup PB0
PORTB |= ( 1 << PB1);		// Pullup PB1

}

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/

/**
	@brief Convert from BCD to Binary
	@param in BCD input (00-99)
	@return Binary output
*/
uint8_t ds1307_decodeBcd(uint8_t in){
	//TODO done
	return ((in>>4)*10+ in%16);
}
/******************************************************************************/

/**
	@brief Convert from Binary to BCD
	@param in Binary input (0-99)
	@return BCD output
*/
uint8_t ds1307_encodeBcd(uint8_t in){
	return ((in / 10) << 4 ) | (in % 10); 
}
/******************************************************************************/


/**
	@brief Show time/date with the LCD
*/
void display_standby(uint16_t t){
	char str[16];
	
	// Time and Year
	snprintf(str, 16, "%02d:%02d:%02d  20%02d", hour, minute,
			second, year);
	
	lcd_clear();
	lcd_string(str);
	
	
	// Date and Temperature
	snprintf(str, 16, "%02d.%02d  %d.%d C", day, month, t/10, t%10);
	
	lcd_setcursor(0,2);
	lcd_string(str);

	return;
}
/******************************************************************************/

/**
	@brief Write a row byte to the DS1307
	@param adr address to write
	@param data byte to write
*/
void ds1307_write(uint8_t adr, uint8_t data){
	
	if (i2c_master_open_write(DS1307_I2C_ADR))
		return;
	
	i2c_master_write(adr);
	i2c_master_write(data);
	
	i2c_master_close();
}
/******************************************************************************/

/**
	@brief Read a row byte from the DS1307
	@param adr address to read
	@return the received byte
*/
uint8_t ds1307_read(uint8_t adr){
	uint8_t ret;

	if (i2c_master_open_write(DS1307_I2C_ADR))
		return 0;
	
	i2c_master_write(adr);
	i2c_master_open_read(DS1307_I2C_ADR);
	ret = i2c_master_read_last();
	
	i2c_master_close();

	return ret;

}
/******************************************************************************/

/**
	@brief Start or freeze the clock of the DS1307
	@param run zero for stop, all other for run
*/
void ds1307_rtc(uint8_t run){
	
	uint8_t readout;
	
	// Read current value
	readout = ds1307_read(0x00);
	
	
	// Set CH bit
	if (run)
		readout &= ~(0x80);
	else
		readout |= 0x80;
		
	// Write value back
	ds1307_write(0x00, readout);
}
/******************************************************************************/

/**
	@brief Write the current time to the DS1307
	@return zero for no error, one for communication error
*/
uint8_t ds1307_setTime(void){
	uint8_t chbit = ds1307_read(0x00) & 0x80;

	// Open device for write
	if (i2c_master_open_write(DS1307_I2C_ADR))
		return 1;

	i2c_master_write(0x00);
	if (chbit)
		i2c_master_write(ds1307_encodeBcd(second) | 0x80);
	else
		i2c_master_write(ds1307_encodeBcd(second) & 0x7F);		
	
	i2c_master_write(ds1307_encodeBcd(minute));
	i2c_master_write(ds1307_encodeBcd(hour));
	
	i2c_master_write(weekday);		
	
	i2c_master_write(ds1307_encodeBcd(day));
	i2c_master_write(ds1307_encodeBcd(month));
	i2c_master_write(ds1307_encodeBcd(year));		
	
	
	// Close I2C bus
	i2c_master_close();
	
	return 0;
}
/******************************************************************************/

/**
	@brief Get the current time from the DS1307
	@return zero for no error, one for communication error
*/
uint8_t ds1307_getTime(void){

	// Open device for write
	if (i2c_master_open_write(DS1307_I2C_ADR))
		return 1;
	
	// select reading position (0x00)
	i2c_master_write(0x00);
	
	// (Re-)Open device for read
	i2c_master_open_read(DS1307_I2C_ADR);
	
	// Read value
	second = ds1307_decodeBcd(i2c_master_read_next() & 0x7F);
	// TODO minute, hour, ...
	minute = ds1307_decodeBcd(i2c_master_read_next());													 //0x01

	hour = ds1307_decodeBcd(i2c_master_read_next() & 0x3F);		//0b0011 1111 -24 hour					//0x02

	weekday = ds1307_decodeBcd(i2c_master_read_next());													//0x03

	day = ds1307_decodeBcd(i2c_master_read_next());														//0x04

	month = ds1307_decodeBcd(i2c_master_read_next());													//0x05

	year = ds1307_decodeBcd(i2c_master_read_last());													//0x06

	
	// Close I2C bus
	i2c_master_close();
	
	return 0;
	
	// Close I2C bus
	i2c_master_close();
	
	return 0;
}
/******************************************************************************/


void nexttime(void){
	uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	// Set second to zero (jump is in minutes)
	second =0;

	
	// Goto next minute
	minute++;
	if (minute >= 60){
		minute = 0;
		hour++;
	} else {
		return;
	}
	
	// Check hour
	if (hour >= 24){
		hour = 0;
		day++;
	} else {
		return;
	}
	
	// Check for gap year
	// TODO done
	if ( (year % 400 == 0) || (year % 4 == 0 && year % 100 != 0)){
		days[1] = 29;
	}
	
	// Check day
	if (day > days[month]){
		day= 1;
		month++;
	}
	
	// Check month
	if (month >= 12){
		month = 1;
		year++;
	}
}
/******************************************************************************/


/**
	@brief Load 8 bit value from the EEPROM
	@return loaded value
*/

uint8_t load_value8bit(uint8_t pos){
	uint8_t value;

	/* TODO */	//done
	i2c_master_open_write(160);
	i2c_master_write(pos);
	i2c_master_open_read(161);
	value=i2c_master_read_next();
	i2c_master_read_last();
	i2c_master_close();
	
	return value;
}
/******************************************************************************/


/**
	@brief Load a 16 bit value from the EEPROM
	@return loaded value
*/

uint16_t load_value16bit(uint8_t pos){
	uint8_t highbyte, lowbyte;

	/* TODO */ //done
	i2c_master_open_write(160);
	i2c_master_write(pos);
	i2c_master_open_read(161);
	lowbyte=i2c_master_read_next();
	highbyte=i2c_master_read_next();
	i2c_master_read_last();
	i2c_master_close();
	
	return highbyte * 256 + lowbyte;
}
/******************************************************************************/

/**
	@brief Save a 8 bit value to the EEPROM
	@param tosave value to save
*/

void save_value8bit(uint8_t tosave, uint8_t pos){

	/* TODO */	//done
	if (i2c_master_open_write(160))
		return;
	i2c_master_write(pos);
	i2c_master_write(tosave);



	i2c_master_close();
	_delay_ms(10); // wait 10ms to make sure that data is written
}
/******************************************************************************/


/**
	@brief Save a 16 bit value to the EEPROM
	@param tosave value to save
*/
void save_value16bit(uint16_t tosave, uint8_t pos){
	uint8_t highbyte, lowbyte;
	
	/* TODO */	//done
	highbyte=tosave/256;
	lowbyte=tosave%256;
	if (i2c_master_open_write(160)) //address EEPROM
		return;
	i2c_master_write(pos);
	i2c_master_write(lowbyte);
	i2c_master_write(highbyte);

	i2c_master_close();
	_delay_ms(10); // wait 10ms to make sure that data is written	
}
/******************************************************************************/


/**
	@brief Read the temperature with the internal analog sensor
	@return temperature in 1/10 deg. Celsius
*/
uint16_t adc_temperature_oversample(void){
	uint8_t i;
	uint32_t sum = 0;
	
	for (i = 0; i < 128; i++){
		ADCSRA |= (1 << ADSC)| (1 << ADEN); // Start ADC
	
		while( ADCSRA & (1 << ADSC) ) // wait for ADC complete
			;
	
		sum += ADCW;
	}
	

	sum /= 32;

	// substract offset
	sum -= TEMPSENSOR_OFFSET;
	
	// 0.27 deg. Celsius per step
	sum *= 27;
	sum /= 10;
	
	return sum;
}
/******************************************************************************/

/******************************************************************************/

void log_data(void){
	// TODO
	if(E){
		lcd_clear();
		lcd_string("Memory is NOT empty");
		_delay_ms(1000);
		return;
	}
	save_value8bit(year,1) ;
	save_value8bit(month,2);
	save_value8bit(day,3) ;
	save_value8bit(hour,4);
	save_value8bit(minute,5);
	save_value8bit(second,6); 

	uint8_t n;
	uint16_t temp;
	uint8_t prevminute;
	uint8_t prevsecond;
	char str[16];

	_delay_ms(1000);
	for(n=1;n<=125;n++){
		E++;
		prevminute=minute;
		prevsecond=second;
		temp=adc_temperature_oversample();
		save_value16bit(temp,2*(n-1)+7);
		save_value8bit(n,0);
		while((minute==prevminute && second-prevsecond<5)||(prevminute!=minute && (60-prevsecond)+second<5)){
			_delay_ms(100);
			temp=adc_temperature_oversample();
			lcd_clear();
			lcd_string("Recording data");
			lcd_setcursor(0,2);
			snprintf(str,16,"n: %d, %d.%d C",n,temp/10,temp%10);
			lcd_string(str);
			if(~PINB&(1<<PB0) && ~PINB&(1<<PB1)){
				lcd_clear();
				lcd_string("STOP RECORDING");
				_delay_ms(1000);
				return;
			}
			ds1307_getTime();
		}
	}
	lcd_clear();
	lcd_string("Memory full");
	_delay_ms(1000);



}
/******************************************************************************/


void show_data(void){
	// TODO
	char str[16];
	if(E==0){
		lcd_clear();
		lcd_string("No data found!");
		_delay_ms(1000);
		return;
	}
	uint8_t n=load_value8bit(0);
	year=load_value8bit(1);
	month=load_value8bit(2);
	day=load_value8bit(3);
	hour=load_value8bit(4);
	minute=load_value8bit(5);
	second=load_value8bit(6);
	while(1){
		_delay_ms(100);
		lcd_clear();
		snprintf(str,16,"n=%d",n);
		lcd_string(str);
		lcd_setcursor(0,2);
		snprintf(str,16,"Time: %02d:%02d",hour,minute);
		lcd_string(str);
		if(~PINB&(1<<PB0)){
			_delay_ms(75);
			if(~PINB&(1<<PB1)){
				_delay_ms(75);
				E=0;
				n=0;
				break;
				while(~PINB&(1<<PB0)||~PINB&(1<<PB1)){
					_delay_ms(75);
					return;
				}
			}
			while(~PINB&(1<<PB0));
			_delay_ms(75);
			break;
		}

	}
	uint16_t temp;
	int i=1;
	while(1){
		_delay_ms(100);
		temp=load_value16bit(2*(i-1)+7);
		lcd_clear();
		snprintf(str,16,"Temp: %d.%d C",temp/10,temp%10);
		lcd_string(str);
		lcd_setcursor(0,2);
		snprintf(str,16,"Time %02d:%02d:%02d",hour,minute,second);
		lcd_string(str);
		if(~PINB&(1<<PB0)){
			_delay_ms(75);
			if(~PINB&(1<<PB1)){
				_delay_ms(75);
				E=0;
				n=0;
				while(~PINB&(1<<PB0)||~PINB&(1<<PB1));
				_delay_ms(75);
				return;
			}
			while(~PINB & (1<<PB0));
			_delay_ms(75);
			if(i==n) return;
			i++;
			nexttime();
		}
	}
}
/******************************************************************************/

/******************************************************************************/
void ds1307_setTimeNow(void){ 
	
	// Defined start time
	second = 30;
	minute = 30;
	hour = 14;
	day = 20;  
	month = 11;
	year = 22;
	weekday = 7;
	
	ds1307_setTime(); //write to set time
	}
/*******************************************************************************/


/**
	@brief Main function
	@return only a dummy to avoid a compiler warning, not used
*/
int main(void){
	uint16_t nowtemp;
	init(); 	// Function to initialise I/Os
	lcd_init(); // Function to initialise LCD display
	i2c_master_init(1, 10); // Init TWI
	ds1307_rtc(1); //run
	ds1307_setTimeNow();


	// Analog Input
	ADMUX |=(1<<MUX3);	//select channel 8-temp sensor enabled
	ADMUX |= (1<<REFS1)|(1<<REFS0);//TODO // 1.1V as reference
	ADCSRA = (1 << ADPS2)| (1 << ADPS1); // ADC Prescale by 64
	ADCSRA |= (1 << ADSC)| (1 << ADEN); // Start first conversion (dummy read)


	// Loop forever
	for(;;){
		// Short delay
		_delay_ms(100);
		
		
		// Mesure temperature
		nowtemp = adc_temperature_oversample();
		
		
		// Load current time/date from DS1307
		// TODO
		ds1307_getTime();
		
		// Show current time/date
		display_standby(nowtemp);

		// Show recorded data
		if ( ~PINB & (1 << PB0) ){
			while ( ~PINB & (1 << PB0) ){
				_delay_ms(50);
			}
			show_data();
		}
		
		// Start Recording
		if (~PINB & (1 << PB1)){
			while ( ~PINB & (1 << PB1) ){
				_delay_ms(50);
			}
			log_data();
		}
	}

	return 0;
}
/******************************************************************************/
