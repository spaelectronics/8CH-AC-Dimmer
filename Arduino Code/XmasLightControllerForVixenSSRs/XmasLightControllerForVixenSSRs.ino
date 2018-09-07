/********************************************************
 * 
 * This is the code for Version 2 of the Christmas Lights 
 * Controller.
 *
 * This code is targetting - 32 channels with 256 dimming 
 * levels and a 50ms frame rate from Vixen.
 *
 * - Copyright Scott Shaver 2013
 * 
 ********************************************************
 *
 * The code for version 1 and 2 can be found at this URL:
 * http://code.google.com/p/arduino-christmas-lights-control-system-for-vixen/
 *
 * !!! Do NOT use the Ctrl-T code reformat tool in the Arduino IDE on this file, it screws up the macro definitions. !!!
 *
 * You are going to see a lot of "strange" things in this code.  For instance
 * places where it would seem the obvious thing to do is use a loop or a local
 * variable.  Don't be fooled, the code is the way it is for a reason, speed.
 *
 * Thanks to the folks at DoItYourselfChristmas.com for following along from the 
 * start of version one through the end of version 2. Very motivating.
 * http://doityourselfchristmas.com/forums/showthread.php?23956-Home-Brew-Lighting-System-Arduino-Mega-SainSmart-16-Channel-Relay-Board
 *
 * Thanks to the folks in the arduino forums for helping optimize this code for speed.
 * http://arduino.cc/forum/index.php/topic,158285.0.html
 *
 * Thanks to the author (Nick Gammon) of this web page for information on the interrupts:
 * http://www.gammon.com.au/forum/?id=11488
 *
 * Videos for Version 2 of the controller can be found at this URL:
 * https://www.youtube.com/watch?v=ozCosT5Ki_Y&list=PLp21wF8e4XnjCTjyARowU-Qdw3qkuTACi
 *
 * Version 2 uses:
 * 1 - Arduino Mega 2560
 * 4 - SainSmart SSR 8 channel boards
 * 1 - Vixen 2.5.0.8 software package
 * 32 - electrical outlets
 * 1 - FT232R breakout board
 * 32 - 3mm LEDs
 * 1 - 12V/20mA 4mm Green LED with Holder
 * 32 - 1k resistors
 * 32 - random cross SSRs, G3MC-202PL-DC5 (these replace the zero-cross SSRs that come on the SainSmart boards)
 * 1 - H11AA1 DIP forthe zero-cross detection (driving the interrupts on the Arduino)
 * 1 - 5v Enercell 5V/3.6 Amp AC Adapter with USB
 * A boat load of ribbon cabel and 14 guage electrical wire.
 * 1 - Whole House FM Transmitter http://www.amazon.com/Whole-House-FM-Transmitter-2-0/dp/B003FNQHOW/ref=sr_1_1?ie=UTF8&qid=136457053&sr=8-1&keywords=Whole+House+FM+Transmitter+2.0
 *
 ********************************************************
 *
 * Videos for Version 1 of the controller can be found at this URL:
 * http://www.youtube.com/watch?v=B5Wsovc2TSg&list=PLp21wF8e4XnjNSQHJcnvNSDdb1wsPOfHi 
 *
 * Version 1 used:
 * 1 - Arduino Mega 2560
 * 1 - SainSmart 16-Channel 12V Relay Module
 * 1 - Vixen 2.5.0.8 software package
 * 16 - electrical outlets
 * 1 - FT232R breakout board
 * 16 - 3mm LEDs
 * 16 - 1k resistors
 * 1 - 32v HP power supply
 * A boat load of ribbon cabel and 14 guage electrical wire.
 *
 * It attempted to use the PWM pins on the Arduino to do dimming. The relays were
 * mechanical not solid state so this didn't work. It was REALLY noisey and the
 * dimming was more or less just a flashing effect.  Plus the mechanical relays
 * would have eventually worn out.
 *
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Note: One instruction cycle is 62.5 nanoseconds (16 cycles per microsecond)
 ********************************************************/

#include <avr/power.h>

// comment this out if AC is 50Hz, uncomment if AC is 60Hz
#define HERTZ60 true             

#define CHANNEL_COUNT 32         // 32 channels to read from vixen
#define CHANNEL_PIN_START 22     // 32 channels, assuming all are in order, start at what pin

#define DIMMING_LEVELS 256  // the number of levels of dimming we will get from Vixen 0-255

// Ticks Per Half AC Cycle
#ifdef HERTZ60
#define TICK_HALF_CYCLE 133333
#else
#define TICK_HALF_CYCLE 160000
#endif

// This is the number of clock cycles the timer will cause an interrupt at.
#define TIMER_CYCLE_COUNT (int)((float)(TICK_HALF_CYCLE)/(float)(DIMMING_LEVELS-1))

#define ZCD_INT 0 // use interrupt 0 for zero cross detection
#define ZCD_PIN 2 // interrupt 0 is on pin 2

// Which pins is the Random/Vixen mode switch using
#define RANDOM_MODE_PININ 6
#define RANDOM_MODE_PINOUT 7
#define RANDOM_MODE_SPEED 10

// speed for the comm port for talking with vixen, this is the comm
// port on the FT232R breakout board. 57600 seems to be the max reliable
// speed I can get.
// TODO: try again at higher speeds
#define VIXEN_COM_SPEED 57600

// speed for talking with the serial monitor in the IDE, this is the
// comm port built into the ardunio with the USB port
#define PC_COM_SPEED 57600

// comment this out for "production" execution. This will keep us from
// spending time sending message text to the PC for debugging.
// warning: turning this on and entring Vixen mode while using the
// serial monitor AND having all of the code in the ISRs will probably
// hang your IDE.
//#define PC_COM_ON true 

// this just lets us know if we are switching from random mode to vixen mode
boolean startingVixenMode = true;

/********************************************************
 * 
 * Volatile directs the compiler to load the variable 
 * from RAM and not from a storage register. While this 
 * ensures you are always reading the correct value it 
 * also slows things down.
 *
 * Let's experiment with volatile on these vars, we may 
 * not REALLY need to have them loaded on each access.
 * 
 ********************************************************/
volatile unsigned int tickCounter = 0;             // track number of clock interrupts since last ZC detection, this needs to be pretty accurate, keep as volatile
volatile byte channelValue[CHANNEL_COUNT];         // channel values 0-255 vixen values stored here - TODO: try this as non-volatile

/********************************************************
 * 
 * Pins on the Arduino that control the various Vixen 
 * channels.  The pins for the channels are assumed to
 * in order and contiguous.
 * 
 ********************************************************/
//byte channelPins[CHANNEL_COUNT];      // channel pins on the Arduino - changed this to #defines below so the macros produce the fastest code
#define CHANNEL_PIN_1 CHANNEL_PIN_START
#define CHANNEL_PIN_2 CHANNEL_PIN_START+1
#define CHANNEL_PIN_3 CHANNEL_PIN_START+2
#define CHANNEL_PIN_4 CHANNEL_PIN_START+3
#define CHANNEL_PIN_5 CHANNEL_PIN_START+4
#define CHANNEL_PIN_6 CHANNEL_PIN_START+5
#define CHANNEL_PIN_7 CHANNEL_PIN_START+6
#define CHANNEL_PIN_8 CHANNEL_PIN_START+7
#define CHANNEL_PIN_9 CHANNEL_PIN_START+8
#define CHANNEL_PIN_10 CHANNEL_PIN_START+9
#define CHANNEL_PIN_11 CHANNEL_PIN_START+10
#define CHANNEL_PIN_12 CHANNEL_PIN_START+11
#define CHANNEL_PIN_13 CHANNEL_PIN_START+12
#define CHANNEL_PIN_14 CHANNEL_PIN_START+13
#define CHANNEL_PIN_15 CHANNEL_PIN_START+14
#define CHANNEL_PIN_16 CHANNEL_PIN_START+15
#define CHANNEL_PIN_17 CHANNEL_PIN_START+16
#define CHANNEL_PIN_18 CHANNEL_PIN_START+17
#define CHANNEL_PIN_19 CHANNEL_PIN_START+18
#define CHANNEL_PIN_20 CHANNEL_PIN_START+19
#define CHANNEL_PIN_21 CHANNEL_PIN_START+20
#define CHANNEL_PIN_22 CHANNEL_PIN_START+21
#define CHANNEL_PIN_23 CHANNEL_PIN_START+22
#define CHANNEL_PIN_24 CHANNEL_PIN_START+23
#define CHANNEL_PIN_25 CHANNEL_PIN_START+24
#define CHANNEL_PIN_26 CHANNEL_PIN_START+25
#define CHANNEL_PIN_27 CHANNEL_PIN_START+26
#define CHANNEL_PIN_28 CHANNEL_PIN_START+27
#define CHANNEL_PIN_29 CHANNEL_PIN_START+28
#define CHANNEL_PIN_30 CHANNEL_PIN_START+29
#define CHANNEL_PIN_31 CHANNEL_PIN_START+30
#define CHANNEL_PIN_32 CHANNEL_PIN_START+31

/********************************************************
 * 
 * The following vars were local to some of the methods 
 * below.  waitForVixenHeader() and readFromVixen(). I'm 
 * hoping moving them up here so they don't have to be 
 * allocated on the stack everytime the methods are 
 * executed will speed things up a tiny bit. This, 
 * however, may not be the case, the compier may be smart 
 * enough to optimize the allocation and access.
 * 
 ********************************************************/
const char *vixenDataFooter = "END";  // the header sent from Vixen before the channel data, for each frame
const char *vixenDataHeader = "STRT"; // the footer sent from Vixen after the channel data, for each frame
unsigned long time;  // used for timing during serial reads from Vixen
int index = 0;
int inByte = 0;     // used to read bytes in from Vixen
char buffer[4];     // max length of either the data header or data footer should be this buffer length

// ATmega2560 pin/port mapping (Write to PORTn to set value, read from PINn to get value, write 1 to PINn to flip pin value)
// Noting this here as we want to use port manipulation instead of digitalWrite to make things faster.
// Port                  Arduino Pin        Pin Name
// A                        D22-D29         PA0(AD0) - PA7(AD7)
// C                        D30-D37         PC7(A15) - PC0(A8)
// D                        D38             PD7(T0)
// G                        D39-D41         PG2(ALE) - PG0(WR)
// L                        D42-D49         PL7 - PL0(ICP4)
// B                        D50-D53         PB3(MISO/PCINT3) - PB0(SS/PCINT0)

/********************************************************
 * START OF FASTER DIGITAL WRITE LIBARY CODE
 * 
 * The following set of macros where grabbed from the faster digital write library code which can be found
 * here: http://sourceforge.net/projects/fastdigwrite/files/ I have renamed the digitalWrite, digitalRead and
 * pinMode macros to customDigitalWrite, customDigitalRead and customPinMode in this code. I've only included
 * the version of the macros that will work for the ArduinoMega (1280 and 2560)
 * 
 * The idea is using these macros instead of the normal function we are using direct port manipulation which
 * is MUCH faster.  The macros will do this anywhere you can hardcode the pin numbers to write to, for instance
 * customDigitalWrite(9,HIGH) will do direct port writes.  However if you give the pin number as a variable, say
 * in a loop, such as customDigitalWrite(p,HIGH), the macro uses the normal function calls which are slow.
 * Basically the pin number has to be known at compile time not at runtime.
 * 
 * The basic scheme for this was developed by Paul Stoffregen with digitalWrite.
 * extended to pinMode by John Raines and 
 * extended to digitalRead by John Raines with considerable assistance by William Westfield
 * Copyright (c) 2008-2010 PJRC.COM, LLC
 */

// Arduino Mega Pins
#define digitalPinToPortReg(P) \
(((P) >= 22 && (P) <= 29) ? &PORTA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PORTB : \
(((P) >= 30 && (P) <= 37) ? &PORTC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PORTD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PORTE : \
(((P) >= 54 && (P) <= 61) ? &PORTF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PORTG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PORTH : \
(((P) == 14 || (P) == 15) ? &PORTJ : \
(((P) >= 62 && (P) <= 69) ? &PORTK : &PORTL))))))))))

#define digitalPinToDDRReg(P) \
(((P) >= 22 && (P) <= 29) ? &DDRA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &DDRB : \
(((P) >= 30 && (P) <= 37) ? &DDRC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &DDRD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &DDRE : \
(((P) >= 54 && (P) <= 61) ? &DDRF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &DDRG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &DDRH : \
(((P) == 14 || (P) == 15) ? &DDRJ : \
(((P) >= 62 && (P) <= 69) ? &DDRK : &DDRL))))))))))

#define digitalPinToPINReg(P) \
(((P) >= 22 && (P) <= 29) ? &PINA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PINB : \
(((P) >= 30 && (P) <= 37) ? &PINC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PIND : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PINE : \
(((P) >= 54 && (P) <= 61) ? &PINF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PING : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PINH : \
(((P) == 14 || (P) == 15) ? &PINJ : \
(((P) >= 62 && (P) <= 69) ? &PINK : &PINL))))))))))

#define __digitalPinToBit(P) \
(((P) >=  7 && (P) <=  9) ? (P) - 3 : \
(((P) >= 10 && (P) <= 13) ? (P) - 6 : \
(((P) >= 22 && (P) <= 29) ? (P) - 22 : \
(((P) >= 30 && (P) <= 37) ? 37 - (P) : \
(((P) >= 39 && (P) <= 41) ? 41 - (P) : \
(((P) >= 42 && (P) <= 49) ? 49 - (P) : \
(((P) >= 50 && (P) <= 53) ? 53 - (P) : \
(((P) >= 54 && (P) <= 61) ? (P) - 54 : \
(((P) >= 62 && (P) <= 69) ? (P) - 62 : \
(((P) == 0 || (P) == 15 || (P) == 17 || (P) == 21) ? 0 : \
(((P) == 1 || (P) == 14 || (P) == 16 || (P) == 20) ? 1 : \
(((P) == 19) ? 2 : \
(((P) == 5 || (P) == 6 || (P) == 18) ? 3 : \
(((P) == 2) ? 4 : \
(((P) == 3 || (P) == 4) ? 5 : 7)))))))))))))))

// 15 PWM
#define __digitalPinToTimer(P) \
(((P) == 13 || (P) ==  4) ? &TCCR0A : \
(((P) == 11 || (P) == 12) ? &TCCR1A : \
(((P) == 10 || (P) ==  9) ? &TCCR2A : \
(((P) ==  5 || (P) ==  2 || (P) ==  3) ? &TCCR3A : \
(((P) ==  6 || (P) ==  7 || (P) ==  8) ? &TCCR4A : \
(((P) == 46 || (P) == 45 || (P) == 44) ? &TCCR5A : 0))))))

#define __digitalPinToTimerBit(P) \
(((P) == 13) ? COM0A1 : (((P) ==  4) ? COM0B1 : \
(((P) == 11) ? COM1A1 : (((P) == 12) ? COM1B1 : \
(((P) == 10) ? COM2A1 : (((P) ==  9) ? COM2B1 : \
(((P) ==  5) ? COM3A1 : (((P) ==  2) ? COM3B1 : (((P) ==  3) ? COM3C1 : \
(((P) ==  6) ? COM4A1 : (((P) ==  7) ? COM4B1 : (((P) ==  8) ? COM4C1 : \
(((P) == 46) ? COM5A1 : (((P) == 45) ? COM5B1 : COM5C1))))))))))))))

#define __atomicWrite__(A,P,V) \
if ( (int)(A) < 0x40) { \
  bitWrite(*((volatile uint8_t*)A), __digitalPinToBit(P), (V) ); \
} \
else {   \
  uint8_t register saveSreg = SREG;   \
  cli();   \
  bitWrite(*((volatile uint8_t*)A), __digitalPinToBit(P), (V) );   \
  SREG=saveSreg;   \
} 

#define customDigitalWrite(P, V) \
do {   \
  if (__builtin_constant_p(P) && __builtin_constant_p(V))   __atomicWrite__((uint8_t*) digitalPinToPortReg(P),P,V) \
else  digitalWrite((P), (V));   \
}while (0)

#define customPinMode(P, V) \
do { \
  if (__builtin_constant_p(P) && __builtin_constant_p(V)) __atomicWrite__((uint8_t*) digitalPinToDDRReg(P),P,V) \
  else pinMode((P), (V));   \
} while (0)

#define customDigitalRead(P) ( (int) _digitalReadFast2_((P)) )
#define _digitalReadFast2_(P ) \
(__builtin_constant_p(P) ) ? ( \
( bitRead(*digitalPinToPINReg(P), __digitalPinToBit(P))) ) : \
digitalRead((P))
/********************************************************
 * 
 * END OF FASTER DIGITAL WRITE LIBARY CODE
 * 
 ********************************************************/

void setup()
{
  // for random mode
  randomSeed( 614 ); // should do the same thing as this - randomSeed(analogRead(0));

#ifdef PC_COM_ON
  Serial.begin(PC_COM_SPEED);
#endif

  // set up the switch for Vixen or Random mode
  pinMode(RANDOM_MODE_PININ, INPUT);
  digitalWrite(RANDOM_MODE_PININ,HIGH); // turn on the internal pull-up resistor
  pinMode(RANDOM_MODE_PINOUT, OUTPUT);
  
  // init the value arrays
  for(int c=0;c<CHANNEL_COUNT;c++)
  {
    pinMode(CHANNEL_PIN_START + c, OUTPUT); // set the channel pin to output mode
    channelValue[c] = 255; // currently off
  }

  turnAllLightsOff(); // set the channels to off

  powerOnSelfTest(); // do a quick test of the channels

  // make sure we start out in the correct mode based on the random mode switch
  if(digitalRead(RANDOM_MODE_PININ)==LOW)
    setupRandomMode();
  else
    setupVixenMode();
}

/********************************************************
 * 
 * main program loop, just decides which mode we are in
 * Random or Vixen and makes the calls needed from there.
 * 
 ********************************************************/
void loop()
{
  if(digitalRead(RANDOM_MODE_PININ)==HIGH) // not Random mode, Vixen mode
  {  // play from Vixen mode
    if(startingVixenMode)
    {
      turnAllLightsOff();
      startingVixenMode = false; // make sure we know we are already in Vixen
      setupVixenMode();
    }
    if(readFromVixen())
    {
#ifdef PC_COM_ON
      Serial.println("read");
#endif
    }
    else // comment out this else and the line below if you don't want the lights to turn off when vixen doesn't send data because the channel values didn't change from one from to the next.
      turnAllLightsOff(); // set the channels to off
  }
  else
  { // random mode
    if(!startingVixenMode) // indicates we were just reading data from Vixen
    {
      turnAllLightsOff();
      setupRandomMode();
    }
    startingVixenMode=true;
    doRandomLights();
  }
}

/********************************************************
 * 
 * setup everything we need for dealing with Vixen mode 
 * including the ISRs
 * 
 ********************************************************/
void setupVixenMode()
{
#ifdef PC_COM_ON
  Serial.println("Setting up Vixen mode.");
#endif
  // make sure we can read from Vixen COM port
  Serial1.begin(VIXEN_COM_SPEED);

  /* Disable all of the unused peripherals. Some of these
   * peripherals may generate interrupts and slow us down.
   */
  power_adc_disable();     //Disable the Analog to Digital Converter module.
  power_spi_disable();     //Disable the Serial Peripheral Interface module.
//  power_timer0_disable();  //Disable the Timer 0 module. This kills the delay() and millis() methods
  power_timer2_disable();  //Disable the Timer 2 module.
  power_timer3_disable();  //Disable the Timer 3 module.
  power_timer4_disable();  //Disable the Timer 4 module.
  power_timer5_disable();  //Disable the Timer 5 module.
  power_twi_disable();     //Disable the Two Wire Interface module.
  power_usart2_disable();  //Disable the USART 2 module.
  power_usart3_disable();  //Disable the USART 3 module.

  turnAllLightsOff();
  noInterrupts(); // turn the interrupts off while we are setting everything up

  // attach the ISR for getting zero cross signals
  attachInterrupt(ZCD_INT, ZeroCrossDetected, CHANGE);

  // "In Clear Timer on Compare or CTC mode (WGMn3:0 = 4 or 12), the OCRnA or ICRn Register
  // are used to manipulate the counter resolution. In CTC mode the counter is cleared to zero when
  // the counter value (TCNTn) matches either the OCRnA (WGMn3:0 = mode 4) or the ICRn (WGMn3:0 = mode 12). 
  // The OCRnA or ICRn define the top value for the counter, hence also its resolution. This
  // mode allows greater control of the compare match output frequency. It also simplifies the opera-
  // tion of counting external events."

  // Timer/Counter Control Register A
  // Clear this register we don't need the bits set. = normal operation
  TCCR1A = 0;

  // Timer/Counter Control Register B
  // (Clock Select Bit - CSn0) set the timer prescaler to 1 = timer increments every clock cycle 
  // Set CTC (Clear Timer on Compare) mode with WGMn2 (waveform generation) bit 
  // makes this a counter that counts from 0 to the value of OCR1A 
  // (the value of Output Compare Register 1 A). Do not send PWM signals
  // to a pin. 
  TCCR1B = _BV(WGM12) | _BV(CS10); 

  // Clear the timer counter register
  TCNT1 = 0; 

  // Output Compare Register 1 A
  // Set it to TIMER_CYCLE_COUNT so we get an interrupt every TIMER_CYCLE_COUNT clock cycles.
  //
  // Interrupt info from - http://www.gammon.com.au/forum/?id=11488
  // "...an ISR using the ISR define will take you 2.625 uS to execute, 
  // plus whatever the code itself does." (42 cycles-ish - 0.625 uS per cycle)
  OCR1A = TIMER_CYCLE_COUNT;

  // enable timer/counter 1 Output Compare A Match interrupt
  TIMSK1 = _BV(OCIE1A);

  interrupts(); // okay, turn the interrupts on, let's go!
}

/********************************************************
 * 
 * setup everything we need for dealing with random mode 
 * and remove anything we use in Vixen mode that isn't needed
 * 
 ********************************************************/
void setupRandomMode()
{
#ifdef PC_COM_ON
  Serial.println("Setting up Random mode.");
#endif
  // stop reading from Vixen COM port
  Serial1.end();

  /* Disable all of the unused peripherals. Some of these
   * peripherals may generate interrupts and slow us down.
   */
  power_adc_disable();     //Disable the Analog to Digital Converter module.
  power_spi_disable();     //Disable the Serial Peripheral Interface module.
//  power_timer0_disable();  //Disable the Timer 0 module. This kills the delay() and millis() methods
  power_timer2_disable();  //Disable the Timer 2 module.
  power_timer3_disable();  //Disable the Timer 3 module.
  power_timer4_disable();  //Disable the Timer 4 module.
  power_timer5_disable();  //Disable the Timer 5 module.
  power_twi_disable();     //Disable the Two Wire Interface module.
  power_usart2_disable();  //Disable the USART 2 module.
  power_usart3_disable();  //Disable the USART 3 module.

  turnAllLightsOff();
  noInterrupts(); // turn the interrupts off while we are setting everything up

  // attach the ISR for getting zero cross signals
  attachInterrupt(ZCD_INT, ZeroCrossDetected, CHANGE);

  // "In Clear Timer on Compare or CTC mode (WGMn3:0 = 4 or 12), the OCRnA or ICRn Register
  // are used to manipulate the counter resolution. In CTC mode the counter is cleared to zero when
  // the counter value (TCNTn) matches either the OCRnA (WGMn3:0 = mode 4) or the ICRn (WGMn3:0 = mode 12). 
  // The OCRnA or ICRn define the top value for the counter, hence also its resolution. This
  // mode allows greater control of the compare match output frequency. It also simplifies the opera-
  // tion of counting external events."

  // Timer/Counter Control Register A
  // Clear this register we don't need the bits set. = normal operation
  TCCR1A = 0;

  // Timer/Counter Control Register B
  // (Clock Select Bit - CSn0) set the timer prescaler to 1 = timer increments every clock cycle 
  // Set CTC (Clear Timer on Compare) mode with WGMn2 (waveform generation) bit 
  // makes this a counter that counts from 0 to the value of OCR1A 
  // (the value of Output Compare Register 1 A). Do not send PWM signals
  // to a pin. 
  TCCR1B = _BV(WGM12) | _BV(CS10); 

  // Clear the timer counter register
  TCNT1 = 0; 

  // Output Compare Register 1 A
  // Set it to TIMER_CYCLE_COUNT so we get an interrupt every TIMER_CYCLE_COUNT clock cycles.
  //
  // Interrupt info from - http://www.gammon.com.au/forum/?id=11488
  // "...an ISR using the ISR define will take you 2.625 uS to execute, 
  // plus whatever the code itself does." (42 cycles-ish - 0.625 uS per cycle)
  OCR1A = TIMER_CYCLE_COUNT;

  // enable timer/counter 1 Output Compare A Match interrupt
  TIMSK1 = _BV(OCIE1A);

  interrupts(); // okay, turn the interrupts on, let's go!
}

/********************************************************
 * 
 * This is just a quick test of each channel.  One is turned on for a
 * short period of time and then turned off.  We move through all of the 
 * channels in this manner.
 * 
 ********************************************************/
void powerOnSelfTest()
{
#ifdef PC_COM_ON
  Serial.println("Power on self test running.");
#endif
  turnAllLightsOn();
  delay(250);
  turnAllLightsOff();  
#ifdef PC_COM_ON
    Serial.print("Channel: ");
#endif
  for(int channelIndex=0;channelIndex<CHANNEL_COUNT;channelIndex++){
#ifdef PC_COM_ON
    Serial.print(channelIndex+1,DEC); // print channel number not pin number
    Serial.print(" "); // print channel number not pin number
#endif
    digitalWrite(channelIndex+CHANNEL_PIN_START, HIGH); // turn on one channel at a time
    delay(250); // wait .5 seconds
    digitalWrite(channelIndex+CHANNEL_PIN_START, LOW); // turn it back off
  }
#ifdef PC_COM_ON
    Serial.println("\nPOST Done.");
#endif
  turnAllLightsOff();
}

/********************************************************
 * 
 * Turn them all off, this function doesn't need to be fast, 
 * DON'T USE THIS ANYWHERE WHEN PROCESSING DATA FROM VIXEN!!!
 * 
 ********************************************************/
void turnAllLightsOff()
{
  for(int c=0;c<CHANNEL_COUNT;c++)
  {
    digitalWrite(c+CHANNEL_PIN_START,LOW);
    channelValue[c] = 255; // currently off
  }
}

/********************************************************
 * 
 * Turn them all on, this function doesn't need to be fast, 
 * DON'T USE THIS ANYWHERE WHEN PROCESSING DATA FROM VIXEN!!!
 * 
 ********************************************************/
void turnAllLightsOn()
{
  for(int c=0;c<CHANNEL_COUNT;c++)
  {
    digitalWrite(c+CHANNEL_PIN_START,HIGH);
    channelValue[c] = 0; // currently on
  }
}

/********************************************************
 * 
 * Random mode, dimming. Randomly sets each channel to
 * a value and then dims until all channels are off.
 * RANDOM_MODE_SPEED determines how fast the dimming occurs.
 * 
 ********************************************************/
void doRandomLights()
{
#ifdef PC_COM_ON
  Serial.println("Writing random values.");
#endif
  for(int channelIndex=0;channelIndex<CHANNEL_COUNT;channelIndex++)
  {
    int randNumber = random(255);
    if(channelValue[channelIndex]==255 && random(255)>240)
      channelValue[channelIndex] = randNumber;
#ifdef PC_COM_ON
    Serial.print(randNumber, DEC);
    Serial.print(",");
#endif
  }
#ifdef PC_COM_ON
  Serial.println("");
#endif
  for(int channelIndex=0;channelIndex<CHANNEL_COUNT;channelIndex++)
  {
    if(channelValue[channelIndex]<255)
      channelValue[channelIndex]++;
  }
  delay(RANDOM_MODE_SPEED);
}

/********************************************************
 * 
 * Read in data from the Vixen COM port.
 * Wait for Vixen data header and then...
 * Stay in here for one second (at most without having 
 * read data) waiting for the vixen channel data.
 * If we go over a second return so the main loop gets
 * another chance to run.
 * 
 * return true if read in the channel data, otherwise false
 ********************************************************/
boolean readFromVixen()
{
#ifdef PC_COM_ON
  Serial.println("Waiting for data from Vixen.");
#endif

  if(waitForVixenHeader()) // we got the data header now read the channel data
  {
    // these were made globals I'm not sure this actually reduces the time this method takes.
    // hoping making them global will avoid overhead of allocation in this method.
    time = millis();
    index=0;

    while (index<CHANNEL_COUNT)
    {
      inByte = Serial1.read();
      if(inByte==-1)
      {
        if(millis()-time>1000) // we haven't read anything in a second
        {
#ifdef PC_COM_ON
          Serial.println("Skipping read of Vixen data due to timeout.");
          Serial.println("");
#endif
          return false;
        }
        continue;
      }
      // we read something update the time and channel value
      time = millis();
      channelValue[index++] = map(inByte,0,DIMMING_LEVELS,DIMMING_LEVELS-1,0); // save the channel value, reverse it 0-255 becomes 255-0
    }

    // we've read in the channel data now read in the data footer
    index=0;
    while (index<3 /*length of footer*/)
    {
      time = millis();  
      inByte = Serial1.read();
      if(inByte==-1)
      {
        if(millis()-time>1000) // we haven't read anything in a second
        {
#ifdef PC_COM_ON
          Serial.println("Skipping read of Vixen data footer due to timeout.");
          Serial.println("");
#endif
          return false;
        }
        continue;
      }
      // we read something update the time and index
      time = millis();
      index++;
    }
#ifdef PC_COM_ON
    Serial.println(vixenDataFooter);
    Serial.println("");
#endif
    return true;
  }
  return false;
}

/********************************************************
 * 
 * Look for the channel data header coming in from Vixen, 
 * once read we can then read the channel data for the
 * current frame of the sequence.
 *
 * Stay in here for one second (at most without having 
 * read data) waiting for the vixen data header.
 * If we go over a second return so the main loop gets 
 * another chance to run. Return true if we read the header, 
 * false otherwise.
 * 
 ********************************************************/
boolean waitForVixenHeader()
{
#ifdef PC_COM_ON
  Serial.println("Waiting for Vixen data header.");
#endif
  // these were made globals I'm not sure this actually reduces the time this method takes.
  // hoping making them global will avoid overhead of allocation in this method.
  index = 0;
  time = millis();

  while (true) 
  {
    inByte = Serial1.read();
    if(inByte==-1)
    {
      if(millis()-time>1000) // we haven't read anything in a second
      {
#ifdef PC_COM_ON
        Serial.println("Skipping read of Vixen data header due to timeout.");
        Serial.println("");
#endif
        return false;
      }
      continue;
    }
    // we read something update the time, buffer and index
    time = millis();
    buffer[index] = inByte;
    if(buffer[index]!=vixenDataHeader[index]) // not the right header sequence restart
      index=-1;
    index++;
    if(index==4 /*length of header*/)
    {
#ifdef PC_COM_ON
      Serial.println(vixenDataHeader);
#endif
      return true;
    }
  }
  // no need for a return we can't get here
}

/********************************************************
 * 
 * ISR executed when we detect Zero Cross. Reset the 
 * tickCounter to 0 after every zero cross.  Turn off 
 * each channel.
 *
 * 82 cycles (5.125 uS in total) as overhead plus the 
 * code in this method.
 * 
 ********************************************************/
void ZeroCrossDetected()
{
  // reset the tick counter to zero at the start of the half AC cycle
  tickCounter = 0; 

  // turn channels off immediately after zero cross here
  customDigitalWrite(CHANNEL_PIN_1, LOW);
  customDigitalWrite(CHANNEL_PIN_2, LOW);
  customDigitalWrite(CHANNEL_PIN_3, LOW);
  customDigitalWrite(CHANNEL_PIN_4, LOW);
  customDigitalWrite(CHANNEL_PIN_5, LOW);
  customDigitalWrite(CHANNEL_PIN_6, LOW);
  customDigitalWrite(CHANNEL_PIN_7, LOW);
  customDigitalWrite(CHANNEL_PIN_8, LOW);
  customDigitalWrite(CHANNEL_PIN_9, LOW);
  customDigitalWrite(CHANNEL_PIN_10, LOW);
  customDigitalWrite(CHANNEL_PIN_11, LOW);
  customDigitalWrite(CHANNEL_PIN_12, LOW);
  customDigitalWrite(CHANNEL_PIN_13, LOW);
  customDigitalWrite(CHANNEL_PIN_14, LOW);
  customDigitalWrite(CHANNEL_PIN_15, LOW);
  customDigitalWrite(CHANNEL_PIN_16, LOW);
  customDigitalWrite(CHANNEL_PIN_17, LOW);
  customDigitalWrite(CHANNEL_PIN_18, LOW);
  customDigitalWrite(CHANNEL_PIN_19, LOW);
  customDigitalWrite(CHANNEL_PIN_20, LOW);
  customDigitalWrite(CHANNEL_PIN_21, LOW);
  customDigitalWrite(CHANNEL_PIN_22, LOW);
  customDigitalWrite(CHANNEL_PIN_23, LOW);
  customDigitalWrite(CHANNEL_PIN_24, LOW);
  customDigitalWrite(CHANNEL_PIN_25, LOW);
  customDigitalWrite(CHANNEL_PIN_26, LOW);
  customDigitalWrite(CHANNEL_PIN_27, LOW);
  customDigitalWrite(CHANNEL_PIN_28, LOW);
  customDigitalWrite(CHANNEL_PIN_29, LOW);
  customDigitalWrite(CHANNEL_PIN_30, LOW);
  customDigitalWrite(CHANNEL_PIN_31, LOW);
  customDigitalWrite(CHANNEL_PIN_32, LOW);
}

/********************************************************
 * 
 * ISR executed everytime Timer/Counter1 counts to the value set in
 * Output Compare Register A (OCR1A), TIMER_CYCLE_COUNT.
 *
 * This is keeping track of the number of counter interrupts
 * that have occurred since the zero cross was detected and 
 * it turns on channels that should be turned on.
 *
 * Interrupt info from - http://www.gammon.com.au/forum/?id=11488
 * "...an ISR using the ISR define will take you 2.625 uS to execute, 
 * plus whatever the code itself does." (42 cycles-ish)
 * 
 ********************************************************/
ISR(TIMER1_COMPA_vect)
{
  register uint16_t tick = tickCounter;
  // the lower the channel value the faster we want to turn the channel on.
  // if the channel value is 0 (full bright) and this is the first interrupt (tick=0) after 
  // the ZC we want the channel on now
  if(channelValue[0] <= tick) customDigitalWrite(CHANNEL_PIN_1, HIGH); // turn the channel on here
  if(channelValue[1] <= tick) customDigitalWrite(CHANNEL_PIN_2, HIGH); // turn the channel on here
  if(channelValue[2] <= tick) customDigitalWrite(CHANNEL_PIN_3, HIGH); // turn the channel on here
  if(channelValue[3] <= tick) customDigitalWrite(CHANNEL_PIN_4, HIGH); // turn the channel on here
  if(channelValue[4] <= tick) customDigitalWrite(CHANNEL_PIN_5, HIGH); // turn the channel on here
  if(channelValue[5] <= tick) customDigitalWrite(CHANNEL_PIN_6, HIGH); // turn the channel on here
  if(channelValue[6] <= tick) customDigitalWrite(CHANNEL_PIN_7, HIGH); // turn the channel on here
  if(channelValue[7] <= tick) customDigitalWrite(CHANNEL_PIN_8, HIGH); // turn the channel on here
  if(channelValue[8] <= tick) customDigitalWrite(CHANNEL_PIN_9, HIGH); // turn the channel on here
  if(channelValue[9] <= tick) customDigitalWrite(CHANNEL_PIN_10, HIGH); // turn the channel on here
  if(channelValue[10] <= tick) customDigitalWrite(CHANNEL_PIN_11, HIGH); // turn the channel on here
  if(channelValue[11] <= tick) customDigitalWrite(CHANNEL_PIN_12, HIGH); // turn the channel on here
  if(channelValue[12] <= tick) customDigitalWrite(CHANNEL_PIN_13, HIGH); // turn the channel on here
  if(channelValue[13] <= tick) customDigitalWrite(CHANNEL_PIN_14, HIGH); // turn the channel on here
  if(channelValue[14] <= tick) customDigitalWrite(CHANNEL_PIN_15, HIGH); // turn the channel on here
  if(channelValue[15] <= tick) customDigitalWrite(CHANNEL_PIN_16, HIGH); // turn the channel on here
  if(channelValue[16] <= tick) customDigitalWrite(CHANNEL_PIN_17, HIGH); // turn the channel on here
  if(channelValue[17] <= tick) customDigitalWrite(CHANNEL_PIN_18, HIGH); // turn the channel on here
  if(channelValue[18] <= tick) customDigitalWrite(CHANNEL_PIN_19, HIGH); // turn the channel on here
  if(channelValue[19] <= tick) customDigitalWrite(CHANNEL_PIN_20, HIGH); // turn the channel on here
  if(channelValue[20] <= tick) customDigitalWrite(CHANNEL_PIN_21, HIGH); // turn the channel on here
  if(channelValue[21] <= tick) customDigitalWrite(CHANNEL_PIN_22, HIGH); // turn the channel on here
  if(channelValue[22] <= tick) customDigitalWrite(CHANNEL_PIN_23, HIGH); // turn the channel on here
  if(channelValue[23] <= tick) customDigitalWrite(CHANNEL_PIN_24, HIGH); // turn the channel on here
  if(channelValue[24] <= tick) customDigitalWrite(CHANNEL_PIN_25, HIGH); // turn the channel on here
  if(channelValue[25] <= tick) customDigitalWrite(CHANNEL_PIN_26, HIGH); // turn the channel on here
  if(channelValue[26] <= tick) customDigitalWrite(CHANNEL_PIN_27, HIGH); // turn the channel on here
  if(channelValue[27] <= tick) customDigitalWrite(CHANNEL_PIN_28, HIGH); // turn the channel on here
  if(channelValue[28] <= tick) customDigitalWrite(CHANNEL_PIN_29, HIGH); // turn the channel on here
  if(channelValue[29] <= tick) customDigitalWrite(CHANNEL_PIN_30, HIGH); // turn the channel on here
  if(channelValue[30] <= tick) customDigitalWrite(CHANNEL_PIN_31, HIGH); // turn the channel on here
  if(channelValue[31] <= tick) customDigitalWrite(CHANNEL_PIN_32, HIGH); // turn the channel on here

  // increase tick counter by one
  tickCounter++; 
}

