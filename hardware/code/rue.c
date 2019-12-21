/*

quad servo loop controller by rue_mohr

Run on 16Mhz avr
9600 N81


-- added PB0 as ascii replied ID request
-- corrected position report code
-- corrected current report code


TODO ideas:

- PID 
- fix offset id reporting to proper 3 ascii digiits
- self correcting polarity (use dERROR/drive?)



*/
/*
 +-------------------------------------+
 |                          Sig Vd Gnd |
 |  +---------+   5V O     PB0 [o o o] | baud test input
 |  | 7805  O |   Vd O     PB1 [o o o] | 
 |  +---------+   V+ .     PB2 [o o o] | 
 |                         PB3 [o o o] | chan0 pwm
 |                         PB4 [o o o] | chan0 dir
 |                         PB5 [o o o] | chan1 dir
 |                         PB6 [o o o] | chan2 dir
 |                         PB7 [o o o] | chan3 dir
 |                         PA0 [o o o] | chan0 current sense
 |                         PA1 [o o o] | chan0 feedback voltage
 |        +----------+     PA2 [o o o] | chan1 current sense
 |        |O         |     PA3 [o o o] | chan1 feedback voltage
 |        |          |     PA4 [o o o] | chan2 current sense
 |        |          |     PA5 [o o o] | chan2 feedback voltage
 |        |          |     PA6 [o o o] | chan3 current sense
 |        |          |     PA7 [o o o] | chan3 feedback voltage
 |        |          |     PC7 [o o o] | chan3 status -
 |        |          |     PC6 [o o o] | chan3 status +
 |        |          |     PC5 [o o o] | chan2 status -
 |        | ATMEGA32 |     PC4 [o o o] | chan2 status +
 |        |          |     PC3 [o o o] | chan1 status -
 |        |          |     PC2 [o o o] | chan1 status +
 |        |          |     PC1 [o o o] | chan0 status -
 |        |          |     PC0 [o o o] | chan0 status +
 |        |          |     PD7 [o o o] | chan3 pwm
 |        |          |     PD2 [o o o] |
 |        |          |     PD3 [o o o] |
 |        |          |     PD4 [o o o] | chan2 pwm
 |        |          |     PD5 [o o o] | chan1 pwm
 |        +----------+     PD6 [o o o] | Debug
 |      E.D.S BABYBOARD III            |
 +-------------------------------------+


*/
/*
Servo Recieves:

 [0][c3][c2][c1][c0][v2][v1][v0]
 [1][v9][v8][v7][v6][v5][v4][v3]
 
 c0-c3 are command
 v0-v9 are value
 
 Commands:
 0  listen (servo number) 256 = all                      {always obey command} // sticks through listen once
 1  ignore (servo number) 256 = all                      {always obey command} // overrides listen once
 2  One Time listen (servo number)                       {always obey command}
 3  set flags (flags) (+toggle debug)                    { bitwise obey }
    0 enguage cached position                              {always obey command}
    1 turn servo on                                        {obey if listening}
    2 turn servo off                                       {obey if listening}
    3 set cmdpos to curpos                                 {obey if listening}
 4  set servo position (position)                        {obey if listening}
 5  set cached position (position)                       {obey if listening}
 6 get servo current  (servo number)                    {servo number} 
 7 get servo position (servo number)                    {servo number} 
 8 send device model  (servo number)                    {servo number}

 
Servo Sends:

 if the ID line is put low, the controller sends, in ascii, the base channel address.

 [0][p3][p2][p1][p0][v2][v1][v0]
 [1][v9][v8][v7][v6][v5][v4][v3] 
 
 p0-p3 are paramiter number
 v0-v9 are value
 
 Paramiter numbers:
 0 servo position (returns position sensor value)
 1 servo current 
 2 device model   (returns 10 bit device model number)

*/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"


#define Version 5

#define ReplyPos 0
#define ReplyCur 1
#define ReplyVer 2

#define PackBits(V,P) (((V << 5) & 0xFF00)|(V & 0x07)|0x8000|(P<<3))

#define IsHigh(BIT, PORT)    ((PORT & (1<<BIT)) != 0)
#define IsLow(BIT, PORT)     ((PORT & (1<<BIT)) == 0)
#define SetBit(BIT, PORT)     PORT |= (1<<BIT)
#define ClearBit(BIT, PORT)   PORT &= ~(1<<BIT)

#define Kp  1
#define Ki  4

#define ABS(a)                ((a) < 0 ? -(a) : (a))
#define SIGN(x)               (x)==0?0:(x)>0?1:-1
#define NOP()                 asm volatile ("nop"::)

// limit v to within low (l) and high (h) limits
#define limit(v, l, h)        ((v) > (h)) ? (h) : ((v) < (l)) ? (l) : (v)

//check bounds
#define inBounds(v, l, h)        ((v) > (h)) ? (0) : ((v) < (l)) ? (0) : (1)

// write value to bit on port
#define WriteBit(b, p, v)    p = (p & ~(1<<b))|((v&1)<<b)

 // debug pin
#define debugH()      SetBit(6, PORTD)
#define debugL()      ClearBit(6, PORTD)
#define debugToggle() PORTD ^= (1<<PD6)


#define pwm0on()  SetBit( COM01, TCCR0)
#define pwm0off() ClearBit( COM01, TCCR0)
#define pwm0stat() IsHigh( COM01, TCCR0)

#define pwm1on()  SetBit( COM1A1, TCCR1A)
#define pwm1off() ClearBit( COM1A1, TCCR1A)
#define pwm1stat() IsHigh( COM1A1, TCCR1A)

#define pwm2on()  SetBit( COM1B1, TCCR1A)
#define pwm2off() ClearBit( COM1B1, TCCR1A)
#define pwm2stat() IsHigh( COM1B1, TCCR1A)

#define pwm3on()  SetBit( COM21, TCCR2)
#define pwm3off() ClearBit( COM21, TCCR2)
#define pwm3stat() IsHigh( COM21, TCCR2)

#define IDRequest() IsLow( 0, PORTB)


#define Status0Red()   SetBit(1, PORTC);   ClearBit(0, PORTC)
#define Status0Green() SetBit(0, PORTC);   ClearBit(1, PORTC)
#define Status0Off()   ClearBit(0, PORTC); ClearBit(1, PORTC)
#define Status0Tog()   PORTC ^=(3<<PC0);

#define Status1Red()   SetBit(3, PORTC);   ClearBit(2, PORTC)
#define Status1Green() SetBit(2, PORTC);   ClearBit(3, PORTC)
#define Status1Off()   ClearBit(2, PORTC); ClearBit(3, PORTC)
#define Status1Tog()   PORTC ^= (3<<PC2);

#define Status2Red()   SetBit(5, PORTC);   ClearBit(4, PORTC)
#define Status2Green() SetBit(4, PORTC);   ClearBit(5, PORTC)
#define Status2Off()   ClearBit(4, PORTC); ClearBit(5, PORTC)
#define Status2Tog()   PORTC ^= (3<<PC4);

#define Status3Red()   SetBit(7, PORTC);   ClearBit(6, PORTC)
#define Status3Green() SetBit(6, PORTC);   ClearBit(7, PORTC)
#define Status3Off()   ClearBit(6, PORTC); ClearBit(7, PORTC)
#define Status3Tog()   PORTC ^= (3<<PC6);


#define OUTPUT   1
#define INPUT    0

#define DIR0BIT  4
#define DIR1BIT  5
#define DIR2BIT  6
#define DIR3BIT  7

#define DIR0PORT PORTB
#define DIR1PORT PORTB
#define DIR2PORT PORTB
#define DIR3PORT PORTB

// integrator antiwindup threshold

#define INTTHRESH 220

// -----------------  servo id numbers ----------------------

#define IDOFF 0
//#define IDOFF 4
//#define IDOFF 8
//#define IDOFF 12
//#define IDOFF 16
//#define IDOFF 20

#define ID0  (0+IDOFF)
#define ID1  (1+IDOFF)
#define ID2  (2+IDOFF)
#define ID3  (3+IDOFF)

#define pwm0  OCR0  
#define pwm1  OCR1A
#define pwm2  OCR1B 
#define pwm3  OCR2  

typedef enum servoFlags_e {
   ENGCACHE = 0,
   SERVOON  ,
   SERVOOFF , 
   CMD2CUR  
} servoFlags_t;
/*****************************| VARIABLES |********************************/
 
volatile int AdcValues[8];
volatile unsigned char p0, p1, p2, p3;
volatile unsigned char calcWait;

unsigned char pwmcount;
unsigned char txbuff;
unsigned char listen[4];
unsigned char power[4];
unsigned int cmdpos[4];
unsigned int cachepos[4];
 
/************************| FUNCTION DECLARATIONS |*************************/
 
void AnalogInit (void);
void pwmInit(void) ;
int  Analog (int n);
unsigned int dobyte(char data) ;
unsigned int servoCmd(unsigned int command, unsigned int argument) ;


/****************************| CODE... |***********************************/

int main (void) {
 
  int error;
  int In[4];
  unsigned int txtemp;

  // set up directions 
  DDRA = (INPUT << PA0 | INPUT << PA1 |INPUT << PA2 |INPUT << PA3 |INPUT << PA4 |INPUT << PA5 |INPUT << PA6 |INPUT << PA7);
  DDRB = (INPUT << PB0 | INPUT << PB1 |INPUT << PB2 |OUTPUT << PB3 |OUTPUT << PB4 |OUTPUT << PB5 |OUTPUT << PB6 |OUTPUT << PB7);
  DDRC = (OUTPUT << PC0 | OUTPUT << PC1 |OUTPUT << PC2 |OUTPUT << PC3 |OUTPUT << PC4 |OUTPUT << PC5 |OUTPUT << PC6 |OUTPUT << PC7);
  DDRD = (INPUT << PD0 | INPUT << PD1 |INPUT << PD2 |INPUT << PD3 |OUTPUT << PD4 |OUTPUT << PD5 |OUTPUT << PD6 |OUTPUT << PD7);        

  PORTB = 0x01; // pullup on baud test

  pwm0 = 0;
  pwm1 = 0;
  pwm2 = 0;
  pwm3 = 0;
  
  pwm0off();
  pwm1off();
  pwm2off();
  pwm3off();
  
  Status0Red();
  Status1Red();
  Status2Red();
  Status3Red();
  
  
  USART_Init( 103 ); // 9600 @ 16Mhz
  AnalogInit();
  pwmInit();
  In[0]    = 0; In[1]    = 0; In[2]    = 0; In[3]    = 0; // reset Integrators
  
  // turn interrupts on
  sei();
  
  calcWait = 0;
  
  // calculate servo loops
  while(1){
    // while we wait to be cleared for recalculating, check the uart
    while(calcWait){
    
      // check for uart byte
      if ( (UCSRA & (1<<RXC)) ) {  
         if ((txtemp = dobyte(USART_Receive())) != 0) {
           USART_Transmit( txtemp & 0x00FF );
           txbuff = (txtemp >> 8);
         }  
      } 
      
      if ( (UCSRA & (1<<UDRE)) ) {
        if (txbuff != 0) {
          USART_Transmit( txbuff );
          txbuff = 0;
        } else { // check for id request
          if (IDRequest())  USART_Transmit( '0'+IDOFF );           
        }
      }
      
      /* update servo loop status leds
         yellow = off
         red    = overcurrent
         off    = error greater than thresh
         green  = ok         
      */
      
      if (0) {
      } else if (!pwm0stat()) {  // channel off
        Status0Tog();
      } else if ((AdcValues[0]>768)&(AdcValues[0]<256)) { // Overload (-3A) 0  (-1.5A)256    (0A)512   (1.5A)768  (3A) 1023
        Status0Red();
      } else if (pwm0 > 64) {  // piles of error yet
        Status0Off();
      } else {                   // ok
        Status0Green();
      }
      
      if (0) {
      } else if (!pwm1stat()) {  // channel off
        Status1Tog();
      } else if ((AdcValues[2]>768)&(AdcValues[2]<256)) { // Overload (-3A) 0  (-1.5A)256    (0A)512   (1.5A)768  (3A) 1023
        Status1Red();
      } else if (pwm1 > 64) {  // piles of error yet
        Status1Off();
      } else {                   // ok
        Status1Green();
      }
      
      if (0) {
      } else if (!pwm2stat()) {  // channel off
        Status2Tog();
      } else if ((AdcValues[4]>768)&(AdcValues[4]<256)) { // Overload (-3A) 0  (-1.5A)256    (0A)512   (1.5A)768  (3A) 1023
        Status2Red();
      } else if (pwm2 > 64) {  // piles of error yet
        Status2Off();
      } else {                   // ok
        Status2Green();
      }
      
      if (0) {
      } else if (!pwm3stat()) {  // channel off
        Status3Tog();
      } else if ((AdcValues[6]>768)&(AdcValues[6]<256)) { // Overload (-3A) 0  (-1.5A)256    (0A)512   (1.5A)768  (3A) 1023
        Status3Red();
      } else if (pwm3 > 64) {  // piles of error yet
        Status3Off();
      } else {                   // ok
        Status3Green();
      }
      
      
    }  
   
   
    /* do drive calculations
    
   http://www.ctc-control.com/customer/elearning/servotut/adjus.asp
   
   error      = setpoint - actual_position
   integral   = integral + (error*dt)
   derivative = (error - previous_error)/dt
   output     = (Kp*error) + (Ki*integral) - (Kd*derivative)
   previous_error = error
   
   <Tom_L> output = (p->Kp * Perror + p->Kd * (Perror - p->PrevErr) + p->Ki * p->Ierror)/p->Ko;
  <Tom_L> Kp = 30; Kd = 10; Ki = 2; Ko = 15;
   
   */
   
    error = cmdpos[0]-AdcValues[1];    
    if (ABS(error) < INTTHRESH) {  In[0] = limit(In[0] + error, -INTTHRESH, INTTHRESH); } 
    else                        {  In[0] = 0;                                           }    
    error = (Kp*error)+(In[0]/Ki);
    pwm0 = limit(ABS(error), 0, 255);            
    if (error > 0)     SetBit(DIR0BIT, DIR0PORT);
    else               ClearBit(DIR0BIT, DIR0PORT);
    
    
    error = cmdpos[1]-AdcValues[3];    
    if (ABS(error) < INTTHRESH) {  In[1] = limit(In[1] + error, -INTTHRESH, INTTHRESH); } 
    else                        {  In[1] = 0;                                           }    
    error = (Kp*error)+(In[1]/Ki);
    pwm1 = limit(ABS(error), 0, 255);       
    if (error > 0)     SetBit(DIR1BIT, DIR1PORT);
    else               ClearBit(DIR1BIT, DIR1PORT);
    
    
    error = cmdpos[2]-AdcValues[5];    
    if (ABS(error) < INTTHRESH) {  In[2] = limit(In[2] + error, -INTTHRESH, INTTHRESH); } 
    else                        {  In[2] = 0;                                           }    
    error = (Kp*error)+(In[2]/Ki);
    pwm2 = limit(ABS(error), 0, 255);    
    if (error > 0)     SetBit(DIR2BIT, DIR2PORT);
    else               ClearBit(DIR2BIT, DIR2PORT);
    
    
    error = cmdpos[3]-AdcValues[7];    
    if (ABS(error) < INTTHRESH) {  In[3] = limit(In[3] + error, -INTTHRESH, INTTHRESH); } 
    else                        {  In[3] = 0;                                           }    
    error = (Kp*error)+(In[3]/Ki);
    pwm3 = limit(ABS(error), 0, 255);     
    if (error > 0)     SetBit(DIR3BIT, DIR3PORT);
    else               ClearBit(DIR3BIT, DIR3PORT);
    
    calcWait = 1;
  }

}

//------------------------| FUNCTIONS |------------------------



unsigned int dobyte(char data) {

  static unsigned char state = 0;
  static unsigned int command;
  static unsigned int  argument;

  if (state == 0) {
    if ((data & 0x80) == 0) {
      state    = 1;
      command  = (data >> 3);
      argument = (argument & 0xFFF8) | (data & 0x07); // glue in its 0 through 2
    } 
  } else {
    state = 0;
    if ((data & 0x80) != 0) {
      argument = (argument & 0x0007) | ((data & 0x7F) << 3); //glue in bits 3 through 9
      return servoCmd(command, argument);   
    }
  } 

  return 0;
}



/*
 
 0  listen (servo number) 256 = all                      {always obey command} // sticks through listen once
 1  ignore (servo number) 256 = all                      {always obey command} // overrides listen once
 2  One Time listen (servo number)                       {always obey command}
 3  set flags (flags) (+toggle debug)                    { bitwise obey }
    0 enguage cached position                              {always obey command}
    1 turn servo on                                        {obey if listening}
    2 turn servo off                                       {obey if listening}
    3 set cmdpos to curpos                                 {obey if listening}
 4  set servo position (position)                        {obey if listening}
 5  set cached position (position)                       {obey if listening}
 6 get servo current  (servo number)                    {servo number} 
 7 get servo position (servo number)                    {servo number} 
 8 send device model  (servo number)                    {servo number}

*/


unsigned int servoCmd(unsigned int command, unsigned int argument) {

  unsigned int        reply;
  static unsigned int chainAddress = 1023;
  
  reply = 0;

  switch (command) {
     
     case 0: // listen(id)
       chainAddress = 1023 ;
       if (argument == ID0)      listen[0] |= 2;
       else if (argument == ID1) listen[1] |= 2;
       else if (argument == ID2) listen[2] |= 2;
       else if (argument == ID3) listen[3] |= 2;
       else if (argument == 256) { listen[0] |= 2; listen[1] |= 2; listen[2] |= 2; listen[3] |= 2; }      
     break;
     
     case 1: // ignore(id)
       chainAddress = 1023 ;
       if (argument == ID0)      listen[0] = 0;
       else if (argument == ID1) listen[1] = 0;
       else if (argument == ID2) listen[2] = 0;
       else if (argument == ID3) listen[3] = 0;
       else if (argument == 256) { listen[0] = 0; listen[1] = 0; listen[2] = 0; listen[3] = 0; }
     break;
          
     case 2: // listen to only the next command
       chainAddress = 1023;
       if (argument == ID0)      listen[0] |= 1;
       else if (argument == ID1) listen[1] |= 1;
       else if (argument == ID2) listen[2] |= 1;
       else if (argument == ID3) listen[3] |= 1;
       else if (argument == 256) { listen[0] |= 1; listen[1] |= 1; listen[2] |= 1; listen[3] |= 1; }
       else if (argument >= 512) { 
         chainAddress = argument - 512; 
         listen[0] = 0; if (chainAddress == ID0) listen[0] |= 1;
         listen[1] = 0; if (chainAddress == ID1) listen[1] |= 1;
         listen[2] = 0; if (chainAddress == ID2) listen[2] |= 1;
         listen[3] = 0; if (chainAddress == ID3) listen[3] |= 1;
       }       
     break;
               
     /*
       0 enguage cached position                              {always obey command}
       1 turn servo on                                        {obey if listening}
       2 turn servo off                                       {obey if listening}
       3 set cmdpos to curpos                                 {obey if listening}
     */          
     case 3: // set flags
     
       debugToggle();
       
       if (IsHigh(ENGCACHE, argument)) {
         cmdpos[0] = cachepos[0];
         cmdpos[1] = cachepos[1];
         cmdpos[2] = cachepos[2];
         cmdpos[3] = cachepos[3];
       }
       
       if (IsHigh(CMD2CUR, argument)) {
         if (listen[0]){ cmdpos[0] = AdcValues[1];}
         if (listen[1]){ cmdpos[1] = AdcValues[3];}
         if (listen[2]){ cmdpos[2] = AdcValues[5];}
         if (listen[3]){ cmdpos[3] = AdcValues[7];}
       } 
       
       if (IsHigh(SERVOON, argument)) {
         if (listen[0]){ pwm0on(); }
         if (listen[1]){ pwm1on(); }
         if (listen[2]){ pwm2on(); }
         if (listen[3]){ pwm3on(); }
       } else if (IsHigh(SERVOOFF, argument)) {
         if (listen[0]){ pwm0off(); Status0Green(); }
         if (listen[1]){ pwm1off(); Status1Green(); }
         if (listen[2]){ pwm2off(); Status2Green(); }
         if (listen[3]){ pwm3off(); Status3Green(); }
       }
                           
     break;
     
     case 4: // set servo position 
       if (listen[0]){ cmdpos[0] = argument;  }
       if (listen[1]){ cmdpos[1] = argument;  }
       if (listen[2]){ cmdpos[2] = argument;  }
       if (listen[3]){ cmdpos[3] = argument;  }
     break; 
     
     case 5: // set cached position 
       if (listen[0]){ cachepos[0] = argument;  }
       if (listen[1]){ cachepos[1] = argument;  }
       if (listen[2]){ cachepos[2] = argument;  }
       if (listen[3]){ cachepos[3] = argument;  }
     break; 

     case 6: // get servo current
       if      (argument == ID0){ reply = PackBits(AdcValues[0], ReplyCur); }
       else if (argument == ID1){ reply = PackBits(AdcValues[2], ReplyCur); }
       else if (argument == ID2){ reply = PackBits(AdcValues[4], ReplyCur); }
       else if (argument == ID3){ reply = PackBits(AdcValues[6], ReplyCur); }
     break; 
     
     case 7: // get servo position
       if      (argument == ID0){ reply = PackBits(AdcValues[1], ReplyPos);}
       else if (argument == ID1){ reply = PackBits(AdcValues[3], ReplyPos);}
       else if (argument == ID2){ reply = PackBits(AdcValues[5], ReplyPos);}
       else if (argument == ID3){ reply = PackBits(AdcValues[7], ReplyPos);}
     break; 
     
     case 8: // get model
       if ((argument == ID0) || (argument == ID1) || (argument == ID2)|| (argument == ID3)){         
         reply = PackBits(Version, ReplyVer);
       }
     break;
     
   }
   
   switch(command) { // clear one time flags
     case 3:
     case 4:
     case 5:
       listen[0] &= 2; 
       listen[1] &= 2; 
       listen[2] &= 2; 
       listen[3] &= 2; 
       if (chainAddress != 1023) {
         chainAddress++;
         if      (chainAddress == ID0)  listen[0] = 1;
         else if (chainAddress == ID1)  listen[1] = 1;
         else if (chainAddress == ID2)  listen[2] = 1;
         else if (chainAddress == ID3)  listen[3] = 1;
       }
     break;
   }
      
   return reply;
   
}

void AnalogInit (void) {

  int i;

  // Activate ADC with Prescaler 
  ADCSRA =  1 << ADEN  |
            1 << ADSC  | /* start a converstion, irq will take from here */
            0 << ADATE |
            0 << ADIF  |
            1 << ADIE  | /* enable interrupt */
            1 << ADPS2 |
            0 << ADPS1 |
            0 << ADPS0 ;
                        
            
  for (i = 0; i < 8; AdcValues[i++] = 0);
  
}

/*

initialize pwm channels but leave off

16Mhz
/64 = ~1khz
/256 = ~240hz
/1024 = ~61Hz

*/
void pwmInit() {
  // clear pwm levels
  OCR0  = 0; 
  OCR2  = 0;
  OCR1A = 0;
  OCR1B = 0;
  
  // set up WGM, clock, and mode for timer 0
  TCCR0 = 0 << FOC0  | /* force output compare */
          1 << WGM00 | /* fast pwm */
          0 << COM01 | /* ** normal polarity */
          0 << COM00 | /*   this bit 1 for interted, 0 for normal  */
          1 << WGM01 | /* fast pwm */
          0 << CS02  | /* CLKio/64 */
          1 << CS01  |
          1 << CS00  ;
  
  // set up WGM, clock, and mode for timer 1A / 1B
  TCCR1A = 0 << COM1A1 | /* ** normal polarity */
           0 << COM1A0 | /* this bit 1 for interted, 0 for normal  */
           0 << COM1B1 | /* ** normal polarity */
           0 << COM1B0 | /*   this bit 1 for interted, 0 for normal  */
           0 << FOC1A  | /* force output compare */
           0 << FOC1B  | /* force output compare */
           0 << WGM11  | /*  8 bit fast pwm  */
           1 << WGM10  ;
  
  TCCR1B = 0 << ICNC1  | /* input capture noise cancel */
           0 << ICES1  | /* input capture edge select  */           
           0 << WGM13  | /*   8 bit fast pwm */
           1 << WGM12  | /* 8 bit fast pwm */
           0 << CS12   | /*  CLKio/64  */
           1 << CS11   | 
           1 << CS10   ;    
  
  
  // set up WGM, clock, and mode for timer 2
  TCCR2 = 0 << FOC2  | /* force output compare */
          1 << WGM20 | /* fast pwm*/
          0 << COM21 | /* ** normal polarity */
          0 << COM20 | /*   this bit 1 for interted, 0 for normal  */
          1 << WGM21 | /* fast pwm */
          1 << CS22  | /* CLKio/64 */
          0 << CS21  |
          0 << CS20  ;
  
 }
 
int Analog (int n) {
  return AdcValues[n & 7];
}

//SIGNAL(SIG_ADC) {
ISR(ADC_vect) {
  int i;

  i = ADMUX & 7;       // what channel we on?
  AdcValues[i] = ADC;  // save value
  i++;                 // next value
  ADMUX = i & 7;       // select channel
  ADCSRA |= _BV(ADSC); // start next conversion

  if (!ADMUX) {
  //   debugToggle();
  calcWait = 0;
  }

  return;
}


