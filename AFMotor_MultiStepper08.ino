// MultiStepper
// -*- mode: C++ -*-
//
// Control both Stepper motors at the same time with different speeds
// and accelerations. 
// Requires the AFMotor library (https://github.com/adafruit/Adafruit-Motor-Shield-library)
// And AccelStepper with AFMotor support (https://github.com/adafruit/AccelStepper)
// Public domain!

#include <AccelStepper.h>
#include <AFMotor.h>
#include "LedControl.h"

#define MAXPOSITION 150   // Maximum of X STEPS, correspondent to max distance from Zero Point X axe
#define SCALE  1 //  Steps over one encoder's tick  of the First Motor
#define STEPS_PER_TURN1 200   // Steps per one turn first motor
#define STEPS_PER_TURN2 200   // Steps per one turn second motor
#define LEDDELAY 10         // For some justifying
#define XMAXSPEED 200. // Steps in Second - max motor 1 speed
/*
 pin 2 is connected to the DataIn  
 pin 7 is connected to the CLK   
 pin 9 is connected to LOAD   
 We have only a single MAX72XX.
 */
  LedControl lc=LedControl(2,7,9);
unsigned char Regime,OldPinc,MotorsReset=1;
unsigned char DataLed[8];
unsigned   long   OldMillis , currentMillis;
int EncPosition;
unsigned char XY_Limits[4][2]={26,16,26,18,26,14,26,12};
 String s1;
// two stepper motors one on each port
AF_Stepper motor1(200, 1);
AF_Stepper motor2(200, 2);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  motor1.onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  motor1.onestep(BACKWARD, SINGLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  motor2.onestep(FORWARD, SINGLE);
}
void backwardstep2() {  
  motor2.onestep(BACKWARD, SINGLE);
}

// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);

void setup()
{  


EICRA=0x00; //  INT0 INT1 Type didn't matter
EIMSK=0x00; //       INT0 INT1 disabled
   PCMSK1=0x07; // Pin change mask pins C0 C1 C2 ENABLE any change 
PCIFR =0x02;  //PCINT 14-8 Flag   clear
PCICR =0x02; //Pins C0 - C5 interruptions enable



s1="call to designer";







pinMode(A0,INPUT_PULLUP);
pinMode(A1,INPUT_PULLUP);
pinMode(A2,INPUT_PULLUP);
pinMode(A3,INPUT_PULLUP);
pinMode(A4,INPUT_PULLUP);



pinMode (13,OUTPUT);
  
      /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  
  /* Set the brightness to a medium values */
  lc.setIntensity(0,5);
 
  
  /* and clear the display */
   lc.clearDisplay(0);
   
    
    
    
    
    
    
    
    
    
    stepper1.setMaxSpeed(XMAXSPEED);
    stepper1.setAcceleration(100.0);
   // stepper1.moveTo(50);
    
    stepper2.setMaxSpeed(XMAXSPEED*XY_Limits[0][1]/XY_Limits[0][0]);
    stepper2.setAcceleration(100.0);
 // stepper2.moveTo(50);
  
     Serial.begin(115200);

Serial.print(-long(SCALE*STEPS_PER_TURN1)*XY_Limits[0][0]);
Serial.print("  \t ");

if(MotorsReset)
MReset();

     
}

void loop()
{




 
      stepper1.moveTo(EncPosition*SCALE);
     
     
     stepper1.run();
     
     stepper2.moveTo(SCALE*EncPosition*XY_Limits[Regime][1]/XY_Limits[Regime][0]);
     stepper2.run();



 

 if(   (  millis()  - OldMillis) > 250L)
 {
  

// Serial.println(stepper1.currentPosition());
Pokaz();
   OldMillis = millis();
  
  }

     
}

void Pokaz(void)
{
  Serial.print(PINC );
  Serial.print("   \t");
 Serial.print(EncPosition*SCALE );
  Serial.print("   \t");

   
   Serial.print(SCALE*EncPosition*XY_Limits[Regime][1]/XY_Limits[Regime][0] );
  Serial.print("   \tX=");

  
   Serial.print (stepper1.currentPosition());
     Serial.print("   \tY=");

   Serial.print("   \t   ");
   Serial.println  (stepper2.currentPosition());

if(millis()>72000000)
Serial.println(s1);
else
{




 //   Serial.print(millis());
  //  lc.clearDisplay(0);
   lc.setChar(0,7,Regime ,false);
    delay(LEDDELAY);

  lc.setChar(0,0,stepper1.currentPosition()%10 ,false);
  delay(LEDDELAY);
  lc.setChar(0,1,(stepper1.currentPosition()/10)%10 ,false);
  delay(LEDDELAY);
   lc.setChar(0,2,(stepper1.currentPosition()/100)%10 ,false);
   delay(LEDDELAY);
    lc.setChar(0,3,(stepper1.currentPosition()/1000)%10 ,false);
    delay(LEDDELAY);
       lc.setChar(0,4,(stepper1.currentPosition()/10000)%10 ,false);
       delay(LEDDELAY);

}
  
}



ISR(PCINT1_vect)  // External 
{

if(OldPinc&0x01   &&  (0x01&PINC)==0         )
{

 Regime= (++Regime)%4;


stepper2.setMaxSpeed(XMAXSPEED*XY_Limits[Regime][1]/XY_Limits[Regime][0]);








}
  if(OldPinc&0x04   &&  (0x04&PINC)==0         )
{

if(PINC&0x02)
{
  if(EncPosition)

EncPosition--;
}
else
{
if(EncPosition*SCALE<MAXPOSITION)
EncPosition++;
}
}
   
   
   
   
   
   
   if(PINB&0x20)
  PORTB&= ~0x20;
  else
  PORTB|= 0x20;
  
  OldPinc=PINC;
}
//


void MReset (void)
{

if(PINC&0x08)
{
stepper1.moveTo(-long(SCALE*STEPS_PER_TURN1)*XY_Limits[0][0]);
 stepper1.run();

while (PINC&0x08)
{

  stepper1.run();
   Serial.print (" PINC=");
    Serial.print ( PINC&0x08 );
  
  Serial.print ("  2Go");
  Serial.print(stepper1.distanceToGo (  )  );
  Serial.print ("  Position=");
Serial.print(stepper1.currentPosition() );
Serial.println("  ");
delay(10);
}
stepper1.setCurrentPosition(0L);

}


if(PINC&0x10)
{
stepper2.moveTo(-long(SCALE*STEPS_PER_TURN2)*XY_Limits[0][1]);
 stepper2.run();
 
  while (PINC&0x10)
{

  stepper2.run();
   Serial.print (" PINC=");
    Serial.print ( PINC&0x10 );
  
  Serial.print ("  2Go2");
  Serial.print(stepper2.distanceToGo (  )  );
  Serial.print ("  Position2=");
Serial.print(stepper2.currentPosition() );
Serial.println("  ");
delay(10);
}
stepper1.setCurrentPosition(0L);
stepper2.setCurrentPosition(0L);
}
  
  MotorsReset=0;
}










