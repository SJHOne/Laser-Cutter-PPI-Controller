///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Laser PPI controller - based on the work published by dirktheng :
// http://www.buildlog.net/blog/2011/12/getting-more-power-and-cutting-accuracy-out-of-your-home-built-laser-system

// Original routine and hardware replaced by external interrupts on lines D2 (Step X) and D3 (Step Y)
// Direction pin ignored (distances travelled are positive absolute)
// Laser input on pin 6, PPI output on pin 7
// LED Output (ready) on pin 8 - code reads config values from A0 and A1 once at startup, press reset to re-read 
// (disable laser power first!)

// Uses the digitalPin fast read/write library
// Uses MsTimer2 library

// Version 0.1
// Replaced external hardware with interrupt driven counters
// Version 0.2
// Optimized code
// Version 0.3
// Replaced millis() with interrupt timer
// Version 0.4
// Moved pin assignments
// Version 0.5
// Used function pointer to define routine to use
// Steve Hobley 2012 - www.stephenhobley.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/interrupt.h>
#include <DigitalPin.h>
#include <MsTimer2.h>

#define STEPS_PER_INCH_X 506.178   // This needs to be set for steppers
#define STEPS_PER_INCH_Y 506.178   // This needs to be set for steppers

#define LASER_PULSE_DURATION_MS 3  // Starting Value

#define LASERCONTROL_IN 6
#define LASERCONTROL_OUT 5
#define LED_OUT_PPI 10
#define LED_OUT_NORMAL 9

#define TURN_LASER_OFF writePinLaser.write(LOW)  // macro sets laser off
#define TURN_LASER_ON writePinLaser.write(HIGH)  // macro sets laser on
#define LASER_CMD_PIN readPinLaser.read()        // defines pin that laser comnd is on

double PPI = 100.0;
int laserPulseWidth = LASER_PULSE_DURATION_MS;

long xPrevCnt = 0; // initialize the x axis integration variable
long yPrevCnt = 0; // initialize the y axis integration variable
long xCurrentCnt = 0; // non-volatile working copy
long yCurrentCnt = 0; // non-volatile working copy

double PPI_Inv = 0;
double distTravelled = 0;

int laserCmd = 0;
int laserCmdPrev = 0;
int firstOnState = 0;

DigitalPin<LASERCONTROL_IN> readPinLaser;
DigitalPin<LASERCONTROL_OUT> writePinLaser;
DigitalPin<LED_OUT_PPI> writePinLEDPPI;
DigitalPin<LED_OUT_NORMAL> writePinLEDNormal;

volatile unsigned long xCount=0; // external interrupt counter
volatile unsigned long yCount=0; // external interrupt counter

typedef void (* PPIFuncPtr) (); // define function to call
PPIFuncPtr routineToCall;

///////////////////////////////////////////////////////////////////////////
void setup(void)
{
    Serial.begin(9600);    

    writePinLEDPPI.mode(OUTPUT);
    writePinLEDNormal.mode(OUTPUT);
    
    writePinLEDPPI.write(LOW);
    writePinLEDNormal.write(LOW);
    
    // input mode with pullup disabled
    readPinLaser.config(INPUT, LOW);
    writePinLaser.mode(OUTPUT);
    
    ReadAnalogValues();
    if (PPI == 0)
    {
      // Both pots set to 0, use constant wave mode
      routineToCall = NormalRoutine;
      
      Serial.println("Constant Wave Mode");
      
    }
    else
    {
      // Use PPI Mode
      routineToCall = PPIRoutine;
    
      PPI_Inv = 1/PPI;
    
      // Configure pulse length timer
      MsTimer2::set(laserPulseWidth, ClearPulse); 
    
      // TODO: Do we want pullups (2/3) on?
    
      DDRD &= ~(1 << DDD2);     // Clear the PD2 pin - PD2 (PCINT0 pin) is now an input
      PORTD |= (1 << PORTD2);   // turn On the Pull-up - PD2 is now an input with pull-up enabled 
    
      DDRD &= ~(1 << DDD3);     // Clear the PD3 pin - PD3 (PCINT0 pin) is now an input
      PORTD |= (1 << PORTD3);   // turn On the Pull-up - PD3 is now an input with pull-up enabled 
    
      EIMSK |= (1 << INT0);     // Enable external interrupt INT0
      EICRA |= (1 << ISC01);    // Trigger INT0 on falling edge
    
      EIMSK |= (1 << INT1);     // Enable external interrupt INT1
      EICRA |= (1 << ISC11);    // Trigger INT1 on falling edge

      sei();                    // Enable global interrupts

      writePinLEDPPI.write(HIGH);  // Ready to go

      Serial.println("PPI Mode");
      Serial.print(laserPulseWidth);
      Serial.print(", ");
      Serial.print(PPI);
      Serial.print(", ");
      Serial.println(PPI_Inv);
    }

}
///////////////////////////////////////////////////////////////////////////
void ReadAnalogValues()
{
  //PPI = (double)mymap(analogRead(0), 0, 1023, 0, 400.0);
  PPI = mymap(analogRead(0));
  
  laserPulseWidth = map(analogRead(1), 0, 1023, 0, 3);
  laserPulseWidth*=2; // double this value -> 0, 2, 4, 6)
}

///////////////////////////////////////////////////////////////////////////
double mymap(int input)
{
    if (input >= 537)
      return 400.0;
    if (input >= 150)
      return 200.0;
    if (input >= 72)
      return 100.0;
    if (input > 0)
      return 10.0;

  return 0;      
}
///////////////////////////////////////////////////////////////////////////
void FireLaser()
{
   distTravelled = 0;
   xPrevCnt = xCurrentCnt;
   yPrevCnt = yCurrentCnt;
        
   TURN_LASER_ON;
   MsTimer2::start();
}
///////////////////////////////////////////////////////////////////////////
void loop(void)
{
  // Call either the PPI or Normal routine through a function pointer
  routineToCall();
} 

///////////////////////////////////////////////////////////////////////////
void PPIRoutine(void)
{
  writePinLEDPPI.write(HIGH);  // Ready to go
  updateLaserCmd();
  
  if (laserCmd) 
  {
    if (firstOnState) 
    {
      FireLaser();
      firstOnState = 0;
    }
    
    // This routine contains the critical section
    updateTravel();

    // If we have moved over the threshold distance, turn laser on 
    if (distTravelled >= PPI_Inv) 
    {
      FireLaser();
    }
  }
  else
  {
    TURN_LASER_OFF; // Keep laser off while not cutting
  }
} 
///////////////////////////////////////////////////////////////////////////
void NormalRoutine(void)
{

  writePinLEDNormal.write(HIGH);  // Ready to go
  updateLaserCmd();
  
  if (laserCmd) 
  {
    TURN_LASER_ON;
  }
  else
  {
    TURN_LASER_OFF; 
  }
} 

///////////////////////////////////////////////////////////////////////////
void ClearPulse() 
{
  TURN_LASER_OFF;
  MsTimer2::stop();
}

///////////////////////////////////////////////////////////////////////////
// Called when the laser control is off - we're not interested when the head is rapid moving.
void ResetCounts()
{
  xPrevCnt = 0;
  yPrevCnt = 0;
  xCurrentCnt = 0;
  yCurrentCnt = 0;
  
  // Keep this critical section as short as possible
  noInterrupts();
  xCount = 0;
  yCount = 0;
  interrupts();
}

///////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine attached to INT0 (2) vector
ISR(INT0_vect)
{
  xCount++;  
}

///////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine attached to INT1 (3) vector
ISR(INT1_vect)
{
  yCount++;  
}

///////////////////////////////////////////////////////////////////////////
void updateTravel() 
{
  // Keep this critical section as short as possible
  noInterrupts();
  xCurrentCnt = xCount;
  yCurrentCnt = yCount;
  interrupts();
  
  distTravelled = sqrt(pow((xCurrentCnt-xPrevCnt)/STEPS_PER_INCH_X,2) + pow((yCurrentCnt-yPrevCnt)/STEPS_PER_INCH_Y,2));
}

///////////////////////////////////////////////////////////////////////////
void updateLaserCmd() 
{
  laserCmdPrev = laserCmd;
  laserCmd = LASER_CMD_PIN;
  
  // If we have transitioned from off to on
  if ((laserCmdPrev == 0) && (laserCmd == 1)) 
  {
    ResetCounts();
    firstOnState = 1;
  }
}

