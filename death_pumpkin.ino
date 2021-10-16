/*
 * File:  death_pumpkin.ino
 * --------------------
 * For all Arduino Variants (Mini/Uno/Mega):
 * The delay function uses timer 0, so this timer shan't be used in the application.  
 * 
 * SERVOS: 
 * For 'lighter' Arduino variants (Mini/Uno), the servo library uses timer 1,
 * thus this timer shan't be used in the application if needing the servo library.
 * 
 * Controlling a servo position using a potentiometer (variable resistor)
 * by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>*
 * modified on 8 Nov 2013 by Scott Fitzgerald
 * http://www.arduino.cc/en/Tutorial/Knob
 *
 * For the MG996R Servo Motor:
 *  Orange Wire == Signal (drive PWM into this wire)
 *  BRN         == GND
 *  RED         == Attach to 5+
 * 
 * 
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

#define STAT_LED                      13
#define DESIRED_BAUD                  115200

// #define NO_SPRAY_PWM_VAL              115      // Lower == CW / Higher == CCW
#define UNLOAD_CAN_PWM_VAL            19
#define NO_SPRAY_PWM_VAL              67     
// #define DIFF_TO_SPRAY                 50
// #define SPRAY_PWM_VAL                 (NO_SPRAY_PWM_VAL - DIFF_TO_SPRAY)     
#define SPRAY_PWM_VAL                 136

// #define LOAD_SPRING_PWM_VAL           130     // Higher the number, the more CCW (verified in cal)
#define LOAD_SPRING_PWM_VAL           NO_SPRAY_PWM_VAL     // TODO: can we get rid of this?  
#define SECS_BETWEEN_SPRAY            8
#define ms100_SPRAY_DWELL             4

enum State {
  Undefined,
  Spray,
  noSpray
};

/* Time keeping variables */
unsigned int  ticks_10ms              = 0;
unsigned int  ticks_20ms              = 0;
unsigned int  ticks_100ms             = 0;
unsigned int  ticks_500ms             = 0;
unsigned int  ticks_1000ms            = 0;
bool          Time10msFlag            = false;
bool          Time20msFlag            = false;
bool          Time100msFlag           = false;
bool          Time500msFlag           = false;
bool          Time1000msFlag          = false;

unsigned int  current_state           = noSpray;
unsigned int  ms1000_counter          = 0;
unsigned int  ms500_counter           = 0;
unsigned int  ms100_counter           = 0;

void setup() {
  
  Serial.begin(DESIRED_BAUD);             // For the serial monitor
  pinMode(STAT_LED,OUTPUT);
  myservo.attach(9);                      // attaches the servo on pin 9 to the servo object
  
  noInterrupts();                           //Disable interrupts
  
  /* Setup Time 2 for timebase */
  // set compare match register for 50Hz increments (20ms interrupt)
  // The value for match register is:  Register Value = [(target_time_interval) / (prescaler_value / MCU_CLK)] -1  
  // The aforementioned equation can be derived from p.150 in the datasheet
  // The register value must be <= 255 for timer 2 (8 bit)
  
  TCCR2A = 0;                               // set entire TCCR2A register to 0
  TCCR2B = 0;                               // same for TCCR2B
  TCNT2  = 0;                               // initialize counter value to 0

  OCR2A = 155;                                         

  // WGM[2:0] set to 0b010 (Clear Timer on Compare Match -- so automatically set reg to zero) p.160
  // Bits [1:0] are accessable in TCCR2A 
  // Bits COM2A1 COM2A0 = 0b00 -- Normal port operation, OC0A disconnected.
  TCCR2A = (1 << WGM21) | (0 << WGM20);

  // A CS value of 0b111 sets the prescaler value to 1024 p.162
  TCCR2B = (0 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);           
  TIMSK2 |= (1 << OCIE2A);                                    // Enable timer compare interrupt

  setServoPosition(LOAD_SPRING_PWM_VAL);
  delay(10000);
  
  interrupts();                                      // Enable interrupts

}

void loop() {
  
  if(Time10msFlag == true) {
    Time10msFlag = false;
  }
  
  if(Time20msFlag == true) {
    Time20msFlag = false;
  }

  if(Time100msFlag == true) {
    Time100msFlag = false;
    ms100_counter++;
  }

  if(Time500msFlag == true) {
    Time500msFlag = false;
    ms500_counter++;
  }
  
  if(Time1000msFlag == true) {
    Time1000msFlag = false;
    ms1000_counter++;
    digitalWrite(STAT_LED,!digitalRead(STAT_LED));
  }

  if(ms1000_counter >= SECS_BETWEEN_SPRAY && current_state == noSpray) {
    setServoPosition(SPRAY_PWM_VAL);
    current_state = Spray;
    ms1000_counter = 0;
    ms500_counter = 0;
    ms100_counter = 0;
  }
  
  if(ms100_counter >= ms100_SPRAY_DWELL && current_state == Spray) {
    setServoPosition(NO_SPRAY_PWM_VAL);
    current_state = noSpray;
    ms1000_counter = 0;
    ms500_counter = 0;
    ms100_counter = 0;
  }
  
}

void setOutputSpray (void) {
  myservo.write(20);                  // sets the servo position according to the scaled value
}

void setOutputNoSpray (void) {
  myservo.write(80);                  // sets the servo position according to the scaled value
}

void setServoPosition (unsigned int position) {
  myservo.write(position);                        
}

ISR(TIMER2_COMPA_vect){           // Timer2 interrupt 
  
  Time10msFlag = true;            // Enter this ISR every 10ms
  
  if(ticks_10ms >= 1) {
    ticks_10ms = 0;              
    Time20msFlag = true;
    
    if(ticks_20ms >= 4) {       
      ticks_20ms = 0;           
      Time100msFlag = true;  

      if(ticks_100ms >= 4) {
        ticks_100ms = 0;
        Time500msFlag = true;

        if(ticks_500ms >= 1) {
          ticks_500ms = 0;
          Time1000msFlag = true;
          
          if(ticks_1000ms >= 59){
            ticks_1000ms = 0;
          }
          else {
            ticks_1000ms++;
          }

        }
        else {
          ticks_500ms++;
        }

      }
      else {
        ticks_100ms++;
      }

    }
    else {
      ticks_20ms++;            
    }
  }
  else {
      ticks_10ms++;
  }

}