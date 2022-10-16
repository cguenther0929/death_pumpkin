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

#define DESIRED_BAUD                    115200

/**
 * Define IO
 */
#define STOP_BUTTON                     7
#define START_BUTTON                    12

#define STAT_LED_PIN                    13
#define GRN_LED_PIN                     4
#define RED_LED_PIN                     5
#define PWM_PIN                         9

#define BUTTON_DEBOUNCE_SEC             .100
// #define BUTTON_DEBOUNCE_CNT             (unsigned int)(BUTTON_DEBOUNCE_SEC/BUTTON_DEBOUNCE_CNT)
#define BUTTON_DEBOUNCE_CNT             5

unsigned int stop_button_bounce_counter = 0;
bool stop_spray_function                = false;

unsigned int start_button_bounce_counter = 0;

/**
 * PWM and Spray-related 
 * details.
 * Increasing the PWM
 * value equates to 
 * CCW movement
 */
#define UNLOAD_CAN_PWM_VAL              41 //added 10
#define NO_SPRAY_PWM_VAL                61     
#define SPRAY_PWM_VAL                   120

#define SECS_BETWEEN_SPRAY            8
#define ms100_SPRAY_DWELL             4

enum fsm_states {
  Undefined,
  UnloadCan,
  Spray,
  noSpray
};

fsm_states current_state              = noSpray;

enum color_active {
  GRN_LED_COLOR,
  RED_LED_COLOR
};

color_active led_color                = GRN_LED_COLOR;


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

unsigned int  ms1000_counter          = 0;
unsigned int  ms500_counter           = 0;
unsigned int  ms100_counter           = 0;

void setup() {
  
  Serial.begin(DESIRED_BAUD);             // For the serial monitor
  
  pinMode(START_BUTTON,INPUT_PULLUP);
  pinMode(STOP_BUTTON,INPUT_PULLUP);
  pinMode(GRN_LED_PIN,OUTPUT);
  pinMode(RED_LED_PIN,OUTPUT);
  
  pinMode(STAT_LED_PIN,OUTPUT);
  myservo.attach(PWM_PIN);                      // attaches the servo on pin 9 to the servo object
  
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

  interrupts();                                      // Enable interrupts

}

void loop() {
  
  if(Time10msFlag == true) {
    Time10msFlag = false;
  }
  
  if(Time20msFlag == true) {
    Time20msFlag = false;

    if(digitalRead(STOP_BUTTON) == LOW && stop_spray_function == false) {
      stop_button_bounce_counter++;
      
      if(stop_button_bounce_counter >= BUTTON_DEBOUNCE_CNT) {
        Serial.println("STOP SPRAY FUNCTION");
        stop_spray_function = true;
        stop_button_bounce_counter = 0;
      }
    }
    else {
      if(stop_button_bounce_counter > 0){
        stop_button_bounce_counter--;
      }
    }
    
    if(digitalRead(START_BUTTON) == LOW && stop_spray_function == true) {
      
      start_button_bounce_counter++;
      
      if(start_button_bounce_counter >= BUTTON_DEBOUNCE_CNT) {
        Serial.println("COMMENCE SPRAY FUNCTION");
        stop_spray_function = false;
        start_button_bounce_counter = 0;
      }
    }
    else {
      if(start_button_bounce_counter > 0){
        start_button_bounce_counter--;
      }
    }

  }

  if(Time100msFlag == true) {
    Time100msFlag = false;
    ms100_counter++;
    
    if(led_color == GRN_LED_COLOR){
      digitalWrite(RED_LED_PIN,LOW);
      digitalWrite(GRN_LED_PIN,!digitalRead(GRN_LED_PIN));
    }
    else {
      digitalWrite(GRN_LED_PIN,LOW);
      digitalWrite(RED_LED_PIN,!digitalRead(RED_LED_PIN));
    }
    
  
  }

  if(Time500msFlag == true) {
    Time500msFlag = false;
    ms500_counter++;
  }
  
  if(Time1000msFlag == true) {
    Time1000msFlag = false;
    ms1000_counter++;
    digitalWrite(STAT_LED_PIN,!digitalRead(STAT_LED_PIN));

    if(stop_spray_function == true){ 
      Serial.println("Cease spray.");
    }
    else{ 
      Serial.println("OKAY to spray.");
    }
    
  }

  evaluateState();

}

void evaluateState (void) {
  switch(current_state) {
    
    case(UnloadCan):
      
      ms1000_counter = 0;
      ms500_counter = 0;
      ms100_counter = 0;

      if(stop_spray_function == false){
        setServoPosition(NO_SPRAY_PWM_VAL);
        current_state = noSpray;
        led_color = GRN_LED_COLOR;
      }

    break;
    
    case(Spray):

      if(stop_spray_function == true){
        current_state = UnloadCan;
        setServoPosition(UNLOAD_CAN_PWM_VAL);
        led_color = GRN_LED_COLOR;
        break;
      }
    
      if(ms100_counter >= ms100_SPRAY_DWELL) {
        current_state = noSpray;
        led_color = GRN_LED_COLOR;
        setServoPosition(NO_SPRAY_PWM_VAL);
        ms1000_counter = 0;
        ms500_counter = 0;
        ms100_counter = 0;
      }
    
    
    break;

    case(noSpray):

      if(stop_spray_function == true){
        current_state = UnloadCan;
        led_color = GRN_LED_COLOR;
        setServoPosition(UNLOAD_CAN_PWM_VAL);
        break;
      }
      
      if(ms1000_counter >= SECS_BETWEEN_SPRAY) {
        current_state = Spray;
        led_color = RED_LED_COLOR;
        setServoPosition(SPRAY_PWM_VAL);
        
        ms1000_counter = 0;
        ms500_counter = 0;
        ms100_counter = 0;
      }

    break;

    default:
      setServoPosition(NO_SPRAY_PWM_VAL);
    break;

  } /* End Switch*/
} /*End function*/





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
