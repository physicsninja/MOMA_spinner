
#include <Arduino.h>
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
 

// Quadrature encoder
#define c_EncoderInterruptApin 3 // we only have two interrupts on the arduino uno
#define c_OptoInterruptpin 2
#define c_EncoderInterruptBpin 5
//#define EncoderIsReversed
volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;

//system status variables
bool homed = false;
bool in_motion = false;

 //setup conversion factors
 double mdegrees_per_pulse = 360/2; // millidegrees per pulse
 double encoder_angle = 0.00;
 double rotation_speed = 1.00; //degrees per seconds
 double timeout = 300.0*1000;



 #define MOTOR_ENGAGE_PIN 4

void setup() {

  Serial.begin(9600);
     
 
  // Quadrature encoder
  pinMode(c_EncoderInterruptApin, INPUT);      
  digitalWrite(c_EncoderInterruptApin, LOW);  // turn on pullup resistors
  pinMode(c_OptoInterruptpin, INPUT);      
  digitalWrite(c_OptoInterruptpin, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_EncoderInterruptApin), HandleInterruptA, RISING);
  attachInterrupt(digitalPinToInterrupt(c_OptoInterruptpin), HandleOptoInterrupt, RISING);

  pinMode(MOTOR_ENGAGE_PIN, OUTPUT);      
  digitalWrite(MOTOR_ENGAGE_PIN, LOW);  
  
 
}

void loop() {
  
    if( Serial.available()){
      serial_parse();
    }
    delay(100);
    
}

void safe_rotate(double ang)
{
  double angle = ang -.3; //correction to account for magnet inertia
  double current = _EncoderTicks*mdegrees_per_pulse/1000;
  double dir = 1;
  in_motion = true;
  int start = millis();
  int current_time = millis();
  digitalWrite(MOTOR_ENGAGE_PIN, HIGH);
  if((angle - current) <0 ){
    while ((angle - current) <0){
      if( Serial.available()){
      serial_parse();
    }
      current = _EncoderTicks*mdegrees_per_pulse/1000;
      if((current_time-start)>timeout){
      digitalWrite(MOTOR_ENGAGE_PIN, LOW);
      break;
    }
    }
  }
  while(current < angle){
    if( Serial.available()){
      serial_parse();
    }
    
    current = _EncoderTicks*mdegrees_per_pulse/1000;
    if((current_time-start)>timeout){
      digitalWrite(MOTOR_ENGAGE_PIN, LOW);
      break;
    }
   
  }
  digitalWrite(MOTOR_ENGAGE_PIN, LOW);
  
  in_motion = false;
}

int home_magnet(void){
  int start = millis();
  int current = millis();
  in_motion = true;
  digitalWrite(MOTOR_ENGAGE_PIN, HIGH);
  while(!homed){
    if( Serial.available()){
      serial_parse();
    }
    current = millis();
    if((current-start)>timeout){
      digitalWrite(MOTOR_ENGAGE_PIN, LOW);
      in_motion = false;
      return -1;
    }
  }
  digitalWrite(MOTOR_ENGAGE_PIN, LOW);
  in_motion = false;
  return 1;
  
}

int rotate_absolute(double angle){

  if(angle >359){ // I have to draw the line somewhere and modulo doesn't work on floats
    return -1;
  }
  
  if(!homed){
   home_magnet(); //hard to rotate absolute when you don't know where 0 is
  }
  
  safe_rotate(angle);
  return 1;
}

void serial_parse(void){
 String content;
 char character;
     while(Serial.available()) {
      character = Serial.read();
      content.concat(character);
    }

    if(content.startsWith("HOME")){
      int stat = home_magnet();
      if(stat<0){
        //Serial.println("Homing Failed, disabling motors");
      }
      else{
        //Serial.println(stat);
      }
    }

    if(content.startsWith("*IDN?")){
    Serial.println("MOMA Spinning magnet Rev 1.0");
  Serial.println("Last update 8-15-2020 by Neal");
  Serial.print("milidegrees per encoder pulse: ");
  Serial.println(mdegrees_per_pulse);
 }

    if(content.startsWith("HOME?")){
       Serial.println(homed);
     
    }
    
    if(content.startsWith("LOC?")){
      if(!in_motion){
        Serial.println(_EncoderTicks*mdegrees_per_pulse/1000);
        }
      else{
        Serial.println(-1);
      }
       
    }
    
    if(content.startsWith("GOTO ")){
       content = content.substring(5);
       int stat = rotate_absolute(content.toFloat());
    }
 }


void HandleInterruptA()
{
  _EncoderASet = digitalReadFast(c_EncoderInterruptApin);    //get quadrature states
  _EncoderBSet = digitalReadFast(c_EncoderInterruptBpin);   //get quadrature states
 
  // and adjust counter + if A leads B
  #ifdef EncoderIsReversed
    if((_EncoderASet && !_EncoderBSet) || (!_EncoderASet && _EncoderBSet)){
      _EncoderTicks -= 1;
    }
    else{
      _EncoderTicks += 1;
    }
  #else
    if((_EncoderASet && !_EncoderBSet) || (!_EncoderASet && _EncoderBSet)){
      _EncoderTicks += 1;
    }
    else{
      _EncoderTicks -= 1;
    }
  #endif
}

void HandleOptoInterrupt()
{
  homed = true;
  _EncoderTicks = 0;
}

 
