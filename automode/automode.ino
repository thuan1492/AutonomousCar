//#include <AutoPID.h>
#include <Encoder.h>
#include <Servo.h> 

//#include "status.h"
#define PWMA 12    
#define DIRA1 34 
#define DIRA2 35  
#define PWMB 8   
#define DIRB1 37 
#define DIRB2 36  
#define PWMC 9   
#define DIRC1 43 
#define DIRC2 42  
#define PWMD 5    
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  
#define lift1 25 
#define lift2 28 
#define PWMLift 13
#define servo 4

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,LOW);digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,LOW);digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)

#define LIFT_U(pwm)    do{digitalWrite(lift1,HIGH); digitalWrite(lift2,LOW);analogWrite(PWMLift,pwm);}while(0)
#define LIFT_STOP(x)    do{digitalWrite(lift1,LOW); digitalWrite(lift2,LOW); analogWrite(PWMLift,0);}while(0)
#define LIFT_D(pwm)  do{digitalWrite(lift1,LOW); digitalWrite(lift2,HIGH); analogWrite(PWMLift,pwm);}while(0)

#define MAX_PWM   200
#define MIN_PWM   100
#define echo_front 47 
#define trig_front 48 
#define echo_right 22 
#define trig_right 24 
Servo servoLift;
int Motor_PWM = 70;
int Lift_PWM = 50;
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

// definitions/mappings to ints:
const int stop_int = 0;
const int forward_int = 1;
const int backward_int = 2;
const int right_int = 3;
const int left_int = 4;
const int shiftright_int = 5;
const int shiftleft_int = 6;
const int upperright_int = 7;
const int upperleft_int = 8;
const int lowerright_int = 9;
const int lowerleft_int = 10;
const int manual_int = 11;
const int auto_int = 12;
const int lift_up = 13;
const int lift_down = 14;
const int servo_drop = 15;
const int servo_hold = 16;
int ultra_distance[2], old_distance[2], curr_distance[2];
uint8_t blue_percentage = 0; // the percentage of pixels in the current camera frame that are blue (or at least 'blue' enough)
uint8_t wall_percentage = 0;
uint8_t obstacle_percentage = 0;

uint8_t manual_state = stop_int;
uint8_t mode = manual_int;

int servoLiftCenter = 0;                             // Vertical lift center position
int servoLiftCenterMax = 155;                         // Vertical lift maximum position forward:  85 + 70
int servoLiftCenterMin = 0;                          // Vertical lift minimum position backward: 85 - 50

int next_direction = -1;
int start = 0;

float P = 2 ;
float D = 0.9 ;
float I = 0.5 ;
float oldErrorP ;
float totalError ;
int offset = 5 ;
int wall_threshold = 12 ;
//int left_threshold = 10 ;
//int right_threshold = 10 ;
int front_threshold = 15 ;

int baseSpeed = 50 ;

int RMS ;
int LMS ;
#define CORRIDOR_SIDE_DISTANCE 35 // distance for determining whether in corridor or not
#define DEAD_END_DISTANCE 20 // distance to the front wall when it's time to turn around in a dead end

int cycle_count; // counter that keeps track of the number of cycles (loops) in a specific state

int front_sensor()
{
  // defines variables
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement
  // Clears the trigPin condition
  digitalWrite(trig_front, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trig_front, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_front, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo_front, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}

int right_sensor()
{
  // defines variables
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement
  // Clears the trigPin condition
  digitalWrite(trig_right, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trig_right, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_right, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo_right, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}

void read_distance()
{
  ultra_distance[0] = front_sensor();
  ultra_distance[1] = right_sensor();

  curr_distance[0] = (ultra_distance[0] + old_distance[0])/2;
  curr_distance[1] = (ultra_distance[1] + old_distance[1])/2;

  old_distance[0] = curr_distance[0];
  old_distance[1] = curr_distance[1];
}

void IO_init()
{
  
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);pinMode(DIRD2, OUTPUT);
  pinMode(lift1, OUTPUT);pinMode(lift2, OUTPUT);
  pinMode(PWMLift, OUTPUT);
  
  pinMode(trig_right, OUTPUT);
  pinMode(echo_right, INPUT); 
  pinMode(trig_front, OUTPUT);
  pinMode(echo_front, INPUT); 
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
  
  
}

void pid_start() {
  if(curr_distance[1] < 30)
  {
    float errorP = wall_threshold - curr_distance[1] ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3.0) * errorI + errorP ;
  
    totalError = P * errorP + D * errorD + I * errorI ;
    
    oldErrorP = errorP ;
    
    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;
    if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;
    
   
  }
  else 
  {
    RMS = 0;
    LMS = 0;
  }
   MOTORA_FORWARD(LMS);
    MOTORB_FORWARD(RMS);
    MOTORC_FORWARD(LMS);
    MOTORD_FORWARD(RMS);

}
//----------------------------- wall follow  control -------------------------------//

void setup() {
 
  Serial.begin(9600);  // baudrate = 9600 bps
  IO_init();
  
 
}
void loop()
{
  read_distance();
  if(curr_distance[0] >15)
  {
    pid_start();
  }
  else 
  {
    MOTORA_STOP(0);
    MOTORB_STOP(0);
    MOTORC_STOP(0);
    MOTORD_STOP(0);
  }
  Serial.print(" Right : ");
  Serial.print(curr_distance[1]);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(curr_distance[0]);
  Serial.println(" cm ");

  //measure error & print the result to the serial monitor


  Serial.print("error=");
  Serial.println(totalError);
}
