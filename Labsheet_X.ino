#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"



#define FWD LOW
#define RVS LOW


#define RED_LED_PIN 17
#define ORANGE_LED_PIN 13  // Pin to activate the orange LED
#define GREEN_LED_PIN 30
#define BUZZER_PIN 6

float MAX_PWM = 20;
float MAX_TURN_PWM = 40;

float MAX_SPEED = 20;
float MAX_TURN_SPEED = 40;

int needed_Count_L = 0;
int needed_Count_R = 0;


boolean led_state;  // Variable to "remember" the state
                    // of the LED, and toggle it.

//motor class
Motors_c motors;

//Line Sensor class
LineSensor_c lineSensors;

//Kinematics
Kinematics_c kinematics;

//PID Controller
PID_c right_PID;
PID_c left_PID;
PID_c turning_PID;

float currentTurnValue = 0;

//turning variables
float wantedAngle = 0;
float startingAngle = 0;

//Begin Variables
bool foundLine = false;
bool skippedLine = false;

//state variables
float stateTimer = 0;

// state enum
enum State_enum {
  BEGIN = 0,
  FOLLOWING = 1,
  RETURN_HOME = 2,
  TURN = 3,
  LOST_LINE = 4,
  TURN_RIGHT = 5,
  TURN_LEFT = 6,
  TURN_ON_LINE = 7,
  TURN_AROUND = 8,
  DO_NOTHING = 9,
  PREP_TURN_LEFT = 10,
  ROTATE_HOME = 11
};

State_enum state = BEGIN;


//rotatiom speeds - refactor into kinematics.h
long timeSinceLastUpdate = 0;
//long encoderCount_e0 = 0;
//long encoderCount_e1 = 0;

//timer count
long timer = 0;

long finishedTimer = 40;
bool finish = false;
float x_length_home = 0;



// put your setup code here, to run once:
void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  Serial.println(sin(0));
  state = BEGIN;
  //wantedAngle = 180;
  // Set LED pin as an output
  pinMode(ORANGE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  setupEncoder0();
  setupEncoder1();
  //Set orange to high so we can see its booting
  digitalWrite(ORANGE_LED_PIN, HIGH);

  //init classes

  motors.initialise();
  lineSensors.initialize();
  kinematics.initialize();
  right_PID.initialize(0.6f);
  left_PID.initialize(0.6f);
  turning_PID.initialize(0.25f);


  //set orange LED to LOW
  //set green LED to HIGH to show boot was successful
  //Slight delay to make boot more obvious
  delay(500);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(ORANGE_LED_PIN, LOW);
  //tone(BUZZER_PIN, 400,500);
  delay(500);
  digitalWrite(GREEN_LED_PIN, LOW);

  // Set initial state of the LED
  led_state = false;

  //encoderCount_e0 = count_e0;
  //encoderCount_e1 = count_e1;
  timeSinceLastUpdate = micros();
  stateTimer = micros();
  //motors.SetMotorPower(20, 0);
}



void MotorControls() {
  int leftMotorValue = 10;
  int rightMotorValue = 10;
  if (state == 1) {
    GetMotorValues(leftMotorValue, rightMotorValue);
    //if(count_e0 <-358){rightMotorValue = 0;} //these can be overshot dependant on the motorspeed
    //if(count_e1 <-358){leftMotorValue = 0;}  //these can be overshot dependant on the motorspeed
    motors.SetMotorPower(leftMotorValue, rightMotorValue);
  }
  if (leftMotorValue == 0 && rightMotorValue == 0) state = 2;
  if (state == 2) {
    int r = LOW;
    int l = LOW;
    if (count_e0 < 0) {
      r = HIGH;
    }
    if (count_e1 < 0) {
      l = HIGH;
    }
    motors.SetMotorDirection(l, r);
    motors.SetMotorPower(abs(count_e1), abs(count_e0));
  }
}





// put your main code here, to run repeatedly:
void loop() {


  if(millis()/1000 > finishedTimer && finish == false){
    finish = true;
    digitalWrite(ORANGE_LED_PIN, HIGH);
    
  }
  //if(kinematics.x_pos > 4000){
  //  digitalWrite(ORANGE_LED_PIN, HIGH);
  //  digitalWrite(GREEN_LED_PIN, LOW);
  //}
  //Serial.print("Right Sensor - ");
  //Serial.print(lineSensors.ReadRightMostSensor());
  //
  Serial.print(" - XPOS - ");
  Serial.print(kinematics.TurnsToMM(kinematics.x_pos));
  //
  Serial.print(" - Left Sensor - ");
  Serial.println(state);
  //PID WORKING OUT

  if (stateTimer + 2500 < micros()) {
    Brain();
    //SetPower(lineSensors.FollowLineSensorReadings());
    stateTimer = micros();
  }

  if (timer + 10000 < micros()) {
    kinematics.VelocityEstimation();

    timer = micros();

  }
}
void SetPower(float normalVal) {
  normalVal = turning_PID.update(normalVal,currentTurnValue);
  currentTurnValue = normalVal;
  float leftPWM = MAX_PWM - (normalVal * MAX_TURN_PWM);
  float rightPWM = MAX_PWM + (normalVal * MAX_TURN_PWM);
  //float leftPWM = MAX_SPEED - (normalVal * MAX_TURN_SPEED);
  //float rightPWM = MAX_SPEED + (normalVal * MAX_TURN_SPEED);
  //Serial.print("Left Power: ");
  //Serial.print(leftPWM);
  //Serial.print(" - Right Power: ");
  //Serial.println(rightPWM);
  float lVal = left_PID.update(leftPWM, motors.current_l_speed);
  float rVal = right_PID.update(rightPWM, motors.current_r_speed);

  //float lVal = left_PID.update(leftPWM, -kinematics.e1Vel) * 0.1f;
  //float rVal = right_PID.update(rightPWM, -kinematics.e0Vel) * 0.1f;

  //Serial.print(("Left PWM is: "));
  //Serial.print(lVal);
  //Serial.print((" - Right PWM is: "));
  //Serial.print(-kinematics.e1Vel);
  //Serial.print((" - Normal Value is: "));
  //Serial.print(leftPWM);
  //Serial.print((" - Intergral Value is: "));
  //Serial.print(left_PID.integral);
  //Serial.print((" - PTerm Value is: "));
  //Serial.print(left_PID.p_term);
  //Serial.print((" - Derivitive Value is: "));
  //Serial.println(left_PID.derivitive);
  //motors.current_l_speed += lVal;
  motors.SetMotorPower(motors.current_l_speed + lVal, motors.current_r_speed + rVal);
  //motors.SetMotorPower(leftPWM,rightPWM);
}


void Brain() {

  if (state == BEGIN) {
    BeginState();
    return;
  }
  if (state == FOLLOWING) {
    LineFollowState();
    return;
  }
  if (state == TURN_LEFT) {
    TurnLeftState();
    return;
  }
  if (state == PREP_TURN_LEFT){
    PrepTurnLeft();
    return;
  }
  if (state == TURN_AROUND){
    TurnAround();
    return;
  }
  if(state == DO_NOTHING){
    //digitalWrite(ORANGE_LED_PIN, HIGH);
    PrepTurn();
    return;
  }
  if(state == RETURN_HOME){
    GoHome();
    return;
  }
  if(state == ROTATE_HOME){
    RotateHome();
    return;
  }
}

void BeginState() {
  if (state == BEGIN) {
    if (!skippedLine && !foundLine) {
      SetPower(0);
    }
    if (!skippedLine && lineSensors.IsOnLine()) {
      Serial.println("workin");
      SetPower(0);
      foundLine = true;
    }
    if (!skippedLine && foundLine && !lineSensors.IsOnLine()) {
      skippedLine = true;
      foundLine = false;
    }
    if (skippedLine && lineSensors.IsOnLine()) {
      foundLine = true;
      motors.Stop();
      state = FOLLOWING;
    }
  }
}

void LineFollowState() {
  digitalWrite(GREEN_LED_PIN, HIGH);

  SetPower(lineSensors.FollowLineSensorReadings());
  if (lineSensors.ReadLeftMostSensor() > 1200) {
    digitalWrite(GREEN_LED_PIN, LOW);
    motors.Stop();
    //state = TURN_LEFT;
    state = PREP_TURN_LEFT;
    Serial.println("TURNING LEFT?");
    
    return;
  }

  if (lineSensors.IsOnLine() == false) {
    if(millis()/1000 > 40){
      digitalWrite(RED_LED_PIN,HIGH);
      motors.Stop();
      state = ROTATE_HOME;
      kinematics.theta = 0;
      wantedAngle = atan2(kinematics.y_pos, kinematics.x_pos);
    }
    state = TURN_AROUND;
    needed_Count_L = count_e0 - 20;
    needed_Count_R = count_e1 - 20;
    wantedAngle = kinematics.theta + 25;

    digitalWrite(GREEN_LED_PIN, LOW);
    return;
  }
}

void TurnLeftState() {
  Serial.println("Turn Left State");
  motors.SetMotorPower(-30, 30);
  if (kinematics.theta < wantedAngle) {
    Serial.println("Finished Turning");
    if (lineSensors.IsOnLine()) {
    state = FOLLOWING;
    return;
    } 
  }
}

void PrepTurnLeft(){
  //SetPower(0);
  //if(lineSensors.ReadLeftMostSensor() < 1000){
    motors.Stop();
    wantedAngle = kinematics.theta - 30;
    state = TURN_LEFT;
    return;
  //}
}

void TurnAround(){
  motors.SetMotorPower(30, -30);
  if(kinematics.theta > wantedAngle){
    if(lineSensors.IsOnLine()) state = FOLLOWING;
  }
}

void PrepTurn(){
  digitalWrite(RED_LED_PIN, HIGH);
  motors.SetMotorPower(18, 18);
  if(needed_Count_L > count_e1 && needed_Count_R > count_e0){
    motors.Stop();
    state = TURN_AROUND;
  }
}

void RotateHome(){
  motors.SetMotorPower(30,-30);
  if(kinematics.theta > wantedAngle){
    motors.Stop();
    x_length_home = sqrt(sq(kinematics.x_pos) + sq(kinematics.y_pos));
    kinematics.theta = 0;
    state = RETURN_HOME;
  }
}

void GoHome(){
  motors.SetMotorPower(20, 20);
  if(kinematics.x_pos > x_length_home){
    motors.Stop();
  }
}

//int needed_Count_L = 0;
//int needed_Count_R = 0;


void GetMotorValues(int& left, int& right) {
  unsigned long midSensorValue = lineSensors.ReadMiddleSensor();
  unsigned long rightMostSensorValue = lineSensors.ReadRightMostSensor();
  unsigned long leftMostSensorValue = lineSensors.ReadLeftMostSensor();
  unsigned long rightSensorValue = lineSensors.ReadRightSensor();
  unsigned long leftSensorValue = lineSensors.ReadLeftSensor();


  //TODO: REWRITE TO USE LM ND RM
  // focus on keeping in the centre line
  // need to record left anf righ t turns
  if (midSensorValue > 1000) {
    left = 20;
    right = 20;
  }
  if (leftMostSensorValue > 1000) {
    right += 5;
    left -= 5;
  }
  if (rightMostSensorValue > 1000) {
    left += 5;
    right -= 5;
  }
  if (leftSensorValue > 1000) {
    right = 10;
    left = 0;
    return;
  }
  if (rightSensorValue > 1000) {
    left = 10;
    right = 0;
    return;
  }

  left = 0;
  right = 0;
}
