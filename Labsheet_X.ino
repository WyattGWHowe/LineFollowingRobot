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

float MAX_PWM = 25;
float MAX_TURN_PWM = 50;


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
// state enum
enum State_enum {
  FOLLOWING = 1,
  BACK_TO_START = 2
};

State_enum state;


//rotatiom speeds - refactor into kinematics.h
long timeSinceLastUpdate = 0;
long encoderCount_e0 = 0;
long encoderCount_e1 = 0;

//timer count
long timer = 0;



// put your setup code here, to run once:
void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  Serial.println(sin(0));
  state = 1;
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
  right_PID.initialize(0.2f);
  left_PID.initialize(0.2f);
  turning_PID.initialize(0.2f);


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

  encoderCount_e0 = count_e0;
  encoderCount_e1 = count_e1;
  timeSinceLastUpdate = micros();
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

void VelocityEstimation() {
  long newEncoderCount_e0 = count_e0;
  long newEncoderCount_e1 = count_e1;

  long time = micros() / 1000000;

  long diff_e0 = newEncoderCount_e0 - encoderCount_e0;
  long diff_e1 = newEncoderCount_e1 - encoderCount_e1;

  Serial.print("micros is: ");
  Serial.println(time);

  long velocity_e0 = diff_e0 / (time - (timer / 1000000));
  long velocity_e1 = diff_e1 / (time - (timer / 1000000));

  Serial.print("Velocity_e0 = ");
  Serial.println(diff_e0);

  Serial.print("Velocity_e1 = ");
  Serial.println(diff_e1);


  kinematics.work_out_rots(diff_e1, diff_e0);


  Serial.print("[X,Y,THETA] ");
  Serial.print(kinematics.x_pos);
  Serial.print(",");
  Serial.print(kinematics.y_pos);
  Serial.print(",");
  Serial.println(kinematics.theta);  // * (180.0f/3.14f));// * (180/3.14f));

  Serial.print("[Right, Left] ");
  Serial.print(count_e0);
  Serial.print(",");
  Serial.println(count_e1);
  //Serial.print(",");
  //Serial.println(kinematics.theta);
  timeSinceLastUpdate = micros();
  encoderCount_e0 = count_e0;
  encoderCount_e1 = count_e1;
}



// put your main code here, to run repeatedly:
void loop() {

  //kinematics.update();

  //SetPower(lineSensors.FollowLineSensorReadings());
  if (kinematics.theta  < 90.0f) {
    motors.SetMotorPower(30, -30);

    //VelocityEstimation();
  } else {
    //Serial.println("I SHOULD STOP");
    motors.SetMotorPower(0, 0);
  }
  //
  //Serial.print("PID reading: ");
  //Serial.println(PID.update(0, lineSensors.FollowLineSensorReadings()));
  if (timer + 7000 < micros()) {
    VelocityEstimation();
    timer = micros();
    //Serial.print("Total Turned: ");
    //Serial.println(kinematics.TurnsToMM(count_e0));
  }
}
void SetPower(float normalVal) {
  normalVal = turning_PID.update(0, normalVal);
  float leftPWM = MAX_PWM - (normalVal * MAX_TURN_PWM);
  float rightPWM = MAX_PWM + (normalVal * MAX_TURN_PWM);
  //Serial.print("Left Power: ");
  //Serial.print(leftPWM);
  //Serial.print(" - Right Power: ");
  //Serial.println(rightPWM);
  //motors.SetMotorPower(leftPWM, rightPWM);
  //motors.SetMotorPower(leftPWM,rightPWM);
}




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
