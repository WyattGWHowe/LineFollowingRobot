// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H

//Motor Controls
#define LEFT_MOTOR_PWM 10 
#define LEFT_MOTOR_DIRECTION 16
#define RIGHT_MOTOR_PWM 9
#define RIGHT_MOTOR_DIRECTION 15



// Class to operate the motor(s).
class Motors_c {
  public:

  float current_l_speed = 0;
  float current_r_speed = 0;
    // Constructor, must exist.
    Motors_c() {

    } 

    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    void initialise() {
      pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
      pinMode(RIGHT_MOTOR_PWM, OUTPUT);
      pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
      pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);

      digitalWrite(RIGHT_MOTOR_DIRECTION, LOW);
      digitalWrite(LEFT_MOTOR_DIRECTION, LOW);

      analogWrite(RIGHT_MOTOR_PWM, 0);
      analogWrite(LEFT_MOTOR_PWM, 0);




    }


    //Function to cease power to the motors
void Stop(){
  analogWrite(RIGHT_MOTOR_PWM, 0);
  analogWrite(LEFT_MOTOR_PWM, 0);
}

int SetMotorPower(float lftpwm, float rhtpwm){
  int lftDir = lftpwm > 0 ? 0 : 1;
  int rightDir = rhtpwm > 0 ? 0 : 1;

  SetMotorDirection(lftDir, rightDir);

  current_l_speed = lftpwm;
  current_r_speed = rhtpwm;

  lftpwm = lftpwm < 0 ? lftpwm * -1 : lftpwm;
  rhtpwm = rhtpwm < 0 ? rhtpwm * -1 : rhtpwm;



  analogWrite(RIGHT_MOTOR_PWM, rhtpwm );
  analogWrite(LEFT_MOTOR_PWM, lftpwm );



  return lftpwm;
}

void SetPowerDirect_DEBUG(int lftpwm, int rhtpwm){
  analogWrite(RIGHT_MOTOR_PWM, rhtpwm);
  analogWrite(LEFT_MOTOR_PWM, lftpwm); 
}

void SetMotorDirection(int lftdir, int rightdir){
  digitalWrite(LEFT_MOTOR_DIRECTION, lftdir);
  digitalWrite(RIGHT_MOTOR_DIRECTION,rightdir);
}
};



#endif
