// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#define PI 3.1415926535897932384626433832795

/*
  NOTES
    wheel radius is 16mm
*/









// Class to track robot position.
class Kinematics_c {
public:
  float x_pos = 0;
  float y_pos = 0;

  float e0Vel = 0;
  float e1Vel = 0;

  float theta = 0;

  long encoderCount_e0 = 0;
  long encoderCount_e1 = 0;
  unsigned long previousTime = 0;

  const float wheelRad = 16.0f;
  const float CPR = 358.5f;
  const float wheel_dis_from_centre = 40.5f;
  float rot_per_turn = 0;
  // Constructor, must exist.
  Kinematics_c() {
  }

  void initialize() {
    //this isnt needed but nice to check
    x_pos = 0;
    y_pos = 0;
    theta = 0;
    rot_per_turn = 358.5 / 360;
  }

  // Use this function to update
  // your kinematics
  void update() {
    //Serial.print("rot of R : ");
    //Serial.println(work_out_rots());
  }

  //float work_out_rots(float leftVel, float rightVel) {

    //float radiusAngVelRight = wheelRad * TurnsToMM(rightVel);
    //float radiusAngVelLeft = wheelRad * TurnsToMM(leftVel);

    //leftVel = TurnsToMM(leftVel);
    //rightVel = TurnsToMM(rightVel);



    //return theta;
  //}

  long TurnsToMM(float turns) {
    //float turnsNormal = turns % 358.5f;
    float turnsPercent = turns / CPR;
    //float turnsIntoDeg = turnsPercent*360.0f;


    long circumfrance = 2 * PI * wheelRad;
    long totalDistance = (turnsPercent)*circumfrance;
    return totalDistance;
  }

  void VelocityEstimation() {
    long newEncoderCount_e0 = count_e0;
    long newEncoderCount_e1 = count_e1;

    long time = micros();

    long diff_e0 = newEncoderCount_e0 - encoderCount_e0;
    long diff_e1 = newEncoderCount_e1 - encoderCount_e1;

    long changeInTime = time - previousTime;

    long velocity_e0 = diff_e0 / (changeInTime/1000000);
    long velocity_e1 = diff_e1 / (changeInTime/1000000);

    e0Vel = velocity_e0;
    e1Vel = velocity_e1;

    previousTime = micros();
    encoderCount_e0 = count_e0;
    encoderCount_e1 = count_e1;

    long rightVel = diff_e0;
    long leftVel = diff_e1;

    float Xr = wheelRad * (leftVel + rightVel) / 2.0f;
    float Yr = 0;



    float theta_temp = wheelRad * ((rightVel - leftVel) / (2 * wheel_dis_from_centre));

    x_pos += Xr * cos(theta);
    y_pos += Xr * sin(theta);
    theta += theta_temp;

    //work_out_rots(diff_e1, diff_e0);


  }
};



#endif
