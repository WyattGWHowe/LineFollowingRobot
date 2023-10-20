// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define EMIT_PIN 11
#define LS_LEFTMOST_PIN 12
#define LS_LEFT_PIN 18
#define LS_MID_PIN 20
#define LS_RIGHT_PIN 21
#define LS_RIGHTMOST_PIN 22

// Class to operate the linesensor(s).
class LineSensor_c {
  public:
  
    // Constructor, must exist.
    LineSensor_c() {

    } 

    void initialize(){
      pinMode(EMIT_PIN, OUTPUT);
      pinMode(LS_LEFTMOST_PIN, INPUT);
      pinMode(LS_LEFT_PIN, INPUT);
      pinMode(LS_MID_PIN, INPUT);
      pinMode(LS_RIGHT_PIN,INPUT);
      pinMode(LS_RIGHTMOST_PIN, INPUT);
    }

    //Refactor this into a single function that can take in a int which is usng 

    unsigned long ReadMiddleSensor(){
      //Emit to HIGH and OUTPUT
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN,HIGH);
      //Sensor to HIGH and OUTPUT
      pinMode(LS_MID_PIN, OUTPUT);
      digitalWrite(LS_MID_PIN,HIGH);
      //delay 10 microseconds
      delayMicroseconds(10);
      //Sensor to INPUT
      pinMode(LS_MID_PIN, INPUT);
      
      //capture start time
      unsigned long startTime = micros();
      //wait til sensor reads low
      while(digitalRead(LS_MID_PIN) != LOW){
        if(micros() - startTime >= 2000) break;
      }
      //capture end time
      unsigned long endTime = micros();

      pinMode(EMIT_PIN,INPUT);
      unsigned long elapsedTime = endTime - startTime;
      return elapsedTime;


    }

    unsigned long ReadLeftSensor(){
      //Emit to HIGH and OUTPUT
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN,HIGH);
      //Sensor to HIGH and OUTPUT
      pinMode(LS_LEFT_PIN, OUTPUT);
      digitalWrite(LS_LEFT_PIN,HIGH);
      //delay 10 microseconds
      delayMicroseconds(10);
      //Sensor to INPUT
      pinMode(LS_LEFT_PIN, INPUT);
      
      //capture start time
      unsigned long startTime = micros();
      //wait til sensor reads low
      while(digitalRead(LS_LEFT_PIN) != LOW){
        //wait
        if(micros() - startTime >= 2000) break;
      }
      //capture end time
      unsigned long endTime = micros();

      pinMode(EMIT_PIN,INPUT);
      unsigned long elapsedTime = endTime - startTime;
      return elapsedTime;


    }
    unsigned long ReadRightSensor(){
      //Emit to HIGH and OUTPUT
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN,HIGH);
      //Sensor to HIGH and OUTPUT
      pinMode(LS_RIGHT_PIN, OUTPUT);
      digitalWrite(LS_RIGHT_PIN,HIGH);
      //delay 10 microseconds
      delayMicroseconds(10);
      //Sensor to INPUT
      pinMode(LS_RIGHT_PIN, INPUT);
      
      //capture start time
      unsigned long startTime = micros();
      //wait til sensor reads low
      while(digitalRead(LS_RIGHT_PIN) != LOW){
        //wait
        if(micros() - startTime >= 2000) break;
      }
      //capture end time
      unsigned long endTime = micros();

      pinMode(EMIT_PIN,INPUT);
      unsigned long elapsedTime = endTime - startTime;
      return elapsedTime;


    }

    unsigned long ReadLeftMostSensor(){
      //Emit to HIGH and OUTPUT
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN,HIGH);
      //Sensor to HIGH and OUTPUT
      pinMode(LS_LEFTMOST_PIN, OUTPUT);
      digitalWrite(LS_LEFTMOST_PIN,HIGH);
      //delay 10 microseconds
      delayMicroseconds(10);
      //Sensor to INPUT
      pinMode(LS_LEFTMOST_PIN, INPUT);
      
      //capture start time
      unsigned long startTime = micros();
      //wait til sensor reads low
      while(digitalRead(LS_LEFTMOST_PIN) != LOW){
        //wait
        if(micros() - startTime >= 2000) break;
      }
      //capture end time
      unsigned long endTime = micros();

      pinMode(EMIT_PIN,INPUT);
      unsigned long elapsedTime = endTime - startTime;
      return elapsedTime;


    }
    unsigned long ReadRightMostSensor(){
      //Emit to HIGH and OUTPUT
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN,HIGH);
      //Sensor to HIGH and OUTPUT
      pinMode(LS_RIGHTMOST_PIN, OUTPUT);
      digitalWrite(LS_RIGHTMOST_PIN,HIGH);
      //delay 10 microseconds
      delayMicroseconds(10);
      //Sensor to INPUT
      pinMode(LS_RIGHTMOST_PIN, INPUT);
      
      //capture start time
      unsigned long startTime = micros();
      //wait til sensor reads low
      while(digitalRead(LS_RIGHTMOST_PIN) != LOW){
        //wait
        if(micros() - startTime >= 2000) break;
      }
      //capture end time
      unsigned long endTime = micros();

      pinMode(EMIT_PIN,INPUT);
      unsigned long elapsedTime = endTime - startTime;
      return elapsedTime;


    }

    //returns a normalized value between -1 : 1 
    float FollowLineSensorReadings(){
      unsigned long rightSensorValue = ReadRightSensor();
      unsigned long middleSensorValue = ReadMiddleSensor();
      unsigned long leftSensorValue = ReadLeftSensor();

      float maxValue = rightSensorValue + leftSensorValue; 

      float rightNormal = (rightSensorValue/maxValue) * 2;
      float leftNormal = (leftSensorValue/maxValue) * 2;

      return (leftNormal - rightNormal);
      
    }
};



#endif
