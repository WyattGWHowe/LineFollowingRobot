// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef encoders.h
#define _PID_H

float p_term = 0;
float feedback_signal = 0;
float gain = 0;

float previousRun = 0;
float previousError = 0;
float integral = 0;
float integralGain = 0;


//timer count
//long timeSinceLastUpdate = 0;


// Class to contain generic PID algorithm.
class PID_c {
  public:
  
    // Constructor, must exist.
    PID_c() {

    } 

    void initialize(float gainValue){
      //timeSinceLastUpdate = 0;
      p_term = 0;
      feedback_signal = 0;
      integral = 0;
      gain = gainValue;
      previousError = millis();
      integralGain = 0.001f;
    }

    float update(float demand, float measurement){
      float e = demand - measurement; //Set the error
      float dt = millis() - previousRun; //set the delta time
      p_term = gain * e; //set the proportional term
      integral = integral + e * dt;

      float returnValue = gain * e + integralGain * integral;

      return p_term;



      previousRun = millis();
      return returnValue;
    }

    


};



#endif
