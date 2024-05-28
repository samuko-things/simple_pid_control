#include "simple_pid_control.h"

///////////////////////////////////////////////
double outMin = -255.0, outMax = 255.0;
double kp = 0.0, ki = 0.0, kd = 0.0;
double output; // variable to store the output of the PID computation
double target = 0.00; // change as desired

// motor pid control
SimplePID pidMotor(kp, ki, kd, outMin, outMax);

// set the frequncy/time at which the PID loop will run
unsigned long pidTime, pidSampleTime = 20; //ms -> (1000/sampleTime) hz (i.e 50Hz)
/////////////////////////////////////////////

// Don't forget to add necessary motor related variable


void setup() {
  Serial.begin(115200);

  pidMotor.begin();
  pidInit();

  // Don't forget to initialize your motor

  pidTime = millis();
}


void loop() {

  ////////////// PID OPERATION /////////////////////////////////
  if ((millis() - pidTime) >= pidSampleTime) {
    // change the actual value to that which you want to check and control
    // against the target (e.g the sensor resding)
    output = pidMotorA.compute(target, actual_value); 
    motor.sendPWM((int)output); // change this to your motor send pwm function
    pidTime = millis();
  }
  //////////////////////////////////////////////////////////////////////////

}
