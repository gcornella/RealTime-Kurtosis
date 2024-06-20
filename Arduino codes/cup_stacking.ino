#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>

// motor pins
// pins for motor driving shoulder rotation
const int dirPin_rot = 2;
const int stepPin_rot = 3;
// pins for motor driving elbow flexion and extension
const int dirPin_ext = 4;
const int stepPin_ext = 5;
// pins for motor driving wrist pronation and supination
const int dirPin_sup = 6;
const int stepPin_sup = 7;

// defining stepper motors
AccelStepper rotMotor(1, stepPin_rot, dirPin_rot);
AccelStepper extMotor(1, stepPin_ext, dirPin_ext);
AccelStepper supMotor(1, stepPin_sup, dirPin_sup);

// SLOW: step modes = {16, 16, 32} , speed /6 , accel*1
// FAST: step modes = {4, 4, 32}   , speed /1 , accel*1

// Considering Step Angle for motors w/ gearbox: 0.094 deg
// for 360 degree is 3829.78723 steps in full step
// to calculate for 90º divide by 4
// rotation and extension motor set to (slow = 1/16th step mode), (fast = 1/4th)
const int rotStepsPerRevolution = (3828/4)*16;
const int extStepsPerRevolution = (3828/4)*16;

// Considering Step Angle for the motor without gearbox: 1.8 deg
// for 360 degree is 200 steps in full step
// to calculate for 180º divide by 2
// supination motor set to (slow = 1/32th step mode), (fast = 1/32th)
const int supStepsPerRevolution = (200/2/3.6)*32;

// Set the speed (steps per second) (divide by seconds
const int rotSpeed = rotStepsPerRevolution/6;
const int extSpeed = extStepsPerRevolution/6;
const int supSpeed = supStepsPerRevolution/6;

void setup() {
  // Serial.begin(115200);
  // Declare pins as Outputs for motor
  pinMode(stepPin_rot, OUTPUT); 
  pinMode(dirPin_rot, OUTPUT);
  pinMode(stepPin_ext, OUTPUT);
  pinMode(dirPin_ext, OUTPUT);
  pinMode(stepPin_sup, OUTPUT);
  pinMode(dirPin_sup, OUTPUT);

  
  // Set the initial conditions
  rotMotor.setMaxSpeed(rotSpeed);
  extMotor.setMaxSpeed(extSpeed);
  supMotor.setMaxSpeed(supSpeed);
  rotMotor.setAcceleration(rotSpeed);
  extMotor.setAcceleration(extSpeed);
  supMotor.setAcceleration(supSpeed);

  // Set the home position as 0
  rotMotor.setCurrentPosition(0);
  extMotor.setCurrentPosition(0);
  supMotor.setCurrentPosition(0);
  
  delay(3000);
  
  // Move the motors to the starting position
  rotMotor.moveTo(-rotStepsPerRevolution);
  while(rotMotor.distanceToGo() != 0){
    rotMotor.run();
    }
  extMotor.moveTo(extStepsPerRevolution);
  while(extMotor.distanceToGo() != 0){
    extMotor.run();
    }
  supMotor.moveTo(-supStepsPerRevolution);
  while(supMotor.distanceToGo() != 0){
    supMotor.run();
    }
  delay(2000);
  
  // Reset the initial conditions again!
  rotMotor.setMaxSpeed(rotSpeed);
  extMotor.setMaxSpeed(extSpeed);
  supMotor.setMaxSpeed(supSpeed);
  rotMotor.setAcceleration(rotSpeed);
  extMotor.setAcceleration(extSpeed);
  supMotor.setAcceleration(supSpeed);
  
  // Re-define the initial position to ease the loop code
  rotMotor.setCurrentPosition(-rotStepsPerRevolution);
  extMotor.setCurrentPosition(-extStepsPerRevolution);
  supMotor.setCurrentPosition(0);

  rotMotor.moveTo(-0.5*rotStepsPerRevolution);
  extMotor.moveTo(-1.5*extStepsPerRevolution);

  delay(3000);
  
}

int phase = 1;
int reps = 0;

void loop() {
  while (reps<15) {
    rotMotor.run();
    extMotor.run();
    
    switch(phase) {
      case 1:
        if (rotMotor.distanceToGo()==0 && extMotor.distanceToGo()==0){
            phase = 2;
            rotMotor.moveTo(0);
            extMotor.moveTo(-extStepsPerRevolution);
          }
    
      case 2:
        if (rotMotor.distanceToGo()==0 && extMotor.distanceToGo()==0){
            phase = 3;
            rotMotor.moveTo(-rotStepsPerRevolution);
            extMotor.moveTo(-extStepsPerRevolution);
            
          }   
      case 3:
        if (rotMotor.distanceToGo()==0 && extMotor.distanceToGo()==0){
            phase = 1;
            rotMotor.moveTo(-0.5*rotStepsPerRevolution);
            extMotor.moveTo(-1.5*extStepsPerRevolution);  
            reps +=1; 
            delay(100);         
            
          }
    }
  }
  // Serial.println("Trajectory finished after n repetitions");
}
