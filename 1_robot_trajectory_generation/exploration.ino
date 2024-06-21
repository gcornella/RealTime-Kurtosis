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
const int rotStepsPerRevolution = (3828/4)*4;
const int extStepsPerRevolution = (3828/4)*4;

// Considering Step Angle for the motor without gearbox: 1.8 deg
// for 360 degree is 200 steps in full step
// to calculate for 180º divide by 2
// supination motor set to (slow = 1/32th step mode), (fast = 1/32th)
const int supStepsPerRevolution = (200/2)*32;

// Set the speed (steps per second) (divide by seconds
const int rotSpeed = rotStepsPerRevolution;
const int extSpeed = extStepsPerRevolution;
const int supSpeed = supStepsPerRevolution;

void setup() {
  // Serial.begin(115200);
  
  // Declare pins as Outputs for motor
  pinMode(stepPin_rot, OUTPUT); 
  pinMode(dirPin_rot, OUTPUT);
  pinMode(stepPin_ext, OUTPUT);
  pinMode(dirPin_ext, OUTPUT);
  pinMode(stepPin_sup, OUTPUT);
  pinMode(dirPin_sup, OUTPUT);

  // Set speed and position for motors
  rotMotor.setMaxSpeed(rotSpeed);
  extMotor.setMaxSpeed(extSpeed);
  supMotor.setMaxSpeed(supSpeed);
  rotMotor.setAcceleration(rotSpeed);
  extMotor.setAcceleration(extSpeed);
  supMotor.setAcceleration(supSpeed);
  
  rotMotor.moveTo(-rotStepsPerRevolution);
  extMotor.moveTo(extStepsPerRevolution);
  supMotor.moveTo(-supStepsPerRevolution);

  delay(5000);
}

int switch_direction_count = 0;
int reps = 0;

void loop() {
 
  while (reps<15) {
    rotMotor.run();
    extMotor.run();
    supMotor.run();
    
    if (rotMotor.distanceToGo()==0 && extMotor.distanceToGo()==0 && supMotor.distanceToGo()==0){
      ++switch_direction_count;
      if (switch_direction_count % 2 == 0) {
        reps +=1; // add 1 to the reps counter     
      }
      delay(100);
      rotMotor.moveTo(-rotMotor.currentPosition()-rotStepsPerRevolution);
      extMotor.moveTo(-extMotor.currentPosition()+extStepsPerRevolution);
      supMotor.moveTo(-supMotor.currentPosition()-supStepsPerRevolution);
    }
  }
  Serial.println("Trajectory finished after n repetitions");


}
