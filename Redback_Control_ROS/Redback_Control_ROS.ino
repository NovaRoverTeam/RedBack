#include <ros.h>
#include <AccelStepper.h>
#include <Encoder.h>





///////////////////////////// Defitions ////////////////////////////////////////
//DOUBLE CHECK PINS
#define encoderPin1 2
#define encoderPin2 3

#define stepperEnablePin 4
#define stepperDirPin 5
#define stepperStepPin 6

#define sensorDirPin 11
#define sensorPWMPin 12

#define actuatorDirPin 9
#define actuatorPWMPin 10

#define drillDirPin 7
#define drillPWMPin 8

#define stepperSpeed 300
#define steppperMaxPos 1000
#define stepperAccel 100

#define cacheActuatorPos 300

#define pwmRangeConstant 20
#define stepperWorkaround 10

//#define ROSSTUFF //UNCOMMENT THIS LINE WHEN ROS IS REQUIRED FOR COMPILE

///////////////////////////// Variables ////////////////////////////////////////

bool isDrillSpinning = false;
bool isActuatorMoving = false;
bool isCalibrating = false;
bool actuatorState = false; //Relative Move:False, Absolute Move:True


int curActuatorPos = -999;
int oldActuatorPos = -999;
int curActuatorSpeed = 0;

int curDrillSpeed = 0;
int curSensorSpeed = 0;

int requestedActuatorPos;
int requestedActuatorSpeed;
int requestedStepperCmd;
int requestedDrillSpeed;
int requestedSensorSpeed;

////////////////////////////// ROS CALLBACK ////////////////////////////////////////
#ifdef ROSSTUFF

#include <rover/RedCmd.h>
// Declare required ROS variables
ros::NodeHandle  nh;

void msgCallback (const rover::RedCmd& msg){

  //KILL SWITCH
  if (msg.kill == 1){
    killAll();
  }

  //DRILL STUFF
  requestedDrillSpeed = msg.drillSpd;

  //Actuator Stuff
  requestedActuatorSpeed = msg.actuatorSpeed;

  //Sensor Stuff
  requestedSensorSpeed = msg.sensorSpeed;
  
  if (requestedActuatorSpeed == 300){ 
    actuatorState = true; //Absolute Move
    //requestedActuatorPos = cacheActuatorPos;
    requestedActuatorSpeed = 0;
  }else{
    actuatorState = false; //Relative Move
  }

  //STEPPER STUFF
  requestedStepperCmd = msg.stepperPos;
  stepperRun();

}

ros::Subscriber<rover::RedCmd> red_sub("red_cmd_data", &msgCallback);

#endif
//////////////////////////////// Motor Setup /////////////////////////////////////////

AccelStepper stepper(AccelStepper::DRIVER, stepperStepPin, stepperDirPin);

Encoder myEnc(encoderPin1, encoderPin2);

//////////////////////////////// SETUP /////////////////////////////////////////
void setup() {
  
  #ifdef ROSSTUFF
    nh.initNode();
    nh.subscribe(red_sub);
    nh.loginfo("Calling Setup function.");
  #endif
  
  //Serial.begin(57600);

  pinMode(drillDirPin, OUTPUT);
  pinMode(drillPWMPin, OUTPUT);
  pinMode(sensorDirPin, OUTPUT);
  pinMode(sensorPWMPin, OUTPUT);
  pinMode(actuatorDirPin, OUTPUT);
  pinMode(actuatorPWMPin, OUTPUT);

  //stepper.setEnablePin(13);
  //stepper.enableOutputs();
  stepper.setMaxSpeed(stepperSpeed);
  stepper.setSpeed(500);
  stepper.setAcceleration(stepperAccel);

  delay(2000);

  calibrateSystem();

  stepper.moveTo(200);


}


//////////////////////////////////// Main Software Loop /////////////////////////////////////
void loop() {
  

  /*//Update Encoder position as the current actuator pos
  long curActuatorPos = myEnc.read();
  if (curActuatorPos != oldActuatorPos) {
    oldActuatorPos = curActuatorPos;
    #ifdef ROSSTUFF
      nh.loginfo("Drill spinning.");
    #endif
    //Serial.println("ActuatorPos: " + curActuatorPos);
  }*/

  
  drillRun();
  actuatorRun();
  sensorRun();


  //for(int i = 0; i < stepperWorkaround; i++){
    
    stepper.run();
    
  //}
  
  

  
  #ifdef ROSSTUFF
    nh.spinOnce();
  #endif
}


//////////////////////////////////// Functionalities /////////////////////////////////////



void stepperRun(){
 

    if(!isActuatorMoving){ //Only move Sidesways if actuator isnt moving

      stepper.move(requestedStepperCmd*50);
      //Serial.println("Stepper Moving");

    }
  
}


void sensorRun(){

  int spd;
  //if (requestedSensorSpeed != curSensorSpeed){
  if ((requestedSensorSpeed >= curSensorSpeed + pwmRangeConstant) || (requestedSensorSpeed <= curSensorSpeed - pwmRangeConstant)){
    if (requestedSensorSpeed > 0) { // Clockwise Rotations
  
      curSensorSpeed = requestedSensorSpeed;
      digitalWrite(sensorDirPin, LOW);
      analogWrite(sensorPWMPin, requestedSensorSpeed);
      //nh.loginfo("Sesor spinning.");

  
    } else if (requestedSensorSpeed < 0) {
  
      curSensorSpeed = requestedSensorSpeed;
      digitalWrite(sensorDirPin, HIGH);
      spd = abs(requestedSensorSpeed);
      analogWrite(sensorPWMPin, spd);
      //nh.loginfo("Sensor spinning.");
  
    } else {
  
      digitalWrite(sensorDirPin, LOW);
      analogWrite(sensorPWMPin, 0);
      //nh.loginfo("Sensor stopped.");
    }
    
  }

  
}

void drillRun() {
  int spd;
  //if (requestedDrillSpeed != curDrillSpeed){
  if ((requestedDrillSpeed >= curDrillSpeed + pwmRangeConstant) || (requestedDrillSpeed <= curDrillSpeed - pwmRangeConstant)){
    
    if (requestedDrillSpeed > 0) { // Clockwise Rotations
  
      isDrillSpinning = true;
      curDrillSpeed = requestedDrillSpeed;
      digitalWrite(drillDirPin, LOW);
      analogWrite(drillPWMPin, requestedDrillSpeed);
      //nh.loginfo("Drill spinning.");

  
    } else if (requestedDrillSpeed < 0) {
  
      isDrillSpinning = true;
      curDrillSpeed = requestedDrillSpeed;
      digitalWrite(drillDirPin, HIGH);
      spd = abs(requestedDrillSpeed);
      analogWrite(drillPWMPin, requestedDrillSpeed);
      //nh.loginfo("Drill spinning.");
  
    } else {
  
      isDrillSpinning = false;
      digitalWrite(drillDirPin, LOW);
      analogWrite(drillPWMPin, 0);
      //nh.loginfo("Drill stopped.");
    }
    
  }
}


void moveActuatorAbs(int pos) {

  requestedActuatorPos = pos;
  actuatorRun();

}

void actuatorRun(){

  if (actuatorState == false){
    moveActuatorRel();
  }else{

    int difference = requestedActuatorPos - curActuatorPos;
    int spdV = 100; // Change how fast the actuator Moves
    int howClose = 10; //If it has trouble going to 

    //spdV = div(difference,2); // RATE TEST value for 500 length of actuator
    
    if(difference != 0){
    
      if (difference > howClose) { //Drill is Lower than requested pos, Move Up
        requestedActuatorSpeed = spdV;
        moveActuatorRel();
      
      } else if (difference < howClose) { //Encoder is Higher than requested Pos, Move Down
        
        requestedActuatorSpeed = -spdV;
        moveActuatorRel();
    
      } else {
        //Set Motor OFF
        requestedActuatorSpeed = 0;
        moveActuatorRel();
      }
   }
  }
}

void moveActuatorRel() {
  int spd;
  //if (requestedActuatorSpeed != curActuatorSpeed){
  if ((requestedActuatorSpeed >= curActuatorSpeed + pwmRangeConstant) || (requestedActuatorSpeed <= curActuatorSpeed - pwmRangeConstant)){
    if (requestedActuatorSpeed > 0) { // Clockwise Rotations

    isActuatorMoving = true;
    curActuatorSpeed = requestedActuatorSpeed;
    digitalWrite(actuatorDirPin, HIGH);
    analogWrite(actuatorPWMPin, requestedActuatorSpeed);
    //nh.loginfo("Actuator moving.");

    } else if (requestedActuatorSpeed < 0) { // Counter Clockwise Rotations

    isActuatorMoving = true;
    curActuatorSpeed = requestedActuatorSpeed;
    digitalWrite(actuatorDirPin, LOW);
    spd = abs(requestedActuatorSpeed);
    analogWrite(actuatorPWMPin, spd);
    //nh.loginfo("Actuator moving.");

    } else { //Not Moving

    isActuatorMoving = false;
    digitalWrite(actuatorDirPin, LOW);
    analogWrite(actuatorPWMPin, 0);
    curActuatorSpeed = 0;
    ////nh.loginfo("Actuator stopped.");

    }
  }
}


void calibrateSystem() {
  
  isCalibrating = true;

  //requestedActuatorSpeed = 100; // calibration to set to top
  //moveActuatorRel(); //Full Speed up
  //delay(5000); //Make sure actuator reaches top endstop
  //curActuatorPos = 0;
  
  stepper.setCurrentPosition(0);

  

}



void killAll() {

  //Implement Kill switch
  digitalWrite(stepperEnablePin, LOW);
  digitalWrite(stepperDirPin, LOW);
  digitalWrite(stepperStepPin, LOW);
  digitalWrite(drillDirPin, LOW);
  digitalWrite(drillPWMPin, LOW);
  digitalWrite(actuatorDirPin, LOW);
  digitalWrite(actuatorPWMPin, LOW);
  

}


