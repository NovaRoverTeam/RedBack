//#include <ros.h>
#include <AccelStepper.h>
#include <Encoder.h>


///////////////////////////// Defitions ////////////////////////////////////////
//DOUBLE CHECK PINS
#define encoderPin1 2
#define encoderPin2 3

#define stepperEnablePin 4
#define stepperDirPin 5
#define stepperStepPin 6

#define drillDirPin 7
#define drillPWMPin 8

#define actuatorDirPin 9
#define actuatorPWMPin 10

#define killSwitchPin 11

#define stepperSpeed 100
#define steppperMaxPos 1000

#define cachePos1 0
#define cachePos2 100
#define cachePos3 400
#define drillPos 250

#define cacheActuatorPos 100



///////////////////////////// Variables ////////////////////////////////////////

bool isDrillSpinning = false;
bool isActuatorMoving = false;
int curStepperPos; //Center = 0, Cache #1 = 1,Cache #2 = 2
int curActuatorPos = -999;
int oldActuatorPos = -999;
int requestedActuatorPos = 200;

////////////////////////////// ROS CALLBACK ////////////////////////////////////////
/*
// Declare required ROS variables
ros::NodeHandle  nh;


// may need to subscribe to "mainframe/arm_cmd_data" instead if this doesn't work
ros::Subscriber<rover::redCmd> sub("red_cmd_data", &msgCallback);


void msgCallback (const rover::redCmd& msg){



  int lastDrillSpd = 0;
  int drillSpd = 0;

  int stepperPos = 0;

  int lastActuatorPos = 0;
  int lastActuatorSpeed = 0;
  int actuatorSpd = 0;


  if (msg.kill == 0){
    killAll();
  }


  //DRILL STUFF
  lastDrillSpd = drillSpd;
  drillSpd = msg.drillSpd;

  if(lastDrillSpd != drillSpd){
    setDrillSpeed(drillSpd);
  }

  //STEPPER STUFF
  stepperPos = msg.stepperPos;

  if(stepperPos == 0){

    stepper.moveTo(cachePos1);

  }else if(stepperPos == 1){

    stepper.moveTo(cachePos2);

  }else if(stepperPos == 2){

    stepper.moveTo(cachePos3);

  }


  //Actuator Stuff
    lastActuatorSpd = actuatorSpd;
  actuatorSpd = msg.actuatorSpeed;

  if(lastActuatorSpd != actuatorSpd){
    setDrillSpeed(actuatorSpd);
  }

}
*/

//////////////////////////////// Motor Setup /////////////////////////////////////////

AccelStepper stepper(AccelStepper::DRIVER, stepperStepPin, stepperDirPin);

Encoder myEnc(encoderPin1, encoderPin2);

//////////////////////////////// SETUP /////////////////////////////////////////
void setup() {

  //nh.initNode();
  //nh.subscribe(red_sub); //?????

  Serial.begin(57600);

  pinMode(drillDirPin, OUTPUT);
  pinMode(drillPWMPin, OUTPUT);
  pinMode(actuatorDirPin, OUTPUT);
  pinMode(actuatorPWMPin, OUTPUT);
  pinMode(killSwitchPin, INPUT);

  stepper.setEnablePin(13);
  stepper.enableOutputs();
  stepper.setSpeed(stepperSpeed);

  delay(2000);

  calibrateSystem();


}


//////////////////////////////////// Main Software Loop /////////////////////////////////////
void loop() {

  long curActuatorPos = myEnc.read();
  if (curActuatorPos != oldActuatorPos) {
    oldActuatorPos = curActuatorPos;
    //Serial.println(curActuatorPos);
  }

  stepper.run(); // Move Stepper if position is requested
  actuatorRun();

 temporaryDrilling();
  
}


//////////////////////////////////// Functionalities /////////////////////////////////////


void setDrillSpeed(int spd) {


  if (spd > 0) { // Clockwise Rotations

    isDrillSpinning = true;
    digitalWrite(drillDirPin, LOW);
    analogWrite(drillPWMPin, spd);
    Serial.print("spinning");

  } else if (spd < 0) {

    isDrillSpinning = true;
    digitalWrite(drillDirPin, HIGH);
    spd = abs(spd);
    analogWrite(drillPWMPin, spd);

  } else {

    isDrillSpinning = false;
    digitalWrite(drillDirPin, LOW);
    analogWrite(drillPWMPin, 0);

  }
}


void moveActuatorAbs(int pos) {

  requestedActuatorPos = pos;
  actuatorRun();

}

void actuatorRun(){

  int difference = requestedActuatorPos - curActuatorPos;
  int spdV = 0;
  int howClose = 10; //If it has trouble going to 

  while(difference != 0){

     long curActuatorPos = myEnc.read();
     if (curActuatorPos != oldActuatorPos) {
        oldActuatorPos = curActuatorPos;
        //Serial.println(curActuatorPos);
      }   
  
    if (difference > howClose) { //Drill is Higher than requested pos
      
      moveActuatorRel(100);
    
    } else if (difference < howClose) { //Encoder is higher than cache, Move up
      
      moveActuatorRel(-100);
  
    } else {
      //Set Motor OFF
      moveActuatorRel(0);
    }

   delay(100);
  }
  
}

void moveActuatorRel(int spd) {

  if (spd > 0) { // Clockwise Rotations

    isActuatorMoving = true;
    digitalWrite(actuatorDirPin, HIGH);
    analogWrite(actuatorPWMPin, spd);

  } else if (spd < 0) { // Counter Clockwise Rotations

    isActuatorMoving = true;
    digitalWrite(actuatorDirPin, LOW);
    spd = abs(spd);
    analogWrite(actuatorPWMPin, spd);


  } else { //Not Moving

    isActuatorMoving = false;
    digitalWrite(actuatorDirPin, LOW);
    analogWrite(actuatorPWMPin, 0);

  }

}


bool calibrateSystem() {
  
  moveActuatorRel(-255); //Full Speed up
  delay(5000); //Make sure actuator reaches top endstop
  curActuatorPos = 0;


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

void temporaryDrilling() {
  
    #define drillingSpeed 160 // How fast to Spin Drill
    #define actuatorInitialPos 200 // Where to start drilling from
    #define speedOfActuator 100 
    #define timeToDrill 4000 //In MS
    #define timeToReturn 5000 // in MS

    moveActuatorAbs(actuatorInitialPos); // THIS NUMBeR IS HOW FAR DOWN IN ENCODER STEPS TO GO DOWN

    setDrillSpeed(drillingSpeed); //Speed of Drill
    moveActuatorRel(speedOfActuator); // How fast to go down
    
    delay(timeToDrill); // How long to drill For
    
    setDrillSpeed(-drillingSpeed); // Speed of Return Drill
    moveActuatorRel(-speedOfActuator); // Speed of going back up again
    
    delay(timeToReturn); // How long to go up for
    

    setDrillSpeed(0); // Stop Motor
    moveActuatorRel(0); // Stop Motor

    delay(1000000); //Stop indefinitely


}


