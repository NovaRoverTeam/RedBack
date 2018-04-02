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

#define drillDirPin 7
#define drillPWMPin 8

#define actuatorDirPin 9
#define actuatorPWMPin 10

#define stepperEndstopPin 11

#define stepperSpeed 300
#define steppperMaxPos 1000

//REDEFINE THE VALUES BELOW FOR SETUP!
#define cachePos1 0
#define cachePos2 100
#define sensorPos1 400
#define cacheActuatorPos 100



///////////////////////////// Variables ////////////////////////////////////////

bool isDrillSpinning = false;
bool isActuatorMoving = false;
bool isCalibrating = false;
bool actuatorState = false; //Relative Move:False, Absolute Move:True

int curStepperLocation; // Cache #1 = -1, Center = 0, Cache #2 = 1, Sensor #1 = 2
int oldStepperPos = 0;

int curActuatorPos = -999;
int oldActuatorPos = -999;

int curDrillSpeed = 0;

int requestedActuatorPos;
int requestedActuatorSpeed;
int requestedStepperPos;
int requestedDrillSpeed;

////////////////////////////// ROS CALLBACK ////////////////////////////////////////

// Declare required ROS variables
ros::NodeHandle  nh;


// may need to subscribe to "mainframe/arm_cmd_data" instead if this doesn't work
ros::Subscriber<rover::redCmd> sub("red_cmd_data", &msgCallback);


void msgCallback (const rover::redCmd& msg){

  int lastActuatorPos = 0;
  int lastActuatorSpeed = 0;
  int actuatorSpd = 0;


  if (msg.kill == 0){
    killAll();
  }


  //DRILL STUFF
  requestedDrillSpeed = msg.drillSpd

  //Actuator Stuff
  requestedActuatorSpeed = msg.actuatorSpeed
  
  if (requestedActuatorSpeed == 300){ 
    actuatorState = true; //Absolute Move
    requestedActuatorPos = cacheActuatorPos;
    requestedActuatorSpeed = 0;
  }else{
    actuatorState = false; //Relative Move
  }

  //STEPPER STUFF
  requestedStepperPos = msg.stepperPos;



}


//////////////////////////////// Motor Setup /////////////////////////////////////////

AccelStepper stepper(AccelStepper::DRIVER, stepperStepPin, stepperDirPin);

Encoder myEnc(encoderPin1, encoderPin2);

//////////////////////////////// SETUP /////////////////////////////////////////
void setup() {

  nh.initNode();
  nh.subscribe(red_sub);

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

  /*if(isCalibrating){
    endstopState = digitalRead(stepperEndstopPin);
    
    // check if the stepper endstop is reached
    if (endstopState == HIGH) {
    // Stop Calibrating, set pos as 0
      stepper.stop();
      stepper.setCurrentPosition(0);
      isCalibrating = false;
    } else {
      Serial.println("Stepper Calibrating");
    }
  }*/

  //Update Encoder position as the current actuator pos
  long curActuatorPos = myEnc.read();
  if (curActuatorPos != oldActuatorPos) {
    oldActuatorPos = curActuatorPos;
    //Serial.println(curActuatorPos);
  }

  
  drillRun();
  actuatorRun();
  stepperRun();
  stepper.run();

  //temporaryDrilling();
  
}


//////////////////////////////////// Functionalities /////////////////////////////////////



void stepperRun(){
 

  if (requestedStepperPos != stepper.currentPosition()){
    if(!isActuatorMoving){ //Only move Sidesways if actuator isnt moving
    
      //Cache #1
      if(requestedStepperPos == -1){
        if(curActuatorPos > cacheActuatorPos){ //CHECK ENCODER VALUE +ve  OR -VE
          stepper.moveTo(cachePos1);
        }  
      } 
  
      //Center Drilling
      if(requestedStepperPos == 0){
        if(curActuatorPos > cacheActuatorPos){ //CHECK ENCODER VALUE +ve  OR -VE
          stepper.moveTo(cachePos2);
        }  
      } 
  
  
      //Cache #2
      if(requestedStepperPos == 1){
        if(curActuatorPos > cacheActuatorPos){ //CHECK ENCODER VALUE +ve  OR -VE
          stepper.moveTo(cachePos3);
        }  
      } 
  
  
      //Sensor #1
      if(requestedStepperPos == 2){
        if(curActuatorPos > cacheActuatorPos){ //CHECK ENCODER VALUE +ve  OR -VE
          stepper.moveTo(cachePos4);
        }  
      } 

    }
  }
  
}


void drillRun() {
  if (requestedDrillSpeed != curDrillSpeed){
    
    if (requestedDrillSpeed > 0) { // Clockwise Rotations
  
      isDrillSpinning = true;
      curDrillSpeed = requestedDrillSpeed;
      digitalWrite(drillDirPin, LOW);
      analogWrite(drillPWMPin, requestedDrillSpeed);
      Serial.print("spinning");
  
    } else if (requestedDrillSpeed < 0) {
  
      isDrillSpinning = true;
      curDrillSpeed = requestedDrillSpeed;
      digitalWrite(drillDirPin, HIGH);
      spd = abs(spd);
      analogWrite(drillPWMPin, requestedDrillSpeed);
  
    } else {
  
      isDrillSpinning = false;
      digitalWrite(drillDirPin, LOW);
      analogWrite(drillPWMPin, 0);
  
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

  if (requestedActuatorSpeed != curActuatorSpeed){

    if (requestedActuatorSpeed > 0) { // Clockwise Rotations

    isActuatorMoving = true;
    curActuatorSpeed = requestedActuatorSpeed;
    digitalWrite(actuatorDirPin, HIGH);
    analogWrite(actuatorPWMPin, spd);

    } else if (requestedActuatorSpeed < 0) { // Counter Clockwise Rotations

    isActuatorMoving = true;
    curActuatorSpeed = requestedActuatorSpeed;
    digitalWrite(actuatorDirPin, LOW);
    spd = abs(spd);
    analogWrite(actuatorPWMPin, spd);


    } else { //Not Moving

    isActuatorMoving = false;
    digitalWrite(actuatorDirPin, LOW);
    analogWrite(actuatorPWMPin, 0);

    }
  }
}


void calibrateSystem() {
  
  isCalibrating = true;

  moveActuatorRel(-255); //Full Speed up
  delay(5000); //Make sure actuator reaches top endstop
  curActuatorPos = 0;

  //stepper.move(-500)
  stepper.setCurrentPosition(0);
  //Main loop will let motor run till the sensor is reached


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
  
    #define drillingSpeed 0 // How fast to Spin Drill
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


