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

//#define stepperEndstopPin 11 Unnecessary

#define stepperSpeed 300
#define steppperMaxPos 1000





///////////////////////////// Variables ////////////////////////////////////////

bool isDrillSpinning = false;
bool isActuatorMoving = false;
bool isCalibrating = false;
bool actuatorState = false; //Relative Move:False, Absolute Move:True


int curActuatorPos = -999;
int oldActuatorPos = -999;
int curActuatorSpeed = 0;

int curDrillSpeed = 0;

int requestedActuatorPos;
int requestedActuatorSpeed;
int requestedStepperCmd;
int requestedDrillSpeed;

////////////////////////////// ROS CALLBACK ////////////////////////////////////////

// Declare required ROS variables
//ros::NodeHandle  nh;


// may need to subscribe to "mainframe/arm_cmd_data" instead if this doesn't work
//ros::Subscriber<rover::redCmd> sub("red_cmd_data", &msgCallback);


/*void msgCallback (const rover::redCmd& msg){


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
  requestedStepperCmd = msg.stepperPos;



}

*/
//////////////////////////////// Motor Setup /////////////////////////////////////////

AccelStepper stepper(AccelStepper::DRIVER, stepperStepPin, stepperDirPin);

Encoder myEnc(encoderPin1, encoderPin2);

//////////////////////////////// SETUP /////////////////////////////////////////
void setup() {

//  nh.initNode();
//  nh.subscribe(red_sub);

  Serial.begin(57600);

  pinMode(drillDirPin, OUTPUT);
  pinMode(drillPWMPin, OUTPUT);
  pinMode(actuatorDirPin, OUTPUT);
  pinMode(actuatorPWMPin, OUTPUT);

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
    Serial.println("ActuatorPos: " + curActuatorPos);
  }

  
  drillRun();
  actuatorRun();
  stepperRun();
  stepper.run();

  
}


//////////////////////////////////// Functionalities /////////////////////////////////////



void stepperRun(){
 

  if (requestedStepperCmd != stepper.currentPosition()){
    if(!isActuatorMoving){ //Only move Sidesways if actuator isnt moving

      stepper.move(requestedStepperCmd*5);
      Serial.println("Stepper Moving");

    }
  }
  
}


void drillRun() {
  int spd;
  if (requestedDrillSpeed != curDrillSpeed){
    
    if (requestedDrillSpeed > 0) { // Clockwise Rotations
  
      isDrillSpinning = true;
      curDrillSpeed = requestedDrillSpeed;
      digitalWrite(drillDirPin, LOW);
      analogWrite(drillPWMPin, requestedDrillSpeed);
      Serial.print("Drill Spinning");
  
    } else if (requestedDrillSpeed < 0) {
  
      isDrillSpinning = true;
      curDrillSpeed = requestedDrillSpeed;
      digitalWrite(drillDirPin, HIGH);
      spd = abs(requestedActuatorSpeed);
      analogWrite(drillPWMPin, requestedDrillSpeed);
      Serial.print("Drill Spinning");
  
    } else {
  
      isDrillSpinning = false;
      digitalWrite(drillDirPin, LOW);
      analogWrite(drillPWMPin, 0);
      Serial.print("Drill Stopped");
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
  if (requestedActuatorSpeed != curActuatorSpeed){

    if (requestedActuatorSpeed > 0) { // Clockwise Rotations

    isActuatorMoving = true;
    curActuatorSpeed = requestedActuatorSpeed;
    digitalWrite(actuatorDirPin, HIGH);
    analogWrite(actuatorPWMPin, requestedActuatorSpeed);
    curActuatorSpeed = requestedActuatorSpeed;

    } else if (requestedActuatorSpeed < 0) { // Counter Clockwise Rotations

    isActuatorMoving = true;
    curActuatorSpeed = requestedActuatorSpeed;
    digitalWrite(actuatorDirPin, LOW);
    spd = abs(requestedActuatorSpeed);
    analogWrite(actuatorPWMPin, spd);
    curActuatorSpeed = requestedActuatorSpeed;

    } else { //Not Moving

    isActuatorMoving = false;
    digitalWrite(actuatorDirPin, LOW);
    analogWrite(actuatorPWMPin, 0);
    curActuatorSpeed = 0;

    }
  }
}


void calibrateSystem() {
  
  isCalibrating = true;

  requestedActuatorSpeed = -255;
  moveActuatorRel(); //Full Speed up
  delay(5000); //Make sure actuator reaches top endstop
  curActuatorPos = 0;

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

