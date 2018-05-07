#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFunTSL2561.h>
#include <Wire.h>




#define CCS811_ADDR 0x5B //Default I2C Address CCS811 Sensor

//Sensor Objects
CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;
SFE_TSL2561 light;

String allSensorData;
const int chipSelect = 8; //SD Shield Enable
int timecounter = 0;
unsigned char time = 2;

//Light Sensor
boolean gain = 0;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds


void setup() {

  Serial.begin(115200);

  
  //This begins the CCS811 sensor and prints error status of .begin()
  myCCS811.begin();

  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;

  //Initialize BME280
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;
  //Calling .begin() causes the settings to be loaded
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  myBME280.begin();

  Serial.println("Set LUX timing...");
  light.begin();
  light.setTiming(gain,time,ms);
  Serial.println("Powerup LUX...");
  light.setPowerUp();

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }

}

void loop() {
  // put your main code here, to run repeatedly:

    formatData();
    
    delay(2000);
}


void formatData(){
  float tempC;
  float humidity;
  allSensorData = "";

  allSensorData += String(timecounter)+",";
  Serial.println(myCCS811.getCO2());  
  if (myCCS811.dataAvailable())
  {
    myCCS811.readAlgorithmResults();
    allSensorData += String(myCCS811.getCO2()) + ",";
    allSensorData += String(myCCS811.getTVOC()) + ",";
    
    tempC = myBME280.readTempC();
    humidity = myBME280.readFloatHumidity();
    myCCS811.setEnvironmentalData(humidity, tempC);
    
    
    //Environment Sensor Data

    allSensorData += String(tempC) + ",";
    allSensorData += String(myBME280.readFloatPressure()) + ",";
    allSensorData += String(myBME280.readFloatAltitudeMeters()) + ",";
    allSensorData += String(humidity) + ",";
  }else{
    allSensorData += "0,";
    allSensorData += "0,";
    allSensorData += "0,";
    allSensorData += "0,";
    allSensorData += "0,";
    allSensorData += "0,";
  }

  unsigned int data0, data1;
  
  if (light.getData(data0,data1))
  {
  
    // To calculate lux, pass all your settings and readings
    // to the getLux() function.
    
    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor
  
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    // Perform lux calculation:

    good = light.getLux(gain,ms,data0,data1,lux);
    
    allSensorData += String(lux) + "";
    
  }else{

    allSensorData += "0";
  }

  Serial.println(allSensorData); 
  writeFile();
  timecounter +=2;
  
}

void printHeader(){
  Serial.println("Co2,TVOC,TempC,Pressure,Altitude,Humidity,Lux");
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println("Co2,TVOC,TempC,Pressure,Altitude,Humidity,Lux");
    dataFile.close();
  }

  
}

void writeFile(){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(allSensorData);
    dataFile.close();
  }
}


void pollHydra(){



  
}

