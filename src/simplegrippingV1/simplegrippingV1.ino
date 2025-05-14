
#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include "config.h"
#include <SimpleFOC.h>

struct InputDataStruct{
 double x, y, z;
 float tleSensor;
 float targetVoltage;
 bool button1;
  bool button2;
};

struct OutputDataStruct{
    float target_voltage;
};

struct stateStruct{
    float target_voltage;
    float current;
    float voltage;
    float position;
    float velocity;
    float rawAngle;
    float oldRawAngle=0;
    bool gripping;
    float v;
    float absoluteAngle;
    float currentRawAngle;
    float totalDegreesTraveled;
    float degreeOffset;
    bool zeroing=true;
    float x_filtered=0;
    float y_filtered=0;
    float z_filtered=0;
    float magneticAlphaFilter=0.02;
    int numberOfRevolutions=0;
    
};
struct PIDStruct{
    float k;
    long T;
    long T_old;
    long T_passed;
    float Ti;
    float Td;
    float Tt;
    float N=5;
    float umax;
    float umin;
    float y;
    float r;
    float P;
    float D;
    float v;
    float I;
    float y_old;
    float b;
    float u;
};
//
InputDataStruct inputDataLoop; // sadrzi informacije o senzorima
OutputDataStruct outputDataLoop; // sadrzi izlazne podatke
stateStruct stateDataLoop; // sadrzi stanje motora
PIDStruct PIDDataLoop; // sadrzi PID parametre
// define SPI pins for TLE5012 sensor
#define PIN_SPI1_SS0 94  // Chip Select (CS) pin
#define PIN_SPI1_MOSI 69 // MOSI pin
#define PIN_SPI1_MISO 95 // MISO pin
#define PIN_SPI1_SCK 68  // SCK pin
// create an instance of SPIClass3W for 3-wire SPI communication
tle5012::SPIClass3W tle5012::SPI3W1(2);
// create an instance of TLE5012Sensor
TLE5012Sensor tle5012Sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI,
                            PIN_SPI1_SCK);

// BLDC motor instance BLDCMotor (polepairs, motor phase resistance, motor KV
// rating, motor phase inductance)
BLDCMotor motor = BLDCMotor(
    7, 0.24, 360,
    0.000133); // 7 pole pairs, 0.24 Ohm phase resistance, 360 KV and 0.000133H
// you can find more data of motor in the doc

// define driver pins
const int U = 11;
const int V = 10;
const int W = 9;
const int EN_U = 6;
const int EN_V = 5;
const int EN_W = 3;

// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(U, V, W, EN_U, EN_V, EN_W);

// voltage set point variable
float target_voltage = -1;

#if ENABLE_MAGNETIC_SENSOR
// create a instance of 3D magnetic sensor
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
// define the number of calibration samples
const int CALIBRATION_SAMPLES = 20;
// offsets for calibration
double xOffset = 0, yOffset = 0, zOffset = 0;
#endif


void readInputs(InputDataStruct &inputData) {
  
  // read the buttons
  inputData.button1 = digitalRead(BUTTON1);
  inputData.button2 = digitalRead(BUTTON2);
  // read the target voltage from the buttons
  
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&inputData.x, &inputData.y, &inputData.z);

  // subtract the offsets from the raw data
  inputData.x -= xOffset;
  inputData.y -= yOffset;
  inputData.z -= zOffset;
  if (inputData.button1 == LOW) 
    stateDataLoop.gripping = true;
  else if (inputData.button2 == LOW) 
    stateDataLoop.gripping = false;
  
  tle5012Sensor.update();
  inputData.tleSensor=tle5012Sensor.getSensorAngle();
}
void executeLogic(InputDataStruct &inputData, OutputDataStruct &outputData){
  filterMagneticData();
  stateDataLoop.target_voltage=0;//kasnije ovo izmeniti
  discretePID();
  gripperPositionTracking();
  if (inputData.button2==LOW)
  {
    stateDataLoop.target_voltage = 1;
    stateDataLoop.gripping=false;
  } 
  if (inputData.button1==LOW)
  {
    stateDataLoop.gripping=true;
  }
  if (stateDataLoop.gripping)
    findingStablePosition();
   
  if (stateDataLoop.zeroing)
      zeriongFunc();
  outputDataLoop.target_voltage=stateDataLoop.target_voltage;
}
void outputResults(OutputDataStruct &outputData){

    motor.loopFOC();
    motor.move(outputData.target_voltage);
}
void serialComunication(InputDataStruct &inputData, OutputDataStruct &outputData){
/*
    Serial.print(inputData.x);
    Serial.print(",");
    Serial.print(inputData.y);
    Serial.print(",");
    Serial.print(inputData.z);
    Serial.print(",");
    Serial.print(inputData.tleSensor);
    Serial.println();
*/
Serial.print(filterMagneticData.x_filtered);
Serial.print(",");
Serial.println(outputData.target_voltage==0?0:1);
}

void discretePID()
{
  // Implement the discrete PID control logic here
  // This function is called in the loop() function
  // to perform the PID control calculations
  // and update the motor's target voltage accordingly.
  // You can use the inputData and outputData structures
  PIDDataLoop.T_passed=millis();
  PIDDataLoop.T=PIDDataLoop.T_passed-PIDDataLoop.T_old;
  PIDDataLoop.T_old=PIDDataLoop.T_passed;

  float bi=(PIDDataLoop.k*PIDDataLoop.T)/PIDDataLoop.Ti;
  float br=PIDDataLoop.T/PIDDataLoop.Tt;
  float ad=PIDDataLoop.Td/(PIDDataLoop.Td+PIDDataLoop.N*PIDDataLoop.T);
  float bd=(PIDDataLoop.k*PIDDataLoop.Td*PIDDataLoop.N)/(PIDDataLoop.Td+PIDDataLoop.N*PIDDataLoop.T);

  PIDDataLoop.y=inputDataLoop.targetVoltage;
  PIDDataLoop.r=inputDataLoop.x;
  PIDDataLoop.P=PIDDataLoop.k*(PIDDataLoop.b*PIDDataLoop.r-PIDDataLoop.y);
  PIDDataLoop.D=ad*PIDDataLoop.D-bd*(PIDDataLoop.y-PIDDataLoop.y_old);
  
  PIDDataLoop.v=PIDDataLoop.P+PIDDataLoop.I+PIDDataLoop.D;
  
  if (PIDDataLoop.v>PIDDataLoop.umax)
    PIDDataLoop.u=PIDDataLoop.umax;
  else if (PIDDataLoop.v<PIDDataLoop.umin)
    PIDDataLoop.u=PIDDataLoop.umin;
  else
    PIDDataLoop.u=PIDDataLoop.v;

  PIDDataLoop.I=PIDDataLoop.I+bi*(PIDDataLoop.r-PIDDataLoop.y)+br*(PIDDataLoop.u-PIDDataLoop.v);
  PIDDataLoop.y_old=PIDDataLoop.y;

  
}
void initDiscretePID()
{
  PIDDataLoop.b=0.9;
  PIDDataLoop.Ti=1;
  PIDDataLoop.Tt=1;
  PIDDataLoop.Td=1;
  PIDDataLoop.N=6;
  PIDDataLoop.k=1;
  PIDDataLoop.P=0;
  PIDDataLoop.I=0;
  PIDDataLoop.D=0;
  PIDDataLoop.umax=1;
  PIDDataLoop.umin=-1;
  PIDDataLoop.y_old=0;
  PIDDataLoop.y=0;
  PIDDataLoop.r=0;
  PIDDataLoop.T_old=millis();

  /*
      float k;
    float T_old;
    float T_passed;
    float Ti;
    float Tt;
    float N=5;
    float umax;
    float umin;
    float y;
    float r;
    float P;
    float D;
    float v;
    float I;
    float y_old;*/
}

void zeriongFunc() {
  // Initialize total path traveled (sum of absolute movements).
  if (abs(stateDataLoop.x_filtered)<0.07)
  {
    
    stateDataLoop.target_voltage=-1;
    Serial.println("zeroing in progress");
    Serial.println(stateDataLoop.x_filtered);
  }
  else 
  {
    stateDataLoop.target_voltage=0;
    Serial.println("zeroing done");
    stateDataLoop.zeroing=false;
    stateDataLoop.degreeOffset=inputDataLoop.tleSensor* (57.295779513); // convert to degrees
    Serial.println(stateDataLoop.degreeOffset);
  }
}

void gripperPositionTracking() {
  // 1. Read the new sensor value
  float newRaw = tle5012Sensor.getSensorAngle() * (57.295779513); // convert to degrees
  // 2. Compute raw delta
  if (newRaw <30 && stateDataLoop.oldRawAngle > 330) {
    // if the new value is less than 30 and the old value is greater than 330
    // then we have crossed the 0 degree line
    stateDataLoop.numberOfRevolutions++;
  } else if (newRaw > 330 && stateDataLoop.oldRawAngle < 30) {
    // if the new value is greater than 330 and the old value is less than 30
    // then we have crossed the 360 degree line
    stateDataLoop.numberOfRevolutions--;
  }
  // 3. Unwrap at ±half‐circle
  
  stateDataLoop.absoluteAngle = newRaw + stateDataLoop.numberOfRevolutions * 360- stateDataLoop.degreeOffset;  

      stateDataLoop.oldRawAngle=newRaw;
}

void filterMagneticData() {
  // Apply a low-pass filter to the magnetic field data
  stateDataLoop.x_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.x_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.x;
  stateDataLoop.y_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.y_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.y;
  stateDataLoop.z_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.z_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.z;
}

void findingStablePosition() {
  // Implement the logic to find the stable position of the gripper
  // This function can be called when the gripper is not gripping
  // and you want to find a stable position based on the magnetic field data.
  // You can use the filtered magnetic data (x_filtered, y_filtered, z_filtered)
  // to determine the stable position.
  if(abs(stateDataLoop.x_filtered)>0.1)
  {
    stateDataLoop.target_voltage=0;
    
  }
  else
  {
    stateDataLoop.target_voltage=-5;
    
  }

}
void setup() {
  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  tle5012Sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&tle5012Sensor);

  // power supply voltage
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 2;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  // motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  Serial.println(F("Motor ready."));

  // start 3D magnetic sensor
  dut.begin();
  // calibrate 3D magnetic sensor to get the offsets
  calibrateSensor();
  Serial.println("3D magnetic sensor Calibration completed.");

  initDiscretePID();

  // set the pin modes for buttons
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  Serial.print("setup done.\n");

  _delay(1000);
}

void loop() {

    readInputs(inputDataLoop);
    executeLogic(inputDataLoop,outputDataLoop);
    outputResults(outputDataLoop);
    serialComunication(inputDataLoop,outputDataLoop);
  

}

#if ENABLE_MAGNETIC_SENSOR
/**
 * @brief Calibrates the magnetic field sensor by calculating the average
 * offsets for the X, Y, and Z axes over a series of samples.
 */
void calibrateSensor() {
  double sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    double temp;
    double valX, valY, valZ;

    dut.getMagneticFieldAndTemperature(&valX, &valY, &valZ, &temp);
    sumX += valX;
    sumY += valY;
    sumZ += valZ;

    delay(10); // Adjust delay as needed
  }

  // Calculate average offsets
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;
}
#endif