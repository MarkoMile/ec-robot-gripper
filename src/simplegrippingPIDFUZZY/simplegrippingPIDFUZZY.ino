#include "config.h"
#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include <SimpleFOC.h>

// This is a modified version of the simplegrippingPIDFUZZY.ino file
// with added serial command handling to update the PID setpoint (r value)

#define MAX_ANGLE 1000
#define PID_SETPOINT 0.9  // Removed semicolon - this was causing issues

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
    bool gripping=false;
    float v;
    float absoluteAngle;
    float oldAbsoluteAngle=-1234;
    float currentRawAngle;
    float totalDegreesTraveled;
    float degreeOffset;
    bool zeroing=true;
    float x_filtered=0;
    float y_filtered=0;
    float z_filtered=0;
    float magneticAlphaFilter=0.02;
    int numberOfRevolutions=0;
    // zeroing tracking variables
    unsigned long zeroLastTime = 0;
    float zeroLastAngle = 0;
    unsigned long zeroStableStart = 0;
    float x_filtered_prev=0.0;
    float x_speed=0.0;
    unsigned long x_speed_time_prev;
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

// Initialize all the structs needed for control
InputDataStruct inputDataLoop;
OutputDataStruct outputDataLoop;
stateStruct stateDataLoop; 
PIDStruct PIDDataLoop;

// Variables for serial command handling
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;     // whether the string is complete

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

// BLDC motor instance
BLDCMotor motor = BLDCMotor(
    7, 0.24, 360,
    0.000133); // 7 pole pairs, 0.24 Ohm phase resistance, 360 KV and 0.000133H

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

// create a instance of 3D magnetic sensor
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
// define the number of calibration samples
const int CALIBRATION_SAMPLES = 20;
// offsets for calibration
double xOffset = 0, yOffset = 0, zOffset = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  inputString.reserve(200); // reserve 200 bytes for the inputString
  
  // Initialize magnetic sensor hardware
  tle5012Sensor.init();
  // Link the motor to the sensor
  motor.linkSensor(&tle5012Sensor);

  // Power supply voltage
  driver.voltage_power_supply = 12;
  // Limit the maximal DC voltage
  driver.voltage_limit = 6;
  
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  
  // Link the motor and the driver
  motor.linkDriver(&driver);

  // Set motor parameters
  motor.voltage_sensor_align = 2;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  // Initialize motor
  motor.init();
  motor.initFOC();
  Serial.println(F("Motor ready."));

  // Start 3D magnetic sensor
  dut.begin();
  // Calibrate 3D magnetic sensor to get the offsets
  calibrateSensor();
  Serial.println("3D magnetic sensor Calibration completed.");

  // Initialize PID controller
  initDiscretePID();

  // Set the pin modes for buttons (if used)
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  Serial.println("Setup complete. Ready to receive commands.");
  Serial.println("Format: 'PIDSetpoint:X' where X is the setpoint value (e.g., 'PIDSetpoint:0.8')");
  
  stateDataLoop.x_speed_time_prev = millis();

  delay(1000);
}

void loop() {
  // Check for any incoming serial data before processing
  checkSerial();
  
  // Process any incoming serial commands
  processSerialCommands();
  
  // Read sensor inputs
  readInputs(inputDataLoop);
  
  // Process control logic
  executeLogic(inputDataLoop, outputDataLoop);
  
  // Output to motor
  outputResults(outputDataLoop);
  
  // Send data back to the serial interface
  serialCommunication(inputDataLoop, outputDataLoop);
}

// New function to explicitly check for serial data
void checkSerial() {
  while (Serial.available()) {
    // Get the new byte
    char inChar = (char)Serial.read();
    
    // If the incoming character is a newline, set a flag
    // so the main loop can process the complete string
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // Add the incoming character to the inputString
      inputString += inChar;
    }
  }
}

void processSerialCommands() {
  // Check if there's a complete command to process
  if (stringComplete) {
    Serial.print("Received command: ");
    Serial.println(inputString);
    
    // Check if the command is for setting PID setpoint
    if (inputString.startsWith("PIDSetpoint:")) {
      // Extract the setpoint value after the colon
      String valueStr = inputString.substring(12);
      // Convert to float
      float newSetpoint = valueStr.toFloat();
      
      // Set the new setpoint value
      PIDDataLoop.r = newSetpoint;
      
      // Acknowledge receipt of command
      Serial.print("PID Setpoint updated to: ");
      Serial.println(newSetpoint);
    }
    
    // Clear the string for the next command
    inputString = "";
    stringComplete = false;
  }
}

void readInputs(InputDataStruct &inputData) {
  // Read buttons if configured
  inputData.button1 = digitalRead(BUTTON1);
  inputData.button2 = digitalRead(BUTTON2);
  
  // Read magnetic sensor
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&inputData.x, &inputData.y, &inputData.z);

  // Subtract the offsets from the raw data
  inputData.x -= xOffset;
  inputData.y -= yOffset;
  inputData.z -= zOffset;
  
  if (inputData.button1 == LOW) 
    stateDataLoop.gripping = true;
  else if (inputData.button2 == LOW) 
    stateDataLoop.gripping = false;
  
  tle5012Sensor.update();
  inputData.tleSensor = tle5012Sensor.getSensorAngle();
}

void executeLogic(InputDataStruct &inputData, OutputDataStruct &outputData){
  filterMagneticData();
  discretePID();
  gripperPositionTracking();
  
  if (inputData.button2 == LOW) {
    stateDataLoop.target_voltage = 1;
    stateDataLoop.gripping = false;
  } 
  else {
    stateDataLoop.target_voltage = 0;
  }
  
  if (inputData.button1 == LOW) {
    stateDataLoop.gripping = true;
  }
  
  if (stateDataLoop.gripping) {
    stateDataLoop.target_voltage = -PIDDataLoop.u; // Because - is positive
  }
   
  if (stateDataLoop.zeroing) {
    zeroingFunc();
  }

  if (stateDataLoop.absoluteAngle > MAX_ANGLE && stateDataLoop.target_voltage > 0) {
    stateDataLoop.target_voltage = 0;
  }
  outputDataLoop.target_voltage = stateDataLoop.target_voltage;
}

void outputResults(OutputDataStruct &outputData){
  motor.loopFOC();
  motor.move(outputData.target_voltage);
}

void serialCommunication(InputDataStruct &inputData, OutputDataStruct &outputData){
  // Calculate magnitude for monitoring
  float magnitude = calculateMagnitude(stateDataLoop.x_filtered, stateDataLoop.y_filtered, stateDataLoop.z_filtered);
  
  // Output formatted data for serial plotter (comma separated values)
  Serial.print(magnitude);        // Current magnetic field magnitude
  Serial.print(",");
  Serial.print(PIDDataLoop.r);    // PID Setpoint
  Serial.print(",");  
  Serial.print(PIDDataLoop.k);    // Current k value
  Serial.print(",");  
  Serial.print(PIDDataLoop.u);    // Current control output
  Serial.print(",");
  Serial.println(outputData.target_voltage); // Actual voltage sent to motor
}

void discretePID() {
  // Implement the discrete PID control logic here
  PIDDataLoop.y = calculateMagnitude(stateDataLoop.x_filtered, stateDataLoop.y_filtered, stateDataLoop.z_filtered);
  PIDDataLoop.T_passed = millis();
  PIDDataLoop.T = (PIDDataLoop.T_passed - PIDDataLoop.T_old) / 1000.0;
  PIDDataLoop.T_old = PIDDataLoop.T_passed;

  float bi = (PIDDataLoop.k * PIDDataLoop.T) / PIDDataLoop.Ti;
  float br = PIDDataLoop.T / PIDDataLoop.Tt;
  float ad = PIDDataLoop.Td / (PIDDataLoop.Td + PIDDataLoop.N * PIDDataLoop.T);
  float bd = (PIDDataLoop.k * PIDDataLoop.Td * PIDDataLoop.N) / (PIDDataLoop.Td + PIDDataLoop.N * PIDDataLoop.T);

  PIDDataLoop.P = PIDDataLoop.k * (PIDDataLoop.b * PIDDataLoop.r - PIDDataLoop.y);
  PIDDataLoop.D = ad * PIDDataLoop.D - bd * (PIDDataLoop.y - PIDDataLoop.y_old);
  
  PIDDataLoop.v = PIDDataLoop.P;
  
  if (PIDDataLoop.v > PIDDataLoop.umax)
    PIDDataLoop.u = PIDDataLoop.umax;
  else if (PIDDataLoop.v < PIDDataLoop.umin)
    PIDDataLoop.u = PIDDataLoop.umin;
  else
    PIDDataLoop.u = PIDDataLoop.v;

  PIDDataLoop.I = PIDDataLoop.I + bi * (PIDDataLoop.r - PIDDataLoop.y) + br * (PIDDataLoop.u - PIDDataLoop.v);
  PIDDataLoop.y_old = PIDDataLoop.y;  
}

void initDiscretePID() {
  PIDDataLoop.b = 0.9;
  PIDDataLoop.Ti = 1;
  PIDDataLoop.Tt = 1;
  PIDDataLoop.Td = 1;
  PIDDataLoop.N = 6;
  PIDDataLoop.k = 1;
  PIDDataLoop.P = 0;
  PIDDataLoop.I = 0;
  PIDDataLoop.D = 0;
  PIDDataLoop.umax = 3;
  PIDDataLoop.umin = -1;
  PIDDataLoop.y_old = 0;
  PIDDataLoop.y = 0;
  PIDDataLoop.r = PID_SETPOINT; // Use the define without semicolon
  PIDDataLoop.T_old = millis() / 1000.0;
}

void zeroingFunc() {
  unsigned long now = millis();
  float currentAngle = inputDataLoop.tleSensor * (57.295779513); // convert to degrees
  float dt = (stateDataLoop.zeroLastTime == 0) ? 0 : (now - stateDataLoop.zeroLastTime) / 1000.0;
  stateDataLoop.zeroLastTime = now;
  float speed = (dt > 0) ? abs(currentAngle - stateDataLoop.zeroLastAngle) / dt : 0;
  stateDataLoop.zeroLastAngle = currentAngle;
  const float speedThreshold = 15; // deg/s threshold for stability
  
  const unsigned long stableDuration = 1000; // ms required to be stable
  
  if (speed > speedThreshold) {
    // still moving: continue zeroing motion
    stateDataLoop.target_voltage = -1;
    stateDataLoop.zeroStableStart = 0;
    Serial.println("zeroing in progress");
  } else {
    // detected low speed: start stable timer
    if (stateDataLoop.zeroStableStart == 0) {
      stateDataLoop.zeroStableStart = now;
    }
    if (now - stateDataLoop.zeroStableStart >= stableDuration) {
      // arm is stable: set as zero position
      stateDataLoop.target_voltage = 0;
      stateDataLoop.zeroing = false;
      stateDataLoop.degreeOffset = currentAngle;
      stateDataLoop.numberOfRevolutions = 0;
      Serial.println("zeroing done");
    } else {
      // waiting to confirm stability
      stateDataLoop.target_voltage = -1;
      Serial.println("zeroing stable...");
    }
  }
}

void gripperPositionTracking() {
  // 1. Read the new sensor value
  float newRaw = tle5012Sensor.getSensorAngle() * (57.295779513); // convert to degrees
  // 2. Compute raw delta
  if (newRaw < 30 && stateDataLoop.oldRawAngle > 330) {
    // if the new value is less than 30 and the old value is greater than 330
    // then we have crossed the 0 degree line
    stateDataLoop.numberOfRevolutions++;
  } else if (newRaw > 330 && stateDataLoop.oldRawAngle < 30) {
    // if the new value is greater than 330 and the old value is less than 30
    // then we have crossed the 360 degree line
    stateDataLoop.numberOfRevolutions--;
  }
  
  stateDataLoop.absoluteAngle = newRaw + (stateDataLoop.numberOfRevolutions * 360) - stateDataLoop.degreeOffset;  
  stateDataLoop.oldRawAngle = newRaw;
}

float calculateMagnitude(float x, float y, float z) {
  // Calculate the magnitude (Euclidean norm) of a 3D vector
  return sqrt(x*x + y*y + z*z);
}

void filterMagneticData() {
  // Apply a low-pass filter to the magnetic field data
  stateDataLoop.x_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.x_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.x;
  stateDataLoop.y_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.y_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.y;
  stateDataLoop.z_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.z_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.z;
}

void calibrateSensor() {
  double sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    double temp;
    double valX, valY, valZ;

    dut.getMagneticFieldAndTemperature(&valX, &valY, &valZ, &temp);
    sumX += valX;
    sumY += valY;
    sumZ += valZ;

    delay(10);
  }

  // Calculate average offsets
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;
}