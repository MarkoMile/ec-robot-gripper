#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include <SimpleFOC.h>


// Enable or disable commander functionality, please check: https://docs.simplefoc.com/commander_interface


#define MAX_ANGLE 1000
#define PID_Setpoint 0.9;
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

struct FuzzyTunerStruct {
    float setpoint;       // Target magnitude for the magnetic field (R)
    float prevError;      // Previous error for calculating change
    float prevTime;       // Previous time for calculating delta time
    float k_min;          // Minimum value for k
    float k_max;          // Maximum value for k
};

// Initialize fuzzy tuner struct
FuzzyTunerStruct fuzzyTuner = {
    .setpoint = 0.2,      // Target setpoint magnitude
    .prevError = 0.0,
    .prevTime = 0.0,
    .k_min = 0.3,         // Lowered from 0.5
    .k_max = 10.0         // Lowered from 20.0
};

// Define constants for fuzzy tuning regions with more crossover
#define ERROR_SMALL 0.07  // Increased from 0.05
#define ERROR_MEDIUM 0.13 // Decreased from 0.15
#define ERROR_CHANGE_SMALL 0.015 // Increased from 0.01
#define ERROR_CHANGE_MEDIUM 0.045 // Decreased from 0.05

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

// create a instance of 3D magnetic sensor
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
// define the number of calibration samples
const int CALIBRATION_SAMPLES = 20;
// offsets for calibration
double xOffset = 0, yOffset = 0, zOffset = 0;


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
  updateFuzzyParameters(); //change this
  discretePID();
  gripperPositionTracking();
  if (inputData.button2==LOW)
  {
    stateDataLoop.target_voltage = 1;
    stateDataLoop.gripping=false;
  } 
  else
    stateDataLoop.target_voltage = 0;
  if (inputData.button1==LOW)
  {
    stateDataLoop.gripping=true;
  }
  if (stateDataLoop.gripping)
      stateDataLoop.target_voltage=-PIDDataLoop.u; //besause - is positive
   
  if (stateDataLoop.zeroing)
      zeriongFunc();

  
  if (stateDataLoop.absoluteAngle> MAX_ANGLE  && stateDataLoop.target_voltage>0)
  {
    stateDataLoop.target_voltage=0;
  }
  outputDataLoop.target_voltage=stateDataLoop.target_voltage;

}
void outputResults(OutputDataStruct &outputData){

    motor.loopFOC();
    motor.move(outputData.target_voltage);
}
void serialComunication(InputDataStruct &inputData, OutputDataStruct &outputData){
  // Calculate magnitude for monitoring
  float magnitude = calculateMagnitude(stateDataLoop.x_filtered, stateDataLoop.y_filtered, stateDataLoop.z_filtered);
  
  static unsigned long lastDisplayTime = 0;
  static float previousK = 0;
  
  // Output formatted data for serial plotter (comma separated values)
  Serial.print(magnitude);        // Current magnetic field magnitude
  Serial.print(",");
  Serial.print(fuzzyTuner.setpoint); // Setpoint (target value)
  Serial.print(",");  
  Serial.print(PIDDataLoop.k);    // Current k value from fuzzy tuner
  Serial.print(",");  
  Serial.print(PIDDataLoop.u);    // Current control output
  Serial.print(",");
  Serial.println(outputData.target_voltage); // Actual voltage sent to motor
  
  // Every 500ms, print a detailed text report to show K changes clearly
  unsigned long currentMillis = millis();
  if (currentMillis - lastDisplayTime > 500) {
    lastDisplayTime = currentMillis;
    
    // Calculate change in K value
    float deltaK = PIDDataLoop.k - previousK;
    previousK = PIDDataLoop.k;
    
    // Print a more detailed text report
    Serial.print("MAG:");
    Serial.print(magnitude, 4);
    Serial.print(" K:");
    Serial.print(PIDDataLoop.k, 4);
    Serial.print(" ΔK:");
    Serial.print(deltaK, 4);
    Serial.print(" ERROR:");
    Serial.print(fuzzyTuner.setpoint - magnitude, 4);
    Serial.print(" OUT:");
    Serial.println(PIDDataLoop.u, 2);
  }
}

void discretePID()
{
  // Implement the discrete PID control logic here
  // This function is called in the loop() function
  // to perform the PID control calculations
  // and update the motor's target voltage accordingly.
  // You can use the inputData and outputData structures
  PIDDataLoop.y=calculateMagnitude(stateDataLoop.x_filtered, stateDataLoop.y_filtered, stateDataLoop.z_filtered);
  PIDDataLoop.T_passed=millis();
  PIDDataLoop.T=(PIDDataLoop.T_passed-PIDDataLoop.T_old)/1000.0;
  PIDDataLoop.T_old=PIDDataLoop.T_passed;

  float bi=(PIDDataLoop.k*PIDDataLoop.T)/PIDDataLoop.Ti;
  float br=PIDDataLoop.T/PIDDataLoop.Tt;
  float ad=PIDDataLoop.Td/(PIDDataLoop.Td+PIDDataLoop.N*PIDDataLoop.T);
  float bd=(PIDDataLoop.k*PIDDataLoop.Td*PIDDataLoop.N)/(PIDDataLoop.Td+PIDDataLoop.N*PIDDataLoop.T);


  PIDDataLoop.P=PIDDataLoop.k*(PIDDataLoop.b*PIDDataLoop.r-PIDDataLoop.y);
  PIDDataLoop.D=ad*PIDDataLoop.D-bd*(PIDDataLoop.y-PIDDataLoop.y_old);
  
  PIDDataLoop.v=PIDDataLoop.P;
  
  //PIDDataLoop.v=PIDDataLoop.P+PIDDataLoop.I+PIDDataLoop.D;
  
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
  PIDDataLoop.umax=3;
  PIDDataLoop.umin=-1;
  PIDDataLoop.y_old=0;
  PIDDataLoop.y=0;
  PIDDataLoop.r=PID_Setpoint;
  PIDDataLoop.T_old=millis()/1000.0;

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
    unsigned long now = millis();
    float currentAngle = inputDataLoop.tleSensor * (57.295779513); // convert to degrees
    float dt = (stateDataLoop.zeroLastTime == 0) ? 0 : (now - stateDataLoop.zeroLastTime) / 1000.0;
    stateDataLoop.zeroLastTime = now;
    float speed = (dt > 0) ? abs(currentAngle - stateDataLoop.zeroLastAngle) / dt : 0;
    stateDataLoop.zeroLastAngle = currentAngle;
    const float speedThreshold = 15; // deg/s threshold for stability
    Serial.println(dt);
    const unsigned long stableDuration = 1000; // ms required to be stable
    Serial.print("Speed: ");
    Serial.println(speed);
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
  
  stateDataLoop.absoluteAngle = newRaw + (stateDataLoop.numberOfRevolutions * 360) - stateDataLoop.degreeOffset;  
  stateDataLoop.oldRawAngle=newRaw;
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

void findingStablePosition() {
  // Implement the logic to find the stable position of the gripper
  // This function can be called when the gripper is not gripping
  // and you want to find a stable position based on the magnetic field data.
  // You can use the filtered magnetic data (x_filtered, y_filtered, z_filtered)
  // to determine the stable position.
  if(abs(stateDataLoop.x_filtered)>0.08)
  {
    stateDataLoop.target_voltage=0;
    
  }
  else
  {

    stateDataLoop.target_voltage=-5;
    
  }

}

void updateFuzzyParameters() {
  // Use the same setpoint as the PID reference input
  fuzzyTuner.setpoint = PIDDataLoop.r;
  
  // Calculate current magnetic field vector magnitude
  float magnitude = calculateMagnitude(stateDataLoop.x_filtered, stateDataLoop.y_filtered, stateDataLoop.z_filtered);
  
  // Calculate current error (difference between setpoint and actual magnitude)
  float error = fuzzyTuner.setpoint - magnitude;
  
  // Calculate time passed since last adjustment
  unsigned long currentTime = millis();
  float deltaTime = (fuzzyTuner.prevTime == 0) ? 0 : (currentTime - fuzzyTuner.prevTime) / 1000.0;
  
  // Calculate rate of change of error (only if we have a valid time difference)
  float errorChange = 0;
  if (deltaTime > 0.01) { // Avoid division by very small numbers
    errorChange = (error - fuzzyTuner.prevError) / deltaTime;
  }
  
  // Store current values for next iteration
  fuzzyTuner.prevError = error;
  fuzzyTuner.prevTime = currentTime;
  
  // Apply fuzzy logic to adjust the k value
  
  // Convert error and errorChange to absolute values for rule evaluation
  float absError = abs(error);
  float absErrorChange = abs(errorChange);
  
  // Membership values for error
  float errorSmall = 0, errorMedium = 0, errorLarge = 0;
  
  // Determine error membership
  if (absError < ERROR_SMALL) {
    errorSmall = 1.0;
  } else if (absError < ERROR_MEDIUM) {
    errorSmall = (ERROR_MEDIUM - absError) / (ERROR_MEDIUM - ERROR_SMALL);
    errorMedium = 1.0 - errorSmall;
  } else {
    errorMedium = (absError > 2*ERROR_MEDIUM) ? 0 : (2*ERROR_MEDIUM - absError) / ERROR_MEDIUM;
    errorLarge = 1.0 - errorMedium;
  }
  
  // Membership values for error change rate
  float changeSmall = 0, changeMedium = 0, changeLarge = 0;
  
  // Determine error change membership
  if (absErrorChange < ERROR_CHANGE_SMALL) {
    changeSmall = 1.0;
  } else if (absErrorChange < ERROR_CHANGE_MEDIUM) {
    changeSmall = (ERROR_CHANGE_MEDIUM - absErrorChange) / (ERROR_CHANGE_MEDIUM - ERROR_CHANGE_SMALL);
    changeMedium = 1.0 - changeSmall;
  } else {
    changeMedium = (absErrorChange > 2*ERROR_CHANGE_MEDIUM) ? 0 : (2*ERROR_CHANGE_MEDIUM - absErrorChange) / ERROR_CHANGE_MEDIUM;
    changeLarge = 1.0 - changeMedium;
  }
  
  // Fuzzy rule base - determine output membership with bias towards smaller K values
  float kSmall = 0, kMedium = 0, kLarge = 0;
  
  // Rule 1: If error is small and change is small, k should be small
  float rule1 = min(errorSmall, changeSmall);
  kSmall = max(kSmall, rule1);
  
  // Rule 2: If error is small and change is medium, k should be small (changed from medium)
  float rule2 = min(errorSmall, changeMedium);
  kSmall = max(kSmall, rule2 * 0.7);
  kMedium = max(kMedium, rule2 * 0.3);
  
  // Rule 3: If error is small and change is large, k should be medium (changed from large)
  float rule3 = min(errorSmall, changeLarge);
  kMedium = max(kMedium, rule3);
  
  // Rule 4: If error is medium and change is small, k should be small (changed from medium)
  float rule4 = min(errorMedium, changeSmall);
  kSmall = max(kSmall, rule4 * 0.6);
  kMedium = max(kMedium, rule4 * 0.4);
  
  // Rule 5: If error is medium and change is medium, k should be medium
  float rule5 = min(errorMedium, changeMedium);
  kMedium = max(kMedium, rule5);
  
  // Rule 6: If error is medium and change is large, k should be medium (changed from large)
  float rule6 = min(errorMedium, changeLarge);
  kMedium = max(kMedium, rule6 * 0.7);
  kLarge = max(kLarge, rule6 * 0.3);
  
  // Rule 7: If error is large and change is small, k should be medium (changed from large)
  float rule7 = min(errorLarge, changeSmall);
  kMedium = max(kMedium, rule7);
  
  // Rule 8: If error is large and change is medium, k should be large
  float rule8 = min(errorLarge, changeMedium);
  kMedium = max(kMedium, rule8 * 0.3);
  kLarge = max(kLarge, rule8 * 0.7);
  
  // Rule 9: If error is large and change is large, k should be large
  float rule9 = min(errorLarge, changeLarge);
  kLarge = max(kLarge, rule9);
  
  // Calculate medium value closer to the small value for lower overall gain
  float kMediumValue = fuzzyTuner.k_min + (fuzzyTuner.k_max - fuzzyTuner.k_min) * 0.4; // Biased towards lower value (0.4 instead of 0.5)
  
  // Defuzzify using weighted average method with bias towards smaller K values
  float numerator = (kSmall * fuzzyTuner.k_min * 1.2) +  // Give more weight to small K
                    (kMedium * kMediumValue) + 
                    (kLarge * fuzzyTuner.k_max * 0.9);   // Give less weight to large K
  
  float denominator = (kSmall * 1.2) + kMedium + (kLarge * 0.9);
  
  // Apply the calculated k value with safety checks
  if (denominator > 0.01) { // Avoid division by very small numbers
    PIDDataLoop.k = numerator / denominator;
    
    // Ensure k stays within bounds
    PIDDataLoop.k = constrain(PIDDataLoop.k, fuzzyTuner.k_min, fuzzyTuner.k_max);
    
    // Debug output
    Serial.print("Magnitude: ");
    Serial.print(magnitude);
    Serial.print(", Error: ");
    Serial.print(error);
    Serial.print(", Change: ");
    Serial.print(errorChange);
    Serial.print(", New K: ");
    Serial.println(PIDDataLoop.k);
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
  stateDataLoop.x_speed_time_prev = millis();

  _delay(1000);
}

void loop() {

    readInputs(inputDataLoop);
    executeLogic(inputDataLoop,outputDataLoop);
    outputResults(outputDataLoop);
    serialComunication(inputDataLoop,outputDataLoop);  // Uncommented this line to show values
  

}

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
