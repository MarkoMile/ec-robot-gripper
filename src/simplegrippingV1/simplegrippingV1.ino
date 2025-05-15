#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include "config.h"
#include <SimpleFOC.h>

#define MAX_ANGLE 1000
struct InputDataStruct
{
  double x, y, z;
  float tleSensor;
  float targetVoltage;
  bool button1;
  bool button2;
};

struct OutputDataStruct
{
  float target_voltage;
};

const bool DEBUG_LOGS = false;
struct stateStruct
{
  float target_voltage;
  float current;
  float voltage;
  float position;
  float velocity;
  float rawAngle;

  float oldRawAngle = 0;
  bool gripping;
  float v;
  float absoluteAngle;
  float oldAbsoluteAngle = -1234;
  float currentRawAngle;
  float totalDegreesTraveled;
  float degreeOffset;
  bool zeroing = true;
  float x_filtered = 0;
  float y_filtered = 0;
  float z_filtered = 0;
  float magneticAlphaFilter = 0.02;
  int numberOfRevolutions = 0;
  // zeroing tracking variables
  unsigned long zeroLastTime = 0;
  float zeroLastAngle = 0;
  unsigned long zeroStableStart = 0; // Adaptive gripping variables
  bool objectDetected = false;
  bool isHardObject = false;
  float stallDetectionThreshold; // degrees - minimum angle change expected
  unsigned long stallCheckLastTime = 0;
  float stallCheckLastAngle = 0;
  unsigned long stallDetectionTime = 0; // Time when stall was first detected
  unsigned long stallConfirmationTime;  // Time in ms to confirm stall
  float adaptiveGripForce;              // Default grip force
  float hardObjectForce;                // Higher force for hard/heavy objects
  float softObjectForce;                // Lower force for soft/deformable objects
  float magneticMagnitudeHardThreshold; // Threshold for hard object detection using magnetic magnitude
  float lastMagneticMagnitude;          // Last measured magnetic magnitude for comparison
  float angleChangeRate;                // Rate of angle change for elasticity detection
  float minAngleChangeRate;             // Minimum rate threshold for angle change
  // New variables for improved object detection
  bool isSoftObjectConfirmed = false;           // Flag for tracking confirmed soft objects
  unsigned long softObjectConfirmationTime = 0; // Time when soft object was first detected
  unsigned long softConfirmationDuration = 300; // Time required to confirm soft object (ms)
  int elasticityCounter = 0;                    // Counter for tracking elasticity detections
  int elasticityThreshold = 3;                  // Number of elasticity detections required to confirm soft object
  float elasticityMemory[5] = {0};              // Circular buffer to store recent elasticity readings
  int elasticityMemoryIndex = 0;                // Current index in elasticity memory
  float hardToSoftRatio = 3.0;                  // Ratio for hysteresis in hard-to-soft transition
  float softToHardRatio = 1.5;                  // Ratio for hysteresis in soft-to-hard transition
  // Magnetic sensor calibration variables
  float objectDetectionThreshold;
  bool calibrationNeeded = false;
  bool isCalibrating = false;
  unsigned long calibrationStartTime = 0;
  unsigned long calibrationDuration = 1500; // Time to complete calibration in ms
};
struct PIDStruct
{
  float k;
  long T;
  long T_old;
  long T_passed;
  float Ti;
  float Td;
  float Tt;
  float N = 5;
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
InputDataStruct inputDataLoop;   // sadrzi informacije o senzorima
OutputDataStruct outputDataLoop; // sadrzi izlazne podatke
stateStruct stateDataLoop;       // sadrzi stanje motora
PIDStruct PIDDataLoop;           // sadrzi PID parametre
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

void readInputs(InputDataStruct &inputData)
{
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

  // Check for calibration request (both buttons pressed)
  checkAndHandleMagneticCalibration(inputData);
  // Only handle button2 for stopping gripping
  if (!stateDataLoop.isCalibrating && !stateDataLoop.calibrationNeeded)
  {
    // Button1 now initiates calibration in checkAndHandleMagneticCalibration
    if (inputData.button2 == LOW)
      stateDataLoop.gripping = false;
  }

  tle5012Sensor.update();
  inputData.tleSensor = tle5012Sensor.getSensorAngle();
}
void executeLogic(InputDataStruct &inputData, OutputDataStruct &outputData)
{
  filterMagneticData();
  discretePID();
  gripperPositionTracking();

  // Previous gripping state to detect changes
  bool wasGripping = stateDataLoop.gripping;

  // Skip normal control logic during calibration
  if (stateDataLoop.isCalibrating || stateDataLoop.calibrationNeeded)
  {
    // Force motor to stop during calibration
    stateDataLoop.target_voltage = 0;
    outputDataLoop.target_voltage = 0;
    return;
  }

  if (inputData.button2 == LOW)
  {
    stateDataLoop.target_voltage = 1;
    stateDataLoop.gripping = false;
  }
  else
    stateDataLoop.target_voltage = 0;
  // Button1 now initiates calibration in checkAndHandleMagneticCalibration

  // If gripping state changed, reset adaptive grip state
  if (wasGripping != stateDataLoop.gripping)
  {
    resetAdaptiveGripState();
    if (stateDataLoop.gripping)
    {
      Serial.println("Starting adaptive gripping procedure");
    }
    else
    {
      Serial.println("Stopping gripping procedure");
    }
  }

  if (stateDataLoop.gripping)
    findingStablePosition();

  if (stateDataLoop.zeroing)
    zeroingFunction();
  if (stateDataLoop.absoluteAngle > MAX_ANGLE && stateDataLoop.target_voltage > 0)
  {
    stateDataLoop.target_voltage = 0;
  }
  outputDataLoop.target_voltage = stateDataLoop.target_voltage;
}
void outputResults(OutputDataStruct &outputData)
{

  motor.loopFOC();
  motor.move(outputData.target_voltage);
}
void serialComunication(InputDataStruct &inputData, OutputDataStruct &outputData)
{
  // Only send detailed status data periodically to avoid flooding serial port
  static unsigned long lastDetailedStatus = 0;
  unsigned long currentTime = millis();

  // Always send calibration status changes
  static bool lastCalibrationState = false;
  bool currentCalibrationState = stateDataLoop.isCalibrating || stateDataLoop.calibrationNeeded;

  if (currentCalibrationState != lastCalibrationState)
  {
    if (currentCalibrationState)
    {
      Serial.println("STATUS: Magnetic sensor calibration in progress");
    }
    else if (lastCalibrationState)
    {
      Serial.println("STATUS: Magnetic sensor calibration complete");

      // Print the new calibration values
      Serial.print("New calibration values - X: ");
      Serial.print(xOffset);
      Serial.print(", Y: ");
      Serial.print(yOffset);
      Serial.print(", Z: ");
      Serial.println(zOffset);
    }
    lastCalibrationState = currentCalibrationState;
  }

  // Send detailed status every 400ms
  if (currentTime - lastDetailedStatus >= 400)
  {
    // Uncomment this for detailed JSON status

    Serial.print("{\"x\":");
    Serial.print(stateDataLoop.x_filtered);
    Serial.print(",\"y\":");
    Serial.print(stateDataLoop.y_filtered);
    Serial.print(",\"z\":");
    Serial.print(stateDataLoop.z_filtered);
    Serial.print(",\"angle\":");
    Serial.print(stateDataLoop.absoluteAngle);
    Serial.print(",\"targetVoltage\":");
    Serial.print(outputData.target_voltage);
    Serial.print(",\"objectDetected\":");
    Serial.print(stateDataLoop.objectDetected ? "true" : "false");
    Serial.print(",\"isHardObject\":");
    Serial.print(stateDataLoop.isHardObject ? "true" : "false");
    Serial.print(",\"gripping\":");
    Serial.print(stateDataLoop.gripping ? "true" : "false");
    Serial.print(",\"calibrating\":");
    Serial.print(currentCalibrationState ? "true" : "false");
    Serial.print(",\"isSoftObjectConfirmed\":");
    Serial.print(stateDataLoop.isSoftObjectConfirmed ? "true" : "false");
    Serial.print(",\"angleChangeRate\":");
    Serial.print(stateDataLoop.angleChangeRate);
    Serial.print(",\"magneticMagnitude\":");
    Serial.print(calculateMagneticMagnitude());
    Serial.println("}"); // print all variables used in logic for debugging
    if (DEBUG_LOGS)
    {
      Serial.println("\n=== OBJECT DETECTION DEBUG ===");
      Serial.print("Magnetic Magnitude: ");
      Serial.print(calculateMagneticMagnitude());
      Serial.print(" (Threshold: ");
      Serial.print(stateDataLoop.objectDetectionThreshold);
      Serial.println(")");

      Serial.print("Hard Object Threshold: ");
      Serial.println(stateDataLoop.magneticMagnitudeHardThreshold);
      Serial.print("Angle Change Rate: ");
      Serial.print(stateDataLoop.angleChangeRate);
      Serial.print(" deg/s (Min: ");
      Serial.print(stateDataLoop.minAngleChangeRate);
      Serial.println(" deg/s)");

      Serial.println("\n=== OBJECT CLASSIFICATION STATUS ===");
      Serial.print("Object Detected: ");
      Serial.println(stateDataLoop.objectDetected ? "YES" : "NO");
      Serial.print("Is Hard Object: ");
      Serial.println(stateDataLoop.isHardObject ? "YES" : "NO");
      Serial.print("Is Soft Confirmed: ");
      Serial.println(stateDataLoop.isSoftObjectConfirmed ? "YES" : "NO");

      Serial.println("\n=== HYSTERESIS VARIABLES ===");
      Serial.print("Elasticity Counter: ");
      Serial.print(stateDataLoop.elasticityCounter);
      Serial.print("/");
      Serial.println(stateDataLoop.elasticityThreshold);

      // Calculate average elasticity
      // float avgElasticity = 0;
      // for (int i = 0; i < 5; i++)
      // {
      //   avgElasticity += stateDataLoop.elasticityMemory[i];
      // }
      // avgElasticity /= 5;

      // Serial.print("Average Elasticity: ");
      // Serial.println(avgElasticity);
      Serial.print("Soft-to-Hard Ratio: ");
      Serial.println(stateDataLoop.softToHardRatio);
      Serial.print("Hard-to-Soft Ratio: ");
      Serial.println(stateDataLoop.hardToSoftRatio);

      Serial.println("\n=== TIMING VARIABLES ===");
      Serial.print("Stall Detection Time: ");
      if (stateDataLoop.stallDetectionTime > 0)
      {
        Serial.print(currentTime - stateDataLoop.stallDetectionTime);
        Serial.print("/");
        Serial.println(stateDataLoop.stallConfirmationTime);
      }
      else
      {
        Serial.println("Not active");
      }

      Serial.print("Soft Confirmation Time: ");
      if (stateDataLoop.softObjectConfirmationTime > 0)
      {
        Serial.print(currentTime - stateDataLoop.softObjectConfirmationTime);
        Serial.print("/");
        Serial.println(stateDataLoop.softConfirmationDuration);
      }
      else
      {
        Serial.println("Not active");
      }

      Serial.print("Target Voltage: ");
      Serial.println(stateDataLoop.target_voltage);
      Serial.println("==============================\n");
    }

    lastDetailedStatus = currentTime;
  }
}

void discretePID()
{
  // Implement the discrete PID control logic here
  // This function is called in the loop() function
  // to perform the PID control calculations
  // and update the motor's target voltage accordingly.
  // You can use the inputData and outputData structures
  PIDDataLoop.T_passed = millis();
  PIDDataLoop.T = (PIDDataLoop.T_passed - PIDDataLoop.T_old) / 1000.0;
  PIDDataLoop.T_old = PIDDataLoop.T_passed;
  float bi = (PIDDataLoop.k * PIDDataLoop.T) / PIDDataLoop.Ti;
  float br = PIDDataLoop.T / PIDDataLoop.Tt;
  float ad = PIDDataLoop.Td / (PIDDataLoop.Td + PIDDataLoop.N * PIDDataLoop.T);
  float bd = (PIDDataLoop.k * PIDDataLoop.Td * PIDDataLoop.N) / (PIDDataLoop.Td + PIDDataLoop.N * PIDDataLoop.T);

  PIDDataLoop.y = inputDataLoop.targetVoltage;
  // Use magnetic magnitude instead of just x component
  PIDDataLoop.r = calculateMagneticMagnitude();
  PIDDataLoop.P = PIDDataLoop.k * (PIDDataLoop.b * PIDDataLoop.r - PIDDataLoop.y);
  PIDDataLoop.D = ad * PIDDataLoop.D - bd * (PIDDataLoop.y - PIDDataLoop.y_old);

  PIDDataLoop.v = PIDDataLoop.P + PIDDataLoop.I + PIDDataLoop.D;

  if (PIDDataLoop.v > PIDDataLoop.umax)
    PIDDataLoop.u = PIDDataLoop.umax;
  else if (PIDDataLoop.v < PIDDataLoop.umin)
    PIDDataLoop.u = PIDDataLoop.umin;
  else
    PIDDataLoop.u = PIDDataLoop.v;

  PIDDataLoop.I = PIDDataLoop.I + bi * (PIDDataLoop.r - PIDDataLoop.y) + br * (PIDDataLoop.u - PIDDataLoop.v);
  PIDDataLoop.y_old = PIDDataLoop.y;
}
void initDiscretePID()
{
  PIDDataLoop.b = 0.9;
  PIDDataLoop.Ti = 1;
  PIDDataLoop.Tt = 1;
  PIDDataLoop.Td = 1;
  PIDDataLoop.N = 6;
  PIDDataLoop.k = 1;
  PIDDataLoop.P = 0;
  PIDDataLoop.I = 0;
  PIDDataLoop.D = 0;
  PIDDataLoop.umax = 1;
  PIDDataLoop.umin = -1;
  PIDDataLoop.y_old = 0;
  PIDDataLoop.y = 0;
  PIDDataLoop.r = 0;
  PIDDataLoop.T_old = millis() / 1000.0;

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

void zeroingFunction()
{
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
  if (speed > speedThreshold)
  {
    // still moving: continue zeroing motion
    stateDataLoop.target_voltage = -1;
    stateDataLoop.zeroStableStart = 0;
    Serial.println("zeroing in progress");
  }
  else
  {
    // detected low speed: start stable timer
    if (stateDataLoop.zeroStableStart == 0)
    {
      stateDataLoop.zeroStableStart = now;
    }
    if (now - stateDataLoop.zeroStableStart >= stableDuration)
    {
      // arm is stable: set as zero position
      stateDataLoop.target_voltage = 0;
      stateDataLoop.zeroing = false;
      stateDataLoop.degreeOffset = currentAngle;
      stateDataLoop.numberOfRevolutions = 0;
      Serial.println("zeroing done");
    }
    else
    {
      // waiting to confirm stability
      stateDataLoop.target_voltage = -1;
      Serial.println("zeroing stable...");
    }
  }
}

void gripperPositionTracking()
{
  // 1. Read the new sensor value
  float newRaw = tle5012Sensor.getSensorAngle() * (57.295779513); // convert to degrees
  // 2. Compute raw delta
  if (newRaw < 30 && stateDataLoop.oldRawAngle > 330)
  {
    // if the new value is less than 30 and the old value is greater than 330
    // then we have crossed the 0 degree line
    stateDataLoop.numberOfRevolutions++;
  }
  else if (newRaw > 330 && stateDataLoop.oldRawAngle < 30)
  {
    // if the new value is greater than 330 and the old value is less than 30
    // then we have crossed the 360 degree line
    stateDataLoop.numberOfRevolutions--;
  }
  // 3. Unwrap at ±half‐circle

  stateDataLoop.absoluteAngle = newRaw + (stateDataLoop.numberOfRevolutions * 360) - stateDataLoop.degreeOffset;
  stateDataLoop.oldRawAngle = newRaw;
}

void filterMagneticData()
{
  // Apply a low-pass filter to the magnetic field data
  stateDataLoop.x_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.x_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.x;
  stateDataLoop.y_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.y_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.y;
  stateDataLoop.z_filtered = (1 - stateDataLoop.magneticAlphaFilter) * stateDataLoop.z_filtered + stateDataLoop.magneticAlphaFilter * inputDataLoop.z;

  // Update lastMagneticMagnitude for reference
  stateDataLoop.lastMagneticMagnitude = calculateMagneticMagnitude();
}

void findingStablePosition()
{
  unsigned long currentTime = millis();
  float currentAngle = stateDataLoop.absoluteAngle; // First, detect if an object is present using magnetic field magnitude
  // Using magnetic magnitude instead of just x-axis for more accurate detection
  float magneticMagnitude = calculateMagneticMagnitude();

  if (magneticMagnitude > stateDataLoop.objectDetectionThreshold) // Threshold for initial object detection
  {
    // Object detected
    if (!stateDataLoop.objectDetected)
    {
      // First detection of object
      stateDataLoop.objectDetected = true;
      stateDataLoop.stallCheckLastTime = currentTime;
      stateDataLoop.stallCheckLastAngle = currentAngle;
      stateDataLoop.stallDetectionTime = 0;
      stateDataLoop.softObjectConfirmationTime = 0;
      Serial.println("Object detected, starting adaptive grip");
    }

    // Apply adaptive grip force based on object hardness
    // If we've confirmed it's a soft object, always use soft force
    if (stateDataLoop.isSoftObjectConfirmed)
    {
      stateDataLoop.target_voltage = stateDataLoop.softObjectForce;
    }
    else
    {
      stateDataLoop.target_voltage = stateDataLoop.isHardObject ? stateDataLoop.hardObjectForce : stateDataLoop.softObjectForce;
    }

    if (currentTime - stateDataLoop.stallCheckLastTime >= 100)
    { // Check every 100ms
      // Calculate the magnetic field magnitude
      float magneticMagnitude = calculateMagneticMagnitude();

      // Calculate angle change rate in degrees per second
      float dt = (currentTime - stateDataLoop.stallCheckLastTime) / 1000.0; // Time in seconds
      float angleChange = abs(currentAngle - stateDataLoop.stallCheckLastAngle);
      float angleRate = (dt > 0) ? angleChange / dt : 0;

      // Save current angle and time for next calculation
      stateDataLoop.stallCheckLastAngle = currentAngle;
      stateDataLoop.stallCheckLastTime = currentTime;

      // Store angle change rate for reference
      stateDataLoop.angleChangeRate = angleRate;

      // Add this reading to our elasticity memory
      stateDataLoop.elasticityMemory[stateDataLoop.elasticityMemoryIndex] = angleRate;
      stateDataLoop.elasticityMemoryIndex = (stateDataLoop.elasticityMemoryIndex + 1) % 3; // Circular buffer, 3 elem

      // Debug output
      // Serial.print("Angle rate: ");
      // Serial.print(angleRate);
      // Serial.print(" deg/s, Magnetic magnitude: ");
      // Serial.println(magneticMagnitude);      // Combined detection logic using both magnetic magnitude and angle change rate
      bool isMovingSignificantly = angleRate > stateDataLoop.minAngleChangeRate;
      bool hasMagneticSignal = magneticMagnitude > stateDataLoop.magneticMagnitudeHardThreshold;

      if (stateDataLoop.isSoftObjectConfirmed)
      {
        // Only consider changing to hard if magnetic signal is much stronger than threshold
        // This creates hysteresis to prevent oscillation
        if (hasMagneticSignal &&
            !isMovingSignificantly &&
            magneticMagnitude > (stateDataLoop.magneticMagnitudeHardThreshold * stateDataLoop.softToHardRatio))
        {
          // Require longer confirmation for switching from soft to hard (5x)
          if (stateDataLoop.stallDetectionTime == 0)
          {
            stateDataLoop.stallDetectionTime = currentTime;
          }
          else if (currentTime - stateDataLoop.stallDetectionTime >= stateDataLoop.stallConfirmationTime * 5)
          {
            // Hard object confirmed after extended confirmation period
            stateDataLoop.isSoftObjectConfirmed = false; // No longer confirmed as soft
            stateDataLoop.isHardObject = true;
            Serial.print("Soft object changed to hard (magnitude: ");
            Serial.print(magneticMagnitude);
            // Serial.print(", avg elasticity: ");
            // Serial.print(avgElasticity);
            Serial.println("), increasing grip force");
            stateDataLoop.target_voltage = stateDataLoop.hardObjectForce;
          }
        }
        else
        {
          // Any sign of movement confirms it's still soft
          stateDataLoop.stallDetectionTime = 0;
        }
      }
      // Not confirmed as soft yet

      else if (hasMagneticSignal && !isMovingSignificantly)
      {
        // High magnetic signal but not moving significantly - likely a hard object
        if (stateDataLoop.stallDetectionTime == 0)
        {
          // First time detecting potential hard object
          stateDataLoop.stallDetectionTime = currentTime;
        }
        else if (currentTime - stateDataLoop.stallDetectionTime >= stateDataLoop.stallConfirmationTime)
        {
          // Hard object confirmed after confirmation period, but only if not confirmed as soft
          if (!stateDataLoop.isHardObject && !stateDataLoop.isSoftObjectConfirmed)
          {
            stateDataLoop.isHardObject = true;
            Serial.print("Hard object detected (magnitude: ");
            Serial.print(magneticMagnitude);
            Serial.print(", angle rate: ");
            Serial.print(angleRate);
            Serial.println(" deg/s), increasing grip force");
            stateDataLoop.target_voltage = stateDataLoop.hardObjectForce;
          }
        }
      }
      // else if ((isMovingSignificantly || avgElasticity > stateDataLoop.minAngleChangeRate) && hasMagneticSignal)
      else if (isMovingSignificantly && hasMagneticSignal)
      {
        // If we're still moving significantly even with magnetic signal, it's probably elastic/soft
        stateDataLoop.stallDetectionTime = 0;

        // Count this as evidence for a soft object
        stateDataLoop.elasticityCounter++;

        // Start tracking when we first detected elasticity
        if (stateDataLoop.softObjectConfirmationTime == 0)
        {
          stateDataLoop.softObjectConfirmationTime = currentTime;
        }

        // Check if we have enough evidence for a soft object
        // // We use both a counter threshold AND a time threshold
        // // Also check that average elasticity is above threshold for more reliable classification
        // if ((stateDataLoop.elasticityCounter >= stateDataLoop.elasticityThreshold && avgElasticity > stateDataLoop.minAngleChangeRate) ||
        //     (currentTime - stateDataLoop.softObjectConfirmationTime >= stateDataLoop.softConfirmationDuration && avgElasticity > stateDataLoop.minAngleChangeRate))
        if (stateDataLoop.elasticityCounter >= stateDataLoop.elasticityThreshold ||
            (currentTime - stateDataLoop.softObjectConfirmationTime >= stateDataLoop.softConfirmationDuration))
        {
          // Confirm as a soft object
          stateDataLoop.isSoftObjectConfirmed = true;
          stateDataLoop.isHardObject = false;

          if (stateDataLoop.isHardObject)
          {
            Serial.print("Hard object changed to soft/elastic (moving at: ");
            Serial.print(angleRate);
            // Serial.print(" deg/s, avg elasticity: ");
            // Serial.print(avgElasticity);
            Serial.println("), applying softer grip force");
          }
          else
          {
            Serial.print("Soft/elastic object confirmed (moving at: ");
            Serial.print(angleRate);
            // Serial.print(" deg/s, avg elasticity: ");
            // Serial.print(avgElasticity);
            Serial.println("), maintaining soft grip force");
          }

          stateDataLoop.target_voltage = stateDataLoop.softObjectForce;
        }
      }
      else
      {
        // Reset stall detection if conditions not met
        stateDataLoop.stallDetectionTime = 0;
      }

      // Save the last magnitude for comparison
      stateDataLoop.lastMagneticMagnitude = magneticMagnitude; // More detailed debug output for inline monitoring
      if (DEBUG_LOGS && currentTime % 1000 < 100)
      { // Only print every ~1 second to avoid flooding
        Serial.println("\n--- REAL-TIME CLASSIFICATION DATA ---");
        Serial.print("Current Object State: ");
        if (stateDataLoop.isSoftObjectConfirmed)
        {
          Serial.println("CONFIRMED SOFT");
        }
        else if (stateDataLoop.isHardObject)
        {
          Serial.println("HARD");
        }
        else if (stateDataLoop.objectDetected)
        {
          Serial.println("UNCLASSIFIED");
        }
        else
        {
          Serial.println("NONE");
        }

        Serial.print("Current criteria: Moving=");
        Serial.print(isMovingSignificantly ? "YES" : "NO");
        Serial.print(", MagneticSignal=");
        Serial.println(hasMagneticSignal ? "YES" : "NO");

        Serial.print("Decision logic: ");
        if (stateDataLoop.isSoftObjectConfirmed)
        {
          Serial.println("Staying in SOFT mode until strong evidence shows otherwise");
        }
        else if (hasMagneticSignal && !isMovingSignificantly)
        {
          Serial.println("Potential HARD object: high magnetic signal, low movement");
        }
        // else if ((isMovingSignificantly || avgElasticity > stateDataLoop.minAngleChangeRate) && hasMagneticSignal)
        else if (isMovingSignificantly && hasMagneticSignal)
        {
          Serial.println("Potential SOFT object: significant movement despite magnetic signal");
        }
        else
        {
          Serial.println("Inconclusive - monitoring");
        }
        Serial.println("---------------------------------------");
      }
    }
  }
  else
  {
    // No object detected, close gripper at normal speed
    stateDataLoop.target_voltage = stateDataLoop.adaptiveGripForce;
    stateDataLoop.objectDetected = false;
    stateDataLoop.isHardObject = false;
    stateDataLoop.isSoftObjectConfirmed = false;
    stateDataLoop.stallDetectionTime = 0;
    stateDataLoop.elasticityCounter = 0;
  }
}

void resetAdaptiveGripState()
{
  stateDataLoop.objectDetected = false;
  stateDataLoop.isHardObject = false;
  stateDataLoop.isSoftObjectConfirmed = false;
  stateDataLoop.stallDetectionTime = 0;
  stateDataLoop.softObjectConfirmationTime = 0;
  stateDataLoop.elasticityCounter = 0;
  stateDataLoop.elasticityMemoryIndex = 0;
  stateDataLoop.stallCheckLastTime = millis();
  stateDataLoop.stallCheckLastAngle = stateDataLoop.absoluteAngle;

  // Reset elasticity memory
  for (int i = 0; i < 5; i++)
  {
    stateDataLoop.elasticityMemory[i] = 0;
  }
}

/**
 * @brief Calculates the magnitude of the magnetic field vector using the filtered x, y, and z components
 * @return The magnitude of the magnetic field vector
 */
float calculateMagneticMagnitude()
{
  // Calculate magnitude using the Euclidean norm formula: sqrt(x² + y² + z²)
  return sqrt(
      (stateDataLoop.x_filtered * stateDataLoop.x_filtered) +
      (stateDataLoop.y_filtered * stateDataLoop.y_filtered) +
      (stateDataLoop.z_filtered * stateDataLoop.z_filtered));
}

void initAdaptiveGripping()
{ // forces and thresholds and rates:
  // force is negative for gripping (larger absolute value means more force)
  stateDataLoop.adaptiveGripForce = -1.5;             // Default grip force
  stateDataLoop.hardObjectForce = -3.5;               // Higher force for hard/heavy objects
  stateDataLoop.softObjectForce = -1;                 // Lower force for soft/deformable objects
  stateDataLoop.stallDetectionThreshold = 0.8;        // degrees - minimum angle change expected (lowered for sensitivity)
  stateDataLoop.stallConfirmationTime = 150;          // Time to confirm object hardness (ms) - increased for stability
  stateDataLoop.magneticMagnitudeHardThreshold = 0.3; // Threshold for hard object detection using magnitude
  // Note: Initial detection threshold is lower (0.07) to catch all objects
  stateDataLoop.objectDetectionThreshold = 0.08; // magnetic magnitude threshold
  stateDataLoop.minAngleChangeRate = 50;         // Minimum angle change rate for elastic objects (degrees/100ms)
  stateDataLoop.lastMagneticMagnitude = 0.0;
  stateDataLoop.angleChangeRate = 0.0;

  // Improved hysteresis values to prevent oscillation
  stateDataLoop.hardToSoftRatio = 2.5; // Reduced from 3.0 for easier transition to soft
  stateDataLoop.softToHardRatio = 3;   // Increased from 1.5 for more stability

  // Initialize new detection variables
  stateDataLoop.elasticityCounter = 0;
  stateDataLoop.elasticityThreshold = 2;
  stateDataLoop.elasticityMemoryIndex = 0;
  stateDataLoop.isSoftObjectConfirmed = false;
  stateDataLoop.softObjectConfirmationTime = 0;
  stateDataLoop.softConfirmationDuration = 300; // ms

  // Clear elasticity memory array
  for (int i = 0; i < 5; i++)
  {
    stateDataLoop.elasticityMemory[i] = 0;
  }

  resetAdaptiveGripState();
}

void setup()
{
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
  if (!driver.init())
  {
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
  initAdaptiveGripping();

  // set the pin modes for buttons
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  Serial.print("setup done.\n");

  _delay(1000);
}

void loop()
{

  readInputs(inputDataLoop);
  executeLogic(inputDataLoop, outputDataLoop);
  outputResults(outputDataLoop);
  serialComunication(inputDataLoop, outputDataLoop);
}

/**
 * @brief Check if grip button is pressed and handle magnetic calibration.
 */
void checkAndHandleMagneticCalibration(InputDataStruct &inputData)
{
  unsigned long currentTime = millis();

  // Check if grip button is pressed
  if (inputData.button1 == LOW)
  {
    // button 1 pressed, trigger calibration
    if (!stateDataLoop.isCalibrating && !stateDataLoop.calibrationNeeded)
    {
      stateDataLoop.calibrationNeeded = true;
      Serial.println("Magnetic sensor calibration initiated!");
    }
  }

  // Handle calibration process
  if (stateDataLoop.calibrationNeeded)
  {
    if (!stateDataLoop.isCalibrating)
    {
      // Start calibration
      stateDataLoop.isCalibrating = true;
      stateDataLoop.calibrationStartTime = currentTime;

      // Stop motor during calibration
      stateDataLoop.target_voltage = 0;

      // Perform the actual calibration
      Serial.println("Starting magnetic sensor recalibration...");
      recalibrateMagneticSensor();

      Serial.println("Magnetic sensor recalibration complete!");
    }
    else
    { // Calibration cooldown period to allow sensor readings to stabilize
      if (currentTime - stateDataLoop.calibrationStartTime >= stateDataLoop.calibrationDuration)
      {
        stateDataLoop.isCalibrating = false;
        stateDataLoop.calibrationNeeded = false;
        // Automatically start gripping after calibration completes
        stateDataLoop.gripping = true;
        Serial.println("Calibration process complete. Starting gripping automatically.");
      }
    }
  }
}

/**
 * @brief Recalibrates the magnetic field sensor by recalculating offsets.
 * This function should be called when the magnetic sensor needs to be
 * recalibrated due to deformation or environmental changes.
 */
void recalibrateMagneticSensor()
{
  double sumX = 0, sumY = 0, sumZ = 0;

  // Visual indication that calibration is happening
  Serial.println("Hold gripper still for calibration");

  // Temporarily halt all motion
  motor.move(0);

  for (int i = 0; i < CALIBRATION_SAMPLES; ++i)
  {
    double temp;
    double valX, valY, valZ;

    dut.getMagneticFieldAndTemperature(&valX, &valY, &valZ, &temp);
    sumX += valX;
    sumY += valY;
    sumZ += valZ;

    // Flash LED or some other visual feedback
    if (i % 5 == 0)
    {
      Serial.print(".");
    }

    delay(10); // Time between samples
  }

  Serial.println("");

  // Calculate and apply new offsets
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;

  // Print new calibration values
  Serial.print("New calibration values - X: ");
  Serial.print(xOffset);
  Serial.print(", Y: ");
  Serial.print(yOffset);
  Serial.print(", Z: ");
  Serial.println(zOffset);
}

#if ENABLE_MAGNETIC_SENSOR
/**
 * @brief Calibrates the magnetic field sensor by calculating the average
 * offsets for the X, Y, and Z axes over a series of samples.
 */
void calibrateSensor()
{
  double sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; ++i)
  {
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