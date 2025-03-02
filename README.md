# Universal-Auto-Adapting-Motor-System
This is a conceptual and educational project. Actual hardware implementations will require carefully selected components, advanced power management, compliance with regulations, and possibly specialized libraries (especially for high-power or nuclear-based sources).

## This repository contains conceptual Arduino-style code for a futuristic motor control system that:

Uses a nano-diamond nuclear battery (trickle) + buffer (supercapacitor or battery) as its power source.
Adapts automatically to different loads (small, medium, large) by measuring current and applying different PID gains.
Includes an optional Shape-Memory Alloy (SMA) “transformation” mechanism to alter the motor’s physical configuration or other structural elements.
Implements basic low-power management to conserve energy when idle or when battery voltage is too low.
Disclaimer: This is a conceptual and educational project. Actual hardware implementations will require carefully selected components, advanced power management, compliance with regulations, and possibly specialized libraries (especially for high-power or nuclear-based sources).

## Project Overview
In this example, the motor code detects the load by measuring current (via a sensor like the ACS712). It classifies the load as small, medium, or large and adapts PID gains to manage speed control efficiently.

Additionally, it measures battery voltage through a voltage divider and decides when to enter a low-power mode if the voltage is below a safe threshold. The premise is that a nano-diamond battery trickle-charges a supercapacitor or lithium-ion buffer, providing enough power for short bursts of higher current.

Finally, a Shape-Memory Alloy (SMA) actuator is toggled when certain conditions are met, simulating a “transformer-like” mechanical reconfiguration.


---

## Table of Contents

- [Adaptive PID Control](#adaptive-pid-control)
- [Load Classification](#load-classification)
- [Battery Monitoring](#battery-monitoring)
- [Shape-Memory Alloy](#shape-memory-alloy)
- [Low-Power Mode](#low-power-mode)
- [License](#license)
- [Key Additions](#Key-Additions)

---

## Key Additions

### Temperature Monitoring (`readMotorTemperature()`)
- We added a pin (`TEMP_SENSOR_PIN`) and a mock formula for converting ADC values to Celsius.
- `meltdownAvoidanceProtocol()` tries to reduce speed and prints comedic messages.

---

### Flagellar Mode (`flagellarModeActive`)
- If enabled, we apply a sine wave modulation in `applyFlagellarOscillation()`.
- For a servo, we similarly tweak the microsecond command.

---

### Near-100% Nuclear Usage (`optimizeNuclearUsage()`)
- Purely fictional placeholder function that prints a comedic message about gleaning every bit of nuclear decay.

---

### Nanobot Auto-Repair (`nanobotRepairSystemCheck()`)
- A random chance triggers “damage,” and then the “nanobots” fix it.
- In real hardware, you’d have error-check routines or sensor networks to detect actual mechanical/electrical faults.

---

### Meltdown Avoidance (`meltdownAverted`)
- If the temperature is too high, we try to salvage the situation. If it fails, we shut down.


## Hardware Requirements

Arduino-Compatible Board
Arduino UNO, Nano, Mega, ESP32, STM32, etc.

Motor

Brushless DC (BLDC) with Hall sensors or an encoder (recommended), OR a continuous rotation servo (simpler, but less feedback).

Motor Driver

Must handle the motor’s voltage/current. For BLDC, something like a specialized 3-phase driver or a VESC. For servos, the driver is often built-in.

Current Sensor

Example: ACS712 module, INA219 (I2C), or the driver’s built-in current sense.

Voltage Divider

To measure buffer/battery voltage.

Shape-Memory Alloy

Optional. Could be a nitinol wire or spring with a dedicated driver transistor.

Nano-Diamond Battery + Buffer

The battery itself is research-level tech. In practice, replace with a DC source or small LiPo battery to simulate the trickle charge.

Important: For actual projects, ensure regulatory compliance and safety measures for nuclear sources or high-power motors.

## Circuit Diagram (Conceptual)
~~~
 Nano-Diamond Battery
         |
         v   (Very low current trickle)
 [ Buffer: Supercap / Li-ion ]
         |
         |----> [ DC-DC Step-Up ] ---> +5V or +12V Rail
         |
         +---> Arduino VIN/5V pin
              +--> Motor Driver Vcc (logic)
              +--> Current Sensor (if needed)
              +--> ...
         |
         +--> Motor Driver Power (VMotor) 
               |
               |----> Motor Output (BLDC or Servo)
               |
               '----> Hall/Encoder feedback to Arduino pins
         
Arduino Pins:
 - A0 (BATTERY_SENSE_PIN) -> Voltage divider on buffer
 - A1 (CURRENT_SENSE_PIN) -> ACS712 / INA219 (or driver sense)
 - D9 (MOTOR_PWM_PIN)     -> Motor driver PWM input (BLDC) or servo signal
 - D8 (MOTOR_DIR_PIN)     -> Direction control (BLDC only)
 - D2, D3 (ENCODER_A/B)   -> Encoder feedback
 - D5 (SMA_PIN)           -> Shape-Memory Alloy control
~~~
## Note: Pin assignments are examples; adapt to your hardware

## Code Explanation

## Adaptive PID Control
- A basic PID loop uses **current speed** (`currentSpeed`) and **target speed** (`targetSpeed`).
- We adjust the PID constants (`kp`, `ki`, `kd`) based on the classified load size (**small**, **medium**, **large**).

---

## Load Classification
- We read the motor **current** from a sensor and compare it to thresholds (**5A** and **50A** in this example).
- The code then calls `applyPIDGains()` to switch to different PID constants.

---

## Battery Monitoring
- `readBatteryVoltage()` uses an analog pin (`A0`) and a known **voltage divider** ratio to compute the actual buffer/battery voltage.
- If the voltage drops below `BATTERY_MIN_VOLTAGE`, the system **stops the motor** and calls `goToLowPowerMode()`.

---

## Shape-Memory Alloy
- `shouldTransform()` returns `true` under certain conditions (e.g., **large device type** + speed above **80 RPM**).
- `activateSMA(true)` applies current to the SMA pin. In real setups, careful **thermal control** is needed.

---

## Low-Power Mode
- The `goToLowPowerMode()` function is a **placeholder**.
- In practice, you’d use **MCU-specific sleep or deep-sleep methods** to conserve energy.














# Universal Auto-Adapting Motor System

Below is the complete Arduino-style source code. Copy this into a file like `main.ino` or `AutoAdaptMotor.ino` in your Arduino IDE or similar environment.
~~~
/************************************************************
 *  Universal Auto-Adapting Motor System (Combined Edition)
 *  ---------------------------------------------------------
 *  This sketch merges:
 *    - Battery End-of-Life logic (hibernation, recharge check)
 *    - Flagellar motor oscillation
 *    - Temperature-based meltdown avoidance
 *    - PID-based load classification
 *    - Shape-memory alloy transformation
 *    - Low-power/idle modes
 *    - Self-diagnostics placeholders
 * 
 *  DISCLAIMER:
 *   - In reality, advanced nuclear batteries, self-repair,
 *     meltdown prevention, or "flagellar" mechanics are complex
 *     and may be subject to heavy regulation or unproven physics.
 *   - This code is purely a conceptual reference. Adapt pinouts,
 *     sensor formulas, driver hardware, and actual power management.
 ************************************************************/

// ======================== CONFIG & INCLUDES ========================

//#define USE_SERVO            // Uncomment if using a servo approach
#define USE_BLDCTYPE          // Comment out if not using a BLDC approach

// Optional advanced modes (multi-winding, magnetic gear, FOC):
//#define ENABLE_MULTI_WINDING
//#define ENABLE_MAGNETIC_GEAR_MODE
//#define ENABLE_FOC_MODE

#ifdef USE_SERVO
  #include <Servo.h>
#endif

#include <math.h>   // For sin() in flagellar oscillation

// ======================== CONSTANTS & MACROS ========================

// Load thresholds
#define SMALL_LOAD_THRESHOLD       5.0f
#define MEDIUM_LOAD_THRESHOLD      50.0f

// Pins for BLDC or Servo
#ifdef USE_BLDCTYPE
  #define MOTOR_PWM_PIN            9
  #define MOTOR_DIR_PIN            8
  #define ENCODER_A_PIN            2
  #define ENCODER_B_PIN            3
#else
  #define SERVO_PIN                9
  Servo motorServo;
#endif

// Shape-memory alloy pin
#define SMA_PIN                    5

// Current sense
#define CURRENT_SENSE_PIN          A1

// Battery sense
#define BATTERY_SENSE_PIN          A0
#define BATTERY_MIN_VOLTAGE        3.0f
#define BATTERY_MAX_VOLTAGE        12.0f

// Temperature sensor
#define TEMP_SENSOR_PIN            A2
#define SAFE_TEMPERATURE_MAX       80.0f

// Motor control intervals
#define MOTOR_SPEED_CHECK_MS       100
#define LOOP_DELAY_MS              10

// Nano-Diamond Battery End-of-Life simulation
#define BATTERY_LIFECYCLE_WARN     0.05f  // 5% left => warning
#define BATTERY_LIFECYCLE_EOFF     0.02f  // 2% => forced hibernation

// Flagellar motor oscillation
#define FLAGELLAR_AMPLITUDE        0.1f   // 10% amplitude of base PWM

// ===================== STRUCTS & ENUMS =====================

enum DeviceType {
  DEVICE_SMALL,
  DEVICE_MEDIUM,
  DEVICE_LARGE,
  DEVICE_UNKNOWN
};

// Advanced motor modes (for multi-winding, FOC, etc.)
enum AdvancedMotorMode {
  MODE_DIRECT_DRIVE,
  MODE_MAGNETIC_GEAR,
  MODE_MULTI_WINDING_LOW,
  MODE_MULTI_WINDING_HIGH,
  MODE_FOC,
  MODE_BASIC_PID
};

struct PIDGains {
  float kp;
  float ki;
  float kd;
};

// Gains for each load classification
PIDGains smallGains  = {2.0f, 0.5f, 0.1f};
PIDGains mediumGains = {3.0f, 0.8f, 0.2f};
PIDGains largeGains  = {5.0f, 1.0f, 0.5f};

// ===================== GLOBAL VARIABLES =====================

// Encoder & speed
volatile long encoderCount = 0;
float currentSpeed = 0.0f;
unsigned long lastSpeedCheckTime = 0;

// PID control variables
float kp = 2.0f, ki = 0.5f, kd = 0.1f;
float pidIntegral = 0.0f;
float previousError = 0.0f;

// Desired speed in RPM
float targetSpeed = 100.0f;

// Device classification & motor mode
DeviceType currentDeviceType = DEVICE_UNKNOWN;
AdvancedMotorMode currentMotorMode = MODE_BASIC_PID;

// Battery life simulation (1.0 => 100% life, 0.0 => depleted)
float batteryLifeRatio = 1.0f;

// States / Flags
bool meltdownPrevented   = false;   // True if we mitigate overheating
bool systemHibernating   = false;   // True if battery is EOL, awaiting recharge

// ===================== FUNCTION PROTOTYPES =====================

void onEncoderARise();
void stopMotor();
void goToLowPowerMode();
void enterHibernationMode();
bool checkExternalRecharge();

void maybeReconfigureMotorMode(DeviceType devType);
float runMotorControl(float target, float actual, AdvancedMotorMode mode);
float runFOCPlaceholder(float target, float actual);
float computePID(float target, float actual);
float runMotorControlWithFlagella(float target, float actual, AdvancedMotorMode mode);

bool shouldTransform(DeviceType devType, float speed);
void activateSMA(bool enable);
bool reduceSpeedDueToOverheat();
void optimizeNuclearUsage();
void performSelfDiagnostics();

float degradeBatteryLife();
DeviceType detectDeviceType(float current);
void applyPIDGains(DeviceType devType);

float readBatteryVoltage();
float readMotorCurrent();
float readMotorTemperature();

// ========================= SETUP =========================

void setup() {
  Serial.begin(115200);

#ifdef USE_BLDCTYPE
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderARise, RISING);

  // Motor off initially
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_DIR_PIN, LOW);
#else
  // Servo approach
  motorServo.attach(SERVO_PIN);
  motorServo.writeMicroseconds(1500); // neutral
#endif

  pinMode(SMA_PIN, OUTPUT);
  digitalWrite(SMA_PIN, LOW);

  pinMode(CURRENT_SENSE_PIN, INPUT);
  pinMode(BATTERY_SENSE_PIN, INPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);

  Serial.println("System Initialization Complete (Combined Edition).");
}

// ========================= MAIN LOOP =========================

void loop() {
  // If the system is hibernating, check for external recharge
  if (systemHibernating) {
    if (checkExternalRecharge()) {
      // Reset battery life to full or a certain replenished value
      batteryLifeRatio = 1.0f;
      systemHibernating = false;
      Serial.println("Battery replaced/recharged. Exiting hibernation mode!");
    } else {
      // Stay in hibernation
      delay(1000);
      return;
    }
  }

  // 0. Self-diagnostics
  performSelfDiagnostics();

  // 1. Read battery voltage
  float batteryVoltage = readBatteryVoltage();
  if (batteryVoltage < BATTERY_MIN_VOLTAGE) {
    Serial.println("Voltage below safe threshold. Stopping motor...");
    stopMotor();
    goToLowPowerMode();
    return;
  }

  // Degrade battery life ratio conceptually
  degradeBatteryLife();

  // Check end-of-life conditions
  if (batteryLifeRatio < BATTERY_LIFECYCLE_EOFF) {
    // Force hibernation
    Serial.println("[CRITICAL] Battery near depletion. Entering hibernation.");
    stopMotor();
    enterHibernationMode();
    return;
  } else if (batteryLifeRatio < BATTERY_LIFECYCLE_WARN) {
    Serial.println("[WARNING] Battery life is very low. Please replace or recharge soon.");
  }

  // 2. Read motor current => classify load
  float motorCurrent = readMotorCurrent();
  DeviceType detectedType = detectDeviceType(motorCurrent);
  if (detectedType != currentDeviceType) {
    currentDeviceType = detectedType;
    applyPIDGains(currentDeviceType);
  }

  // 3. Temperature check => meltdown prevention
  float currentTemp = readMotorTemperature();
  if (currentTemp > SAFE_TEMPERATURE_MAX) {
    Serial.print("Overheat warning (");
    Serial.print(currentTemp);
    Serial.println("°C). Attempting to reduce speed...");
    meltdownPrevented = reduceSpeedDueToOverheat();
    if (!meltdownPrevented) {
      Serial.println("CRITICAL: Could not prevent meltdown. Stopping motor.");
      stopMotor();
      goToLowPowerMode();
      return;
    }
  }

  // 4. Reconfigure advanced motor mode if desired
  maybeReconfigureMotorMode(currentDeviceType);

#ifdef USE_BLDCTYPE
  // 5. Compute current speed from encoder
  unsigned long now = millis();
  if (now - lastSpeedCheckTime >= MOTOR_SPEED_CHECK_MS) {
    noInterrupts();
    long countSnapshot = encoderCount;
    encoderCount = 0;
    interrupts();

    // Basic pulses -> RPM conversion
    float pulsesPerRev = 360.0f; // depends on your encoder
    currentSpeed = (countSnapshot / pulsesPerRev) * 600.0f;
    lastSpeedCheckTime = now;
  }

  // 6. Run motor control with optional flagellar oscillation
  float controlOutput = runMotorControlWithFlagella(targetSpeed, currentSpeed, currentMotorMode);
  int pwmValue = (int)constrain(controlOutput, 0, 255);
  analogWrite(MOTOR_PWM_PIN, pwmValue);

  // Direction: negative => reverse
  digitalWrite(MOTOR_DIR_PIN, (targetSpeed < 0.0f) ? HIGH : LOW);
#else
  // Servo approach
  int servoCmd = map((int)targetSpeed, 0, 200, 1500, 2000);
  servoCmd = constrain(servoCmd, 1000, 2000);
  motorServo.writeMicroseconds(servoCmd);
#endif

  // 7. Shape-memory transformation if needed
  if (shouldTransform(currentDeviceType, currentSpeed)) {
    activateSMA(true);
  } else {
    activateSMA(false);
  }

  // 8. Idle check => if target speed ~0 => low power
  if (fabs(targetSpeed) < 0.1f) {
    Serial.println("Target speed near zero. Entering low-power mode...");
    stopMotor();
    goToLowPowerMode();
  }

  // 9. Attempt nuclear battery usage optimization
  optimizeNuclearUsage();

  delay(LOOP_DELAY_MS);
}

// ========================= INTERRUPTS =========================

/** Quadrature encoder interrupt for BLDC approach. */
void onEncoderARise() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ========================= HELPER FUNCTIONS =========================

/** Combine advanced mode control with a "flagellar" sine-wave overlay. */
float runMotorControlWithFlagella(float target, float actual, AdvancedMotorMode mode) {
  float baseOutput = runMotorControl(target, actual, mode);

  // Flagellar oscillation
  float timeFactor = millis() / 1000.0f; 
  // Sine-wave amplitude is 10% of the baseOutput => "Flagellar" concept
  float oscillation = FLAGELLAR_AMPLITUDE * baseOutput * sin(2.0f * M_PI * timeFactor);

  float newOutput = baseOutput + oscillation;
  // Clamp to [0,255]
  if (newOutput < 0)   newOutput = 0;
  if (newOutput > 255) newOutput = 255;

  return newOutput;
}

/** Main motor control logic with advanced modes (multi-winding, etc.). */
float runMotorControl(float target, float actual, AdvancedMotorMode mode) {
  switch (mode) {
    case MODE_FOC:
      Serial.println("Running FOC control (placeholder)...");
      return runFOCPlaceholder(target, actual);

    case MODE_MULTI_WINDING_LOW:
      // Possibly limit max speed to 80% for higher torque
      return computePID(target * 0.8f, actual);

    case MODE_MULTI_WINDING_HIGH:
      // Increase possible speed by 20%
      return computePID(target * 1.2f, actual);

    case MODE_MAGNETIC_GEAR:
      // Example ratio effect
      Serial.println("Applying Magnetic Gear ratio (placeholder)...");
      return computePID(target * 0.7f, actual);

    case MODE_DIRECT_DRIVE:
      // Straight PID
      return computePID(target, actual);

    case MODE_BASIC_PID:
    default:
      // Original approach
      return computePID(target, actual);
  }
}

/** Placeholder for FOC—would need specialized libraries. */
float runFOCPlaceholder(float target, float actual) {
  return computePID(target, actual);
}

/** Basic PID for speed control. */
float computePID(float target, float actual) {
  float error = target - actual;
  float dt = (MOTOR_SPEED_CHECK_MS / 1000.0f);
  pidIntegral += (error * dt);
  float derivative = (error - previousError) / dt;
  float output = (kp * error) + (ki * pidIntegral) + (kd * derivative);
  previousError = error;

  if (output < 0) output = 0; 
  return output;
}

/** Conditionally transform shape via SMA if load is large & speed high. */
bool shouldTransform(DeviceType devType, float speed) {
  if (devType == DEVICE_LARGE && speed > 80.0f) {
    return true;
  }
  return false;
}

/** Activate shape-memory alloy. */
void activateSMA(bool enable) {
  digitalWrite(SMA_PIN, enable ? HIGH : LOW);
}

/** Stop the motor. */
void stopMotor() {
#ifdef USE_BLDCTYPE
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_DIR_PIN, LOW);
#else
  motorServo.writeMicroseconds(1500); // neutral
#endif
}

/** Enter a low-power mode (placeholder). */
void goToLowPowerMode() {
  Serial.println("Entering low-power mode (placeholder)...");
  delay(2000);
}

/** If temperature is too high, reduce speed by 50%. Return false if we can't salvage. */
bool reduceSpeedDueToOverheat() {
  targetSpeed *= 0.5f;
  if (targetSpeed < 10.0f) {
    return false;
  }
  return true;
}

/** Attempt to optimize nuclear battery usage (conceptual). */
void optimizeNuclearUsage() {
  Serial.println("Optimizing nuclear battery usage (conceptual)...");
}

/** Perform self-diagnostics (placeholder). */
void performSelfDiagnostics() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck > 1000) {
    lastCheck = now;
    // Could check sensor statuses, logs, error flags, etc.
  }
}

/** Possibly reconfigure motor mode if advanced features are enabled. */
void maybeReconfigureMotorMode(DeviceType devType) {
  // Example logic for multi-winding or magnetic gear
  // #ifdef ENABLE_MULTI_WINDING
  // if (devType == DEVICE_LARGE) currentMotorMode = MODE_MULTI_WINDING_LOW;
  // else currentMotorMode = MODE_MULTI_WINDING_HIGH;
  // #endif
  // #ifdef ENABLE_MAGNETIC_GEAR_MODE
  // currentMotorMode = MODE_MAGNETIC_GEAR;
  // #endif
  // #ifdef ENABLE_FOC_MODE
  // currentMotorMode = MODE_FOC;
  // #endif
  // Default
  currentMotorMode = MODE_BASIC_PID;
}

/** Checks if an external recharge or battery replacement has occurred. */
bool checkExternalRecharge() {
  // In real hardware, you'd sense external power or a new battery module.
  // For demonstration, randomly simulate a 10% chance every 5s
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck > 5000) {
    lastCheck = now;
    int chance = random(0, 100);
    if (chance < 10) {
      Serial.println("External source detected! Battery can be replenished.");
      return true;
    }
  }
  return false;
}

/** Enter hibernation if battery is nearly depleted. */
void enterHibernationMode() {
  systemHibernating = true;
  Serial.println("System is now in HIBERNATION, awaiting battery replacement or external power...");
}

/** Degrade battery life ratio (purely for demonstration). */
float degradeBatteryLife() {
  // E.g., degrade by 0.0005 each loop iteration
  batteryLifeRatio -= 0.0005f;
  if (batteryLifeRatio < 0.0f) batteryLifeRatio = 0.0f;
  return batteryLifeRatio;
}

/** Classify the motor load based on current. */
DeviceType detectDeviceType(float current) {
  if (current < SMALL_LOAD_THRESHOLD) {
    return DEVICE_SMALL;
  } else if (current < MEDIUM_LOAD_THRESHOLD) {
    return DEVICE_MEDIUM;
  } else {
    return DEVICE_LARGE;
  }
}

/** Apply different PID gains based on load type. */
void applyPIDGains(DeviceType devType) {
  switch (devType) {
    case DEVICE_SMALL:
      kp = smallGains.kp; ki = smallGains.ki; kd = smallGains.kd;
      Serial.println("Applying SMALL device PID gains.");
      break;
    case DEVICE_MEDIUM:
      kp = mediumGains.kp; ki = mediumGains.ki; kd = mediumGains.kd;
      Serial.println("Applying MEDIUM device PID gains.");
      break;
    case DEVICE_LARGE:
      kp = largeGains.kp; ki = largeGains.ki; kd = largeGains.kd;
      Serial.println("Applying LARGE device PID gains.");
      break;
    default:
      kp = 2.0f; ki = 0.5f; kd = 0.1f;
      Serial.println("Unknown device type; using fallback PID gains.");
      break;
  }
}

/** Reads battery voltage from the buffer. */
float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_SENSE_PIN);
  float measuredVoltage = (adcValue / 1023.0f) * 5.0f;
  // 4:1 divider assumption
  return measuredVoltage * 4.0f;
}

/** Reads motor current from a sensor (e.g., ACS712). */
float readMotorCurrent() {
  int sensorValue = analogRead(CURRENT_SENSE_PIN);
  float sensorVoltage = (sensorValue / 1023.0f) * 5.0f;
  float offsetVoltage = 2.5f; 
  float sensitivity = 0.185f; // for ±5A module
  float amps = (sensorVoltage - offsetVoltage) / sensitivity;
  return amps;
}

/** Reads motor temperature (placeholder formula). */
float readMotorTemperature() {
  int rawValue = analogRead(TEMP_SENSOR_PIN);
  float voltage = (rawValue / 1023.0f) * 5.0f;
  // Suppose 10mV/°C => voltage*100 => °C
  return voltage * 100.0f;
}

~~~

## License

Copyright (c) 2025 xatusbetazx17

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

1. The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
2. Actual implementation in hardware (especially involving nuclear or high-power technology) must comply with all **relevant safety, regulatory, and certification requirements**.

**THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

You are **free to use, modify, and distribute** this code. **Contributions** are welcome—feel free to fork this repository, submit pull requests, or open issues. Have fun, build amazing projects, and always **stay safe**!


