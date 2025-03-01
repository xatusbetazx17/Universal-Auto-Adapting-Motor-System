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
```cpp
/************************************************************
 *  Universal Auto-Adapting Motor System (Conceptual Example)
 *  ---------------------------------------------------------
 *  This is a comprehensive Arduino-style sketch illustrating:
 *    - Power monitoring (battery w/ nuclear nano-diamond source)
 *    - Adaptive load classification (small/medium/large)
 *    - PID-based speed control (placeholder)
 *    - Shape-memory "transformation" toggle
 *    - Low-power or idle mode
 * 
 *  NOTE: You MUST adapt to real hardware, libraries, pinouts, etc.
 ************************************************************/

// ========== CONFIG & INCLUDES ==========
//#define USE_SERVO  // Uncomment if using a standard/continuous servo
#define USE_BLDCTYPE  // Comment if using a servo

#ifdef USE_SERVO
  #include <Servo.h>
#endif

// Load thresholds (in Amps)
#define SMALL_LOAD_THRESHOLD     5.0f
#define MEDIUM_LOAD_THRESHOLD    50.0f

// Example pins
#ifdef USE_BLDCTYPE
  #define MOTOR_PWM_PIN   9
  #define MOTOR_DIR_PIN   8
  #define ENCODER_A_PIN   2
  #define ENCODER_B_PIN   3
#else
  // Servo pin
  #define SERVO_PIN       9
  Servo motorServo;
#endif

// Shape-memory alloy
#define SMA_PIN           5

// Current sense
#define CURRENT_SENSE_PIN A1

// Battery sense
#define BATTERY_SENSE_PIN A0
#define BATTERY_MIN_VOLTAGE 3.0f
#define BATTERY_MAX_VOLTAGE 12.0f

// Timing
#define MOTOR_SPEED_CHECK_MS 100
#define LOOP_DELAY_MS        10

// ========== STRUCTS & ENUMS ==========
enum DeviceType {
  DEVICE_SMALL,
  DEVICE_MEDIUM,
  DEVICE_LARGE,
  DEVICE_UNKNOWN
};

struct PIDGains {
  float kp;
  float ki;
  float kd;
};

// Example gains for each load classification
PIDGains smallGains  = {2.0f, 0.5f, 0.1f};
PIDGains mediumGains = {3.0f, 0.8f, 0.2f};
PIDGains largeGains  = {5.0f, 1.0f, 0.5f};

// ========== GLOBAL CONTROL VARIABLES ==========
volatile long encoderCount = 0;
float currentSpeed = 0.0f;
unsigned long lastSpeedCheckTime = 0;

// PID dynamic variables
float kp = 2.0f, ki = 0.5f, kd = 0.1f;
float pidIntegral = 0.0f;
float previousError = 0.0f;

// Desired speed in RPM
float targetSpeed = 100.0f;  

DeviceType currentDeviceType = DEVICE_UNKNOWN;

// ========== SETUP ==========

void setup() {
  Serial.begin(115200);

#ifdef USE_BLDCTYPE
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderARise, RISING);

  // Motor off initially
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_DIR_PIN, LOW);

#else
  // Servo
  motorServo.attach(SERVO_PIN);
  motorServo.writeMicroseconds(1500); // neutral for continuous servo
#endif

  pinMode(SMA_PIN, OUTPUT);
  digitalWrite(SMA_PIN, LOW);

  pinMode(CURRENT_SENSE_PIN, INPUT);
  pinMode(BATTERY_SENSE_PIN, INPUT);

  Serial.println("System Initialization Complete.");
}

// ========== MAIN LOOP ==========

void loop() {
  // 1. Check battery or buffer voltage
  float batteryVoltage = readBatteryVoltage();
  if (batteryVoltage < BATTERY_MIN_VOLTAGE) {
    Serial.println("Voltage below safe threshold. Stopping motor...");
    stopMotor();
    goToLowPowerMode();
    return;
  }

  // 2. Read current usage (to guess load scale)
  float motorCurrent = readMotorCurrent();

  // 3. Classify device
  DeviceType detectedType = detectDeviceType(motorCurrent);
  if (detectedType != currentDeviceType) {
    currentDeviceType = detectedType;
    applyPIDGains(currentDeviceType);
  }

#ifdef USE_BLDCTYPE
  // 4. Compute current speed
  unsigned long now = millis();
  if (now - lastSpeedCheckTime >= MOTOR_SPEED_CHECK_MS) {
    noInterrupts();
    long countSnapshot = encoderCount;
    encoderCount = 0;
    interrupts();

    float pulsesPerRev = 360.0f; // depends on your motor's encoder
    currentSpeed = (countSnapshot / pulsesPerRev) * 600.0f;
    lastSpeedCheckTime = now;
  }

  // 5. PID control
  float outputPWM = computePID(targetSpeed, currentSpeed);
  int pwmValue = (int)constrain(outputPWM, 0, 255);
  analogWrite(MOTOR_PWM_PIN, pwmValue);

  digitalWrite(MOTOR_DIR_PIN, (targetSpeed < 0.0f) ? HIGH : LOW);

#else
  // Servo approach
  // Example: map targetSpeed from [0..200] RPM to [1500..2000] microseconds
  int servoCmd = map((int)targetSpeed, 0, 200, 1500, 2000);
  servoCmd = constrain(servoCmd, 1000, 2000);
  motorServo.writeMicroseconds(servoCmd);
#endif

  // 6. SMA "transformation"
  if (shouldTransform(currentDeviceType, currentSpeed)) {
    activateSMA(true);
  } else {
    activateSMA(false);
  }

  // 7. Optional idle check
  if (fabs(targetSpeed) < 0.1f) {
    Serial.println("No speed requested. Entering low power mode...");
    stopMotor();
    goToLowPowerMode();
  }

  delay(LOOP_DELAY_MS);
}

// ========== INTERRUPTS ==========

// For a BLDC encoder
void onEncoderARise() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ========== HELPER FUNCTIONS ==========

float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_SENSE_PIN);
  float measuredVoltage = (adcValue / 1023.0f) * 5.0f;
  // Assume 4:1 divider
  return measuredVoltage * 4.0f;
}

float readMotorCurrent() {
  // Example for ACS712 (5A version):
  int sensorValue = analogRead(CURRENT_SENSE_PIN);
  float sensorVoltage = (sensorValue / 1023.0f) * 5.0f;
  float offsetVoltage = 2.5f;
  float sensitivity = 0.185f;  // V per A
  float amps = (sensorVoltage - offsetVoltage) / sensitivity;
  return amps;
}

DeviceType detectDeviceType(float current) {
  if (current < SMALL_LOAD_THRESHOLD) {
    return DEVICE_SMALL;
  } else if (current < MEDIUM_LOAD_THRESHOLD) {
    return DEVICE_MEDIUM;
  } else {
    return DEVICE_LARGE;
  }
}

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
      Serial.println("Unknown device type, fallback PID gains.");
      break;
  }
}

// Basic PID
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

// Optional shape transformation logic
bool shouldTransform(DeviceType devType, float speed) {
  // Example condition: large device type + speed > 80
  if (devType == DEVICE_LARGE && speed > 80.0f) {
    return true;
  }
  return false;
}

void activateSMA(bool enable) {
  if (enable) {
    digitalWrite(SMA_PIN, HIGH);
    // In real hardware, watch for overheating or time-limiting
  } else {
    digitalWrite(SMA_PIN, LOW);
  }
}

void stopMotor() {
#ifdef USE_BLDCTYPE
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_DIR_PIN, LOW);
#else
  motorServo.writeMicroseconds(1500); // neutral
#endif
}

void goToLowPowerMode() {
  // Placeholder for real low-power or sleep implementation
  Serial.println("Entering low-power mode (placeholder)...");
  delay(3000);
}
~~~
