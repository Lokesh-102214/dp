/*   hand_e_kraft_5servo_esp32_bt.ino

  ESP32-based tremor-suppression glove controller:
   - 5 servos (using ESP32Servo library)
   - MPU6050 IMU for tremor detection (FFT Frequency + Power Threshold)
   - Classic Bluetooth Serial (BluetoothSerial) for app control & telemetry
   - Modes: automatic tremor suppression (AUTO) and manual app control (MANUAL)
*/
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"
#include <ArduinoFFT.h> //FFT Library

// -------------------- FFT CONFIG --------------------
#define SAMPLES 64             // Must be a power of 2
#define SAMPLING_FREQUENCY 50.0  // Hz (20ms delay between samples)

// Structure to hold the result of the FFT analysis
struct TremorAnalysis {
  double frequency = 0.0; // Dominant frequency found by FFT (Hz)
  double power = 0.0;     // Magnitude/Power of that dominant frequency peak
};

// FFT buffers for Gyro X, Y, and Z
double gxReal[SAMPLES], gxImag[SAMPLES];
double gyReal[SAMPLES], gyImag[SAMPLES];
double gzReal[SAMPLES], gzImag[SAMPLES];
// FFT object is initialized with the arrays
ArduinoFFT<double> FFT = ArduinoFFT<double>(gxReal, gxImag, SAMPLES, SAMPLING_FREQUENCY);

// -------------------- HARDWARE / PIN CONFIG --------------------
Adafruit_MPU6050 mpu;
BluetoothSerial BT; // Classic Bluetooth serial

const int SERVO_PINS[5] = {5, 18, 19, 23, 13}; // GPIOs connected to each servo signal
Servo SERVO_CHANNELS[5];// Servo objects
const int LED_PIN = 2;// LED to indicate tremor active / mode

// Servo mechanical positions (0..180). Tune these per your mechanical build.
int restAngles[5] = {0, 0, 0, 0, 0};// neutral positions for the 5 servos
int pinchAngles[5] = {60, 120, 70, 110, 80};// positions used when stabilizing

// -------------------- SERVO DIRECTION CONFIG --------------------
// true = inverted direction (180° = rest), false = normal direction
bool servoInverted[5] = { false, true, false, true, true };  // 1-based servos 2,4,5 are inverted

// Safety limits for servo motion (constrain targets)
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

// -------------------- TREMOR DETECTION PARAMETERS (MODIFIED) --------------------

// Frequency Factor: The range of pathological human tremor (3-12 Hz)
const float TREMOR_FREQ_LOW = 3.0f;// Lower frequency bound (Hz)
const float TREMOR_FREQ_HIGH = 10.0f;// Upper frequency bound (Hz)

// Power Threshold: The magnitude (amplitude) required to overcome sensor noise.
// **TUNE THIS VALUE** by checking the 'power' when the MPU6050 is sitting still.
float POWER_THRESHOLD = 1.0f;// Noise filter threshold (Arbitrary start value)

unsigned long tremorHoldTime = 2000;// keep servos active for 2000 ms after tremor stops
unsigned long lastTremorTime = 0;

// Current measured tremor metrics
float currentTremorFrequency = 0.0f;
float currentTremorPower = 0.0f;

// -------------------- MODES & FLAGS --------------------
bool autoMode = true;// if true, automatic tremor suppression is enabled
bool manualGrip = false;// when user commands a manual grip via BT
bool servosActive = false;// current active state (engaged or relaxed)
bool gripActive = false;

// -------------------- SMOOTH SERVO CONTROL --------------------
int currentAngle[5] = {0, 0, 0, 0, 0};
int targetAngle[5]  = {0, 0, 0, 0, 0};
unsigned long lastMoveTime = 0;
unsigned long moveInterval = 10; // smaller = faster motion

// -------------------- FUNCTION PROTOTYPES --------------------
void setServoAngle(int channel, int angle);
void smoothMoveServos();
void gripOscillate();
int applyServoDirection(int idx, int angle);
void activateAllServos(bool active, float tremorLevel);
TremorAnalysis computeTremorAnalysis();//FFT analysis returns struct
void calibrateGyroBaseline();// Calibrates power threshold
void handleBTCommands();// read and process BT commands
void sendStatusTelemetry();

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\n=== Hand-e-Kraft ESP32 (FFT Tremor Detection) ===");

  // Initialize I2C (SDA,SCL defaults: 21,22)
  Wire.begin(21, 22);

  // Initialize Bluetooth (device name: HandEKraft)
  if (!BT.begin("HandEKraft")) {
    Serial.println("BT start failed.");
  } else {
    Serial.println("Bluetooth started as 'HandEKraft'.");
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(" MPU6050 not found. Check wiring!");
    while (1) { delay(1000); } // stop here
  }
  Serial.println("✅ MPU6050 initialized.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Setup servos and move to rest angles
  for (int i = 0; i < 5; ++i) {
    SERVO_CHANNELS[i].attach(SERVO_PINS[i]);   // attach pin to servo object
    int startAngle = applyServoDirection(i, restAngles[i]);   //  use inversion
    SERVO_CHANNELS[i].write(startAngle); 
    currentAngle[i] = startAngle;
    targetAngle[i] = startAngle;   // move servo to rest position
  }
  delay(500); // let servos settle

  // Calibrate gyro baseline to determine a realistic power threshold
  calibrateGyroBaseline();

  Serial.println("Setup complete. Starting main loop.\n");
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // 1) Handle incoming Bluetooth commands (non-blocking)
  handleBTCommands();

  // Your Bluetooth read, tremor detection code here

  if (gripActive) {
      gripOscillate(); // Oscillate non-stop
  }

  // 3) Decide activation based on mode
  if (autoMode) {
    TremorAnalysis analysis = computeTremorAnalysis();
    currentTremorFrequency = analysis.frequency;
    currentTremorPower = analysis.power;
    if(currentTremorFrequency>10){
      currentTremorFrequency=10;
    }

    // CONDITION 1: Check if frequency is in the pathological band
    bool freqInRange = (currentTremorFrequency >= TREMOR_FREQ_LOW && currentTremorFrequency <= TREMOR_FREQ_HIGH);

    // CONDITION 2: Check if the movement power is above the noise floor
    bool powerHighEnough = (currentTremorPower > POWER_THRESHOLD);

    // Tremor is detected only if BOTH frequency AND power conditions are met
    if (freqInRange && powerHighEnough) {
      lastTremorTime = millis();
      activateAllServos(true, currentTremorPower);
      servosActive = true;
      Serial.println("🟢 TREMOR DETECTED (Freq & Power OK) → Servo suppression ON");
    } else if (millis() - lastTremorTime < tremorHoldTime) {
      // Hysteresis: hold for tremorHoldTime after last detection
      activateAllServos(true, currentTremorPower);
      servosActive = true;
      Serial.println("🟡 HOLDING (Post-Tremor Hysteresis) → Servo suppression ON");
    } else {
      // stable or movement is not a tremor (e.g. slow voluntary movement or noise)
      activateAllServos(false, 0.0f);
      servosActive = false;
      Serial.println("⚫ STABLE (No Tremor Detected) → Servo relaxation");
    }
  } else {
    // manual mode
    smoothMoveServos();//  Smoothly move servos toward targetAngle
    servosActive = true;
  }

  // 5) Send telemetry occasionally over Bluetooth/Serial
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry > 250) { // every 250 ms
    sendStatusTelemetry();
    lastTelemetry = millis();
  }

  // The loop delay is implicitly handled by the FFT sampling inside computeTremorAnalysis()
}

// -------------------- SMOOTH SERVO MOTION --------------------

void gripOscillate() {
  static bool goingToMax = true;

  if (goingToMax) {
    for (int s = 0; s < 5; s++) {
      int target = applyServoDirection(s, 180);
      SERVO_CHANNELS[s].write(target); // Go to 180
      currentAngle[s] = target;
    }
    delay(500);
    goingToMax = false;
  } else {
    for (int s = 0; s < 5; s++) {
      int target = applyServoDirection(s,restAngles[s]);
      SERVO_CHANNELS[s].write(target); // Come back to rest
      currentAngle[s] = target;
    }
    goingToMax = true;
  }
  delay(500);
}

void smoothMoveServos() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMoveTime >= moveInterval) {
    lastMoveTime = currentMillis;
    for (int i = 0; i < 5; i++) {
      if (currentAngle[i] < targetAngle[i]) {
        currentAngle[i]++;
        SERVO_CHANNELS[i].write(currentAngle[i]);
      } 
      else if (currentAngle[i] > targetAngle[i]) {
        currentAngle[i]--;
        SERVO_CHANNELS[i].write(currentAngle[i]);
      }
    }
  }
}
// -------------------- FFT TREMOR DETECTION (NEW FUNCTION) --------------------
TremorAnalysis computeTremorAnalysis() {
  unsigned long startSampling = millis();

  // 1. Data Acquisition
  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    // Use RAW data (no abs) for correct frequency detection
    gxReal[i] = g.gyro.x; 
    gyReal[i] = g.gyro.y;
    gzReal[i] = g.gyro.z;
    gxImag[i] = gyImag[i] = gzImag[i] = 0.0;
    // Wait for the required sampling period
    delay((1000.0 / SAMPLING_FREQUENCY));
  }

  // Track the overall max power and its corresponding frequency across all axes
  double maxFreq = 0.0;
  double maxPower = 0.0;

  // Function to process one axis (reusable with different arrays)
  auto processAxis = [&](double realArr[], double imagArr[]) {
    FFT.windowing(realArr, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(realArr, imagArr, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(realArr, imagArr, SAMPLES);
    double freq = FFT.majorPeak(realArr, SAMPLES, SAMPLING_FREQUENCY);

    // Find the magnitude (power) at the major peak index
    // Note: The magnitude array is stored back in the realArr
    int peakIndex = constrain((int)round(freq * SAMPLES / SAMPLING_FREQUENCY), 0, SAMPLES-1);
    double power = realArr[peakIndex];

    if (power > maxPower) { 
      maxPower = power; 
      maxFreq = freq; 
    }
    return std::pair<double, double>(freq, power);
  };

  // 2. Process all three axes and find the overall dominant peak
  std::pair<double, double> analysisX = processAxis(gxReal, gxImag);
  std::pair<double, double> analysisY = processAxis(gyReal, gyImag);
  std::pair<double, double> analysisZ = processAxis(gzReal, gzImag);

  Serial.printf("FFT Pks — X:%.2f(P:%.2f) Y:%.2f(P:%.2f) Z:%.2f(P:%.2f) | Dom: %.2f Hz, P: %.2f\n", 
              analysisX.first, analysisX.second, 
              analysisY.first, analysisY.second, 
              analysisZ.first, analysisZ.second, 
              maxFreq, maxPower);

  return {maxFreq, maxPower};
}


int applyServoDirection(int idx, int angle) {
  if (servoInverted[idx]) {
    return 180 - angle;     // flip the angle for inverted servos
  } else {
    return angle;
  }
}

// -------------------- SUPPORT FUNCTIONS --------------------

void setServoAngle(int channel, int angle) {
  int adj = applyServoDirection(channel, angle);
  targetAngle[channel] = constrain(adj, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

// Activate or relax all servos. Adds a mild oscillation when active for natural motion.
void activateAllServos(bool active, float tremorLevel) {
  if (active) {
    // The servo oscillation amplitude scales with tremorLevel (clamped)
    // The tremorLevel is now the FFT Power (amplitude)
    float amp = 3.0f + min(tremorLevel, 10.0f) * 0.7f; // degrees of oscillation
    float osc = amp * sin(millis() / 100.0f); // smooth sinusoidal oscillation

    for (int i = 0; i < 5; ++i) {
      // compute target = pinchAngles[i] plus small oscillation
      int target = (int)round(pinchAngles[i] + osc);
      target = constrain(target, 60, 150); // tighter safety bounds while active
      target = applyServoDirection(i, target);   // apply inversion
      SERVO_CHANNELS[i].write(target);
      currentAngle[i] = target;
      targetAngle[i] = target;
    }
  } else if(autoMode) {
    // relax to rest angles
    for (int i = 0; i < 5; ++i) {
      int target = applyServoDirection(i, restAngles[i]);
      SERVO_CHANNELS[i].write(target);
      targetAngle[i] = target;
    }
  }
}
// Calibrates gyro baseline noise and sets POWER_THRESHOLD accordingly
void calibrateGyroBaseline() {
  Serial.println("Calibrating gyro baseline... Keep the device steady for 2 seconds.");
  unsigned long start = millis();
  int samples = 0;
  double sum_of_power = 0;
  // Collect a series of FFT readings to determine the average noise power
  for (int i=0; i < 10; ++i) {
      TremorAnalysis analysis = computeTremorAnalysis();
      sum_of_power += analysis.power;
      samples++;
  }

  float baseline_power = (samples > 0) ? (float)(sum_of_power / samples) : 0.0f;

  // Set threshold > baseline. Use multiplier (3.0f) to avoid false positives from noise.
  // We ensure a minimum power threshold is maintained.
  POWER_THRESHOLD = max(1.0f, baseline_power ); 

  Serial.print("Calibration done. Baseline Power=");
  Serial.print(baseline_power, 3);
  Serial.print("  -> POWER_THRESHOLD=");
  Serial.println(POWER_THRESHOLD, 3);
}

// Read and process Bluetooth commands (non-blocking)
void handleBTCommands() {
  if (!BT.hasClient()) return; // no connected BT client

  // Read a line if available
  if (BT.available()) {
    String cmd = BT.readStringUntil('\n');
    cmd.trim(); // remove whitespace
    cmd.toUpperCase(); // convert to uppercase

    Serial.print("BT cmd: ");
    Serial.println(cmd);

    if (cmd == "GRIP") {
      gripActive =true;
      manualGrip = true;       // manual grip engaged
      autoMode = false;        // switch to manual mode
      BT.println("OK:GRIP");
    } else if (cmd == "RELEASE") {
      manualGrip = false;
      autoMode = false;
      gripActive = false;
      for (int i = 0; i < 5; i++) {
        int target=applyServoDirection(i, restAngles[i]);
        targetAngle[i] = target;
      }
      BT.println("OK:RELEASE");
    } else if (cmd == "AUTO_ON") {
      autoMode = true;
      manualGrip = false;
      gripActive = false;
      BT.println("OK:AUTO_ON");
    } else if (cmd == "AUTO_OFF") {
      autoMode = false;
      for (int i = 0; i < 5; i++) {
        int target=applyServoDirection(i, restAngles[i]);
        targetAngle[i] = target;
      }
      BT.println("OK:AUTO_OFF");
    } else if (cmd == "STATUS") {
      String s="";
      s += "TREMOR:";
      s += String(currentTremorFrequency, 3);
      s += ",TREMOR_POWER:";
      s += String(currentTremorPower, 3); // Send current Freq     // Send current Power
      s += ",THR=";
      s += String(POWER_THRESHOLD, 3); 
      s+= "MODE:";
      s += (autoMode ? "AUTO" : "MANUAL");
      s += ",SERVOS:";
      s += (servosActive ? "ENGAGED" : "RELAXED");
             // Send Power Threshold
      BT.println(s);
    } else if (cmd.startsWith("SERVO ")) {
      int firstSpace = cmd.indexOf(' ');
      int secondSpace = cmd.indexOf(' ', firstSpace + 1);
      if (secondSpace > 0) {
        int idx = cmd.substring(firstSpace + 1, secondSpace).toInt();
        int angle = cmd.substring(secondSpace + 1).toInt();
        if (idx >= 1 && idx <= 5 && angle >= 0 && angle <= 180) {
          autoMode = false; // switch to manual
          manualGrip = false;
          for (int i = 0; i < 5; i++) {
            targetAngle[i] = currentAngle[i];
          }
          setServoAngle(idx - 1, angle); 
          BT.printf("OK:SERVO %d %d\n", idx, angle);
        } else {
          BT.println("ERR:SERVO_RANGE");
        }
      } else {
        BT.println("ERR:SERVO_CMD");
      }
    } else {
      BT.println("ERR:UNKNOWN");
    }
  }
}

// Send telemetry (tremor value & mode) over Bluetooth and Serial
void sendStatusTelemetry() {
  // Construct telemetry string
  String tele="";
  tele += "TREMOR:";
  tele += String(currentTremorFrequency, 3);
  tele += ",TREMOR_POWER:";
  tele += String(currentTremorPower, 3);
  tele += ",MODE:";
  tele += (autoMode ? "AUTO" : "MANUAL");
  tele += ",SERVOS:";
  tele += (servosActive ? "ENGAGED" : "RELAXED");
  tele += ",THR=";
  tele += String(POWER_THRESHOLD, 3);

  Serial.println(tele);     // local debug
  if (BT.hasClient()) {
    BT.println(tele);       // send to connected BT client
  }
}
