/*
 * ESP32-S3 TMC2209 Web Server Controller
 * 
 * Mobile-friendly web interface with circular button layout
 * ESP32-S3 creates its own WiFi Access Point
 * 
 * Connect to WiFi network: "ESP32-MotorController"
 * Password: "motor12345"
 * Open browser and go to: http://192.168.4.1
 * 
 * Hardware Connections (ESP32-S3 GPIO mapping):
 * Motor 1: STEP=GPIO 2,  DIR=GPIO 4,  EN=GPIO 5
 * Motor 2: STEP=GPIO 12, DIR=GPIO 13, EN=GPIO 14
 * Motor 3: STEP=GPIO 16, DIR=GPIO 17, EN=GPIO 18
 * Motor 4: STEP=GPIO 38, DIR=GPIO 39, EN=GPIO 40
 * 
 * PCA9685 to ESP32-S3:
 *   VCC -> 3.3V
 *   GND -> GND
 *   SDA -> GPIO 8
 *   SCL -> GPIO 9
 *   V+  -> 5V (external power for servos recommended)
 * 
 * Servos on PCA9685 channels 0-3 (S1-S4) and channel 8 (S5)
 * 
 * 0.96" OLED Display (SSD1306) to ESP32-S3:
 *   VCC -> 3.3V
 *   GND -> GND
 *   SDA -> GPIO 8  (shared with PCA9685)
 *   SCL -> GPIO 9  (shared with PCA9685)
 * 
 * Ultrasonic Sensor 1 - Front (HC-SR04) to ESP32-S3:
 *   VCC  -> 5V
 *   GND  -> GND
 *   TRIG -> GPIO 6
 *   ECHO -> GPIO 7
 * 
 * Ultrasonic Sensor 2 - Left (HC-SR04) to ESP32-S3:
 *   VCC  -> 5V
 *   GND  -> GND
 *   TRIG -> GPIO 15
 *   ECHO -> GPIO 10
 * 
 * Ultrasonic Sensor 3 - Right (HC-SR04) via PCF8574T:
 *   VCC  -> 5V
 *   GND  -> GND
 *   TRIG -> PCF8574T P0
 *   ECHO -> ESP32-S3 GPIO 11
 * 
 * Ultrasonic Sensor 4 - Back (HC-SR04) via PCF8574T:
 *   VCC  -> 5V
 *   GND  -> GND
 *   TRIG -> PCF8574T P2
 *   ECHO -> ESP32-S3 GPIO 21
 * 
 * PCF8574T I2C Expander to ESP32-S3:
 *   VCC -> 3.3V
 *   GND -> GND
 *   SDA -> GPIO 8  (shared I2C)
 *   SCL -> GPIO 9  (shared I2C)
 *   A0, A1, A2 -> GND (address 0x20)
 * 
 * TL-W5MC1 Inductive Proximity Sensor via PCF8574T:
 *   Brown  -> 5V (VCC)
 *   Blue   -> GND
 *   Black  -> PCF8574T P1 (signal, NPN NO - LOW when metal detected)
 *
 * Limit Switch 1 via PCF8574T:
 *   Terminal 1 -> PCF8574T P3
 *   Terminal 2 -> GND
 *   (NO switch: LOW when pressed, HIGH when open via internal pull-up)
 *
 * Limit Switch 2 via PCF8574T:
 *   Terminal 1 -> PCF8574T P4
 *   Terminal 2 -> GND
 *   (NO switch: LOW when pressed, HIGH when open via internal pull-up)
 *
 * Limit Switch 3 via PCF8574T:
 *   Terminal 1 -> PCF8574T P5
 *   Terminal 2 -> GND
 *   (NO switch: LOW when pressed, HIGH when open via internal pull-up)
 *
 * Capacitive Touch Sensor (TTP223) via PCF8574T:
 *   VCC -> 3.3V
 *   GND -> GND
 *   SIG -> PCF8574T P6 (HIGH when touched)
 *
 * Yahboom 8-Channel Line Sensor to ESP32-S3:
 *   VCC -> 5V
 *   GND -> GND
 *   SDA -> GPIO 8  (shared I2C bus)
 *   SCL -> GPIO 9  (shared I2C bus)
 *   I2C Address: 0x12
 * 
 * Grow GM805 QR/Barcode Scanner to ESP32-S3:
 *   VCC -> 5V
 *   GND -> GND
 *   TX  -> GPIO 41 (ESP32-S3 RX)
 *   RX  -> GPIO 42 (ESP32-S3 TX) - optional, not needed for reading
 *   Baud Rate: 9600
 *
 * Note: ESP32-S3 GPIO 19/20 reserved for USB, GPIO 26-37 reserved
 * for SPI flash/PSRAM on WROOM-1 modules.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_timer.h>

// Access Point credentials - CHANGE THESE IF NEEDED
const char* ap_ssid = "ESP32-MotorController";
const char* ap_password = "motor12345";  // Min 8 characters

WebServer server(80);

// Motor pin definitions
const int STEP_PINS[4] = {2, 12, 16, 38};
const int DIR_PINS[4] = {4, 13, 17, 39};
const int EN_PINS[4] = {5, 14, 18, 40};

// PCA9685 servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define PCA9685_OE_PIN 47  // GPIO to PCA9685 OE pin (HIGH=disabled, LOW=enabled)

// OLED Display (0.96" SSD1306 128x64)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Ultrasonic sensor pins
#define TRIG_PIN_FRONT 6
#define ECHO_PIN_FRONT 7
#define TRIG_PIN_LEFT 15
#define ECHO_PIN_LEFT 10
// Right sensor uses PCF8574T for TRIG
#define ECHO_PIN_RIGHT 11
// Back sensor uses PCF8574T for TRIG
#define ECHO_PIN_BACK 21
#define SOUND_SPEED 0.034  // cm/us
#define MAX_DISTANCE 400   // cm

// PCF8574T I2C expander
#define PCF8574_ADDR 0x20  // A0=A1=A2=GND
#define PCF_TRIG_RIGHT 0   // P0 pin for right sensor
#define PCF_INDUCTIVE 1    // P1 pin for inductive sensor
#define PCF_TRIG_BACK 2    // P2 pin for back sensor
#define PCF_LIMIT_SW1 3    // P3 pin for limit switch 1
#define PCF_LIMIT_SW2 4    // P4 pin for limit switch 2
#define PCF_LIMIT_SW3 5    // P5 pin for limit switch 3
#define PCF_CAP_TOUCH 6    // P6 pin for capacitive touch sensor

// Ultrasonic sensor variables
float distanceFront = 0;
float distanceLeft = 0;
float distanceRight = 0;
float distanceBack = 0;
long frontPulseDurationUs = 0;  // Raw pulse width for front ultrasonic (0 = timeout/no echo)
unsigned long lastDistanceRead = 0;
const unsigned long DISTANCE_READ_INTERVAL = 100;  // Read every 100ms

// Inductive sensor
bool metalDetected = false;

// Limit switches (NO via PCF8574T - true = pressed/triggered)
bool limitSwitch1 = false;
bool limitSwitch2 = false;
bool limitSwitch3 = false;

// Capacitive touch sensor (via PCF8574T - true = touched)
bool capTouched = false;

// Yahboom 8-channel line sensor
#define LINE_SENSOR_ADDR 0x12  // Yahboom line sensor I2C address
#define INVERT_LINE_SENSOR true  // Sensor outputs 0 for black, so invert
uint8_t lineSensorData = 0;  // 8 bits for 8 channels
uint8_t lastPrintedSensorData = 255;  // Track last printed value
unsigned long lastLineSensorRead = 0;
unsigned long lastSensorPrint = 0;
const unsigned long LINE_SENSOR_INTERVAL = 50;  // Read every 50ms
const unsigned long SENSOR_PRINT_INTERVAL = 200;  // Print to serial every 200ms
bool serialSensorOutput = false;  // Toggle for continuous serial output

// Servo configuration
#define SERVO_FREQ 50      // Standard servo frequency
#define SERVOMIN 102       // Min pulse length (~0 degrees) ~0.5ms
#define SERVOMAX 512       // Max pulse length (~180 degrees) ~2.5ms
#define SERVO_STEP_DELAY 25   // Delay in ms between each degree during normal use
#define SERVO_INIT_DELAY 80   // ms per degree during startup (very slow, no ramp)
#define SERVO_PICKUP_DELAY 45 // Delay in ms between each degree during auto pickup
// Dedicated S5 pulse limits (safe extended range, tune carefully if needed)
#define SERVO5_PULSE_MIN 409  // ~= old mapping at 135°
#define SERVO5_PULSE_MAX 560  // extended but capped for safer operation

// Servo channel assignments on PCA9685 (S1-S5)

const int NUM_SERVOS = 5;
const int SERVO_CHANNEL[5] = {0, 1, 2, 9, 8};

// Servo positions (tracked after movement)
int servoPositions[5] = {90, 0, 135, 90, 180};  // Default / home positions

// Approximate angle where each servo rests under gravity when unpowered.
// Tune these to match where each servo physically sits before power.
const int SERVO_GRAVITY[5] = {90, 0, 90, 90, 180};

// Per-servo angle limits: {min, max} for S1-S5
const int SERVO_MIN[5] = {0,  0,  0,  0,   135};
const int SERVO_MAX[5] = {180, 90, 180, 165, 250};

// Robot arm
const float IK_L1 = 135.0;    // Upper arm length (S2 to S3) in mm
const float IK_L2 = 125.0;    // Forearm length (S3 to S4) in mm
const float IK_CLAW = 100.0;  // Claw length (S4 wrist to grab point) in mm
const float IK_H  = 35.0;     // Vertical offset from S1 base to S2 shoulder pivot (mm)

// Motor parameters
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 16;
const int STEP_DELAY = 500;

// Motor state
bool motorEnabled[4] = {false, false, false, false};
volatile bool isMoving = false;
String currentDirection = "";

// Timer-based smooth stepper control (manual movement only)
esp_timer_handle_t stepTimerHandle = NULL;
volatile bool timerStepping = false;
volatile bool stepPhase = false;
volatile bool stepMotorActive[4] = {true, true, true, true};

// Line following mode
bool lineFollowMode = false;
int lineFollowSpeed = 400;  // Step delay for line following (lower = faster)

// Line follow state machine
enum LineFollowState {
  LF_FORWARD,       // Center sensors (4,5) active - drive straight
  LF_CREEP_RIGHT,   // Creeping forward before right 90-degree turn
  LF_CREEP_LEFT,    // Creeping forward before left 90-degree turn
  LF_TURN_RIGHT_90, // Executing 90-degree right turn
  LF_TURN_LEFT_90,  // Executing 90-degree left turn
  LF_SEEK           // Lost line - searching
};

LineFollowState lfState = LF_FORWARD;
int lfTurnStepCount = 0;
unsigned long lfSeekStartTime = 0;
int lfSeekDirection = 0;  // -1=search left, 1=search right

// Line follow tuning constants (adjust for your robot geometry)
const int CREEP_FORWARD_STEPS = 1900;        // Forward steps before turning (move wheels to intersection)
const int TURN_90_STEPS = 700;               // Microsteps to complete a 90-degree rotation
const int LF_SEEK_SPEED = 600;               // Slower step delay during seek (higher = slower)
const unsigned long LF_SEEK_TIMEOUT = 3000;  // Stop seeking after 3 seconds

// Sensor bit masks (1-indexed sensors, 0-indexed bits)
// Sensor 1=bit0(leftmost) ... Sensor 8=bit7(rightmost)
#define LF_CENTER_MASK      0x18  // Sensors 4,5 (bits 3,4)
#define LF_RIGHT_TURN_MASK  0xF8  // Sensors 4,5,6,7,8 (bits 3-7)
#define LF_LEFT_TURN_MASK   0x1F  // Sensors 1,2,3,4,5 (bits 0-4)

// QR Scanner (Grow GM805)
#define QR_RX_PIN 41  // ESP32-S3 RX <- Scanner TX
#define QR_TX_PIN 42  // ESP32-S3 TX -> Scanner RX (optional)
#define QR_BAUD 9600
String lastQRCode = "";  // Store last scanned code
unsigned long lastQRTime = 0;  // Time of last scan

// Lane following mode (ultrasonic wall alignment)
bool laneFollowMode = false;
int laneFollowSpeed = 500;  // Step delay for lane following

enum LaneFollowState {
  LANE_MEASURE,      // Stopped: wait for stable sensor readings, then decide
  LANE_CORRECT_CW,   // Rotating CW for a fixed step burst
  LANE_CORRECT_CCW,  // Rotating CCW for a fixed step burst
  LANE_SHIFT_LEFT,   // Driving forward-left for a fixed step burst
  LANE_SHIFT_RIGHT,  // Driving forward-right for a fixed step burst
  LANE_DRIVE         // Driving straight forward for a fixed step burst
};

LaneFollowState lnState = LANE_MEASURE;
unsigned long lnMeasureStart = 0;
int lnStepCount = 0;

// Lane follow tuning (adjust for your corridor width and sensor noise)
const float LANE_ALIGN_THRESHOLD = 2.0;       // cm: max allowed difference between right & back
const float LANE_CENTER_THRESHOLD = 3.5;      // cm: max allowed difference between left & right
const unsigned long LANE_MEASURE_MS = 500;    // ms: stop and wait for stable readings before deciding
const int LANE_CORRECT_STEPS = 90;            // steps per rotation correction burst
const int LANE_SHIFT_STEPS = 300;             // steps per diagonal shift burst
const int LANE_DRIVE_STEPS = 400;             // steps per forward driving burst
// Auto pickup mode (front ultrasonic + IK)
bool autoPickupMode = false;
unsigned long pickupDetectStart = 0;
unsigned long pickupCooldownUntil = 0;
float pickupTargetBaseDistanceCm = 0.0;
float pickupTargetXmm = 0.0;
float pickupTargetZmm = 0.0;
float pickupTargetGrabYmm = 127.0;
String pickupTargetSensor = "none";

enum PickupSensorSelect {
  PICKUP_SENSOR_FRONT_SELECT,
  PICKUP_SENSOR_RIGHT_SELECT,
  PICKUP_SENSOR_LEFT_SELECT
};

PickupSensorSelect pickupPreferredSensor = PICKUP_SENSOR_RIGHT_SELECT;  // Focus on right sensor for now
bool pickupOnlyPreferredSensor = false;                                 // Allow fallback to other sensors
bool pickupEnableFrontSensor = true;
bool pickupEnableRightSensor = true;
bool pickupEnableLeftSensor = true;

enum AutoPickupState {
  AP_IDLE,
  AP_WAIT_OPEN,
  AP_WAIT_APPROACH,
  AP_WAIT_DESCEND,
  AP_WAIT_GRIP,
  AP_WAIT_LIFT,
  AP_WAIT_PARK
};

AutoPickupState pickupState = AP_IDLE;
unsigned long pickupStateStart = 0;

// Non-blocking servo interpolation for pickup sequence
bool pickupServoMotionActive = false;
int pickupServoTargets[5] = {90, 0, 135, 90, 180};
unsigned long pickupServoNextStepAt = 0;
int pickupServoStepDelayMs = SERVO_PICKUP_DELAY;
int pickupServoActiveOrderIdx = 0;  // 0..4 -> S1..S5 sequential motion
unsigned long pickupSw3DetectStart = 0;

// Auto pickup tuning
// Distances are referenced from the arm base center (not from sensor face)
const float PICKUP_FRONT_SENSOR_FORWARD_OFFSET_CM = 5.0;   // Front sensor center offset along +X (forward)
const float PICKUP_RIGHT_SENSOR_FORWARD_OFFSET_CM = 5.0;   // Right sensor center offset along +X (forward)
const float PICKUP_RIGHT_SENSOR_RIGHT_OFFSET_CM = 5.0;     // Right sensor center offset along +Z (right)
const float PICKUP_LEFT_SENSOR_FORWARD_OFFSET_CM = 5.0;    // Left sensor center offset along +X (forward)
const float PICKUP_LEFT_SENSOR_LEFT_OFFSET_CM = 5.0;       // Left sensor center offset toward left (−Z)
// Right sensor 4-point calibration (distanceRight in cm -> x,y,z in mm)
// User-provided dataset (Y raised +12mm for higher grab):
const int PICKUP_RIGHT_CAL_POINT_COUNT = 4;
const float PICKUP_RIGHT_CAL_DIST_CM[PICKUP_RIGHT_CAL_POINT_COUNT] = {6.5, 8.8, 10.8, 12.5};
const float PICKUP_RIGHT_CAL_X_MM[PICKUP_RIGHT_CAL_POINT_COUNT] = {67.0, 32.0, 29.0, 53.0};
const float PICKUP_RIGHT_CAL_Y_MM[PICKUP_RIGHT_CAL_POINT_COUNT] = {124.0, 109.0, 99.0, 125.0};
const float PICKUP_RIGHT_CAL_Z_MM[PICKUP_RIGHT_CAL_POINT_COUNT] = {218.0, 263.0, 273.0, 273.0};
// Left sensor calibration (mirror of right: same X & Y, Z negated)
const int PICKUP_LEFT_CAL_POINT_COUNT = 4;
const float PICKUP_LEFT_CAL_DIST_CM[PICKUP_LEFT_CAL_POINT_COUNT] = {6.5, 8.8, 10.8, 12.5};
const float PICKUP_LEFT_CAL_X_MM[PICKUP_LEFT_CAL_POINT_COUNT] = {67.0, 32.0, 29.0, 53.0};
const float PICKUP_LEFT_CAL_Y_MM[PICKUP_LEFT_CAL_POINT_COUNT] = {124.0, 109.0, 99.0, 125.0};
const float PICKUP_LEFT_CAL_Z_MM[PICKUP_LEFT_CAL_POINT_COUNT] = {-218.0, -263.0, -273.0, -273.0};
// Front sensor 2-point calibration (distanceFront+offset in cm -> x,y,z in mm)
const int PICKUP_FRONT_CAL_POINT_COUNT = 2;
const float PICKUP_FRONT_CAL_DIST_CM[PICKUP_FRONT_CAL_POINT_COUNT] = {9.1, 12.9};
const float PICKUP_FRONT_CAL_X_MM[PICKUP_FRONT_CAL_POINT_COUNT] = {240.0, 280.0};
const float PICKUP_FRONT_CAL_Y_MM[PICKUP_FRONT_CAL_POINT_COUNT] = {34.0, 42.0};
const float PICKUP_FRONT_CAL_Z_MM[PICKUP_FRONT_CAL_POINT_COUNT] = {6.0, 6.0};
float pickupMinBaseDistanceCm = 2.0;                  // Adjustable minimum pickup distance
float pickupMaxBaseDistanceCm = 15.0;                 // Adjustable maximum pickup distance
const unsigned long PICKUP_CONFIRM_MS = 350;          // Object must be stable in-range this long
const unsigned long PICKUP_COOLDOWN_MS = 1500;        // Pause between pickup attempts

// Auto pickup motion tuning
const int PICKUP_CLAW_OPEN_ANGLE = 135;               // S5 max open
const int PICKUP_CLAW_CLOSE_ANGLE = 250;              // S5 fully closed/grip
const unsigned long PICKUP_STAGE_PAUSE_MS = 200;      // Settling pause between stages
const unsigned long PICKUP_GRIP_SETTLE_MS = 350;      // Extra settle after grip
const bool PICKUP_REQUIRE_SW3 = false;                // SW3 disabled (switch unreliable)
const unsigned long PICKUP_SW3_TIMEOUT_MS = 700;      // Max extra wait after grip settle for SW3 trigger
const unsigned long PICKUP_SW3_STABLE_MS = 80;        // SW3 must stay pressed this long
const float PICKUP_APPROACH_Y_MM = 147.0;             // Approach height
const float PICKUP_GRAB_Y_MM = 127.0;                 // Calibrated grab height reference
const float PICKUP_LIFT_Y_MM = 172.0;                 // Lift height after gripping
const float PICKUP_APPROACH_Y_OFFSET_MM = (PICKUP_APPROACH_Y_MM - PICKUP_GRAB_Y_MM);
const float PICKUP_LIFT_Y_OFFSET_MM = (PICKUP_LIFT_Y_MM - PICKUP_GRAB_Y_MM);
const float PICKUP_GRAB_X_EXTRA_OFFSET_MM = -8.0;    // Extra X offset added to all grab targets (tweak me)
const float PICKUP_GRAB_Y_EXTRA_OFFSET_MM = -20.0;   // Extra Y offset added to all grab targets (tweak me)
const float PICKUP_GRAB_Z_EXTRA_OFFSET_MM = -31.0;    // Extra Z offset added to all grab targets (tweak me)
const float PICKUP_GRAB_FRONT_X_EXTRA_OFFSET_MM = -30.0;  // Extra X offset for front sensor only (tweak me)
const float PICKUP_GRAB_FRONT_Y_EXTRA_OFFSET_MM = 45.0;  // Extra Y offset for front sensor only (tweak me)
const float PICKUP_GRAB_FRONT_Z_EXTRA_OFFSET_MM = 0.0;  // Extra Z offset for front sensor only (tweak me)
const float PICKUP_GRAB_Y_MIN_MM = 60.0;
const float PICKUP_GRAB_Y_MAX_MM = 230.0;
const float PICKUP_PARK_X_MM = 120.0;                 // Park pose after pickup
const float PICKUP_PARK_Y_MM = 180.0;
const float PICKUP_PARK_Z_MM = 0.0;

// Forward declarations
void handleRoot();
void handleMove();
void handleStop();
void handleServo();
void handleDistance();
void handleLineSensor();
void handleLineFollow();
void handleQRCode();
void handleLimitSwitches();
void handleLaneFollow();
void handleIK();
void handleCapTouch();
void handleAutoPickup();
bool solveIK(float x, float y, float z, int s4in, int &s1, int &s2, int &s3);
bool solveIKAuto(float x, float y, float z, int s4pref, int &s1, int &s2, int &s3, int &s4out);
bool moveArmIKSlow(float x, float y, float z, int s4pref, int servoDelayMs);
void autoPickupStep();
void setAutoPickupState(AutoPickupState state);
void resetAutoPickupSequence();
void startAutoPickupServoMove(int s1, int s2, int s3, int s4, int s5, int stepDelayMs);
bool updateAutoPickupServoMove();
bool startAutoPickupIKMove(float x, float y, float z);
void failAutoPickup(const char* reason);
void applyMotorDirection();
void lineFollowStep();
void lfStepMotors(const bool dir[4], int speed);
void lfDriveForward();
void lfRotateCW(int speed);
void lfRotateCCW(int speed);
void laneFollowStep();
void lnStepMotors(const bool dir[4], const bool active[4], int speed);
void lnDriveForward();
void lnDriveForwardLeft();
void lnDriveForwardRight();
void lnRotateCW();
void lnRotateCCW();
void processSerialCommand();
void setServoAngle(uint8_t channel, int angle);
void updateDisplay();
float readDistanceSensor(int trigPin, int echoPin);
float readDistancePCF(int pcfPin, int echoPin);
void readAllDistances();
void pcfWrite(uint8_t pin, bool state);
bool pcfRead(uint8_t pin);
uint8_t readLineSensor();
void printSensorData();
void checkQRScanner();
void readInductiveSensor();
void readLimitSwitches();
void readCapTouch();
void setServoAngleTimed(uint8_t channel, int targetAngle, int stepDelayMs);
void computeRightPickupTarget(float distanceCm, float &xMm, float &grabYmm, float &zMm);
void computeLeftPickupTarget(float distanceCm, float &xMm, float &grabYmm, float &zMm);
void computeFrontPickupTarget(float distanceCm, float &xMm, float &grabYmm, float &zMm);

// Timer ISR for smooth manual stepping
void IRAM_ATTR stepTimerCallback(void* arg) {
  if (!timerStepping) return;
  
  stepPhase = !stepPhase;
  
  for (int i = 0; i < 4; i++) {
    if (stepMotorActive[i]) {
      digitalWrite(STEP_PINS[i], stepPhase ? HIGH : LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize QR Scanner serial
  Serial2.begin(QR_BAUD, SERIAL_8N1, QR_RX_PIN, QR_TX_PIN);
  Serial.println("QR Scanner initialized (Serial2)");
  
  // Initialize motor pins  
  for (int i = 0; i < 4; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], HIGH);  // Disable on startup
    digitalWrite(STEP_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], LOW);
  }
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  digitalWrite(TRIG_PIN_FRONT, LOW);
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  digitalWrite(TRIG_PIN_LEFT, LOW);
  // Right sensor uses PCF8574T for TRIG
  pinMode(ECHO_PIN_RIGHT, INPUT);
  // Back sensor uses PCF8574T for TRIG
  pinMode(ECHO_PIN_BACK, INPUT);
  
  // Initialize I2C bus FIRST (shared by PCA9685, OLED, PCF8574T, line sensor)
  Wire.begin(1, 3);  // SDA=1, SCL=3 (moved off shield-conflicting 8/9)
  
  // Initialize PCF8574T - all pins high (input mode with pull-ups)
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(0xFF);
  Wire.endTransmission();
  Serial.println("Ultrasonic sensors initialized (4x, PCF8574T)");
  

  // Initialize PCA9685 servo driver with outputs DISABLED
  pinMode(PCA9685_OE_PIN, OUTPUT);
  digitalWrite(PCA9685_OE_PIN, HIGH);  // Disable outputs before init
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  Serial.println("PCA9685 initialized (outputs disabled)");

  Serial.println("Line sensor initialized (I2C)");
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    Serial.println("OLED initialized");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
  }
  
  
  // Create Access Point
  Serial.println("\nCreating Access Point...");
  WiFi.softAP(ap_ssid, ap_password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("\nAccess Point created!");
  Serial.print("AP SSID: ");
  Serial.println(ap_ssid);
  Serial.print("AP Password: ");
  Serial.println(ap_password);
  Serial.print("IP address: ");
  Serial.println(IP);
  Serial.println("Connect to the AP and open http://192.168.4.1");
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/stop", handleStop);
  server.on("/servo", handleServo);
  server.on("/distance", handleDistance);
  server.on("/linesensor", handleLineSensor);
  server.on("/linefollow", handleLineFollow);
  server.on("/qrcode", handleQRCode);
  server.on("/limits", handleLimitSwitches);
  server.on("/lanefollow", handleLaneFollow);
  server.on("/ik", handleIK);
  server.on("/captouch", handleCapTouch);
  server.on("/autopickup", handleAutoPickup);
  
  server.begin();
  Serial.println("Web server started");
  Serial.println("\n=== Serial Commands ===");
  Serial.println("s<1-5> <angle>  - Servo control (e.g. s1 90)");
  Serial.println("lf              - Start line following");
  Serial.println("stop            - Stop all movement");
  Serial.println("sensor          - Show sensor reading once");
  Serial.println("dist            - Show distance reading");
  Serial.println("mon             - Toggle continuous sensor monitoring");
  Serial.println("scan            - Scan I2C bus for devices");
  Serial.println("metal           - Show inductive sensor state");
  Serial.println("limit           - Show limit switch states");
  Serial.println("cap             - Show capacitive touch state");
  Serial.println("ln              - Toggle lane following");
  Serial.println("pickup          - Toggle auto pickup mode");
  Serial.println("ik x y z        - Move arm to position (mm)");
  Serial.println("=========================\n");
  
  // Setup step timer for smooth manual movement
  const esp_timer_create_args_t stepTimerArgs = {
    .callback = stepTimerCallback,
    .name = "step_timer"
  };
  esp_timer_create(&stepTimerArgs, &stepTimerHandle);
  esp_timer_start_periodic(stepTimerHandle, STEP_DELAY);
  Serial.println("Step timer initialized");
  
  // Set all servos to gravity positions while outputs are disabled
  for (int i = 0; i < NUM_SERVOS; i++) {
    int start = constrain(SERVO_GRAVITY[i], SERVO_MIN[i], SERVO_MAX[i]);
    setServoAngleDirect(i, start);
  }
  delay(500);

  // Enable outputs — servos now powered at gravity position
  digitalWrite(PCA9685_OE_PIN, LOW);
  Serial.println("PCA9685 outputs enabled");
  delay(300);

  // Slowly sweep each servo from gravity to home, one at a time
  for (int i = 0; i < NUM_SERVOS; i++) {
    int target = constrain(servoPositions[i], SERVO_MIN[i], SERVO_MAX[i]);
    int start  = constrain(SERVO_GRAVITY[i], SERVO_MIN[i], SERVO_MAX[i]);
    int dir    = (target >= start) ? 1 : -1;
    for (int a = start + dir; a != target + dir; a += dir) {
      setServoAngleDirect(i, a);
      delay(SERVO_INIT_DELAY);
    }

    servoPositions[i] = target;
    Serial.printf("Servo %d -> %d°\n", i + 1, target);
    delay(300);
  }
  Serial.println("Servos initialized");
  
  // Show initial display
  updateDisplay();
}

// Set servo to angle immediately (used internally)
void setServoAngleDirect(uint8_t channel, int angle) {
  angle = constrain(angle, SERVO_MIN[channel], SERVO_MAX[channel]);
  int pulse;
  if (channel == 4) {
    // S5 uses separate mapping so it can run 135..250 without affecting S1-S4
    pulse = map(angle, SERVO_MIN[4], SERVO_MAX[4], SERVO5_PULSE_MIN, SERVO5_PULSE_MAX);
    pulse = constrain(pulse, SERVO5_PULSE_MIN, SERVO5_PULSE_MAX);
  } else {
    pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  }
  pwm.setPWM(SERVO_CHANNEL[channel], 0, pulse);
}


// Move servo smoothly from current position to target angle
void setServoAngle(uint8_t channel, int targetAngle) {
  setServoAngleTimed(channel, targetAngle, SERVO_STEP_DELAY);
}

// Move servo smoothly with configurable speed (ms per degree)
void setServoAngleTimed(uint8_t channel, int targetAngle, int stepDelayMs) {
  targetAngle = constrain(targetAngle, SERVO_MIN[channel], SERVO_MAX[channel]);
  int currentAngle = servoPositions[channel];
  if (stepDelayMs < 1) stepDelayMs = 1;
  
  // Move incrementally
  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      setServoAngleDirect(channel, angle);
      delay(stepDelayMs);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      setServoAngleDirect(channel, angle);
      delay(stepDelayMs);
    }
  }
  servoPositions[channel] = targetAngle;
}

// Inverse kinematics
// Arm geometry: S1 ─ IK_H ─ S2 ─ L1 ─ S3 ─ L2 ─ S4(wrist angle) ─ CLAW ─ grab
// S2 = shoulder. S3 = elbow opening (0=fully folded, 180=inline).
// S4 rotates the claw relative to the forearm; the solver uses a virtual
// link that combines L2 + angled claw into an equivalent single link.
// Input: x,y,z (mm) relative to S1 base, s4in = current S4 servo angle
// Output: s1 (base), s2 (shoulder), s3 (elbow)
bool solveIK(float x, float y, float z, int s4in, int &s1, int &s2, int &s3) {
  // Wrist relative angle: S4=90 → inline, deviation bends the claw
  float wristRel = (s4in - 90.0) * M_PI / 180.0;
  
  // Virtual second link: forearm + angled claw combined
  float a  = IK_L2 + IK_CLAW * cos(wristRel);
  float b  = IK_CLAW * sin(wristRel);
  float Rv = sqrt(a * a + b * b);
  float phi = atan2(b, a);
  
  // Base rotation (S1): 0°=right, 90°=forward, 180°=left
  float r = sqrt(x * x + z * z);
  float baseDeg = 90.0 - atan2(z, x) * 180.0 / M_PI;
  
  // Handle targets behind the robot: flip base 180° and negate r
  if (baseDeg > 180.0) {
    baseDeg -= 180.0;
    r = -r;
  } else if (baseDeg < 0.0) {
    baseDeg += 180.0;
    r = -r;
  }
  
  if (baseDeg < SERVO_MIN[0] || baseDeg > SERVO_MAX[0]) return false;
  s1 = (int)round(baseDeg);
  
  if (fabs(r) < 0.1) r = (r >= 0) ? 0.1 : -0.1;
  
  // Subtract shoulder height offset
  float yAdj = y - IK_H;
  
  // 2-link IK with Link 1 = IK_L1, Link 2 = Rv (virtual)
  float D_sq = r * r + yAdj * yAdj;
  float cosElbow = (D_sq - IK_L1 * IK_L1 - Rv * Rv) / (2.0 * IK_L1 * Rv);
  
  // Clamp tiny float overshoot (32-bit float precision)
  if (cosElbow < -1.01f || cosElbow > 1.01f) return false;  // truly unreachable
  cosElbow = constrain(cosElbow, -1.0f, 1.0f);
  
  // Try elbow-up first, fall back to elbow-down
  for (int attempt = 0; attempt < 2; attempt++) {
    float elbowRel = (attempt == 0) ? acos(cosElbow) : -acos(cosElbow);
    
    float k1 = IK_L1 + Rv * cos(elbowRel);
    float k2 = Rv * sin(elbowRel);
    
    float theta1 = atan2(yAdj, r) - atan2(k2, k1);
    float virtualT2 = theta1 + elbowRel;     // virtual link angle
    float theta2abs = virtualT2 - phi;        // actual forearm angle
    
    // S2 from absolute shoulder angle, S3 from relative elbow angle
    // S3=90 → inline, S3=180 → forward from up, S3=0 → backward
    int s2i = (int)round(90.0 - theta1 * 180.0 / M_PI);
    int s3i = (int)round(90.0 - (theta2abs - theta1) * 180.0 / M_PI);
    
    if (s2i >= SERVO_MIN[1] && s2i <= SERVO_MAX[1] &&
        s3i >= SERVO_MIN[2] && s3i <= SERVO_MAX[2]) {
      s2 = s2i;
      s3 = s3i;
      return true;
    }
  }
  return false;
}

// Auto-search wrapper: tries preferred S4 first, then searches outward
// for the nearest S4 angle that makes the position reachable.
bool solveIKAuto(float x, float y, float z, int s4pref, int &s1, int &s2, int &s3, int &s4out) {
  // Try preferred S4 first
  if (solveIK(x, y, z, s4pref, s1, s2, s3)) {
    s4out = s4pref;
    return true;
  }
  // Search outward from preferred S4 for closest working angle
  for (int delta = 1; delta <= SERVO_MAX[3]; delta++) {
    for (int sign = 1; sign >= -1; sign -= 2) {
      int s4t = s4pref + sign * delta;
      if (s4t >= SERVO_MIN[3] && s4t <= SERVO_MAX[3]) {
        if (solveIK(x, y, z, s4t, s1, s2, s3)) {
          s4out = s4t;
          return true;
        }
      }
    }
  }
  return false;
}

// Solve IK for target and move arm slowly/steadily
bool moveArmIKSlow(float x, float y, float z, int s4pref, int servoDelayMs) {
  int s1, s2, s3, s4;
  if (!solveIKAuto(x, y, z, s4pref, s1, s2, s3, s4)) {
    return false;
  }
  setServoAngleTimed(3, s4, servoDelayMs);  // Wrist first
  setServoAngleTimed(0, s1, servoDelayMs);
  setServoAngleTimed(1, s2, servoDelayMs);
  setServoAngleTimed(2, s3, servoDelayMs);
  return true;
}

void setAutoPickupState(AutoPickupState state) {
  pickupState = state;
  pickupStateStart = millis();
}

void resetAutoPickupSequence() {
  pickupDetectStart = 0;
  pickupTargetBaseDistanceCm = 0.0;
  pickupTargetXmm = 0.0;
  pickupTargetZmm = 0.0;
  pickupTargetGrabYmm = PICKUP_GRAB_Y_MM;
  pickupTargetSensor = "none";
  pickupServoActiveOrderIdx = 0;
  pickupSw3DetectStart = 0;
  pickupServoMotionActive = false;
  setAutoPickupState(AP_IDLE);
}

void startAutoPickupServoMove(int s1, int s2, int s3, int s4, int s5, int stepDelayMs) {
  int targets[5] = {s1, s2, s3, s4, s5};
  for (int i = 0; i < NUM_SERVOS; i++) {
    pickupServoTargets[i] = constrain(targets[i], SERVO_MIN[i], SERVO_MAX[i]);
  }
  pickupServoStepDelayMs = max(1, stepDelayMs);
  pickupServoActiveOrderIdx = 0;
  pickupServoNextStepAt = millis();
  pickupServoMotionActive = true;
}

bool updateAutoPickupServoMove() {
  if (!pickupServoMotionActive) return true;

  unsigned long now = millis();
  if ((long)(now - pickupServoNextStepAt) < 0) return false;
  // Advance to first servo that still needs movement
  while (pickupServoActiveOrderIdx < NUM_SERVOS &&
         servoPositions[pickupServoActiveOrderIdx] == pickupServoTargets[pickupServoActiveOrderIdx]) {
    pickupServoActiveOrderIdx++;
  }

  if (pickupServoActiveOrderIdx >= NUM_SERVOS) {
    pickupServoMotionActive = false;
    return true;
  }

  int i = pickupServoActiveOrderIdx;
  int cur = servoPositions[i];
  int tgt = pickupServoTargets[i];
  if (cur < tgt) {
    cur++;
  } else if (cur > tgt) {
    cur--;
  }
  if (cur != servoPositions[i]) {
    setServoAngleDirect(i, cur);
    servoPositions[i] = cur;
  }

  // If this servo reached target now, move to next in S1->S5 order on next tick
  if (servoPositions[i] == pickupServoTargets[i]) {
    pickupServoActiveOrderIdx++;
  }

  pickupServoNextStepAt = now + pickupServoStepDelayMs;
  return false;
}

bool startAutoPickupIKMove(float x, float y, float z) {
  int s1, s2, s3, s4;
  if (!solveIKAuto(x, y, z, servoPositions[3], s1, s2, s3, s4)) {
    return false;
  }
  startAutoPickupServoMove(s1, s2, s3, s4, servoPositions[4], SERVO_PICKUP_DELAY);
  return true;
}

void failAutoPickup(const char* reason) {
  Serial.print("Pickup: ");
  Serial.println(reason);
  pickupCooldownUntil = millis() + PICKUP_COOLDOWN_MS;
  resetAutoPickupSequence();
  updateDisplay();
}
void computeRightPickupTarget(float distanceCm, float &xMm, float &grabYmm, float &zMm) {
  // Clamp to nearest endpoint outside calibrated distance range
  if (distanceCm <= PICKUP_RIGHT_CAL_DIST_CM[0]) {
    xMm = PICKUP_RIGHT_CAL_X_MM[0];
    grabYmm = PICKUP_RIGHT_CAL_Y_MM[0];
    zMm = PICKUP_RIGHT_CAL_Z_MM[0];
    return;
  }

  for (int i = 0; i < PICKUP_RIGHT_CAL_POINT_COUNT - 1; i++) {
    float d0 = PICKUP_RIGHT_CAL_DIST_CM[i];
    float d1 = PICKUP_RIGHT_CAL_DIST_CM[i + 1];
    if (distanceCm <= d1) {
      float span = d1 - d0;
      float t = (span > 0.0001f) ? ((distanceCm - d0) / span) : 0.0f;
      xMm = PICKUP_RIGHT_CAL_X_MM[i] + t * (PICKUP_RIGHT_CAL_X_MM[i + 1] - PICKUP_RIGHT_CAL_X_MM[i]);
      grabYmm = PICKUP_RIGHT_CAL_Y_MM[i] + t * (PICKUP_RIGHT_CAL_Y_MM[i + 1] - PICKUP_RIGHT_CAL_Y_MM[i]);
      zMm = PICKUP_RIGHT_CAL_Z_MM[i] + t * (PICKUP_RIGHT_CAL_Z_MM[i + 1] - PICKUP_RIGHT_CAL_Z_MM[i]);
      return;
    }
  }

  int last = PICKUP_RIGHT_CAL_POINT_COUNT - 1;
  xMm = PICKUP_RIGHT_CAL_X_MM[last];
  grabYmm = PICKUP_RIGHT_CAL_Y_MM[last];
  zMm = PICKUP_RIGHT_CAL_Z_MM[last];
}

void computeLeftPickupTarget(float distanceCm, float &xMm, float &grabYmm, float &zMm) {
  // Clamp to nearest endpoint outside calibrated distance range
  if (distanceCm <= PICKUP_LEFT_CAL_DIST_CM[0]) {
    xMm = PICKUP_LEFT_CAL_X_MM[0];
    grabYmm = PICKUP_LEFT_CAL_Y_MM[0];
    zMm = PICKUP_LEFT_CAL_Z_MM[0];
    return;
  }

  for (int i = 0; i < PICKUP_LEFT_CAL_POINT_COUNT - 1; i++) {
    float d0 = PICKUP_LEFT_CAL_DIST_CM[i];
    float d1 = PICKUP_LEFT_CAL_DIST_CM[i + 1];
    if (distanceCm <= d1) {
      float span = d1 - d0;
      float t = (span > 0.0001f) ? ((distanceCm - d0) / span) : 0.0f;
      xMm = PICKUP_LEFT_CAL_X_MM[i] + t * (PICKUP_LEFT_CAL_X_MM[i + 1] - PICKUP_LEFT_CAL_X_MM[i]);
      grabYmm = PICKUP_LEFT_CAL_Y_MM[i] + t * (PICKUP_LEFT_CAL_Y_MM[i + 1] - PICKUP_LEFT_CAL_Y_MM[i]);
      zMm = PICKUP_LEFT_CAL_Z_MM[i] + t * (PICKUP_LEFT_CAL_Z_MM[i + 1] - PICKUP_LEFT_CAL_Z_MM[i]);
      return;
    }
  }

  int last = PICKUP_LEFT_CAL_POINT_COUNT - 1;
  xMm = PICKUP_LEFT_CAL_X_MM[last];
  grabYmm = PICKUP_LEFT_CAL_Y_MM[last];
  zMm = PICKUP_LEFT_CAL_Z_MM[last];
}

void computeFrontPickupTarget(float distanceCm, float &xMm, float &grabYmm, float &zMm) {
  // Clamp to nearest endpoint outside calibrated distance range
  if (distanceCm <= PICKUP_FRONT_CAL_DIST_CM[0]) {
    xMm = PICKUP_FRONT_CAL_X_MM[0];
    grabYmm = PICKUP_FRONT_CAL_Y_MM[0];
    zMm = PICKUP_FRONT_CAL_Z_MM[0];
    return;
  }

  for (int i = 0; i < PICKUP_FRONT_CAL_POINT_COUNT - 1; i++) {
    float d0 = PICKUP_FRONT_CAL_DIST_CM[i];
    float d1 = PICKUP_FRONT_CAL_DIST_CM[i + 1];
    if (distanceCm <= d1) {
      float span = d1 - d0;
      float t = (span > 0.0001f) ? ((distanceCm - d0) / span) : 0.0f;
      xMm = PICKUP_FRONT_CAL_X_MM[i] + t * (PICKUP_FRONT_CAL_X_MM[i + 1] - PICKUP_FRONT_CAL_X_MM[i]);
      grabYmm = PICKUP_FRONT_CAL_Y_MM[i] + t * (PICKUP_FRONT_CAL_Y_MM[i + 1] - PICKUP_FRONT_CAL_Y_MM[i]);
      zMm = PICKUP_FRONT_CAL_Z_MM[i] + t * (PICKUP_FRONT_CAL_Z_MM[i + 1] - PICKUP_FRONT_CAL_Z_MM[i]);
      return;
    }
  }

  int lastF = PICKUP_FRONT_CAL_POINT_COUNT - 1;
  xMm = PICKUP_FRONT_CAL_X_MM[lastF];
  grabYmm = PICKUP_FRONT_CAL_Y_MM[lastF];
  zMm = PICKUP_FRONT_CAL_Z_MM[lastF];
}

// Detect in-range object and run a non-blocking staged pickup sequence
void autoPickupStep() {
  updateAutoPickupServoMove();

  // Keep behavior exclusive from drive/autonomous wheel control
  if (timerStepping || isMoving || lineFollowMode || laneFollowMode) {
    resetAutoPickupSequence();
    return;
  }

  unsigned long now = millis();

  switch (pickupState) {
    case AP_IDLE: {
      if (now < pickupCooldownUntil) return;

      // Build candidate from front sensor (calibrated lookup)
      bool frontValidEcho = (distanceFront > 0.1f && distanceFront < (MAX_DISTANCE - 0.1f));
      float frontAxisCm = PICKUP_FRONT_SENSOR_FORWARD_OFFSET_CM + distanceFront;
      bool frontInRange = pickupEnableFrontSensor && frontValidEcho &&
                          frontAxisCm >= pickupMinBaseDistanceCm &&
                          frontAxisCm <= pickupMaxBaseDistanceCm;
      float frontTargetXmm = 0.0f;
      float frontTargetGrabYmm = PICKUP_GRAB_Y_MM;
      float frontTargetZmm = 0.0f;
      if (frontValidEcho) {
        computeFrontPickupTarget(frontAxisCm, frontTargetXmm, frontTargetGrabYmm, frontTargetZmm);
        frontTargetGrabYmm = constrain(frontTargetGrabYmm, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM);
      }

      // Build candidate from right sensor
      bool rightValidEcho = (distanceRight > 0.1f && distanceRight < (MAX_DISTANCE - 0.1f));
      float rightAxisCm = distanceRight;
      bool rightInRange = pickupEnableRightSensor && rightValidEcho &&
                          rightAxisCm >= pickupMinBaseDistanceCm &&
                          rightAxisCm <= pickupMaxBaseDistanceCm;
      float rightTargetXmm = 0.0f;
      float rightTargetGrabYmm = PICKUP_GRAB_Y_MM;
      float rightTargetZmm = 0.0f;
      if (rightValidEcho) {
        computeRightPickupTarget(distanceRight, rightTargetXmm, rightTargetGrabYmm, rightTargetZmm);
        rightTargetGrabYmm = constrain(rightTargetGrabYmm, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM);
      }

      // Build candidate from left sensor (calibrated, mirrors right side)
      bool leftValidEcho = (distanceLeft > 0.1f && distanceLeft < (MAX_DISTANCE - 0.1f));
      float leftAxisCm = distanceLeft;
      bool leftInRange = pickupEnableLeftSensor && leftValidEcho &&
                         leftAxisCm >= pickupMinBaseDistanceCm &&
                         leftAxisCm <= pickupMaxBaseDistanceCm;
      float leftTargetXmm = 0.0f;
      float leftTargetGrabYmm = PICKUP_GRAB_Y_MM;
      float leftTargetZmm = 0.0f;
      if (leftValidEcho) {
        computeLeftPickupTarget(distanceLeft, leftTargetXmm, leftTargetGrabYmm, leftTargetZmm);
        leftTargetGrabYmm = constrain(leftTargetGrabYmm, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM);
      }

      bool selected = false;
      float selectedAxisCm = 0.0f;
      float selectedXmm = 0.0f;
      float selectedGrabYmm = PICKUP_GRAB_Y_MM;
      float selectedZmm = 0.0f;
      String selectedSensor = "none";

      // Preferred sensor selection (right-focused now)
      if (pickupPreferredSensor == PICKUP_SENSOR_RIGHT_SELECT && rightInRange) {
        selected = true;
        selectedAxisCm = rightAxisCm;
        selectedXmm = rightTargetXmm;
        selectedGrabYmm = rightTargetGrabYmm;
        selectedZmm = rightTargetZmm;
        selectedSensor = "right";
      } else if (pickupPreferredSensor == PICKUP_SENSOR_FRONT_SELECT && frontInRange) {
        selected = true;
        selectedAxisCm = frontAxisCm;
        selectedXmm = frontTargetXmm;
        selectedGrabYmm = frontTargetGrabYmm;
        selectedZmm = frontTargetZmm;
        selectedSensor = "front";
      } else if (pickupPreferredSensor == PICKUP_SENSOR_LEFT_SELECT && leftInRange) {
        selected = true;
        selectedAxisCm = leftAxisCm;
        selectedXmm = leftTargetXmm;
        selectedGrabYmm = leftTargetGrabYmm;
        selectedZmm = leftTargetZmm;
        selectedSensor = "left";
      }

      // Optional fallback to other sensors if preferred is not in range
      if (!selected && !pickupOnlyPreferredSensor) {
        if (rightInRange) {
          selected = true;
          selectedAxisCm = rightAxisCm;
          selectedXmm = rightTargetXmm;
          selectedGrabYmm = rightTargetGrabYmm;
          selectedZmm = rightTargetZmm;
          selectedSensor = "right";
        } else if (frontInRange) {
          selected = true;
          selectedAxisCm = frontAxisCm;
          selectedXmm = frontTargetXmm;
          selectedGrabYmm = frontTargetGrabYmm;
          selectedZmm = frontTargetZmm;
          selectedSensor = "front";
        } else if (leftInRange) {
          selected = true;
          selectedAxisCm = leftAxisCm;
          selectedXmm = leftTargetXmm;
          selectedGrabYmm = leftTargetGrabYmm;
          selectedZmm = leftTargetZmm;
          selectedSensor = "left";
        }
      }

      if (!selected) {
        pickupDetectStart = 0;
        return;
      }

      if (pickupDetectStart == 0) {
        pickupDetectStart = now;
        return;
      }

      if (now - pickupDetectStart < PICKUP_CONFIRM_MS) return;

      pickupDetectStart = 0;
      pickupTargetBaseDistanceCm = selectedAxisCm;
      // Apply offsets: front gets its own X/Z offsets, left/right get shared offsets
      if (selectedSensor == "front") {
        pickupTargetXmm = selectedXmm + PICKUP_GRAB_FRONT_X_EXTRA_OFFSET_MM;
        pickupTargetZmm = selectedZmm + PICKUP_GRAB_FRONT_Z_EXTRA_OFFSET_MM;
      } else if (selectedSensor == "left") {
        pickupTargetXmm = selectedXmm + PICKUP_GRAB_X_EXTRA_OFFSET_MM;
        pickupTargetZmm = selectedZmm - PICKUP_GRAB_Z_EXTRA_OFFSET_MM;
      } else if (selectedSensor == "right") {
        pickupTargetXmm = selectedXmm + PICKUP_GRAB_X_EXTRA_OFFSET_MM;
        pickupTargetZmm = selectedZmm + PICKUP_GRAB_Z_EXTRA_OFFSET_MM;
      } else {
        pickupTargetXmm = selectedXmm;
        pickupTargetZmm = selectedZmm;
      }
      if (selectedSensor == "front") {
        pickupTargetGrabYmm = constrain(selectedGrabYmm + PICKUP_GRAB_FRONT_Y_EXTRA_OFFSET_MM, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM);
      } else {
        pickupTargetGrabYmm = constrain(selectedGrabYmm + PICKUP_GRAB_Y_EXTRA_OFFSET_MM, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM);
      }
      pickupTargetSensor = selectedSensor;

      Serial.print("Pickup: sensor=");
      Serial.print(pickupTargetSensor);
      Serial.print(" axis=");
      Serial.print(pickupTargetBaseDistanceCm, 1);
      Serial.print("cm x=");
      Serial.print(pickupTargetXmm, 1);
      Serial.print("mm z=");
      Serial.print(pickupTargetZmm, 1);
      Serial.print("mm y=");
      Serial.print(pickupTargetGrabYmm, 1);
      Serial.println("mm");

      startAutoPickupServoMove(servoPositions[0], servoPositions[1], servoPositions[2], servoPositions[3], PICKUP_CLAW_OPEN_ANGLE, SERVO_PICKUP_DELAY);
      setAutoPickupState(AP_WAIT_OPEN);
      updateDisplay();
      break;
    }

    case AP_WAIT_OPEN:
      if (pickupServoMotionActive) return;
      if (now - pickupStateStart < PICKUP_STAGE_PAUSE_MS) return;
      if (!startAutoPickupIKMove(pickupTargetXmm, constrain(pickupTargetGrabYmm + PICKUP_APPROACH_Y_OFFSET_MM, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM), pickupTargetZmm)) {
        Serial.print("Pickup: approach IK target failed x=");
        Serial.print(pickupTargetXmm, 1);
        Serial.print(" y=");
        Serial.print(constrain(pickupTargetGrabYmm + PICKUP_APPROACH_Y_OFFSET_MM, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM), 1);
        Serial.print(" z=");
        Serial.println(pickupTargetZmm, 1);
        failAutoPickup("approach IK failed");
        return;
      }
      setAutoPickupState(AP_WAIT_APPROACH);
      break;

    case AP_WAIT_APPROACH:
      if (pickupServoMotionActive) return;
      if (now - pickupStateStart < PICKUP_STAGE_PAUSE_MS) return;
      if (!startAutoPickupIKMove(pickupTargetXmm, pickupTargetGrabYmm, pickupTargetZmm)) {
        failAutoPickup("descend IK failed");
        return;
      }
      setAutoPickupState(AP_WAIT_DESCEND);
      break;

    case AP_WAIT_DESCEND:
      if (pickupServoMotionActive) return;
      if (now - pickupStateStart < PICKUP_STAGE_PAUSE_MS) return;
      startAutoPickupServoMove(servoPositions[0], servoPositions[1], servoPositions[2], servoPositions[3], PICKUP_CLAW_CLOSE_ANGLE, SERVO_PICKUP_DELAY);
      pickupSw3DetectStart = 0;
      setAutoPickupState(AP_WAIT_GRIP);
      break;

    case AP_WAIT_GRIP:
      if (pickupServoMotionActive) return;
      if (now - pickupStateStart < PICKUP_GRIP_SETTLE_MS) return;
      if (PICKUP_REQUIRE_SW3) {
        unsigned long sw3ReadyElapsed = now - (pickupStateStart + PICKUP_GRIP_SETTLE_MS);

        if (limitSwitch3) {
          if (pickupSw3DetectStart == 0) {
            pickupSw3DetectStart = now;
            return;
          }
          if (now - pickupSw3DetectStart < PICKUP_SW3_STABLE_MS) return;
        } else {
          pickupSw3DetectStart = 0;
          if (sw3ReadyElapsed < PICKUP_SW3_TIMEOUT_MS) return;
          failAutoPickup("SW3 not triggered (grip failed)");
          return;
        }
      }
      if (!startAutoPickupIKMove(pickupTargetXmm, constrain(pickupTargetGrabYmm + PICKUP_LIFT_Y_OFFSET_MM, PICKUP_GRAB_Y_MIN_MM, PICKUP_GRAB_Y_MAX_MM), pickupTargetZmm)) {
        failAutoPickup("lift IK failed");
        return;
      }
      setAutoPickupState(AP_WAIT_LIFT);
      break;

    case AP_WAIT_LIFT:
      if (pickupServoMotionActive) return;
      if (now - pickupStateStart < PICKUP_STAGE_PAUSE_MS) return;
      if (!startAutoPickupIKMove(PICKUP_PARK_X_MM, PICKUP_PARK_Y_MM, PICKUP_PARK_Z_MM)) {
        failAutoPickup("park IK failed");
        return;
      }
      setAutoPickupState(AP_WAIT_PARK);
      break;

    case AP_WAIT_PARK:
      if (pickupServoMotionActive) return;
      Serial.println("Pickup: complete");
      pickupCooldownUntil = millis() + PICKUP_COOLDOWN_MS;
      resetAutoPickupSequence();
      updateDisplay();
      break;
  }
}

void loop() {
  server.handleClient();
  
  // Check for serial commands
  if (Serial.available() > 0) {
    processSerialCommand();
  }
  
  // Check QR scanner for data
  checkQRScanner();
  
  // Read ultrasonic sensors periodically
  if (millis() - lastDistanceRead >= DISTANCE_READ_INTERVAL) {
    readAllDistances();
    readInductiveSensor();
    readLimitSwitches();
    readCapTouch();
    lastDistanceRead = millis();
  }
  
  // Read line sensor periodically
  if (millis() - lastLineSensorRead >= LINE_SENSOR_INTERVAL) {
    lineSensorData = readLineSensor();
    if (INVERT_LINE_SENSOR) {
      lineSensorData = ~lineSensorData;  // Invert if sensor outputs 0 for black
    }
    lastLineSensorRead = millis();
  }
  
  // Print sensor data to serial if monitoring enabled
  if (serialSensorOutput && (millis() - lastSensorPrint >= SENSOR_PRINT_INTERVAL)) {
    printSensorData();
    lastSensorPrint = millis();
  }
  
  // Autonomous modes (line/lane follow use their own stepping)
  if (lineFollowMode) {
    lineFollowStep();
  } else if (laneFollowMode) {
    laneFollowStep();
  } else if (autoPickupMode) {
    autoPickupStep();
  }
  // Manual movement is handled by the step timer ISR
}

// Check QR Scanner for incoming data
void checkQRScanner() {
  if (Serial2.available() > 0) {
    String qrData = Serial2.readStringUntil('\n');
    qrData.trim();
    
    if (qrData.length() > 0) {
      lastQRCode = qrData;
      lastQRTime = millis();
      
      Serial.print("QR Code Scanned: ");
      Serial.println(qrData);
      
      // You can add actions based on QR code content here
      // Example: if (qrData == "FORWARD") { ... }
    }
  }
}

// Print sensor data to serial monitor
void printSensorData() {
  Serial.print("Line Sensor: [");
  for (int i = 0; i < 8; i++) {
    if ((lineSensorData >> i) & 1) {
      Serial.print("■");  // Black detected
    } else {
      Serial.print("□");  // White/no line
    }
    if (i < 7) Serial.print(" ");
  }
  Serial.print("] Raw: ");
  Serial.print(lineSensorData, BIN);
  Serial.print(" (");
  Serial.print(lineSensorData);
  Serial.println(")");
}

// === Line follow motor helpers ===

// Step all 4 motors with given direction pattern
void lfStepMotors(const bool dir[4], int speed) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], dir[i] ? LOW : HIGH);
  }
  for (int i = 0; i < 4; i++) {
    digitalWrite(STEP_PINS[i], HIGH);
  }
  delayMicroseconds(speed);
  for (int i = 0; i < 4; i++) {
    digitalWrite(STEP_PINS[i], LOW);
  }
  delayMicroseconds(speed);
}

// Drive straight forward (same pattern as manual forward)
void lfDriveForward() {
  const bool dir[4] = {true, false, true, true};
  lfStepMotors(dir, lineFollowSpeed);
}

// Rotate clockwise in place (for right turns)
// Derived from mecanum kinematics: FL fwd, FR back, RL fwd, RR back
// Tune these booleans if the robot rotates the wrong way
void lfRotateCW(int speed) {
  const bool dir[4] = {true, true, true, false};
  lfStepMotors(dir, speed);
}

// Rotate counter-clockwise in place (for left turns)
void lfRotateCCW(int speed) {
  const bool dir[4] = {false, false, false, true};
  lfStepMotors(dir, speed);
}

// === Line following state machine ===
void lineFollowStep() {
  uint8_t data = lineSensorData;  // Already read & inverted by main loop

  // Evaluate sensor patterns
  bool centerDetected    = (data & LF_CENTER_MASK) != 0;                     // Sensor 4 or 5
  bool rightTurnDetected = (data & LF_RIGHT_TURN_MASK) == LF_RIGHT_TURN_MASK; // All of 4-8
  bool leftTurnDetected  = (data & LF_LEFT_TURN_MASK) == LF_LEFT_TURN_MASK;   // All of 1-5

  // Cross intersection (both patterns match) - continue forward
  if (rightTurnDetected && leftTurnDetected) {
    rightTurnDetected = false;
    leftTurnDetected = false;
  }

  switch (lfState) {

    // ---- FORWARD: center sensors see the line ----
    case LF_FORWARD:
      if (rightTurnDetected) {
        lfState = LF_CREEP_RIGHT;
        lfTurnStepCount = 0;
        Serial.println("LF: -> CREEP RIGHT");
      } else if (leftTurnDetected) {
        lfState = LF_CREEP_LEFT;
        lfTurnStepCount = 0;
        Serial.println("LF: -> CREEP LEFT");
      } else if (centerDetected) {
        lfDriveForward();
      } else {
        // Lost line - enter seek
        lfState = LF_SEEK;
        lfSeekStartTime = millis();
        // Choose seek direction from remaining sensor activity
        int lw = 0, rw = 0;
        for (int i = 0; i < 4; i++) { if ((data >> i) & 1) lw++; }
        for (int i = 4; i < 8; i++) { if ((data >> i) & 1) rw++; }
        lfSeekDirection = (lw >= rw) ? -1 : 1;
        Serial.println("LF: -> SEEK");
      }
      break;

    // ---- CREEP: drive forward to align wheels with intersection ----
    case LF_CREEP_RIGHT:
      if (lfTurnStepCount >= CREEP_FORWARD_STEPS) {
        lfState = LF_TURN_RIGHT_90;
        lfTurnStepCount = 0;
        Serial.println("LF: Creep done -> TURN RIGHT 90");
      } else {
        lfDriveForward();
        lfTurnStepCount++;
      }
      break;

    case LF_CREEP_LEFT:
      if (lfTurnStepCount >= CREEP_FORWARD_STEPS) {
        lfState = LF_TURN_LEFT_90;
        lfTurnStepCount = 0;
        Serial.println("LF: Creep done -> TURN LEFT 90");
      } else {
        lfDriveForward();
        lfTurnStepCount++;
      }
      break;

    // ---- 90-degree RIGHT turn (rotate CW for TURN_90_STEPS) ----
    case LF_TURN_RIGHT_90:
      if (lfTurnStepCount >= TURN_90_STEPS) {
        lfState = LF_FORWARD;
        Serial.println("LF: Turn R done -> FORWARD");
      } else {
        lfRotateCW(lineFollowSpeed);
        lfTurnStepCount++;
      }
      break;

    // ---- 90-degree LEFT turn (rotate CCW for TURN_90_STEPS) ----
    case LF_TURN_LEFT_90:
      if (lfTurnStepCount >= TURN_90_STEPS) {
        lfState = LF_FORWARD;
        Serial.println("LF: Turn L done -> FORWARD");
      } else {
        lfRotateCCW(lineFollowSpeed);
        lfTurnStepCount++;
      }
      break;

    // ---- SEEK: line lost, rotate to re-acquire ----
    case LF_SEEK:
      if (rightTurnDetected) {
        lfState = LF_CREEP_RIGHT;
        lfTurnStepCount = 0;
        Serial.println("LF: Seek -> CREEP RIGHT");
      } else if (leftTurnDetected) {
        lfState = LF_CREEP_LEFT;
        lfTurnStepCount = 0;
        Serial.println("LF: Seek -> CREEP LEFT");
      } else if (centerDetected) {
        lfState = LF_FORWARD;
        Serial.println("LF: Seek -> FORWARD");
      } else if (data != 0) {
        // Partial detection - steer towards active sensors
        int lw = 0, rw = 0;
        for (int i = 0; i < 4; i++) { if ((data >> i) & 1) lw++; }
        for (int i = 4; i < 8; i++) { if ((data >> i) & 1) rw++; }
        if (lw > rw) {
          lfRotateCCW(LF_SEEK_SPEED);
        } else {
          lfRotateCW(LF_SEEK_SPEED);
        }
        lfSeekStartTime = millis();  // Reset timeout while partially detecting
      } else if (millis() - lfSeekStartTime < LF_SEEK_TIMEOUT) {
        // No detection at all - rotate in last known direction
        if (lfSeekDirection <= 0) {
          lfRotateCCW(LF_SEEK_SPEED);
        } else {
          lfRotateCW(LF_SEEK_SPEED);
        }
      } else {
        // Seek timeout - hold motors still
        for (int i = 0; i < 4; i++) {
          digitalWrite(STEP_PINS[i], LOW);
        }
      }
      break;
  }
}

// === Lane follow motor helpers ===

// Step selected motors with given direction pattern
void lnStepMotors(const bool dir[4], const bool active[4], int speed) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);  // Enable all to prevent sliding
    digitalWrite(DIR_PINS[i], dir[i] ? LOW : HIGH);
  }
  for (int i = 0; i < 4; i++) {
    if (active[i]) digitalWrite(STEP_PINS[i], HIGH);
  }
  delayMicroseconds(speed);
  for (int i = 0; i < 4; i++) {
    if (active[i]) digitalWrite(STEP_PINS[i], LOW);
  }
  delayMicroseconds(speed);
}

void lnDriveForward() {
  const bool dir[4]    = {true, false, true, true};
  const bool active[4] = {true, true, true, true};
  lnStepMotors(dir, active, laneFollowSpeed);
}

// Forward-left diagonal (same pattern as manual forward-left)
void lnDriveForwardLeft() {
  const bool dir[4]    = {true, false, true, false};
  const bool active[4] = {true, false, true, false};
  lnStepMotors(dir, active, laneFollowSpeed);
}

// Forward-right diagonal (same pattern as manual forward-right)
void lnDriveForwardRight() {
  const bool dir[4]    = {true, false, true, true};
  const bool active[4] = {false, true, false, true};
  lnStepMotors(dir, active, laneFollowSpeed);
}

void lnRotateCW() {
  const bool dir[4]    = {true, true, true, false};
  const bool active[4] = {true, true, true, true};
  lnStepMotors(dir, active, laneFollowSpeed);
}

void lnRotateCCW() {
  const bool dir[4]    = {false, false, false, true};
  const bool active[4] = {true, true, true, true};
  lnStepMotors(dir, active, laneFollowSpeed);
}

// === Lane following state machine ===
// Stop-Measure-Act approach: never reads sensors while moving.
// MEASURE: stop motors, wait for stable ultrasonic readings, decide next action.
// CORRECT/SHIFT/DRIVE: execute a fixed number of motor steps, then re-measure.
void laneFollowStep() {
  LaneFollowState prevState = lnState;

  switch (lnState) {

    // ---- MEASURE: motors stopped, wait for stable reading, then decide ----
    case LANE_MEASURE: {
      // Keep all motors stopped
      for (int i = 0; i < 4; i++) {
        digitalWrite(STEP_PINS[i], LOW);
      }
      // Wait for sensors to stabilize
      if (millis() - lnMeasureStart < LANE_MEASURE_MS) break;

      // Sensors have had time to settle — read current values
      float alignErr  = distanceRight - distanceBack;   // positive → CCW needed
      float centerErr = distanceLeft  - distanceRight;  // positive → shift left

      Serial.print("LN: R="); Serial.print(distanceRight,1);
      Serial.print(" B="); Serial.print(distanceBack,1);
      Serial.print(" L="); Serial.print(distanceLeft,1);
      Serial.print(" aErr="); Serial.print(alignErr,1);
      Serial.print(" cErr="); Serial.println(centerErr,1);

      // Priority 1: Angular alignment (parallel to right wall)
      if (alignErr > LANE_ALIGN_THRESHOLD) {
        lnState = LANE_CORRECT_CCW;
        lnStepCount = 0;
      } else if (alignErr < -LANE_ALIGN_THRESHOLD) {
        lnState = LANE_CORRECT_CW;
        lnStepCount = 0;
      }
      // Priority 2: Lateral centering
      else if (centerErr > LANE_CENTER_THRESHOLD) {
        lnState = LANE_SHIFT_LEFT;
        lnStepCount = 0;
      } else if (centerErr < -LANE_CENTER_THRESHOLD) {
        lnState = LANE_SHIFT_RIGHT;
        lnStepCount = 0;
      }
      // Aligned and centered — drive forward
      else {
        lnState = LANE_DRIVE;
        lnStepCount = 0;
      }
      break;
    }

    // ---- CORRECT: rotate a fixed burst then re-measure ----
    case LANE_CORRECT_CW:
      if (lnStepCount >= LANE_CORRECT_STEPS) {
        lnState = LANE_MEASURE;
        lnMeasureStart = millis();
      } else {
        lnRotateCW();
        lnStepCount++;
      }
      break;

    case LANE_CORRECT_CCW:
      if (lnStepCount >= LANE_CORRECT_STEPS) {
        lnState = LANE_MEASURE;
        lnMeasureStart = millis();
      } else {
        lnRotateCCW();
        lnStepCount++;
      }
      break;

    // ---- SHIFT: drive diagonally a fixed burst then re-measure ----
    case LANE_SHIFT_LEFT:
      if (lnStepCount >= LANE_SHIFT_STEPS) {
        lnState = LANE_MEASURE;
        lnMeasureStart = millis();
      } else {
        lnDriveForwardLeft();
        lnStepCount++;
      }
      break;

    case LANE_SHIFT_RIGHT:
      if (lnStepCount >= LANE_SHIFT_STEPS) {
        lnState = LANE_MEASURE;
        lnMeasureStart = millis();
      } else {
        lnDriveForwardRight();
        lnStepCount++;
      }
      break;

    // ---- DRIVE: go straight a fixed burst then re-measure ----
    case LANE_DRIVE:
      if (lnStepCount >= LANE_DRIVE_STEPS) {
        lnState = LANE_MEASURE;
        lnMeasureStart = millis();
      } else {
        lnDriveForward();
        lnStepCount++;
      }
      break;
  }

  // Log state transitions
  if (lnState != prevState) {
    Serial.print("LN: -> ");
    switch (lnState) {
      case LANE_MEASURE:     Serial.println("MEASURE"); break;
      case LANE_CORRECT_CW:  Serial.println("CORRECT CW"); break;
      case LANE_CORRECT_CCW: Serial.println("CORRECT CCW"); break;
      case LANE_SHIFT_LEFT:  Serial.println("SHIFT L"); break;
      case LANE_SHIFT_RIGHT: Serial.println("SHIFT R"); break;
      case LANE_DRIVE:       Serial.println("DRIVE"); break;
    }
  }
}

// Read distance from a single ultrasonic sensor
float readDistanceSensor(int trigPin, int echoPin) {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo pulse with timeout
  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  
  if (duration == 0) {
    return MAX_DISTANCE;  // No echo received
  }
  
  // Calculate distance in cm
  float dist = (duration * SOUND_SPEED) / 2.0;
  return constrain(dist, 0, MAX_DISTANCE);
}

// Write to PCF8574T pin
void pcfWrite(uint8_t pin, bool state) {
  static uint8_t pcfState = 0xFF;  // Track all pin states
  if (state) {
    pcfState |= (1 << pin);   // Set bit high
  } else {
    pcfState &= ~(1 << pin);  // Set bit low
  }
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(pcfState);
  Wire.endTransmission();
}

// Read from PCF8574T pin (returns true if pin is HIGH)
bool pcfRead(uint8_t pin) {
  // Set pin high to read (PCF8574 uses quasi-bidirectional I/O)
  pcfWrite(pin, HIGH);
  delayMicroseconds(10);
  
  Wire.requestFrom(PCF8574_ADDR, (uint8_t)1);
  if (Wire.available()) {
    uint8_t state = Wire.read();
    return (state & (1 << pin)) != 0;
  }
  return true;  // Default high (no metal)
}

// Read distance using PCF8574T for trigger
float readDistancePCF(int pcfPin, int echoPin) {
  // Send trigger pulse via PCF8574T
  pcfWrite(pcfPin, LOW);
  delayMicroseconds(2);
  pcfWrite(pcfPin, HIGH);
  delayMicroseconds(10);
  pcfWrite(pcfPin, LOW);
  
  // Read echo pulse with timeout
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  if (duration == 0) {
    return MAX_DISTANCE;
  }
  
  float dist = (duration * SOUND_SPEED) / 2.0;
  return constrain(dist, 0, MAX_DISTANCE);
}

// Read inductive proximity sensor
void readInductiveSensor() {
  // NPN NO sensor: LOW when metal detected, HIGH when no metal
  metalDetected = !pcfRead(PCF_INDUCTIVE);
}

// Read limit switches (NO: LOW when pressed, HIGH when open)
void readLimitSwitches() {
  limitSwitch1 = !pcfRead(PCF_LIMIT_SW1);
  limitSwitch2 = !pcfRead(PCF_LIMIT_SW2);
  limitSwitch3 = !pcfRead(PCF_LIMIT_SW3);
}

// Read capacitive touch sensor (HIGH when touched)
void readCapTouch() {
  capTouched = pcfRead(PCF_CAP_TOUCH);
}

// Read all ultrasonic sensors
void readAllDistances() {
  // Read front sensor (direct GPIO) with raw pulse diagnostics
  digitalWrite(TRIG_PIN_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_FRONT, LOW);
  frontPulseDurationUs = pulseIn(ECHO_PIN_FRONT, HIGH, 30000);
  if (frontPulseDurationUs == 0) {
    distanceFront = MAX_DISTANCE;
  } else {
    distanceFront = constrain((frontPulseDurationUs * SOUND_SPEED) / 2.0, 0, MAX_DISTANCE);
  }
  delayMicroseconds(50);
  
  // Read left sensor (direct GPIO)
  distanceLeft = readDistanceSensor(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  delayMicroseconds(50);
  
  // Read right sensor (PCF8574T trigger)
  distanceRight = readDistancePCF(PCF_TRIG_RIGHT, ECHO_PIN_RIGHT);
  delayMicroseconds(50);
  
  // Read back sensor (PCF8574T trigger)
  distanceBack = readDistancePCF(PCF_TRIG_BACK, ECHO_PIN_BACK);
}

// Read Yahboom 8-channel line sensor via I2C
uint8_t readLineSensor() {
  Wire.beginTransmission(LINE_SENSOR_ADDR);
  Wire.write(0x00);  // Register to read
  Wire.endTransmission();
  
  Wire.requestFrom(LINE_SENSOR_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// HTML page with omni wheel robot controls
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Robot Controller</title>
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
    }
    body {
      font-family: Arial, sans-serif;
      background: #fff;
      min-height: 100vh;
      padding: 20px;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    h1 {
      font-size: 24px;
      margin-bottom: 10px;
      text-align: center;
      color: #000;
    }
    #status {
      background: #f0f0f0;
      padding: 10px 20px;
      border: 1px solid #ccc;
      border-radius: 5px;
      margin-bottom: 10px;
      font-size: 14px;
    }
    #distance {
      background: #e8f8e8;
      padding: 10px 20px;
      border: 1px solid #4a4;
      border-radius: 5px;
      margin-bottom: 10px;
      font-size: 16px;
      font-weight: bold;
      text-align: center;
    }
    #linesensor {
      background: #f8f8e8;
      padding: 10px 15px;
      border: 1px solid #aa4;
      border-radius: 5px;
      margin-bottom: 20px;
      display: flex;
      justify-content: center;
      gap: 4px;
    }
    .line-ch {
      width: 28px;
      height: 28px;
      border-radius: 4px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 11px;
      font-weight: bold;
    }
    .line-black {
      background: #222;
      color: #fff;
    }
    .line-white {
      background: #fff;
      color: #000;
      border: 1px solid #333;
    }
    #sensors {
      background: #e8f4f8;
      padding: 10px 20px;
      border: 1px solid #999;
      border-radius: 5px;
      margin-bottom: 20px;
      font-size: 13px;
      font-family: monospace;
    }
    .sensor-val {
      display: inline-block;
      padding: 2px 8px;
      margin: 0 5px;
      border-radius: 3px;
      font-weight: bold;
    }
    .sensor-black {
      background: #333;
      color: #fff;
    }
    .sensor-white {
      background: #fff;
      color: #000;
      border: 1px solid #333;
    }
    .container {
      width: 100%;
      max-width: 350px;
    }
    .dpad-container {
      position: relative;
      width: 280px;
      height: 280px;
      margin: 0 auto 30px;
    }
    .dpad-btn {
      position: absolute;
      width: 70px;
      height: 70px;
      border: 2px solid #000;
      background: #fff;
      font-size: 20px;
      font-weight: bold;
      cursor: pointer;
      transition: background 0.1s;
      user-select: none;
    }
    .dpad-btn:active {
      background: #ddd;
    }
    .dpad-up { top: 0; left: 50%; transform: translateX(-50%); }
    .dpad-down { bottom: 0; left: 50%; transform: translateX(-50%); }
    .dpad-left { left: 0; top: 50%; transform: translateY(-50%); }
    .dpad-right { right: 0; top: 50%; transform: translateY(-50%); }
    .dpad-up-left { top: 15px; left: 15px; width: 60px; height: 60px; font-size: 16px; }
    .dpad-up-right { top: 15px; right: 15px; width: 60px; height: 60px; font-size: 16px; }
    .dpad-down-left { bottom: 15px; left: 15px; width: 60px; height: 60px; font-size: 16px; }
    .dpad-down-right { bottom: 15px; right: 15px; width: 60px; height: 60px; font-size: 16px; }
    .stop-btn {
      width: 100%;
      padding: 15px;
      margin-top: 15px;
      border: 3px solid #000;
      background: #fff;
      font-size: 18px;
      font-weight: bold;
      cursor: pointer;
    }
    .stop-btn:active {
      background: #ddd;
    }
    .linefollow-btn {
      width: 100%;
      padding: 15px;
      margin-top: 10px;
      border: 3px solid #060;
      background: #efe;
      font-size: 18px;
      font-weight: bold;
      cursor: pointer;
    }
    .linefollow-btn.active {
      background: #4a4;
      color: #fff;
    }
    .linefollow-btn:active {
      background: #8c8;
    }
    .lanefollow-btn {
      width: 100%;
      padding: 15px;
      margin-top: 10px;
      border: 3px solid #006;
      background: #eef;
      font-size: 18px;
      font-weight: bold;
      cursor: pointer;
    }
    .lanefollow-btn.active {
      background: #44a;
      color: #fff;
    }
    .lanefollow-btn:active {
      background: #88c;
    }
    .autopickup-btn {
      width: 100%;
      padding: 15px;
      margin-top: 10px;
      border: 3px solid #650;
      background: #f7efe4;
      font-size: 18px;
      font-weight: bold;
      cursor: pointer;
    }
    .autopickup-btn.active {
      background: #b56a00;
      color: #fff;
    }
    .autopickup-btn:active {
      background: #d08d32;
    }
    .control-group {
      background: #f9f9f9;
      border: 1px solid #ccc;
      padding: 15px;
      margin-bottom: 15px;
    }
    .control-group h2 {
      font-size: 16px;
      margin-bottom: 10px;
      color: #000;
    }
    .servo-control {
      display: flex;
      align-items: center;
      gap: 10px;
    }
    .servo-slider {
      flex: 1;
      height: 30px;
    }
    .servo-value {
      min-width: 45px;
      text-align: center;
      font-weight: bold;
      font-size: 14px;
    }
    #limits {
      background: #f0e8f8;
      padding: 8px 20px;
      border: 1px solid #94a;
      border-radius: 5px;
      margin-bottom: 20px;
      font-size: 14px;
      font-weight: bold;
      text-align: center;
    }
    .lim-ind {
      display: inline-block;
      padding: 3px 12px;
      margin: 0 8px;
      border-radius: 4px;
    }
    .lim-off { background: #ddd; color: #666; }
    .lim-on { background: #c33; color: #fff; }
  </style>
</head>
<body>
  <h1>Robot Controller</h1>
  <div id="status">Ready</div>
  <div id="distance">F:-- L:-- R:-- B:-- cm</div>
  <div id="linesensor">
    <div class="line-ch line-white" id="L1">1</div>
    <div class="line-ch line-white" id="L2">2</div>
    <div class="line-ch line-white" id="L3">3</div>
    <div class="line-ch line-white" id="L4">4</div>
    <div class="line-ch line-white" id="L5">5</div>
    <div class="line-ch line-white" id="L6">6</div>
    <div class="line-ch line-white" id="L7">7</div>
    <div class="line-ch line-white" id="L8">8</div>
  </div>
  <div id="limits">
    <span class="lim-ind lim-off" id="lim1">SW1: OFF</span>
    <span class="lim-ind lim-off" id="lim2">SW2: OFF</span>
    <span class="lim-ind lim-off" id="lim3">SW3: OFF</span>
    <span class="lim-ind lim-off" id="capInd">CAP: OFF</span>
  </div>
  
  <div class="container">
    <div class="dpad-container">
      <button class="dpad-btn dpad-up" onclick="move('forward')">▲</button>
      <button class="dpad-btn dpad-left" onclick="move('left')">◄</button>
      <button class="dpad-btn dpad-right" onclick="move('right')">►</button>
      <button class="dpad-btn dpad-down" onclick="move('backward')">▼</button>
      <button class="dpad-btn dpad-up-left" onclick="move('forward-left')">↖</button>
      <button class="dpad-btn dpad-up-right" onclick="move('forward-right')">↗</button>
      <button class="dpad-btn dpad-down-left" onclick="move('backward-left')">↙</button>
      <button class="dpad-btn dpad-down-right" onclick="move('backward-right')">↘</button>
    </div>
    
    <button class="stop-btn" onclick="stopMove()">STOP</button>
    <button class="linefollow-btn" id="lfBtn" onclick="toggleLineFollow()">LINE FOLLOW: OFF</button>
    <button class="lanefollow-btn" id="lnBtn" onclick="toggleLaneFollow()">LANE FOLLOW: OFF</button>
    <button class="autopickup-btn" id="pickupBtn" onclick="toggleAutoPickup()">AUTO PICKUP: OFF</button>
    
    <div class="control-group">
      <h2>Servo 1</h2>
      <div class="servo-control">
        <input type="range" class="servo-slider" id="servo0" min="0" max="180" value="90" oninput="updateLabel(0, this.value)">
        <span class="servo-value" id="val0">90°</span>
      </div>
    </div>
    
    <div class="control-group">
      <h2>Servo 2</h2>
      <div class="servo-control">
        <input type="range" class="servo-slider" id="servo1" min="0" max="90" value="0" oninput="updateLabel(1, this.value)">
        <span class="servo-value" id="val1">0°</span>
      </div>
    </div>
    
    <div class="control-group">
      <h2>Servo 3</h2>
      <div class="servo-control">
        <input type="range" class="servo-slider" id="servo2" min="0" max="180" value="135" oninput="updateLabel(2, this.value)">
        <span class="servo-value" id="val2">135°</span>
      </div>
    </div>
    
    <div class="control-group">
      <h2>Servo 4</h2>
      <div class="servo-control">
        <input type="range" class="servo-slider" id="servo3" min="0" max="165" value="90" oninput="updateLabel(3, this.value)">
        <span class="servo-value" id="val3">90°</span>
      </div>
    </div>
    
    <div class="control-group">
      <h2>Servo 5</h2>
      <div class="servo-control">
        <input type="range" class="servo-slider" id="servo4" min="135" max="250" value="180" oninput="updateLabel(4, this.value)">
        <span class="servo-value" id="val4">180°</span>
      </div>
    </div>
    
    <div class="control-group">
      <h2>Arm Position (IK)</h2>
      <div style="display:flex;gap:8px;margin-bottom:8px;flex-wrap:wrap;align-items:center;">
        <label>X<input type="number" id="ikX" value="100" style="width:55px;margin-left:4px;"></label>
        <label>Y<input type="number" id="ikY" value="200" style="width:55px;margin-left:4px;"></label>
        <label>Z<input type="number" id="ikZ" value="0" style="width:55px;margin-left:4px;"></label>
        <span style="font-size:12px;color:#666;">mm</span>
      </div>
      <button style="width:100%;padding:12px;font-size:16px;font-weight:bold;background:#2196F3;color:#fff;border:none;border-radius:5px;cursor:pointer;" onclick="moveIK()">MOVE TO</button>
      <div id="ikStatus" style="margin-top:6px;font-size:13px;color:#555;"></div>
    </div>
  </div>

  <script>
    let moving = false;
    function updateStatus(msg) {
      document.getElementById('status').textContent = msg;
    }
    
    async function move(direction) {
      if (moving) return;
      moving = true;
      updateStatus('Moving ' + direction + '...');
      try {
        const response = await fetch('/move?dir=' + direction);
        const data = await response.text();
        updateStatus(data);
        document.getElementById('lfBtn').className = 'linefollow-btn';
        document.getElementById('lfBtn').textContent = 'LINE FOLLOW: OFF';
        document.getElementById('lnBtn').className = 'lanefollow-btn';
        document.getElementById('lnBtn').textContent = 'LANE FOLLOW: OFF';
        document.getElementById('pickupBtn').className = 'autopickup-btn';
        document.getElementById('pickupBtn').textContent = 'AUTO PICKUP: OFF';
      } catch (e) {
        updateStatus('Error: ' + e.message);
      }
      moving = false;
    }
    
    async function stopMove() {
      updateStatus('Stopping...');
      try {
        const response = await fetch('/stop');
        const data = await response.text();
        updateStatus(data);
        document.getElementById('lfBtn').className = 'linefollow-btn';
        document.getElementById('lfBtn').textContent = 'LINE FOLLOW: OFF';
        document.getElementById('lnBtn').className = 'lanefollow-btn';
        document.getElementById('lnBtn').textContent = 'LANE FOLLOW: OFF';
        document.getElementById('pickupBtn').className = 'autopickup-btn';
        document.getElementById('pickupBtn').textContent = 'AUTO PICKUP: OFF';
      } catch (e) {
        updateStatus('Error: ' + e.message);
      }
    }
    
    async function toggleLineFollow() {
      try {
        const response = await fetch('/linefollow');
        const data = await response.text();
        updateStatus(data);
        const btn = document.getElementById('lfBtn');
        if (data.includes('ON')) {
          btn.className = 'linefollow-btn active';
          btn.textContent = 'LINE FOLLOW: ON';
          document.getElementById('lnBtn').className = 'lanefollow-btn';
          document.getElementById('lnBtn').textContent = 'LANE FOLLOW: OFF';
          document.getElementById('pickupBtn').className = 'autopickup-btn';
          document.getElementById('pickupBtn').textContent = 'AUTO PICKUP: OFF';
        } else {
          btn.className = 'linefollow-btn';
          btn.textContent = 'LINE FOLLOW: OFF';
        }
      } catch (e) {
        updateStatus('Error: ' + e.message);
      }
    }
    
    async function toggleLaneFollow() {
      try {
        const response = await fetch('/lanefollow');
        const data = await response.text();
        updateStatus(data);
        const btn = document.getElementById('lnBtn');
        if (data.includes('ON')) {
          btn.className = 'lanefollow-btn active';
          btn.textContent = 'LANE FOLLOW: ON';
          document.getElementById('lfBtn').className = 'linefollow-btn';
          document.getElementById('lfBtn').textContent = 'LINE FOLLOW: OFF';
          document.getElementById('pickupBtn').className = 'autopickup-btn';
          document.getElementById('pickupBtn').textContent = 'AUTO PICKUP: OFF';
        } else {
          btn.className = 'lanefollow-btn';
          btn.textContent = 'LANE FOLLOW: OFF';
        }
      } catch (e) {
        updateStatus('Error: ' + e.message);
      }
    }

    async function toggleAutoPickup() {
      try {
        const response = await fetch('/autopickup');
        const data = await response.text();
        updateStatus(data);
        const btn = document.getElementById('pickupBtn');
        if (data.includes('ON')) {
          btn.className = 'autopickup-btn active';
          btn.textContent = 'AUTO PICKUP: ON';
          document.getElementById('lfBtn').className = 'linefollow-btn';
          document.getElementById('lfBtn').textContent = 'LINE FOLLOW: OFF';
          document.getElementById('lnBtn').className = 'lanefollow-btn';
          document.getElementById('lnBtn').textContent = 'LANE FOLLOW: OFF';
        } else {
          btn.className = 'autopickup-btn';
          btn.textContent = 'AUTO PICKUP: OFF';
        }
      } catch (e) {
        updateStatus('Error: ' + e.message);
      }
    }
    
    function updateLabel(channel, angle) {
      document.getElementById('val' + channel).textContent = angle + '°';
    }
    
    // Send servo command only on actual touch/mouse release
    document.querySelectorAll('.servo-slider').forEach(slider => {
      function sendOnRelease() {
        const ch = parseInt(slider.id.replace('servo', ''));
        setServo(ch, slider.value);
      }
      slider.addEventListener('mouseup', sendOnRelease);
      slider.addEventListener('touchend', sendOnRelease);
    });
    
    async function setServo(channel, angle) {
      document.getElementById('val' + channel).textContent = angle + '°';
      try {
        await fetch('/servo?ch=' + channel + '&angle=' + angle);
        updateStatus('Servo ' + (channel+1) + ': ' + angle + '°');
      } catch (e) {
        updateStatus('Error: ' + e.message);
      }
    }
    
    async function moveIK() {
      const x = document.getElementById('ikX').value;
      const y = document.getElementById('ikY').value;
      const z = document.getElementById('ikZ').value;
      document.getElementById('ikStatus').textContent = 'Moving...';
      try {
        const response = await fetch('/ik?x=' + x + '&y=' + y + '&z=' + z);
        const data = await response.text();
        if (response.ok) {
          const a = data.split(',');
          document.getElementById('servo0').value = a[0];
          document.getElementById('val0').textContent = a[0] + '\u00b0';
          document.getElementById('servo1').value = a[1];
          document.getElementById('val1').textContent = a[1] + '\u00b0';
          document.getElementById('servo2').value = a[2];
          document.getElementById('val2').textContent = a[2] + '\u00b0';
          if (a.length > 3) {
            document.getElementById('servo3').value = a[3];
            document.getElementById('val3').textContent = a[3] + '\u00b0';
          }
          document.getElementById('ikStatus').textContent = 'S1=' + a[0] + ' S2=' + a[1] + ' S3=' + a[2] + ' S4=' + (a[3]||'');
        } else {
          document.getElementById('ikStatus').textContent = data;
        }
      } catch (e) {
        document.getElementById('ikStatus').textContent = 'Error: ' + e.message;
      }
    }
    
    async function updateDistance() {
      try {
        const response = await fetch('/distance');
        const dist = await response.text();
        document.getElementById('distance').textContent = dist + ' cm';
      } catch (e) {
        document.getElementById('distance').textContent = 'Distance: Error';
      }
    }
    
    async function updateLineSensor() {
      try {
        const response = await fetch('/linesensor');
        const data = parseInt(await response.text());
        for (let i = 0; i < 8; i++) {
          const el = document.getElementById('L' + (i + 1));
          const isBlack = (data >> i) & 1;
          el.className = 'line-ch ' + (isBlack ? 'line-black' : 'line-white');
        }
      } catch (e) {
        console.error('Line sensor error:', e);
      }
    }
    
    async function updateLimits() {
      try {
        const response = await fetch('/limits');
        const parts = (await response.text()).split(',');
        ['lim1','lim2','lim3'].forEach((id, i) => {
          const el = document.getElementById(id);
          const on = parts[i] === '1';
          el.className = 'lim-ind ' + (on ? 'lim-on' : 'lim-off');
          el.textContent = 'SW' + (i+1) + ': ' + (on ? 'ON' : 'OFF');
        });
      } catch (e) {}
    }
    
    async function updateCapTouch() {
      try {
        const response = await fetch('/captouch');
        const on = (await response.text()) === '1';
        const el = document.getElementById('capInd');
        el.className = 'lim-ind ' + (on ? 'lim-on' : 'lim-off');
        el.textContent = 'CAP: ' + (on ? 'ON' : 'OFF');
      } catch (e) {}
    }
    
    // Update sensors periodically
    setInterval(updateDistance, 200);
    setInterval(updateLineSensor, 100);
    setInterval(updateLimits, 200);
    setInterval(updateCapTouch, 200);
    updateDistance();
    updateLineSensor();
    updateLimits();
    updateCapTouch();
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

// Handle robot movement for omni wheels
void handleMove() {
  if (!server.hasArg("dir")) {
    server.send(400, "text/plain", "Missing direction");
    return;
  }
  
  // Stop any autonomous modes
  lineFollowMode = false;
  laneFollowMode = false;
  autoPickupMode = false;
  resetAutoPickupSequence();
  
  currentDirection = server.arg("dir");
  applyMotorDirection();
  
  timerStepping = true;
  isMoving = true;
  updateDisplay();
  server.send(200, "text/plain", "Moving " + currentDirection);
}

// Apply motor directions and enable flags for timer-based stepping
void applyMotorDirection() {
  // Motor control: set reverseMotor (true=LOW/false=HIGH) and activeMotor (true=ON/false=OFF)
  // Motor layout: 0=M1, 1=M2, 2=M3, 3=M4
  
  bool reverseMotor[4] = {false, false, false, false};
  bool activeMotor[4] = {true, true, true, true};
  
  // ===== FORWARD =====
  if (currentDirection == "forward") {
    reverseMotor[0] = true;   // M1: reversed
    reverseMotor[1] = false;  // M2: normal
    reverseMotor[2] = true;   // M3: reversed
    reverseMotor[3] = true;   // M4: reversed
  }
  
  // ===== BACKWARD =====
  else if (currentDirection == "backward") {
    reverseMotor[0] = false;  // M1: normal
    reverseMotor[1] = true;   // M2: reversed
    reverseMotor[2] = false;  // M3: normal
    reverseMotor[3] = false;  // M4: normal
  }
  
  // ===== RIGHT =====
  else if (currentDirection == "right") {
    reverseMotor[0] = true;
    reverseMotor[1] = true;
    reverseMotor[2] = false;
    reverseMotor[3] = true;
  }
  
  // ===== LEFT =====
  else if (currentDirection == "left") {
    reverseMotor[0] = false;
    reverseMotor[1] = false;
    reverseMotor[2] = true;
    reverseMotor[3] = false;
  }
  
  // ===== FORWARD-RIGHT =====
  else if (currentDirection == "forward-right") {
    reverseMotor[0] = true;
    reverseMotor[1] = false;
    reverseMotor[2] = true;
    reverseMotor[3] = true;
    activeMotor[0] = false;
    activeMotor[2] = false;
  }
  
  // ===== FORWARD-LEFT =====
  else if (currentDirection == "forward-left") {
    reverseMotor[0] = true;
    reverseMotor[1] = false;
    reverseMotor[2] = true;
    reverseMotor[3] = false;
    activeMotor[1] = false;
    activeMotor[3] = false;
  }
  
  // ===== BACKWARD-LEFT =====
  else if (currentDirection == "backward-left") {
    reverseMotor[0] = false;
    reverseMotor[1] = true;
    reverseMotor[2] = false;
    reverseMotor[3] = false;
    activeMotor[0] = false;
    activeMotor[2] = false;
  }
  
  // ===== BACKWARD-RIGHT =====
  else if (currentDirection == "backward-right") {
    reverseMotor[0] = false;
    reverseMotor[1] = false;
    reverseMotor[2] = false;
    reverseMotor[3] = false;
    activeMotor[1] = false;
    activeMotor[3] = false;
  }
  
  // Enable all motors and set direction (keeps inactive motors from sliding)
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], reverseMotor[i] ? LOW : HIGH);
  }
  
  // Update volatile active flags for timer ISR
  for (int i = 0; i < 4; i++) {
    stepMotorActive[i] = activeMotor[i];
  }
}

// Stop all motors
void handleStop() {
  timerStepping = false;
  stepPhase = false;
  isMoving = false;
  lineFollowMode = false;
  laneFollowMode = false;
  autoPickupMode = false;
  resetAutoPickupSequence();
  currentDirection = "";
  // Disable all motors
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], HIGH);
    digitalWrite(STEP_PINS[i], LOW);
    motorEnabled[i] = false;
  }
  
  updateDisplay();
  server.send(200, "text/plain", "Stopped");
}

// Handle servo control via PCA9685
void handleServo() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "Missing ch or angle");
    return;
  }
  
  int channel = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();
  
  if (channel < 0 || channel >= NUM_SERVOS) {
    server.send(400, "text/plain", "Invalid channel");
    return;
  }
  
  angle = constrain(angle, SERVO_MIN[channel], SERVO_MAX[channel]);
  setServoAngle(channel, angle);  // This updates servoPositions internally
  
  Serial.print("Servo ");
  Serial.print(channel + 1);
  Serial.print(": ");
  Serial.print(angle);
  Serial.println("°");
  
  updateDisplay();
  server.send(200, "text/plain", "OK");
}

// Handle distance request
void handleDistance() {
  String response = "F:" + String(distanceFront, 1) + " L:" + String(distanceLeft, 1) + " R:" + String(distanceRight, 1) + " B:" + String(distanceBack, 1) + " Pf:" + String(frontPulseDurationUs) + "us";
  server.send(200, "text/plain", response);
}

// Handle line sensor request
void handleLineSensor() {
  server.send(200, "text/plain", String(lineSensorData));
}

// Handle limit switch request
void handleLimitSwitches() {
  String response = String(limitSwitch1 ? 1 : 0) + "," + String(limitSwitch2 ? 1 : 0) + "," + String(limitSwitch3 ? 1 : 0);
  server.send(200, "text/plain", response);
}

// Handle capacitive touch request
void handleCapTouch() {
  server.send(200, "text/plain", String(capTouched ? 1 : 0));
}

// Handle QR code request
void handleQRCode() {
  String response = lastQRCode;
  if (lastQRCode.length() == 0) {
    response = "No code scanned";
  }
  server.send(200, "text/plain", response);
}

// Handle inverse kinematics move
void handleIK() {
  if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("z")) {
    server.send(400, "text/plain", "Missing x, y, or z");
    return;
  }
  
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  float z = server.arg("z").toFloat();
  
  int s1, s2, s3, s4;
  int s4orig = servoPositions[3];
  if (!solveIKAuto(x, y, z, s4orig, s1, s2, s3, s4)) {
    Serial.print("IK: Unreachable ("); Serial.print(x,1);
    Serial.print(","); Serial.print(y,1);
    Serial.print(","); Serial.print(z,1); Serial.println(")");
    server.send(400, "text/plain", "Position unreachable");
    return;
  }
  
  setServoAngle(3, s4);  // S4 first (may have been auto-adjusted)
  setServoAngle(0, s1);
  setServoAngle(1, s2);
  setServoAngle(2, s3);
  
  Serial.print("IK: ("); Serial.print(x,1); Serial.print(",");
  Serial.print(y,1); Serial.print(","); Serial.print(z,1);
  Serial.print(") -> S1="); Serial.print(s1);
  Serial.print(" S2="); Serial.print(s2);
  Serial.print(" S3="); Serial.print(s3);
  if (s4 != s4orig) { Serial.print(" S4="); Serial.print(s4); Serial.print(" (auto)"); }
  Serial.println();
  
  updateDisplay();
  server.send(200, "text/plain", String(s1) + "," + String(s2) + "," + String(s3) + "," + String(s4));
}

// Handle auto pickup toggle
void handleAutoPickup() {
  autoPickupMode = !autoPickupMode;
  resetAutoPickupSequence();
  pickupCooldownUntil = 0;

  if (autoPickupMode) {
    // Mutually exclusive with wheel-based autonomous/manual movement
    timerStepping = false;
    stepPhase = false;
    isMoving = false;
    lineFollowMode = false;
    laneFollowMode = false;
    currentDirection = "pickup";

    for (int i = 0; i < 4; i++) {
      digitalWrite(EN_PINS[i], HIGH);
      digitalWrite(STEP_PINS[i], LOW);
    }

    Serial.println("Auto pickup: ON");
    server.send(200, "text/plain", "Auto Pickup: ON");
  } else {
    currentDirection = "";
    Serial.println("Auto pickup: OFF");
    server.send(200, "text/plain", "Auto Pickup: OFF");
  }
  updateDisplay();
}

// Handle lane follow toggle
void handleLaneFollow() {
  laneFollowMode = !laneFollowMode;
  
  if (laneFollowMode) {
    timerStepping = false;
    isMoving = false;
    lineFollowMode = false;  // Mutually exclusive
    autoPickupMode = false;  // Mutually exclusive
    resetAutoPickupSequence();
    currentDirection = "lanefollow";
    lnState = LANE_MEASURE;
    lnMeasureStart = millis();
    lnStepCount = 0;
    // Enable motors
    for (int i = 0; i < 4; i++) {
      digitalWrite(EN_PINS[i], LOW);
    }
    Serial.println("Lane following: ON (measuring...)");
    server.send(200, "text/plain", "Lane Follow: ON");
  } else {
    // Disable motors
    for (int i = 0; i < 4; i++) {
      digitalWrite(EN_PINS[i], HIGH);
      digitalWrite(STEP_PINS[i], LOW);
    }
    currentDirection = "";
    Serial.println("Lane following: OFF");
    server.send(200, "text/plain", "Lane Follow: OFF");
  }
  updateDisplay();
}

// Handle line follow toggle
void handleLineFollow() {
  lineFollowMode = !lineFollowMode;
  
  if (lineFollowMode) {
    timerStepping = false;
    isMoving = false;
    laneFollowMode = false;  // Mutually exclusive
    autoPickupMode = false;  // Mutually exclusive
    resetAutoPickupSequence();
    currentDirection = "linefollow";
    lfState = LF_FORWARD;
    lfTurnStepCount = 0;
    // Enable motors
    for (int i = 0; i < 4; i++) {
      digitalWrite(EN_PINS[i], LOW);
    }
    Serial.println("Line following: ON");
    server.send(200, "text/plain", "Line Follow: ON");
  } else {
    // Disable motors
    for (int i = 0; i < 4; i++) {
      digitalWrite(EN_PINS[i], HIGH);
      digitalWrite(STEP_PINS[i], LOW);
    }
    currentDirection = "";
    Serial.println("Line following: OFF");
    server.send(200, "text/plain", "Line Follow: OFF");
  }
  updateDisplay();
}

// Process serial commands
void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();
  
  Serial.print("Command: ");
  Serial.println(command);
  
  // Lane follow command
  if (command == "ln") {
    laneFollowMode = !laneFollowMode;
    if (laneFollowMode) {
      timerStepping = false;
      isMoving = false;
      lineFollowMode = false;
      autoPickupMode = false;
      resetAutoPickupSequence();
      currentDirection = "lanefollow";
      lnState = LANE_MEASURE;
      lnMeasureStart = millis();
      lnStepCount = 0;
      for (int i = 0; i < 4; i++) {
        digitalWrite(EN_PINS[i], LOW);
      }
      Serial.println("Lane following: ON (measuring...)");
    } else {
      for (int i = 0; i < 4; i++) {
        digitalWrite(EN_PINS[i], HIGH);
        digitalWrite(STEP_PINS[i], LOW);
      }
      currentDirection = "";
      Serial.println("Lane following: OFF");
    }
    updateDisplay();
    return;
  }

  // Auto pickup command
  if (command == "pickup") {
    autoPickupMode = !autoPickupMode;
    resetAutoPickupSequence();
    pickupCooldownUntil = 0;

    if (autoPickupMode) {
      timerStepping = false;
      stepPhase = false;
      isMoving = false;
      lineFollowMode = false;
      laneFollowMode = false;
      currentDirection = "pickup";
      for (int i = 0; i < 4; i++) {
        digitalWrite(EN_PINS[i], HIGH);
        digitalWrite(STEP_PINS[i], LOW);
      }
      Serial.println("Auto pickup: ON");
    } else {
      currentDirection = "";
      Serial.println("Auto pickup: OFF");
    }
    updateDisplay();
    return;
  }
  
  // Line follow command
  if (command == "lf") {
    lineFollowMode = !lineFollowMode;
    if (lineFollowMode) {
      timerStepping = false;
      isMoving = false;
      laneFollowMode = false;  // Mutually exclusive
      autoPickupMode = false;
      resetAutoPickupSequence();
      currentDirection = "linefollow";
      lfState = LF_FORWARD;
      lfTurnStepCount = 0;
      for (int i = 0; i < 4; i++) {
        digitalWrite(EN_PINS[i], LOW);
      }
      Serial.println("Line following: ON");
    } else {
      for (int i = 0; i < 4; i++) {
        digitalWrite(EN_PINS[i], HIGH);
        digitalWrite(STEP_PINS[i], LOW);
      }
      currentDirection = "";
      Serial.println("Line following: OFF");
    }
    updateDisplay();
    return;
  }
  
  // Stop command
  if (command == "stop") {
    timerStepping = false;
    stepPhase = false;
    lineFollowMode = false;
    laneFollowMode = false;
    autoPickupMode = false;
    resetAutoPickupSequence();
    serialSensorOutput = false;
    isMoving = false;
    currentDirection = "";
    for (int i = 0; i < 4; i++) {
      digitalWrite(EN_PINS[i], HIGH);
      digitalWrite(STEP_PINS[i], LOW);
    }
    Serial.println("Stopped");
    updateDisplay();
    return;
  }
  
  // Sensor reading command (one-time)
  if (command == "sensor") {
    printSensorData();
    return;
  }
  
  // Distance reading command
  if (command == "dist") {
    readAllDistances();
    Serial.print("Distance Front: ");
    Serial.print(distanceFront, 1);
    Serial.println(" cm");
    Serial.print("Front pulse:    ");
    Serial.print(frontPulseDurationUs);
    Serial.println(" us");
    Serial.print("Front logic:    TRIG=");
    Serial.print(digitalRead(TRIG_PIN_FRONT));
    Serial.print(" ECHO=");
    Serial.println(digitalRead(ECHO_PIN_FRONT));
    Serial.print("Distance Left:  ");
    Serial.print(distanceLeft, 1);
    Serial.println(" cm");
    Serial.print("Distance Right: ");
    Serial.print(distanceRight, 1);
    Serial.println(" cm");
    Serial.print("Distance Back:  ");
    Serial.print(distanceBack, 1);
    Serial.println(" cm");
    return;
  }
  
  // QR code reading command
  if (command == "qr") {
    if (lastQRCode.length() > 0) {
      Serial.print("Last QR Code: ");
      Serial.println(lastQRCode);
    } else {
      Serial.println("No QR code scanned yet");
    }
    return;
  }
  
  // QR scanner trigger/wake command
  if (command == "qrtrig") {
    // Send trigger commands to wake up GM805
    Serial2.write(0x16);  // Start scanning command
    Serial2.write(0x54);  // Trigger
    Serial2.write(0x0D);  // Carriage return
    Serial.println("QR Scanner trigger sent");
    return;
  }
  
  // Monitor toggle command (continuous output)
  if (command == "mon") {
    serialSensorOutput = !serialSensorOutput;
    if (serialSensorOutput) {
      Serial.println("Sensor monitoring: ON (type 'mon' or 'stop' to disable)");
    } else {
      Serial.println("Sensor monitoring: OFF");
    }
    return;
  }
  
  // I2C scanner command
  if (command == "scan") {
    Serial.println("Scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("  Found device at 0x");
        if (addr < 16) Serial.print("0");
        Serial.println(addr, HEX);
        found++;
      }
    }
    if (found == 0) {
      Serial.println("  No devices found!");
    } else {
      Serial.print("  Total: ");
      Serial.print(found);
      Serial.println(" device(s)");
    }
    return;
  }
  
  // Inductive sensor reading command
  if (command == "metal") {
    Serial.print("Metal Detected: ");
    Serial.println(metalDetected ? "YES" : "NO");
    return;
  }
  
  // Limit switch reading command
  if (command == "limit") {
    Serial.print("Limit Switch 1: ");
    Serial.println(limitSwitch1 ? "PRESSED" : "OPEN");
    Serial.print("Limit Switch 2: ");
    Serial.println(limitSwitch2 ? "PRESSED" : "OPEN");
    Serial.print("Limit Switch 3: ");
    Serial.println(limitSwitch3 ? "PRESSED" : "OPEN");
    return;
  }
  
  // Capacitive touch reading command
  if (command == "cap") {
    Serial.print("Capacitive Touch: ");
    Serial.println(capTouched ? "TOUCHED" : "NOT TOUCHED");
    return;
  }
  
  // Inverse kinematics command: ik x y z
  if (command.startsWith("ik ")) {
    int idx1 = command.indexOf(' ', 3);
    int idx2 = (idx1 > 0) ? command.indexOf(' ', idx1 + 1) : -1;
    if (idx1 > 0 && idx2 > 0) {
      float x = command.substring(3, idx1).toFloat();
      float y = command.substring(idx1 + 1, idx2).toFloat();
      float z = command.substring(idx2 + 1).toFloat();
      int s1, s2, s3, s4;
      int s4orig = servoPositions[3];
      if (solveIKAuto(x, y, z, s4orig, s1, s2, s3, s4)) {
        setServoAngle(3, s4);
        setServoAngle(0, s1);
        setServoAngle(1, s2);
        setServoAngle(2, s3);
        Serial.print("IK: S1="); Serial.print(s1);
        Serial.print(" S2="); Serial.print(s2);
        Serial.print(" S3="); Serial.print(s3);
        if (s4 != s4orig) { Serial.print(" S4="); Serial.print(s4); Serial.print("(auto)"); }
        Serial.println();
      } else {
        Serial.println("IK: Position unreachable");
      }
      updateDisplay();
      return;
    }
    Serial.println("Usage: ik <x> <y> <z> (mm)");
    return;
  }
  
  // Parse servo command: s1 90, s2 45, etc.
  if (command.startsWith("s") && command.length() >= 3) {
    int servoNum = command.charAt(1) - '0';  // Get servo number (1-5)
    int spaceIdx = command.indexOf(' ');
    
    if (servoNum >= 1 && servoNum <= NUM_SERVOS && spaceIdx > 0) {
      int angle = command.substring(spaceIdx + 1).toInt();
      int channel = servoNum - 1;
      angle = constrain(angle, SERVO_MIN[channel], SERVO_MAX[channel]);
      
      setServoAngle(channel, angle);  // This updates servoPositions internally
      
      Serial.print("Servo ");
      Serial.print(servoNum);
      Serial.print(": ");
      Serial.print(angle);
      Serial.println("°");
      updateDisplay();
      return;
    }
  }
  
  Serial.println("Invalid. Commands: lf, ln, pickup, stop, sensor, mon, s<1-5> <angle>");
}

// Update OLED display with current status
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: WiFi info
  display.setCursor(0, 0);
  display.print("WiFi: ");
  display.println(ap_ssid);
  
  // Line 2: IP address
  display.setCursor(0, 10);
  display.print("IP: ");
  display.println(WiFi.softAPIP());
  
  // Line 3: Movement status
  display.setCursor(0, 22);
  if (lineFollowMode) {
    display.print("LF:");
    switch(lfState) {
      case LF_FORWARD:       display.println("FORWARD"); break;
      case LF_CREEP_RIGHT:   display.println("CREEP R"); break;
      case LF_CREEP_LEFT:    display.println("CREEP L"); break;
      case LF_TURN_RIGHT_90: display.println("TURN R");  break;
      case LF_TURN_LEFT_90:  display.println("TURN L");  break;
      case LF_SEEK:          display.println("SEEK");    break;
    }
  } else if (laneFollowMode) {
    display.print("LN:");
    switch(lnState) {
      case LANE_MEASURE:     display.println("MEASURE"); break;
      case LANE_CORRECT_CW:  display.println("CORR CW"); break;
      case LANE_CORRECT_CCW: display.println("CORR CCW"); break;
      case LANE_SHIFT_LEFT:  display.println("SHIFT L"); break;
      case LANE_SHIFT_RIGHT: display.println("SHIFT R"); break;
      case LANE_DRIVE:       display.println("DRIVE"); break;
    }
  } else if (autoPickupMode) {
    display.println("Mode: PICKUP");
  } else if (isMoving && currentDirection.length() > 0) {
    display.print("Move: ");
    display.println(currentDirection);
  } else {
    display.println("Mode: Stopped");
  }
  
  // Line 4: Distances (F/L/R)
  display.setCursor(0, 34);
  display.print("D:");
  display.print((int)distanceFront);
  display.print(" ");
  display.print((int)distanceLeft);
  display.print(" ");
  display.print((int)distanceRight);
  display.print(" ");
  display.print((int)distanceBack);
  
  // Line 5: Line sensor (8 channels as visual boxes)
  display.setCursor(0, 46);
  display.print("Line:");
  for (int i = 0; i < 8; i++) {
    if ((lineSensorData >> i) & 1) {
      display.fillRect(36 + i * 11, 46, 9, 8, SSD1306_WHITE);
    } else {
      display.drawRect(36 + i * 11, 46, 9, 8, SSD1306_WHITE);
    }
  }
  
  // Line 6: Servo positions (compact)
  display.setCursor(0, 56);
  display.print(servoPositions[0]);
  display.print(" ");
  display.print(servoPositions[1]);
  display.print(" ");
  display.print(servoPositions[2]);
  display.print(" ");
  display.print(servoPositions[3]);
  display.print(" ");
  display.println(servoPositions[4]);
  
  display.display();
}

