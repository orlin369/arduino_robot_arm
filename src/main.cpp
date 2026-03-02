#include <Arduino.h>
#if defined(TARGET_ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif

// PWM pin assignments — one MG-996R servo per joint
// Arduino Sensor Shield labels are kept as comments for cross-reference.
#if defined(TARGET_ESP32)
  #define PIN_BASE         25  // D3  → GPIO25
  #define PIN_SHOULDER     16  // D5  → GPIO16
  #define PIN_ELBOW        27  // D6  → GPIO27
  #define PIN_WRIST_PITCH  13  // D9  → GPIO13
  #define PIN_WRIST_ROLL    5  // D10 → GPIO5
  #define PIN_GRIPPER      23  // D11 → GPIO23
#else  // Arduino UNO
  #define PIN_BASE          3  // D3
  #define PIN_SHOULDER      5  // D5
  #define PIN_ELBOW         6  // D6
  #define PIN_WRIST_PITCH   9  // D9
  #define PIN_WRIST_ROLL   10  // D10
  #define PIN_GRIPPER      11  // D11
#endif

// Servo objects — one per joint
Servo base;
Servo shoulder;
Servo elbow;
Servo wristPitch;
Servo wristRoll;
Servo gripper;

// ─────────────────────────────────────────────────────────────────────────────
// SERVO_CENTER_TEST — bare-metal smoke test: attach servos and drive to center
// (1500 μs). No TCM, no serial.
//   All servos, full ESP32Servo init : pio run -e esp32-test  -t upload
//   Base only,  full ESP32Servo init : pio run -e esp32-test1 -t upload
//   Base only,  raw attach (no alloc): pio run -e esp32-test2 -t upload
// ─────────────────────────────────────────────────────────────────────────────
#if defined(SERVO_CENTER_TEST)

void setup() {
#if defined(SERVO_BARE_TEST)
  // Sweep test: 1000 → 2000 → 1500 μs so any movement is clearly visible.
  // Serial prints confirm the code is running.
  Serial.begin(115200);
  Serial.println("SERVO_BARE_TEST start");
  base.attach(PIN_BASE);
  Serial.print("attached pin "); Serial.println(PIN_BASE);
  delay(500);
  Serial.println("-> 1000 us (full CCW)");
  base.writeMicroseconds(1000);
  delay(2000);
  Serial.println("-> 2000 us (full CW)");
  base.writeMicroseconds(2000);
  delay(2000);
  Serial.println("-> 1500 us (center)");
  base.writeMicroseconds(1500);
  Serial.println("done");
#else
#if defined(TARGET_ESP32)
  ESP32PWM::allocateTimer(0);
#if !defined(SERVO_SINGLE_TEST)
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
#endif
  base.setPeriodHertz(50);
#if !defined(SERVO_SINGLE_TEST)
  shoulder.setPeriodHertz(50);
  elbow.setPeriodHertz(50);
  wristPitch.setPeriodHertz(50);
  wristRoll.setPeriodHertz(50);
  gripper.setPeriodHertz(50);
#endif
#endif
  base.attach(PIN_BASE, 500, 2500);
#if !defined(SERVO_SINGLE_TEST)
  shoulder.attach(PIN_SHOULDER, 500, 2500);
  elbow.attach(PIN_ELBOW, 500, 2500);
  wristPitch.attach(PIN_WRIST_PITCH, 500, 2500);
  wristRoll.attach(PIN_WRIST_ROLL, 500, 2500);
  gripper.attach(PIN_GRIPPER, 500, 2500);
#endif

  base.writeMicroseconds(1500);
#if !defined(SERVO_SINGLE_TEST)
  shoulder.writeMicroseconds(1500);
  elbow.writeMicroseconds(1500);
  wristPitch.writeMicroseconds(1500);
  wristRoll.writeMicroseconds(1500);
  gripper.writeMicroseconds(1500);
#endif
#endif // SERVO_BARE_TEST
}

void loop() {}

// ─────────────────────────────────────────────────────────────────────────────
// Normal firmware — TCM trapezoidal profiles + serial command interface
// ─────────────────────────────────────────────────────────────────────────────
#else

#include <TCMController.h>
#include <SerialProtocol.h>

// Angle limits per joint (degrees, 0–180 hardware range)
#define BASE_MIN          0
#define BASE_MAX        180
#define SHOULDER_MIN     15
#define SHOULDER_MAX    165
#define ELBOW_MIN         0
#define ELBOW_MAX       180
#define WRIST_PITCH_MIN   0
#define WRIST_PITCH_MAX 180
#define WRIST_ROLL_MIN    0
#define WRIST_ROLL_MAX  180
#define GRIPPER_MIN      10
#define GRIPPER_MAX     170

// MG-996R servo pulse-width range: 500 μs (0°) → 2500 μs (180°), center = 1500 μs (90°)
#define BASE_MIN_US          500
#define BASE_MAX_US         2500
#define SHOULDER_MIN_US      500
#define SHOULDER_MAX_US     2500
#define ELBOW_MIN_US         500
#define ELBOW_MAX_US        2500
#define WRIST_PITCH_MIN_US   500
#define WRIST_PITCH_MAX_US  2500
#define WRIST_ROLL_MIN_US    500
#define WRIST_ROLL_MAX_US   2500
#define GRIPPER_MIN_US       500
#define GRIPPER_MAX_US      2500

// Center position compensation per joint (degrees, tune per physical servo)
// Positive = shift center clockwise, negative = counter-clockwise
#define BASE_CENTER_OFFSET          0
#define SHOULDER_CENTER_OFFSET      0
#define ELBOW_CENTER_OFFSET         0
#define WRIST_PITCH_CENTER_OFFSET   0
#define WRIST_ROLL_CENTER_OFFSET    0
#define GRIPPER_CENTER_OFFSET       0

// TCM motion profile constraints per joint
// maxVelocity  — cruise speed cap (deg/s)
// maxAccel     — acceleration / deceleration magnitude (deg/s²)
#define BASE_MAX_VEL        60.0f
#define BASE_MAX_ACCEL     120.0f
#define SHOULDER_MAX_VEL    60.0f
#define SHOULDER_MAX_ACCEL 120.0f
#define ELBOW_MAX_VEL       60.0f
#define ELBOW_MAX_ACCEL    120.0f
#define WRIST_PITCH_MAX_VEL    90.0f
#define WRIST_PITCH_MAX_ACCEL 180.0f
#define WRIST_ROLL_MAX_VEL    90.0f
#define WRIST_ROLL_MAX_ACCEL 180.0f
#define GRIPPER_MAX_VEL    90.0f
#define GRIPPER_MAX_ACCEL 180.0f

// TCM controller — manages trapezoidal motion profiles for all joints
TCMController controller;

// Serial command interface — uses hardware UART for robot control only
SerialProtocol protocol;

void setup() {
#if defined(TARGET_ESP32)
  // ESP32Servo requires PWM timers to be allocated before any servo attach().
  // Without this the attach() call silently fails and no PWM is generated.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
#endif

  // Joint configurations — pin, limits, center offset, and TCM constraints.
  // The controller attaches the servos and parks them at center on begin().
  TCMController::JointConfig joints[TCM_NUM_JOINTS] = {
    { &base,       PIN_BASE,        BASE_MIN,        BASE_MAX,        BASE_CENTER_OFFSET,        BASE_MAX_VEL,        BASE_MAX_ACCEL,        BASE_MIN_US,        BASE_MAX_US        },
    { &shoulder,   PIN_SHOULDER,    SHOULDER_MIN,    SHOULDER_MAX,    SHOULDER_CENTER_OFFSET,    SHOULDER_MAX_VEL,    SHOULDER_MAX_ACCEL,    SHOULDER_MIN_US,    SHOULDER_MAX_US    },
    { &elbow,      PIN_ELBOW,       ELBOW_MIN,       ELBOW_MAX,       ELBOW_CENTER_OFFSET,       ELBOW_MAX_VEL,       ELBOW_MAX_ACCEL,       ELBOW_MIN_US,       ELBOW_MAX_US       },
    { &wristPitch, PIN_WRIST_PITCH, WRIST_PITCH_MIN, WRIST_PITCH_MAX, WRIST_PITCH_CENTER_OFFSET, WRIST_PITCH_MAX_VEL, WRIST_PITCH_MAX_ACCEL, WRIST_PITCH_MIN_US, WRIST_PITCH_MAX_US },
    { &wristRoll,  PIN_WRIST_ROLL,  WRIST_ROLL_MIN,  WRIST_ROLL_MAX,  WRIST_ROLL_CENTER_OFFSET,  WRIST_ROLL_MAX_VEL,  WRIST_ROLL_MAX_ACCEL,  WRIST_ROLL_MIN_US,  WRIST_ROLL_MAX_US  },
    { &gripper,    PIN_GRIPPER,     GRIPPER_MIN,     GRIPPER_MAX,     GRIPPER_CENTER_OFFSET,     GRIPPER_MAX_VEL,     GRIPPER_MAX_ACCEL,     GRIPPER_MIN_US,     GRIPPER_MAX_US     },
  };

  controller.begin(joints);
#if defined(TARGET_ESP32)
  delay(500);  // ESP32 LEDC needs time to stabilize PWM after attach().
#endif
  protocol.begin(&controller);
  float home[TCM_NUM_JOINTS] = {90, 90, 90, 90, 90, 90};
  controller.moveTo(home, false);
}

void loop() {
  // Advance all trapezoidal motion profiles and write servo positions.
  controller.update();
  // Parse incoming serial commands and dispatch to the controller.
  protocol.update();
}

#endif // SERVO_CENTER_TEST
