#include <Arduino.h>
#if defined(TARGET_ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif
#include <TCMController.h>
#include <SerialProtocol.h>

// PWM pin assignments — one MG995 servo per joint
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

// Servo pulse-width calibration per joint (μs at 0° / 180°)
#define BASE_MIN_US          600
#define BASE_MAX_US         2300
#define SHOULDER_MIN_US      800
#define SHOULDER_MAX_US     2300
#define ELBOW_MIN_US         750
#define ELBOW_MAX_US        2300
#define WRIST_PITCH_MIN_US   750
#define WRIST_PITCH_MAX_US  2300
#define WRIST_ROLL_MIN_US    500
#define WRIST_ROLL_MAX_US   2000
#define GRIPPER_MIN_US      1700
#define GRIPPER_MAX_US      2800

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

// Servo objects — one per joint
Servo base;
Servo shoulder;
Servo elbow;
Servo wristPitch;
Servo wristRoll;
Servo gripper;

// TCM controller — manages trapezoidal motion profiles for all joints
TCMController controller;

// Serial command interface — uses hardware UART for robot control only
SerialProtocol protocol;

void setup() {
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