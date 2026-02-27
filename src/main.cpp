#include <Arduino.h>
#include <Servo.h>
#include <TCMController.h>
#include <SerialProtocol.h>

// PWM pin assignments — one MG995 servo per joint
#define PIN_BASE         3   // Base yaw
#define PIN_SHOULDER     5   // Shoulder pitch
#define PIN_ELBOW        6   // Elbow pitch
#define PIN_WRIST_PITCH  9   // Wrist pitch
#define PIN_WRIST_ROLL  10   // Wrist roll
#define PIN_GRIPPER     11   // Gripper open/close

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
    { &base,       PIN_BASE,        BASE_MIN,        BASE_MAX,        BASE_CENTER_OFFSET,        BASE_MAX_VEL,        BASE_MAX_ACCEL        },
    { &shoulder,   PIN_SHOULDER,    SHOULDER_MIN,    SHOULDER_MAX,    SHOULDER_CENTER_OFFSET,    SHOULDER_MAX_VEL,    SHOULDER_MAX_ACCEL    },
    { &elbow,      PIN_ELBOW,       ELBOW_MIN,       ELBOW_MAX,       ELBOW_CENTER_OFFSET,       ELBOW_MAX_VEL,       ELBOW_MAX_ACCEL       },
    { &wristPitch, PIN_WRIST_PITCH, WRIST_PITCH_MIN, WRIST_PITCH_MAX, WRIST_PITCH_CENTER_OFFSET, WRIST_PITCH_MAX_VEL, WRIST_PITCH_MAX_ACCEL },
    { &wristRoll,  PIN_WRIST_ROLL,  WRIST_ROLL_MIN,  WRIST_ROLL_MAX,  WRIST_ROLL_CENTER_OFFSET,  WRIST_ROLL_MAX_VEL,  WRIST_ROLL_MAX_ACCEL  },
    { &gripper,    PIN_GRIPPER,     GRIPPER_MIN,     GRIPPER_MAX,     GRIPPER_CENTER_OFFSET,     GRIPPER_MAX_VEL,     GRIPPER_MAX_ACCEL     },
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