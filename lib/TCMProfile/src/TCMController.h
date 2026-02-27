#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "TCMProfile.h"

static const uint8_t TCM_NUM_JOINTS = 6;

/**
 * TCMController — Coordinates six TCMProfile instances, one per servo joint.
 *
 * Responsibilities:
 *   - Attaches Servo objects to their PWM pins.
 *   - Applies per-joint center-offset calibration transparently.
 *   - Enforces per-joint software angle limits (clamping) before every move.
 *   - Optionally synchronises all joints so they finish simultaneously
 *     (longest-joint sets the common duration; others slow to match it).
 *   - Drives a fixed-rate update tick (default 20 ms) via millis().
 *
 * Usage:
 * @code
 *   TCMController ctrl;
 *
 *   void setup() {
 *       TCMController::JointConfig cfg[TCM_NUM_JOINTS] = { ... };
 *       ctrl.begin(cfg);
 *   }
 *
 *   void loop() {
 *       ctrl.update();          // call every loop iteration
 *   }
 *
 *   // Move all joints:
 *   float targets[TCM_NUM_JOINTS] = {90, 45, 120, 90, 90, 30};
 *   ctrl.moveTo(targets);       // synchronized by default
 * @endcode
 *
 * Inspired by the Robko01 update_interpolator / update_drivers pattern:
 *   https://github.com/robko01/robko01_retrofit_fw
 */
class TCMController {
public:
    TCMController();

    // ── Joint configuration ───────────────────────────────────────────────

    /**
     * Per-joint configuration passed to begin().
     * Angles are in the logical (user) space — center offset is added
     * internally before writing to the servo.
     */
    struct JointConfig {
        Servo   *servo;         ///< Pointer to the pre-declared Servo object.
        uint8_t  pin;           ///< Arduino PWM pin number.
        int16_t  minAngle;      ///< Software minimum angle (degrees, physical).
        int16_t  maxAngle;      ///< Software maximum angle (degrees, physical).
        int8_t   centerOffset;  ///< Calibration offset: physical = logical + offset.
        float    maxVelocity;   ///< Cruise speed cap (deg/s).
        float    maxAccel;      ///< Acceleration / deceleration (deg/s²).
    };

    // ── Lifecycle ─────────────────────────────────────────────────────────

    /**
     * Initialise controller, attach all servos, and park at center.
     * @param joints  Array of exactly TCM_NUM_JOINTS JointConfig entries.
     * @param tickMs  Update interval in milliseconds (default 20).
     */
    void begin(JointConfig joints[TCM_NUM_JOINTS], uint16_t tickMs = 20);

    // ── Motion commands ───────────────────────────────────────────────────

    /**
     * Command a move for all joints.
     *
     * Target angles are in logical (user) space (degrees).  The center
     * offset for each joint is added internally before motion planning,
     * and the result is clamped to [minAngle, maxAngle].
     *
     * @param targetAngles  Desired logical angles, one per joint.
     * @param synchronize   true  → all joints arrive simultaneously
     *                      false → each joint uses its natural duration.
     */
    void moveTo(float targetAngles[TCM_NUM_JOINTS], bool synchronize = true);

    /**
     * Emergency stop: halts all joints at their current positions.
     * Call moveTo() again to resume motion.
     */
    void stop();

    // ── Runtime ───────────────────────────────────────────────────────────

    /**
     * Advance motion profiles and write servo positions.
     * Must be called on every loop() iteration.
     */
    void update();

    // ── Status ────────────────────────────────────────────────────────────

    /** Returns true while any joint is still moving. */
    bool  isRunning()        const;

    /** Current computed position of joint j in physical degrees. */
    float position(uint8_t j) const;

private:
    float _clamp(float val, float lo, float hi) const;
    void  _writeServo(uint8_t j, float physDeg);

    TCMProfile  _profiles[TCM_NUM_JOINTS];
    JointConfig _joints[TCM_NUM_JOINTS];
    uint16_t    _tickMs;
    uint32_t    _lastTick;
    bool        _initialized;
};
