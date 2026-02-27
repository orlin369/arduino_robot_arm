#pragma once
#include <Arduino.h>

/**
 * TCMProfile — Single-joint Trapezoidal Control Motion profile.
 *
 * Generates smooth position setpoints (degrees) using a three-phase
 * trapezoidal velocity profile:
 *
 *   Velocity
 *      ^
 *      |    /‾‾‾‾‾‾‾‾‾‾‾‾‾\
 *  Vmax|   /               \
 *      |  /  ACCEL  CRUISE  DECEL  \
 *      | /                          \
 *      |/____________________________\__> time
 *      t0   t1          t2          t3
 *
 * For short moves where Vmax is never reached a triangle profile is used
 * (CRUISE phase has zero duration).
 *
 * Call update(dt) at a fixed interval (e.g. every 20 ms) to advance the
 * profile.  The returned value is the new commanded position in degrees.
 *
 * Inspired by the Robko01 JointInterpolator pattern:
 *   https://github.com/robko01/robko01_retrofit_fw
 */
class TCMProfile {
public:
    enum class Phase : uint8_t { IDLE, ACCEL, CRUISE, DECEL };

    TCMProfile();

    /**
     * Configure motion constraints and set the initial position.
     * @param maxVelocity     Cruise velocity cap (deg/s).
     * @param maxAcceleration Acceleration / deceleration magnitude (deg/s²).
     * @param initialPosition Starting angle in degrees (default 0).
     */
    void begin(float maxVelocity, float maxAcceleration,
               float initialPosition = 0.0f);

    /**
     * Teleport to a position without motion.
     * Useful for re-homing after an E-stop.
     */
    void setPosition(float deg);

    /**
     * Plan and start a move to targetDeg at the configured constraints.
     * @return Planned move duration in seconds (0 if already at target).
     */
    float moveTo(float targetDeg);

    /**
     * Plan a move that fills exactly `duration` seconds by reducing the
     * cruise velocity.  Used to synchronise multiple joints so they all
     * arrive at the same time.
     *
     * If `duration` is shorter than physically possible the fastest natural
     * profile is used and the actual (shorter) duration is returned.
     *
     * @return Actual planned duration in seconds.
     */
    float moveToDuration(float targetDeg, float duration);

    /**
     * Advance the profile by dt seconds.
     * @return Updated position in degrees.
     */
    float update(float dt);

    // ── Accessors ──────────────────────────────────────────────────────────
    float position()  const { return _pos; }
    float velocity()  const { return _vel; }
    Phase phase()     const { return _phase; }
    bool  isRunning() const { return _phase != Phase::IDLE; }
    /** Total planned duration of the current move (seconds). */
    float duration()  const { return _duration; }

private:
    /** Internal: plan profile for |absDist| degrees at cruiseVel. */
    void _plan(float absDist, float cruiseVel);

    float _pos;       ///< current position (degrees)
    float _vel;       ///< current speed magnitude (deg/s)
    float _maxVel;    ///< configured cruise velocity cap (deg/s)
    float _maxAccel;  ///< configured acceleration magnitude (deg/s²)
    float _target;    ///< destination (degrees)
    float _dir;       ///< +1 or -1
    float _duration;  ///< total planned move time (s)
    float _elapsed;   ///< time elapsed since moveTo (s)
    float _t1;        ///< end of ACCEL phase (s)
    float _t2;        ///< end of CRUISE phase (s)  [== _t1 for triangle]
    Phase _phase;
};
