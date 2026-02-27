#include "TCMProfile.h"
#include <math.h>

// ── Construction / initialisation ────────────────────────────────────────────

TCMProfile::TCMProfile()
    : _pos(0.0f), _vel(0.0f),
      _maxVel(60.0f), _maxAccel(120.0f),
      _target(0.0f), _dir(1.0f),
      _duration(0.0f), _elapsed(0.0f),
      _t1(0.0f), _t2(0.0f),
      _phase(Phase::IDLE)
{}

void TCMProfile::begin(float maxVelocity, float maxAcceleration,
                       float initialPosition)
{
    _maxVel   = maxVelocity;
    _maxAccel = maxAcceleration;
    _pos      = initialPosition;
    _target   = initialPosition;
    _vel      = 0.0f;
    _phase    = Phase::IDLE;
}

void TCMProfile::setPosition(float deg)
{
    _pos      = deg;
    _target   = deg;
    _vel      = 0.0f;
    _elapsed  = 0.0f;
    _duration = 0.0f;
    _phase    = Phase::IDLE;
}

// ── Motion planning ───────────────────────────────────────────────────────────

/*
 * _plan — core trapezoidal geometry.
 *
 * Given a non-negative distance |absDist| and a desired cruise velocity
 * `cruiseVel` (<= _maxVel), compute the timing breakpoints:
 *
 *   Triangle  (cruiseVel never reached):
 *     v_peak = sqrt(absDist * _maxAccel)
 *     t1 = t2 = v_peak / _maxAccel
 *     duration = 2 * t1
 *
 *   Trapezoid:
 *     t1 = cruiseVel / _maxAccel
 *     t2 = t1 + (absDist - cruiseVel²/_maxAccel) / cruiseVel
 *     duration = t2 + t1
 */
void TCMProfile::_plan(float absDist, float cruiseVel)
{
    float t_acc   = cruiseVel / _maxAccel;
    float s_acc   = 0.5f * _maxAccel * t_acc * t_acc;  // = cruiseVel² / (2*a)

    if (2.0f * s_acc >= absDist) {
        // ── Triangle profile ─────────────────────────────────────────────
        float t_peak = sqrtf(absDist / _maxAccel);
        _t1      = t_peak;
        _t2      = t_peak;          // zero-length cruise
        _duration = 2.0f * t_peak;
    } else {
        // ── Trapezoidal profile ──────────────────────────────────────────
        float s_cruise = absDist - 2.0f * s_acc;
        float t_cruise = s_cruise / cruiseVel;
        _t1      = t_acc;
        _t2      = t_acc + t_cruise;
        _duration = _t2 + t_acc;
    }
}

float TCMProfile::moveTo(float targetDeg)
{
    float dist = targetDeg - _pos;
    if (fabsf(dist) < 0.1f) {
        _phase = Phase::IDLE;
        return 0.0f;
    }

    _target  = targetDeg;
    _dir     = (dist > 0.0f) ? 1.0f : -1.0f;
    _elapsed = 0.0f;
    _vel     = 0.0f;

    _plan(fabsf(dist), _maxVel);
    _phase = Phase::ACCEL;
    return _duration;
}

float TCMProfile::moveToDuration(float targetDeg, float duration)
{
    float dist = targetDeg - _pos;
    if (fabsf(dist) < 0.1f) {
        _phase = Phase::IDLE;
        return 0.0f;
    }

    _target  = targetDeg;
    _dir     = (dist > 0.0f) ? 1.0f : -1.0f;
    _elapsed = 0.0f;
    _vel     = 0.0f;

    float absDist = fabsf(dist);
    float a       = _maxAccel;

    // ── Compute natural (fastest) duration at maxVel ──────────────────────
    float naturalDur;
    {
        float t_acc = _maxVel / a;
        float s_acc = 0.5f * a * t_acc * t_acc;
        if (2.0f * s_acc >= absDist) {
            naturalDur = 2.0f * sqrtf(absDist / a);
        } else {
            float s_c  = absDist - 2.0f * s_acc;
            naturalDur = 2.0f * t_acc + s_c / _maxVel;
        }
    }

    if (duration <= naturalDur) {
        // Requested duration is shorter than possible → use fastest profile.
        _plan(absDist, _maxVel);
    } else {
        // Solve for reduced cruise velocity V that gives exactly `duration`:
        //
        //   Trapezoidal time equation:
        //     T = V/a + absDist/V          (derived in header doc)
        //     V² - T·a·V + a·absDist = 0
        //     V = (T·a  ±  sqrt(T²·a² - 4·a·absDist)) / 2
        //   Take the smaller root (slower velocity → longer duration).
        //
        // Discriminant is guaranteed >= 0 when duration >= naturalDur.
        float T          = duration;
        float disc       = T * T * a * a - 4.0f * a * absDist;
        float cruiseVel;

        if (disc <= 0.0f) {
            // Edge case: pure triangle — no cruise phase possible.
            cruiseVel = sqrtf(absDist * a);
        } else {
            cruiseVel = (T * a - sqrtf(disc)) * 0.5f;
        }

        // Guard: never exceed configured max.
        if (cruiseVel > _maxVel) cruiseVel = _maxVel;

        _plan(absDist, cruiseVel);
    }

    _phase = Phase::ACCEL;
    return _duration;
}

// ── Runtime update ────────────────────────────────────────────────────────────

float TCMProfile::update(float dt)
{
    if (_phase == Phase::IDLE) return _pos;

    _elapsed += dt;

    // ── Finished? ────────────────────────────────────────────────────────
    if (_elapsed >= _duration) {
        _pos   = _target;
        _vel   = 0.0f;
        _phase = Phase::IDLE;
        return _pos;
    }

    // ── Velocity magnitude for current phase ─────────────────────────────
    if (_elapsed <= _t1) {
        // Acceleration: v = a · t
        _phase = Phase::ACCEL;
        _vel   = _maxAccel * _elapsed;

    } else if (_elapsed <= _t2) {
        // Cruise: constant velocity (= a · t1 = speed reached at end of accel)
        _phase = Phase::CRUISE;
        _vel   = _maxAccel * _t1;

    } else {
        // Deceleration: v = a · (duration - t)
        _phase = Phase::DECEL;
        _vel   = _maxAccel * (_duration - _elapsed);
    }

    _pos += _dir * _vel * dt;
    return _pos;
}
