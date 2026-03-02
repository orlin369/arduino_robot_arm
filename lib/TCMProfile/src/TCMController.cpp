#include "TCMController.h"

// ── Construction ──────────────────────────────────────────────────────────────

TCMController::TCMController()
    : _tickMs(20), _lastTick(0), _initialized(false)
{}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void TCMController::begin(JointConfig joints[TCM_NUM_JOINTS], uint16_t tickMs)
{
    _tickMs = tickMs;

    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        _joints[j] = joints[j];

        // Physical center = logical 90° + calibration offset.
        float initPhys = 90.0f + (float)joints[j].centerOffset;
        initPhys = _clamp(initPhys,
                          (float)joints[j].minAngle,
                          (float)joints[j].maxAngle);

        // Configure motion profile (works in physical-degree space).
        _profiles[j].begin(joints[j].maxVelocity,
                           joints[j].maxAccel,
                           initPhys);

        // Attach servo with calibrated pulse-width range and drive to initial position.
        joints[j].servo->attach(joints[j].pin, joints[j].minUs, joints[j].maxUs);
        _writeServo(j, initPhys);
    }

    _lastTick    = millis();
    _initialized = true;
}

// ── Motion commands ───────────────────────────────────────────────────────────

void TCMController::moveTo(float targetAngles[TCM_NUM_JOINTS], bool synchronize)
{
    if (!_initialized) return;

    // ── Step 1: plan each joint at its natural (fastest) speed ───────────
    float durations[TCM_NUM_JOINTS];
    float maxDur = 0.0f;

    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        // Convert logical angle → physical angle, then clamp to limits.
        float phys = (float)targetAngles[j] + (float)_joints[j].centerOffset;
        phys = _clamp(phys,
                      (float)_joints[j].minAngle,
                      (float)_joints[j].maxAngle);

        durations[j] = _profiles[j].moveTo(phys);
        if (durations[j] > maxDur) maxDur = durations[j];
    }

    if (!synchronize || maxDur < 0.001f) return;

    // ── Step 2: re-plan faster joints to match the longest duration ───────
    // Joints that are already at target (duration == 0) are left idle.
    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        if (durations[j] > 0.001f && durations[j] < maxDur) {
            float phys = (float)targetAngles[j] + (float)_joints[j].centerOffset;
            phys = _clamp(phys,
                          (float)_joints[j].minAngle,
                          (float)_joints[j].maxAngle);
            _profiles[j].moveToDuration(phys, maxDur);
        }
    }
}

void TCMController::stop()
{
    if (!_initialized) return;

    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        // A zero-distance move immediately transitions the profile to IDLE.
        _profiles[j].setPosition(_profiles[j].position());
    }
}

// ── Runtime ───────────────────────────────────────────────────────────────────

void TCMController::update()
{
    if (!_initialized) return;

    uint32_t now     = millis();
    uint32_t elapsed = now - _lastTick;   // unsigned: handles millis() rollover

    if (elapsed < _tickMs) return;

    // Use the actual elapsed time so the profile stays accurate under jitter,
    // but cap at 3× tick to avoid a large step after a long stall.
    float dt = (float)elapsed * 0.001f;
    const float dtMax = (float)_tickMs * 3.0f * 0.001f;
    if (dt > dtMax) dt = dtMax;

    _lastTick = now;

    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        float pos = _profiles[j].update(dt);
        _writeServo(j, pos);
    }
}

// ── Status ────────────────────────────────────────────────────────────────────

bool TCMController::isRunning() const
{
    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        if (_profiles[j].isRunning()) return true;
    }
    return false;
}

float TCMController::position(uint8_t j) const
{
    if (j >= TCM_NUM_JOINTS) return 0.0f;
    // Return logical position: physical position minus the calibration offset.
    return _profiles[j].position() - (float)_joints[j].centerOffset;
}

// ── Private helpers ───────────────────────────────────────────────────────────

float TCMController::_clamp(float val, float lo, float hi) const
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void TCMController::_writeServo(uint8_t j, float physDeg)
{
    // Final hardware clamp: respect both software limits and servo [0, 180].
    float safe = _clamp(physDeg,
                        (float)_joints[j].minAngle,
                        (float)_joints[j].maxAngle);
    safe = _clamp(safe, 0.0f, 180.0f);
    _joints[j].servo->write((int)safe);
}
