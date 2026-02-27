#include "SerialProtocol.h"
#include <string.h>
#include <stdlib.h>

// Joint name → index lookup table (stored in SRAM; total < 60 bytes)
static const struct { const char *name; uint8_t idx; } JOINTS[] = {
    { "BASE",        0 },
    { "SHOULDER",    1 },
    { "ELBOW",       2 },
    { "WRIST_PITCH", 3 },
    { "WRIST_ROLL",  4 },
    { "GRIPPER",     5 },
};
static const uint8_t NUM_JOINT_NAMES =
    (uint8_t)(sizeof(JOINTS) / sizeof(JOINTS[0]));

// ── Construction ──────────────────────────────────────────────────────────────

SerialProtocol::SerialProtocol() : _ctrl(nullptr), _len(0)
{
    memset(_buf, 0, BUF_SIZE);
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void SerialProtocol::begin(TCMController *controller, uint32_t baud)
{
    _ctrl = controller;
    Serial.begin(baud);
}

// ── Runtime ───────────────────────────────────────────────────────────────────

void SerialProtocol::update()
{
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\r') continue;        // ignore CR in CRLF line endings

        if (c == '\n') {
            _buf[_len] = '\0';
            if (_len > 0) _processLine();
            _len = 0;
            return;                     // process one line per update() call
        }

        if (_len < BUF_SIZE - 1) {
            _buf[_len++] = c;
        }
        // silently drop bytes that overflow the buffer
    }
}

// ── Line parser ───────────────────────────────────────────────────────────────

void SerialProtocol::_processLine()
{
    // Convert to upper-case for case-insensitive matching.
    for (uint8_t i = 0; i < _len; i++) {
        if (_buf[i] >= 'a' && _buf[i] <= 'z') _buf[i] -= 32;
    }

    char *cmd = strtok(_buf, " ");
    if (!cmd) return;

    if (strcmp(cmd, "HOME") == 0) {
        _cmdHome();
    } else if (strcmp(cmd, "STOP") == 0) {
        _cmdStop();
    } else if (strcmp(cmd, "POS") == 0) {
        _cmdPos();
    } else if (strcmp(cmd, "MOVE") == 0) {
        // Pass the remainder of the buffer (everything after "MOVE ").
        char *rest = strtok(nullptr, "");
        _cmdMove(rest);
    } else {
        // Try to interpret as: <JOINT_NAME> <angle>
        char jointName[16];
        strncpy(jointName, cmd, sizeof(jointName) - 1);
        jointName[sizeof(jointName) - 1] = '\0';

        char *angleStr = strtok(nullptr, " ");
        if (!angleStr) {
            Serial.println(F("ERR MISSING_ANGLE"));
            return;
        }
        _cmdJoint(jointName, atof(angleStr));
    }
}

// ── Command handlers ──────────────────────────────────────────────────────────

void SerialProtocol::_cmdJoint(const char *name, float angle)
{
    for (uint8_t j = 0; j < NUM_JOINT_NAMES; j++) {
        if (strcmp(name, JOINTS[j].name) != 0) continue;

        // Build a full target array: keep all other joints at current position.
        float targets[TCM_NUM_JOINTS];
        for (uint8_t k = 0; k < TCM_NUM_JOINTS; k++) {
            targets[k] = _ctrl->position(k);  // logical degrees
        }
        targets[JOINTS[j].idx] = angle;

        // Single-joint move — no synchronization needed.
        _ctrl->moveTo(targets, false);

        Serial.print(F("OK "));
        Serial.print(name);
        Serial.print(' ');
        Serial.println(angle, 1);
        return;
    }
    Serial.println(F("ERR UNKNOWN_JOINT"));
}

void SerialProtocol::_cmdMove(char *args)
{
    if (!args) {
        Serial.println(F("ERR MISSING_ARGS"));
        return;
    }

    float targets[TCM_NUM_JOINTS];
    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        char *tok = strtok((j == 0) ? args : nullptr, " ");
        if (!tok) {
            Serial.println(F("ERR TOO_FEW_ARGS"));
            return;
        }
        targets[j] = atof(tok);
    }

    _ctrl->moveTo(targets, true);   // synchronized: all joints arrive together
    Serial.println(F("OK MOVE"));
}

void SerialProtocol::_cmdHome()
{
    float targets[TCM_NUM_JOINTS];
    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) targets[j] = 90.0f;
    _ctrl->moveTo(targets, true);
    Serial.println(F("OK HOME"));
}

void SerialProtocol::_cmdStop()
{
    _ctrl->stop();
    Serial.println(F("OK STOP"));
}

void SerialProtocol::_cmdPos()
{
    Serial.print(F("OK POS"));
    for (uint8_t j = 0; j < TCM_NUM_JOINTS; j++) {
        Serial.print(' ');
        Serial.print(_ctrl->position(j), 1);
    }
    Serial.println();
}
