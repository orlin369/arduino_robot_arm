#pragma once
#include <Arduino.h>
#include <TCMController.h>

/**
 * SerialProtocol — Non-blocking line-oriented command parser for the robot arm.
 *
 * The hardware UART is used exclusively for robot control; no debug output
 * is emitted on this port.
 *
 * ── Command reference ────────────────────────────────────────────────────────
 *
 *  Single-joint move (logical degrees):
 *    BASE <angle>
 *    SHOULDER <angle>
 *    ELBOW <angle>
 *    WRIST_PITCH <angle>
 *    WRIST_ROLL <angle>
 *    GRIPPER <angle>
 *
 *  All-joints move (synchronized, logical degrees, space-separated):
 *    MOVE <base> <shoulder> <elbow> <wristPitch> <wristRoll> <gripper>
 *
 *  Utility:
 *    HOME          — move all joints to logical 90°
 *    STOP          — emergency halt at current positions
 *    POS           — report current logical positions
 *
 * ── Response format ──────────────────────────────────────────────────────────
 *
 *    OK <JOINT> <angle>      ← single-joint move accepted
 *    OK MOVE                 ← MOVE accepted
 *    OK HOME                 ← HOME accepted
 *    OK STOP                 ← STOP executed
 *    OK POS <f0> <f1> ... <f5>
 *    ERR <reason>            ← UNKNOWN_JOINT | MISSING_ANGLE |
 *                               MISSING_ARGS | TOO_FEW_ARGS | UNKNOWN_CMD
 *
 * ── Usage ────────────────────────────────────────────────────────────────────
 *
 *   SerialProtocol proto;
 *
 *   void setup() {
 *       proto.begin(&controller);   // 115200 baud by default
 *   }
 *   void loop() {
 *       controller.update();
 *       proto.update();
 *   }
 */
class SerialProtocol {
public:
    SerialProtocol();

    /**
     * Initialise the protocol and open the Serial port.
     * @param controller Pointer to the TCMController that receives commands.
     * @param baud       Baud rate (default 115200).
     */
    void begin(TCMController *controller, uint32_t baud = 115200);

    /**
     * Read available bytes and process complete lines.
     * Call every loop() iteration.
     */
    void update();

private:
    void _processLine();
    void _cmdJoint(const char *name, float angle);
    void _cmdMove(char *args);
    void _cmdHome();
    void _cmdStop();
    void _cmdPos();

    TCMController *_ctrl;

    static const uint8_t BUF_SIZE = 64;
    char    _buf[BUF_SIZE];
    uint8_t _len;
};
