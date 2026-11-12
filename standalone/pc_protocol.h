/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * PC-side protocol for STM32 UART7 motor bus (wdblh platform).
 * Matches pc_protocol.h on the STM32 firmware.
 *
 * Command (PC → STM32): 292 bytes, 1 kHz
 *   [0]      head  = 0xAA
 *   [1-288]  12 motors × 24B: {uint16_t id, uint16_t mode, float T, float W, float Pos, float K_P, float K_W}
 *   [289]    crc8  over bytes 0-288
 *   [290-291] reserve
 *
 * Status (STM32 → PC): 124 bytes, 1 kHz
 *   [0]      head  = 0xBB
 *   [1]      cmd   = 0x02
 *   [2]      num   = 12
 *   [3-122]  12 motors × 10B: {uint8_t id, uint8_t mode, int16_t T, int16_t W, int32_t Pos}
 *   [123]    crc8  over bytes 0-122
 *
 * Conversion (status → physical):
 *   T   = T_raw / 256.0
 *   W   = W_raw / 256.0
 *   Pos = Pos_raw * 6.2832 / 32768.0
 */

#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H

#include <cstdint>

#define PC_MOTOR_NUM          12
#define PC_CMD_PACKET_SIZE    292
#define PC_STATUS_PACKET_SIZE 124

// ── Packet structures (packed, matching firmware) ─────────────

#pragma pack(push, 1)
struct MotorCmdRaw {
    uint16_t id;
    uint16_t mode;
    float    T;      // feedforward torque
    float    W;      // feedforward speed
    float    Pos;    // target position (rad)
    float    K_P;    // position gain
    float    K_W;    // velocity gain
};

struct MotorStatusRaw {
    uint8_t  id;
    uint8_t  mode;
    int16_t  T;      // torque × 256
    int16_t  W;      // speed × 256
    int32_t  Pos;    // position × (32768 / 2π)
};

struct PC_CmdPacket {
    uint8_t      head;
    MotorCmdRaw  motors[PC_MOTOR_NUM];
    uint8_t      crc;
    uint8_t      reserve[2];
};

struct PC_StatusPacket {
    uint8_t         head;
    uint8_t         cmd;
    uint8_t         num;
    MotorStatusRaw  motors[PC_MOTOR_NUM];
    uint8_t         crc;
};
#pragma pack(pop)

static_assert(sizeof(PC_CmdPacket)    == PC_CMD_PACKET_SIZE,    "CmdPacket size mismatch");
static_assert(sizeof(PC_StatusPacket) == PC_STATUS_PACKET_SIZE, "StatusPacket size mismatch");

// ── Physical-unit motor data (bridge to rl_sar) ───────────────

struct MotorCommand {
    float q;    // target position (rad)
    float dq;   // target velocity (rad/s)
    float tau;  // feedforward torque (N·m)
    float kp;   // position gain
    float kd;   // velocity gain
};

struct MotorState {
    float q;       // position (rad)
    float dq;      // velocity (rad/s)
    float tau_est; // estimated torque (N·m)
    uint8_t mode;  // motor status from feedback
};

// ── CRC-8/ATM (polynomial 0x07) ───────────────────────────────

uint8_t pc_crc8(const uint8_t *data, uint16_t len);

// ── Protocol functions ────────────────────────────────────────

// Build a 292-byte command packet from 12 motor commands.
// Returns packet size (292).
uint16_t pc_build_command(uint8_t *buffer, const MotorCommand *motors);

// Parse a 124-byte status packet into 12 motor states.
// Returns 0 on success, -1 on header/CRC error.
int pc_parse_status(const uint8_t *buffer, MotorState *motors);

// Fill all 12 motors with idle command (mode=0, zero setpoints).
void pc_make_idle(MotorCommand *motors);

#endif // PC_PROTOCOL_H
