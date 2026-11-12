/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Implementation of PC-side STM32 UART7 motor protocol.
 */

#include "pc_protocol.h"
#include <cstring>

// ── CRC-8/ATM (polynomial 0x07) ───────────────────────────────

uint8_t pc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ── Build command packet ──────────────────────────────────────

uint16_t pc_build_command(uint8_t *buffer, const MotorCommand *motors)
{
    PC_CmdPacket *pkt = reinterpret_cast<PC_CmdPacket *>(buffer);

    pkt->head = 0xAA;

    for (int i = 0; i < PC_MOTOR_NUM; i++) {
        pkt->motors[i].id   = static_cast<uint16_t>(i);
        pkt->motors[i].mode = (motors[i].tau != 0.0f || motors[i].kp != 0.0f || motors[i].kd != 0.0f) ? 1 : 0;
        pkt->motors[i].T    = motors[i].tau;
        pkt->motors[i].W    = motors[i].dq;
        pkt->motors[i].Pos  = motors[i].q;
        pkt->motors[i].K_P  = motors[i].kp;
        pkt->motors[i].K_W  = motors[i].kd;
    }

    // CRC over head + motors (289 bytes)
    pkt->crc = pc_crc8(buffer, 289);
    pkt->reserve[0] = 0;
    pkt->reserve[1] = 0;

    return PC_CMD_PACKET_SIZE;
}

// ── Parse status packet ───────────────────────────────────────

int pc_parse_status(const uint8_t *buffer, MotorState *motors)
{
    const PC_StatusPacket *pkt = reinterpret_cast<const PC_StatusPacket *>(buffer);

    if (pkt->head != 0xBB)
        return -1;

    uint8_t calc = pc_crc8(buffer, PC_STATUS_PACKET_SIZE - 1);
    if (calc != pkt->crc)
        return -1;

    for (int i = 0; i < PC_MOTOR_NUM; i++) {
        int idx = pkt->motors[i].id;
        if (idx < PC_MOTOR_NUM) {
            motors[idx].q       = pkt->motors[i].Pos * (6.2832f / 32768.0f);
            motors[idx].dq      = pkt->motors[i].W   / 256.0f;
            motors[idx].tau_est = pkt->motors[i].T   / 256.0f;
            motors[idx].mode    = pkt->motors[i].mode;
        }
    }

    return 0;
}

// ── Idle command ──────────────────────────────────────────────

void pc_make_idle(MotorCommand *motors)
{
    for (int i = 0; i < PC_MOTOR_NUM; i++) {
        motors[i].q   = 0.0f;
        motors[i].dq  = 0.0f;
        motors[i].tau = 0.0f;
        motors[i].kp  = 0.0f;
        motors[i].kd  = 0.0f;
    }
}
