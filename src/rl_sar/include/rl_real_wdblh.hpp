// Modified 2026-11-11: WDBLH-only extraction and portable configuration.
/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * rl_real_wdblh – RL controller for WDBLH robot platform.
 * IMU:   DM-series external IMU via serial port (standalone driver).
 * Motor: STM32 UART7 protocol via serial port (pc_protocol).
 */

#ifndef RL_REAL_WDBLH_HPP
#define RL_REAL_WDBLH_HPP

// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_wdblh.hpp"

// DM IMU driver (standalone, zero ROS deps)
#include "dm_imu_raw.h"

// STM32 motor protocol
#include "pc_protocol.h"
#include "wdblh_motor_gate.hpp"

#include <csignal>
#include <atomic>
#include <cstdint>
#include <thread>
#include <memory>




class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();


private:
    // ── RL interface ────────────────────────────────────
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // ── Loop threads ────────────────────────────────────
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;


    // ── DM IMU (external serial) ────────────────────────
    DmImuRaw dm_imu;
    std::string imu_port_;   // auto-detect (or CLI arg)
    int         imu_baud_   = 921600;

    // ── Motor bus (STM32 UART7) ─────────────────────────
    std::string motor_port_; // auto-detect (or CLI arg)
    int         motor_baud_ = 4000000;

    int                motor_fd_{-1};
    std::thread        motor_thread_;
    std::atomic<bool>  motor_running_{false};
    wdblh::MotorFeedbackGate motor_gate_;  // protected by motor_mutex_
    std::atomic<bool> motor_motion_authorized_{false};
    // 最后一次成功写出 292 字节指令的时刻。USB 背压导致持续丢帧时，
    // 读线程可能仍在正常收包，但电机已经收不到新指令 —— 必须独立看门狗。
    std::atomic<int64_t> motor_last_tx_ok_ms_{0};
    std::atomic<int64_t> cmd_heartbeat_{0};
    std::atomic<int64_t> rl_heartbeat_{0};

    // Wall climb state

    // UDP navigation receiver
    int                udp_fd_{-1};
    std::thread        udp_thread_;
    std::atomic<bool>  udp_running_{false};
    std::atomic<float> udp_vx_{0}, udp_vy_{0}, udp_vyaw_{0};

    // Web Gamepad bridge (gamepad_bridge/server.py -> localhost UDP 9877)
    int                gamepad_fd_{-1};
    std::thread        gamepad_thread_;
    std::atomic<bool>  gamepad_running_{false};
    std::atomic<float> gamepad_lx_{0}, gamepad_ly_{0}, gamepad_rx_{0};
    std::atomic<uint32_t> gamepad_buttons_{0};
    std::atomic<int64_t> gamepad_last_rx_ms_{0};
    // Keep axes/buttons/receive time from the same packet for release-to-arm.
    mutable std::mutex gamepad_mutex_;

    mutable std::mutex motor_mutex_;
    MotorState          motor_state_[PC_MOTOR_NUM]{};
    MotorCommand        motor_command_[PC_MOTOR_NUM]{};

    void MotorInit();
    void MotorGetState(RobotState<float> *state);
    void MotorSetCommand(const RobotCommand<float> *command);
    void MotorDeinit();
    void MotorReadThread();
    void WebGamepadRecvThread();

    // ── ROS cmd_vel subscriber (navigation mode) ────────
};

#endif // RL_REAL_WDBLH_HPP
