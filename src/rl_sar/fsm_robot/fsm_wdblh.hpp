// Modified 2026-11-11: WDBLH-only extraction and portable configuration.
/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * FSM for WDBLH robot platform.
 */

#ifndef WDBLH_FSM_HPP
#define WDBLH_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <cmath>
#include <fstream>
#include <string>

namespace wdblh_fsm
{

// Cancel even a pending Passive -> GetUp transition on feedback loss.
// RequestStateChange alone does not cancel it when the current state is Passive.
inline void ResetMotorWaitToPassive(RL& rl, const RobotState<float>* state,
                                   RobotCommand<float>* command)
{
    rl.rl_init_done = false;
    if (rl.fsm.GetCurrentStateName() == "RLFSMStatePassive")
    {
        rl.fsm.next_state_ = rl.fsm.current_state_;
        rl.fsm.mode_ = FSM::Mode::NORMAL;
    }
    else
    {
        rl.fsm.RequestStateChange("RLFSMStatePassive");
        rl.StateController(state, command);
    }
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to GetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 0;
            fsm_command->motor_command.tau[i] = 0;
        }
        // C key: print joint angles
        static bool last_c = false;
        bool c_key = (rl.control.current_keyboard == Input::Keyboard::C);
        if (c_key && !last_c) {
            const auto& names = rl.params.Get<std::vector<std::string>>("joint_names");
            printf("\n[JOINTS] ");
            for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); i++)
                printf("%s:%.3f ", names[i].c_str(), fsm_state->motor_state.q[i]);
            printf("\n");
        }
        last_c = c_key;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_getup = 0.0f;
    int calib_joint = 0;          // selected joint for calibration
    bool calib_mode = false;      // true when adjusting
    bool torque_test = false;     // true when testing torque direction
    bool debug_display = false;   // true when showing full joint table
    bool last_left = false, last_right = false, last_up = false, last_down = false, last_t = false, last_d = false;
    std::vector<float> calib_target;  // adjusted target positions

    void Enter() override
    {
        percent_getup = 0.0f;
        calib_joint = 0;
        calib_mode = false;
        torque_test = false;
        debug_display = false;
        calib_target.clear();
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        int num_dofs = rl.params.Get<int>("num_of_dofs");

        if (percent_getup >= 1.0f)
        {
            auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
            auto kd = rl.params.Get<std::vector<float>>("fixed_kd");

            // Initialize calib_target once
            if (calib_target.empty()) {
                calib_target = rl.params.Get<std::vector<float>>("default_dof_pos");
            }

            // Calibration mode: joint selection and adjustment
            bool left  = (rl.control.current_keyboard == Input::Keyboard::Left);
            bool right = (rl.control.current_keyboard == Input::Keyboard::Right);
            bool up    = (rl.control.current_keyboard == Input::Keyboard::Up);
            bool down  = (rl.control.current_keyboard == Input::Keyboard::Down);

            if (left && !last_left) {
                calib_joint = (calib_joint - 1 + num_dofs) % num_dofs;
                calib_mode = true;
            }
            if (right && !last_right) {
                calib_joint = (calib_joint + 1) % num_dofs;
                calib_mode = true;
            }
            if (up && !last_up && calib_mode) {
                calib_target[calib_joint] += 0.05f;
            }
            if (down && !last_down && calib_mode) {
                calib_target[calib_joint] -= 0.05f;
            }
            // Toggle torque test on selected joint (0.2 Nm)
            bool t_key = (rl.control.current_keyboard == Input::Keyboard::T);
            if (t_key && !last_t && calib_mode) {
                torque_test = !torque_test;
            }
            last_left = left; last_right = right; last_up = up; last_down = down; last_t = t_key;
            // D key: toggle full joint table display
            bool d_key = (rl.control.current_keyboard == Input::Keyboard::D);
            if (d_key && !last_d) { debug_display = !debug_display; }
            last_d = d_key;

            // Apply control
            for (int i = 0; i < num_dofs; ++i)
            {
                if (torque_test && i == calib_joint) {
                    // PD torque test: set target offset, compute torque
                    float test_tgt = calib_target[i] + 0.2f;
                    float pd_tau = 40.0f * (test_tgt - fsm_state->motor_state.q[i])
                                 - 1.0f  * fsm_state->motor_state.dq[i];
                    fsm_command->motor_command.tau[i] = pd_tau;
                    fsm_command->motor_command.q[i]   = 0;
                    fsm_command->motor_command.dq[i]  = 0;
                    fsm_command->motor_command.kp[i]  = 0;
                    fsm_command->motor_command.kd[i]  = 0;
                } else if (torque_test) {
                    // Other joints: PD torque hold at calib_target
                    float pd_tau = 40.0f * (calib_target[i] - fsm_state->motor_state.q[i])
                                 - 1.0f  * fsm_state->motor_state.dq[i];
                    fsm_command->motor_command.tau[i] = pd_tau;
                    fsm_command->motor_command.q[i]   = 0;
                    fsm_command->motor_command.dq[i]  = 0;
                    fsm_command->motor_command.kp[i]  = 0;
                    fsm_command->motor_command.kd[i]  = 0;
                } else {
                    fsm_command->motor_command.q[i]  = calib_target[i];
                    fsm_command->motor_command.dq[i] = 0;
                    fsm_command->motor_command.kp[i] = kp[i];
                    fsm_command->motor_command.kd[i] = kd[i];
                    fsm_command->motor_command.tau[i] = 0;
                }
            }

            // C key: print all joint angles (compact one-liner)
            static bool last_c_calib = false;
            bool c_calib = (rl.control.current_keyboard == Input::Keyboard::C);
            if (c_calib && !last_c_calib) {
                const auto& names = rl.params.Get<std::vector<std::string>>("joint_names");
                printf("\n[JOINTS] ");
                for (int i = 0; i < num_dofs; i++)
                    printf("%s:%.3f ", names[i].c_str(), fsm_state->motor_state.q[i]);
                printf("\n");
            }
            last_c_calib = c_calib;

            // Periodic display (every 500ms)
            static auto last_calib_print = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_calib_print).count() >= 500) {
                const auto& names = rl.params.Get<std::vector<std::string>>("joint_names");

                if (debug_display) {
                    // Full joint table — clear and redraw
                    printf("\033[2J\033[H");
                    printf("=== WDBLH Joint Debug ===  [D:toggle]  %s  %s\n",
                           calib_mode ? "CALIB" : "READY",
                           torque_test ? "TORQUE-TEST" : "");
                    printf("J#  Name           Tgt      Act      Vel      Trq    Kp    Kd\n");
                    printf("--- -------------- -------- -------- -------- ------ ---- ----\n");
                    for (int i = 0; i < num_dofs; i++) {
                        const char *sel = (i == calib_joint && calib_mode) ? ">" : " ";
                        // Shorten joint names
                        std::string short_name = names[i];
                        // strip _joint suffix
                        size_t pos = short_name.find("_joint");
                        if (pos != std::string::npos) short_name = short_name.substr(0, pos);
                        printf("%s%2d %-14s %+7.3f  %+7.3f  %+7.3f  %+5.2f %4.0f %4.1f\n",
                               sel, i, short_name.c_str(),
                               calib_target[i],
                               fsm_state->motor_state.q[i],
                               fsm_state->motor_state.dq[i],
                               fsm_state->motor_state.tau_est[i],
                               fsm_command->motor_command.kp[i],
                               fsm_command->motor_command.kd[i]);
                    }
                    // Hint line
                    if (calib_mode)
                        printf("\n[%d/%d %s] tgt=%.3f  ←→select  ↑↓±0.05  T=torque  C=oneliner  D=off\n",
                               calib_joint, num_dofs-1, names[calib_joint].c_str(), calib_target[calib_joint]);
                    else
                        printf("\n← → calibrate joints  1=RL  9=GetDown  P=stop  D=off\n");
                    fflush(stdout);
                } else if (calib_mode) {
                    // Single-line calibration info
                    printf("\r\033[K[CALIB] J%d/%d [%s]: tgt=%.3f act=%.3f  ← → select  ↑↓ adjust  D=table  T=torque%s",
                           calib_joint, num_dofs-1, names[calib_joint].c_str(),
                           calib_target[calib_joint], fsm_state->motor_state.q[calib_joint],
                           torque_test ? "[ON 0.2Nm]" : "[OFF]");
                    fflush(stdout);
                } else {
                    // Ready line
                    printf("\r\033[K[READY] Standing. ← → calibrate joints, 1=RL, 9=GetDown, P=stop, D=table.");
                    fflush(stdout);
                }
                last_calib_print = now;
            }
            return;
        }

        if (percent_getup == 0.0f)
        {
            float max_diff = 0.0f;
            auto target = rl.params.Get<std::vector<float>>("default_dof_pos");
            for (int i = 0; i < num_dofs; ++i)
                max_diff = std::max(max_diff, std::abs(rl.now_state.motor_state.q[i] - target[i]));
            if (max_diff < 0.1f) { percent_getup = 1.0f; return; }
        }

        // Fixed-speed profile: each joint moves at same velocity,
        // slowest joint determines total duration
        {
            auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
            auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
            auto target = rl.params.Get<std::vector<float>>("default_dof_pos");
            auto& start = rl.now_state.motor_state.q;

            // Compute max displacement for hips and legs separately
            float max_hip = 0.001f, max_leg = 0.001f;
            for (int i = 0; i < num_dofs; ++i) {
                float d = std::abs(target[i] - start[i]);
                if (i % 3 == 0) max_hip = std::max(max_hip, d);
                else            max_leg = std::max(max_leg, d);
            }

            float speed = 0.8f;  // rad/s — same speed for all joints
            float dur_hip = max_hip / speed;
            float dur_leg = max_leg / speed;
            float dt = rl.params.Get<float>("dt");

            // Advance hip and leg progress independently
            static float prog_hip = 0.0f, prog_leg = 0.0f;
            if (percent_getup == 0.0f) { prog_hip = 0.0f; prog_leg = 0.0f; }

            // Hips: active first, legs wait until hips done
            if (prog_hip < 1.0f) {
                prog_hip += dt / dur_hip;
                if (prog_hip >= 1.0f) prog_hip = 1.0f;
            } else {
                prog_leg += dt / dur_leg;
                if (prog_leg >= 1.0f) prog_leg = 1.0f;
            }

            for (int i = 0; i < num_dofs; ++i) {
                float pct = (i % 3 == 0) ? prog_hip : prog_leg;
                pct = std::min(pct, 1.0f);
                fsm_command->motor_command.q[i]  = (1 - pct) * start[i] + pct * target[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = kp[i];
                fsm_command->motor_command.kd[i] = kd[i];
                fsm_command->motor_command.tau[i] = 0;
            }

            percent_getup = (prog_hip + prog_leg) * 0.5f;
            if (prog_hip >= 1.0f && prog_leg >= 1.0f) percent_getup = 1.0f;
        }

        static float last_pct = 0;
        if (percent_getup - last_pct >= 0.25f || percent_getup >= 1.0f) {
            LOGGER::PrintProgress(percent_getup, "Getting up (hip→leg)");
            last_pct = percent_getup;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
                rl.control.current_gamepad == Input::Gamepad::Y)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
                     rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown,
                    rl.now_state.motor_state.q,
                    rl.start_state.motor_state.q,
                    2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X ||
            percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    std::thread init_thread;
    bool init_done = false;
    std::vector<float> saved_tgt;
    int settle_cnt = 0;
    static constexpr int SETTLE_CYCLES = 1;  // 1 FSM cycle (~1ms) — no settle

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.config_name = "himloco";
        saved_tgt.clear();  // will be re-initialized from motor state when RL ready
        settle_cnt = 0;
        init_done = false;
        // ── 模型已加载 (开机预加载或上次RL) → 直接就绪, 不在站立时重载卡顿 ──
        if (rl.model) { init_done = true; return; }
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        init_thread = std::thread([this, robot_config_path]() {
            try {
                rl.InitRL(robot_config_path);
                rl.now_state = *fsm_state;
            } catch (const std::exception &e) {
                std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
                rl.rl_init_done = false;
                rl.fsm.RequestStateChange("RLFSMStatePassive");
            }
            init_done = true;
        });
    }

    void Run() override
    {
        if (rl.control.model_switch_pending)
        {
            rl.control.model_switch_pending = false;
            std::cout << LOGGER::INFO << "[MODEL] Keeping configured WDBLH policy" << std::endl;
            return;
        }

        // Hold position while model loads in background
        if (!init_done) {
            auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
            auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
            auto target = rl.params.Get<std::vector<float>>("default_dof_pos");
            int num = rl.params.Get<int>("num_of_dofs");
            for (int i = 0; i < num; i++) {
                fsm_command->motor_command.q[i]  = target[i];
                fsm_command->motor_command.kp[i] = kp[i];
                fsm_command->motor_command.kd[i] = kd[i];
                fsm_command->motor_command.tau[i] = 0;
            }
            static auto last_load = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_load).count() >= 500) {
                printf("\r\033[K[RL] Loading model...");
                fflush(stdout);
                last_load = now;
            }
            return;
        }

        if (!rl.rl_init_done) {
            rl.rl_init_done = true;
            // Init saved_tgt from current motor state so torque mode holds position
            int n = rl.params.Get<int>("num_of_dofs");
            saved_tgt.resize(n);
            for (int i = 0; i < n; i++)
                saved_tgt[i] = fsm_state->motor_state.q[i];
        }

        // Read control type once
        static auto ctrl_type = rl.params.Get<std::string>("control_type");
        bool is_torque = (ctrl_type == "torque");

        RLControl();

        int num_dofs = rl.params.Get<int>("num_of_dofs");
        if (is_torque) {
            const auto &tau_lim = rl.params.Get<std::vector<float>>("torque_limits");
            const auto &t_kp = rl.control.rl_kp_override.empty()
                ? rl.params.Get<std::vector<float>>("rl_kp")
                : rl.control.rl_kp_override;
            const auto &t_kd = rl.control.rl_kd_override.empty()
                ? rl.params.Get<std::vector<float>>("rl_kd")
                : rl.control.rl_kd_override;
            static int diag_cnt = 0;
            if (diag_cnt < 3) printf("\n[TAU] ");
            // During settle: hold position, don't apply RL output
            bool settling = (settle_cnt++ < SETTLE_CYCLES);
            if (settling && settle_cnt == 1)
                printf("\n[SETTLE] Holding position for %dms (%d FSM cycles)...\n", SETTLE_CYCLES, SETTLE_CYCLES);

            for (int i = 0; i < num_dofs; ++i) {
                if (saved_tgt.size() != (size_t)num_dofs) saved_tgt.resize(num_dofs);
                float val = fsm_command->motor_command.q[i];
                if (!settling && val != 0.0f && std::isfinite(val))
                    saved_tgt[i] = saved_tgt[i] * 0.8f + val * 0.2f;
                float target_q = saved_tgt[i];
                float pd_tau = t_kp[i] * (target_q - fsm_state->motor_state.q[i])
                             - t_kd[i] * fsm_state->motor_state.dq[i];
                if (pd_tau >  tau_lim[i]) pd_tau =  tau_lim[i];
                if (pd_tau < -tau_lim[i]) pd_tau = -tau_lim[i];
                fsm_command->motor_command.tau[i] = pd_tau;
                fsm_command->motor_command.q[i]   = 0;
                fsm_command->motor_command.dq[i]  = 0;
                fsm_command->motor_command.kp[i]  = 0;
                fsm_command->motor_command.kd[i]  = 0;
                if (diag_cnt < 3) printf("%.2f/%.2f/%.1f ", target_q, fsm_state->motor_state.q[i], pd_tau);
            }
            if (diag_cnt++ < 3) printf("\n");
        }
    }

    void Exit() override
    {
        if (init_thread.joinable()) init_thread.join();
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

} // namespace wdblh_fsm

class WdblhFSMFactory : public FSMFactory
{
public:
    WdblhFSMFactory(const std::string &initial) : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<wdblh_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<wdblh_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<wdblh_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<wdblh_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }

    std::string GetType() const override { return "wdblh"; }

    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"};
    }

    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(WdblhFSMFactory, "RLFSMStatePassive")

#endif // WDBLH_FSM_HPP
