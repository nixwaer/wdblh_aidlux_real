// Exercise the real WDBLH FSM with synthetic joint state; no hardware/controller.
#include "fsm_wdblh.hpp"
#include <stdexcept>

static void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

class OfflineRL : public RL
{
public:
    std::vector<float> Forward() override { throw std::runtime_error("unexpected inference"); }
    void GetState(RobotState<float>*) override { throw std::runtime_error("unexpected hardware read"); }
    void SetCommand(const RobotCommand<float>*) override { throw std::runtime_error("unexpected hardware write"); }
};

int main()
{
    try
    {
        OfflineRL rl;
        rl.ReadYaml("wdblh", "base.yaml");
        rl.InitJointNum(12);
        rl.robot_state.motor_state.q.assign(12, 0.2f);
        rl.fsm.AddState(std::make_shared<wdblh_fsm::RLFSMStatePassive>(&rl));
        rl.fsm.AddState(std::make_shared<wdblh_fsm::RLFSMStateGetUp>(&rl));
        rl.fsm.AddState(std::make_shared<wdblh_fsm::RLFSMStateGetDown>(&rl));
        rl.fsm.SetInitialState("RLFSMStatePassive");
        auto tick = [&] { rl.StateController(&rl.robot_state, &rl.robot_command); };
        auto wait = [&] {
            rl.control.current_gamepad = rl.control.last_gamepad = Input::Gamepad::None;
            rl.control.current_keyboard = rl.control.last_keyboard = Input::Keyboard::None;
            wdblh_fsm::ResetMotorWaitToPassive(rl, &rl.robot_state, &rl.robot_command);
        };
        auto idle = [&] {
            for (int i = 0; i < 12; ++i)
                require(rl.robot_command.motor_command.kp[i] == 0 &&
                        rl.robot_command.motor_command.kd[i] == 0 &&
                        rl.robot_command.motor_command.tau[i] == 0, "Passive must have zero effort");
        };
        tick(); idle();

        rl.control.current_gamepad = Input::Gamepad::A;
        tick(); // schedules GetUp for the NEXT control cycle
        require(rl.fsm.mode_ == FSM::Mode::CHANGE, "A must schedule GetUp");
        wait();
        tick(); idle();
        require(rl.fsm.GetCurrentStateName() == "RLFSMStatePassive", "waiting must cancel pending GetUp");

        // Fresh A after recovery captures the new measured posture, not old start.
        rl.robot_state.motor_state.q.assign(12, 0.4f);
        rl.control.current_gamepad = Input::Gamepad::A;
        tick();
        rl.control.current_gamepad = Input::Gamepad::None;
        tick();
        require(rl.fsm.GetCurrentStateName() == "RLFSMStateGetUp", "fresh A starts GetUp");
        require(rl.now_state.motor_state.q == rl.robot_state.motor_state.q,
                "GetUp must capture current feedback");
        require(rl.robot_command.motor_command.kp[0] > 0, "GetUp must produce active commands");
        wait(); tick(); idle();
        require(rl.fsm.GetCurrentStateName() == "RLFSMStatePassive", "loss during GetUp returns Passive");

        // A second recovery must restart interpolation and use another new pose.
        rl.robot_state.motor_state.q.assign(12, -0.3f);
        rl.control.current_gamepad = Input::Gamepad::A;
        tick();
        rl.control.current_gamepad = Input::Gamepad::None;
        tick();
        require(rl.now_state.motor_state.q == rl.robot_state.motor_state.q,
                "recovered GetUp must not reuse old interpolation start");
        wait(); tick(); idle();
        std::cout << "PASS: pending GetUp canceled, active GetUp reset, recovery captures current joints\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
