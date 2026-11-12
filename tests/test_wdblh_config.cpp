// Offline contract checks for the preserved WDBLH policy/configuration.
#include "rl_sdk.hpp"
#include <filesystem>
#include <stdexcept>

class OfflineRL final : public RL
{
public:
    std::vector<float> Forward() override { throw std::runtime_error("unexpected inference"); }
    void GetState(RobotState<float>*) override { throw std::runtime_error("unexpected device read"); }
    void SetCommand(const RobotCommand<float>*) override { throw std::runtime_error("unexpected device write"); }
};

static void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    try
    {
        OfflineRL rl;
        rl.ReadYaml("wdblh", "base.yaml");
        const auto base_angles = rl.params.Get<std::vector<float>>("default_dof_pos");
        require(base_angles.size() == 12, "expected 12 base angles");
        require(rl.params.Get<int>("num_of_dofs") == 12, "expected 12 DOFs");
        require(std::abs(rl.params.Get<float>("dt") - .001f) < 1e-8, "control period changed");
        require(rl.params.Get<int>("decimation") == 20, "policy period changed");
        rl.ReadYaml("wdblh/himloco", "config.yaml");
        const auto angles = rl.params.Get<std::vector<float>>("default_dof_pos");
        require(std::abs(base_angles.at(1) - .795709f) < 1e-6, "base calibration changed");
        require(std::abs(angles.at(1) - -.125924f) < 1e-6, "policy calibration changed");
        require(rl.params.Get<std::vector<int>>("joint_mapping") ==
            std::vector<int>({6,7,4,9,3,1,2,8,0,10,5,11}), "mapping changed");
        require(rl.params.Get<std::vector<int>>("motor_direction") ==
            std::vector<int>({1,1,-1,1,-1,1,-1,1,-1,-1,-1,1}), "direction changed");
        rl.InitJointNum(12);
        rl.InitObservations();
        rl.obs.base_quat = {1, 0, 0, 0};
        auto obs = rl.ComputeObservation();
        require(obs.size() == 46, "expected 46 observations");
        require(obs.at(3) == .7f && obs.at(9) == -1, "height/gravity order changed");
        require(rl.obs_dims == std::vector<int>({3,1,3,3,12,12,12}), "observation layout changed");
        const auto history = rl.params.Get<std::vector<int>>("observations_history");
        require(history == std::vector<int>({0,1,2,3,4,5}), "history order changed");
        ObservationBuffer buffer(1, rl.obs_dims, 6, "time");
        for (int i = 1; i <= 6; ++i) { obs[0] = static_cast<float>(i); buffer.insert(obs); }
        const auto packed = buffer.get_obs_vec(history);
        require(packed.size() == 276, "expected 276 model inputs");
        for (int frame = 0; frame < 6; ++frame)
            require(packed.at(frame * 46) == 6 - frame, "history must be newest first");
        const std::string model_name = rl.params.Get<std::string>("model_name");
        require(model_name == "himloco1_fix_qcs8550_fp16.qnn248.ctx.bin.aidem", "model changed");
        const auto path = std::filesystem::path(PolicyDirectory()) / "wdblh/himloco" / model_name;
        require(std::filesystem::is_regular_file(path), "deployment model missing");
        require(InferenceRuntime::ModelFactory::detect_model_type(path.string()) ==
            InferenceRuntime::ModelFactory::ModelType::QNN, "AIDEM dispatched to wrong backend");
        std::cout << "PASS: 12 joints, preserved calibration, 46x6 observations, AIDEM selection\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
