// Offline exercise of the exact ModelFactory linked into rl_real_wdblh.
// This executable never opens serial devices or constructs the robot controller.
#include "inference_runtime.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace
{
void print_actions(const char* name, const std::vector<float>& actions)
{
    std::cout << name << "=[";
    for (size_t i = 0; i < actions.size(); ++i)
        std::cout << (i ? "," : "") << std::setprecision(10) << actions[i];
    std::cout << "]\n";
}
void validate_actions(const std::vector<float>& actions)
{
    if (actions.size() != 12 || !std::all_of(actions.begin(), actions.end(),
        [](float x) { return std::isfinite(x); }))
        throw std::runtime_error("Expected 12 finite actions");
}
}

int main(int argc, char** argv)
{
    try
    {
        if (argc < 3 || argc > 4)
            throw std::runtime_error("Usage: test_inference_runtime MODEL INPUT_ELEMENTS [RUNS=200]");
        const int size = std::stoi(argv[2]);
        const int runs = argc == 4 ? std::stoi(argv[3]) : 200;
        if ((size != 270 && size != 276) || runs < 1)
            throw std::runtime_error("Expected 270/276 input elements and positive runs");
        auto model = InferenceRuntime::ModelFactory::load_model(argv[1]);
        if (!model || !model->is_loaded())
            throw std::runtime_error("Model did not load");
        std::vector<float> nominal(size, 0.0f);
        const int frame_size = size / 6;
        for (int i = 0; i < 6; ++i)
        {
            nominal[i * frame_size + (size == 276 ? 9 : 8)] = -1.0f;
            if (size == 276) nominal[i * frame_size + 3] = 0.7f;
        }
        auto zero_actions = model->forward({std::vector<float>(size, 0.0f)});
        auto nominal_actions = model->forward({nominal});
        validate_actions(zero_actions);
        validate_actions(nominal_actions);
        print_actions("zeros", zero_actions);
        print_actions("nominal", nominal_actions);

        if (model->get_model_type() == "qnn248")
        {
            const auto expect_rejected = [&](const std::vector<std::vector<float>>& input) {
                try { model->forward(input); }
                catch (const std::runtime_error&) { return; }
                throw std::runtime_error("Invalid input was not rejected");
            };
            expect_rejected({std::vector<float>(size - 1)});
            expect_rejected({nominal, nominal});
            auto invalid = nominal;
            invalid[0] = std::numeric_limits<float>::quiet_NaN();
            expect_rejected({invalid});
            invalid[0] = std::numeric_limits<float>::infinity();
            expect_rejected({invalid});
            std::cout << "Invalid shape/NaN/Inf checks passed\n";
        }
        for (int i = 0; i < 20; ++i) validate_actions(model->forward({nominal}));
        std::mt19937 rng(30);
        std::normal_distribution<float> noise(0.0f, 0.05f);
        std::vector<double> times;
        for (int i = 0; i < runs; ++i)
        {
            auto input = nominal;
            for (auto& value : input) value += noise(rng);
            const auto start = std::chrono::steady_clock::now();
            auto actions = model->forward({input});
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            validate_actions(actions);
            times.push_back(ms);
        }
        std::sort(times.begin(), times.end());
        double total = 0;
        for (double ms : times) total += ms;
        std::cout << "PASS backend=" << model->get_model_type() << " runs=" << runs
                  << " mean_ms=" << total / runs << " p95_ms=" << times[(runs - 1) * 95 / 100]
                  << " max_ms=" << times.back() << std::endl;
        if (model->get_model_type() == "qnn248")
        {
            // Exercise replacement/destruction, used when a controller reloads a policy.
            if (!model->load(argv[1])) throw std::runtime_error("QNN reload failed");
            const auto reloaded = model->forward({nominal});
            if (reloaded != nominal_actions) throw std::runtime_error("QNN reload changed output");
            std::cout << "Reload/destruction check passed" << std::endl;
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << std::endl;
        return 1;
    }
}
