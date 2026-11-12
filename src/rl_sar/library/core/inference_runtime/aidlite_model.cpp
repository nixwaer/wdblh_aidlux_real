#include "inference_runtime.hpp"

#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>

#ifdef USE_AIDLITE
#include <aidlux/aidlite/aidlite.hpp>
#endif

namespace InferenceRuntime
{
#ifdef USE_AIDLITE
namespace AL = Aidlux::Aidlite;

namespace
{
void check(int32_t code, const char* operation)
{
    if (code != 0)
        throw std::runtime_error(std::string("AidLite ") + operation + " failed: " + std::to_string(code));
}

size_t validate_tensor(const AL::TensorInfo& tensor)
{
    if (tensor.shape.size() != 2 || tensor.shape[0] != 1 || tensor.shape[1] == 0 ||
        tensor.element_count != tensor.shape[1] ||
        tensor.element_count > std::numeric_limits<uint32_t>::max() / sizeof(float))
        throw std::runtime_error("QNN policy requires fixed [1, N] tensors");
    if (tensor.element_type != AL::DataType::TYPE_FLOAT16 &&
        tensor.element_type != AL::DataType::TYPE_FLOAT32)
        throw std::runtime_error("QNN policy requires float16 or float32 tensors");
    return static_cast<size_t>(tensor.element_count);
}
}
#endif

struct QNNModel::Impl
{
    bool loaded = false;
#ifdef USE_AIDLITE
    // Own model/config until the SDK interpreter takes ownership of its Context.
    std::unique_ptr<AL::Model> model;
    std::unique_ptr<AL::Config> config;
    std::unique_ptr<AL::Interpreter> interpreter;
    size_t input_size = 0;
    size_t output_size = 0;
    std::vector<float> input_buffer;
    std::mutex mutex;

    ~Impl()
    {
        if (interpreter)
        {
            const int32_t code = interpreter->destroy();
            if (code != 0)
                std::cerr << LOGGER::WARNING << "AidLite destroy failed: " << code << std::endl;
            interpreter.reset();
        }
    }
#endif
};

QNNModel::QNNModel() : impl_(std::make_unique<Impl>()) {}
QNNModel::~QNNModel() = default;
bool QNNModel::is_loaded() const { return impl_->loaded; }

bool QNNModel::load(const std::string& model_path)
{
    impl_ = std::make_unique<Impl>();
#ifdef USE_AIDLITE
    try
    {
        AL::set_log_level(AL::LogLevel::WARNING);
        AL::log_to_stderr();
        impl_->model.reset(AL::Model::create_instance(model_path));
        impl_->config.reset(AL::Config::create_instance());
        if (!impl_->model || !impl_->config)
            throw std::runtime_error("AidLite could not create model/config");
        impl_->config->framework_type = AL::FrameworkType::TYPE_QNN248;
        impl_->config->accelerate_type = AL::AccelerateType::TYPE_DSP;
        impl_->interpreter = AL::InterpreterBuilder::build_interpreter_from_model_and_config(
            impl_->model.get(), impl_->config.get());
        if (!impl_->interpreter)
            throw std::runtime_error("AidLite could not build interpreter");
        // AidLite's Interpreter owns Context, which deletes both Model and Config.
        // Keeping separate unique_ptr ownership would double-free them on exit/reload.
        impl_->model.release();
        impl_->config.release();
        check(impl_->interpreter->init(), "init");
        check(impl_->interpreter->load_model(), "load_model");
        std::vector<std::vector<AL::TensorInfo>> inputs, outputs;
        check(impl_->interpreter->get_input_tensor_info(inputs), "get_input_tensor_info");
        check(impl_->interpreter->get_output_tensor_info(outputs), "get_output_tensor_info");
        if (inputs.size() != 1 || inputs[0].size() != 1 ||
            outputs.size() != 1 || outputs[0].size() != 1)
            throw std::runtime_error("QNN policy requires one graph, one input and one output");
        impl_->input_size = validate_tensor(inputs[0][0]);
        impl_->output_size = validate_tensor(outputs[0][0]);
        impl_->input_buffer.resize(impl_->input_size);
        impl_->loaded = true;
        std::cout << LOGGER::INFO << "Successfully loaded QNN248/DSP model: " << model_path
                  << " (input [1," << impl_->input_size << "], output [1," << impl_->output_size
                  << "], float32 API / native "
                  << (inputs[0][0].element_type == AL::DataType::TYPE_FLOAT16 ? "float16" : "float32")
                  << ")" << std::endl;
        return true;
    }
    catch (const std::exception& error)
    {
        std::cerr << LOGGER::ERROR << "QNN model load failed: " << error.what() << std::endl;
        impl_ = std::make_unique<Impl>();
        return false;
    }
#else
    std::cerr << LOGGER::ERROR << "Cannot load " << model_path
              << ": AidLite backend not built; configure with -DENABLE_AIDLITE=ON" << std::endl;
    return false;
#endif
}

std::vector<float> QNNModel::forward(const std::vector<std::vector<float>>& inputs)
{
    if (!is_loaded())
        throw std::runtime_error("QNN model is not loaded");
#ifdef USE_AIDLITE
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (inputs.size() != 1 || inputs[0].size() != impl_->input_size)
        throw std::runtime_error("QNN input must contain exactly " +
                                 std::to_string(impl_->input_size) + " elements in one tensor");
    if (!std::all_of(inputs[0].begin(), inputs[0].end(), [](float x) {
            return std::isfinite(x) && std::abs(x) <= 65504.0f;
        }))
        throw std::runtime_error("QNN input contains NaN/Inf or exceeds float16 range");
    std::copy(inputs[0].begin(), inputs[0].end(), impl_->input_buffer.begin());
    // Non-native mode converts the public float32 buffers to/from native FP16.
    check(impl_->interpreter->set_input_tensor(uint32_t{0}, impl_->input_buffer.data(), false), "set_input_tensor");
    check(impl_->interpreter->invoke(), "invoke");
    void* output = nullptr;
    uint32_t output_bytes = 0;
    check(impl_->interpreter->get_output_tensor(uint32_t{0}, &output, false, &output_bytes), "get_output_tensor");
    if (!output || output_bytes != impl_->output_size * sizeof(float))
        throw std::runtime_error("QNN returned an invalid float32 output buffer (length " +
                                 std::to_string(output_bytes) + ")");
    const auto* data = static_cast<const float*>(output);
    std::vector<float> result(data, data + impl_->output_size);
    if (!std::all_of(result.begin(), result.end(), [](float x) { return std::isfinite(x); }))
        throw std::runtime_error("QNN output contains NaN/Inf");
    return result;
#else
    throw std::runtime_error("AidLite backend not built");
#endif
}
} // namespace InferenceRuntime
