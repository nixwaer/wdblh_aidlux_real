// Modified 2026-11-11: WDBLH-only extraction and portable configuration.
/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inference_runtime.hpp"
#include <stdexcept>
#include <iostream>
#include <numeric>

namespace InferenceRuntime
{

// ============================================================================
// ONNXModel Implementation
// ============================================================================

ONNXModel::ONNXModel()
#ifdef USE_ONNX
    : memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
#endif
{
#ifdef USE_ONNX
    // Initialize ONNX Runtime environment
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXModel");
#endif
}

ONNXModel::~ONNXModel()
{
#ifdef USE_ONNX
    session_.reset();
    env_.reset();
#endif
}

bool ONNXModel::load(const std::string& model_path)
{
    try
    {
#ifdef USE_ONNX
        // Configure session options
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        // Create inference session
        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);

        // Setup input/output information
        setup_input_output_info();

        model_path_ = model_path;
        loaded_ = true;
        std::cout << LOGGER::INFO << "Successfully loaded ONNX model: " << model_path << std::endl;
        return true;
#else
        std::cout << LOGGER::WARNING << "ONNX support not compiled. Please define USE_ONNX." << std::endl;
        loaded_ = false;
        return false;
#endif
    }
    catch (const std::exception& e)
    {
        std::cout << LOGGER::ERROR << "Failed to load ONNX model: " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

std::vector<float> ONNXModel::forward(const std::vector<std::vector<float>>& inputs)
{
    if (!loaded_)
    {
        throw std::runtime_error("Model not loaded");
    }

#ifdef USE_ONNX
    try
    {
        // Create memory info
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        // Get input (use first input only)
        const auto& input = inputs[0];
        auto input_shape_raw = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        std::vector<int64_t> input_shape;
        for (auto dim : input_shape_raw) {
            input_shape.push_back(dim <= 0 ? 1 : dim);
        }

        // Create input tensor
        auto input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            const_cast<float*>(input.data()),
            input.size(),
            input_shape.data(),
            input_shape.size()
        );

        // Prepare input/output names
        const char* input_names[] = {input_node_names_[0].c_str()};
        const char* output_names[] = {output_node_names_[0].c_str()};

        // Execute inference
        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1
        );

        // Extract output data
        return extract_output_data(outputs);
    }
    catch (const std::exception& e)
    {
        std::cout << LOGGER::ERROR << "ONNX inference error: " << e.what() << std::endl;
        throw;
    }
#else
    throw std::runtime_error("ONNX support not compiled");
#endif
}

#ifdef USE_ONNX
void ONNXModel::setup_input_output_info()
{
    // Get input node information
    size_t num_input_nodes = session_->GetInputCount();
    input_node_names_.reserve(num_input_nodes);
    input_shapes_.reserve(num_input_nodes);

    for (size_t i = 0; i < num_input_nodes; ++i)
    {
        // Get input name
        auto input_name = session_->GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        input_node_names_.push_back(std::string(input_name.get()));

        // Get input shape
        Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_dims = input_tensor_info.GetShape();

        std::vector<int64_t> shape;
        for (auto dim : input_dims)
        {
            // Handle dynamic dimensions (string-named or -1)
            if (dim <= 0)
            {
                shape.push_back(1);
            }
            else
            {
                shape.push_back(dim);
            }
        }
        input_shapes_.push_back(shape);
    }

    // Get output node information
    size_t num_output_nodes = session_->GetOutputCount();
    output_node_names_.reserve(num_output_nodes);
    output_shapes_.reserve(num_output_nodes);

    for (size_t i = 0; i < num_output_nodes; ++i)
    {
        // Get output name
        auto output_name = session_->GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        output_node_names_.push_back(std::string(output_name.get()));

        // Get output shape
        Ort::TypeInfo output_type_info = session_->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto output_dims = output_tensor_info.GetShape();

        std::vector<int64_t> shape;
        for (auto dim : output_dims)
        {
            // Handle dynamic dimensions (string-named or -1)
            if (dim <= 0)
            {
                shape.push_back(1);
            }
            else
            {
                shape.push_back(dim);
            }
        }
        output_shapes_.push_back(shape);
    }
}

std::vector<float> ONNXModel::extract_output_data(const std::vector<Ort::Value>& outputs)
{
    if (outputs.empty())
    {
        throw std::runtime_error("No outputs from ONNX model");
    }

    // Get first output tensor
    auto& output = outputs[0];
    float* output_data = const_cast<float*>(output.GetTensorData<float>());

    // Calculate total number of output elements
    auto output_shape = output.GetTensorTypeAndShapeInfo().GetShape();

    int64_t num_elements = 1;
    for (auto dim : output_shape)
    {
        if (dim > 0)
        {
            num_elements *= dim;
        }
    }

    // Copy output data to vector
    std::vector<float> result(output_data, output_data + num_elements);

    return result;
}
#endif

// ============================================================================
// ModelFactory Implementation
// ============================================================================

std::unique_ptr<Model> ModelFactory::create_model(ModelType type)
{
    switch (type)
    {
        case ModelType::ONNX:
            return std::make_unique<ONNXModel>();
        case ModelType::QNN:
            return std::make_unique<QNNModel>();
        default:
            return nullptr;
    }
}

ModelFactory::ModelType ModelFactory::detect_model_type(const std::string& model_path)
{
    // Extract file extension from path
    std::filesystem::path path(model_path);
    std::string extension = path.extension().string();

    // Convert to lowercase for case-insensitive comparison
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    // Determine model type based on extension
    if (extension == ".onnx")
    {
        return ModelType::ONNX;
    }
    else if (extension == ".aidem")
    {
        return ModelType::QNN;
    }
    else
    {
        throw std::runtime_error("Unknown model file extension: " + extension + ". Supported: .onnx, .aidem");
    }
}

std::unique_ptr<Model> ModelFactory::load_model(const std::string& model_path, ModelType type)
{
    // If type is AUTO, automatically detect model type
    if (type == ModelType::AUTO)
    {
        type = detect_model_type(model_path);
    }

    // Create and load model
    auto model = create_model(type);
    if (model && model->load(model_path))
    {
        return model;
    }
    return nullptr;
}

} // namespace InferenceRuntime
