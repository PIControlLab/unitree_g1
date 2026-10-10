#pragma once

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <onnxruntime_cxx_api.h>

namespace isaaclab
{

class Algorithms
{
public:
    virtual ~Algorithms() = default;
    virtual std::vector<float> act(
        const std::unordered_map<std::string, std::vector<float>>& observations) = 0;
};

class OrtRunner : public Algorithms
{
public:
    explicit OrtRunner(const std::vector<std::filesystem::path>& model_files)
        : env_(ORT_LOGGING_LEVEL_WARNING, "policy")
    {
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        if (model_files.size() == 1)
        {
            session_ = std::make_unique<Ort::Session>(
                env_, model_files.front().c_str(), session_options_);
        }
        else
        {
            for (const auto& path : model_files)
            {
                std::ifstream stream(path, std::ios::binary);
                model_bytes_.insert(
                    model_bytes_.end(), std::istreambuf_iterator<char>(stream), {});
            }
            session_ = std::make_unique<Ort::Session>(
                env_, model_bytes_.data(), model_bytes_.size(), session_options_);
        }

        for (size_t i = 0; i < session_->GetInputCount(); ++i)
        {
            input_names_.push_back(session_->GetInputNameAllocated(i, allocator_).get());
            input_shapes_.push_back(session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            if (input_shapes_.back()[0] < 0) input_shapes_.back()[0] = 1;
        }
        output_name_ = session_->GetOutputNameAllocated(0, allocator_).get();
        output_shape_ = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (output_shape_[0] < 0) output_shape_[0] = 1;
    }

    std::vector<float> act(
        const std::unordered_map<std::string, std::vector<float>>& observations) override
    {
        auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
        std::vector<Ort::Value> tensors;
        std::vector<const char*> names;
        tensors.reserve(input_names_.size());
        names.reserve(input_names_.size());

        for (size_t i = 0; i < input_names_.size(); ++i)
        {
            const auto& values = observations.at(input_names_[i]);
            names.push_back(input_names_[i].c_str());
            tensors.push_back(Ort::Value::CreateTensor<float>(
                memory, const_cast<float*>(values.data()), values.size(),
                input_shapes_[i].data(), input_shapes_[i].size()));
        }

        const char* output_name = output_name_.c_str();
        auto outputs = session_->Run(
            Ort::RunOptions{nullptr}, names.data(), tensors.data(), tensors.size(),
            &output_name, 1);
        const float* data = outputs.front().GetTensorData<float>();
        return std::vector<float>(data, data + output_shape_[1]);
    }

private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<char> model_bytes_;
    std::vector<std::string> input_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::string output_name_;
    std::vector<int64_t> output_shape_;
};

} // namespace isaaclab
