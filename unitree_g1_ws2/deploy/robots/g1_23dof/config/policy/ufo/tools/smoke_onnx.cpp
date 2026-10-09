#include <onnxruntime_cxx_api.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: smoke_onnx UFO_POLICY_DIRECTORY\n";
        return 2;
    }
    const std::filesystem::path root(argv[1]);
    std::vector<char> policy;
    for (const char* name : {"FBcprAuxModel.onnx.part00", "FBcprAuxModel.onnx.part01"}) {
        std::ifstream stream(root / "model" / name, std::ios::binary);
        policy.insert(policy.end(), std::istreambuf_iterator<char>(stream), {});
    }
    std::ifstream reference_stream(root / "reference" / "walk1_subject1_130_140.uforef", std::ios::binary);
    std::array<char, 24> header{};
    reference_stream.read(header.data(), header.size());
    uint32_t frames = 0;
    std::memcpy(&frames, header.data() + 12, sizeof(frames));
    std::vector<float> reference(static_cast<size_t>(frames) * 410);
    reference_stream.read(reinterpret_cast<char*>(reference.data()), reference.size() * sizeof(float));

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ufo_smoke");
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session encoder(env, (root / "model" / "backward_encoder.onnx").c_str(), options);
    Ort::Session actor(env, policy.data(), policy.size(), options);
    std::vector<float> states(static_cast<size_t>(frames) * 52), privileged(static_cast<size_t>(frames) * 358);
    for (uint32_t frame = 0; frame < frames; ++frame) {
        std::copy_n(reference.data() + static_cast<size_t>(frame) * 410, 52, states.data() + static_cast<size_t>(frame) * 52);
        std::copy_n(reference.data() + static_cast<size_t>(frame) * 410 + 52, 358, privileged.data() + static_cast<size_t>(frame) * 358);
    }
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
    const std::array<int64_t, 2> state_shape{frames, 52}, privileged_shape{frames, 358};
    std::array<Ort::Value, 2> encoder_inputs{
        Ort::Value::CreateTensor<float>(memory, states.data(), states.size(), state_shape.data(), 2),
        Ort::Value::CreateTensor<float>(memory, privileged.data(), privileged.size(), privileged_shape.data(), 2)
    };
    const std::array<const char*, 2> encoder_input_names{"state", "privileged_state"};
    const std::array<const char*, 1> encoder_output_names{"z"};
    auto encoder_outputs = encoder.Run(Ort::RunOptions{nullptr}, encoder_input_names.data(), encoder_inputs.data(), 2, encoder_output_names.data(), 1);
    const float* z = encoder_outputs[0].GetTensorData<float>();
    float z_norm = 0;
    for (int i = 0; i < 256; ++i) z_norm += z[i] * z[i];

    std::array<float, 631> observation{};
    std::copy_n(z, 256, observation.data() + 375);
    const std::array<int64_t, 2> actor_shape{1, 631};
    auto actor_input = Ort::Value::CreateTensor<float>(memory, observation.data(), observation.size(), actor_shape.data(), 2);
    const std::array<const char*, 1> actor_input_names{"actor_obs"}, actor_output_names{"action"};
    auto actor_outputs = actor.Run(Ort::RunOptions{nullptr}, actor_input_names.data(), &actor_input, 1, actor_output_names.data(), 1);
    const float* action = actor_outputs[0].GetTensorData<float>();
    float max_action = 0;
    for (int i = 0; i < 23; ++i) {
        if (!std::isfinite(action[i])) return 3;
        max_action = std::max(max_action, std::abs(action[i]));
    }
    std::cout << "frames=" << frames << " z_norm=" << std::sqrt(z_norm)
              << " max_abs_raw_action=" << max_action << '\n';
}
