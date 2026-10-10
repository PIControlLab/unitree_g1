#include "State_UFO.h"
#include "param.h"

State_UFO::State_UFO(int state_mode, const std::string& state_string)
    : State_PolicyBase(state_mode, state_string)
{
    reference_ = std::make_shared<policy::UfoLatentReference>(
        param::resolve_path(deploy_cfg_["latents"]["catalog"].as<std::string>()),
        deploy_cfg_["latents"]["selection_file"].as<std::string>());
    hold_after_finished_ = deploy_cfg_["latents"]["hold_after_finished"].as<std::string>();

    ObservationSources sources;
    sources["reference_latent"] =
        [this](isaaclab::ManagerBasedRLEnv*, YAML::Node) { return reference_->latent(); };

    initialize(
        std::move(sources),
        [this](long step) { update_reference(step); },
        {},
        [this]() { reset_reference(); });

}

void State_UFO::reset_reference()
{
    reference_->select_on_entry(hold_after_finished_);
    holding_finished_pose_ = true;
    active_start_step_ = 0;
    reference_->update(0);
    spdlog::info(
        "[{}] UFO function={} task={} ({})",
        getStateString(), reference_->active_function(), reference_->active_id(),
        reference_->active_label());
}

void State_UFO::update_reference(long step)
{
    if (reference_->refresh_from_file())
    {
        active_start_step_ = step;
        holding_finished_pose_ = reference_->active_id() == hold_after_finished_;
        spdlog::info(
            "[{}] UFO switched function={} task={} ({})",
            getStateString(), reference_->active_function(), reference_->active_id(),
            reference_->active_label());
    }

    if (!holding_finished_pose_ && reference_->finished())
    {
        reference_->select(hold_after_finished_);
        holding_finished_pose_ = true;
        active_start_step_ = step;
        spdlog::info(
            "[{}] UFO task finished; holding {} ({})",
            getStateString(), reference_->active_id(), reference_->active_label());
    }

    const long relative_step = step - active_start_step_;
    reference_->update(static_cast<size_t>(std::max(0L, relative_step)));
}