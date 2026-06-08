/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ADCCollisionSensor.h"
#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

ADCCollisionSensor::~ADCCollisionSensor() = default;

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::init(mc_control::MCGlobalController & controller,
                               const mc_rtc::Configuration & config)
{

  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  // Make sure to have obstacle detection datastore entry
  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }
  
  // ── Config ───────────────────────────────────────────────────────────────
  std::string voltage_topic = config("voltage_topic",        std::string("/collision/voltage"));
  activate_verbose_ = config("verbose",               true);
  threshold_offset_ = config("threshold_offset",     5.0);   // Volts
  threshold_filtering_ = config("threshold_filtering",  1.0);  // LPF alpha

  // LpfThreshold::setValues(offset, filtering, jointNumber)
  // jointNumber=1 because we have a single scalar ADC channel
  lpf_.setValues(threshold_offset_, threshold_filtering_, 1);

  // ── ROS 2 subscriber ─────────────────────────────────────────────────────
  if(!ctl.controller().datastore().has("ros_spin"))
  {
     ctl.controller().datastore().make<bool>("ros_spin", false);
  }
  node_ = mc_rtc::ROSBridge::get_node_handle();
  if(!ctl.controller().datastore().get<bool>("ros_spin"))
  {
    spinThread_ = std::thread(std::bind(&ADCCollisionSensor::rosSpinner, this));
    ctl.controller().datastore().assign("ros_spin", true);
  }

  voltage_sub_.subscribe(node_, voltage_topic);
  voltage_sub_.maxTime(ctl.timestep());

  addGui(ctl);
  addLog(ctl);

  mc_rtc::log::info("[ADCCollisionSensor] init called with configuration:\n{}",config.dump(true, true));
}

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("[ADCCollisionSensor] reset called");
}

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::before(mc_control::MCGlobalController & controller)
{
   auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  // ── 1. Grab latest voltage (thread-safe) ─────────────────────────────────
  voltage_in_ = voltage_sub_.data().value();

  // ── 2. Compute adaptive threshold bounds ─────────────────────────────────
  threshold_high_ = lpf_.adaptiveThreshold(voltage_in_, true);
  threshold_low_  = lpf_.adaptiveThreshold(voltage_in_, false);

  // ── 3. Collision decision ─────────────────────────────────────────────────
  obstacle_detected_ = (voltage_in_ > threshold_high_) || (voltage_in_ < threshold_low_);

  // ── 4. Edge logging ───────────────────────────────────────────────────────
  if(obstacle_detected_ && !prev_obstacle_detected_)
  {
    prev_obstacle_detected_ = true;
    if(activate_verbose_)
    {
      mc_rtc::log::info("[ADCCollisionSensor] Obstacle detected v={:.3f} V  high={:.3f}  low={:.3f}",
                        voltage_in_, threshold_high_, threshold_low_);
    }
  }

  if(collision_stop_activated_)
  {
    ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::after(mc_control::MCGlobalController & /*controller*/)
{
  // Reserved for post-controller diagnostics.
}

// ─────────────────────────────────────────────────────────────────────────────
mc_control::GlobalPlugin::GlobalPluginConfiguration ADCCollisionSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after  = false;
  out.should_always_run = true;
  return out;
}

void ADCCollisionSensor::rosSpinner(void)
{
  mc_rtc::log::info("[ADCCollisionSensor][ROS Spinner] thread created for voltage subscriber");
  rclcpp::Rate r(1000); // Hz
  while(rclcpp::ok()and !stop_thread)
  {
    rclcpp::spin_some(node_);
    r.sleep();
  }
  mc_rtc::log::info("[ADCCollisionSensor][ROS Spinner] spinner destroyed");
}

void ADCCollisionSensor::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().gui()->addElement(
    {"Plugins", "ADCCollisionSensor"},
    mc_rtc::gui::NumberInput(
      "threshold_filtering",
      [this]() { return threshold_filtering_; },
      [this](double v)
      {
        threshold_filtering_ = v;
        lpf_.setFiltering(v);
      }),
    mc_rtc::gui::NumberInput(
      "threshold_offset (V)",
      [this]() { return threshold_offset_; },
      [this](double v)
      {
        threshold_offset_ = v;
        lpf_.setOffset(v);
      }),
    mc_rtc::gui::Label("voltage",
      [this]() { return std::to_string(voltage_in_); }),
    mc_rtc::gui::Label("threshold_high",
      [this]() { return std::to_string(threshold_high_); }),
    mc_rtc::gui::Label("threshold_low",
      [this]() { return std::to_string(threshold_low_); }),
    mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
    mc_rtc::gui::Checkbox(
      "Verbose",activate_verbose_)
  );
}

void ADCCollisionSensor::addLog(mc_control::MCGlobalController & controller)
{
  auto & logger = controller.controller().logger();
  logger.addLogEntry("ADCCollisionSensor_voltage",
                     [this]() { return voltage_in_; });
  logger.addLogEntry("ADCCollisionSensor_threshold_high",
                     [this]() { return threshold_high_; });
  logger.addLogEntry("ADCCollisionSensor_threshold_low",
                     [this]() { return threshold_low_; });
  logger.addLogEntry("ADCCollisionSensor_obstacle_detected",
                     [this]() { return obstacle_detected_; });
  logger.addLogEntry("ADCCollisionSensor_threshold_offset",
                     [this]() { return threshold_offset_; });
  logger.addLogEntry("ADCCollisionSensor_threshold_filtering",
                     [this]() { return threshold_filtering_; });
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ADCCollisionSensor", mc_plugin::ADCCollisionSensor)
