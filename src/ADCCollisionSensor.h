/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
 
#include "LpfThreshold.h"
#include "ROS2Subscriber.h" 

#include <mc_control/GlobalPlugin.h>
 
namespace mc_plugin
{

/**
 * ADCCollisionSensor — mc_rtc GlobalPlugin
 *
 * Monitors the voltage output of an EL3102 ADC channel published as a
 * ROS 2 Float64 topic by the standalone EtherCAT ROS node.  Applies an
 * IIR low-pass filter, compares against a configurable threshold, and
 * writes the result to the mc_rtc datastore so controllers can react.
 *
 * This plugin contains no EtherCAT code — the ROS node owns the bus.
 * No spin thread is needed: mc_rtc's ROSBridge spins the node.
 *
 * Datastore keys written:
 *   "ADC_voltage"   (double) — filtered voltage, updated every cycle
 *   "ADC_collision" (bool)   — true when filtered voltage > threshold
 *   "Obstacle detected" (bool) — same as ADC_collision; shared
 *                                convention with other lab plugins
 */

struct ADCCollisionSensor : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller,
            const mc_rtc::Configuration & config) override;
 
  void reset(mc_control::MCGlobalController & controller) override;
 
  void before(mc_control::MCGlobalController & controller) override;
 
  void after(mc_control::MCGlobalController & controller) override;

  void addGui(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);
 
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;
 
  ~ADCCollisionSensor() override;
 
private:

  // GUI
  bool activate_verbose_ = true;

  // ── ROS 2 ────────────────────────────────────────────────────────────────
  std::shared_ptr<rclcpp::Node> node_;
  std::thread spinThread_;              // Thread to spin the ROS node
  std::mutex mutex_;                    // Mutex to lock the Force sensor data
  ROSFloatSubscriber voltage_sub_; // ROS Voltage subscriber
  void rosSpinner(void);     // ROS spinner function
  bool stop_thread = false;

  double voltage_in_ = 0.0; ///< latest value from ROS callback
 
  // ── Signal processing ────────────────────────────────────────────────────
  // LpfThreshold tracks a filtered version of the signal and returns
  // upper/lower threshold bounds.  We call adaptiveThreshold(v, true/false)
  // to get those bounds, then compare the raw signal against them.
  LpfThreshold lpf_;
  double threshold_offset_  = 1.0;  ///< volts above/below filtered signal
  double threshold_filtering_ = 0.05; ///< LPF coefficient (0–1)
  double threshold_high_ = 0.0;
  double threshold_low_  = 0.0;
 
  bool obstacle_detected_ = false;
  bool prev_obstacle_detected_ = false;
  bool collision_stop_activated_ = false;
};
 
} // namespace mc_plugin

