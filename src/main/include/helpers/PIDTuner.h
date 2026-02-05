// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/config/SparkBaseConfig.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * An update to a single PID term, of a specific slot
 */
struct PIDUpdate {
  /**
   * A specific PID term, limited to the basic feedforward model
   */
  enum class PIDTerm {
    kP,
    kI,
    kD,
    kFF,
  } term;

  /**
   * The new value for the specified term
   */
  double value;

  /**
   * Which slot to update on multi-slot devices
   */
  uint slot;
};

/**
 * Generic tuner for adjusting arbitrary PID loops live from the dashboard
 * The provided handler is called with a [`PIDUpdate`] whenever a change is made
 */
class PIDTuner : public wpi::Sendable, public wpi::SendableHelper<PIDTuner> {
public:
  explicit PIDTuner(std::function<void(PIDUpdate)> handler) :
      PIDTuner(handler, 0.0, 0.0, 0.0) {};
  PIDTuner(std::function<void(PIDUpdate)> handler, double kP, double kI, double kD) :
      PIDTuner(handler, kP, kI, kD, 0.0) {};
  PIDTuner(std::function<void(PIDUpdate)> handler, double kP, double kI, double kD, double kFF) :
      handler(handler), kP{kP}, kI{kI}, kD{kD}, kFF{kFF} {};
  ~PIDTuner() = default;

  void InitSendable(wpi::SendableBuilder &builder) override;

  /**
   * The currently selected slot
   */
  uint Slot();

private:
  // The handler to call with new PID updates
  std::function<void(PIDUpdate)> handler;

  // The currently selected PID slot
  uint slot = 0;

  // The current proportional term
  double kP;
  // The current integral term
  double kI;
  // The current derivative term
  double kD;
  // The current feedforward term
  double kFF;
};
