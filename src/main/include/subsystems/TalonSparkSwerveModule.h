// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <networktables/DoubleArrayTopic.h>
#include <rev/SparkMax.h>

#include <string>

#include "SwerveModule.h"

class TalonSparkSwerveModule : public SwerveModule {
public:
  // Initializer
  TalonSparkSwerveModule(int driveCanID, int steerCanID, units::angle::turn_t offset);

  // Test Methods
  void TestInit(std::string name) override;
  void TestExit() override;
  void TestDebug() override;

  // Live PID tuning
  void UpdateDrivePID(PIDUpdate &update) override;
  void UpdateSteerPID(PIDUpdate &update) override;

  // Is the swerve module in a "healthy" state?
  bool GetStatus() const override;

  void SetSteerOffset(units::angle::turn_t offset) override;
  units::angle::turn_t GetSteerPosition() override;
  void SetSteerPosition(units::angle::turn_t position) override;
  void StopSteer() override;

  void SetDriveVelocity(units::velocity::meters_per_second_t velocity);
  void SetDrivePercent(double percent) override;
  void StopDrive() override;

  const frc::SwerveModuleState GetState() override;
  const frc::SwerveModulePosition GetPosition() override;
  void SetDesiredState(frc::SwerveModuleState &state) override;
  void Stop() override;

private:
  ctre::phoenix6::hardware::TalonFX driveMotor;
  rev::spark::SparkMax steerMotor;

  double driveVff;

  std::function<units::turns_per_second_t()> driveVelocity;
  std::function<units::turn_t()> drivePosition;
  rev::spark::SparkAbsoluteEncoder steerEncoder;

  std::optional<nt::DoubleArrayPublisher> nt_driveOutput = {};
  std::optional<nt::DoubleArrayPublisher> nt_steerOutput = {};
};
