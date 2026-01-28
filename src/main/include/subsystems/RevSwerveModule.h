// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <networktables/DoubleArrayTopic.h>
#include <rev/SparkFlex.h>

#include <string>

#include "SwerveModule.h"

class RevSwerveModule : public SwerveModule {
public:
  // Initializer
  RevSwerveModule(int driveCanID, int steerCanID, units::angle::turn_t offset);

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
  rev::spark::SparkFlex driveMotor;
  rev::spark::SparkFlex steerMotor;

  double driveVff;

  rev::spark::SparkRelativeEncoder driveEncoder;
  rev::spark::SparkAbsoluteEncoder steerEncoder;

  std::optional<nt::DoubleArrayPublisher> nt_driveOutput = {};
  std::optional<nt::DoubleArrayPublisher> nt_steerOutput = {};
};
