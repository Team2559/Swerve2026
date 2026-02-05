// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <string>

#include "helpers/PIDTuner.h"

class SwerveModule {
public:
  // Initializer
  SwerveModule() = default;
  ~SwerveModule() = default;

  // Test Methods
  virtual void TestInit(std::string name) = 0;
  virtual void TestExit() = 0;
  virtual void TestDebug() = 0;

  // Live PID tuning
  virtual void UpdateDrivePID(PIDUpdate &update) = 0;
  virtual void UpdateSteerPID(PIDUpdate &update) = 0;

  // Is the swerve module in a "healthy" state?
  virtual bool GetStatus() const = 0;

  // Steer functions; get, set, and stop moving; set causes a small motion
  // towards the setpoint
  virtual void SetSteerOffset(units::angle::turn_t offset) = 0;
  virtual units::turn_t GetSteerPosition() = 0;
  virtual void SetSteerPosition(units::turn_t position) = 0;
  virtual void StopSteer() = 0;

  // Drive functions; set, reset, and percent control; set causes a small
  // acceleration towards the setpoint, percent is useful for test profiling
  virtual void SetDriveVelocity(units::meters_per_second_t velocity) = 0;
  virtual void SetDrivePercent(double percent) = 0;
  virtual void StopDrive() = 0;

  // SysID functions; gives low-level access to the motors
  virtual void SetSteerVoltage(units::volt_t voltage) = 0;
  virtual void LogSteerInfo(frc::sysid::SysIdRoutineLog::MotorLog log) = 0;
  virtual void SetDriveVoltage(units::volt_t voltage) = 0;
  virtual void LogDriveInfo(frc::sysid::SysIdRoutineLog::MotorLog log) = 0;

  // Combined swerve module functions
  virtual const frc::SwerveModuleState GetState() = 0;
  virtual const frc::SwerveModulePosition GetPosition() = 0;
  virtual void SetDesiredState(frc::SwerveModuleState &state) = 0;
  virtual void Stop() = 0;
};
