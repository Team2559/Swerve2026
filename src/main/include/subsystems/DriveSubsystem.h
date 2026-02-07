// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <choreo/Choreo.h>
#include <frc/controller/PIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator3d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <networktables/DoubleTopic.h>
#include <studica/Navx.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <memory>

#include "Constants.h"
#include "helpers/PIDTuner.h"
#include "helpers/SysIdChooser.h"
#include "subsystems/SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void TestInit();
  void TestExit();

  /**
   *  Reset forward for the driver to be the way the robot is currently facing
   */
  void ResetFieldOrientation(bool inverted = false);

  /**
   * Move at the requested set of speeds measured relative to either the robot or the field
   */
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative) {
    Drive(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }

  /**
   * Move at the requested set of speeds measured relative to either the robot or the field, with an arbitrary center of rotation
   */
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative, units::meter_t x_center, units::meter_t y_center);

  /**
   * Move towards the choreo trajectory sample position (with PID feedback)
   */
  void FollowTrajectory(const choreo::SwerveSample &sample);

  /**
   * Move the module steering to be aligned for the requested direction of travel
   */
  void SteerTo(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative) {
    SteerTo(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }

  /**
   * Move the module steering to be aligned for the requested direction of travel, with an arbitrary center of rotation
   */
  void SteerTo(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative, units::meter_t x_center, units::meter_t y_center);

  /**
   * Stop all drivetrain movement
   */
  void Stop();

  /**
   * Reset the robot's pose to the provided pose
   */
  void ResetPose(frc::Pose3d pose);

  /**
   * Gets the robot's current pose (position + orientation)
   */
  frc::Pose3d GetPose();

  /**
   * Incorporate a vision pose measurement into the robots cumulative pose estimation
   */
  void UpdateVisionPose(frc::Pose3d measurement, units::millisecond_t timestamp);

  /**
   * Get the current steer angle and wheel positions for all modules for odometry
   */
  const std::array<frc::SwerveModulePosition, 4> GetModulePositions();

  /**
   * Sets the desired steer angle and wheel velocity for all modules
   */
  void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates, bool steerOnly = false);

  // Front: +x, Rear: -x; Left: +y, Right -y. Zero heading is to the front
  // and +rotation is counter-clockwise. This is all standard, although it
  // means the robot's front is along the x-axis, which is often pointed to
  // the right, as things are occasionally drawn.
  /**
   * Drive kinematics for converting robot velocities into module velocities and vice-versa
   */
  frc::SwerveDriveKinematics<4> kDriveKinematics{
    frc::Translation2d(+DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
    frc::Translation2d(+DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2),
    frc::Translation2d(-DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
    frc::Translation2d(-DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2)
  };

  /**
   * Field drawing for dashboard debug information
   */
  frc::Field2d field;

  // Individual mechanism SysId routines
  std::unique_ptr<frc2::sysid::SysIdRoutine> SteerSysId();
  std::unique_ptr<frc2::sysid::SysIdRoutine> DriveSysId();

  // Combined SysId routine command
  frc2::CommandPtr SysId();

private:
  // The four swerve modules.
  std::unique_ptr<SwerveModule>
    frontLeftModule;
  std::unique_ptr<SwerveModule> frontRightModule;
  std::unique_ptr<SwerveModule> rearLeftModule;
  std::unique_ptr<SwerveModule> rearRightModule;

  // The navX gyro sensor.
  studica::Navx m_navX;

  // Pose estimator combines odometry with vision readings to yield an accurate robot pose; 4 specifies the number of modules.
  frc::SwerveDrivePoseEstimator3d<4> m_poseEstimator;

  // Tuners to adjust PID values live from the dashboard; greatly increases the ease of tuning
  PIDTuner m_driveTuner;
  PIDTuner m_steerTuner;

  // PID controllers for autonomous path following
  frc::PIDController m_xController;
  frc::PIDController m_yController;
  frc::PIDController m_rController;

  // SysId chooser for calibrating during test mode
  std::optional<SysIdChooser> m_sysIdChooser{};

  // Dashboard logging entries
  // -----------------------------------

  nt::DoublePublisher nt_xPosition;
  nt::DoublePublisher nt_xSetpoint;
  nt::DoublePublisher nt_xOutput;

  nt::DoublePublisher nt_yPosition;
  nt::DoublePublisher nt_ySetpoint;
  nt::DoublePublisher nt_yOutput;

  nt::DoublePublisher nt_rPosition;
  nt::DoublePublisher nt_rSetpoint;
  nt::DoublePublisher nt_rOutput;
};
