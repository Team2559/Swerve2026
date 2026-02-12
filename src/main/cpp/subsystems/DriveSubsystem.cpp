// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <networktables/NetworkTableInstance.h>

#include <memory>
#include <string>

#include "Constants.h"
#include "subsystems/TalonSparkSwerveModule.h"

using namespace DriveConstants;

std::optional<frc::Rotation3d> GetRotation3D(studica::Navx &navX) {
  frc::Rotation3d rotation;
  if (navX.GetRotation3D(rotation) == 0) {
    return rotation;
  } else {
    return {};
  }
}

DriveSubsystem::DriveSubsystem() :
    SubsystemBase("Drive Subsystem"),
    frontLeftModule{
      new TalonSparkSwerveModule(kFrontLeftDriveMotorCanID, kFrontLeftSteerMotorCanID, kFrontLeftSteerOffset)
    },
    frontRightModule{
      new TalonSparkSwerveModule(kFrontRightDriveMotorCanID, kFrontRightSteerMotorCanID, kFrontRightSteerOffset)
    },
    rearLeftModule{
      new TalonSparkSwerveModule(kRearLeftDriveMotorCanID, kRearLeftSteerMotorCanID, kRearLeftSteerOffset)
    },
    rearRightModule{
      new TalonSparkSwerveModule(kRearRightDriveMotorCanID, kRearRightSteerMotorCanID, kRearRightSteerOffset)
    },
    m_navX{0},
    m_poseEstimator{kDriveKinematics, GetRotation3D(m_navX).value_or(frc::Rotation3d()), GetModulePositions(), frc::Pose3d()},
    m_driveTuner{
      [this](PIDUpdate update) {
        frontLeftModule->UpdateDrivePID(update);
        frontRightModule->UpdateDrivePID(update);
        rearLeftModule->UpdateDrivePID(update);
        rearRightModule->UpdateDrivePID(update);
      },
      DrivePID::kP,
      DrivePID::kI,
      DrivePID::kD,
      DrivePID::kV
    },
    m_steerTuner{
      [this](PIDUpdate update) {
        frontLeftModule->UpdateSteerPID(update);
        frontRightModule->UpdateSteerPID(update);
        rearLeftModule->UpdateSteerPID(update);
        rearRightModule->UpdateSteerPID(update);
      },
      SteerPID::kP,
      SteerPID::kI,
      SteerPID::kD
    },
    m_xController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
    m_yController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
    m_rController{OrientationPID::kP, OrientationPID::kI, OrientationPID::kD} {
  const frc::Pose3d initialPose = m_poseEstimator.GetEstimatedPosition();

  auto nt_instance = nt::NetworkTableInstance::GetDefault();
  auto table = nt_instance.GetTable("SmartDashboard/Drive");

  auto xTable = table->GetSubTable("X");
  nt_xPosition = xTable->GetDoubleTopic("Position").Publish();
  nt_xSetpoint = xTable->GetDoubleTopic("Setpoint").Publish();
  nt_xOutput = xTable->GetDoubleTopic("Output").Publish();

  nt_xPosition.SetDefault(initialPose.X().value());
  nt_xSetpoint.SetDefault(0.0);
  nt_xOutput.SetDefault(0.0);

  auto yTable = table->GetSubTable("Y");
  nt_yPosition = yTable->GetDoubleTopic("Position").Publish();
  nt_ySetpoint = yTable->GetDoubleTopic("Setpoint").Publish();
  nt_yOutput = yTable->GetDoubleTopic("Output").Publish();

  nt_yPosition.SetDefault(initialPose.Y().value());
  nt_ySetpoint.SetDefault(0.0);
  nt_yOutput.SetDefault(0.0);

  auto rTable = table->GetSubTable("R");
  nt_rPosition = rTable->GetDoubleTopic("Position").Publish();
  nt_rSetpoint = rTable->GetDoubleTopic("Setpoint").Publish();
  nt_rOutput = rTable->GetDoubleTopic("Output").Publish();

  nt_rPosition.SetDefault(initialPose.Rotation().ToRotation2d().Radians().value());
  nt_rSetpoint.SetDefault(0.0);
  nt_rOutput.SetDefault(0.0);

  // Bind test init and test exit to mode transition
  frc2::RobotModeTriggers::Test()
    .OnTrue(
      frc2::InstantCommand([this]() { TestInit(); })
        .AndThen(
          frc2::RunCommand([this]() {
            frontLeftModule->TestDebug();
            frontRightModule->TestDebug();
            rearLeftModule->TestDebug();
            rearRightModule->TestDebug();
          }).WithTimeout(1.0_s)
        )
    )
    .OnFalse(frc2::InstantCommand([this]() { TestExit(); }).ToPtr());

  frc::SmartDashboard::PutData(&field);
}

void DriveSubsystem::Periodic() {
  frc::Rotation3d heading = GetRotation3D(m_navX).value_or(frc::Rotation3d());

  frc::Pose3d pose = m_poseEstimator.Update(heading, GetModulePositions());

  nt_xPosition.Set(pose.X().value());
  nt_yPosition.Set(pose.Y().value());
  nt_rPosition.Set(pose.Rotation().ToRotation2d().Radians().value());

  frc::SmartDashboard::PutNumber("Front left drive", frontLeftModule->GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("Front right drive", frontRightModule->GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("Rear left drive", rearLeftModule->GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("Rear right drive", rearRightModule->GetPosition().distance.value());

  frc::SmartDashboard::PutNumber("Front left steer", frontLeftModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Front right steer", frontRightModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Rear left steer", rearLeftModule->GetSteerPosition().convert<units::deg>().value());
  frc::SmartDashboard::PutNumber("Rear right steer", rearRightModule->GetSteerPosition().convert<units::deg>().value());

  field.SetRobotPose(pose.ToPose2d());
}

void DriveSubsystem::SimulationPeriodic() {
}

void DriveSubsystem::TestInit() {
  frc::SmartDashboard::PutData("Drive Setup/PID", &m_driveTuner);
  frc::SmartDashboard::PutData("Steer Setup/PID", &m_steerTuner);

  // driveSetupTab.Add("Legend", "(Blue) setpoint, (Red) measured, (Green) output");
  // steerSetupTab.Add("Legend", "(Blue) setpoint, (Red) measured, (Green) output");

  frontLeftModule->TestInit("Front left");
  frontRightModule->TestInit("Front right");
  rearLeftModule->TestInit("Rear left");
  rearRightModule->TestInit("Rear right");

  m_sysIdChooser = {std::pair{std::string{"Steer"}, SteerSysId()}, std::pair{std::string{"Drive"}, DriveSysId()}};

  frc::SmartDashboard::PutData("Drive Setup/SysId", &m_sysIdChooser.value());
}

void DriveSubsystem::TestExit() {
  frontLeftModule->TestExit();
  frontRightModule->TestExit();
  rearLeftModule->TestExit();
  rearRightModule->TestExit();
}

void DriveSubsystem::ResetFieldOrientation(bool inverted) {
  m_poseEstimator.ResetPose(frc::Pose3d(GetPose().Translation(), inverted ? frc::Rotation3d(frc::Rotation2d(180_deg)) : frc::Rotation3d()));
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative, units::meter_t x_center, units::meter_t y_center) {
  frc::Rotation2d heading = GetPose().ToPose2d().Rotation();

  nt_xOutput.Set(xSpeed.value());
  nt_yOutput.Set(ySpeed.value());
  nt_rOutput.Set(rot.value());

  SetModuleStates(kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading)
                  : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
    {x_center, y_center}
  ));
}

void DriveSubsystem::FollowTrajectory(const choreo::SwerveSample &sample) {
  frc::Pose2d pose = GetPose().ToPose2d();

  units::radian_t robot_heading = pose.Rotation().Radians();
  units::radian_t sample_heading = sample.heading;
  units::radian_t target_heading = frc::AngleModulus(sample_heading - robot_heading) + robot_heading;

  units::meters_per_second_t xFeedback{m_xController.Calculate(pose.X().value(), sample.x.value())};
  units::meters_per_second_t yFeedback{m_yController.Calculate(pose.Y().value(), sample.y.value())};
  units::radians_per_second_t rotFeedback{
    m_rController.Calculate(robot_heading.value(), target_heading.value())
  };

  nt_xSetpoint.Set(sample.x.value());
  nt_ySetpoint.Set(sample.y.value());
  nt_rSetpoint.Set(sample.heading.value());

  Drive(
    sample.vx + xFeedback,
    sample.vy + yFeedback,
    sample.omega + rotFeedback,
    true
  );
}

void DriveSubsystem::SteerTo(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative, units::meter_t x_center, units::meter_t y_center) {
  frc::Rotation2d heading = GetPose().ToPose2d().Rotation();

  SetModuleStates(kDriveKinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading) : frc::ChassisSpeeds{xSpeed, ySpeed, rot}, {x_center, y_center}), true);
}

void DriveSubsystem::Stop() {
  nt_xOutput.Set(0.0);
  nt_yOutput.Set(0.0);
  nt_rOutput.Set(0.0);

  frontLeftModule->Stop();
  frontRightModule->Stop();
  rearLeftModule->Stop();
  rearRightModule->Stop();
}

const std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() {
  return {
    frontLeftModule->GetPosition(),
    frontRightModule->GetPosition(),
    rearLeftModule->GetPosition(),
    rearRightModule->GetPosition()
  };
}

void DriveSubsystem::SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates, bool steerOnly) {
  auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

  if (frontLeft.speed == 0.0_mps &&
      frontRight.speed == 0.0_mps &&
      rearLeft.speed == 0.0_mps &&
      rearRight.speed == 0.0_mps) {
    frontLeftModule->Stop();
    frontRightModule->Stop();
    rearLeftModule->Stop();
    rearRightModule->Stop();

    return;
  }

  if (!steerOnly) {
    frontLeftModule->SetDesiredState(frontLeft);
    frontRightModule->SetDesiredState(frontRight);
    rearLeftModule->SetDesiredState(rearLeft);
    rearRightModule->SetDesiredState(rearRight);
  } else {
    frontLeftModule->SetSteerPosition(frontLeft.angle.Radians());
    frontRightModule->SetSteerPosition(frontRight.angle.Radians());
    rearLeftModule->SetSteerPosition(rearLeft.angle.Radians());
    rearRightModule->SetSteerPosition(rearRight.angle.Radians());
  }
}

void DriveSubsystem::ResetPose(frc::Pose3d pose) {
  m_poseEstimator.ResetPose(pose);
}

frc::Pose3d DriveSubsystem::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::UpdateVisionPose(frc::Pose3d measurement, units::millisecond_t timestamp) {
  m_poseEstimator.AddVisionMeasurement(measurement, timestamp, {0.5, 0.5, 0.5, 0.8});
}

std::unique_ptr<frc2::sysid::SysIdRoutine> DriveSubsystem::SteerSysId() {
  return std::make_unique<frc2::sysid::SysIdRoutine>(
    frc2::sysid::Config{{}, 3_V, {}, nullptr},
    frc2::sysid::Mechanism{
      [this](units::volt_t steerVoltage) {
        frontLeftModule->SetSteerVoltage(steerVoltage);
        frontRightModule->SetSteerVoltage(steerVoltage);
        rearLeftModule->SetSteerVoltage(steerVoltage);
        rearRightModule->SetSteerVoltage(steerVoltage);
      },
      [this](frc::sysid::SysIdRoutineLog *log) {
        frontLeftModule->LogSteerInfo(log->Motor("frontLeftSteer"));
        frontRightModule->LogSteerInfo(log->Motor("frontRightSteer"));
        rearLeftModule->LogSteerInfo(log->Motor("rearLeftSteer"));
        rearRightModule->LogSteerInfo(log->Motor("rearRightSteer"));
      },
      this
    }
  );
}

std::unique_ptr<frc2::sysid::SysIdRoutine> DriveSubsystem::DriveSysId() {
  return std::make_unique<frc2::sysid::SysIdRoutine>(
    frc2::sysid::Config{{}, 3_V, {}, nullptr},
    frc2::sysid::Mechanism{
      [this](units::volt_t driveVoltage) {
        this->SteerTo(1.0_mps, 0.0_mps, 0.0_rad_per_s, false);
        frontLeftModule->SetDriveVoltage(driveVoltage);
        frontRightModule->SetDriveVoltage(driveVoltage);
        rearLeftModule->SetDriveVoltage(driveVoltage);
        rearRightModule->SetDriveVoltage(driveVoltage);
      },
      [this](frc::sysid::SysIdRoutineLog *log) {
        frontLeftModule->LogDriveInfo(log->Motor("frontLeftDrive"));
        frontRightModule->LogDriveInfo(log->Motor("frontRightDrive"));
        rearLeftModule->LogDriveInfo(log->Motor("rearLeftDrive"));
        rearRightModule->LogDriveInfo(log->Motor("rearRightDrive"));
      },
      this
    }
  );
}

frc2::CommandPtr DriveSubsystem::SysId() {
  return frc2::cmd::Defer(
    [this]() {
      if (m_sysIdChooser.has_value()) {
        return m_sysIdChooser->RunSelected();
      } else {
        return frc2::cmd::None();
      }
    },
    {this}
  );
}
