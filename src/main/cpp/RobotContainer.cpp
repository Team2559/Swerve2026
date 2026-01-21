// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/Trigger.h>

#include <tuple>

#include "ButtonUtil.h"
#include "commands/Autos.h"

RobotContainer::RobotContainer() :
    m_visionSubsystem(
      [this]() { return m_driveSubsystem.GetPose(); },
      [this](frc::Pose3d measurement, units::millisecond_t timestamp) { m_driveSubsystem.UpdateVisionPose(measurement, timestamp); }
    ) {
  // Initialize all of your commands and subsystems here

  m_driveSubsystem.SetDefaultCommand(
    frc2::RunCommand(
      [this]() {
        const auto controls = GetDriveTeleopControls();

        m_driveSubsystem.Drive(
          std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
          std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
          std::get<2>(controls) * DriveConstants::kMaxTurnRate,
          std::get<3>(controls)
        );
      },
      {&m_driveSubsystem}
    )
      .WithName("TeleopDrive")
  );

  // Configure the button bindings
  ConfigureBindings();

  nt::NetworkTableInstance nt_instance = nt::NetworkTableInstance::GetDefault();

  nt_fastDriveSpeed = nt_instance.GetDoubleTopic("/Drive/Max Speed").GetEntry(1.0);

  frc2::RobotModeTriggers::Disabled().OnFalse(
    frc2::InstantCommand([this]() {
      std::optional<frc::Pose3d> pose = m_visionSubsystem.SeedPose();
      if (pose.has_value()) {
        m_driveSubsystem.ResetPose(pose.value());
      }

      std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
      if (alliance.has_value()) {
        m_isRedAlliance = alliance.value() == frc::DriverStation::Alliance::kRed;
      }
    }).WithName("InitializeVision")
  );

  // frc::Shuffleboard::GetTab("LiveWindow")
  //   .AddBoolean("Live Window Enabled", frc::LiveWindow::IsEnabled);
  // TODO: Remove shuffleboard so it works with elastic
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  static auto steerOnlyCommand = frc2::RunCommand(
    [this]() {
      const auto controls = GetDriveTeleopControls();

      m_driveSubsystem.SteerTo(
        std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
        std::get<2>(controls) * DriveConstants::kMaxTurnRate,
        std::get<3>(controls)
      );
    },
    {&m_driveSubsystem}
  );

  frc::SmartDashboard::PutData("/Drive/Steer Only", &steerOnlyCommand);

  m_driverController.Back().OnTrue(
    frc2::InstantCommand(
      [this]() {
        m_driveSubsystem.ResetFieldOrientation(m_isRedAlliance);
      },
      {&m_driveSubsystem}
    )
      .IgnoringDisable(true)
      .WithName("Reset Field Orientation")
  );

  m_driverController.Start().ToggleOnTrue(
    frc2::StartEndCommand(
      []() { frc::LiveWindow::SetEnabled(true); },
      []() { frc::LiveWindow::SetEnabled(false); }
    ).WithName("LiveWindow")
  );
}

void RobotContainer::ListAutonomousCommands() {
  using namespace autos;
  m_autoChooser.SetDefaultOption("Example", AutoProgram::kExample);
  frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return autos::ExampleAuto(m_driveSubsystem);
}

std::tuple<double, double, double, bool> RobotContainer::GetDriveTeleopControls() {
  /*
  The robot's frame of reference is the standard unit circle, from
  trigonometry. However, the front of the robot is facing along the positive
  X axis. This means the positive Y axis extends outward from the left (or
  port) side of the robot. Positive rotation is counter-clockwise. On the
  other hand, as the controller is held, the Y axis is aligned with forward.
  And, specifically, it is the negative Y axis which extends forward. So,
  the robot's X is the controllers inverted Y. On the controller, the X
  axis lines up with the robot's Y axis. And, the controller's positive X
  extends to the right. So, the robot's Y is the controller's inverted X.
  Finally, the other controller joystick is used for commanding rotation and
  things work out so that this is also an inverted X axis.
  */
  double LeftTrigAnalogVal = m_driverController.GetLeftTriggerAxis();
  double LeftStickX = -m_driverController.GetLeftY();
  double LeftStickY = -m_driverController.GetLeftX();
  double rightStickRot = -m_driverController.GetRightX();

  // Limit driving speed according to the current maximum speed
  if (LeftTrigAnalogVal < .05) {
    LeftStickX *= DriveConstants::kSlowDrivePercent;
    LeftStickY *= DriveConstants::kSlowDrivePercent;
  } else {
    double fastDrivePercent = nt_fastDriveSpeed.Get(1.0);
    LeftStickX *= fastDrivePercent;
    LeftStickY *= fastDrivePercent;
  }

  // Flip field-relative velocities if the driver is on the red side of the field
  if (m_isRedAlliance) {
    LeftStickX *= -1.0;
    LeftStickY *= -1.0;
  }

  // Rescale final speeds to be more controllable
  if (m_triggerSpeedEnabled) // scale speed by analog trigger
  {
    double RightTrigAnalogVal = m_driverController.GetRightTriggerAxis();
    RightTrigAnalogVal = ConditionRawTriggerInput(RightTrigAnalogVal);

    if (LeftStickX != 0 || LeftStickY != 0) {
      if (LeftStickX != 0) {
        double LeftStickTheta = atan(LeftStickY / LeftStickX);
        LeftStickX = RightTrigAnalogVal * cos(LeftStickTheta);
        LeftStickY = RightTrigAnalogVal * sin(LeftStickTheta);
      } else {
        LeftStickY = std::copysign(RightTrigAnalogVal, LeftStickY);
      }
    }
  } else // scale speed by analog stick
  {
    LeftStickX = ConditionRawJoystickInput(LeftStickX);
    LeftStickY = ConditionRawJoystickInput(LeftStickY);
  }

  // Rescale final rotation speed to be more controllable
  rightStickRot = ConditionRawJoystickInput(rightStickRot);

  return std::make_tuple(LeftStickX, LeftStickY, rightStickRot, m_fieldOriented);
}
