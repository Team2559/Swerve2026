// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include <rev/config/SparkMaxConfig.h>
#include <wpi/sendable/Sendable.h>

#include <string>

#include "Constants.h"
#include "subsystems/TalonSparkSwerveModule.h"

using namespace DriveConstants;
using namespace rev::spark;

static units::turn_t wrapOffset(units::turn_t value) {
  return value - units::math::floor(value);
}

TalonSparkSwerveModule::TalonSparkSwerveModule(int driveCanID, int steerCanID, units::angle::turn_t offset) :
    driveMotor{driveCanID},
    steerMotor{steerCanID, SparkMax::MotorType::kBrushless},
    driveVelocity{driveMotor.GetVelocity().AsSupplier()},
    drivePosition{driveMotor.GetPosition().AsSupplier()},
    steerEncoder{steerMotor.GetAbsoluteEncoder()} {
  {
    using namespace ctre::phoenix6::configs;
    using namespace ctre::phoenix6::signals;
    TalonFXConfiguration driveConfig;
    driveConfig.CurrentLimits
      .WithStatorCurrentLimit(50.0_A)
      .WithSupplyCurrentLimit(50.0_A)
      .WithSupplyCurrentLowerLimit(40.0_A);
    driveConfig.MotorOutput
      .WithInverted(InvertedValue::Clockwise_Positive)
      .WithNeutralMode(NeutralModeValue::Brake);

    driveConfig.Slot0
      .WithGravityType(GravityTypeValue::Elevator_Static)
      .WithKP(DrivePID::kP)
      .WithKI(DrivePID::kI)
      .WithKD(DrivePID::kD)
      .WithKS(DrivePID::kS)
      .WithKV(DrivePID::kV)
      .WithKA(DrivePID::kA);

    driveMotor.GetConfigurator().Apply(driveConfig);
  }
  {
    SparkMaxConfig steerConfig;
    steerConfig
      .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
      .SmartCurrentLimit(60.0)
      .Inverted(kSteerMotorInverted);

    steerConfig.encoder
      .PositionConversionFactor(kSteerFeedbackScale / kSteerGearRatio)
      .VelocityConversionFactor(kSteerFeedbackScale / kSteerGearRatio * (1.0_s / 1.0_min));

    steerConfig.absoluteEncoder
      .ZeroOffset(wrapOffset(offset).value())
      .Inverted(kSteerSensorInverted)
      .PositionConversionFactor(kSteerFeedbackScale)
      .VelocityConversionFactor(kSteerFeedbackScale * (1.0_s / 1.0_min));

    steerConfig.closedLoop
      .SetFeedbackSensor(FeedbackSensor::kAbsoluteEncoder)
      .PositionWrappingEnabled(true)
      .PositionWrappingInputRange(-0.5 * kSteerFeedbackScale, 0.5 * kSteerFeedbackScale)
      .Pid(SteerPID::kP, SteerPID::kI, SteerPID::kD);

    steerConfig.closedLoop.feedForward
      .kS(SteerPID::kS);

    steerMotor.Configure(steerConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }
}

void TalonSparkSwerveModule::TestInit(std::string name) {
  frc::ShuffleboardTab &driveSetupTab = frc::Shuffleboard::GetTab("Drive Setup");
  frc::ShuffleboardTab &steerSetupTab = frc::Shuffleboard::GetTab("Steer Setup");

  nt_driveOutput = driveSetupTab.Add(name, std::array{0.0, 0.0, 0.0}).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  nt_steerOutput = steerSetupTab.Add(name, std::array{0.0, 0.0, 0.0}).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
}

void TalonSparkSwerveModule::TestExit() {
  nt_driveOutput = {};
  nt_steerOutput = {};
}

void TalonSparkSwerveModule::TestDebug() {
  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{1.0, 2.0, 3.0});
  }
}

static inline void BuildTalonPIDConfig(ctre::phoenix6::configs::SlotConfigs &config, const PIDUpdate &update) {

  switch (update.term) {
    case PIDUpdate::PIDTerm::kP:
      config.WithKP(update.value);
      break;
    case PIDUpdate::PIDTerm::kI:
      config.WithKI(update.value);
      break;
    case PIDUpdate::PIDTerm::kD:
      config.WithKD(update.value);
      break;
    case PIDUpdate::PIDTerm::kFF:
      // TODO: Make for flexible?
      config.WithKV(update.value);
      break;
  }
}

static inline void BuildSparkMaxPIDConfig(SparkMaxConfig &config, const PIDUpdate &update) {
  ClosedLoopSlot slot;
  switch (update.slot) {
    case 0:
      slot = kSlot0;
      break;
    case 1:
      slot = kSlot1;
      break;
    case 2:
      slot = kSlot2;
      break;
    case 3:
      slot = kSlot3;
      break;
    default:
      return;
  }

  switch (update.term) {
    case PIDUpdate::PIDTerm::kP:
      config.closedLoop.P(update.value, slot);
      break;
    case PIDUpdate::PIDTerm::kI:
      config.closedLoop.I(update.value, slot);
      break;
    case PIDUpdate::PIDTerm::kD:
      config.closedLoop.D(update.value, slot);
      break;
    case PIDUpdate::PIDTerm::kFF:
      // TODO: Make for flexible?
      config.closedLoop.feedForward.kV(update.value, slot);
      break;
  }
}

void TalonSparkSwerveModule::UpdateDrivePID(PIDUpdate &update) {
  if (update.term == PIDUpdate::PIDTerm::kFF) {
    driveVff = update.value;
    return;
  }
  ctre::phoenix6::configs::SlotConfigs slotConfig;
  BuildTalonPIDConfig(slotConfig, update);
  ctre::phoenix6::configs::TalonFXConfiguration config;
  switch (update.slot) {
    case 0:
      config.Slot0.From(slotConfig);
      break;
    case 1:
      config.Slot1.From(slotConfig);
      break;
    case 2:
      config.Slot2.From(slotConfig);
      break;
  }
  driveMotor.GetConfigurator().Apply(config);
}

void TalonSparkSwerveModule::UpdateSteerPID(PIDUpdate &update) {
  SparkMaxConfig config;
  BuildSparkMaxPIDConfig(config, update);
  steerMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
}

bool TalonSparkSwerveModule::GetStatus() const {
  // TODO: actually check things
  return true;
}

void TalonSparkSwerveModule::SetSteerOffset(units::angle::turn_t offset) {
  SparkMaxConfig steerConfig;

  steerConfig.absoluteEncoder.ZeroOffset(wrapOffset(offset).value());

  steerMotor.Configure(steerConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
}

units::angle::turn_t TalonSparkSwerveModule::GetSteerPosition() {
  return units::angle::turn_t{steerEncoder.GetPosition() * kInvSteerFeedbackScale};
}

void TalonSparkSwerveModule::SetSteerPosition(units::angle::turn_t position) {
  steerMotor.GetClosedLoopController().SetSetpoint(position.value() * kSteerFeedbackScale, SparkMax::ControlType::kPosition);
}

void TalonSparkSwerveModule::StopSteer() {
  steerMotor.Set(0.0);
}

void TalonSparkSwerveModule::SetDriveVelocity(units::velocity::meters_per_second_t velocity) {
  driveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage(velocity / kDriveDistancePerRotation));

  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{velocity.value(), (driveVelocity() * kDriveDistancePerRotation).value(), driveMotor.GetMotorVoltage().GetValue().value()});
  }
}

void TalonSparkSwerveModule::SetDrivePercent(double percent) {
  driveMotor.Set(percent);

  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{0.0, (driveVelocity() * kDriveDistancePerRotation).value(), driveMotor.GetMotorVoltage().GetValue().value()});
  }
}

void TalonSparkSwerveModule::StopDrive() {
  driveMotor.StopMotor();

  if (nt_driveOutput.has_value()) {
    nt_driveOutput.value()->SetDoubleArray(std::array{0.0, (driveVelocity() * kDriveDistancePerRotation).value(), driveMotor.GetMotorVoltage().GetValue().value()});
  }
}

const frc::SwerveModuleState TalonSparkSwerveModule::GetState() {
  return {driveMotor.GetVelocity(true).GetValue() * kDriveDistancePerRotation, GetSteerPosition()};
}

const frc::SwerveModulePosition TalonSparkSwerveModule::GetPosition() {
  return {drivePosition() * kDriveDistancePerRotation, GetSteerPosition()};
}

void TalonSparkSwerveModule::SetDesiredState(frc::SwerveModuleState &state) {
  auto currentAngle = frc::Rotation2d(GetSteerPosition());
  // Allow modules to flip their "positive" direction when rapidly changing requested directions
  state.Optimize(currentAngle);

  // Reduce speed of misoriented modules
  state.speed *= (state.angle - currentAngle).Cos();

  SetSteerPosition(state.angle.Radians());
  SetDriveVelocity(state.speed);
}

void TalonSparkSwerveModule::Stop() {
  StopSteer();
  StopDrive();
}
