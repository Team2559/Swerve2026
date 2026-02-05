// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SelectCommand.h>
#include <frc2/command/sysid/SysIdRoutine.h>

enum class SysIdSubroutine {
  kQuasistaticForward,
  kQuasistaticReverse,
  kDynamicForward,
  kDynamicReverse,
};

class SysIdChooser {
public:
  SysIdChooser(std::initializer_list<std::pair<std::string, std::unique_ptr<frc2::sysid::SysIdRoutine>>> routines);

  frc2::CommandPtr RunSelected();

private:
  static std::vector<std::pair<std::pair<uint, SysIdSubroutine>, std::unique_ptr<frc2::Command>>> ExpandRoutineCommands(
    std::vector<std::unique_ptr<frc2::sysid::SysIdRoutine>> &routines
  );

  static std::vector<std::pair<std::string, std::pair<uint, SysIdSubroutine>>> ExpandRoutineChoices(
    std::vector<std::string> &routine_names
  );

  std::vector<std::unique_ptr<frc2::sysid::SysIdRoutine>> m_routines;
  frc::SendableChooser<std::pair<uint, SysIdSubroutine>> m_sysIdChooser{};
};
