// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SelectCommand.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <memory>
#include <string>
#include <vector>

enum class SysIdSubroutine {
  kQuasistaticForward,
  kQuasistaticReverse,
  kDynamicForward,
  kDynamicReverse,
};

class SysIdChooser: public wpi::Sendable {
public:
  template <std::convertible_to<std::unique_ptr<frc2::sysid::SysIdRoutine>>... SysIdRoutinePtrs>
  SysIdChooser(std::pair<std::string, SysIdRoutinePtrs> &&...routines) {
    std::vector<std::string> routine_names{};

    (routine_names.push_back(routines.first), ...);
    ((void)m_routines.emplace_back(std::move(routines.second)), ...);

    for (auto choice : ExpandRoutineChoices(routine_names)) {
      m_sysIdChooser.AddOption(choice.first, choice.second);
    }
  }

  explicit SysIdChooser(std::vector<std::pair<std::string, std::unique_ptr<frc2::sysid::SysIdRoutine>>> &&routines);

  frc2::CommandPtr RunSelected();

  void InitSendable(wpi::SendableBuilder &builder) override;

private:
  static std::vector<std::pair<uint, std::unique_ptr<frc2::Command>>> ExpandRoutineCommands(
    std::vector<std::unique_ptr<frc2::sysid::SysIdRoutine>> &routines
  );

  static std::vector<std::pair<std::string, uint>> ExpandRoutineChoices(
    std::vector<std::string> &routine_names
  );

  std::vector<std::unique_ptr<frc2::sysid::SysIdRoutine>> m_routines{};
  frc::SendableChooser<uint> m_sysIdChooser{};
};
