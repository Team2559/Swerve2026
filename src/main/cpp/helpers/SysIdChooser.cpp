// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "helpers/SysIdChooser.h"

#include <frc2/command/Commands.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

SysIdChooser::SysIdChooser(std::vector<std::pair<std::string, std::unique_ptr<frc2::sysid::SysIdRoutine>>> &&routines) {
  std::vector<std::string> routine_names{};

  for (auto &&routine : routines) {
    routine_names.push_back(routine.first);
    m_routines.emplace_back(std::move(routine.second));
  }

  for (auto choice : ExpandRoutineChoices(routine_names)) {
    m_sysIdChooser.AddOption(choice.first, choice.second);
  }
}

frc2::CommandPtr SysIdChooser::RunSelected() {
  return (
    frc2::SelectCommand<uint>(
      [this]() { return m_sysIdChooser.GetSelected(); },
      ExpandRoutineCommands(m_routines)
    )
      .ToPtr()
  );
}

void SysIdChooser::InitSendable(wpi::SendableBuilder &builder) {
  m_sysIdChooser.InitSendable(builder);
}

std::vector<std::pair<uint, std::unique_ptr<frc2::Command>>> SysIdChooser::ExpandRoutineCommands(
  std::vector<std::unique_ptr<frc2::sysid::SysIdRoutine>> &routines
) {
  std::vector<std::pair<uint, std::unique_ptr<frc2::Command>>> vec{};
  vec.reserve(routines.size() * 4);

  uint id = 0;
  for (auto &routine : routines) {
    vec.emplace_back(id + (uint)SysIdSubroutine::kQuasistaticForward, routine->Quasistatic(frc2::sysid::Direction::kForward).Unwrap());
    vec.emplace_back(id + (uint)SysIdSubroutine::kQuasistaticReverse, routine->Quasistatic(frc2::sysid::Direction::kReverse).Unwrap());
    vec.emplace_back(id + (uint)SysIdSubroutine::kDynamicForward, routine->Dynamic(frc2::sysid::Direction::kForward).Unwrap());
    vec.emplace_back(id + (uint)SysIdSubroutine::kDynamicForward, routine->Dynamic(frc2::sysid::Direction::kReverse).Unwrap());
    id += 4;
  }

  return vec;
}

std::vector<std::pair<std::string, uint>> SysIdChooser::ExpandRoutineChoices(
  std::vector<std::string> &routine_names
) {
  std::vector<std::pair<std::string, uint>> vec{};
  vec.reserve(routine_names.size() * 4);

  uint id = 0;
  for (auto &routine_name : routine_names) {
    vec.emplace_back(routine_name + " Quasistatic Forward", id + (uint)SysIdSubroutine::kQuasistaticForward);
    vec.emplace_back(routine_name + " Quasistatic Reverse", id + (uint)SysIdSubroutine::kQuasistaticReverse);
    vec.emplace_back(routine_name + " Dynamic Forward", id + (uint)SysIdSubroutine::kDynamicForward);
    vec.emplace_back(routine_name + " Dynamic Reverse", id + (uint)SysIdSubroutine::kDynamicForward);
    id += 4;
  }

  return vec;
}
