#include "commands/SysIdChooser.h"

#include <frc2/command/Commands.h>

SysIdChooser::SysIdChooser(std::initializer_list<std::pair<std::string, std::unique_ptr<frc2::sysid::SysIdRoutine>>> routines) :
  m_routines{}
{
  std::vector<std::string> routine_names{};
  routine_names.reserve(routines.size());
  m_routines.reserve(routines.size());
  for (auto &routine : routines) {
    routine_names.push_back(routine.first);
    m_routines.push_back(routine.second);
  }

  for (auto choice : ExpandRoutineChoices(routine_names)) {
    m_sysIdChooser.AddOption(choice.first, choice.second);
  }
}

frc2::CommandPtr SysIdChooser::RunSelected() {
  return frc2::SelectCommand<std::pair<uint, SysIdSubroutine>>(
    [this]() { return m_sysIdChooser.GetSelected(); },
    ExpandRoutineCommands(m_routines)
  ).ToPtr();
}

std::vector<std::pair<std::pair<uint, SysIdSubroutine>, std::unique_ptr<frc2::Command>>> SysIdChooser::ExpandRoutineCommands(
  std::vector<std::unique_ptr<frc2::sysid::SysIdRoutine>> &routines
) {
  std::vector<std::pair<std::pair<uint, SysIdSubroutine>, std::unique_ptr<frc2::Command>>> vec{};
  vec.reserve(routines.size() * 4);

  uint id = 0;
  for (auto &routine : routines) {
    vec.emplace_back(std::pair{id, SysIdSubroutine::kQuasistaticForward}, routine->Quasistatic(frc2::sysid::Direction::kForward));
    vec.emplace_back(std::pair{id, SysIdSubroutine::kQuasistaticReverse}, routine->Quasistatic(frc2::sysid::Direction::kReverse));
    vec.emplace_back(std::pair{id, SysIdSubroutine::kDynamicForward}, routine->Dynamic(frc2::sysid::Direction::kForward));
    vec.emplace_back(std::pair{id, SysIdSubroutine::kDynamicForward}, routine->Dynamic(frc2::sysid::Direction::kReverse));
    id++;
  }

  return vec;
}

std::vector<std::pair<std::string, std::pair<uint, SysIdSubroutine>>> SysIdChooser::ExpandRoutineChoices(
  std::vector<std::string> &routine_names
) {
  std::vector<std::pair<std::string, std::pair<uint, SysIdSubroutine>>> vec{};
  vec.reserve(routine_names.size() * 4);

  uint id = 0;
  for (auto &routine_name : routine_names) {
    vec.emplace_back(fmt::format("%s Quasistatic Forward", routine_name), std::pair{id, SysIdSubroutine::kQuasistaticForward});
    vec.emplace_back(fmt::format("%s Quasistatic Reverse", routine_name), std::pair{id, SysIdSubroutine::kQuasistaticReverse});
    vec.emplace_back(fmt::format("%s Dynamic Forward", routine_name), std::pair{id, SysIdSubroutine::kDynamicForward});
    vec.emplace_back(fmt::format("%s Dynamic Reverse", routine_name), std::pair{id, SysIdSubroutine::kDynamicReverse});
    id++;
  }

  return vec;
}

