#pragma once

#pragma region Includes
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/Volcano.h"
#pragma endregion

/// @brief Creates a command to turn the volcano flywheel on.
/// @param volcano A pointer to the Volcano subsystem.
/// @return A CommandPtr that turns the flywheel on.
frc2::CommandPtr SetVolcanoFlywheelSpeed(Volcano* volcano, units::turns_per_second_t speed);

/// @brief Creates a command to turn the volcano flywheel on to default speed.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoFlywheelOn(Volcano* volcano);

/// @brief Creates a command to turn the volcano flywheel off.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoFlywheelOff(Volcano* volcano);

/// @brief Creates a command to shoot one ball from the volcano.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoShootOneBall(Volcano* volcano);

/// @brief Creates a command to shoot all balls from the volcano.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoShootAllBalls(Volcano* volcano);

/// @brief Creates a command to stop all action in the volcano. HALTS ERUPTION.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoStopAll(Volcano* volcano);
/// @brief Creates a command to set the volcano flywheel to a variable speed.
/// @param volcano A pointer to the Volcano subsystem.
/// @param upSpeedFunc A function that raises the target speed by 100 turns per second
/// @param downSpeedFunc A function that lowers the target speed by 100 turns per second
frc2::CommandPtr VolcanoVariableFlywheelSpeed(
    Volcano*              volcano,
    std::function<bool()> upSpeedFunc,
    std::function<bool()> downSpeedFunc);