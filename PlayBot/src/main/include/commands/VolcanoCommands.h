#pragma once

#pragma region Includes
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/Volcano.h"
#pragma endregion

frc2::CommandPtr SetVolcanoFlywheelSpeed(Volcano* volcano, units::turns_per_second_t speed);
frc2::CommandPtr VolcanoVariableFlywheelSpeed(Volcano* volcano, std::function<bool()> upSpeedFunc, std::function<bool()> downSpeedFunc);

frc2::CommandPtr VolcanoFlywheelOn(Volcano* volcano);
frc2::CommandPtr VolcanoFlywheelOff(Volcano* volcano);

frc2::CommandPtr VolcanoShootOneBall(Volcano* volcano);
frc2::CommandPtr VolcanoShootAllBalls(Volcano* volcano);

frc2::CommandPtr VolcanoStopAll(Volcano* volcano);
