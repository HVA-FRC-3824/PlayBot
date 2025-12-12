#include "commands/VolcanoCommands.h"

#pragma region VolcanoFlywheelOn
/// @brief Creates a command to turn the volcano flywheel on.
/// @param volcano A pointer to the Volcano subsystem.
/// @return A CommandPtr that turns the flywheel on.
frc2::CommandPtr SetVolcanoFlywheelSpeed(Volcano* volcano, units::turns_per_second_t speed)
{
    // Create and return a FunctionalCommand that turns the flywheel on
    return frc2::InstantCommand{[&] () { volcano->SetFlywheel(speed); }, { volcano }}.ToPtr();
}
#pragma endregion

#pragma region VolcanoFlywheelOn
/// @brief Creates a command to turn the volcano flywheel on to default speed.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoFlywheelOn(Volcano* volcano)
{
    // Create and return a command that turns the flywheel on
    return frc2::InstantCommand{[&] () { volcano->SetFlywheel(); }, { volcano }}.ToPtr();
}
#pragma endregion

#pragma region VolcanoFlywheelOff
/// @brief Creates a command to turn the volcano flywheel off.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoFlywheelOff(Volcano* volcano)
{
    // Create and return a command that turns the flywheel off
    return frc2::InstantCommand{[&] () { volcano->SetFlywheel(0_tps); }, { volcano }}.ToPtr();
}
#pragma endregion

#pragma region VolcanoShootOneBall(Volcano* volcano)
/// @brief Creates a command to shoot one ball from the volcano.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoShootOneBall(Volcano* volcano)
{
    // Turn on Flywheel and wait until at speed if its not already
    return frc2::InstantCommand{[volcano]() {
            volcano->SetIndexers(true);
            volcano->SetFlywheel();
        }, { volcano }}
        .Until([&]() {
            return volcano->IsBallDetected();
        }
        // Then kick the ball to the flywheel and wait a second
        ).AndThen(
            [&]() {
                volcano->SetKicker(true);
            },
            { volcano }
        ).AndThen(frc2::WaitCommand(1_s).ToPtr()
        // Finally, turn off the kicker and indexers
        ).AndThen(
            [&]() {
                volcano->SetKicker(false);
                volcano->SetIndexers(false);
                volcano->SetFlywheel(0_tps);
            },
            { volcano }
        );
}
#pragma endregion

#pragma region VolcanoShootAllBalls(Volcano* volcano)
/// @brief Creates a command to shoot all balls from the volcano.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoShootAllBalls(Volcano* volcano)
{
    // Turn on Flywheel and wait until at speed if its not already
    return VolcanoFlywheelOn(volcano).Until([&]() { return volcano->IsFlywheelAtSpeed(); }
        // Then activate everything, making all balls go through the system
        ).AndThen(
            frc2::InstantCommand{[volcano]() {
                volcano->SetIndexers(true);
                volcano->SetKicker(true);
                volcano->SetFlywheel();
            }, { volcano }}.ToPtr()
        );
}
#pragma endregion

#pragma region VolcanoStopAll(Volcano* volcano)
/// @brief Creates a command to stop all action in the volcano. HALTS ERUPTION.
/// @param volcano A pointer to the Volcano subsystem.
frc2::CommandPtr VolcanoStopAll(Volcano* volcano)
{
    return frc2::InstantCommand{[&]() {
            volcano->SetIndexers(false);
            volcano->SetKicker(false);
            volcano->SetFlywheel(0_tps);
        }, { volcano }}.ToPtr();
}
#pragma endregion

#pragma region VolcanoVariableFlywheelSpeed
/// @brief Creates a command to set the volcano flywheel to a variable speed.
/// @param volcano A pointer to the Volcano subsystem.
/// @param upSpeedFunc A function that raises the target speed by 100 turns per second
/// @param downSpeedFunc A function that lowers the target speed by 100 turns per second
frc2::CommandPtr VolcanoVariableFlywheelSpeed(
    Volcano* volcano,
    std::function<bool()> upSpeedFunc,
    std::function<bool()> downSpeedFunc)
{
    // Create and return a RunCommand that sets the flywheel speed based on the input functions
    return frc2::RunCommand(
        [&]()
        {
            // Persists between each time the command is scheduled
            static units::turns_per_second_t targetSpeed = constants::volcano::targetFlywheelSpeed;

            if (upSpeedFunc())
            {
                targetSpeed += 100_tps;
            }
            else if (downSpeedFunc())
            {
                targetSpeed -= 100_tps;
            }
            volcano->SetFlywheel(targetSpeed);
        },
        { volcano }
    ).ToPtr();
}
#pragma endregion