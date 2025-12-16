#include "commands/ChassisCommands.h"

#pragma region FlipFieldCentricity(Chassis* chassis)
/// @brief Creates a command to flip the field centricity of the chassis.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that flips the field centricity.
frc2::CommandPtr FlipFieldCentricity(Chassis* chassis)
{
    // Create and return a InstantCommand that flips the field centricity
    return frc2::InstantCommand{
        [chassis] () { chassis->FlipFieldCentric(); }, // Execution function
        { chassis } // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisZeroHeading(Chassis* chassis)
/// @brief Creates a command to zero the heading of the gyro.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that resets the gyro yaw to zero.
frc2::CommandPtr ChassisZeroHeading(Chassis* chassis)
{
    // Create and return a InstantCommand that resets the gyro yaw
    return frc2::InstantCommand{
        [chassis] () { chassis->ZeroHeading(); },
        { chassis } // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisDrive(Chassis* chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier)
 /// @brief Creates a command to drive the chassis using the provided speeds supplier.
///  @param chassis A pointer to the chassis subsystem.
///  @param chassisSpeedsSupplier A function that supplies the desired chassis speeds.
///  @return A CommandPtr that executes the chassis drive functionality.
frc2::CommandPtr ChassisDrive(Chassis* chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier)
{
    // Create and return a repeating InstantCommand that drives the chassis
    return frc2::InstantCommand
    {
        [chassis, chassisSpeedsSupplier] () { chassis->Drive(chassisSpeedsSupplier()); },  // Execution function (runs repeatedly while the command is active)
        { chassis }                                                                        // Requirements (subsystems required by this command)
    }.ToPtr().Repeatedly();
    // because of how we implement it, I'm not sure if it needs to be .Repeatedly()'d but it won't hurt
}
#pragma endregion

#pragma region ChassisDrivePose(Chassis* chassis, std::string CommandName)
/// @brief Creates a command to drive the chassis to a specified pose.
/// @param chassis A pointer to the chassis subsystem.
/// @param CommandName The name of the command or path to follow.
/// @return A CommandPtr that drives the chassis to the specified pose.
frc2::CommandPtr ChassisDrivePose(Chassis* chassis, std::string CommandName)
{
    //return AutoBuilder::followPath(PathPlannerPath::fromPathFile(CommandName));

    // Note: Temporary fix for pathplanner not working correctly when called immediately after another command
    return frc2::WaitCommand(0.1_s).ToPtr();
}
#pragma endregion

#pragma region ChassisDrivePose(Chassis* chassis, frc::Pose2d targetPose)
/// @brief Creates a command to drive the chassis to a specified pose.
/// @param chassis A pointer to the chassis subsystem.
/// @param targetPose The target pose to drive to. End goal state relative to the origin, blue alliance side.
/// @return A CommandPtr that drives the chassis to the specified pose.
frc2::CommandPtr ChassisDrivePose(Chassis* chassis, frc::Pose2d targetPose)
{
    // return AutoBuilder::pathfindToPose(targetPose, constants::PathPlanner::Constraints);

    // Note: Temporary fix for pathplanner not working correctly when called immediately after another command
    return frc2::WaitCommand(0.1_s).ToPtr();
}
#pragma endregion

#pragma region AlignToNearestTag(Chassis* chassis, frc::Transform2d targetOffset)
// This command will align the robot to the nearest AprilTag
// It will use the AprilTag's pose to determine the target position and rotation
// The robot will drive towards the target position and rotate to face the target rotation
frc2::CommandPtr AlignToNearestTag(Chassis* chassis, frc::Transform2d targetOffset)
{
        frc::Pose2d targetPosition = chassis->GetNearestTag();

        // Rotate offset and offset it relative to the target position's orientation
        // This does actually work trust chat
        frc::Pose2d targetWithOffset{
            targetPosition.X() + targetOffset.Translation().X() * std::cos(targetPosition.Rotation().Radians().value())
                               - targetOffset.Translation().Y() * std::sin(targetPosition.Rotation().Radians().value()),

            targetPosition.Y() + targetOffset.Translation().X() * std::sin(targetPosition.Rotation().Radians().value())
                               + targetOffset.Translation().Y() * std::cos(targetPosition.Rotation().Radians().value()),

            targetPosition.Rotation().Degrees() + targetOffset.Rotation().Degrees()
        };

    return ChassisDrivePose(chassis, targetWithOffset);
}
#pragma endregion
