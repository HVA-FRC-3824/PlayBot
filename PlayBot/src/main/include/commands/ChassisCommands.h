#pragma once

#pragma region Includes

#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

#include "subsystems/Chassis.h"
#pragma endregion

#pragma region ChassisZeroHeading(Chassis* chassis)
/// @brief Creates a command to zero the heading of the gyro.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that resets the gyro yaw to zero.
inline frc2::CommandPtr ChassisZeroHeading(Chassis* chassis)
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
inline frc2::CommandPtr ChassisDrive(Chassis* chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier)
{
    // Create and return a repeating InstantCommand that drives the chassis
    return frc2::InstantCommand{
        [chassis, chassisSpeedsSupplier] () { chassis->Drive(chassisSpeedsSupplier()); }, // Execution function (runs repeatedly while the command is active)
        { chassis }                                                                       // Requirements (subsystems required by this command)
    }.ToPtr().Repeatedly();
}
#pragma endregion

#pragma region ChassisDrivePose(Chassis* chassis, frc::Pose2d targetPose)
/// @brief Creates a command to drive the chassis to a specified pose.
/// @param chassis A pointer to the chassis subsystem.
/// @param targetPose The target pose to drive to. End goal state relative to the origin, blue alliance side.
/// @return A CommandPtr that drives the chassis to the specified pose.
// TODO: UPDATE std::vector to use ... to do variable amounts of poses
inline frc2::CommandPtr ChassisDrivePose(Chassis* chassis, std::vector<frc::Pose2d> targetPose)
{
    // Set up config for trajectory
    frc::TrajectoryConfig config(constants::swerve::maxSpeed,
                                constants::swerve::maxAngularVelocity);

    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(chassis->GetKinematics());

    frc::Pose2d finalPosition = targetPose.back();
    targetPose.pop_back(); // Remove the end pose
    
    std::vector<frc::Translation2d> waypoints{};
    waypoints.reserve(targetPose.size() - 1);
    for (int i = 0; i < targetPose.size() - 1; i++)
    {
        waypoints[i] = targetPose[i].Translation();
    }

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        chassis.GetPose(),
        // Pass through these two interior waypoints, making an 's' curve path
        waypoints,
        // End 3 meters straight ahead of where we started, facing forward
        targetPose.back(),
        // Pass the config
        config
    );

    frc::ProfiledPIDController<units::radians> thetaController
    {
        1, 0, 0, 
        frc::ProfiledPIDController<units::radians>::Constraints
            {constants::swerve::maxAngularVelocity, constants::swerve::maxAngularVelocity / 2}
    };

    thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                          units::radian_t{ std::numbers::pi});

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return frc2::SwerveControllerCommand<4>(
        exampleTrajectory,
        
        [&]() { return chassis->GetPose(); },

        chassis->GetKinematics(),

        frc::PIDController{1, 0, 0},
        frc::PIDController{1, 0, 0},
        thetaController,

        [&] (auto moduleStates) { chassis->SetModuleStates(moduleStates); },

        { chassis }
    ).AndThen(
        frc2::InstantCommand(
            [&] { chassis->Drive(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}); }, 
            { chassis }
        ).ToPtr()
    );
}
#pragma endregion

#pragma region FlipFieldCentricity(Chassis* chassis)
/// @brief Creates a command to flip the field centricity of the chassis.
/// @param chassis A pointer to the chassis subsystem. 
/// @return A CommandPtr that flips the field centricity.
inline frc2::CommandPtr FlipFieldCentricity(Chassis* chassis)
{
    // Create and return a InstantCommand that flips the field centricity
    return frc2::InstantCommand{
        [chassis] () { chassis->FlipFieldCentric(); }, // Execution function
        { chassis } // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region AlignToNearestTag(Chassis* chassis, frc::Transform2d targetOffset)
// This command will align the robot to the nearest AprilTag
// It will use the AprilTag's pose to determine the target position and rotation
// The robot will drive towards the target position and rotate to face the target rotation
inline frc2::CommandPtr AlignToNearestTag(Chassis* chassis, frc::Transform2d targetOffset)
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

    return ChassisDrivePose(chassis, std::vector<frc::Pose2d>{targetWithOffset});
}
#pragma endregion
