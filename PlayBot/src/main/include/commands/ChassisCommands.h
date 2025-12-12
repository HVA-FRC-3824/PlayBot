#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "subsystems/Chassis.h"
#pragma endregion

/// @brief Creates a command to zero the heading of the gyro.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that resets the gyro yaw to zero.
frc2::CommandPtr ChassisZeroHeading(Chassis* chassis);

 /// @brief Creates a command to drive the chassis using the provided speeds supplier.
///  @param chassis A pointer to the chassis subsystem.
///  @param chassisSpeedsSupplier A function that supplies the desired chassis speeds.
///  @return A CommandPtr that executes the chassis drive functionality.
frc2::CommandPtr ChassisDrive(Chassis* chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier);

/// @brief Creates a command to drive the chassis to a specified pose.
/// @param chassis A pointer to the chassis subsystem.
/// @param CommandName The name of the command or path to follow.
/// @return A CommandPtr that drives the chassis to the specified pose.
frc2::CommandPtr ChassisDrivePose(Chassis* chassis, std::string CommandName);

/// @brief Creates a command to drive the chassis to a specified pose.
/// @param chassis A pointer to the chassis subsystem.
/// @param targetPose The target pose to drive to. End goal state relative to the origin, blue alliance side.
/// @return A CommandPtr that drives the chassis to the specified pose.
frc2::CommandPtr ChassisDrivePose(Chassis* chassis, frc::Pose2d targetPose);

/// @brief Creates a command to flip the field centricity of the chassis.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that flips the field centricity.
frc2::CommandPtr FlipFieldCentricity(Chassis* chassis);

// This command will align the robot to the nearest AprilTag
// It will use the AprilTag's pose to determine the target position and rotation
// The robot will drive towards the target position and rotate to face the target rotation
frc2::CommandPtr AlignToNearestTag(Chassis* chassis, frc::Transform2d targetOffset);
