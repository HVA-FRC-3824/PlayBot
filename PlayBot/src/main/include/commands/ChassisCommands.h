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

frc2::CommandPtr FlipFieldCentricity(Chassis* chassis);

frc2::CommandPtr ChassisZeroHeading(Chassis* chassis);

frc2::CommandPtr ChassisDrive(Chassis* chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier);

frc2::CommandPtr ChassisDrivePose(Chassis* chassis, std::string CommandName);
frc2::CommandPtr ChassisDrivePose(Chassis* chassis, frc::Pose2d targetPose);

frc2::CommandPtr AlignToNearestTag(Chassis* chassis, frc::Transform2d targetOffset);
