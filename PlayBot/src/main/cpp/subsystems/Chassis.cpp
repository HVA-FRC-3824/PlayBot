#include "subsystems/Chassis.h"

#pragma region Chassis
/// @brief Chassis class constructor.
Chassis::Chassis()
{
    // Set the swerve modules to their forward angles
    ResetWheelAnglesToZero();
}
#pragma endregion

#pragma region Periodic
/// @brief Method called once per scheduler run.
void Chassis::Periodic()
{
    // Update odometry
    // Update the pose estimator
    m_poseEstimator.Update(GetHeading(), GetModulePositions());

    // This also updates the pose estimator with vision as well as updating photonvisions internal estimators
    m_cameraRight.Periodic();

    // Process camera results for both cameras
    //ProcessCameraResults(m_cameraRight, "Right");
    ProcessCameraResults(m_cameraLeft,  "Left");

    // Logging
    Log("Swerve Module States ",         GetModuleStates());
    Log("Desired Swerve Module States ", m_desiredStates);

    Log("Swerve Module Positions ", GetModulePositions());

    Log("Desired Chassis Speeds ", m_desiredSpeeds);
    Log("Actual Chassis Speeds ",  m_kinematics.ToChassisSpeeds(GetModuleStates()));

    Log("Robot Pose ", GetPose());

    Log("Nearest Tag Pose ", GetNearestTag());
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the chassis with the specified speeds.
/// @param speeds The desired chassis speeds.
void Chassis::Drive(const frc::ChassisSpeeds& speeds)
{
    frc::SmartDashboard::PutNumber("Drive X",     speeds.vx.value());
    frc::SmartDashboard::PutNumber("Drive Y",     speeds.vy.value());
    frc::SmartDashboard::PutNumber("Drive Omega", speeds.omega.value()); 

    // Log the desired speeds
    m_desiredSpeeds = speeds;

    // Set the module states
    m_desiredStates = m_kinematics.ToSwerveModuleStates(m_isFieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading()) : speeds);

    // Set the desired state for each swerve module
    m_swerveModules[0].SetDesiredState(m_desiredStates[0]);
    m_swerveModules[1].SetDesiredState(m_desiredStates[1]);
    m_swerveModules[2].SetDesiredState(m_desiredStates[2]);
    m_swerveModules[3].SetDesiredState(m_desiredStates[3]);

    // Simulate the gyro in simulation
    // if (frc::RobotBase::IsSimulation())
    //    m_gyro.SimPeriodic(speeds.omega);
}
#pragma endregion

#pragma region ZeroHeading
/// @brief Method to zero the robot heading.
void Chassis::ZeroHeading()
{
    // Zero the gyro heading
    m_gyro.ResetYaw();
}
#pragma endregion

#pragma region ResetWheelAnglesToZero 
/// @brief Method to reset the wheel angles to zero.
void Chassis::ResetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_swerveModules[0].SetWheelAngleToForward(constants::swerve::frontLeftForwardAngle);
    m_swerveModules[1].SetWheelAngleToForward(constants::swerve::frontRightForwardAngle);
    m_swerveModules[2].SetWheelAngleToForward(constants::swerve::rearLeftForwardAngle);
    m_swerveModules[3].SetWheelAngleToForward(constants::swerve::rearRightForwardAngle);
}
#pragma endregion

#pragma region ResetDriveEncoders
/// @brief Method to reset the drive encoders.
void Chassis::ResetDriveEncoders()
{
    // Reset the swerve motor encoders
    for (auto& swerveModule : m_swerveModules)
    {
        swerveModule.ResetDriveEncoder();
    }
}
#pragma endregion

#pragma region GetModuleStates
/// @brief Method to get the current swerve module states.
/// @return The current swerve module states.
wpi::array<frc::SwerveModuleState, 4> Chassis::GetModuleStates()
{
    // Return the swerve module states
    return wpi::array<frc::SwerveModuleState, 4>
    {
        m_swerveModules[0].GetState(),
        m_swerveModules[1].GetState(),
        m_swerveModules[2].GetState(),
        m_swerveModules[3].GetState()
    };
}
#pragma endregion

#pragma region GetModulePositions
/// @brief Method to get the current swerve module positions.
/// @return The current swerve module positions.
wpi::array<frc::SwerveModulePosition, 4> Chassis::GetModulePositions()
{
    // Return the swerve module states
    return wpi::array<frc::SwerveModulePosition, 4>
    {
        m_swerveModules[0].GetPosition(),
        m_swerveModules[1].GetPosition(),
        m_swerveModules[2].GetPosition(),
        m_swerveModules[3].GetPosition()
    };
}
#pragma endregion

#pragma region FlipFieldCentric
/// @brief Method to flip the field centric mode.
void Chassis::FlipFieldCentric()
{
    // Toggle the field relative mode
    m_isFieldRelative = !m_isFieldRelative;
}
#pragma endregion

#pragma region GetHeading
/// @brief Method to get the robot heading.
/// @return The robot heading.
frc::Rotation2d Chassis::GetHeading()
{
    // Return the gyro rotation
    return m_gyro.GetRotation().ToRotation2d();
}
#pragma endregion

#pragma region GetPose
/// @brief Method to get the robot pose.
/// @return The robot pose.
frc::Pose2d Chassis::GetPose()
{
    // Return the estimated robot pose
    return m_poseEstimator.GetEstimatedPosition();
}
#pragma endregion

#pragma region ProcessCameraResults
/// @brief Method to process camera results and display on SmartDashboard.
/// @param camera The PhotonCamera object to process.
/// @param cameraName The name of the camera for SmartDashboard keys.
void Chassis::ProcessCameraResults(photon::PhotonCamera& camera, const std::string& cameraName) 
{
    // Get all unread results from the camera
    auto results = camera.GetAllUnreadResults();

    // Determine if any targets were found
    if (results.size() > 0) 
    {
        frc::SmartDashboard::PutNumber(cameraName + " Number of Results", results.size());

        // Get the last one in the list.
        auto result = results[results.size() - 1];

        // Check if the result has targets
        bool hasTarget = result.HasTargets();
        frc::SmartDashboard::PutBoolean(cameraName + " Has Target", hasTarget);

        // If there is a target, process it
        if (hasTarget) 
        {
            // Get the best target
            auto target = result.GetBestTarget();

            // Extract target information
            double yaw        = target.GetYaw();
            double pitch      = target.GetPitch();
            double area       = target.GetArea();
            double skew       = target.GetSkew();
            double confidence = target.GetDetectedObjectConfidence();
            int fiducialId    = target.GetFiducialId();

            frc::SmartDashboard::PutNumber(cameraName + " Target Yaw", yaw);
            frc::SmartDashboard::PutNumber(cameraName + " Target Pitch", pitch);
            frc::SmartDashboard::PutNumber(cameraName + " Target Area", area);
            frc::SmartDashboard::PutNumber(cameraName + " Target Skew", skew);
            frc::SmartDashboard::PutNumber(cameraName + " Target Confidence", confidence);
            frc::SmartDashboard::PutNumber(cameraName + " Target Fiducial ID", fiducialId);

            // Get result latency
            auto latency = result.GetLatency();

            frc::SmartDashboard::PutNumber(cameraName + " Result Latency", latency.value());

            // Get pose ambiguity
            double poseAmbuguity = target.GetPoseAmbiguity();

            frc::SmartDashboard::PutNumber(cameraName + " Ambiguity", poseAmbuguity);

            // Check for low pose ambiguity
            if (poseAmbuguity < 0.2) 
            {
                // Get the best camera-to-target pose
                auto pose = target.GetBestCameraToTarget();

                // Extract position (x, y, z) in meters
                double x = pose.X().value();
                double y = pose.Y().value();
                double z = pose.Z().value();

                // Extract orientation (roll, pitch, yaw) in degrees
                double roll  = pose.Rotation().Axis().x();
                double pitch = pose.Rotation().Axis().y();
                double yaw   = pose.Rotation().Axis().z();

                // Display position and orientation on SmartDashboard
                frc::SmartDashboard::PutNumber(cameraName + " X (m)", x);
                frc::SmartDashboard::PutNumber(cameraName + " Y (m)", y);
                frc::SmartDashboard::PutNumber(cameraName + " Z (m)", z);
                frc::SmartDashboard::PutNumber(cameraName + " Roll (deg)", roll);
                frc::SmartDashboard::PutNumber(cameraName + " Pitch (deg)", pitch);
                frc::SmartDashboard::PutNumber(cameraName + " Yaw (deg)", yaw);
            } 
        } 
        else 
        {
            frc::SmartDashboard::PutString(cameraName + " Pose Status", "No Targets");
        }
    }
}
#pragma endregion

#pragma region GetNearestTag
/// @brief Method to get the nearest AprilTag pose.
/// @return The nearest AprilTag pose.
frc::Pose2d Chassis::GetNearestTag()
{
    return GetPose().Nearest(constants::vision::AprilTagLocations::Pose2dTagsSpan);
}
#pragma endregion
