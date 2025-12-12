#include "RobotContainer.h"

// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = nullptr;

#pragma region GetInstance
/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer *RobotContainer::GetInstance()
{
    // Detrermine if the class has already been instantiated
    if (m_robotContainer == nullptr)
    {
        // Instantiate the class
        m_robotContainer = new RobotContainer();
    }

    // Return the class pointer
    return m_robotContainer;
}
#pragma endregion

#pragma region RobotContainer
/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer()
{
    // // Configure the chassis default command
    m_chassis.SetDefaultCommand(frc2::RunCommand(
        [this]()
        {
            // Call the chassis drive method with the desired speeds
            m_chassis.Drive(GetChassisSpeeds()());  // Add () to invoke the function
        },
        {&m_chassis} // Subsystem requirements
    ));

    // Array of run-once controls, organized like this for simplicity and readability
    std::pair<Button, frc2::CommandPtr> runOnceControls[] =
    {
        {constants::controller::A,           ChassisZeroHeading(&m_chassis)},
        {constants::controller::B,           FlipFieldCentricity(&m_chassis)},
        {constants::controller::RightBumper, VolcanoFlywheelOn(&m_volcano)},
        {constants::controller::LeftBumper,  VolcanoFlywheelOff(&m_volcano)},
        // {constants::controller::X,           VolcanoShootOneBall(&m_volcano)}
    };

    // // Configure the run-once controls
    // for (auto& [button, command] : runOnceControls)
    // {
    //     frc2::JoystickButton(&m_driveController, int(button)).OnTrue(std::move(command));
    // }

    // // Configure the hold control
    // frc2::JoystickButton(&m_driveController, constants::controller::Y)
    //     .OnTrue (std::move(VolcanoShootAllBalls(&m_volcano)))
    //     .OnFalse(std::move(VolcanoStopAll(&m_volcano)));

    // // Configure the manual flywheel control, this control scheme may be a bit schizo
    // frc2::Trigger([this] () { return m_driveController.GetPOV() == constants::controller::Pov_180; })
    //     .OnTrue(frc2::InstantCommand{[this]() { m_isManualFlywheelControl = true; }}.ToPtr());
    // frc2::Trigger([this] () { return m_driveController.GetPOV() == constants::controller::Pov_0; })
    //     .OnTrue(frc2::InstantCommand{[this]() { m_isManualFlywheelControl = false; }}.ToPtr());

    // While manual mode is enabled, the speed is controlled by the D-pad up and down
    frc2::Trigger([this]() { return m_isManualFlywheelControl; })
        .WhileTrue(std::move(VolcanoVariableFlywheelSpeed(
            &m_volcano,
            [this] () { return m_driveController.GetPOV() == constants::controller::Pov_90; }, // Up on D-pad increases speed
            [this] () { return m_driveController.GetPOV() == constants::controller::Pov_270; } // Down on D-pad decreases speed
        )));
}
#pragma endregion

#pragma region GetChassisSpeeds
/// @brief Method to return the chassis speeds based on joystick inputs.
/// @return The chassis speeds based on joystick inputs.
std::function<frc::ChassisSpeeds()> RobotContainer::GetChassisSpeeds()
{
    // Return the chassis speeds based on joystick inputs
    return [this] () -> frc::ChassisSpeeds
    {
        // Return the chassis speeds based on joystick inputs
        return frc::ChassisSpeeds{
            -constants::swerve::maxSpeed           * frc::ApplyDeadband(m_driveController.GetRawAxis(1), constants::controller::TranslationDeadZone),
            -constants::swerve::maxSpeed           * frc::ApplyDeadband(m_driveController.GetRawAxis(0), constants::controller::TranslationDeadZone),
             constants::swerve::maxAngularVelocity * frc::ApplyDeadband(m_driveController.GetRawAxis(4), constants::controller::RotateDeadZone)
        };
    };
}
#pragma endregion

#pragma region GetExponentialValue
/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resulting exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = (joystickValue < 0.0) ? -1 : 1;
    double absValue  = std::abs(joystickValue);
    double output    = std::pow(absValue, exponent) * direction;

    // Ensure the range of the output
    if (output < -1.0) output = -1.0;
    if (output > 1.0)  output =  1.0;

    // Return the output value
    return output;
}
#pragma endregion


#pragma region ProcessCameraResults
/// @brief Method to process camera results and display on SmartDashboard.
/// @param camera The PhotonCamera object to process.
/// @param cameraName The name of the camera for SmartDashboard keys.
void RobotContainer::ProcessCameraResults(photon::PhotonCamera& camera, const std::string& cameraName) 
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