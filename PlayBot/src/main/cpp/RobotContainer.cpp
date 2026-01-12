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
    m_chassis.SetDefaultCommand(ChassisDrive(&m_chassis, GetChassisSpeeds()));

    // m_led.SetPattern(LEDType::GRADIENT, {frc::Color::kPurple, frc::Color::kGold});

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


    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    // Set the resolution and frame rate of the camera
    camera.SetResolution(640, 480); // Set resolution to 640x480
    camera.SetFPS(30);             // Set frame rate to 30 FPS
}
#pragma endregion

#pragma region GetChassisSpeeds
/// @brief Method to return the chassis speeds based on joystick inputs.
/// @return The chassis speeds based on joystick inputs.
std::function<frc::ChassisSpeeds()> RobotContainer::GetChassisSpeeds()
{
    return [&]()
    {
        // Return the chassis speeds based on joystick inputs
        return frc::ChassisSpeeds{
            -constants::swerve::maxSpeed           * frc::ApplyDeadband(-m_driveController.GetRawAxis(1), constants::controller::TranslationDeadZone),
            -constants::swerve::maxSpeed           * frc::ApplyDeadband(-m_driveController.GetRawAxis(0), constants::controller::TranslationDeadZone),
             constants::swerve::maxAngularVelocity * frc::ApplyDeadband(-m_driveController.GetRawAxis(4), constants::controller::RotateDeadZone)
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
