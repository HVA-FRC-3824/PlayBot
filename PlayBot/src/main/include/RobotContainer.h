#pragma once

#pragma region Includes
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include <photon/PhotonCamera.h>

#include "subsystems/Chassis.h"
#include "subsystems/Volcano.h"

#include "commands/ChassisCommands.h"
#include "commands/VolcanoCommands.h"

#include "Constants.h"
#pragma endregion

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer *GetInstance();

        photon::PhotonCamera &GetRightCamera() { return cameraRight; }
        photon::PhotonCamera &GetLeftCamera()  { return cameraLeft;  }

        void ProcessCameraResults(photon::PhotonCamera& camera, const std::string& cameraName);

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        std::function<frc::ChassisSpeeds()>                  GetChassisSpeeds();

        double                              GetExponentialValue(double joystickValue, double exponent);

        frc::XboxController                 m_driveController{constants::controller::DrivePort};

        frc::SlewRateLimiter<units::scalar> m_flywheelLimiter{1.0 / 0.1_s};  // Full throttle change in 0.5 seconds

        bool                                m_isManualFlywheelControl = false;

        // Instantiate the robot subsystems
        Chassis                             m_chassis;
        Volcano                             m_volcano;

        photon::PhotonCamera cameraRight{"CameraRight"};
        photon::PhotonCamera cameraLeft{"CameraLeft"};
};
