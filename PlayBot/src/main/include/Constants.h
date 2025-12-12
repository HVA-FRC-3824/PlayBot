#pragma once

#pragma region Includes
#include <array>
#include <numbers>

#include <units/base.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/voltage.h>

#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "lib/hardware/motors/Motor.h"
#pragma endregion

typedef int Button;

namespace constants
{
    #pragma region Swerve
    namespace swerve
    {
        // Motor and encoders CAN IDs
        constexpr CANid_t frontLeftDriveCANid    = 10;
        constexpr CANid_t frontLeftTurnCANid     = 20;
        constexpr CANid_t frontLeftEncoderCANid  = 30;

        constexpr CANid_t frontRightDriveCANid   = 11;
        constexpr CANid_t frontRightTurnCANid    = 21;
        constexpr CANid_t frontRightEncoderCANid = 31;

        constexpr CANid_t backLeftDriveCANid     = 13;
        constexpr CANid_t backLeftTurnCANid      = 23;
        constexpr CANid_t backLeftEncoderCANid   = 33;

        constexpr CANid_t backRightDriveCANid    = 12;
        constexpr CANid_t backRightTurnCANid     = 22;
        constexpr CANid_t backRightEncoderCANid  = 32;

        // PID, feedforward, and other configurations for the motors
        constexpr hardware::motor::MotorConfiguration driveMotorConfig
        {
            40_A,                                  // Current Limit
            true,                                  // Brake Mode
            (0.0098022 * std::numbers::pi) / 6.75, // Conversion factor
            0.1, 0.02, 0.0,                        // P, I , D
            0.0, 0.0, 0.0                          // S, V, A
        };

        // These need to be tuned istg
        constexpr hardware::motor::MotorConfiguration turnMotorConfig
        {
            30_A,                          // Current Limit
            true,                          // Brake Mode
            (2 * std::numbers::pi) / 21.5, // Conversion factor
            1.0, 0.0, 0.2,                 // P, I , D 
            0.0, 0.0, 0.0                  // S, V, A  
        };

        // All encoders are going to be slightly off, this corrects that
        constexpr units::turn_t frontRightForwardAngle{-0.193604};
        constexpr units::turn_t frontLeftForwardAngle {-0.422119};
        constexpr units::turn_t rearRightForwardAngle {-0.174561};
        constexpr units::turn_t rearLeftForwardAngle  { 0.268555};

        // These make sure to limit how fast the robot can go
        constexpr units::meters_per_second_t  maxSpeed          {4};
        constexpr units::radians_per_second_t maxAngularVelocity{2 * std::numbers::pi};

        // The physical dimensions of the robot
        constexpr units::meter_t wheelBase {25};
        constexpr units::meter_t trackWidth{25};
    }
    #pragma endregion

    #pragma region Volcano
    namespace volcano
    {
        // CAN IDs
        constexpr CANid_t flywheelMotorCANid      = 40;
        constexpr CANid_t firstIndexerMotorCANid  = 41;
        constexpr CANid_t secondIndexerMotorCANid = 42;
        constexpr CANid_t kickerMotorCANid        = 43;

        constexpr int     ballSensorDIOPort       =  0;

        // Motor Configurations
        constexpr hardware::motor::MotorConfiguration flywheelMotorConfig
        {
            30_A,           // Current Limit
            false,          // Brake Mode
            1,              // Conversion factor
            5.0, 0.0, 0.0,  // P, I , D
            0.0, 2.0, 0.0   // S, V, A
        };

        constexpr hardware::motor::MotorConfiguration indexerMotorConfig
        {
            30_A,           // Current Limit
            true,           // Brake Mode
            1,              // Conversion factor
            1.0, 0.0, 0.0,  // P, I , D
            0.0, 0.0, 0.0   // S, V, A
        };

        constexpr hardware::motor::MotorConfiguration kickMotorConfig
        {
            30_A,           // Current Limit
            true,           // Brake Mode
            1,              // Conversion factor
            1.0, 0.0, 0.0,  // P, I , D
            0.0, 0.0, 0.0   // S, V, A
        };

        constexpr double flywheelTolerancePercent = 0.05; // 5% tolerance

        // Target Speeds TODO: TEST AND TUNE
        constexpr units::turns_per_second_t targetFlywheelSpeed{1000};
        constexpr units::turns_per_second_t targetIndexerSpeed{20};
        constexpr units::turns_per_second_t targetKickerSpeed{40};
    }
    #pragma endregion

    #pragma region LED
    namespace led
    {
        constexpr int length = 60U;

        constexpr int port   = 0;
    }
    #pragma endregion

    #pragma region Controller
    namespace controller
    {
        // Drive Input Configurations
        constexpr int    DrivePort           =    0;

        constexpr double TranslationDeadZone = 0.06;
        constexpr double RotateDeadZone      = 0.06;
        constexpr double FlywheelDeadZone    = 0.06;

        constexpr double ExponentForward     = 3.0;
        constexpr double ExponentStrafe      = 3.0;
        constexpr double ExponentAngle       = 3.0;

        // BUTTONSSSSS
        constexpr Button A                   =   1;
        constexpr Button B                   =   2;
        constexpr Button X                   =   3;
        constexpr Button Y                   =   4;
        constexpr Button LeftBumper          =   5;
        constexpr Button RightBumper         =   6;
        constexpr Button Back                =   7;
        constexpr Button Start               =   8;
        constexpr Button LeftStickButton     =   9;
        constexpr Button RightStickButton    =  10;

        constexpr Button Pov_0               =   0;
        constexpr Button Pov_45              =  45;
        constexpr Button Pov_90              =  90;
        constexpr Button Pov_135             = 135;
        constexpr Button Pov_180             = 180;
        constexpr Button Pov_225             = 225;
        constexpr Button Pov_270             = 270;
        constexpr Button Pov_315             = 315;
    }
    #pragma endregion

    #pragma region Vision
    namespace vision
    {
        constexpr std::string_view            CameraName{"PhotonCamera"};

        constexpr frc::Transform3d            RobotToCam{frc::Translation3d{0_m, 4_in, 15_in}, frc::Rotation3d{}};

        const     frc::AprilTagFieldLayout    TagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);

        const     Eigen::Matrix<double, 3, 1> SingleTagStdDevs{4, 4, 8};
        const     Eigen::Matrix<double, 3, 1> MultiTagStdDevs{0.5, 0.5, 1};

        namespace AprilTagLocations
        {
            // These are kept because I really dont want to lose them. REEFSCAPE 2025
            constexpr frc::Pose2d Tags2d[22] = 
            { 
                { 657.37_in,  25.80_in, { 126_deg} }, { 657.37_in, 291.20_in, { 234_deg} },
                { 455.15_in, 317.15_in, { 270_deg} }, { 365.20_in, 241.64_in, {   0_deg} },
                { 365.20_in,  75.39_in, {   0_deg} }, { 530.49_in, 130.17_in, { 300_deg} },
                { 546.87_in, 158.50_in, {   0_deg} }, { 530.49_in, 186.83_in, {  60_deg} },
                { 497.77_in, 186.83_in, { 120_deg} }, { 481.39_in, 158.50_in, { 180_deg} },
                { 497.77_in, 130.17_in, { 240_deg} }, { 33.51_in,   25.80_in, {  54_deg} },
                { 33.51_in,  291.20_in, { 306_deg} }, { 325.68_in, 241.64_in, { 180_deg} },
                { 325.68_in,  75.39_in, { 180_deg} }, { 235.73_in,  -0.15_in, {  90_deg} },
                { 160.39_in, 130.17_in, { 240_deg} }, { 144.00_in, 158.50_in, { 180_deg} },
                { 160.39_in, 186.83_in, { 120_deg} }, { 193.10_in, 186.83_in, { 60_deg,} },
                { 209.49_in, 158.50_in, { 0_deg, } }, { 193.10_in, 130.17_in, { 300_deg} }
            };

            constexpr std::span<const frc::Pose2d> Pose2dTagsSpan{std::begin(Tags2d), std::end(Tags2d)};
        }
    }
    #pragma endregion
}
