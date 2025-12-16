#pragma once

#pragma region Includes
#include <wpi/array.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "lib/Logging.h"
#include "lib/hardware/gyro/Navx.h"
#include "lib/hardware/vision/PhotonVision.h"
#include "lib/subsystem/SwerveModule.h"

#include "Constants.h"
#pragma endregion

class Chassis : public frc2::SubsystemBase
{
    public:

        explicit                                 Chassis();

        void                                     Periodic() override;

        void                                     Drive(const frc::ChassisSpeeds& speeds);

        void                                     ZeroHeading();
    
        void                                     ResetWheelAnglesToZero();
        void                                     ResetDriveEncoders();

        wpi::array<frc::SwerveModuleState, 4>    GetModuleStates();
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();
    
        void                                     FlipFieldCentric();

        frc::Rotation2d                          GetHeading();
        frc::Pose2d                              GetPose();

        frc::Pose2d                              GetNearestTag();
        
    private:
        
        // Swerve module order for kinematics calculations
        //
        //         Front          Translation2d Coordinates
        //   FL +----------+ FR              ^ X
        //      | 0      1 |                 |
        //      |          |            Y    |
        //      |          |          <------+-------
        //      | 2      3 |                 |
        //   RL +----------+ RR              |
        
        std::array<subsystem::SwerveModule, 4> m_swerveModules
        {
            subsystem::SwerveModule{constants::swerve::frontLeftDriveCANid,  constants::swerve::frontLeftTurnCANid,  constants::swerve::frontLeftEncoderCANid,  
                                    constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig},
            subsystem::SwerveModule{constants::swerve::frontRightDriveCANid, constants::swerve::frontRightTurnCANid, constants::swerve::frontRightEncoderCANid,
                                    constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig},
            subsystem::SwerveModule{constants::swerve::backLeftDriveCANid,   constants::swerve::backLeftTurnCANid,   constants::swerve::backLeftEncoderCANid,
                                    constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig},
            subsystem::SwerveModule{constants::swerve::backRightDriveCANid,  constants::swerve::backRightTurnCANid,  constants::swerve::backRightEncoderCANid,  
                                    constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig}
        };

        frc::SwerveDriveKinematics<4> m_kinematics
        {
            frc::Translation2d{+constants::swerve::wheelBase / 2, +constants::swerve::trackWidth / 2}, // Front Left
            frc::Translation2d{+constants::swerve::wheelBase / 2, -constants::swerve::trackWidth / 2}, // Front Right
            frc::Translation2d{-constants::swerve::wheelBase / 2, +constants::swerve::trackWidth / 2}, // Back Left
            frc::Translation2d{-constants::swerve::wheelBase / 2, -constants::swerve::trackWidth / 2}  // Back Right
        };

        frc::SwerveDrivePoseEstimator<4> m_poseEstimator
        {
            m_kinematics,                                 // Kinematics object
            frc::Rotation2d(),                            // Initial gyro angle
            std::array<frc::SwerveModulePosition, 4>{},   // Initial module positions
            frc::Pose2d()                                 // Initial pose
        };

        frc::ChassisSpeeds                    m_desiredSpeeds{0_mps, 0_mps, 0_rad_per_s};

        wpi::array<frc::SwerveModuleState, 4> m_desiredStates = GetModuleStates();
            
        bool                                  m_isFieldRelative = true;
    
        hardware::gyro::Navx                  m_gyro{};

        PhotonVision m_vision
        {
            constants::vision::CameraName,
            constants::vision::RobotToCam,
            constants::vision::TagLayout,
            constants::vision::SingleTagStdDevs,
            constants::vision::MultiTagStdDevs,

            // Pose consumer to add vision measurements to the pose estimator
            [this] (frc::Pose2d pose, units::second_t timestamp, Eigen::Matrix<double, 3, 1> stddevs)
            {
                m_poseEstimator.AddVisionMeasurement(pose, timestamp, {stddevs[0], stddevs[1], stddevs[2]});
            }
        };
};
