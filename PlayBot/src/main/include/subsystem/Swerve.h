#pragma once

#include <frc2/command/SubsystemBase.h>

#include <wpi/array.h>
#include <units/angle.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "lib/subsystem/SwerveModule.h"

#include "subsystem/Vision.h"
#include "subsystem/Gyro.h"

#include "Constants.h"


class Swerve : public frc2::SubsystemBase
{
    public:
    
        static Swerve* GetInstance()
        {
            static Swerve instance;
            static Swerve* instancePtr = &instance;
            return instancePtr;
        }

        inline void Drive(frc::ChassisSpeeds speeds)
        {
            m_loggedDesiredSpeedsPublisher.Set(speeds);

            // Set the module states
            auto m_desiredStates = m_kinematics.ToSwerveModuleStates(
                m_isFieldRelative ? 
                    frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading()) 
                    : speeds);

            // Set the desired state for each swerve module
            m_swerveModules[0].SetDesiredState(m_desiredStates[0]);
            m_swerveModules[1].SetDesiredState(m_desiredStates[1]);
            m_swerveModules[2].SetDesiredState(m_desiredStates[2]);
            m_swerveModules[3].SetDesiredState(m_desiredStates[3]);

            if (frc::RobotBase::IsSimulation())
                Gyro::GetInstance()->SimPeriodic(speeds.omega);
        }

        inline void Periodic() override
        {
            // This method will be called once per scheduler run
            m_loggedModuleStatePublisher.Set(
                wpi::array<frc::SwerveModuleState, 4>{
                    m_swerveModules[0].GetState(),
                    m_swerveModules[1].GetState(),
                    m_swerveModules[2].GetState(),
                    m_swerveModules[3].GetState()
                }
            );

            m_loggedPosePublisher.Set( GetPose2d() );

            OdometryPeriodic();
        }

        inline void ResetWheelAnglesToZero()
        {
            // Set the swerve wheel angles to zero
            m_swerveModules[0].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.frontLeftForwardAngle);
            m_swerveModules[1].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.frontRightForwardAngle);
            m_swerveModules[2].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.rearLeftForwardAngle);
            m_swerveModules[3].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.rearRightForwardAngle);
        }

        inline void ResetDriveEncoders()
        {
            // Reset the swerve motor encoders
            m_swerveModules[0].ResetDriveEncoder();
            m_swerveModules[1].ResetDriveEncoder();
            m_swerveModules[2].ResetDriveEncoder();
            m_swerveModules[3].ResetDriveEncoder();
        }

        inline wpi::array<frc::SwerveModuleState, 4> GetModuleStates()
        {
            // Return the swerve module states
            std::array<frc::SwerveModuleState, 4> states =
            {
                m_swerveModules[0].GetState(),
                m_swerveModules[1].GetState(),
                m_swerveModules[2].GetState(),
                m_swerveModules[3].GetState()
            };

            if (frc::RobotBase::IsSimulation())
            {
                for (auto state : states)
                {
                    state = {state.speed / constants::swerve::RobotSwerveConfig.driveConversion.value(), 
                             state.angle / constants::swerve::RobotSwerveConfig.angleConversion.value()};
                }
            }

            return states;
        }

        std::array<frc::SwerveModulePosition, 4> GetModulePositions()
        {
            // Return the swerve module states
            std::array<frc::SwerveModulePosition, 4> positions =
            {
                m_swerveModules[0].GetPosition(),
                m_swerveModules[1].GetPosition(),
                m_swerveModules[2].GetPosition(),
                m_swerveModules[3].GetPosition()
            };

            if (frc::RobotBase::IsSimulation())
            {
                for (auto position : positions)
                {
                    position = {position.distance        / constants::swerve::RobotSwerveConfig.driveConversion.value(), 
                                position.angle / constants::swerve::RobotSwerveConfig.angleConversion.value()};
                }
            }

            return positions;
        }

        inline void FlipFieldCentric()
        {
            m_isFieldRelative = !m_isFieldRelative;
        }

        frc::Rotation2d GetHeading()
        {
            return Gyro::GetInstance()->GetRotation().ToRotation2d();
        }

        frc::Pose2d GetPose2d()
        {
            return m_poseEstimator.GetEstimatedPosition();
        }

    protected:

        explicit Swerve() :
                m_kinematics{
                frc::Translation2d{+constants::swerve::RobotSwerveConfig.wheelBase / 2, +constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Front Left
                frc::Translation2d{+constants::swerve::RobotSwerveConfig.wheelBase / 2, -constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Front Right
                frc::Translation2d{-constants::swerve::RobotSwerveConfig.wheelBase / 2, +constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Back Left
                frc::Translation2d{-constants::swerve::RobotSwerveConfig.wheelBase / 2, -constants::swerve::RobotSwerveConfig.trackWidth / 2}  // Back Right
                },
                m_swerveModules{
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.frontLeftDriveCANid,  constants::swerve::RobotSwerveConfig.frontLeftTurnCANid,  constants::swerve::RobotSwerveConfig.frontLeftEncoderCANid,  constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.frontRightDriveCANid, constants::swerve::RobotSwerveConfig.frontRightTurnCANid, constants::swerve::RobotSwerveConfig.frontRightEncoderCANid, constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.backLeftDriveCANid,   constants::swerve::RobotSwerveConfig.backLeftTurnCANid,   constants::swerve::RobotSwerveConfig.backLeftEncoderCANid,   constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
                subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.backRightDriveCANid,  constants::swerve::RobotSwerveConfig.backRightTurnCANid,  constants::swerve::RobotSwerveConfig.backRightEncoderCANid,  constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion}
                },
                m_poseEstimator{m_kinematics, frc::Rotation2d(), std::array<frc::SwerveModulePosition, 4>{}, frc::Pose2d()},
                m_isFieldRelative{true},

                m_vision{
                [this] (frc::Pose2d pose, units::second_t timestamp, Eigen::Matrix<double, 3, 1> stddevs)
                {
                    m_poseEstimator.AddVisionMeasurement(pose, timestamp, {stddevs[0], stddevs[1], stddevs[2]});
                }},
                
                m_loggedModuleStatePublisher{nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/Data/SwerveStates").Publish()},
                m_loggedPosePublisher{nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/Data/CurrentPose").Publish()},
                m_loggedDesiredSpeedsPublisher{nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("/Data/DesiredSpeeds").Publish()}
        {
            // Set the swerve modules to their forward angles
            ResetWheelAnglesToZero();
        }

        void OdometryPeriodic()
        {
            // Update the pose estimator
            m_poseEstimator.Update(
                GetHeading(),
                GetModulePositions()
            );

            // This ALSO updates the pose estimator with vision stuff as well as updating photonvisions internal estimators and etc
            m_vision.Periodic();
        }

        frc::SwerveDriveKinematics<4>           m_kinematics;
        std::array<subsystem::SwerveModule, 4>  m_swerveModules;
        
        frc::SwerveDrivePoseEstimator<4>        m_poseEstimator;   
        bool                                    m_isFieldRelative;

        PhotonVision                            m_vision;

        nt::StructArrayPublisher<frc::SwerveModuleState> m_loggedModuleStatePublisher;
        nt::StructPublisher<frc::Pose2d>                 m_loggedPosePublisher;
        nt::StructPublisher<frc::ChassisSpeeds>          m_loggedDesiredSpeedsPublisher;

};