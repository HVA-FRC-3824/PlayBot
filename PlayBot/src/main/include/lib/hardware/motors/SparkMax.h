#pragma once

#include "Motor.h"

#include <iostream>
#include <numbers>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkLowLevel.h>
#include <rev/config/SparkMaxConfig.h>

namespace hardware
{

namespace motor
{
    
    class SparkMax : public Motor
    {
        
        public:

            inline SparkMax(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfInertia = 0.001_kg_sq_m) 
            : Motor{
                frc::sim::DCMotorSim{
                    frc::LinearSystemId::DCMotorSystem(
                            motorModel,
                            simMomentOfInertia,
                            1
                        ),
                        motorModel
                    }
                },
                m_motor{CANid, rev::spark::SparkLowLevel::MotorType::kBrushless},
                m_angleEncoder{m_motor.GetEncoder()}, 
                m_turnClosedLoopController{m_motor.GetClosedLoopController()},

                m_feedforward{config.S * 1_V, config.V * 1_V * 1_s / 1_tr, config.A * 1_V * 1_s * 1_s / 1_tr},

                m_motorModel{motorModel},
                m_sparkSim{&m_motor, &m_motorModel}
            {
                ConfigureMotor(config);
            }

            inline void ConfigureMotor(MotorConfiguration config) override
            {
                // Configure the angle motor
                rev::spark::SparkMaxConfig sparkMaxConfig{};

                // Configure the motor controller
                sparkMaxConfig
                    .SetIdleMode(config.breakMode 
                                        ? rev::spark::SparkBaseConfig::IdleMode::kBrake 
                                        : rev::spark::SparkBaseConfig::IdleMode::kCoast)
                    .SmartCurrentLimit(config.CurrentLimit.value());

                // Configure encoder conversion factors (ensure units are in turns and turns per second)
                sparkMaxConfig.encoder
                    .PositionConversionFactor(1) // Position in turns
                    .VelocityConversionFactor(1); // Velocity in turns per second (RPM / 60)

                // Configure the closed loop controller
                sparkMaxConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                    .Pid(config.P, config.I, config.D);

                // Write the configuration to the motor controller
                auto status = m_motor.Configure(sparkMaxConfig, 
                                               rev::spark::SparkMax::ResetMode::kResetSafeParameters, 
                                               rev::spark::SparkMax::PersistMode::kPersistParameters);

                // Report configuration status
                if (status != rev::REVLibError::kOk)
                {
                    std::cerr << "***** ERROR: Could not configure SparkMax motor (CAN ID: " 
                              << m_motor.GetDeviceId() << "). Error code: " << static_cast<int>(status) << std::endl;
                }
                else
                {
                    std::cout << "SparkMax motor (CAN ID: " << m_motor.GetDeviceId() 
                              << ") configured successfully." << std::endl;
                }
            }

            inline void SetReferenceState(double motorInput) override
            {
                // Set duty cycle output [-1, 1]
                m_turnClosedLoopController.SetReference(motorInput, 
                                                        rev::spark::SparkMax::ControlType::kDutyCycle,
                                                        rev::spark::ClosedLoopSlot::kSlot0, 
                                                        m_feedforward.Calculate(0_tps).value(),
                                                        rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
            }

            inline void SetReferenceState(units::turns_per_second_t motorInput) override
            {
                // Set velocity control with feedforward
                m_turnClosedLoopController.SetReference(motorInput.value(), 
                                                        rev::spark::SparkMax::ControlType::kVelocity,
                                                        rev::spark::ClosedLoopSlot::kSlot0, 
                                                        m_feedforward.Calculate(motorInput).value(),
                                                        rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
            }

            inline void SetReferenceState(units::volt_t motorInput) override
            {
                // Set voltage control with static friction compensation
                m_turnClosedLoopController.SetReference(motorInput.value() + m_feedforward.Calculate(0_tps).value(),
                                                        rev::spark::SparkMax::ControlType::kVoltage);
            }

            inline void SetReferenceState(units::turn_t motorInput) override
            {
                // Calculate velocity-based feedforward for position control
                // Note: This uses current velocity as a simple approximation
                // For better results, consider using a trajectory follower
                auto currentVelocity = GetVelocity();
                
                // Add support later
                // // Use Smart Motion for smoother position control
                // m_turnClosedLoopController.SetReference(motorInput.value(), 
                //                                         rev::spark::SparkMax::ControlType::kSmartMotion,
                //                                         rev::spark::ClosedLoopSlot::kSlot0, 
                //                                         m_feedforward.Calculate(currentVelocity).value(),
                //                                         rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
                
                // Use standard position control
                m_turnClosedLoopController.SetReference(motorInput.value(), 
                                                        rev::spark::SparkMax::ControlType::kPosition,
                                                        rev::spark::ClosedLoopSlot::kSlot0, 
                                                        m_feedforward.Calculate(currentVelocity).value(),
                                                        rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
            }

            inline units::turn_t GetPosition() override
            {
                if (frc::RobotBase::IsSimulation())
                {
                    // Convert radians to turns
                    return units::turn_t{m_motorSim.GetAngularPosition().value()};
                }
                return units::turn_t{m_angleEncoder.GetPosition()};
            }

            inline units::turns_per_second_t GetVelocity() override
            {
                if (frc::RobotBase::IsSimulation())
                {
                    // Convert radians per second to turns per second
                    return units::turns_per_second_t{m_motorSim.GetAngularVelocity().value()};
                }
                // REV encoder returns velocity in RPM by default, convert to turns per second
                return units::turns_per_second_t{m_angleEncoder.GetVelocity() / 60.0};
            }

            inline void OffsetEncoder(units::turn_t offset) override
            {
                m_angleEncoder.SetPosition(offset.value());
                
                if (frc::RobotBase::IsSimulation())
                {
                    // Convert turns to radians for the simulation
                    m_motorSim.SetState(units::radian_t{offset.value()}, 
                                       m_motorSim.GetAngularVelocity());
                }
            }

            inline void SimPeriodic() override
            {
                // Get the applied output from the SparkMax simulation
                auto appliedOutput = m_sparkSim.GetAppliedOutput();
                auto batteryVoltage = frc::RobotController::GetBatteryVoltage();
                
                // Apply voltage to the motor simulation
                m_motorSim.SetInputVoltage(appliedOutput * batteryVoltage);
                m_motorSim.Update(20_ms);
                
                // Update the SparkMax simulation with new motor state
                // Convert radians per second to RPM for SparkMax sim
                auto velocityRPM = m_motorSim.GetAngularVelocity().value() * 60.0 / (2.0 * std::numbers::pi);
                m_sparkSim.iterate(velocityRPM, batteryVoltage.value(), 0.02);
                
                // Update encoder position in the SparkMax sim
                // Convert radians to turns
                auto positionTurns = m_motorSim.GetAngularPosition().value() / (2.0 * std::numbers::pi);
                m_sparkSim.SetPosition(positionTurns);
            }

            inline units::ampere_t GetCurrent()
            {
                if (frc::RobotBase::IsSimulation())
                {
                    return m_motorSim.GetCurrentDraw();
                }
                return units::ampere_t{m_motor.GetOutputCurrent()};
            }

            inline units::volt_t GetVoltage()
            {
                if (frc::RobotBase::IsSimulation())
                {
                    return units::volt_t{m_motorSim.GetInputVoltage().value()};
                }
                return units::volt_t{m_motor.GetAppliedOutput() * m_motor.GetBusVoltage()};
            }

            inline double GetTemperature()
            {
                return m_motor.GetMotorTemperature();
            }

        private:

            rev::spark::SparkMax                      m_motor;
            rev::spark::SparkRelativeEncoder          m_angleEncoder;
            rev::spark::SparkClosedLoopController     m_turnClosedLoopController;

            frc::SimpleMotorFeedforward<units::turns> m_feedforward;

            frc::DCMotor                              m_motorModel;
            rev::spark::SparkMaxSim                   m_sparkSim;

    };

}

}