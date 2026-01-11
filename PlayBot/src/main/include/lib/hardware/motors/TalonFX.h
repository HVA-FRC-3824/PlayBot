#pragma once

#include <iostream>
#include <numbers>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <units/angle.h>
#include <units/voltage.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "lib/hardware/motors/Motor.h"

namespace hardware
{

namespace motor
{

    class TalonFX : public Motor
    {
        
        public:

            inline TalonFX(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfInertia = 0.001_kg_sq_m) 
                : Motor{frc::sim::DCMotorSim(
                    frc::LinearSystemId::DCMotorSystem(
                        motorModel,
                        simMomentOfInertia,
                        1
                    ),
                    motorModel
                    )},
                m_motor{CANid, "rio"}
            {
                ConfigureMotor(config);
            }

            inline void ConfigureMotor(MotorConfiguration config) override
            {
                constexpr int MAX_CONFIG_RETRIES = 3;
                
                // Create the TalonFX configuration
                ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};
                
                // Configure Motor Output settings
                ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
                motorOutputConfigs.NeutralMode = config.breakMode
                    ? ctre::phoenix6::signals::NeutralModeValue::Brake
                    : ctre::phoenix6::signals::NeutralModeValue::Coast;

                // Configure Current Limits
                ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
                currentLimitsConfigs.StatorCurrentLimit = config.CurrentLimit;
                currentLimitsConfigs.StatorCurrentLimitEnable = true;
                // Also set supply current limit for battery protection
                currentLimitsConfigs.SupplyCurrentLimit = config.CurrentLimit;
                currentLimitsConfigs.SupplyCurrentLimitEnable = true;

                // Configure PID and Feedforward (Slot 0)
                ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
                slot0Configs.kP = config.P;
                slot0Configs.kI = config.I;
                slot0Configs.kD = config.D;
                slot0Configs.kS = config.S;
                slot0Configs.kV = config.V;
                slot0Configs.kA = config.A;

                // Configure MotionMagic parameters
                ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = talonFXConfiguration.MotionMagic;
                motionMagicConfigs.MotionMagicCruiseVelocity = units::turns_per_second_t{config.S};
                motionMagicConfigs.MotionMagicAcceleration = units::turns_per_second_squared_t{config.V};
                motionMagicConfigs.MotionMagicJerk = units::turns_per_second_cubed_t{config.A};

                // Try to apply the configuration with retries
                ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
                for (int attempt = 0; attempt < MAX_CONFIG_RETRIES; attempt++)
                {
                    status = m_motor.GetConfigurator().Apply(talonFXConfiguration);
                    if (status.IsOK())
                    {
                        break;
                    }
                    // Small delay before retry
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                // Report configuration status
                if (!status.IsOK())
                {
                    std::cerr << "***** ERROR: Could not configure TalonFX motor (CAN ID: " 
                              << m_motor.GetDeviceID() << "). Error: " << status.GetName() 
                              << " (" << status.GetDescription() << ")" << std::endl;
                }
                else
                {
                    std::cout << "TalonFX motor (CAN ID: " << m_motor.GetDeviceID() 
                              << ") configured successfully." << std::endl;
                }
            }

            inline void SetReferenceState(double motorInput) override
            {
                // Set the motor duty cycle [-1, 1]
                m_motor.Set(motorInput);

                // In simulation, apply the voltage to the motor model
                if (frc::RobotBase::IsSimulation())
                {
                    m_motorSim.SetInputVoltage(motorInput * frc::RobotController::GetBatteryVoltage());
                }
            }

            inline void SetReferenceState(units::turns_per_second_t motorInput) override
            {
                // Set the motor velocity using closed-loop control
                m_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(motorInput));

                // In simulation, we can't directly set velocity - the physics sim will handle it
                // But we can estimate the voltage needed for this velocity and apply it
                if (frc::RobotBase::IsSimulation())
                {
                    // Simple feedforward estimation: voltage ≈ kV * velocity
                    // The actual closed-loop controller would adjust this, but for sim this is reasonable
                    auto estimatedVoltage = motorInput.value() * m_slot0Configs.kV.value();
                    m_motorSim.SetInputVoltage(units::volt_t{estimatedVoltage});
                }
            }

            inline void SetReferenceState(units::volt_t motorInput) override
            {
                // Set the motor voltage directly
                m_motor.SetVoltage(motorInput);

                // In simulation, apply the voltage to the motor model
                if (frc::RobotBase::IsSimulation())
                {
                    m_motorSim.SetInputVoltage(motorInput);
                }
            }

            inline void SetReferenceState(units::turn_t motorInput) override
            {
                // Set the motor position using MotionMagic
                m_motor.SetControl(m_motionMagicVoltage.WithPosition(motorInput).WithSlot(0));

                // In simulation, the position control will be handled by SimPeriodic
                // We don't directly set position here to maintain realistic physics
            }

            inline units::turn_t GetPosition() override
            {
                if (frc::RobotBase::IsSimulation())
                {
                    // Convert radians to turns
                    return units::turn_t{m_motorSim.GetAngularPosition().value() / (2.0 * std::numbers::pi)};
                }
                return m_motor.GetPosition().GetValue();
            }

            inline units::turns_per_second_t GetVelocity() override
            {
                if (frc::RobotBase::IsSimulation())
                {
                    // Convert radians per second to turns per second
                    return units::turns_per_second_t{m_motorSim.GetAngularVelocity().value() / (2.0 * std::numbers::pi)};
                }
                return m_motor.GetVelocity().GetValue();
            }

            inline void OffsetEncoder(units::turn_t offset) override
            {
                m_motor.SetPosition(offset);
                
                if (frc::RobotBase::IsSimulation())
                {
                    // Convert turns to radians for the simulation
                    m_motorSim.SetState(units::radian_t{offset.value() * 2.0 * std::numbers::pi}, 
                                       m_motorSim.GetAngularVelocity());
                }
            }

            inline void SimPeriodic() override
            {
                auto& talonFXSim = m_motor.GetSimState();

                // Set the supply voltage of the TalonFX
                talonFXSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

                // Get the motor voltage of the TalonFX (this reflects what the controller is commanding)
                auto motorVoltage = talonFXSim.GetMotorVoltage();

                // Use the motor voltage to calculate new position and velocity using physics simulation
                m_motorSim.SetInputVoltage(motorVoltage);
                m_motorSim.Update(20_ms);

                // Apply the new rotor position and velocity back to the TalonFX
                // Convert radians to turns
                talonFXSim.SetRawRotorPosition(units::turn_t{m_motorSim.GetAngularPosition().value() / (2.0 * std::numbers::pi)});
                talonFXSim.SetRotorVelocity(units::turns_per_second_t{m_motorSim.GetAngularVelocity().value() / (2.0 * std::numbers::pi)});
            }

            inline units::ampere_t GetCurrent()
            {
                if (frc::RobotBase::IsSimulation())
                {
                    return m_motorSim.GetCurrentDraw();
                }
                return m_motor.GetStatorCurrent().GetValue();
            }

            inline units::volt_t GetVoltage()
            {
                if (frc::RobotBase::IsSimulation())
                {
                    return units::volt_t{m_motorSim.GetInputVoltage().value()};
                }
                return m_motor.GetMotorVoltage().GetValue();
            }

        private:

            ctre::phoenix6::hardware::TalonFX             m_motor;
            ctre::phoenix6::controls::MotionMagicVoltage  m_motionMagicVoltage{0_tr};
            ctre::phoenix6::configs::Slot0Configs         m_slot0Configs{};
            
    };

}

}