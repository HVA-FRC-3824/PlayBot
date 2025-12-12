#pragma once

#include "Motor.h"

#include <iostream>
#include <numbers>

#include <frc/RobotController.h>

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

            SparkMax(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfIntertia = 0.001_kg_sq_m);

            void ConfigureMotor(MotorConfiguration config) override;

            // MAJOR TODO: whenever the REV library updates, change SetReference (deprecated) to SetSetpoint

            void SetReferenceState(double motorInput) override;

            void SetReferenceState(units::turns_per_second_t motorInput) override;

            void SetReferenceState(units::volt_t motorInput) override;

            void SetReferenceState(units::turn_t motorInput) override;

            units::turn_t GetPosition() override;

            units::turns_per_second_t GetVelocity() override;

            void OffsetEncoder(units::turn_t offset) override;

            void SimPeriodic() override;

        private:

            rev::spark::SparkMax                      m_motor;                    // SparkMax motor controller
            rev::spark::SparkRelativeEncoder          m_angleEncoder;             // Relative encoder onboard the sparkmax
            rev::spark::SparkClosedLoopController     m_turnClosedLoopController; // PID Controller for SparkMax

            frc::SimpleMotorFeedforward<units::turns> m_feedforward;              // Feedforward controller for the motor

            frc::DCMotor                              m_motorModel;               // Type of motor attached to the SparkMax
            rev::spark::SparkMaxSim                   m_sparkSim;                 // Simulated SparkMax model

    };

}

}
