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

            TalonFX(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfIntertia = 0.001_kg_sq_m);

            void ConfigureMotor(MotorConfiguration config) override;

            void SetReferenceState(double motorInput) override;
            void SetReferenceState(units::turns_per_second_t motorInput) override; // output to motor within (-1,1)
            void SetReferenceState(units::volt_t motorInput) override; // output to motor within (-1,1)
            void SetReferenceState(units::turn_t motorInput) override; // output to motor in turns

            units::turn_t GetPosition() override; // Returns the position of the motor in turns

            units::turns_per_second_t GetVelocity() override; // Returns the velocity of the motor in turns

            void OffsetEncoder(units::turn_t offset) override; // Returns the current of the motor in amps

            void SimPeriodic() override;

        private:

            ctre::phoenix6::hardware::TalonFX             m_motor;    // TalonFX motor controller
            ctre::phoenix6::controls::MotionMagicVoltage  m_motionMagicVoltage{0_tr};

            // Conversion Factor, multiply when apply, divide when read
            double m_conversionFactor = 0;
    };

}

}
