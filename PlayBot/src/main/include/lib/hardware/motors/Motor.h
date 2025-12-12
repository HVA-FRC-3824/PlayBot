#pragma once

#pragma region Includes
#include <memory>
#include <optional>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>

#include <frc/RobotBase.h>

#include "lib/hardware/hardware.h"
#pragma endregion

namespace hardware
{
    namespace motor
    {
        struct MotorConfiguration
        {
            units::ampere_t CurrentLimit;
            bool   breakMode;
            double conversionFactor; // Multiply when apply, divide when recieve
            double P;
            double I;
            double D;
            double S; // add your G term to this
            double V;
            double A;
        };
    
        // This class is used to abstract the motor controller interface
        // EVERYTHING is in motor-side turns. Conversions happen in implementation, until we implement gear ratios.
        class Motor : Hardware
        {
            public:
    
                Motor(frc::sim::DCMotorSim motorSim) : m_motorSim{motorSim}
                {
    
                }
    
                // OVERRIDE THESE IN YOUR IMPLEMENTATION
    
                // Call this in your implementation periodic loop when its in simulation
                virtual void                      SimPeriodic()                                           = 0;
                virtual void                      ConfigureMotor(MotorConfiguration config)               = 0;
                virtual void                      SetReferenceState(double motorInput)                    = 0;
                virtual void                      SetReferenceState(units::volt_t motorInput)             = 0;
                virtual void                      SetReferenceState(units::turns_per_second_t motorInput) = 0;
                virtual void                      SetReferenceState(units::turn_t motorInput)             = 0;
                virtual units::turn_t             GetPosition()                                           = 0;
                virtual units::turns_per_second_t GetVelocity()                                           = 0;
                virtual void                      OffsetEncoder(units::turn_t offset)                     = 0;
    
            protected:
    
                frc::sim::DCMotorSim  m_motorSim; // Simulated motor model
        };
    
    }
}
