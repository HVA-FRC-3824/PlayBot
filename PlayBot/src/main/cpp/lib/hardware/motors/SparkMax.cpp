#include "lib/hardware/motors/SparkMax.h"


using namespace hardware::motor;

SparkMax::SparkMax(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfIntertia) : 
    Motor{
        frc::sim::DCMotorSim(
            frc::LinearSystemId::DCMotorSystem(
                motorModel,
                simMomentOfIntertia,
                1
            ),
            motorModel
        )
    },
    m_motor{CANid, config.breakMode ?
          rev::spark::SparkLowLevel::MotorType::kBrushless 
        : rev::spark::SparkLowLevel::MotorType::kBrushed},
    m_angleEncoder{m_motor.GetEncoder()}, 
    m_turnClosedLoopController{m_motor.GetClosedLoopController()},

    m_feedforward{config.S * 1_V, config.V * 1_V * 1_s / 1_tr, config.A * 1_V * 1_s * 1_s / 1_tr},

    m_motorModel{motorModel},
    m_sparkSim{&m_motor, &m_motorModel}
{
    ConfigureMotor(config);
}

void SparkMax::ConfigureMotor(MotorConfiguration config) // Configure the motor with default settings
{
    // Configure the angle motor
    rev::spark::SparkMaxConfig sparkMaxConfig{};

    // Configure the motor controller
    sparkMaxConfig
        .SetIdleMode(config.breakMode 
    ? rev::spark::SparkBaseConfig::IdleMode::kBrake 
    : rev::spark::SparkBaseConfig::IdleMode::kCoast)
        .SmartCurrentLimit(config.CurrentLimit.value());

    sparkMaxConfig.encoder
        .PositionConversionFactor(config.conversionFactor)
        .VelocityConversionFactor(config.conversionFactor / 60);

    // Configure the closed loop controller
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(config.P, config.I, config.D);

    // Write the configuration to the motor controller
    m_motor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

// MAJOR TODO: whenever the REV library updates, change SetReference (deprecated) to SetSetpoint

void SparkMax::SetReferenceState(double motorInput) // output to motor within (-1,1)
{
    m_turnClosedLoopController.SetReference(motorInput, 
        rev::spark::SparkMax::ControlType::kDutyCycle);
}

void SparkMax::SetReferenceState(units::turns_per_second_t motorInput) // output to motor within (-1,1)
{
    m_turnClosedLoopController.SetReference(motorInput.value(), 
        rev::spark::SparkMax::ControlType::kVelocity);
}

void SparkMax::SetReferenceState(units::volt_t motorInput) // output to motor within (-1,1)
{
    m_turnClosedLoopController.SetReference(motorInput.value() + m_feedforward.Calculate(0_tps).value(), // just add the static value
        rev::spark::SparkMax::ControlType::kVoltage);
}

void SparkMax::SetReferenceState(units::turn_t motorInput) // output to motor in turns, this does go through the conversion value in the motor
{
    m_turnClosedLoopController.SetReference(motorInput.value(), 
        rev::spark::SparkMax::ControlType::kPosition);
}

units::turn_t SparkMax::GetPosition() // Returns the position of the motor in turns
{
    return units::turn_t{m_angleEncoder.GetPosition()};
}

units::turns_per_second_t SparkMax::GetVelocity() // Returns the velocity of the motor in turn velocity
{
    return units::turns_per_second_t{m_angleEncoder.GetVelocity() / 60};
}

void SparkMax::OffsetEncoder(units::turn_t offset)
{
    m_angleEncoder.SetPosition(offset.value() * (2 * std::numbers::pi));
}

void SparkMax::SimPeriodic()
{
    // not sure how great this is
    m_motorSim.SetInputVoltage(m_sparkSim.GetAppliedOutput() * frc::RobotController::GetBatteryVoltage());
    m_motorSim.Update(0.02_s);
    m_sparkSim.iterate(m_motorSim.GetAngularVelocity().value(), frc::RobotController::GetBatteryVoltage().value(), 0.02);
}