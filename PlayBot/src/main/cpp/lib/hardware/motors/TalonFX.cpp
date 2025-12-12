#include "lib/hardware/motors/TalonFX.h"

using namespace hardware::motor;

TalonFX::TalonFX(CANid_t CANid, MotorConfiguration config, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfIntertia) : 
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
    m_motor{CANid, "rio"}
{
    ConfigureMotor(config);
}

void TalonFX::ConfigureMotor(MotorConfiguration config) // Configure the motor with default settings
{
    // Create the drive motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};
    // Add the "Motor Output" section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = config.breakMode
        ? ctre::phoenix6::signals::NeutralModeValue::Brake
        : ctre::phoenix6::signals::NeutralModeValue::Coast;

    // Add the "Current Limits" section settings
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit       = config.CurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Add the "Slot0" section settings
    // PID Controls and optional feedforward controls
    ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kP = config.P;
    slot0Configs.kI = config.I;
    slot0Configs.kD = config.D;
    slot0Configs.kS = config.S;
    slot0Configs.kV = config.V;
    slot0Configs.kA = config.A;

    // Try to apply the configuration multiple times in case of failure
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < 3; attempt++) // 3 is the number of names in Dean Lawrence Kamen's name
    {
        // Apply the configuration to the drive motor
        status = m_motor.GetConfigurator().Apply(talonFXConfiguration);
        // Check if the configuration was successful
        if (status.IsOK())
        break;
    }

    m_conversionFactor = config.conversionFactor;

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure TalonFX motor (" << m_motor.GetDeviceID() <<"). Error: " << status.GetName() << std::endl;
}

void TalonFX::SetReferenceState(double motorInput) // output to motor within (-1,1)
{
    // Set the motor speed and angle
    m_motor.Set(motorInput);
    if (frc::RobotBase::IsSimulation())
    {
        m_motorSim.SetInputVoltage(motorInput * frc::RobotController::GetBatteryVoltage());
    }
}

void TalonFX::SetReferenceState(units::turns_per_second_t motorInput) // output to motor within (-1,1)
{
    // Set the motor speed and angle
    m_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(motorInput * m_conversionFactor));

    if (frc::RobotBase::IsSimulation())
    {
        m_motorSim.SetAngularVelocity(units::radians_per_second_t{motorInput.value()});
    }
}

void TalonFX::SetReferenceState(units::volt_t motorInput) // output to motor within (-1,1)
{
    // Set the motor speed and angle
    m_motor.SetVoltage(motorInput);

    if (frc::RobotBase::IsSimulation())
    {
        m_motorSim.SetInputVoltage(motorInput);
    }
}

void TalonFX::SetReferenceState(units::turn_t motorInput) // output to motor in turns
{
    // Set the arm set position
    m_motor.SetControl(m_motionMagicVoltage.WithPosition(motorInput * m_conversionFactor).WithSlot(0));

    if (frc::RobotBase::IsSimulation())
    {
        m_motorSim.SetAngle(units::radian_t{motorInput.value()});
    }
}

units::turn_t TalonFX::GetPosition() // Returns the position of the motor in turns
{
    if (frc::RobotBase::IsSimulation())
    {
        return units::turn_t{m_motorSim.GetAngularPosition().value() / m_conversionFactor};
    }
    return m_motor.GetPosition().GetValue();
}

units::turns_per_second_t TalonFX::GetVelocity() // Returns the velocity of the motor in turns
{
    if (frc::RobotBase::IsSimulation())
    {
        return units::turns_per_second_t{m_motorSim.GetAngularVelocity().value() / m_conversionFactor};
    }
    return m_motor.GetVelocity().GetValue();
}

void TalonFX::OffsetEncoder(units::turn_t offset) // Returns the current of the motor in amps
{
    if (!frc::RobotBase::IsSimulation())
    {
        m_motor.SetPosition(offset * m_conversionFactor);
    }
}

void TalonFX::SimPeriodic()
{
    auto& talonFXSim = m_motor.GetSimState();

    // Set the supply voltage of the TalonFX
    talonFXSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // Get the motor voltage of the TalonFX
    auto motorVoltage = talonFXSim.GetMotorVoltage();

    // Use the motor voltage to calculate new position and velocity
    m_motorSim.SetInputVoltage(motorVoltage);
    m_motorSim.Update(20_ms);

    // Apply the new rotor position and velocity to the TalonFX
    talonFXSim.SetRawRotorPosition(units::turn_t{m_motorSim.GetAngularPosition().value()});
    talonFXSim.SetRotorVelocity(units::turns_per_second_t{m_motorSim.GetAngularVelocity().value()});
}