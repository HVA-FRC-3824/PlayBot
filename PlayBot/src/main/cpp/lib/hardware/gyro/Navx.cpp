#include "lib/hardware/gyro/Navx.h"


using namespace hardware::gyro;

#pragma region Navx
/// @brief Constructor for the Navx gyro class.
Navx::Navx()
{
    
}
#pragma endregion

#pragma region GetRotation
/// @brief Method to retrieve the current rotation with offset applied.
/// @return The current rotation with offset applied.
frc::Rotation3d Navx::GetRotation()
{
    return m_gyro.GetRotation3d() + m_offset;
}
#pragma endregion

#pragma region GetOffset
/// @brief Method to retrieve the current offset rotation.
/// @return The current offset rotation.
frc::Rotation3d Navx::GetOffset()
{
    // Return the offset rotation
    return m_offset;
}
#pragma endregion

#pragma region ResetYaw
/// @brief Method to reset the yaw angle to zero.
void Navx::ResetYaw()
{
    // Reset the gyro yaw angle
    m_gyro.Reset();
    m_simYaw = 0_rad;
}
#pragma endregion

#pragma region SetOffset
/// @brief Method to set the offset rotation.
/// @param offset The offset rotation to set.
void Navx::SetOffset(frc::Rotation3d offset)
{
    // Set the offset rotation
    m_offset = offset;
}
#pragma endregion

#pragma region SimPeriodic
/// @brief Method to simulate the gyro in simulation.
/// @param rate The rate of change in radians per second.
void Navx::SimPeriodic(units::radians_per_second_t rate)
{
    // Update the simulated yaw angle
    m_simRate = rate;
    m_simYaw += units::radian_t{m_simRate.value()}; // assuming 20ms loop time

    // Update the offset to match the simulated yaw
    SetOffset(frc::Rotation3d{m_simYaw, 0_rad, 0_rad});
}
#pragma endregion
