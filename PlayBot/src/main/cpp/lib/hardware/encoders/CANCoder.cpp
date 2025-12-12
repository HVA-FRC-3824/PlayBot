#include "lib/hardware/encoders/CANCoder.h"


using namespace hardware::encoder;

// Constructor for the CANCoder class
// The CANCoder is a CAN device, so the CAN ID is passed in
CANCoder::CANCoder(CANid_t CANid) : m_encoder{CANid, "rio"}
{

}

// Configure the CANCoder
units::turn_t CANCoder::GetTurns()
{
    // Get the absolute position from the CANCoder and return it as turns
    return m_encoder.GetAbsolutePosition().GetValue();
}