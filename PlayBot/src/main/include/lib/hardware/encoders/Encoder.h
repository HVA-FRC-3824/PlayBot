#pragma once

#include <units/angle.h>

#include "lib/hardware/hardware.h"

namespace hardware::encoder
{
    
    class Encoder : public Hardware
    {
        public:

            virtual units::turn_t GetTurns();
    };

}
