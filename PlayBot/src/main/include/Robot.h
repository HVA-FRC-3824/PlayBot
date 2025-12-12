#pragma once

#include <hal/FRCUsageReporting.h>
#include <frc/TimedRobot.h>

#include "lib/Logging.h"
#include "RobotContainer.h"

#include "Constants.h"

class Robot : public frc::TimedRobot
{
    public:

        void RobotInit()     override;
        void RobotPeriodic() override;

    private:

        // Pointer to the autonomous command
        frc2::Command  *m_autonomousCommand = nullptr;

        // A pointer to the robot container class
        RobotContainer *m_robotContainer    = nullptr;
};
