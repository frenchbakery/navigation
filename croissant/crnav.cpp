/**
 * @file crnav.hpp
 * @author melektron
 * @brief Navigation implementation for the croissant robot
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright FrenchBakery (c) 2023
 *
 */

#ifdef __CROISSANT

#include <iostream>
#include <iomanip>
#include <kipr/time/time.h>

#include "crnav.hpp"


#define LEFT_MOTOR_PORT 1
#define RIGHT_MOTOR_PORT 0

/*#define TICKS_PER_ROTATION 2100 // 1900
#define WHEEL_RADIUS_CM 6.9
constexpr double __wheel_circumference = WHEEL_RADIUS_CM * M_PI;
#define WHEEL_CIRCUMFERENCE_CM __wheel_circumference
constexpr double __ticks_per_cm = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_CM;
#define TICKS_PER_CM __ticks_per_cm*/

// constexpr function that allows creating different constants for
// the ticks per cm in different driving functions.
constexpr double GET_TICKS_PER_CM(
    double ticks_per_revolution = 1900,
    double wheel_radius_cm = 6.9
)
{
    // C++ 11 doesn't support local variables in constexpr unfortunately.
    //const double wheel_circumference = wheel_radius_cm * M_PI;
    return ticks_per_revolution / /*wheel_circumference*/ (wheel_radius_cm * M_PI);
}
#define STRAIGHT_TICKS_PER_ROTATION 2100
#define TURNING_TICKS_PER_ROTATION 1900

#define WHEEL_TO_CENTER_CM 8.15  // Distance from the wheel to the center point of the robot (between the two wheels)
constexpr double __track_circumference = 2 * WHEEL_TO_CENTER_CM * M_PI;
#define TRACK_CIRCUMFERENCE __track_circumference
//constexpr double __


CRNav::CRNav()
    : motorl(std::make_shared<kp::RampedMotor>(LEFT_MOTOR_PORT)),
      motorr(std::make_shared<kp::RampedMotor>(RIGHT_MOTOR_PORT))//,
      //engine({motorl, motorr})
{
}

el::retcode CRNav::initialize()
{
    motorl->clearPositionCounter();
    motorr->clearPositionCounter();
    return el::retcode::ok;
}

el::retcode CRNav::terminate()
{
    motorl->off();
    motorr->off();
    return el::retcode::ok;
}

el::retcode CRNav::rotateBy(double angle)
{
    double distance_per_radian = TRACK_CIRCUMFERENCE / (2 * M_PI);
    double distance = angle * distance_per_radian;
    double ticks = distance * GET_TICKS_PER_CM(TURNING_TICKS_PER_ROTATION);
    motorl->moveRelativePosition(configured_speed, -ticks);
    motorr->moveRelativePosition(configured_speed, ticks);
    current_rotation += angle;
    return el::retcode::ok;
}

el::retcode CRNav::driveDistance(double distance)
{
    double ticks = distance * GET_TICKS_PER_CM(STRAIGHT_TICKS_PER_ROTATION);
    motorl->moveRelativePosition(configured_speed, ticks);
    motorr->moveRelativePosition(configured_speed, ticks);
    //engine.moveRelativePosition(configured_speed, ticks);
    current_position += el::polar_t(current_rotation, distance);
    return el::retcode::ok;
}

el::retcode CRNav::awaitTargetReached()
{
    motorr->blockMotorDone();
    motorl->blockMotorDone();
    return el::retcode::ok;
}

el::retcode CRNav::awaitTargetPercentage(int percent)
{
    while (true)
    {
        int l = motorl->getPercentCompleted();
        int r = motorr->getPercentCompleted();
        if ((l+r) / 2 >= std::min(percent, 100))
            break;
        msleep(10);
    }

    return el::retcode::ok;
}

#endif // __CROISSANT