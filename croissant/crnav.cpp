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

#define STRAIGHT_TICKS_PER_ROTATION 1850 //1867
#define STRAIGHT_LMULTP 1.02
#define STRAIGHT_RMULTP 1
#define STRAIGHT_LMULTN -1.02
#define STRAIGHT_RMULTN -1

#define TURNING_TICKS_PER_ROTATION 1930 //1922 //1916
#define TURNING_LMULTP 0.99     // for CW Turn  (- Angle)
#define TURNING_RMULTP 1.02     // for CCW Turn (+ Angle)
#define TURNING_LMULTN -1.04    // for CCW Turn (+ Angle)
#define TURNING_RMULTN -0.97    // for CW Turn  (- Angle)

// constexpr function that allows creating different constants for
// the ticks per cm in different driving functions.
constexpr double GET_TICKS_PER_CM(
    double ticks_per_revolution = 1900,
    double wheel_radius_cm = 6.9
)
{
    double wheel_circumference = wheel_radius_cm * M_PI;
    return ticks_per_revolution / wheel_circumference;
}

#define WHEEL_TO_CENTER_CM 8.15  // Distance from the wheel to the center point of the robot (between the two wheels)
constexpr double __track_circumference = 2 * WHEEL_TO_CENTER_CM * M_PI;
#define TRACK_CIRCUMFERENCE __track_circumference
//constexpr double __



CRNav::CRNav()
    : motorl(std::make_shared<kp::PIDMotor>(LEFT_MOTOR_PORT)),
      motorr(std::make_shared<kp::PIDMotor>(RIGHT_MOTOR_PORT)),
      engine({motorl, motorr})
{
}

el::retcode CRNav::initialize()
{
    Navigation::initialize();
    motorl->clearPositionCounter();
    motorr->clearPositionCounter();
    motorl->enablePositionControl();
    motorr->enablePositionControl();
    return el::retcode::ok;
}

el::retcode CRNav::terminate()
{
    motorl->off();
    motorr->off();
    Navigation::terminate();
    return el::retcode::ok;
}

el::retcode CRNav::rawRotateBy(double angle)
{
    double distance_per_radian = TRACK_CIRCUMFERENCE / (2 * M_PI);
    double distance = angle * distance_per_radian;
    double ticks = std::abs(distance * GET_TICKS_PER_CM(TURNING_TICKS_PER_ROTATION));
    double direction = angle < 0 ? -1 : 1;
    double lmult = -distance > 0 ? TURNING_LMULTP : TURNING_LMULTN;
    double rmult = distance > 0 ? TURNING_RMULTP : TURNING_RMULTN;
    engine.setMovementModifiers({lmult, rmult});   // set modifiers to invert one motor
    engine.moveRelativePosition(configured_speed, ticks);
    current_rotation += angle;
    return el::retcode::ok;
}

el::retcode CRNav::rawDriveDistance(double distance)
{
    double ticks = std::abs(distance * GET_TICKS_PER_CM(STRAIGHT_TICKS_PER_ROTATION));
    double lmult = distance > 0 ? STRAIGHT_LMULTP : STRAIGHT_LMULTN;
    double rmult = distance > 0 ? STRAIGHT_RMULTP : STRAIGHT_RMULTN;
    engine.setMovementModifiers({lmult, rmult});    // both motors in the same direction
    engine.moveRelativePosition(configured_speed, ticks);
    current_position += el::polar_t(current_rotation, distance);
    return el::retcode::ok;
}

bool CRNav::targetReached()
{
    return !engine.sequenceRunning();
}

el::retcode CRNav::awaitTargetReached()
{
    engine.awaitSequenceComplete();
    
    return el::retcode::ok;
}

el::retcode CRNav::awaitTargetPercentage(int percent)
{
    /*while (true)
    {
        int l = motorl->getPercentCompleted();
        int r = motorr->getPercentCompleted();
        if ((l+r) / 2 >= std::min(percent, 100))
            break;
        msleep(10);
    }*/

    return el::retcode::ok;
}


void CRNav::disablePositionControl()
{
    motorl->disablePositionControl();
    motorr->disablePositionControl();
}
void CRNav::enablePositionControl()
{
    motorl->enablePositionControl();
    motorr->enablePositionControl();
}

void CRNav::driveLeftSpeed(int speed)
{
    motorl->moveAtVelocity(speed);
}
void CRNav::driveRightSpeed(int speed)
{
    motorr->moveAtVelocity(speed);
}

void CRNav::resetPositionControllers()
{
    motorl->setAbsoluteTarget(0);
    motorr->setAbsoluteTarget(0);
    motorl->clearPositionCounter();
    motorr->clearPositionCounter();
}

#endif // __CROISSANT