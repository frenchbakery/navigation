/**
 * @file tinav.cpp
 * @author melektron
 * @brief navigation implementatino for tiramisu
 * @version 0.1
 * @date 2023-02-24
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */

#ifdef __TIRAMISU

#include "tinav.hpp"

#include <iostream>
#include <iomanip>
#include <kipr/time/time.h>

#include "tinav.hpp"


#define LEFT_MOTOR_PORT 0
#define RIGHT_MOTOR_PORT 1

#define STRAIGHT_TICKS_PER_CM 23
#define STRAIGHT_LMULTP 1
#define STRAIGHT_RMULTP 1
#define STRAIGHT_LMULTN 1
#define STRAIGHT_RMULTN 1

#define TURNING_TICKS_PER_CM 23
#define TURNING_LMULTP 1    // for CW Turn  (- Angle)
#define TURNING_RMULTP 1    // for CCW Turn (+ Angle)
#define TURNING_LMULTN -1    // for CCW Turn (+ Angle)
#define TURNING_RMULTN -1    // for CW Turn  (- Angle)

#define WHEEL_TO_CENTER_CM 11.5  // Distance from the wheel to the center point of the robot (between the two wheels)
constexpr double __track_circumference = 2 * WHEEL_TO_CENTER_CM * M_PI;
#define TRACK_CIRCUMFERENCE __track_circumference
//constexpr double __



TINav::TINav()
    : motorl(std::make_shared<kp::CreateMotor>(LEFT_MOTOR_PORT)),
      motorr(std::make_shared<kp::CreateMotor>(RIGHT_MOTOR_PORT)),
      engine({motorl, motorr})
{
}

el::retcode TINav::initialize()
{
    motorl->clearPositionCounter();
    motorr->clearPositionCounter();
    motorl->setAbsoluteTarget(0);
    motorr->setAbsoluteTarget(0);
    motorl->enablePositionControl();
    motorr->enablePositionControl();
    return el::retcode::ok;
}

el::retcode TINav::terminate()
{
    motorl->disablePositionControl();
    motorr->disablePositionControl();
    return el::retcode::ok;
}

el::retcode TINav::rotateBy(double angle)
{
    double distance_per_radian = TRACK_CIRCUMFERENCE / (2 * M_PI);
    double distance = angle * distance_per_radian;
    double ticks = std::abs(distance * TURNING_TICKS_PER_CM);
    double direction = angle < 0 ? -1 : 1;
    double lmult = -distance > 0 ? TURNING_LMULTP : TURNING_LMULTN;
    double rmult = distance > 0 ? TURNING_RMULTP : TURNING_RMULTN;
    engine.setMovementModifiers({lmult, rmult});   // set modifiers to invert one motor
    engine.moveRelativePosition(configured_speed, ticks);
    current_rotation += angle;
    return el::retcode::ok;
}

el::retcode TINav::driveDistance(double distance)
{
    double ticks = std::abs(distance * STRAIGHT_TICKS_PER_CM);
    double lmult = distance > 0 ? STRAIGHT_LMULTP : STRAIGHT_LMULTN;
    double rmult = distance > 0 ? STRAIGHT_RMULTP : STRAIGHT_RMULTN;
    engine.setMovementModifiers({lmult, rmult});    // both motors in the same direction
    engine.moveRelativePosition(configured_speed, ticks);
    current_position += el::polar_t(current_rotation, distance);
    return el::retcode::ok;
}

bool TINav::targetReached()
{
    return !engine.sequenceRunning();
}

el::retcode TINav::awaitTargetReached()
{
    engine.awaitSequenceComplete();
    
    return el::retcode::ok;
}

el::retcode TINav::awaitTargetPercentage(int percent)
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





#endif // __TIRAMISU