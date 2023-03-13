/**
 * @file navigation.hpp
 * @author melektron
 * @brief base class for the navigation system that will be inherited
 * by any other navigation implementations for the specific robots
 * @version 0.1
 * @date 2023-02-08
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */

#include <iostream>
#include <kipr/time/time.h>
#include "navigation.hpp"


void noimpl()
{
    std::cout << __FILE__ << ": " << "noimpl" << std::endl;
}

/**
 * @brief normalizes a to the range of only one full rotation
 * All angles are in radians.
 * Example in degrees for better readability: 410° -> 50°
 * @param a any angle in radians
 * @return double normalized angle in radians
 */
double normalizeAngle(double a)
{
    return a - (int)(a / (2 * M_PI)) * 2 * M_PI;
}

const el::vec2_t &Navigation::getCurrentPosition() const
{
    return current_position;
}

double Navigation::getCurrentRotation() const
{
    return current_rotation;
}

el::retcode Navigation::setMotorSpeed(int speed)
{
    configured_speed = speed;
    return el::retcode::ok;
}

el::retcode Navigation::rotateTo(double angle)
{
    double current_norm = normalizeAngle(current_rotation);
    double goal_norm = normalizeAngle(angle);
    double delta = goal_norm - current_norm;

    // if the angle is between -180 and +180 deg
    if (delta < M_PI && delta >= -M_PI)
    {
        rotateBy(delta);
        return el::retcode::ok;
    }
    else
    {
        rotateBy(delta - 2 * M_PI);
        return el::retcode::ok;
    }
    return el::retcode::ok;
}

el::retcode Navigation::driveVector(el::vec2_t d)
{
    el::vec2_t goal = current_position + d;

    rotateTo(d.get_phi());
    awaitTargetReached();
    msleep(500);
    driveDistance(d.get_r());
    awaitTargetReached();
    msleep(500);

    return el::retcode::ok;
}
