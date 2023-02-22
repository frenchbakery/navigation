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
#include "navigation.hpp"


void noimpl()
{
    std::cout << __FILE__ << ": " << "noimpl" << std::endl;
}

el::retcode Navigation::initialize()
{
    noimpl();
    return el::retcode::noimpl;
}

el::retcode Navigation::terminate()
{
    noimpl();
    return el::retcode::noimpl;
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

el::retcode Navigation::rotateBy(double angle)
{
    noimpl();
    return el::retcode::noimpl;
}

el::retcode Navigation::driveDistance(double distance)
{
    noimpl();
    return el::retcode::noimpl;
}

el::retcode Navigation::awaitTargetReached()
{
    noimpl();
    return el::retcode::noimpl;
}

el::retcode Navigation::awaitTargetPercentage(int percent)
{
    noimpl();
    return el::retcode::noimpl;
}