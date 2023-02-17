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

#pragma once

#include <el/retcode.hpp>
#include <el/vec.hpp>

class Navigation
{
protected:
    el::vec2_t current_position;
    double current_rotation = 0;
    int configured_speed = 500;


public:
    virtual el::retcode initialize();
    virtual el::retcode terminate();

    // === System state getters and setters === //
    virtual const el::vec2_t &getCurrentPosition() const;
    virtual double getCurrentRotation() const;

    /**
     * @brief sets the speed used for any subsequent 
     * target operations
     * 
     * @param _speed speed in ticks per second (0 to 1500)
     */
    virtual el::retcode setMotorSpeed(int speed);

    /**
     * @brief rotates the robot by a specific angle.
     * positive is ccw (mathematical angle)
     * 
     * @param angle the angle in radians
     */
    virtual el::retcode rotateBy(double angle);

    /**
     * @brief drives the robot by a certain distance
     * in it's current direction. Positive is foreward, 
     * negative is backward.
     * 
     * @param distance distance in cm
     */
    virtual el::retcode driveDistance(double distance);

    /**
     * @brief blocks until the currently active target is reached.
     * If no target is active it will return immediately.
     * 
     * @return el::retcode 
     */
    virtual el::retcode awaitTargetReached();
    
};