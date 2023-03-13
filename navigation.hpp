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

#include <queue>
#include <atomic>
#include <el/retcode.hpp>
#include <el/vec.hpp>

class Navigation
{
protected:
    el::vec2_t current_position;
    double current_rotation = 0;
    int configured_speed = 500;

    struct nav_target_t
    {
        // distance to drive
        double distance;
        // angle to turn
        double angle;
    };

    std::queue<nav_target_t> target_queue;


public:
    virtual el::retcode initialize() = 0;
    virtual el::retcode terminate() = 0;

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
     * 
     * @param angle the angle in radians
     */
    virtual el::retcode rotateBy(double angle) = 0;

    /**
     * @brief rotates the robot to a specified angle referenced
     * to the root coordinate system. This will subtract the current angle
     * from the goal angle and then rotateBy() that delta to reached the
     * desired goal
     * 
     * @param angle absolute angle to reach
     * @retval ok - rotation started
     */
    virtual el::retcode rotateTo(double angle);
    /**
     * @brief drives the robot by a certain distance
     * in it's current direction. Positive is foreward, 
     * negative is backward.
     * 
     * @param distance distance in cm
     */
    virtual el::retcode driveDistance(double distance) = 0;

    /**
     * @brief drives in a straight line by a specific vector relative to the current position 
     * that is referenced to the root coordinate system. When issued
     * to drive a certain xy distance, the robot will set it's goal position to the 
     * current position plus the specified vector. It will then rotate in the direction of
     * the goal and drive to it in a straight line.
     * 
     * @param d delta vector
     * @retval ok - started driving
     */
    virtual el::retcode driveVector(el::vec2_t d);

    /**
     * @retval true - last target has been reached (no target active)
     * @retval false - target currently active but it hasen't been reached jet
     */
    virtual bool targetReached() = 0;

    /**
     * @brief blocks until the currently active target is reached.
     * If no target is active it will return immediately.
     * 
     * @return el::retcode 
     */
    virtual el::retcode awaitTargetReached() = 0;

    /**
     * @brief blocks until the currently active target is completed
     * to a certain percentage. For example, if the target is driving 
     * forward two meters, awaitTargetPercentage(50) will block until 
     * one meter has been completed. If the requested percentage has
     * already been passed, the function will return immediately.
     * 
     * @param percent percentage of the goal
     * @return el::retcode 
     */
    virtual el::retcode awaitTargetPercentage(int percent) = 0;
    
};