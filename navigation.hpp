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
#include <thread>
#include <mutex>
#include <el/retcode.hpp>
#include <el/vec.hpp>

class Navigation
{
protected:
    el::vec2_t current_position;
    double current_rotation = 0;
    int configured_speed = 500;

    struct seq_cmd_t
    {
        // command type
        enum cmd_type_t
        {
            drive,
            turn
        } type;
        // distance or angle to drive
        double value;
    };

    std::mutex command_queue_guard;
    std::queue<seq_cmd_t> command_queue;
    std::atomic_bool sequence_complete{false};

    std::atomic_bool threxit;
    std::thread sequence_thread;
    void sequenceThreadFn();


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
    virtual void setMotorSpeed(int speed);

    /**
     * @brief Resets the internally kept current position to a specific value.
     * This can be used to initialize the coordinate system.
     * 
     * @param pos new position
     */
    virtual void setCurrentPosition(el::vec2_t pos);

    /**
     * @brief Resets the internally kept current rotation to a specific value.
     * This can be used to initialize the coordinate system.
     * 
     * @param angle new rotation angle
     */
    virtual void setCurrentRotation(double angle);

    /**
     * @brief starts a robot rotation command
     * This will not add a command to the sequence. This is the raw 
     * function used by the sequence processor
     * 
     * @param angle the angle in radians
     */
    virtual el::retcode rawRotateBy(double angle) = 0;

    /**
     * @brief starts a driving command with the specified
     * distance. This has to be implemented by specializations.
     * This will not add a command to the sequence. This is the raw
     * function used by the sequence processor
     * 
     * @param distance distance in cm
     */
    virtual el::retcode rawDriveDistance(double distance) = 0;

    /**
     * @brief rotates the robot by a specific angle.
     * positive is ccw (mathematical angle)
     * This will add a rotate sequence command to the queue.
     * 
     * @param angle the angle in radians
     */
    virtual el::retcode rotateBy(double angle);

    /**
     * @brief rotates the robot to a specified angle referenced
     * to the root coordinate system. This will subtract the current angle
     * from the goal angle and then rotateBy() that delta to reached the
     * desired goal
     * This will add a rotate sequence command to the queue.
     * 
     * @param angle absolute angle to reach
     * @retval ok - rotation started
     */
    virtual el::retcode rotateTo(double angle);

    /**
     * @brief drives the robot by a certain distance
     * in it's current direction. Positive is foreward, 
     * negative is backward.
     * This will add a drive sequence command to the queue.
     * 
     * @param distance distance in cm
     */
    virtual el::retcode driveDistance(double distance);

    /**
     * @brief drives in a straight line by a specific vector relative to the current position 
     * that is referenced to the root coordinate system. When issued
     * to drive a certain xy distance, the robot will set it's goal position to the 
     * current position plus the specified vector. It will then rotate in the direction of
     * the goal and drive to it in a straight line.
     * This will add a drive sequence command to the queue.
     * 
     * @param d delta vector
     * @param bw flag to tell the robot to drive backward. This will cause the angle
     * to shift by 180 degrees as the robot will drive backward instead of forward.
     * @retval ok - started driving
     */
    virtual el::retcode driveVector(el::vec2_t d, bool bw = false);

    /**
     * @brief (not jet fully working) 
     * drives in a straight line to an absolute position in the root coordinate system.
     * This will add a drive sequence command to the queue.
     * 
     * @param pos absolute target position
     * @param bw flag to tell to robot to drive backward instead of forwards
     * @retval ok
     */
    virtual el::retcode driveToPosition(el::vec2_t pos, bool bw = false);

    /**
     * @retval true - last target has been reached (no target active)
     * @retval false - target currently active but it hasen't been reached jet
     */
    virtual bool targetReached() = 0;

    /**
     * @brief blocks until the next sequence target is reached.
     * If no target is active it will return immediately.
     * 
     * @return el::retcode 
     */
    virtual el::retcode awaitTargetReached() = 0;

    /**
     * @brief (Not jet implemented for both tiramisu and croissant)
     * blocks until the currently next sequence target is completed
     * to a certain percentage. For example, if the target is driving 
     * forward two meters, awaitTargetPercentage(50) will block until 
     * one meter has been completed. If the requested percentage has
     * already been passed, the function will return immediately.
     * 
     * @param percent percentage of the goal
     * @return el::retcode 
     */
    virtual el::retcode awaitTargetPercentage(int percent) = 0;

    /**
     * @brief starts processing the current sequence queue.
     * 
     * @retval nak - queue empty
     * @retval err - sequence already running
     * @retval ok - sequence started
     */
    virtual el::retcode startSequence();

    /**
     * @brief blocks until the current sequence is completed.
     * 
     * @retval nak - no sequence active
     * @retval ok - sequence complete
     */
    virtual el::retcode awaitSequenceComplete();
    
};