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

#define WAIT_DELAY 50 // ms
#define UPDATE_DELAY 2 // ms

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

void Navigation::sequenceThreadFn()
{
    bool first_command = true;
    while (!threxit)
    {
        // wait until the sequence is marked incomplete
        if (sequence_complete)
        {
            msleep(WAIT_DELAY);
            continue;
        }
        
        // await the active target completion
        if (!targetReached())
        {
            msleep(UPDATE_DELAY);
            continue;
        }
        
        std::unique_lock lock(command_queue_guard);
        // if the queue is empty, mark the sequence as complete
        if (command_queue.empty())
        {
            // timeout for the last command
            msleep(getCommandTimeout());
            sequence_complete = true;
            first_command = true;
            lock.unlock();
            continue;
        }

        // read and execute the next command
        auto command = command_queue.front();

        // don't wait before the first command
        if (first_command)
            first_command = false;
        else
            msleep(getCommandTimeout());

        switch (command.type)
        {
        case seq_cmd_t::drive:
            rawDriveDistance(command.value);
            break;
        case seq_cmd_t::turn:
            rawRotateBy(command.value);
            break;
        default:
            break;
        }
        // remove the command from the queue
        command_queue.pop();
    }
}

el::retcode Navigation::initialize()
{
    sequence_thread = std::thread(&Navigation::sequenceThreadFn, this);
    return el::retcode::ok;
}
el::retcode Navigation::terminate()
{
    threxit = true;
    if (sequence_thread.joinable())
        sequence_thread.join();
    return el::retcode::ok;
}

const el::vec2_t &Navigation::getCurrentPosition() const
{
    return current_position;
}

double Navigation::getCurrentRotation() const
{
    return current_rotation;
}

void Navigation::setMotorSpeed(int speed)
{
    configured_speed = speed;
}

void Navigation::setCurrentPosition(el::vec2_t pos)
{
    current_position = pos;
}

void Navigation::setCurrentRotation(double angle)
{
    current_rotation = angle;
}

el::retcode Navigation::rotateBy(double angle)
{
    std::lock_guard lock(command_queue_guard);
    seq_cmd_t command;
    command.type = seq_cmd_t::turn;
    command.value = angle;
    command_queue.push(command);
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

el::retcode Navigation::driveDistance(double distance)
{
    std::lock_guard lock(command_queue_guard);
    seq_cmd_t command;
    command.type = seq_cmd_t::drive;
    command.value = distance;
    command_queue.push(command);
    return el::retcode::ok;
}

el::retcode Navigation::driveVector(el::vec2_t d, bool bw)
{
    rotateTo(d.get_phi() + (bw ? M_PI : 0));
    driveDistance(d.get_r() * (bw ? -1 : 1));

    return el::retcode::ok;
}

// TODO: fix, this doesn't quite work jet. the current position might not be accurate
el::retcode Navigation::driveToPosition(el::vec2_t pos, bool bw)
{
    el::vec2_t delta = pos - current_position;
    return driveVector(delta, bw);
}

el::retcode Navigation::startSequence()
{
    if (!sequence_complete)
        return el::retcode::err;
    
    std::lock_guard lock(command_queue_guard);
    if (command_queue.empty())
        return el::retcode::nak;

    // start sequence processing
    sequence_complete = false;
    return el::retcode::ok;
}

el::retcode Navigation::awaitSequenceComplete()
{
    if (sequence_complete)
        return el::retcode::nak;
    
    while (!sequence_complete)
        msleep(UPDATE_DELAY);
    
    return el::retcode::ok;
}