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

#pragma once

#include <kiprplus/ramped_motor.hpp>
#include "../navigation.hpp"

class CRNav : public Navigation
{
    kp::RampedMotor motorl;
    kp::RampedMotor motorr;

public:
    /**
     * @brief Initializes motors and subobjects
     * that need initializer parameters. Any
     * initialization of actual systems is done in
     * the initialize() method.
     */
    CRNav();

    virtual el::retcode initialize() override;
    virtual el::retcode terminate() override;

    using Navigation::getCurrentPosition;
    using Navigation::getCurrentRotation;

    using Navigation::setMotorSpeed;

    virtual el::retcode rotateBy(double angle) override;
    virtual el::retcode driveDistance(double distance) override;
    virtual el::retcode awaitTargetReached() override;

};