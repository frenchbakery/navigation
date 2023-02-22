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
#include <kiprplus/aggregation_engine.hpp>
#include <memory>
#include "../navigation.hpp"

class CRNav : public Navigation
{
    std::shared_ptr<kp::RampedMotor> motorl;
    std::shared_ptr<kp::RampedMotor> motorr;
    kp::AggregationEngine engine;

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
    virtual el::retcode awaitTargetPercentage(int percent) override;

};