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

#pragma once

#include <kiprplus/pid_motor.hpp>
#include <kiprplus/aggregation_engine.hpp>
#include <memory>
#include "../navigation.hpp"

class CRNav : public Navigation
{
    std::shared_ptr<kp::PIDMotor> motorl;
    std::shared_ptr<kp::PIDMotor> motorr;
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

    virtual el::retcode rawRotateBy(double angle) override;
    virtual el::retcode rawDriveDistance(double distance) override;
    virtual bool targetReached() override;
    virtual el::retcode awaitTargetReached() override;
    virtual el::retcode awaitTargetPercentage(int percent) override;

};

#endif // __CROISSANT