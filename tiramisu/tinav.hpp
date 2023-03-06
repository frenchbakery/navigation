/**
 * @file tinav.hpp
 * @author melektron
 * @brief navigation implementatino for tiramisu
 * @version 0.1
 * @date 2023-02-24
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */

#pragma once

#ifdef __TIRAMISU


#include "../navigation.hpp"


class CRNav : public Navigation
{
    

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


#endif // __TIRAMISU