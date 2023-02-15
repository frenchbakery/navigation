/**
 * @file navigation.hpp
 * @author melektron
 * @brief base class for the navigation system that will be inherited
 * by any other navigation implementations for the specific robots
 * @version 0.1
 * @date 2023-02-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <el/retcode.hpp>

class Navigation
{
public:
    virtual el::retcode initialize();
    virtual el::retcode terminate();

    // === System state getters and setters === //
    
};