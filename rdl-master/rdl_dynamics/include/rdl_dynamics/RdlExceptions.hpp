// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_EXCEPTIONS_HPP__
#define __RDL_EXCEPTIONS_HPP__

#include <stdexcept>
#include <exception>

namespace RobotDynamics
{
/**
 * @class RdlException
 * @brief A custom exception
 */
class RdlException : public std::exception
{
  public:
    /**
     * @brief Constructor
     * @param err
     */
    explicit RdlException(const std::string& err) : msg(err)
    {
    }

    virtual const char* what() const throw()
    {
        return msg.c_str();
    }

    std::string msg /**< Custom exception message*/;
};
}  // namespace RobotDynamics
#endif  // ifndef __RDL_EXCEPTIONS_HPP__
