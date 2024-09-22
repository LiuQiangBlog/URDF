// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_FRAME_EXCEPTIONS__
#define __RDL_FRAME_EXCEPTIONS__

/**
 * @page frame_exceptions Frame Exceptions
 *
 * The RobotDynamics::ReferenceFrameException is an exception that is thrown
 * when an operation is performed that violates kinematic rules involving reference
 * frames. For example if two FramePoint objects are added together but they are
 * expressed in different frames. This will throw a RobotDynamics::ReferenceFrameException
 */

#include <stdexcept>
#include <exception>

namespace RobotDynamics
{
/**
 * @class ReferenceFrameException
 * @ingroup reference_frame
 * @brief A custom exception for frame operations
 */
class ReferenceFrameException : public std::exception
{
  public:
    /**
     * @brief Constructor
     * @param err
     */
    explicit ReferenceFrameException(const std::string& err) : msg(err)
    {
    }

    virtual const char* what() const throw()
    {
        return msg.c_str();
    }

    std::string msg /**< Custom exception message*/;
};
}  // namespace RobotDynamics
#endif  // ifndef __RDL_FRAME_EXCEPTIONS__
