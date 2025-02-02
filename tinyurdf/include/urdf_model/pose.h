/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Wim Meeussen */

#ifndef URDF_INTERFACE_POSE_H
#define URDF_INTERFACE_POSE_H

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <urdf_exception/exception.h>
#include <urdf_model/utils.h>

namespace urdf
{

class Vector3
{
public:
    Vector3(double _x, double _y, double _z)
    {
        this->x = _x;
        this->y = _y;
        this->z = _z;
    };
    Vector3()
    {
        this->clear();
    };
    double x;
    double y;
    double z;

    void clear()
    {
        this->x = this->y = this->z = 0.0;
    };
    void init(const std::string &vector_str)
    {
        this->clear();
        std::vector<std::string> pieces;
        std::vector<double>      xyz;
        urdf::split_string(pieces, vector_str, " ");
        for (unsigned int i = 0; i < pieces.size(); ++i)
        {
            if (pieces[i] != "")
            {
                try
                {
                    xyz.push_back(strToDouble(pieces[i].c_str()));
                }
                catch (std::runtime_error &)
                {
                    throw ParseError("Unable to parse component [" + pieces[i] +
                                     "] to a double (while parsing a vector value)");
                }
            }
        }

        if (xyz.size() != 3)
            throw ParseError("Parser found " + std::to_string(xyz.size()) +
                             " elements but 3 expected while parsing vector [" + vector_str + "]");

        this->x = xyz[0];
        this->y = xyz[1];
        this->z = xyz[2];
    }

    Vector3 operator+(Vector3 vec)
    {
        return Vector3(this->x + vec.x, this->y + vec.y, this->z + vec.z);
    };
};

class Rotation
{
public:
    Rotation(double _x, double _y, double _z, double _w)
    {
        this->x = _x;
        this->y = _y;
        this->z = _z;
        this->w = _w;
    };
    Rotation()
    {
        this->clear();
    };
    void getQuaternion(double &quat_x, double &quat_y, double &quat_z, double &quat_w) const
    {
        quat_x = this->x;
        quat_y = this->y;
        quat_z = this->z;
        quat_w = this->w;
    };
    void getRPY(double &roll, double &pitch, double &yaw) const
    {
        double sqw;
        double sqx;
        double sqy;
        double sqz;

        sqx = this->x * this->x;
        sqy = this->y * this->y;
        sqz = this->z * this->z;
        sqw = this->w * this->w;

        // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
        double       sarg = -2 * (this->x * this->z - this->w * this->y);
        const double pi_2 = 1.57079632679489661923;
        if (sarg <= -0.99999)
        {
            pitch = -pi_2;
            roll  = 0;
            yaw   = 2 * atan2(this->x, -this->y);
        }
        else if (sarg >= 0.99999)
        {
            pitch = pi_2;
            roll  = 0;
            yaw   = 2 * atan2(-this->x, this->y);
        }
        else
        {
            pitch = asin(sarg);
            roll  = atan2(2 * (this->y * this->z + this->w * this->x), sqw - sqx - sqy + sqz);
            yaw   = atan2(2 * (this->x * this->y + this->w * this->z), sqw + sqx - sqy - sqz);
        }
    };
    void setFromQuaternion(double quat_x, double quat_y, double quat_z, double quat_w)
    {
        this->x = quat_x;
        this->y = quat_y;
        this->z = quat_z;
        this->w = quat_w;
        this->normalize();
    };
    void setFromRPY(double roll, double pitch, double yaw)
    {
        double phi, the, psi;

        phi = roll / 2.0;
        the = pitch / 2.0;
        psi = yaw / 2.0;

        this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
        this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
        this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
        this->w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

        this->normalize();
    };

    double x, y, z, w;

    void init(const std::string &rotation_str)
    {
        this->clear();
        Vector3 rpy;
        rpy.init(rotation_str);
        setFromRPY(rpy.x, rpy.y, rpy.z);
    }

    void clear()
    {
        this->x = this->y = this->z = 0.0;
        this->w                     = 1.0;
    }

    void normalize()
    {
        const double squared_norm = this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w;

        // Note: This function historically checked if the norm of the elements
        // equaled 0 using a strict floating point equals (s == 0.0) to prevent a
        // divide by zero error. This comparison will cause the compiler to issue a
        // warning with `-Wfloat-equal`. Comparing against a small epsilon would
        // likely be more ideal, but its choice may be application specific and
        // could change behavior of legacy code if chosen poorly. Using `<=`
        // silences the warning without changing the behavior of this function, even
        // though there are no situations squared_norm can be strictly less than 0.
        if (squared_norm <= 0.0)
        {
            this->x = 0.0;
            this->y = 0.0;
            this->z = 0.0;
            this->w = 1.0;
        }
        else
        {
            const double s  = sqrt(squared_norm);
            this->x        /= s;
            this->y        /= s;
            this->z        /= s;
            this->w        /= s;
        }
    };

    // Multiplication operator (copied from gazebo)
    Rotation operator*(const Rotation &qt) const
    {
        Rotation c;

        c.x = this->w * qt.x + this->x * qt.w + this->y * qt.z - this->z * qt.y;
        c.y = this->w * qt.y - this->x * qt.z + this->y * qt.w + this->z * qt.x;
        c.z = this->w * qt.z + this->x * qt.y - this->y * qt.x + this->z * qt.w;
        c.w = this->w * qt.w - this->x * qt.x - this->y * qt.y - this->z * qt.z;

        return c;
    };
    /// Rotate a vector using the quaternion
    Vector3 operator*(Vector3 vec) const
    {
        Rotation tmp;
        Vector3  result;

        tmp.w = 0.0;
        tmp.x = vec.x;
        tmp.y = vec.y;
        tmp.z = vec.z;

        tmp = (*this) * (tmp * this->GetInverse());

        result.x = tmp.x;
        result.y = tmp.y;
        result.z = tmp.z;

        return result;
    };
    // Get the inverse of this quaternion
    Rotation GetInverse() const
    {
        Rotation q;

        double norm = this->w * this->w + this->x * this->x + this->y * this->y + this->z * this->z;

        if (norm > 0.0)
        {
            q.w = this->w / norm;
            q.x = -this->x / norm;
            q.y = -this->y / norm;
            q.z = -this->z / norm;
        }

        return q;
    };
};

class Pose
{
public:
    Pose()
    {
        this->clear();
    };

    Vector3  position;
    Rotation rotation;

    void clear()
    {
        this->position.clear();
        this->rotation.clear();
    };
};

} // namespace urdf

#endif
