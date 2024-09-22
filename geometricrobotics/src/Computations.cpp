/*
 * Copyright (c) 2022, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "GeometricRobotics/Computations.h"
namespace GeoRobotics
{
Matrix4d hat(const Vector6d & xi)
{

  // Assumes the angular part at the end.
  // return hat(xi.segment<3>(0), xi.segment<3>(3));
  return hat(xi.head<3>(), xi.tail<3>());
  /*
  Matrix4d output = Matrix4d::Zero();

  output.block<3,3>(0, 0) = skew(xi.segment<3>(3));

  output.block<3,1>(0, 3) = xi.segment<3>(0);

  return output;
  */
}

Matrix4d hat(const TwistCoordinate & xi)
{
  return hat(xi.linear, xi.angular);
}

Matrix4d hat(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular)
{
  Matrix4d output = Matrix4d::Zero();

  output.block<3, 3>(0, 0).noalias() = skew(angular);

  output.block<3, 1>(0, 3).noalias() = linear;

  return output;
}

void hat(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular, Matrix4d & twist)
{
  twist.setZero();
  twist.block<3, 3>(0, 0).noalias() = skew(angular);
  twist.block<3, 1>(0, 3).noalias() = linear;
}

void skew(const Eigen::Vector3d & input, Matrix3d & output)
{
  output.noalias() = Matrix3d::Zero();
  output(0, 1) = -input(2);
  output(1, 0) = input(2);

  output(0, 2) = input(1);
  output(2, 0) = -input(1);

  output(1, 2) = -input(0);
  output(2, 1) = input(0);
}

Matrix3d skew(const Eigen::Vector3d & input)
{
  Matrix3d skewSymmetricMatrix = Eigen::Matrix3d::Zero();

  skewSymmetricMatrix(0, 1) = -input(2);
  skewSymmetricMatrix(1, 0) = input(2);

  skewSymmetricMatrix(0, 2) = input(1);
  skewSymmetricMatrix(2, 0) = -input(1);

  skewSymmetricMatrix(1, 2) = -input(0);
  skewSymmetricMatrix(2, 1) = input(0);

  return skewSymmetricMatrix;
}

bool checkNull(const Vector3d & axis)
{
  return (std::fabs((axis.norm() - 0.0)) <= GEO_UNIT_EPSILON);
}
bool checkUnit(const Vector3d & axis)
{
  return (std::fabs((axis.norm() - 1.0)) <= GEO_UNIT_EPSILON);
}

#define EPSILON_EXPMAP_THETA 1.0e-3
Matrix3d expMap(const Vector3d & vec)
{

  double theta = vec.norm();
  return expMap(vec / theta, theta);
}

Matrix3d expMap(Vector3d const & axis, double const & theta)
{
  if(!axis.isUnitary())
  {
    RoboticsUtils::throw_runtime_error("The input axis is not a unit vector.", __FILE__, __LINE__);
  }
  Matrix3d const output = Matrix3d::Identity();

  Matrix3d const s_axis = skew(axis);
  Matrix3d const s_axis_2 = s_axis * s_axis;

  if(fabs(theta) <= EPSILON_EXPMAP_THETA)
  {
    // Near-zero angle approximation
    return output + s_axis * theta + 0.5 * s_axis_2 * theta * theta;
  }

  return output + s_axis * sin(theta) + s_axis_2 * (1 - cos(theta));
}

void expMap(Vector3d const & vec, Matrix3d & R, Matrix3d & A)
{

  double theta = vec.norm();
  // Vector3d axis = vec / theta;

  Matrix3d const id = Matrix3d::Identity();

  Matrix3d const s_vec = skew(vec);
  Matrix3d const s_vec_2 = s_vec * s_vec;
  double theta_2 = theta * theta;
  double theta_3 = theta_2 * theta;

  if(fabs(theta) <= EPSILON_EXPMAP_THETA)
  {
    R.noalias() = id + s_vec + 0.5 * s_vec_2;
  }
  else
  {
    R.noalias() = id + s_vec * sin(theta) / theta + s_vec_2 * (1 - cos(theta)) / theta_2;
  }

  // A.noalias() = (output - R) * skew(axis) + axis * axis.transpose() * theta;
  A.noalias() = id + s_vec / theta_2 * (1 - cos(theta)) + s_vec_2 / theta_3 * (theta - sin(theta));
}

/*
void expMap(const Vector3d & vec, Matrix3d & R, Matrix3d & A)
{

  double theta = vec.norm();

  Matrix3d output = Matrix3d::Identity();

  auto s_vec = skew(vec);
  auto s_vec_2 = s_vec * s_vec;
  double theta_2 = theta * theta;
  double theta_3 = theta_2 * theta;

  if(theta <= EPSILON_EXPMAP_THETA)
  {
    R = output + s_vec + 0.5 * s_vec_2;
  }
  else
  {
    R = output + s_vec * sin(theta) / theta + s_vec_2 * (1 - cos(theta)) / theta_2;
  }

  A = output + s_vec * (1 - cos(theta)) / theta_2 + s_vec_2 * (theta - sin(theta)) / theta_3;
}
*/

Eigen::Isometry3d expMap(TwistCoordinate const & xi, double const & theta)
{
  return expMap(xi.linear * theta, xi.angular * theta);
}

Eigen::Isometry3d expMap(const TwistCoordinate & xi)
{
  return expMap(xi.linear, xi.angular);
}

Eigen::Isometry3d expMap(Vector3d const & linear, Vector3d const & angular)
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();

  if(angular.norm() < EPSILON_EXPMAP_THETA)
  {
    output.translation() = linear;
  }
  else
  {
    Matrix3d R, A;

    expMap(angular, R, A);

    output.linear().noalias() = R;
    output.translation().noalias() = A * linear;
  }

  return output;
}

Eigen::Isometry3d expMap(const Vector6d & xi)
{
  const Vector3d & angular = xi.tail<3>();
  const Vector3d & linear = xi.head<3>();

  return expMap(linear, angular);
}

void metaAdgInvTwo(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output)
{
  const Matrix3d & RT = transform.linear().transpose();
  const Vector3d & t = transform.translation();

  output.linear.noalias() = RT * input.linear - skew(RT * t) * RT * input.angular;
  output.angular.noalias() = RT * input.angular;
}
TwistCoordinate metaAdgInv(const Iso3d & transform, const TwistCoordinate & input)
{
  TwistCoordinate output;

  metaAdgInv(transform, input, output);

  return output;
}
void metaAdgInv(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output)
{
  const Matrix3d & RT = transform.linear().transpose();
  const Vector3d & t = transform.translation();
  output.linear.noalias() = RT * input.linear - RT * skew(t) * input.angular;
  output.angular.noalias() = RT * input.angular;
}

TwistCoordinate metaAdg(const Iso3d & transform, const TwistCoordinate & input)
{
  TwistCoordinate output;
  metaAdg(transform, input, output);
  return output;
}
void metaAdg(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output)
{
  const Matrix3d & R = transform.linear();
  const Vector3d & t = transform.translation();

  output.linear.noalias() = R * input.linear + skew(t) * R * input.angular;
  output.angular.noalias() = R * input.angular;
}
Matrix6d metaAdgMatrix(const Iso3d & transform)
{
  Matrix6d output;
  const Matrix3d & R = transform.linear();
  const Matrix3d & st = skew(transform.translation());

  output << R, st * R, Matrix3d::Zero(), R;

  return output;
}
Matrix6d metaAdgTMatrix(const Iso3d & transform)
{
  Matrix6d output;
  const Matrix3d & RT = transform.linear().transpose();
  const Matrix3d & st = skew(transform.translation());

  output << RT, Matrix3d::Zero(), -RT * st, RT;

  return output;
}

Matrix6d metaAdgInvMatrix(const Iso3d & transform)
{
  Matrix6d output;
  const Matrix3d & RT = transform.linear().transpose();
  const Matrix3d & st = skew(transform.translation());

  output << RT, -RT * st, Matrix3d::Zero(), RT;

  return output;
}

Matrix6d metaAdgInvTwoMatrix(const Iso3d & transform)
{
  Matrix6d output;
  const Matrix3d & RT = transform.linear().transpose();
  const Vector3d & t = transform.translation();

  output << RT, -skew(RT * t) * RT, Matrix3d::Zero(), RT;

  return output;
}

void metaAdgT(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output)
{

  const Matrix3d & RT = transform.linear().transpose();
  const Vector3d & t = transform.translation();
  output.linear.noalias() = RT * input.linear;
  output.angular.noalias() = -RT * skew(t) * input.linear + RT * input.angular;
}

TwistCoordinate metaTwistTransform(const Iso3d & transform, const TwistCoordinate & input)
{
  TwistCoordinate output;
  metaAdgInv(transform, input, output);
  return output;
}

/*
Matrix4d metaTwistTransform(const Iso3d & transform, const Matrix4d & input)
{
  return (transform * input * transform.inverse(Eigen::Isometry)).matrix();
}
*/

Matrix6d crossMatrix(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular)
{
  Matrix6d output = Matrix6d::Zero();

  const Matrix3d & w_hat = skew(angular);

  output.block<3, 3>(0, 0).noalias() = w_hat;
  output.block<3, 3>(3, 3).noalias() = w_hat;

  output.block<3, 3>(0, 3).noalias() = skew(linear);

  return output;
}

Matrix6d crossMatrix(const Vector6d & vel)
{
  return crossMatrix(vel.head(3), vel.tail(3));
}

Vector6d cross(const Vector6d & one, const Vector6d & two)
{

  Vector6d output = Vector6d::Zero();

  const Matrix3d & w_one_hat = skew(one.tail(3));
  const Matrix3d & v_one_hat = skew(one.head(3));

  output.head(3).noalias() = w_one_hat * two.head(3) + v_one_hat * two.tail(3);

  output.tail(3).noalias() = w_one_hat * two.tail(3);

  return output;
}
Matrix6d dualCrossMatrix(const Vector6d & input)
{
  const Vector3d & linear = input.head(3);
  const Vector3d & angular = input.tail(3);

  Matrix6d output = Matrix6d::Zero();

  const Matrix3d & w_hat = skew(angular);

  output.block<3, 3>(0, 0).noalias() = w_hat;
  output.block<3, 3>(3, 3).noalias() = w_hat;

  output.block<3, 3>(3, 0).noalias() = skew(linear);

  return output;
}

Vector6d dualCross(const Vector6d & one, const Vector6d & two)
{
  Vector6d output = Vector6d::Zero();

  const Matrix3d & w_one_hat = skew(one.tail(3));
  const Matrix3d & v_one_hat = skew(one.head(3));

  output.head(3).noalias() = w_one_hat * two.head(3);

  output.tail(3).noalias() = v_one_hat * two.head(3) + w_one_hat * two.tail(3);

  return output;
}

void TwistCoordinate::print() const
{
  std::cout << RoboticsUtils::info << "Twist coordinate has the linear part: " << linear.transpose()
            << ", and the angular part: " << angular.transpose() << std::endl;
}
double TwistCoordinate::norm() const
{
  return linear.norm() + angular.norm();
}

bool TwistCoordinate::operator==(const TwistCoordinate & t) const
{
  auto result = *this - t;
  return ::GeoRobotics::checkNull(result.linear) && ::GeoRobotics::checkNull(result.angular);
}

TwistCoordinate TwistCoordinate::operator-(const TwistCoordinate & t) const
{
  TwistCoordinate output;
  output.linear.noalias() = linear - t.linear;
  output.angular = angular - t.angular;

  return output;
}
TwistCoordinate TwistCoordinate::operator+(const Vector6d & t) const
{
  TwistCoordinate output;
  output.linear.noalias() = linear + t.head(3);
  output.angular = angular + t.tail(3);

  return output;
}
TwistCoordinate TwistCoordinate::operator-(const Vector6d & t) const
{
  TwistCoordinate output;
  output.linear.noalias() = linear - t.head(3);
  output.angular = angular - t.tail(3);

  return output;
}
Vector6d TwistCoordinate::vector() const
{
  Vector6d output;
  output.head<3>() = linear;
  output.tail<3>() = angular;
  return output;
}

TwistCoordinate TwistCoordinate::operator+(const TwistCoordinate & t) const
{
  TwistCoordinate output;
  output.linear.noalias() = linear + t.linear;
  output.angular = angular + t.angular;

  return output;
}
TwistCoordinate TwistCoordinate::operator*(const TwistCoordinate & t) const
{
  TwistCoordinate output;
  output.linear.noalias() = angular.cross(t.linear) + linear.cross(t.angular);
  output.angular = angular.cross(t.angular);

  return output;
}

Eigen::Isometry3d TwistCoordinate::expMap() const
{
  return ::GeoRobotics::expMap(linear, angular);
}

Matrix6d TwistCoordinate::crossMatrix() const
{
  return ::GeoRobotics::crossMatrix(linear, angular);
}
void TwistCoordinate::prismatic_reset(const Vector3d & axis)
{
  if(!axis.isUnitary())
  {
    RoboticsUtils::throw_runtime_error("The input axis is not a unit vector.", __FILE__, __LINE__);
  }
  angular.setZero();
  linear.noalias() = axis;
}

void TwistCoordinate::revolute_reset(const Vector3d & point, const Vector3d & axis)
{
  if(!axis.isUnitary())
  {
    RoboticsUtils::throw_runtime_error("The input axis is not a unit vector.", __FILE__, __LINE__);
  }

  angular.noalias() = axis;
  linear.noalias() = -angular.cross(point);
}

double grNumber(std::default_random_engine & re, const double & lb, const double & ub)
{
  std::uniform_real_distribution<double> unif(lb, ub);
  return unif(re);
}
double grNumber(const double & lb, const double & ub)
{
  static std::random_device rd; // seed the std::mt19937 random number engine once
  static std::mt19937 gen(rd()); // The static keyword ensures that the random number engine and distribution objects
                                 // persist across multiple function calls.

  std::uniform_real_distribution<double> unif(lb, ub);
  return unif(gen);
}

AngleAxisd grAngleAxis()
{
  Eigen::Vector3d newVec = grVec3d(-m_pi, m_pi, true);

  return AngleAxisd((AngleAxisd(newVec(0) * m_pi, Vector3d::UnitX()) * AngleAxisd(newVec(1) * m_pi, Vector3d::UnitY())
                     * AngleAxisd(newVec(2) * m_pi, Vector3d::UnitZ()).matrix()));
}

Iso3d grTF(double lb, double ub)
{
  Iso3d tf = Iso3d::Identity();
  tf.linear() = grRotation();
  tf.translation() = grVec3d(lb, ub, false);
  return tf;
}

Matrix3d grRotation()
{
  Eigen::Vector3d newVec = grVec3d(-m_pi, m_pi, true);

  return (Eigen::AngleAxisd(newVec(0) * m_pi, Vector3d::UnitX())
          * Eigen::AngleAxisd(newVec(1) * m_pi, Vector3d::UnitY())
          * Eigen::AngleAxisd(newVec(2) * m_pi, Vector3d::UnitZ()))
      .matrix();
}

Eigen::VectorXd grVec(double lb, double ub, bool normalize, int size)
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(lb, ub);

  Eigen::VectorXd newVec(size);

  for(int i = 0; i < size; ++i)
  {
    newVec(i) = dis(gen);
  }

  if(normalize)
  {
    newVec.normalize();
  }

  return newVec;
}

Eigen::Vector3d grVec3d(double lb, double ub, bool normalize)
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(lb, ub);

  Vector3d newVec(dis(gen), dis(gen), dis(gen));

  if(normalize)
  {
    newVec.normalize();
  }

  return newVec;
}

Vector3d grVec3d(std::default_random_engine & re, const double & lb, const double & ub, const bool & normalize)
{

  // std::uniform_int_distribution<double> dist{lb, ub};
  std::uniform_real_distribution<double> unif(lb, ub);
  // std::default_random_engine re;

  Vector3d newVec(unif(re), unif(re), unif(re));

  if(normalize)
  {
    newVec.normalize();
  }

  return newVec;
}

void grVec3d(std::vector<Vector3d> & bag,
             const size_t & size,
             const double & lb,
             const double & ub,
             const bool & normalize)
{

  // std::uniform_int_distribution<double> dist{lb, ub};
  std::uniform_real_distribution<double> unif(lb, ub);
  std::vector<double> vec(size * 3);
  std::default_random_engine re;

  for(size_t idx = 0; idx < size; idx++)
  {
    Vector3d newVec(unif(re), unif(re), unif(re));

    if(normalize)
    {
      newVec.normalize();
    }

    bag.push_back(newVec);
  }
}

// Integer randInt(Integer n) {
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_int_distribution<Integer> dist(0, n);
//     return dist(gen);
// }

template<typename Integer>
Integer randInt(Integer n)
{
  if((n - 1) == RAND_MAX)
  {
    return rand();
  }
  else
  {
    // Supporting larger values for n would requires an even more
    // elaborate implementation that combines multiple calls to rand()
    assert(n <= RAND_MAX);

    // Chop off all of the values that would cause skew...
    Integer end = RAND_MAX / n; // truncate skew
    assert(end > 0);
    end *= n;

    // ... and ignore results from rand() that fall above that limit.
    // (Worst case the loop condition should succeed 50% of the time,
    // so we can expect to bail out of this loop pretty quickly.)
    Integer r;
    while((r = rand()) >= end)
      ;

    return r % n;
  }
}

template int randInt<int>(int n);
template size_t randInt<size_t>(size_t n);
template long int randInt<long int>(long int n);

} // namespace GeoRobotics
