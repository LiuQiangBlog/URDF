#ifndef SEGMENTEDQUINTICBEZIERTOOLKIT_H_
#define SEGMENTEDQUINTICBEZIERTOOLKIT_H_
/* -------------------------------------------------------------------------- *
 *     OpenSim:  SegmentedQuinticBezierToolkit.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
/*
  Update:
  This is a port of the original code so that it will work with
  the multibody code RBDL written by Martin Felis.

  Author:
  Matthew Millard

  Date:
  Nov 2015

*/
#include <vector>
#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_config.h>
#include "Function.h"

/**
This is a low level Quintic Bezier curve class that contains functions to design
continuous sets of 'C' shaped Bezier curves, and to evaluate their values and
derivatives. A set in this context is used to refer to 2 or more quintic Bezier
curves that are continuously connected to eachother to form one smooth curve,
hence the name QuinticBezierSet.

In the special case when this class is being used to generate and evaluate
2D Bezier curves, that is x(u) and y(u), there are also functions to evaluate
y(x), the first six derivatives of y(x), and also the first integral of y(x).

This class was not designed to be a stand alone Quintic Bezier class, but rather
was developed out of necessity to model muscles. I required curves that, when
linearly extrapolated, were C2 continuous, and by necessity I had to use
quintic Bezier curves. In addition, the curves I was developing were functions
in x,y space, allowing many of the methods (such as the evaluation of y(x) given
that x(u) and y(u), the derivatives of y(x), and its first integral) to be
developed, though in general this can't always be done.

I have parcelled all of these tools into their own class so that others may more
easily use and develop this starting point for their own means. I used the
following text during the development of this code:

Mortenson, Michael E (2006). Geometric Modeling Third Edition. Industrial Press
Inc., New York. Chapter 4 was quite helpful.

<B>Future Upgrades</B>

1. Analytical Inverse to x(u):
  I think this is impossible because it is not possible, in general, to find the
  roots to a quintic polynomial, however, this fact may not preclude forming the
  inverse curve. The impossibility of finding the roots to a quintic polynomial
  was proven by Abel (Abel's Impossibility Theorem) and Galois

  http://mathworld.wolfram.com/QuinticEquation.html

  At the moment I am approximating the curve u(x) using cubic splines to return
  an approximate value for u(x), which I polish using Newton's method to the
  desired precision.

  **Note as of Nov 2015**
  -> The cubic spline approximation of the inverse curve has been
   removed. Since there is no spline class in RBDL (and I'm not
   motivated to port it over from SimTK) this functionality does
   not work. In addition, I've since found that this nice inverse
   only saves a few Newton iterations over a calculated guess.
   It's not worth the effort to include.

2. Analytical Integral of y(x):
  This is possible using the Divergence Theorem applied to 2D curves. A nice
  example application is shown in link 2 for computing the area of a closed
  cubic Bezier curve. While I have been able to get the simple examples to work,
  I have failed to successfully compute the area under a quintic Bezier curve
  correctly. I ran out of time trying to fix this problem, and so at the present
  time I am numerically computing the integral at a number of knot points and
  then evaluating the spline to compute the integral value.

  a. http://en.wikipedia.org/wiki/Divergence_theorem
  b. http://objectmix.com/graphics/133553-area-closed-bezier-curve.html

  **Note as of Nov 2015**
  ->  The splined numeric integral of the curve has been removed. There
    is not an error-controlled numerical integrator in RBDL and so
    it is not straight forward to include this feature.
  ->  For later: discuss options with Martin.

3. calcU:
  Currently the Bezier curve value and its derivative are computed separately to
  evaluate f and df in the Newton iteration to solve for u(x). Code optimized to
  compute both of these quantites at the same time could cut the cost of
  evaluating x(u) and dx/du in half. Since these quantities are evaluated in an
  iterative loop, this one change could yield substantial computational savings.

4. calcIndex:
The function calcIndex computes which spline the contains the point of interest.
This function has been implemented assuming a small number of Bezier curve sets,
and so it simply linearly scans through the sets to determine the correct one to
use. This function should be upgraded to use the bisection method if large
quintic Bezier curve sets are desired.

5. The addition of additional Bezier control point design algorithms, to create
  'S' shaped curves, and possibly do subdivision.

6. Low level Code Optimization:
I have exported all of the low level code as optimized code from Maple. Although
the code produced using this means is reasonably fast, it is usally possible
to obtain superior performance (and sometimes less round off error) by
doing this work by hand.

<B>Computational Cost Details</B>
All computational costs assume the following operation costs:

\verbatim
Operation Type   : #flops
+,-,=,Boolean Op : 1
     / : 10
     sqrt: 20
     trig: 40
\endverbatim

These relative weightings will vary processor to processor, and so any of
the quoted computational costs are approximate.


<B> RBDL Port Notes </B>

The port of this code from OpenSim has been accompanied by a few changes:

1. The 'calcIntegral' method has been removed. Why? This function
  relied on having access to a variable-step error controlled
  integrator. There is no such integrator built into RBDL. Rather
  than add a dependency (by using Boost perhaps) this functionality
  has been removed.

2. The function name .printMuscleCurveToFile(...) has been changed
  to .printCurveToFile().

3.  There have been some improvements to the function calcU in the
  SegmentedQuinticBezierToolkit.cc code. This function evaluates
  u such that x(u) - x* = 0. This is done using a Newton method.
  However, because these curves can be very nonlinear, the Newton
  method requires a very good initial start. In the OpenSim version
  this good initial guess was provided by splined interpolation of
  u(x). In the RBDL version this initial guess is provided by using
  a bisection method until the error of x(u)-x* is within
  sqrt(sqrt(tol)) or 2 Newton steps of the desired tolerance.

@author Matt Millard
@version 0.0

*/
namespace RigidBodyDynamics
{
namespace Addons
{
namespace Geometry
{

class RBDL_ADDON_DLLAPI SegmentedQuinticBezierToolkit
{

public:
    /**
  This scales the users value of curviness to be between [0+delta, 1-delta]
  because if curviness is allowed to equal 0 or 1, the second derivative
  becomes quite violent and the resulting curve is difficult to fit
  splines to.

  @param curviness
  @retval a scaled version of curviness

  */
    static double scaleCurviness(double curviness);

    /**
   This function will compute the u value that correesponds to the given x
   for a quintic Bezier curve.

   @param ax   The x value
   @param bezierPtsX The 6 Bezier point values
   @param tol  The desired tolerance on u.
   @param maxIter  The maximum number of Newton iterations allowed

   \b aborts \b
     -if ax is outside the range defined in this Bezier spline section
     -if the desired tolerance is not met
     -if the derivative goes to 0 to machine precision

   This function will compute the u value that correesponds to the given x
   for a quintic Bezier curve. This is accomplished by using an approximate
   spline inverse of u(x) to get a very good initial guess, and then one or
   two Newton iterations to polish the answer to the desired tolerance.

   <B>Computational Costs</B>
   \verbatim
     ~219 flops
   \endverbatim

   <B>Example:</B>
   @code
     double xVal = 2;

     //Choose the control points
     RigidBodyDynamics::Math::VectorNd vX(6);
     vX(0) = -2;
     vX(1) = 0;
     vX(2) = 0;
     vX(3) = 4;
     vX(4) = 4;
     vX(5) = 6;

     RigidBodyDynamics::Math::VectorNd x(100);
     RigidBodyDynamics::Math::VectorNd u(100);

     //Now evalutate u at the given xVal
     double u = SegmentedQuinticBezierToolkit::
        calcU(xVal,vX, 1e-12,20);

     @endcode
   */
    static double calcU(double ax, const RigidBodyDynamics::Math::VectorNd &bezierPtsX, double tol, int maxIter);

    /**
    Given a set of Bezier curve control points, return the index of the
    set of control points that x lies within.

    @param x    A value that is interpolated by the set of Bezier
          curves
    @param bezierPtsX   A matrix of 6xn Bezier control points

    \b aborts \b
    -If the index is not located within this set of Bezier points

    Given a set of Bezier curve control points, return the index of the
    set of control points that x lies within. This function has been coded
    assuming a small number of Bezier curve sets (less than 10), and so,
    it simply scans through the Bezier curve sets until it finds the correct
    one.


    <B>Computational Costs</B>
    Quoted for a Bezier curve set containing 1 to 5 curves.
    \verbatim
      ~9-25
    \endverbatim

    <B>Example:</B>
    @code
      RigidBodyDynamics::Math::MatrixNd mX(6,2);

      //The first set of spline points
      mX(0,0) = -2;
      mX(1,0) = -1;
      mX(2,0) = -1;
      mX(3,0) = 1;
      mX(4,0) = 1;
      mX(5,0) = 2;
      //The second set of spline points
      mX(0,1) = 2;
      mX(1,1) = 3;
      mX(2,1) = 3;
      mX(3,1) = 5;
      mX(4,1) = 5;
      mX(5,1) = 6;

      //The value of x for which we want the index for
      double xVal = 1.75;
      int idx  = SegmentedQuinticBezierToolkit::calcIndex(xVal,mX);
    @endcode


    */
    static int calcIndex(double x, const RigidBodyDynamics::Math::MatrixNd &bezierPtsX);

    static int calcIndex(double x, const std::vector<RigidBodyDynamics::Math::VectorNd> &bezierPtsX);

    /**
    Calculates the value of a quintic Bezier curve at value u.

    @param u  The independent variable of a Bezier curve, which ranges
        between 0.0 and 1.0.
    @param pts  The locations of the control points in 1 dimension.

    \b aborts \b
      -If u is outside of [0,1]
      -if pts has a length other than 6
    @return   The value of the Bezier curve located at u.

    Calculates the value of a quintic Bezier curve at value u. This
    calculation is acheived by mulitplying a row vector comprised of powers
    of u, by the 6x6 coefficient matrix associated with a quintic Bezier
    curve, by the vector of Bezier control points, pV, in a particular
    dimension. The code to compute the value of a quintic bezier curve has
    been optimized to have the following cost:

    <B>Computational Costs</B>
    \verbatim
     ~54 flops
    \endverbatim



    The math this function executes is decribed in pseudo code as the
    following:

    \verbatim
      uV = [u^5 u^4 u^3 u^2 u 1];

      cM = [ -1   5   -10   10  -5   1;
              5  -20   30  -20   5   0;
            -10   30  -30   10   0   0;
             10  -20   10    0   0   0;
             -5    5    0    0   0   0;
              1    0    0    0   0   0 ];
      pV = [x1; x2; x3; x4; x5; x6];

      xB = (uV*cM)*pV
    \endverbatim



    <B>Example:</B>
      @code
      double u = 0.5;

      //Choose the control points
      RigidBodyDynamics::Math::VectorNd vX(6);
      vX(0) = -2;
      vX(1) = 0;
      vX(2) = 0;
      vX(3) = 4;
      vX(4) = 4;
      vX(5) = 6;

      yVal = SegmentedQuinticBezierToolkit::
         calcQuinticBezierCurveVal(u,vX);
      @endcode


    */
    static double calcQuinticBezierCurveVal(double u, const RigidBodyDynamics::Math::VectorNd &pts);

    /**
    Calculates the value of a quintic Bezier derivative curve at value u.
    @param u  The independent variable of a Bezier curve, which ranges
        between 0.0 and 1.0.
    @param pts  The locations of the control points in 1 dimension.
    @param order  The desired order of the derivative. Order must be >= 1

    \b aborts \b
      -u is outside [0,1]
      -pts is not 6 elements long
      -if order is less than 1
    @return   The value of du/dx of Bezier curve located at u.

    Calculates the value of a quintic Bezier derivative curve at value u.
    This calculation is acheived by taking the derivative of the row vector
    uV and multiplying it by the 6x6 coefficient matrix associated with a
    quintic Bezier curve, by the vector of Bezier control points, pV, in a
    particular dimension.

    Pseudo code for the first derivative (order == 1) would be
    \verbatim
      uV = [5*u^4 4*u^3 3*u^2 2u 1 0];

      cM = [ -1     5   -10    10  -5   1;
              5   -20    30   -20   5   0;
            -10    30   -30    10   0   0;
             10   -20    10     0   0   0;
             -5     5     0     0   0   0;
              1     0     0     0   0   0 ];
      pV = [x1; x2; x3; x4; x5; x6];

      dxdu = (uV*cM)*pV
    \endverbatim

    Note that the derivative of uV only needed to be computed to compute
    dxdu. This process is continued for all 5 derivatives of x(u) until
    the sixth and all following derivatives, which are 0. Higher derivatives
    w.r.t. to U are less expensive to compute than lower derivatives.

    <B>Computational Costs</B>
    \verbatim
      dy/dx  : ~50 flops
      d2x/du2: ~43 flops
      d3x/du3: ~34 flops
      d4x/du4: ~26 flops
      d5x/du5: ~15 flops
      d6x/du6: ~1  flop
    \endverbatim


    <B>Example:</B>
      @code
      double u = 0.5;

      //Choose the control points
      RigidBodyDynamics::Math::VectorNd vX(6);
      vX(0) = -2;
      vX(1) = 0;
      vX(2) = 0;
      vX(3) = 4;
      vX(4) = 4;
      vX(5) = 6;

      double dxdu  =calcQuinticBezierCurveDerivU(u,vX,1);
      @endcode
    */
    static double calcQuinticBezierCurveDerivU(double u, const RigidBodyDynamics::Math::VectorNd &pts, int order);

    /**
    Calculates the value of dydx of a quintic Bezier curve derivative at u.

    @param u  The u value of interest. Note that u must be [0,1]
    @param xpts   The 6 control points associated with the x axis
    @param ypts   The 6 control points associated with the y axis
    @param order  The order of the derivative. Currently only orders from 1-6
        can be evaluated

    \b aborts \b
      -If u is outside [0,1]
      -If xpts is not 6 elements long
      -If ypts is not 6 elements long
      -If the order is less than 1
      -If the order is greater than 6
    @retval   The value of (d^n y)/(dx^n) evaluated at u

    Calculates the value of dydx of a quintic Bezier curve derivative at u.
    Note that a 2D Bezier curve can have an infinite number of derivatives,
    because x and y are functions of u. Thus

    \verbatim
    dy/dx = (dy/du)/(dx/du)
    d^2y/dx^2 = d/du(dy/dx)*du/dx
        = [(d^2y/du^2)*(dx/du) - (dy/du)*(d^2x/du^2)]/(dx/du)^2
        *(1/(dx/du))
    etc.
    \endverbatim

    <B>Computational Costs</B>

    This obviously only functions when the Bezier curve in question has a
    finite derivative. Additionally, higher order derivatives are more
    numerically expensive to evaluate than lower order derivatives. For
    example, here are the number of operations required to compute the
    following derivatives
    \verbatim
      Name  : flops
      dy/dx   : ~102
      d2y/dx2 : ~194
      d3y/dx3 : ~321
      d4y/dx4 : ~426
      d5y/dx5 : ~564
      d6y/dx6 : ~739
    \endverbatim

    <B>Example:</B>
      @code
      RigidBodyDynamics::Math::VectorNd vX(6), vY(6);

      double u = 0.5;

      vX(0) = 1;
      vX(1) = 1.01164;
      vX(2) = 1.01164;
      vX(3) = 1.02364;
      vX(4) = 1.02364;
      vY(5) = 1.04;

      vY(0) = 0;
      vY(1) = 3e-16;
      vY(2) = 3e-16;
      vY(3) = 0.3;
      vY(4) = 0.3;
      vY(5) = 1;


      d2ydx2 = SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivDYDX(
         u,vX, vY, 2);
      @endcode

    */
    static double calcQuinticBezierCurveDerivDYDX(double u,
                                                  const RigidBodyDynamics::Math::VectorNd &xpts,
                                                  const RigidBodyDynamics::Math::VectorNd &ypts,
                                                  int order);

    /**
    Calculates the location of quintic Bezier curve control points to
    create a C shaped curve like that shown in the figure. Using a series
    of these simple and predictably shaped Bezier curves it is easy to build
    quite complex curves.

    \image html fig_GeometryAddon_quinticCornerSections.png


    @param x0   First intercept x location
    @param y0   First intercept y location
    @param dydx0  First intercept slope
    @param x1   Second intercept x location
    @param y1   Second intercept y location
    @param dydx1  Second intercept slope
    @param curviness A parameter that ranges between 0 and 1 to denote a
         straight line or a curve

    \b aborts \b
     -If the curviness parameter is less than 0, or greater than 1;
     -If the points and slopes are chosen so that an "S" shaped curve would
      be produced. This is tested by examining the points (x0,y0) and
      (x1,y1) together with the intersection (xC,yC) of the lines beginning
      at these points with slopes of dydx0 and dydx1 form a triangle. If the
      line segment from (x0,y0) to (x1,y1) is not the longest line segment,
      an exception is thrown. This is an overly conservative test as it
      prevents very deep 'V' shapes from being respresented.

    @return a RigidBodyDynamics::Math::MatrixNd of 6 points Matrix(6,2) that
    correspond to the  X, and Y control points for a quintic Bezier curve that
    has the above properties


    Calculates the location of quintic Bezier curve control points to
    create a C shaped curve that intersects points 0 (x0, y0) and point 1
    (x1, y1) with slopes dydx0 and dydx1 respectively, and a second
    derivative of 0. The curve that results can approximate a line
    (curviness = 0), or in a smooth C shaped curve (curviniess = 1)

    The current implementation of this function is not optimized in anyway
    and has the following costs:

    <B>Computational Costs</B>
    \verbatim
      ~55 flops
    \endverbatim


    <B>Example:</B>
      @code
      double x0 = 1;
      double y0 = 0;
      double dydx0 = 0;
      double x1 = 1.04;
      double y1 = 1;
      double dydx1 = 43;
      double c = 0.75;

      RigidBodyDynamics::Math::MatrixNd p0 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x0, y0, dydx0,x1,y1,dydx01,
                     c);
      @endcode

    */
    static RigidBodyDynamics::Math::MatrixNd calcQuinticBezierCornerControlPoints(double x0,
                                                                                  double y0,
                                                                                  double dydx0,
                                                                                  double x1,
                                                                                  double y1,
                                                                                  double dydx1,
                                                                                  double curviness);

    /*
    This function numerically integrates the Bezier curve y(x).

    @param vX   Values of x to evaluate the integral of y(x)
    @param ic0  The initial value of the integral
    @param intAcc   Accuracy of the integrated solution
    @param uTol   Tolerance on the calculation of the intermediate u term
    @param uMaxIter Maximum number of iterations allowed for u to reach its
        desired tolerance.
    @param mX     The 6xn matrix of Bezier control points for x(u)
    @param mY     The 6xn matrix of Bezier control points for y(u)

    @param flag_intLeftToRight  Setting this flag to true will evaluate the
      integral from the left most point to the right most
      point. Setting this flag to false will cause the
      integral to be evaluated from right to left.
    @param name   Name of caller.
    @return RigidBodyDynamics::Math::MatrixNd Col 0: X vector, Col 1: int(y(x))


    This function numerically integrates the Bezier curve y(x), when really
    both x and y are specified in terms of u. Evaluate the integral at the
    locations specified in vX and return the result.

    <B>Computational Costs</B>

    This the expense of this function depends on the number of points in
    vX, the points for which the integral y(x) must be calculated. The
    integral is evaluated using a Runge Kutta 45 integrator, and so each
    point requires 6 function evaluations.
    (http://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method)

    The cost of evaluating 1 Bezier curve y(x) scales with the number
    of points in xVal:
    \verbatim
      ~1740 flops per point
    \endverbatim

    The example below is quite involved, but just so it can show you an
    example of how to create the array of Spline objects that approximate
    the function u(x). Although the example has been created for only 1
    Bezier curve set, simply changing the size and entries of the matricies
    _mX and _mY will allow multiple sets to be integrated.


    <B>Example:</B>
    @code
    //Integrator and u tolerance settings
    double INTTOL = 1e-12;
    double UTOL   = 1e-14;
    int  MAXITER= 10;

    //Make up a Bezier curve - these happen to be the control points
    //for a tendon curve
    RigidBodyDynamics::Math::MatrixNd _mX(6,1), _mY(6,1);
    _mX(0)= 1;
    _mX(1)= 1.01164;
    _mX(2)= 1.01164;
    _mX(3)= 1.02364;
    _mX(4)= 1.02364;
    _mX(5)= 1.04;

    _mY(0) = 0;
    _mY(1) = 3.10862e-16;
    _mY(2) = 3.10862e-16;
    _mY(3) = 0.3;
    _mY(4) = 0.3;
    _mY(5) = 1;

    _numBezierSections = 1;
    bool _intx0x1 = true; //Says we're integrating from x0 to x1
    //////////////////////////////////////////////////
    //Generate the set of splines that approximate u(x)
    //////////////////////////////////////////////////
    RigidBodyDynamics::Math::VectorNd u(NUM_SAMPLE_PTS);
    //Used for the approximate inverse
    RigidBodyDynamics::Math::VectorNd x(NUM_SAMPLE_PTS);
    //Used for the approximate inverse

    //Used to generate the set of knot points of the integral of y(x)
    RigidBodyDynamics::Math::VectorNd
    xALL(NUM_SAMPLE_PTS*_numBezierSections-(_numBezierSections-1));
    _arraySplineUX.resize(_numBezierSections);
    int xidx = 0;

    for(int s=0; s < _numBezierSections; s++){
      //Sample the local set for u and x
      for(int i=0;i<NUM_SAMPLE_PTS;i++){
      u(i) = ( (double)i )/( (double)(NUM_SAMPLE_PTS-1) );
      x(i) = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveVal(u(i),_mX(s),_name);
      if(_numBezierSections > 1){
         //Skip the last point of a set that has another set of points
         //after it. Why? The last point and the starting point of the
         //next set are identical in value.
        if(i<(NUM_SAMPLE_PTS-1) || s == (_numBezierSections-1)){
        xALL(xidx) = x(i);
        xidx++;
        }
      }else{
        xALL(xidx) = x(i);
        xidx++;
      }
      }
      //Create the array of approximate inverses for u(x)
      _arraySplineUX[s] = SimTK::SplineFitter<double>::
      fitForSmoothingParameter(3,x,u,0).getSpline();
    }


    //////////////////////////////////////////////////
    //Compute the integral of y(x) and spline the result
    //////////////////////////////////////////////////

    RigidBodyDynamics::Math::VectorNd yInt =  SegmentedQuinticBezierToolkit::
      calcNumIntBezierYfcnX(xALL,0,INTTOL, UTOL, MAXITER,_mX, _mY,
      _arraySplineUX,_name);

    if(_intx0x1==false){
      yInt = yInt*-1;
      yInt = yInt - yInt(yInt.nelt()-1);
    }

    _splineYintX = SimTK::SplineFitter<double>::
      fitForSmoothingParameter(3,xALL,yInt,0).getSpline();

    @endcode


    */

    // MM:   Can't port this over to RBDL as RBDL doesn't have an error
    //   controlled integrator. I could add this if a dependency
    //   like Boost was allowed.
    //
    //   static RigidBodyDynamics::Math::MatrixNd
    //   calcNumIntBezierYfcnX(
    //     const RigidBodyDynamics::Math::VectorNd& vX,
    //     double ic0,
    //     double intAcc,
    //     double uTol,
    //     int uMaxIter,
    //     const RigidBodyDynamics::Math::MatrixNd& mX,
    //      const RigidBodyDynamics::Math::MatrixNd& mY,
    //     const SimTK::Array_<SimTK::Spline>& aSplineUX,
    //     bool flag_intLeftToRight,const std::string& name);

private:
    /**
    This function will print cvs file of the column vector col0 and the
    matrix data.

    @param col0   A vector that must have the same number of rows as the
        data matrix. This column vector is printed as the first
        column
    @param data   A matrix of data
    @param filename The name of the file to print
    */
    static void printMatrixToFile(const RigidBodyDynamics::Math::VectorNd &col0,
                                  const RigidBodyDynamics::Math::MatrixNd &data,
                                  std::string &filename);

    /**
      @param curveFit a function that evaluates a curve
      @param ctrlPts control point locations for the fitted Bezier curve
      @param xVal the x values at the control points
      @param yVal the y values at the control points
      @param filename of the output file.

    */
    static void printBezierSplineFitCurves(const Function_<double> &curveFit,
                                           RigidBodyDynamics::Math::MatrixNd &ctrlPts,
                                           RigidBodyDynamics::Math::VectorNd &xVal,
                                           RigidBodyDynamics::Math::VectorNd &yVal,
                                           std::string &filename);

    /**
    This function will return a value that is equal to u, except when u is
    outside of[0,1], then it is clamped to be 0, or 1
    @param u  The parameter to be clamped
    @retval u but restricted to 0,1.
    */
    static double clampU(double u);

    ///@cond
    /**
    Class that contains data that describes the Bezier curve set. This class is used
    by the function calcNumIntBezierYfcnX, which evaluates the numerical integral
    of a Bezier curve set.
    */
    class BezierData
    {
    public:
        /**A 6xn matrix of Bezier control points for the X axis (domain)*/
        RigidBodyDynamics::Math::MatrixNd _mX;
        /**A 6xn matrix of Bezier control points for the Y axis (range)*/
        RigidBodyDynamics::Math::MatrixNd _mY;
        /**An n element array containing the approximate spline fits of the
        inverse function of x(u), namely u(x)*/
        // std::vector< std::vector<double> > _aArraySplineUX;
        /**The initial value of the integral*/
        double _initalValue;
        /**The tolerance to use when computing u. Solving u(x) can only be done
        numerically at the moment, first by getting a good guess (using the
        splines) and then using Newton's method to polish the value up. This
        is the tolerance that is used in the polishing stage*/
        double _uTol;
        /**The maximum number of interations allowed when evaluating u(x) using
        Newton's method. In practice the guesses are usually very close to the
        actual solution, so only 1-3 iterations are required.*/
        int _uMaxIter;
        /**If this flag is true the function is integrated from its left most
        control point to its right most. If this flag is false, the function
        is integrated from its right most control point to its left most.*/
        // bool  _flag_intLeftToRight;
        /**The starting value*/
        // double _startValue;

        /**The name of the curve being intergrated. This is used to generate
        useful error messages when something fails*/
        std::string _name;
    };
    ///@endcond
};

} // namespace Geometry
} // namespace Addons
} // namespace RigidBodyDynamics

#endif
