// SislNurbsGen
// Copyright (C) 2024 KION Group AG
//
// All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// Author
//     Tino Krueger <tino.krueger@kiongroup.com>
//     Ilef Mghirbi <ilef.mghirbi@kiongroup.com>

#ifndef SISLNURBSGEN_H
#define SISLNURBSGEN_H

#include "main/defines/position2d.h"
#include "main/defines/point2d.h"
#include "navigation/path_proxy.h"

#include "sisl.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>

class CurveParameter {
 public:
  int order{6};  /// order of spline curve (degree + 1)
  int kind{1};   /// kind = polynomial B-spline curve
  int dim{2};    /// dimension
  CurveParameter() {}
};
class SislNurbsGen {
 protected:
  struct SislPoint {
    double x{};
    double y{};
  };

  std::unordered_map<int, std::string> err_map{
      {-101, "RackSplErr: -101 Error in memory allocation."},
      {-102, "RackSplErr: -102 Error in input. Dimension less than 1."},
      {-103, "RackSplErr: -103 Error in input. Dimension less than 2."},
      {-104, "RackSplErr: -104 Error in input. Dimension not equal 3."},
      {-105, "RackSplErr: -105 Error in input. Dimension not equal 2 or 3."},
      {-106, "RackSplErr: -106 Error in input. Conflicting dimensions."},
      {-107, "RackSplErr: -107 Unknown error."},
      {-108, "RackSplErr: -108 Error in input. Dimension not equal 2."},
      {-109, "RackSplErr: -109 Error in input. Order less than 2."},
      {-110, "RackSplErr: -110 Error in Curve description. Order less than 1."},
      {-111,
       "RackSplErr: -111 Error in Curve description. Number of vertices less "
       "than order."},
      {-112,
       "RackSplErr: -112 Error in Curve description. Error in knot vector."},
      {-113,
       "RackSplErr: -113 Error in Curve description. Unknown kind of Curve."},
      {-114,
       "RackSplErr: -114 Error in Curve description. Open Curve when expecting "
       "closed."},
      {-115, "RackSplErr: -115 Error in Surf description. Order less than 1."},
      {-116,
       "RackSplErr: -116 Error in Surf description. Number of vertices less "
       "than order."},
      {-117,
       "RackSplErr: -117 Error in Surf description. Error in knot vector."},
      {-118,
       "RackSplErr: -118 Error in Surf description. Unknown kind of Surf."},
      {-119, "RackSplErr: -119 Unknown error."},
      {-120, "RackSplErr: -120 Error in input. Negative relative tolerance."},
      {-121, "RackSplErr: -121 Error in input. Unknown kind of Object."},
      {-122,
       "RackSplErr: -122 Error in input. Unexpected kind of Object found."},
      {-123,
       "RackSplErr: -123 Error in input. Parameter direction does not exist."},
      {-124,
       "RackSplErr: -124 Error in input. Zero length parameter interval."},
      {-125, "RackSplErr: -125 Unknown error."},
      {-126, "RackSplErr: -126 Unknown error."},
      {-127, "RackSplErr: -127 Error in input. The whole curve lies on axis."},
      {-128, "RackSplErr: -128 Unkown error."},
      {-129, "RackSplErr: -129 Unknown error."},
      {-130,
       "RackSplErr: -130 Error in input. Parameter value is outside parameter "
       "area."},
      {-131, "RackSplErr: -131 Unknown error."},
      {-132, "RackSplErr: -132 Unknown error."},
      {-133, "RackSplErr: -133 Unknown error."},
      {-134, "RackSplErr: -134 Unknown error."},
      {-135,
       "RackSplErr: -135 Error in data structure. Intersection point exists "
       "when it should not."},
      {-136,
       "RackSplErr: -136 Error in data structure. Intersection list exists "
       "when it should not."},
      {-137,
       "RackSplErr: -137 Error in data structure. Expected intersection point "
       "not found."},
      {-138,
       "RackSplErr: -138 Error in data structure. Wrong number of "
       "intersections on edges/endpoints."},
      {-139, "RackSplErr: -139 Error in data structure."},
      {-140,
       "RackSplErr: -140 Error in data structure. Intersection interval "
       "crosses subdivision line when not expected to."},
      {-141, "RackSplErr: -141 Error in input. Illegal edge point requested."},
      {-142, "RackSplErr: -142 Unknown error"},
      {-143, "RackSplErr: -143 Unknown error"},
      {-144, "RackSplErr: -144 Unknown kind of intersection curve."},
      {-145,
       "RackSplErr: -145 Unknown kind of intersection list (internal format)."},
      {-146, "RackSplErr: -146 Unknown kind of intersection type."},
      {-147, "RackSplErr: -147 Unknown error"},
      {-148, "RackSplErr: -148 Unknown error"},
      {-149, "RackSplErr: -149 Unknown error"},
      {-150, "RackSplErr: -150 Error in input. NULL pointer was given."},
      {-151,
       "RackSplErr: -151 Error in input. One or more illegal input values."},
      {-152, "RackSplErr: -152 Too many knots to insert."},
      {-153,
       "RackSplErr: -153 Lower level routine reported error. SHOULD use label "
       "\"error\"."},
      {-154, "RackSplErr: -154 Unknown error"},
      {-155, "RackSplErr: -155 Unknown error"},
      {-156, "RackSplErr: -156 Unknown error"},
      {-157,
       "RackSplErr: -157 Illegal derivative requested. Change this label to "
       "err178."},
      {-158, "RackSplErr: -158 Intersection point outside Curve."},
      {-159,
       "RackSplErr: -159 No of vertices less than 1. SHOULD USE err111 or "
       "err116."},
      {-160, "RackSplErr: -160 Error in dimension of interpolation problem."},
      {-161, "RackSplErr: -161 Error in interpolation problem."},
      {-162, "RackSplErr: -162 Matrix may be noninvertible."},
      {-163, "RackSplErr: -163 Matrix part contains diagonal elements."},
      {-164,
       "RackSplErr: -164 No point conditions specified in interpolation "
       "problem."},
      {-165, "RackSplErr: -165 Error in interpolation problem."},
      {-166, "RackSplErr: -166 Unknown error"},
      {-167, "RackSplErr: -167 Unknown error"},
      {-168, "RackSplErr: -168 Unknown error"},
      {-169, "RackSplErr: -169 Unknown error"},
      {-170, "RackSplErr: -170 Internal error: Error in moving knot values."},
      {-171,
       "RackSplErr: -171 Memory allocation failure: Could not create curve or "
       "surface."},
      {-172, "RackSplErr: -172 Input error, inarr < 1 || inarr > 3."},
      {-173, "RackSplErr: -173 Direction vector zero length."},
      {-174, "RackSplErr: -174 Degenerate condition."},
      {-175, "RackSplErr: -175 Unknown degree/type of implicit surface."},
      {-176, "RackSplErr: -176 Unexpected iteration situation."},
      {-177,
       "RackSplErr: -177 Error in input. Negative step length requested."},
      {-178, "RackSplErr: -178 Illegal derivative requested."},
      {-179, "RackSplErr: -179 No. of Curves < 2."},
      {-180, "RackSplErr: -180 Error in torus description."},
      {-181, "RackSplErr: -181 Too few points as input."},
      {-182, "RackSplErr: -182 Unknown error"},
      {-183, "RackSplErr: -183 Order(s) specified to low."},
      {-184, "RackSplErr: -184 Negative tolerance given."},
      {-185, "RackSplErr: -185 Only degenerate or singular guide points."},
      {-186, "RackSplErr: -186 Special error in traversal of curves."},
      {-187, "RackSplErr: -187 Error in description of input curves."},
      {-188, "RackSplErr: -188 Unknown error"},
      {-189, "RackSplErr: -189 Unknown error"},
      {-190, "RackSplErr: -190 Too small array for storing Curve segments."},
      {-191, "RackSplErr: -191 Error in inserted parameter number."},
      {-192, "RackSplErr: -192 Unknown error"},
      {-193, "RackSplErr: -193 Unknown error"},
      {-194, "RackSplErr: -194 Unknown error"},
      {-195, "RackSplErr: -195 Unknown error"},
      {-196, "RackSplErr: -196 Unknown error"},
      {-197, "RackSplErr: -197 Unknown error"},
      {-198, "RackSplErr: -198 Unknown error"},
      {-199, "RackSplErr: -199 Unknown error"}};

 private:
  CurveParameter         param_{};
  SISLCurve*             curve_{nullptr};

  std::vector<double>    knots_;
  std::vector<SislPoint> ctrl_;
  std::vector<SislPoint> points_;
  std::vector<int>       points_type_;
  static constexpr int   kX            = 0;
  static constexpr int   kY            = 1;
  static constexpr int   kZ            = 2;
  bool                   knots_created = false;
  bool                   params_set    = false;
  bool                   is_created_   = false;
  int                    err_num_      = 0;

  enum DISTANCETYPE { EUCLIDEAN = 0, MANHATTAN = 1 };
  static constexpr float kDivZeroPrevention = 0.46e-9;  //>to prevent an
                                                        // division through
                                                        // zero, this is the
                                                        // proving value
                                                        // 1/(2^31) floating
                                                        // point mapping

  /**
   * @brief CreateKnotVector
   * @param n number of control points
   * @param p degree of the polynom
   * @return returns true on success otherwise false;
   */
  bool                   CreateKnotVector(int n, int p);

  /**
   * @brief GetAngleBasedDerivative
   * To compute the first derivative of the curve at a given angle.
   * @param angle
   * @return SislPoint
   */
  SislPoint              GetAngleBasedDerivative(const double angle) {
    SislPoint p{1.0, 0.0};
    p.y = tan(angle);
    return p;
  }

  /**
   * @brief isEqual
   * use this function most of the time to compare two different floating
   * point numbers. Tolerance needs to be meaningful in your context.
   * Default tolerance is DIVZERO_PREVENTION to be safe while using a
   * denominator.
   * @warning please be aware, that the function is trimmed to be easy readable
   * and high performance. Relative errors and ULPS are not considered. Your
   * solution is therefore realated to the floating point representation of your
   * values.
   * @param a floating point number to compare
   * @param b sec floating point number to compare
   * @param tolerance comparism parameter to evaluate equality
   * @return returns true if a is approximately == b and false otherwise
   */
  template <typename T>
  static constexpr bool isEqual(
      T a, T b, T tolerance = static_cast<T>(kDivZeroPrevention)) {
    static_assert(std::is_floating_point_v<T>,
                  "Floating point comparisons "
                  "require type float, double, or long double.");

    if (std::abs(a - b) <= tolerance) return true;

    return false;
  }

  /**
   * @brief isEqualZero
   * use this function to compare single floating
   * point to zero. Tolerance needs to be meaningful in your context.
   * Default tolerance is DIVZERO_PREVENTION to be safe while
   * using a denominator.
   * @param a floating point number to compare with zero
   * @param tolerance comparism parameter to evaluate equality
   * @return returns true if a is approximately == 0, and false otherwise
   */
  template <typename T>
  static constexpr bool isEqualZero(
      T a, T tolerance = static_cast<T>(kDivZeroPrevention)) {
    static_assert(std::is_floating_point_v<T>,
                  "Floating point comparisons "
                  "require type float, double, or long double.");
    return isEqual(a, static_cast<T>(0.0), tolerance);
  }

  /*!
   * @brief Calculate the squared euclidean distance from one point to another
   *        represented by their integer coordinates
   * @param a_x x coordinate of point a
   * @param b_x x coordinate of point b
   * @param a_y y coordiante of point a
   * @param b_y y coordinate of point b
   * @return Squared euclidean distance in between
   */
  float CalcSquaredDist(const float& a_x, const float& b_x, const float& a_y,
                        const float& b_y) {
    return ((a_x - b_x) * (a_x - b_x) + (a_y - b_y) * (a_y - b_y));
  }
  /*!
   * @brief Calculate a type dependent distance between given 2d coordinates
   *        represented by their integer coordinates
   * @param a_x x coordinate of point a
   * @param b_x x coordinate of point b
   * @param a_y y coordiante of point a
   * @param b_y y coordinate of point b
   * @param type  distance type (actually only EUCLIDEAN and MANHATTAN is
   * supported)
   * @return distance in between
   */
  float CalcDist(const float& a_x, const float& b_x, const float& a_y,
                 const float& b_y, DISTANCETYPE type) {
    switch (type) {
      case EUCLIDEAN:
        return (sqrtf(CalcSquaredDist(a_x, b_x, a_y, b_y)));
      case MANHATTAN:
        return (abs(a_x - b_x) + abs(a_y - b_y));
      default:
        return 0.0f;  // not supported;
    }
  }

  template <class T>
  float CalcDist(const T& p1, const T& p2,
                 DISTANCETYPE type = DISTANCETYPE::EUCLIDEAN) {
    return CalcDist((float)p1.x, (float)p2.x, (float)p1.y, (float)p2.y, type);
  }

 public:
  std::vector<point_2d> inter_points;  // interrogation points
  int SetCurveParameter(const int order, const int dim = 2);
  int SetCtrlPoints(const std::vector<point_2d> ctrl_points);
  int SetInterrogationPoints(const std::vector<point_2d>& points);
  int SetInterrogationPoints() { return SetInterrogationPoints(inter_points); }
  int CreateCurve();
  int CreateCurveByInterpolation(const bool no_curvature_at_start = true,
                                 const bool no_curvature_at_end   = true);
  int CreateCurveByConstraintedInterpolation(double angle_start,
                                             double angle_end);
  int CreateCurveByOffset(SISLCurve* base, point_2d& dir, float offset);
  int CreateCurveByPathSegments(path_data* data);
  int CreateCurveByPathSegments(polar_spline* spline, int num_splines);

  int CreateCurveByApproximation();

  /**
   * @brief CreateCurveByBlendingCurves
   * To compute a blending curve between two curves. Two points indicate
   * between which ends the blend is to be produced. The blending curve is
   * either a circle or an approximated conic section if this is possible,
   * other- wise it is a quadratic polynomial spline curve. The output is
   * represented as a B-spline curve.
   * @param first   pointer to an existing SislCurve
   * @param second  pointer to an existing SislCurve
   * @param point_on_first point on an first curve
   * @param point_on_second point on the second curve
   * @param result  pointer to the resulting SislCurve
   * @return 0 for success, > 0 warning, < 0 error codes
   */
  int CreateCurveByBlendingCurves(SISLCurve* first, SISLCurve* second,
                                  point_2d point_on_first,
                                  point_2d point_on_second, SISLCurve** result);

  int CreateCurveByBlendingCurves(SISLCurve* first, SISLCurve* second,
                                  point_2d point_on_first,
                                  point_2d point_on_second) {
    return CreateCurveByBlendingCurves(first, second, point_on_first,
                                       point_on_second, &curve_);
  }

  /**
   * @brief CreateConnectedCurveByBlendingCurves
   * To compute a blending curve between two curves. Two points indicate
   * between which ends the blend is to be produced. The blending curve is
   * either a circle or an approximated conic section if this is possible,
   * other- wise it is a quadratic polynomial spline curve. The full curve
   * starts at the start_point from the first curve and ends at the end_point at
   * the last curve. The blend is create between the start_blended_crv and
   * end_blended_crv. Be aware that the start_blended_crv and end_blended_crv
   * are not the same as the start_point and end_point. The output is
   * represented as a B-spline curve.
   * @param first curve where the construction starts
   * @param second curve where the construction ends
   * @param start_point start point of the full curve
   * @param start_blended_crv start point of the blend
   * @param end_point end point of the full curve
   * @param end_blended_crv end point of the blend
   * @return success status
   */
  int CreateConnectedCurveByBlendingCurves(SISLCurve* first, SISLCurve* second,
                                           point_2d start_point,
                                           point_2d start_blended_crv,
                                           point_2d end_point,
                                           point_2d end_blended_crv);

  /**
   * @brief CreateCurveByCutAndBranching
   * To compute  a curve by cutting two curves at a given point and branching
   * them. The output is represented as a B-spline curve. The branching is done
   * by represending the given point as parameter value for the first curve and
   * the second curve. The first curve is then splitted at the given point and
   * the second curve is splitted at the given point. The resulting curve is a
   * connection from the first part of the first curve and second part of the
   * second curve. Be aware that both curves should have a intersection point at
   * the given point.
   * @warning The function is not able to handle the case where the two curves
   * are not intersecting.
   * @param first curve
   * @param second curve
   * @param point_to_cut
   * @param take_full_second_curve if true the full second curve is taken,
   *                                otherwise the second curve is cut at the
   * given point
   * @return
   */
  int CreateCurveByCutAndBranching(SISLCurve* first, SISLCurve* second,
                                   point_2d point_to_cut);

  bool IsCreated();

  int PlotCurve(std::string filename, float min_par, float max_par,
                int samples = 100);

  int PlotCtrlPoints(std::string filename);

  int PlotCurvature(std::string filename, float min_par, float max_par,
                    int samples);
  /**
   * @brief GetPosition
   * To compute the position based on the first derivatives of the curve at a
   * given par_val Evaluation from the left hand side.
   * @param par_val
   * @param pos calculated position if return = 0
   * @param neg_x if true, the evaluation is based on a negative axis
   * orientation (default -> anti fork direction)
   * @return status messages > 0 : warning; = 0 : ok; < 0 : error
   */
  int GetPosition(const float par_val, position_2d& pos,
                  const bool neg_x = true);

  /**
   * @brief GetCurvature
   * Evaluate the signed curvature of a curve at given parameter value.
   * @param par_val parameter value
   * @param curvature return value of the calculated curvature
   * @return
   */
  int GetCurvature(float par_val, double& curvature);

  double     GetMaxParameterVal();
  double     GetMinParameterVal();

  /**
   * @brief GetPoseRelSplineParVal
   * To compute the parameter value of the curve at a given position.
   * @param pos - the position to be evaluated
   * @param par_val - the parameter value of the curve at the given position
   * @return success status
   */
  int        GetPoseRelSplineParVal(position_2d& pos, double& par_val);

  /**
   * @brief GetPartialCordLength
   * To compute the length of the curve by sampling the curve
   * at a given sampling rate : the bigger the sample rate the more accurate
   * @param min_par
   * @param max_par
   * @param samples number of samples
   * @return the computed length
   */
  double     GetPartialCordLength(float min_par, float max_par, int samples);

  /**
   * @brief GetFullCordLength
   * To compute the length of the curve
   * @return length of the curve
   */
  double     GetFullCordLength();

  SISLCurve* GetSislCurve();

  int        SetSislCurve(SISLCurve* curve);
  void        Reset();

  std::string GetErrorString() {
    if (err_num_ == 0) {
      return "No error";
    } else if (err_num_ > 0) {
      return "Warning";
    } else if (err_map.find(err_num_) == err_map.end()) {
      return "Unknown error";
    }

    return err_map[err_num_];
  }

  SislNurbsGen();
  ~SislNurbsGen();
};

#endif  // SISLNURBSGEN_H
