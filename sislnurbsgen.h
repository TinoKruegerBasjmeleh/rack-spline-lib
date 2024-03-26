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

class CurveParameter {
 public:
  int order{4};  /// order of spline curve (degree + 1)
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
  int SetCurveParameter(const int order, const int dim = 2);
  int SetCtrlPoints(const std::vector<point_2d> ctrl_points);
  int SetInterrogationPoints(const std::vector<point_2d>& points);
  int CreateCurve();
  int CreateCurveByInterpolation(const bool no_curvature_at_start = true,
                                 const bool no_curvature_at_end   = true);
  int CreateCurveByConstraintedInterpolation(double angle_start,
                                             double angle_end);
  int CreateCurveByOffset(SISLCurve* base, point_2d& dir, float offset);
  int CreateCurveByPathSegments(path_data* data);
  int CreateCurveByPathSegments(polar_spline* spline, int num_splines);

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
   * @return 0 for success, > 0 warning, < 0 error codes
   */
  int CreateCurveByBlendingCurves(SISLCurve* first, SISLCurve* second,
                                  point_2d point_on_first,
                                  point_2d point_on_second);

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

  SislNurbsGen();
  ~SislNurbsGen();
};

#endif  // SISLNURBSGEN_H
