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

 public:
  int        SetCurveParameter(const int order, const int dim = 2);
  int        SetCtrlPoints(const std::vector<point_2d> ctrl_points);
  int        SetInterrogationPoints(const std::vector<point_2d>& points);
  int        CreateCurve();
  int        CreateCurveByInterpolation(const bool no_curvature_at_start = true,
                                        const bool no_curvature_at_end = true);
  int        CreateCurveByConstraintedInterpolation(double angle_start,
                                                    double angle_end);
  int        CreateCurveByOffset(SISLCurve* base, point_2d& dir, float offset);
  int        CreateCurveByPathSegments(path_data* data);

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
  int        CreateCurveByBlendingCurves(SISLCurve* first, SISLCurve* second,
                                         point_2d point_on_first,
                                         point_2d point_on_second);

  int        PlotCurve(std::string filename, float min_par, float max_par,
                       int samples = 100);

  int        PlotCtrlPoints(std::string filename);

  int        PlotCurvature(std::string filename, float min_par, float max_par,
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
  int        GetPosition(const float par_val, position_2d& pos,
                         const bool neg_x = true);

  /**
   * @brief GetCurvature
   * Evaluate the signed curvature of a curve at given parameter value.
   * @param par_val parameter value
   * @param curvature return value of the calculated curvature
   * @return
   */
  int        GetCurvature(float par_val, double& curvature);

  double     CalcMaxParameterVal();
  double     CalcMinParameterVal();

  /**
   * @brief SampAccurCordLeng
   * To compute the length of the curve by sampling the curve
   * at a given sampling rate : the bigger the sample rate the more accurate
   * @param min_par
   * @param max_par
   * @param samples number of samples
   * @return the computed length
   */
  double     SampAccurCordLeng(float min_par, float max_par, int samples);

  /**
   * @brief CalcCordLength
   * To compute the length of the curve
   * @return length of the curve
   */
  double     CalcCordLength();

  SISLCurve* GetSislCurve();

  SislNurbsGen();
  ~SislNurbsGen();
};

#endif  // SISLNURBSGEN_H
