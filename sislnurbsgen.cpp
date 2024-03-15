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

#include "sislnurbsgen.h"

bool SislNurbsGen::CreateKnotVector(int n, int p) {
  knots_.clear();
  int m = n + p + 1;
  for (int i = 0; i < (n + p + 1); i++) {
    if (i < p + 1) {
      knots_.push_back(0.0f);
    } else if (i >= (p + 1) && i < m - (p + 1)) {
      float step = 1.0f / (m + 1 - (2 * (p + 1)));
      float k    = knots_[knots_.size() - 1] + step;
      knots_.push_back(k);
    } else {
      knots_.push_back(1.0f);
    }
  }
  knots_created = true;
  return true;
}

int SislNurbsGen::SetCurveParameter(const int order, const int dim) {
  param_.order = order;
  param_.dim   = dim;
  params_set   = false;
  return 0;
}

int SislNurbsGen::SetCtrlPoints(const std::vector<point_2d> ctrl_points) {
  SislPoint p{};
  ctrl_.clear();

  for (point_2d pos : ctrl_points) {
    p.x = pos.x;
    p.y = pos.y;
    ctrl_.push_back(p);
  }
  return 0;
}

int SislNurbsGen::SetInterrogationPoints(const std::vector<point_2d>& points) {
  SislPoint p{};
  points_.clear();

  for (point_2d pos : points) {
    p.x = pos.x;
    p.y = pos.y;
    points_.push_back(p);
    points_type_.push_back(1);
  }
  return 0;
}

int SislNurbsGen::CreateCurve() {
  CreateKnotVector(ctrl_.size(), param_.order - 1);

  if (curve_) {
    freeCurve(curve_);
  }

  if (knots_.size()) {
    curve_ = newCurve(ctrl_.size(),   // number of control points
                      param_.order,   // order of spline curve (degree (p) + 1)
                      knots_.data(),  // pointer to knot vector
                                      // (parametrization)
                      reinterpret_cast<double*>(ctrl_.data()),  // pointer to
                                                                // coefficient
                                                                // vector
                                                                // (control
                                                                // points)
                      1,   // kind = polynomial B-spline curve
                      2,   // dimension
                      0);  // no copying of information, 'borrow' arrays
  }
  if (!curve_) {
    return -ENODATA;
  }

  is_created_ = true;
  return 0;
}

int SislNurbsGen::CreateCurveByConstraintedInterpolation(double angle_start,
                                                         double angle_end) {
  double    cendpar;
  double*   gpar = 0;
  int       jnbpar;
  int       jstat = 0;

  // add constraints at the start and end of the curve
  SislPoint p{};
  p = GetAngleBasedDerivative(angle_start);
  points_.insert(points_.begin(), p);
  points_type_.insert(points_type_.begin(), 3);
  p = GetAngleBasedDerivative(angle_end);
  points_.push_back(p);
  points_type_.push_back(4);

  if (curve_) {
    freeCurve(curve_);
  }
  s1356(reinterpret_cast<double*>(points_.data()),  // pointer to where the
                                                    // point coordinates are
                                                    // stored
        points_.size(),       // number of points to be interpolated
        param_.dim,           // the dimension
        points_type_.data(),  // what type of information is stored at a
                              // particular point
        0,                    // no additional condition at start point
        0,                    // no additional condition at end point
        1,                    // open curve
        param_.order,         // order of the spline curve to be produced
        0.0,                  // parameter value to be used at start of curve
        &cendpar,  // parameter value at the end of the curve (to be determined)
        &curve_,   // the resulting spline curve (to be determined)
        &gpar,     // pointer to the parameter values of the points in the curve
                   // (to be determined)
        &jnbpar,   // number of unique parameter values (to be determined)
        &jstat);   // status message

  is_created_ = true;

  return jstat;
}
int SislNurbsGen::CreateCurveByOffset(SISLCurve* base, point_2d& dir,
                                      float offset) {
  double    epsge = 1.0e-5;  // geometric tolerance
  int       stat{};
  SislPoint d{static_cast<double>(dir.x), static_cast<double>(dir.y)};

  if (curve_) {
    freeCurve(curve_);
  }
  s1360(base,                           // the 'old' curve
        offset,                         // the offset value
        epsge,                          // geometric tolerance
        reinterpret_cast<double*>(&d),  // offset direction
        0,  // max step length.  0 indicate the longest box side of 's1'
        param_.dim,  // the dimension
        &curve_,     // the resulting offset curve
        &stat);      // status variable
  is_created_ = true;

  return stat;
}

int SislNurbsGen::CreateCurveByInterpolation(const bool no_curvature_at_start,
                                             const bool no_curvature_at_end) {
  double  cendpar;
  double* gpar = 0;
  int     jnbpar;
  int     jstat = 0;

  if (curve_) {
    freeCurve(curve_);
  }
  s1356(reinterpret_cast<double*>(points_.data()),  // pointer to where the
                                                    // point coordinates are
                                                    // stored
        points_.size(),       // number of points to be interpolated
        param_.dim,           // the dimension
        points_type_.data(),  // what type of information is stored at a
                              // particular point
        static_cast<int>(no_curvature_at_start),  // no additional condition at
                                                  // start point
        static_cast<int>(no_curvature_at_end),    // no additional condition at
                                                  // end point
        1,                                        // open curve
        param_.order,  // order of the spline curve to be produced
        0.0,           // parameter value to be used at start of curve
        &cendpar,  // parameter value at the end of the curve (to be determined)
        &curve_,   // the resulting spline curve (to be determined)
        &gpar,     // pointer to the parameter values of the points in the curve
                   // (to be determined)
        &jnbpar,   // number of unique parameter values (to be determined)
        &jstat);   // status message

  is_created_ = true;

  return jstat;
}
int SislNurbsGen::PlotCurve(std::string filename, float min_par, float max_par,
                            int samples) {
  std::ofstream out(filename, std::ios::trunc | std::ios::out);
  position_2d   pos{};
  if (!samples) {
    return -EOPNOTSUPP;
  }
  float inc = (max_par - min_par) / static_cast<int>(samples);

  if (out.is_open()) {
    for (float f = min_par; f < max_par; f += inc) {
      GetPosition(f, pos);
      out << pos.x << "\t" << pos.y << "\t" << pos.rho << "\t" << std::endl;
    }
    out.flush();
    out.close();
  } else {
    return -ENOEXEC;
  }

  return 0;
}

int SislNurbsGen::CreateCurveByBlendingCurves(SISLCurve* first,
                                              SISLCurve* second,
                                              point_2d   point_on_first,
                                              point_2d   point_on_second) {
  SislPoint p_on_first{static_cast<double>(point_on_first.x),
                       static_cast<double>(point_on_first.y)};

  SislPoint p_on_second{static_cast<double>(point_on_second.x),
                        static_cast<double>(point_on_second.y)};

  double    epsge     = 1.0e-6;  // geometric precision
  int       blendtype = 0;       // polynomial
  int       stat{};

  // find var_par on first curve
  double    var_par_first{};
  double    dist_first{};
  s1957(first,
        reinterpret_cast<double*>(&p_on_first),  // endpoint of curve 1
                                                 // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_first, &dist_first, &stat);

  if (stat != 0) {
    return stat;
  }

  // find var_par on first curve
  double var_par_second{};
  double dist_second{};
  s1957(second,
        reinterpret_cast<double*>(&p_on_second),  // endpoint of curve 1
                                                  // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_second, &dist_second, &stat);

  if (stat != 0) {
    return stat;
  }

  // Subdivide given curves at parameters
  SISLCurve* first_loc_1;
  SISLCurve* first_loc_2;
  SISLCurve* second_loc_1;
  SISLCurve* second_loc_2;

  s1710(first, var_par_first, &first_loc_1, &first_loc_2, &stat);

  if (first_loc_2) {
    freeCurve(first_loc_2);
  }
  s1710(second, var_par_second, &second_loc_1, &second_loc_2, &stat);

  if (second_loc_1) {
    freeCurve(second_loc_1);
  }

  if (curve_) {
    freeCurve(curve_);
  }
  s1606(first_loc_1,                              // the first input curve
        second_loc_2,                             // the second input curve
        epsge,                                    // geometric tolerance
        reinterpret_cast<double*>(&p_on_first),   // endpoint of curve 1
                                                  // (geometric)
        reinterpret_cast<double*>(&p_on_second),  // endpoint of curve 2
                                                  // (geometric)
        blendtype,     // type of blend curve (circle, conic, polynomial)
        param_.dim,    // dimension
        param_.order,  // order of generated spline curve
        &curve_,       // the generated curve
        &stat);        // status message

  is_created_ = true;
  freeCurve(first_loc_1);
  freeCurve(second_loc_2);
  return stat;
}

int SislNurbsGen::CreateConnectedCurveByBlendingCurves(
    SISLCurve* first, SISLCurve* second, point_2d start_point,
    point_2d start_blended_crv, point_2d end_point, point_2d end_blended_crv) {
  is_created_ = false;
  int       stat{};
  SislPoint sp_on_first{static_cast<double>(start_point.x),
                        static_cast<double>(start_point.y)};

  SislPoint ep_on_second{static_cast<double>(end_point.x),
                         static_cast<double>(end_point.y)};

  SislPoint sp_on_blended{static_cast<double>(start_blended_crv.x),
                          static_cast<double>(start_blended_crv.y)};

  SislPoint ep_on_blended{static_cast<double>(end_blended_crv.x),
                          static_cast<double>(end_blended_crv.y)};

  // create blend curve
  stat = CreateCurveByBlendingCurves(first, second, start_blended_crv,
                                     end_blended_crv);

  if (stat < 0) {
    return stat;
  }

  double epsge = 1.0e-6;  // geometric precision

  // find var_par on first curve
  /////////////////////////////
  double var_par_first_start{};
  double var_par_first_end{};
  double dist_first{};
  s1957(first,
        reinterpret_cast<double*>(&sp_on_first),  // startpoint of curve 1
                                                  // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_first_start, &dist_first, &stat);

  if (stat < 0) {
    return stat;
  }
  s1957(first,
        reinterpret_cast<double*>(&sp_on_blended),  // endpoint of curve 1
                                                    // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_first_end, &dist_first, &stat);

  if (stat < 0) {
    return stat;
  }

  // find var_par on second curve
  /////////////////////////////
  double var_par_second_start{};
  double var_par_second_end{};
  double dist_second{};
  s1957(first,
        reinterpret_cast<double*>(&ep_on_blended),  // startpoint of curve 2
                                                    // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_second_start, &dist_second, &stat);

  if (stat < 0) {
    return stat;
  }
  s1957(first,
        reinterpret_cast<double*>(&ep_on_second),  // endpoint of curve 2
                                                   // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_second_end, &dist_second, &stat);

  if (stat < 0) {
    return stat;
  }

  // Subdivide given curves at parameters
  SISLCurve* first_subdiv;
  SISLCurve* second_subdiv;

  s1712(first, var_par_first_start, var_par_first_end, &first_subdiv, &stat);

  if (stat < 0) {
    return stat;
  }

  s1712(second, var_par_second_start, var_par_second_end, &second_subdiv,
        &stat);

  if (stat < 0) {
    return stat;
  }

  // join first curve with blended curve
  SISLCurve* first_blended;
  s1715(first_subdiv, curve_,
        1,               // join first curve at the end
        0,               // join second curve at the start
        &first_blended,  // result curve
        &stat);

  SISLCurve* full_crv;
  s1715(first_blended, second_subdiv,
        1,          // join first curve at the end
        0,          // join second curve at the start
        &full_crv,  // result curve
        &stat);

  if (curve_) {
    freeCurve(curve_);
  }

  curve_ = copyCurve(full_crv);

  // free created temporal curves
  freeCurve(full_crv);
  freeCurve(first_subdiv);
  freeCurve(second_subdiv);
  freeCurve(first_blended);

  is_created_ = true;

  return stat;
}

int SislNurbsGen::CreateCurveByCutAndBranching(SISLCurve* first,
                                               SISLCurve* second,
                                               point_2d   point_to_cut) {
  int       stat{};
  SislPoint sp_to_cut{static_cast<double>(point_to_cut.x),
                      static_cast<double>(point_to_cut.y)};

  double    epsge = 1.0e-6;  // geometric precision

  // find var_par on first curve
  /////////////////////////////
  double    var_par_first_cut{};
  double    dist_first{};
  s1957(first,
        reinterpret_cast<double*>(&sp_to_cut),  // startpoint of curve 1
                                                // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_first_cut, &dist_first, &stat);

  if (stat < 0) {
    return stat;
  }

  // Subdivide a curve at a given parameter value.
  // Subdivide given curves at parameters
  SISLCurve* first_split_1{};
  SISLCurve* first_split_2{};

  s1710(first, var_par_first_cut, &first_split_1, &first_split_2, &stat);
  if (stat < 0) {
    return stat;
  }

  // release the second part of the first curve
  freeCurve(first_split_2);

  // find var_par on second curve
  double var_par_second_cut{};
  s1957(second,
        reinterpret_cast<double*>(&sp_to_cut),  // startpoint of curve 1
                                                // (geometric)
        param_.dim, 1.0e-9, epsge, &var_par_second_cut, &dist_first, &stat);

  if (stat < 0) {
    return stat;
  }

  // Subdivide a curve at a given parameter value.
  // Subdivide given curves at parameters
  SISLCurve* second_split_1{};
  SISLCurve* second_split_2{};
  s1710(second, var_par_second_cut, &second_split_1, &second_split_2, &stat);
  if (stat < 0) {
    return stat;
  }
  // release the first part of the second curve
  freeCurve(second_split_1);

  // Join the first part of the first curve with
  // the second part of the second curve
  if (curve_) {
    freeCurve(curve_);
  }
  is_created_ = false;

  s1715(first_split_1, second_split_2,
        1,  // take the end of the first curve
        0,  // take the start of the second curve
        &curve_, &stat);

  if (stat < 0) {
    return stat;
  }

  is_created_ = true;
  // this is the end

  freeCurve(first_split_1);
  freeCurve(second_split_2);
  return stat;
}

int SislNurbsGen::CreateCurveByPathSegments(path_data* data) {
  points_.clear();
  points_type_.clear();

  for (int i = 0; i < data->splineNum; i++) {
    points_.push_back({static_cast<double>(data->spline[i].startPos.x),
                       static_cast<double>(data->spline[i].startPos.y)});
    points_type_.push_back(1);
  }
  // take care of the last point
  points_.push_back(
      {static_cast<double>(data->spline[data->splineNum - 1].endPos.x),
       static_cast<double>(data->spline[data->splineNum - 1].endPos.y)});
  points_type_.push_back(1);

  return CreateCurveByInterpolation();
}

int SislNurbsGen::PlotCtrlPoints(std::string filename) {
  std::ofstream out(filename, std::ios::trunc | std::ios::out);

  if (out.is_open()) {
    for (SislPoint cp : ctrl_) {
      out << cp.x << "\t" << cp.y << std::endl;
    }
    out.flush();
    out.close();
  } else {
    return -ENOEXEC;
  }

  return 0;
}

int SislNurbsGen::PlotCurvature(std::string filename, float min_par,
                                float max_par, int samples) {
  std::ofstream out(filename, std::ios::trunc | std::ios::out);
  double        curvature{};
  if (!samples) {
    return -EOPNOTSUPP;
  }
  float inc = (max_par - min_par) / static_cast<int>(samples);
  int   i   = 0;

  if (out.is_open()) {
    for (float f = min_par; f < max_par; f += inc) {
      GetCurvature(f, curvature);
      out << i << "\t" << curvature << std::endl;
      i++;
    }
    out.flush();
    out.close();
  } else {
    return -ENOEXEC;
  }

  return 0;
}

int SislNurbsGen::GetPosition(const float par_val, position_2d& pos,
                              const bool neg_x) {
  int    leftknot{0};
  double derive[param_.dim * 2]{};  // 2x to hold the derivatives 1.order
  int    stat{};
  double add_pi = neg_x ? M_PI : 0;

  pos           = {};
  if (!is_created_) {
    return EOPNOTSUPP;
  }

  s1227(curve_,
        1,  // 0 for position, 1 ..x add derivatives
        par_val, &leftknot, derive, &stat);
  if (!stat) {
    pos.x       = derive[kX];
    pos.y       = derive[kY];
    float angle = AngleTool::normaliseAngleSym0(
        atan2(derive[kY + param_.dim], derive[kX + param_.dim]) + add_pi);
    pos.rho = angle;
  }
  return stat;
}

int SislNurbsGen::GetCurvature(float par_val, double& curvature) {
  int    stat{};
  int    leftknot{0};
  double derive[param_.dim * 3]{};  // 2x to hold the derivatives 1.order
  if (!is_created_) {
    return EOPNOTSUPP;
  }

  s1227(curve_,
        2,  // 0 for position, 1 ..x add derivatives
        par_val, &leftknot, derive, &stat);
  if (!stat) {
    SislPoint first{};
    SislPoint second{};
    first.x    = derive[kX + 2];
    first.y    = derive[kY + 2];
    second.x   = derive[kX + 4];
    second.y   = derive[kY + 4];

    float norm = sqrt(first.x * first.x + first.y * first.y);

    curvature  = std::numeric_limits<double>::max();
    if (!isEqual(norm, 0.0f)) {
      curvature =
          (first.x * second.y - first.y * second.x) / (norm * norm * norm);
    }
  }

  return stat;
}

double SislNurbsGen::GetMaxParameterVal() {
  double max_par_val{};
  max_par_val = curve_->et[curve_->in + curve_->ik - 1];
  return max_par_val;
}
double SislNurbsGen::GetMinParameterVal() {
  double min_par_val{};
  min_par_val = curve_->et[0];
  return min_par_val;
}

double SislNurbsGen::GetPartialCordLength(float min_par, float max_par,
                                          int samples) {
  float       length{};
  double      inc = (max_par - min_par) / samples;
  position_2d pos1{};
  position_2d pos2{};

  GetPosition(min_par, pos1);
  for (float f = min_par + inc; f < max_par; f += inc) {
    GetPosition(f + inc, pos2);
    length += CalcDist(pos1, pos2);
    pos1 = pos2;
  }

  return double(length);
}

double SislNurbsGen::GetFullCordLength() {
  double epsge = 0.001;
  double length;
  int    stat = 0;

  if (curve_) {
    s1240(curve_, epsge, &length, &stat);
  }
  return length;
}

SISLCurve* SislNurbsGen::GetSislCurve() { return curve_; }

SislNurbsGen::SislNurbsGen() {
  ctrl_.reserve(PATH_SPLINE_MAX);
  points_.reserve(PATH_SPLINE_MAX);
  points_type_.reserve(PATH_SPLINE_MAX);
}
SislNurbsGen::~SislNurbsGen() {
  if (curve_) {
    freeCurve(curve_);
  }
}
