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

#include <iostream>

#include "sislnurbsgen.h"

int main() {
  // Create an S shaped spline
  std::cout << "Create an S shaped spline" << std::endl;

  SislNurbsGen          s_spline;
  SislNurbsGen          offset_neg;
  SislNurbsGen          offset_pos;

  std::vector<point_2d> points{{0, 0}, {1500, 6000}, {3000, 50}, {4500, 6000}};

  s_spline.SetCtrlPoints(points);
  s_spline.SetCurveParameter(4);
  s_spline.CreateCurve();

  float max_par = s_spline.GetMaxParameterVal();
  float min_par = s_spline.GetMinParameterVal();

  s_spline.PlotCurve("s_curve_points.txt", min_par, max_par, 50);
  s_spline.PlotCurvature("s_curvature.txt", min_par, max_par, 50);
  s_spline.PlotCtrlPoints("s_controlpoints.txt");

  offset_neg.SetCurveParameter(4);
  point_2d dir{0, -1};
  offset_neg.CreateCurveByOffset(s_spline.GetSislCurve(), dir, -800);
  max_par = s_spline.GetMaxParameterVal();
  min_par = s_spline.GetMinParameterVal();
  offset_neg.PlotCurve("off_neg_curve.txt", min_par, max_par, 50);
  offset_neg.PlotCurvature("off_neg_curvature.txt", min_par, max_par, 50);

  offset_pos.SetCurveParameter(4);
  dir = {0, 1};
  offset_pos.CreateCurveByOffset(s_spline.GetSislCurve(), dir, 800);
  offset_pos.PlotCurve("off_pos_curve.txt", min_par, max_par, 50);
  offset_pos.PlotCurvature("off_pos_curvature.txt", min_par, max_par, 50);

  return 0;
}
