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

  SislNurbsGen          spline;

  std::vector<point_2d> points{
      {0, 0}, {0, 1000}, {0, 6000}, {5000, 6000}, {6000, 6000}};

  spline.SetCtrlPoints(points);
  spline.SetCurveParameter(4);
  spline.CreateCurve();

  float  min_par = spline.GetMinParameterVal();
  float  max_par = spline.GetMaxParameterVal();

  double length  = spline.GetFullCordLength();
  std::cout << "Spline length is: " << length << " mm" << std::endl;

  spline.PlotCurve("s_curve_points.txt", min_par, max_par, 100);
  spline.PlotCurvature("s_curvature.txt", min_par, max_par, 100);
  spline.PlotCtrlPoints("s_controlpoints.txt");

  return 0;
}
