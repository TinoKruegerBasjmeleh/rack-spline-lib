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
  // Create an interpolation spline
  std::cout << "Create an interpolation spline" << std::endl;

  SislNurbsGen          gen;
  SislNurbsGen          inter;
  SislNurbsGen          approx;

  std::vector<point_2d> points{
      {0, 0}, {0, 1000}, {0, 6000}, {5000, 6000}, {6000, 6000}};

  gen.SetCtrlPoints(points);
  gen.SetCurveParameter(4);
  gen.CreateCurve();

  float max_par = gen.GetMaxParameterVal();
  float min_par = gen.GetMinParameterVal();

  inter.SetInterrogationPoints(points);
  inter.SetCurveParameter(4);
  inter.CreateCurveByInterpolation();
  max_par = inter.GetMaxParameterVal();
  min_par = inter.GetMinParameterVal();
  inter.PlotCurve("intercurve_points.txt", min_par, max_par, 50);
  inter.PlotCurvature("intercurvature.txt", min_par, max_par, 50);
  gen.PlotCtrlPoints("interctrlpoints.txt");

  approx.SetInterrogationPoints(points);
  approx.SetCurveParameter(4);
  approx.CreateCurveByApproximation();
  max_par = approx.GetMaxParameterVal();
  min_par = approx.GetMinParameterVal();
  approx.PlotCurve("approxcurve_points.txt", min_par, max_par, 50);
  approx.PlotCurvature("approxcurvature.txt", min_par, max_par, 50);

  return 0;
}
