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
  // Create an offset spline
  std::cout << "Create an offset spline" << std::endl;

  SislNurbsGen          gen;
  SislNurbsGen          off;
  SislNurbsGen          inter;

  std::vector<point_2d> points{
      {0, 0}, {0, 1000}, {0, 6000}, {5000, 6000}, {6000, 6000}};

  gen.SetCtrlPoints(points);
  gen.SetCurveParameter(4);
  gen.CreateCurve();

  float max_par = gen.GetMaxParameterVal();
  float min_par = gen.GetMinParameterVal();

  inter.SetInterrogationPoints(points);
  inter.SetCurveParameter(5);
  inter.CreateCurveByInterpolation();
  max_par = inter.GetMaxParameterVal();
  min_par = inter.GetMinParameterVal();
  inter.PlotCurve("intercurve_points.txt", min_par, max_par, 50);

  inter.PlotCurvature("intercurvature.txt", min_par, max_par, 50);
  off.SetCurveParameter(4);
  point_2d dir{0, -1};
  off.CreateCurveByOffset(inter.GetSislCurve(), dir, -800);
  max_par = inter.GetMaxParameterVal();
  min_par = inter.GetMinParameterVal();
  off.PlotCurve("offcurve_points.txt", min_par, max_par, 50);
  off.PlotCurvature("offcurvature.txt", min_par, max_par, 50);
  gen.PlotCtrlPoints("offctrlpoints.txt");

  return 0;
}
