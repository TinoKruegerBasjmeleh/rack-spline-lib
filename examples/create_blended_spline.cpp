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
  // Create a blended spline
  std::cout << "Create a blended spline" << std::endl;

  SislNurbsGen          gen;
  SislNurbsGen          inter;
  SislNurbsGen          off;
  SislNurbsGen          blended;

  std::vector<point_2d> points{
      {0, 0}, {0, 1000}, {0, 6000}, {5000, 6000}, {6000, 6000}};

  gen.SetCtrlPoints(points);
  gen.SetCurveParameter(4);
  gen.CreateCurve();

  float max_par = gen.CalcMaxParameterVal();
  float min_par = gen.CalcMinParameterVal();

  inter.SetInterrogationPoints(points);
  inter.SetCurveParameter(5);
  inter.CreateCurveByInterpolation();
  max_par = inter.CalcMaxParameterVal();
  min_par = inter.CalcMinParameterVal();

  off.SetCurveParameter(4);
  point_2d dir{0, -1};
  off.CreateCurveByOffset(inter.GetSislCurve(), dir, -800);
  max_par = off.CalcMaxParameterVal();
  min_par = off.CalcMinParameterVal();

  point_2d origin{50, 940};
  point_2d offset{846, 5620};
  blended.SetCurveParameter(4);
  blended.CreateCurveByBlendingCurves(inter.GetSislCurve(), off.GetSislCurve(),
                                      origin, offset);
  max_par = blended.CalcMaxParameterVal();
  min_par = blended.CalcMinParameterVal();
  blended.PlotCurve("blendedcurve_points.txt", min_par, max_par, 50);

  return 0;
}
