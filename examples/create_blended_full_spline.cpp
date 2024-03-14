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
  // Create a full blended spline
  std::cout << "Create a  full blended spline" << std::endl;

  SislNurbsGen          inter;
  SislNurbsGen          off;
  SislNurbsGen          blended;
  double                max_par;
  double                min_par;

  std::vector<point_2d> points{
      {0, 0}, {0, 1000}, {0, 6000}, {5000, 6000}, {6000, 6000}};

  inter.SetInterrogationPoints(points);
  inter.SetCurveParameter(5);
  inter.CreateCurveByInterpolation();
  max_par = inter.GetMaxParameterVal();
  min_par = inter.GetMinParameterVal();
  inter.PlotCurve("intercurve_points.txt", min_par, max_par, 50);

  off.SetCurveParameter(4);
  point_2d dir{0, -1};
  off.CreateCurveByOffset(inter.GetSislCurve(), dir, -800);
  max_par = off.GetMaxParameterVal();
  min_par = off.GetMinParameterVal();
  off.PlotCurve("offcurve_points.txt", min_par, max_par, 50);

  // point_2d blend_start{50, 940};
  // point_2d blended_end{846, 5620};

  double param_range = inter.GetMaxParameterVal() - inter.GetMinParameterVal();
  position_2d pos{};
  inter.GetPosition(inter.GetMinParameterVal() + 0.4 * param_range, pos);
  point_2d start{pos.x, pos.y};
  inter.GetPosition(inter.GetMinParameterVal() + 0.5 * param_range, pos);
  point_2d blend_start{pos.x, pos.y};

  param_range = off.GetMaxParameterVal() - off.GetMinParameterVal();
  off.GetPosition(off.GetMinParameterVal() + 0.8 * param_range, pos);
  point_2d blended_end{pos.x, pos.y};

  off.GetPosition(off.GetMaxParameterVal(), pos);
  point_2d end{pos.x, pos.y};

  blended.SetCurveParameter(4);
  blended.CreateConnectedCurveByBlendingCurves(inter.GetSislCurve(),
                                               off.GetSislCurve(), start,
                                               blend_start, end, blended_end);
  max_par = blended.GetMaxParameterVal();
  min_par = blended.GetMinParameterVal();
  blended.PlotCurve("blendedfullcurve_points.txt", min_par, max_par, 50);

  SislNurbsGen cutnbranch;

  cutnbranch.SetCurveParameter(4);
  cutnbranch.CreateCurveByCutAndBranching(inter.GetSislCurve(),
                                          blended.GetSislCurve(), blend_start);
  max_par = cutnbranch.GetMaxParameterVal();
  min_par = cutnbranch.GetMinParameterVal();
  cutnbranch.PlotCurve("cutnbranch_points.txt", min_par, max_par, 50);

  return 0;
}
