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

#include <chrono>

#include "sislnurbsgen.h"
using namespace std::chrono;

int main() {
  // Create a simple spline
  std::cout << "Create a simple spline" << std::endl;

  SislNurbsGen          gen;
  SislNurbsGen          inter;

  std::vector<point_2d> new_points;

  std::vector<point_2d> points{{0, 0},        {0, 10000},     {0, 15000},
                               {0, 30000},    {10000, 30000}, {15000, 30000},
                               {30000, 30000}};

  gen.SetCtrlPoints(points);
  gen.SetCurveParameter(4);
  gen.CreateCurve();

  float max_par = gen.GetMaxParameterVal();
  float min_par = gen.GetMinParameterVal();

  std::cout << "Max parameter: " << max_par << "\n" << std::endl;
  std::cout << "Min parameter: " << min_par << "\n" << std::endl;

  size_t      step = 10000;
  float       inc  = (max_par - min_par) / step;
  position_2d pos;

  for (int i = 0; i < step; i++) {
    gen.GetPosition(min_par + i * inc, pos);
    new_points.push_back({pos.x, pos.y});
  }

  gen.PlotCurve("time_curve_points.txt", min_par, max_par, 10000);
  gen.PlotCtrlPoints("time_controlpoints.txt");

  std::cout << "Create a spline by interpolation" << std::endl;
  auto start = high_resolution_clock::now();

  inter.SetInterrogationPoints(new_points);
  inter.SetCurveParameter(5);
  inter.CreateCurveByInterpolation();

  auto stop     = high_resolution_clock::now();
  max_par       = inter.GetMaxParameterVal();
  min_par       = inter.GetMinParameterVal();

  auto duration = duration_cast<milliseconds>(stop - start);
  std::cout << "Time taken by function: " << duration.count() << " milliseconds"
            << std::endl;

  inter.PlotCurve("new_intercurve_points.txt", min_par, max_par, 10000);
  inter.PlotCurvature("new_intercurvature.txt", min_par, max_par, 10000);

  return 0;
}
