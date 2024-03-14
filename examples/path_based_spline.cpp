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

#include "navigation/path_proxy.h"
#include "sislnurbsgen.h"

typedef struct {
  path_data    data{};
  polar_spline spline[PATH_SPLINE_MAX]{};
} __attribute__((packed)) path_data_msg;

path_data_msg data;
void          create_data() {
  polar_spline spl{};
  spl.startPos        = {37349, 43525, 90.00 * DEG2RAD};
  data.spline[0]      = spl;
  spl.startPos        = {37350, 43522, 90.34 * DEG2RAD};
  data.spline[1]      = spl;
  spl.startPos        = {37361, 41521, 90.34 * DEG2RAD};
  data.spline[2]      = spl;
  spl.startPos        = {37362, 41461, 90.00 * DEG2RAD};
  data.spline[3]      = spl;
  spl.startPos        = {37362, 41303, 90.00 * DEG2RAD};
  data.spline[4]      = spl;
  spl.startPos        = {37362, 41302, 90.00 * DEG2RAD};
  data.spline[5]      = spl;
  spl.startPos        = {37362, 35303, 90.00 * DEG2RAD};
  data.spline[6]      = spl;
  spl.startPos        = {37362, 35302, 90.00 * DEG2RAD};
  data.spline[7]      = spl;
  spl.startPos        = {37362, 26957, 90.00 * DEG2RAD};
  data.spline[8]      = spl;
  spl.startPos        = {36953, 26546, 0.00 * DEG2RAD};
  data.spline[9]      = spl;

  data.data.splineNum = 10;
  for (int i = 0; i < data.data.splineNum - 1; i++) {
    data.spline[i].endPos = data.spline[i + 1].startPos;
  }
  data.spline[data.data.splineNum - 1].endPos = data.spline[0].startPos;
}

int main() {
  SislNurbsGen gen{};
  float        min_par{};
  float        max_par{};

  create_data();
  gen.SetCurveParameter(4);
  gen.CreateCurveByPathSegments(reinterpret_cast<path_data*>(&data));
  min_par = gen.GetMinParameterVal();
  max_par = gen.GetMaxParameterVal();
  gen.PlotCurve("path_points.txt", min_par, max_par, 150);
  gen.PlotCurvature("path_curvature.txt", min_par, max_par, 150);

  position_2d start{}, end{};
  gen.GetPosition(min_par, start);
  gen.GetPosition(max_par, end);

  std::cout << "angle at start:" << start.rho * RAD2DEG
            << " angle at end:" << end.rho * RAD2DEG << std::endl;
}
