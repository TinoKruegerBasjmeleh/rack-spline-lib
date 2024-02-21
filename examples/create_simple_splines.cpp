#include <iostream>

#include "sislnurbsgen.h"

int main() {
  // Create a simple spline
  std::cout << "Create a simple spline" << std::endl;

  SislNurbsGen          gen;
  SislNurbsGen          inter;
  SislNurbsGen          off;
  SislNurbsGen          blended;
  std::vector<point_2d> points{{
                                   0,
                                   0,
                               },
                               {0, 1000},
                               {0, 6000},
                               {5000, 6000},
                               {6000, 6000}};

  gen.SetCtrlPoints(points);
  gen.SetCurveParameter(4);
  gen.CreateCurve();

  float max_par = gen.CalcMaxParameterVal();
  float min_par = gen.CalcMinParameterVal();

  gen.PlotCurve("curve_points.txt", min_par, max_par, 50);
  gen.PlotCurvature("curvature.txt", min_par, max_par, 50);
  gen.PlotCtrlPoints("controlpoints.txt");

  return 0;
}