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

  inter.SetInterrogationPoints(points);
  inter.SetCurveParameter(5);
  inter.CreateCurveByInterpolation();
  max_par = inter.CalcMaxParameterVal();
  min_par = inter.CalcMinParameterVal();
  inter.PlotCurve("intercurve_points.txt", min_par, max_par, 50);

  inter.PlotCurvature("intercurvature.txt", min_par, max_par, 50);

  off.SetCurveParameter(4);
  point_2d dir{0, -1};
  off.CreateCurveByOffset(inter.GetSislCurve(), dir, -800);
  max_par = inter.CalcMaxParameterVal();
  min_par = inter.CalcMinParameterVal();
  off.PlotCurve("offcurve_points.txt", min_par, max_par, 50);

  off.PlotCurvature("offcurvature.txt", min_par, max_par, 50);

  // point_2d origin{-314,2920};
  point_2d origin{50, 940};
  point_2d offset{846, 5620};
  blended.SetCurveParameter(4);
  blended.CreateCurveByBlendingCurves(inter.GetSislCurve(), off.GetSislCurve(),
                                      origin, offset);
  max_par = blended.CalcMaxParameterVal();
  min_par = blended.CalcMinParameterVal();
  blended.PlotCurve("blendedcurve_points.txt", min_par, max_par, 50);
  blended.PlotCurvature("blendedcurvature.txt", min_par, max_par, 50);


  return 0;
}
