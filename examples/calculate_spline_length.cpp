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

  float  min_par = spline.CalcMinParameterVal();
  float  max_par = spline.CalcMaxParameterVal();

  double length  = spline.CalcCordLength();
  std::cout << "Spline length is: " << length << " mm" << std::endl;

  spline.PlotCurve("s_curve_points.txt", min_par, max_par, 100);
  spline.PlotCurvature("s_curvature.txt", min_par, max_par, 100);
  spline.PlotCtrlPoints("s_controlpoints.txt");

  return 0;
}
