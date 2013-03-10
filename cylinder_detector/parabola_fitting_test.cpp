#include <opencv2/opencv.hpp>
#include <stdlib.h>

#include "parabola_fitting.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  vector<Point2d> points;
    
  double testp = 0.1;
  double testpy0 = 250;
  
  for(int i = 0; i < 30; i++)
  {
    double x = rand() % 250;
    double y = testp * x * x + testpy0;
    
    cout << "point " << i << ": " << Point2d(x, y) << endl;
    
    points.push_back(Point2d(x, y));
  }
  
  TLine centralLine = TLine(-1, 1, 50);
  
  CParabolaFitting* fitting = new CParabolaFitting(centralLine);
  
  vector<Point2d> transfomPoints;
  transfomPoints.clear();
  fitting->transformPointsBack(points, transfomPoints);

  
  TParabola parabola;
  fitting->fitParabola(transfomPoints, parabola);
  
  Mat img(500, 500, CV_8UC3);
  img.setTo(0);
  
  fitting->drawParabola(img, parabola, Scalar(255, 255, 0));

  drawLine(img, centralLine, Scalar(0, 255, 0));
  
  for(int i = 0; i < (int)points.size(); i++)
  {
    drawPoint(img, points.at(i), Scalar(0, 255, 0));
    drawPoint(img, transfomPoints.at(i), Scalar(0, 255, 255));
  }

  
  imshow("Output: parabola", img);
  
  waitKey();
  
  delete fitting;
  
  return 0;
}
