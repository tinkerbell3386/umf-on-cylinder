#include "parabola_fitting.h"

using namespace std;
using namespace cv;

CParabolaFitting::CParabolaFitting(TLine centralLine)
{
  getAngleAndOrigin(TLine(centralLine.a, -centralLine.b, centralLine.c));
  setupTrasfomationMatrices();
}

bool CParabolaFitting::fitParabola(vector<Point> points, 
                                   TParabola& parabola, Mat draw)
{
  if(points.size() < 2) 
  {
    cerr << "At least two points are needed to fit parabola." << endl;
    return false;
  }
  
  double y0;
  double p;
  
  vector<Point2d> pointsTrasformed;
  pointsTrasformed.clear();
  transformPointsToY(points, pointsTrasformed);
  
  /*
  cout << "points.size(): " << points.size() << endl;
  cout << "pointsTrasformed.size(): " << pointsTrasformed.size() << endl;
  
  for(int i = 0; i < (int)pointsTrasformed.size(); i++)
  {
    cout << "pointsTrasformed: " << pointsTrasformed.at(i) << endl;
    drawPoint(draw, pointsTrasformed.at(i), Scalar(0, 0, 255));
  }  
  */
  
  Mat Y(pointsTrasformed.size(), 1, CV_64F);
  Mat Z(pointsTrasformed.size(), 2, CV_64F);
  
  for(int i = 0; i < (int)pointsTrasformed.size(); i++)
  {
    Y.at<double>(i, 0) = pointsTrasformed.at(i).y;
    
    Z.at<double>(i , 0) = 1;
    Z.at<double>(i , 1) = pointsTrasformed.at(i).x * pointsTrasformed.at(i).x;
  }
  
  Mat C = (Z.t() * Z).inv() * Z.t() * Y;
  
  y0 = C.at<double>(0, 0);
  p = C.at<double>(1, 0);
  
  parabola = TParabola(Point2d(0, y0), p, angle, (int)pointsTrasformed.size());
  
  return true;
}

void CParabolaFitting::transformPointsToY(vector<Point> input, 
                                          vector<Point2d>& output)
{
  Mat pointsMatrix(3, input.size(), CV_64F);
  
  for(int i = 0; i < (int)input.size(); i++)
  {
    pointsMatrix.at<double>(0, i) = input.at(i).x;
    pointsMatrix.at<double>(1, i) = input.at(i).y;
    pointsMatrix.at<double>(2, i) = 1;
  }
  
  Mat resultMatrix = transformationMatrix * pointsMatrix;
  
  cout << "pointsMatrix: " << pointsMatrix.rows << "x" << pointsMatrix.cols << endl;
  cout << "transformationMatrix: " << transformationMatrix.rows << "x" << transformationMatrix.cols << endl;
  cout << "resultMatrix: " << resultMatrix.rows << "x" << resultMatrix.cols << endl;
  
  for(int i = 0; i < (int)resultMatrix.cols; i++)
  {
    output.push_back(Point2d(resultMatrix.at<double>(0, i), 
                             resultMatrix.at<double>(1, i)));
  }
}

Point2d CParabolaFitting::transformPointBack(Point input)
{
  Mat pointsMatrix(3, 1, CV_64F);
  pointsMatrix.at<double>(0, 0) = input.x;
  pointsMatrix.at<double>(1, 0) = input.y;
  pointsMatrix.at<double>(2, 0) = 1;
  
  Mat resultMatrix = transformationMatrixInverse * pointsMatrix;
  

  return Point2d(resultMatrix.at<double>(0, 0), resultMatrix.at<double>(1, 0));
}

void CParabolaFitting::transformPointsBack(vector<Point> input, 
                                          vector<Point>& output)
{
  Mat pointsMatrix(3, input.size(), CV_64F);
  
  for(int i = 0; i < (int)input.size(); i++)
  {
    pointsMatrix.at<double>(0, i) = input.at(i).x;
    pointsMatrix.at<double>(1, i) = input.at(i).y;
    pointsMatrix.at<double>(2, i) = 1;
  }
  
  //cout << "pointsMatrix" << endl << pointsMatrix << endl;
  //cout << "transformationMatrixInverse" << endl << transformationMatrixInverse << endl;
  
  Mat resultMatrix = transformationMatrixInverse * pointsMatrix;
  
  //cout << "resultMatrix" << endl  << resultMatrix << endl;
  
  for(int i = 0; i < (int)resultMatrix.cols; i++)
  {
    output.push_back(Point(resultMatrix.at<double>(0, i), 
                             resultMatrix.at<double>(1, i)));
  }
}

void CParabolaFitting::getAngleAndOrigin(TLine line)
{
  if(line.a == 0.0) // pokud jsou osa X a centralni primka jsou rovnobezne
  {
    angle = PI / 2;
    origin = Point2d(0, 0);
  }
  else if (line.b == 0) // pokud jsou osa Y a centralni primka jsou rovnobezne
  {
    angle = 0.0;
    origin = Point2d(0, 0);
  }
  else
  {
    //ax+by+c=0 => y = -ax/b - c/b => smernice: -a/b
    // odecet od PI/2 - prepocet uhlu k X do Y -> viz wiki smernicova rovnice
    angle = PI / 2 - atan(-line.a / line.b);
    
    //ax+bx+c=0 AND y=0 => x = -c/a
    origin = Point2d(-(line.c / line.a), 0);
  }
  cout << "angle: " <<  angle * 180 / PI << endl;
  cout << "origin: " <<  origin << endl;
}

void CParabolaFitting::setupTrasfomationMatrices()
{  
  // translation to the new origin
  Mat T = (Mat_<double>(3,3) << 1, 0, -origin.x, 
                                0, 1, -origin.y, 
                                0, 0, 1);
  
  // back to the origin [0, 0]
  Mat Tinv = (Mat_<double>(3,3) <<  1, 0, origin.x, 
                                    0, 1, origin.y, 
                                    0, 0, 1);
  
  // counter clockwise rotation
  Mat R = (Mat_<double>(3,3) << cos(angle), -sin(angle), 0, 
                                sin(angle), cos(angle),  0, 
                                0,          0,           1);
  // clockwise rotation
  Mat Rinv = (Mat_<double>(3,3) <<  cos(angle),  sin(angle), 0, 
                                    -sin(angle), cos(angle), 0, 
                                    0,           0,          1);
  
  transformationMatrix = R * T;
  transformationMatrixInverse = Tinv * Rinv;
}

void CParabolaFitting::drawParabola(Mat& img, TParabola parabola, Scalar color, 
                                    int thickness)
{
  Point pt1;
  pt1.x = -img.cols;
  pt1.y = parabola.param*pt1.x*pt1.x + parabola.apex.y;
  Point pt2;
  for(int x = parabola.apex.x - img.cols + 1; x < parabola.apex.x + img.cols; x++)
  {
    pt2.x = x;
    pt2.y = parabola.param*x*x + parabola.apex.y; 
    line(img, transformPointBack(pt1), transformPointBack(pt2), color, thickness);
    pt1 = pt2;
  }
}
