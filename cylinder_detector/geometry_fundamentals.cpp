#include <iostream>
#include <assert.h>

#include "geometry_fundamentals.h"

using namespace cv;
using namespace std;

TEllipse::TEllipse(RotatedRect _boundingBox, int _score)
{
  boundingBox = _boundingBox;

  if(_boundingBox.size.width > _boundingBox.size.height)
  {
    a = _boundingBox.size.width / 2;
    b = _boundingBox.size.height / 2;
  }
  else
  {
    a = _boundingBox.size.height / 2;
    b = _boundingBox.size.width / 2;
  }

  Point2f vertices[4];
  _boundingBox.points(vertices);


  if( getPointToPointDistanceSquared(vertices[0], vertices[1]) >
      getPointToPointDistanceSquared(vertices[1], vertices[2]))
  {
    mainEdge = Point2d(vertices[1].x + (vertices[2].x - vertices[1].x) / 2,
                       vertices[1].y + (vertices[2].y - vertices[1].y) / 2);

    secondaryEdge = Point2d(vertices[0].x + (vertices[1].x - vertices[0].x) / 2,
                            vertices[0].y + (vertices[1].y - vertices[0].y) / 2);
  }
  else
  {
    secondaryEdge = Point2d(vertices[1].x + (vertices[2].x - vertices[1].x) / 2,
                            vertices[1].y + (vertices[2].y - vertices[1].y) / 2);

    mainEdge = Point2d(vertices[0].x + (vertices[1].x - vertices[0].x) / 2,
                       vertices[0].y + (vertices[1].y - vertices[0].y) / 2);
  }

  center = _boundingBox.center;

  e = std::sqrt(a*a - b*b);

  thetaRadians = (_boundingBox.angle * PI) / 180; // v radianech

  score = _score;
}

TLine::TLine(Vec4f _lineVector, int _score)
{
  lineVector = _lineVector;

  // need a normal vector for equation: ax + by + c = 0
  a = -_lineVector[1];
  b = _lineVector[0];

  c = - a * _lineVector[2] - b * _lineVector[3];

  score = _score;
}

TLine::TLine(double _a, double _b, double _c, int _score)
{
  a = _a;
  b = _b;
  c = _c;

  lineVector = Vec4f(-a, b, c, 0);

  score = _score;
}

TLine::TLine(cv::Point2d pt1, cv::Point2d pt2, int _score)
{
  // againts division by 0
  assert(pt1 != pt2);

  Vec2d vector(pt1.x-pt2.x, pt1.y-pt2.y);
  lineVector = Vec4f(vector[0] / norm(vector),
                     vector[1] / norm(vector),
                     pt1.x,
                     pt1.y
  );

  a = -lineVector[1];
  b = lineVector[0];

  c = - a * lineVector[2] - b * lineVector[3];

  score = _score;
}

TParabola::TParabola(Point2d _apex, double _param, double _angle, int _score)
{
  apex = _apex;
  param = _param;
  angle = _angle;
  score = _score;
}

Point2d getLineIntersection(TLine p, TLine q)
{
  return Point2d( (-p.c*q.b + p.b*q.c)/(p.a*q.b - p.b*q.a),
                  (-p.a*q.c + p.c*q.a)/(p.a*q.b - p.b*q.a));
}

double getPointToPointDistanceSquared(Point2d a, Point2d b)
{
  return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

double getPointToPointDistance(Point2d a, Point2d b)
{
  return std::sqrt(getPointToPointDistanceSquared(a, b));
}

double getDistanceLineToPointSquared(TLine baseLine, Point2d point)
{
  TLine lineFromPoint(Vec4f(baseLine.a, baseLine.b, point.x, point.y));
  Point2d intersection = getLineIntersection(baseLine, lineFromPoint);
  return getPointToPointDistanceSquared(point, intersection);
}

void drawLine(Mat& img, TLine newLine, Scalar color, int thickness)
{

  if(std::abs(newLine.a) > std::abs(newLine.b))
  {
    line(img, 
         Point(-newLine.c / newLine.a, 0),
         Point(-(newLine.b * img.rows + newLine.c) / newLine.a, img.rows),
         color, 
         thickness);
  } 
  else 
  {
    line(img, 
         Point(0, -newLine.c / newLine.b),
         Point(img.cols, -(newLine.a * img.cols + newLine.c) / newLine.b),
         color, 
         thickness);
  }
  
/*
  line(img, 
       Point(newLine.lineVector[2] + 2*img.cols*newLine.lineVector[0], newLine.lineVector[3] + 2*img.rows*newLine.lineVector[1]),
       Point(newLine.lineVector[2] - 2*img.cols*newLine.lineVector[0], newLine.lineVector[3] - 2*img.rows*newLine.lineVector[1]), 
       color, thickness
      );
*/
}

void drawPoint(Mat& img, Point2d point, Scalar color, int size)
{
  line(img, Point(point.x + size, point.y), Point(point.x - size, point.y), color);
  line(img, Point(point.x, point.y + size), Point(point.x, point.y - size), color);
}

/*
void drawParabola(Mat& img, TParabola parabola, Scalar color, int thickness)
{
  Point pt1(0, parabola.apex.y); // apex
  Point pt2;
  for(int x = 1; x < img.cols; x++)
  {
    pt2.x = x;
    pt2.y = parabola.param*x*x + parabola.apex.y; 
    line(img, pt1, pt2, color, thickness);
    pt1 = pt2;
  }
}
*/

Vec2f normalizeVector(Vec2f vector)
{
  return Vec2f(vector[0] / norm(vector), vector[1] / norm(vector));
}

TLine lineNormalization(TLine inputLine)
{
  TLine outputLine;

  double normalization = std::sqrt(inputLine.a * inputLine.a + inputLine.b * inputLine.b);

  if(asin(inputLine.b/normalization) <= 0.f) normalization *= -1.f;

  outputLine.a = inputLine.a / normalization;
  outputLine.b = inputLine.b / normalization;
  outputLine.c = inputLine.c;
  outputLine.score = inputLine.score;

  outputLine.lineVector = Vec4f(-outputLine.a, outputLine.b, inputLine.lineVector[2], inputLine.lineVector[3]);

  return outputLine;
}

double getSmallerIntersectionAngle(TLine line1, TLine line2)
{
  //skalarni soucin 2 jednotkovych vectoru je cosinus uhlu mezni nimi
  double angle = acos(line1.a * line2.a + line1.b * line2.b) * 180.0 / PI;

  if(angle > 270.0)
  {
    return 360.0 - angle;
  }
  else if(angle > 180.0)
  {
    return angle - 180;
  }
  if(angle > 90.0)
  {
    return 180.0 - angle;
  }
  return angle;
}

Point2d rotatePoint(Point2d point, Point2d origin, double angle)
{
  // posun do pocatku souradnic
  double TmpX = point.x - origin.x;
  double TmpY = point.y - origin.y;

  Point2d newPoint;

  // rotace
  newPoint.x = TmpX * cos(angle) - TmpY * sin(angle);
  newPoint.y = TmpX * sin(angle) + TmpY * cos(angle);

  // posun zpet
  newPoint.x += origin.x;
  newPoint.y += origin.y;

  return newPoint;
}
