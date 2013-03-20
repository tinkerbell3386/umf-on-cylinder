#ifndef DP_GEOMETRY_FUNDAMENTALS_H
#define DP_GEOMETRY_FUNDAMENTALS_H

#include <opencv2/opencv.hpp>
#include <vector>

const double PI = 3.14159265;

struct TEllipse {
  double a;
  double b;
  double e;
  int score; // number of origin points
  cv::Point2f center;
  cv::Point2f mainEdge;
  cv::Point2f secondaryEdge;
  double thetaRadians;
  cv::RotatedRect boundingBox;

  std::vector<cv::Point2f> points;
  
  TEllipse(){}
  TEllipse(cv::RotatedRect _boundingBox, cv::Point2f borderPoint, int score = 0);
};

struct TLine {
  double a;
  double b;
  double c;
  int score; // number of origin points
  double deviation;
  
  std::vector<cv::Point2f> points;
  
  cv::Vec4f lineVector;
  cv::Point2f endPoint1;
  cv::Point2f endPoint2;

  TLine(){}
  TLine(cv::Point2f a, cv::Point2f b, int score = 0);
  TLine(cv::Vec4f _lineVector, int score = 0);
  TLine(double a, double b, double c, int score = 0);
};

struct TParabola {  
  cv::Point2f apex;
  double param;
  double angle;
  double origin;
  int score;
  
  std::vector<cv::Point2f> points;
  
  TParabola(){}
  TParabola(cv::Point2f apex, double param, double angle, double origin, int score = 0);
};


cv::Point2f getLineIntersection(TLine p, TLine q);

double getPointToPointDistanceSquared(cv::Point2f a, cv::Point2f b);

double getDistanceLineToPointSquared(TLine baseLine, cv::Point2f point);

void drawLine(cv::Mat& img, TLine newLine, cv::Scalar color, int thickness = 1);

void drawPoint(cv::Mat& img, cv::Point2f point, cv::Scalar color, int size = 5);

//void drawParabola(cv::Mat& img, TParabola parabola, cv::Scalar color, int thickness = 1);

cv::Vec2f normalizeVector(cv::Vec2f vector);

TLine lineNormalization(TLine inputLine);

double getSmallerIntersectionAngle(TLine line1, TLine line2);

cv::Point2f rotatePoint(cv::Point2f point, cv::Point2f origin, double angle);

double getPointToPointDistance(cv::Point2f a, cv::Point2f b);

#endif // DP_GEOMETRY_FUNDAMENTALS_H
