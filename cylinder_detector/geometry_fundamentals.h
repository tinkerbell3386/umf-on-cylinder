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
  cv::Point2d center;
  cv::Point2d mainEdge;
  cv::Point2d secondaryEdge;
  double thetaRadians;
  cv::RotatedRect boundingBox;

  TEllipse(){}
  TEllipse(cv::RotatedRect _boundingBox, int score = 0);
};

struct TLine {
  double a;
  double b;
  double c;
  int score; // number of origin points
  cv::Vec4f lineVector;
  cv::Point2d endPoint1;
  cv::Point2d endPoint2;

  TLine(){}
  TLine(cv::Point2d a, cv::Point2d b, int score = 0);
  TLine(cv::Vec4f _lineVector, int score = 0);
  TLine(double a, double b, double c, int score = 0);
};

struct TParabola {  
  cv::Point2d apex;
  double param;
  double angle;
  int score;
  
  TParabola(){}
  TParabola(cv::Point2d apex, double param, double angle, int score = 0);
};


cv::Point2d getLineIntersection(TLine p, TLine q);

double getPointToPointDistanceSquared(cv::Point2d a, cv::Point2d b);

double getDistanceLineToPointSquared(TLine baseLine, cv::Point2d point);

void drawLine(cv::Mat& img, TLine newLine, cv::Scalar color, int thickness = 1);

void drawPoint(cv::Mat& img, cv::Point2d point, cv::Scalar color, int size = 5);

//void drawParabola(cv::Mat& img, TParabola parabola, cv::Scalar color, int thickness = 1);

cv::Vec2f normalizeVector(cv::Vec2f vector);

TLine lineNormalization(TLine inputLine);

double getSmallerIntersectionAngle(TLine line1, TLine line2);

cv::Point2d rotatePoint(cv::Point2d point, cv::Point2d origin, double angle);

double getPointToPointDistance(cv::Point2d a, cv::Point2d b);

#endif // DP_GEOMETRY_FUNDAMENTALS_H
