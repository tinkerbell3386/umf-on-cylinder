#ifndef _ELLIPSE_FITTING_H_
#define _ELLIPSE_FITTING_H_

#include "geometry_fundamentals.h"
#include <opencv2/opencv.hpp>
#include <vector>

class CEllipseFitting
{
public:
  CEllipseFitting(){}
  ~CEllipseFitting(){}
  
  bool fitEllipseFromPoints(std::vector<cv::Point2f> points, TEllipse &newEllipse);
  void fitEllipsesFromLines(std::vector<TLine> points, std::vector<TEllipse>& ellipses);
};

#endif
