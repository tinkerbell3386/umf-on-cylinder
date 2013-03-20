#ifndef DP_LINE_FITTING_H
#define DP_LINE_FITTING_H

#include "geometry_fundamentals.h"

class CFittingLine
{
public:
  CFittingLine(){}
  ~CFittingLine(){}
  
  void fitLines(std::vector<std::vector<cv::Point2f> > points,
                std::vector<TLine>& lines);
  bool fitLineFromPoints(std::vector<cv::Point2f> points, TLine& newline);
};

#endif // DP_LINE_FITTING_H
