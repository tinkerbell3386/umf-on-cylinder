#ifndef DP_FITTING_H
#define DP_FITTING_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "geometry_fundamentals.h"


class CLineAndEllipseFitting
{
public:
  CLineAndEllipseFitting(cv::Size imageSize, double _smallRatioThreshold = 0.01);
  CLineAndEllipseFitting(double _smallRatioThreshold = 0.01);
  ~CLineAndEllipseFitting(){};

  void setSizeThreslods(cv::Size imageSize);

  void fitLinesOrEllipse(std::vector<std::vector<cv::Point> > edges,
                         std::vector<TEllipse>& ellipses,
                         std::vector<TLine>& lines
                        );

  enum enShapeType {
    IS_LINE,
    IS_ELLIPSE,
    IS_UNKNOWN
  };

  bool fitLineFromPoints(std::vector<cv::Point> points, TLine &newLine);

  bool fitEllipseFromPoints(std::vector<cv::Point> points, TEllipse &newEllipse);

  enShapeType fitLineOrEllipse(std::vector<cv::Point> points, TLine &newLine, TEllipse &newEllipse);

  bool isLine(double a, double b);

private:
  double smallRatioThreshold;
  int aLongThreshold;
  int aShortThreshold;
};



#endif // DP_FITTING_H




