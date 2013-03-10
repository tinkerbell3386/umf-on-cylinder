#ifndef _UMF_WRAPPER_H_
#define _UMF_WRAPPER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_fundamentals.h"
#include "structures.h"

class CWrapper
{
public:
  CWrapper();
  ~CWrapper(){};

  void setCenter(cv::Point imageCenter);
  int getLineGroups(std::vector<TLine> lines, std::vector<TLine>& linesGroup);

  cv::Point2d GetVanishingPoint(std::vector<TLine> lines, 
                                std::vector<TLine>& outputLlines, 
                                TLine& normal, cv::Point center);

  Line convertLineBase(TLine inputLine);
  Line convertLineWhole(TLine inputLine);

  TLine convertLineBaseReverse(Line inputLine);
  TLine convertLineWholeReverse(Line inputLine);

private:
  TLine refLines[4];
  cv::Point imageCenter;
};


#endif
