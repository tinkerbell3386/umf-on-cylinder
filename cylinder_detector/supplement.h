#ifndef _SUPPLEMENT_H_
#define _SUPPLEMENT_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "geometry_fundamentals.h"
#include "structures.h"


/**
 * Struktura CSupplement
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CSupplement
{
public:
  
  CSupplement(double _distanceSupplementThreshold, double correctnessSupplementThreshold);
  ~CSupplement(){}
  
  void runSupplement(std::vector<TParabola> inputParabolas, 
                     std::vector<TParabola>& outputParabolas, 
                     cv::Point2f referencePoint);
  
  //double computeDistance(int index);
  
  int getScore(int index, std::vector<TParabola> inputParabolas, double& parameter);
  
private:
  
  double distanceSupplementThreshold;
  double correctnessSupplementThreshold;
  //double distance;
};

#endif
