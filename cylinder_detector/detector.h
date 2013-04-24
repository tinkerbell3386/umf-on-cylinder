#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include "edges.h"
#include "edgels.h"
#include "lines_fitting.h"
#include "ellipses_fitting.h"
#include "ellipses_ransac.h"
#include "umf_wrapper.h"
#include "lines_clustering.h"
#include "ellipses_clustering.h"
#include "parabolas_fitting.h"
#include "parabolas_ransac.h"
#include "parabolas_clustering.h"
#include "supplement.h"
#include "find_grid.h"

class CDetector
{
public:
  CDetector(std::string parameterFileName);
  ~CDetector();
  
  void runDetectorTest(std::string fileName);

private:
  // edges
  int adaptiveThreshold;
  int scanlineStep;
  int bufferSize;
  
  // edgels
  int searchStep;
  int searchRadius;
  int searchThreshold;
  
  // RANSAC ellipse
  int numberOfIterationEllipse;
  int modelDistanceTrashold;
  int modelPyramideDistanceTreshold;
  double modelAngleTreshold;
  
  // RANSAC parabola
  int numberOfIterationParabola;
  double modelDistanceThresholdParameters;
  double modelDistanceThresholdHorizon;
  
  // supplement
  double distanceSupplementThreshold;
  double correctnessSupplementThreshold;
  
  CFindEdges* findEdges;
  CFindEdgels* findEdgels;
  CFittingLine* lineFitting;
  CEllipseFitting* ellipseFitting;
  CWrapper* wrapper;
  CEllipseClustring* clusteringellipse;
  CRansacParabola* parabolaRansac;
  CParabolaClustring* parabolaClustring;
  CSupplement* supplement;
};

#endif // _DETECTOR_H_
