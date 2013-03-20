#ifndef _ELLIPSE_CLUSTERING_H_
#define _ELLIPSE_CLUSTERING_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_fundamentals.h"
#include "structures.h"

struct TEllipseCluster
{
  TEllipse centroidEllipse;
  double variation;
  std::vector<TEllipse> ellipses;
};

class CEllipseClustring
{
public:
  CEllipseClustring(double _threshold);
  ~CEllipseClustring(){}
  
  void runEllipsesClustering(std::vector<TEllipse> inputEllipses, 
                          std::vector<TEllipse>& outputEllipses);
  
  void transformEllipsesToClusters(std::vector<TEllipse> ellipses); 
  
  void findMinimumDistancePair( double& minDist1, double& minDist2, 
                                int& positionCluster1, int& positionCluster2);
  
  void joinClusters(int positionCluster1, int positionCluster2);
  
  void getResultEllipses(std::vector<TEllipse>& ellipses);
  
  double computeEuclidDistance4DSquared(TEllipse ellipse1, TEllipse ellipse2);
  
  TEllipse getCentroidEllipse(std::vector<TEllipse> ellipses);
  
  double findMaximumDistance();
  
  double getStdDevMean();
  
  double getStdDev(std::vector<TEllipse> ellipses);
    
private:
  
  std::vector<TEllipseCluster> clusters;  // clusters
  double longestDistance;               // the longest distance so far (previous)
  double threshold;                     // threshold
};


#endif
