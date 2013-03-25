#ifndef _PARABOLA_CLUSTERING_H_
#define _PARABOLA_CLUSTERING_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_fundamentals.h"

struct TParabolaCluster
{
  TParabola centroidParabola;
  double variation;
  std::vector<TParabola> parabolas;
};

class CParabolaClustring
{
public:
  CParabolaClustring(double _threshold);
  ~CParabolaClustring(){}
  
  void runParabolasClustering(std::vector<TParabola> inputParabolas, 
                             std::vector<TParabola>& outputParabolas);
  
  void transformParabolasToClusters(std::vector<TParabola> parabolas); 
  
  void findMinimumDistancePair( double& minDist1, double& minDist2, 
                                int& positionCluster1, int& positionCluster2);
  
  void joinClusters(int positionCluster1, int positionCluster2);
  
  void getResultParabolas(std::vector<TParabola>& parabolas);
  
  double computeEuclidDistanceSquared(TParabola parabola1, TParabola parabola2);
  
  TParabola getCentroidParabola(std::vector<TParabola> parabolas);
  
  double findMaximumDistance();
  
  double getStdDevMean();
  
  double getStdDev(std::vector<TParabola> parabolas);
  
private:
  
  std::vector<TParabolaCluster> clusters;  // clusters
  double longestDistance;               // the longest distance so far (previous)
  double threshold;                     // threshold
};


#endif
