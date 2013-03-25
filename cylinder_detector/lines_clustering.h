#ifndef _LINES_CLUSTERING_H_
#define _LINES_CLUSTERING_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_fundamentals.h"
#include "structures.h"

struct TLinesCluster
{
  TLine centroidLine;
  double variation;
  std::vector<TLine> lines;
};

class CLineClustring
{
public:
  CLineClustring(double _threshold, TLine _centralLine, TLine _borderLine);
  ~CLineClustring(){}

  void runLinesClustering(std::vector<TLine> inputLines, 
                          std::vector<TLine>& outputLines);
  
  void transformLinesToClusters(std::vector<TLine> lines); 
  
  void findMinimumDistancePair( double& minDist1, double& minDist2, 
                                int& positionCluster1, int& positionCluster2);
  
  void joinClusters(int positionCluster1, int positionCluster2);
 
  void getResultLines(std::vector<TLine>& lines);
  
  double computeEuclidDistance3DSquared(TLine line1, TLine line2);
  
  TLine getCentroidLine(std::vector<TLine> lines);
  
  double findMaximumDistance();
  
  double getStdDevMean();
  
  double getStdDev(std::vector<TLine> lines);
  
  bool operator()(TLinesCluster c1, TLinesCluster c2);
  
private:
  bool checkCondition();
  void cutLines(std::vector<TLine> inputLines, std::vector<TLine>& outputLines);
  
  std::vector<TLinesCluster> clusters;  // clusters
  double longestDistance;               // the longest distance so far (previous)
  double threshold;                     // threshold
  
  TLine centralLine;
  TLine borderLine;
};


#endif
