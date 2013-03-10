#include "ellipse_ransac.h"

using namespace cv;
using namespace std;

CRansacEllipse::CRansacEllipse(int _numberOfIteration,
                               TLine _centralLine,
                               Point2d _vanishingPoint,
                               int _modelDistanceTrashold,
                               int _modelPyramideDistanceTreshold,
                               double _modelAngleTreshold                             
                              ) :
  CRansac(_numberOfIteration),
  centralLine(_centralLine),
  vanishingPoint(_vanishingPoint),
  modelDistanceTrashold(_modelDistanceTrashold),
  modelPyramideDistanceTreshold(_modelPyramideDistanceTreshold),
  modelAngleTreshold(_modelAngleTreshold)
{}

int CRansacEllipse::fitEllipseRANSAC(vector<TEllipse> ellipses,
                                  vector<TEllipse>& inliers)
{
  vector<TEllipse> eliminatedEllipses;
  eliminateWrongEllipses(ellipses, eliminatedEllipses);
  
  inliers.clear();
  return runRansac(eliminatedEllipses, inliers);
}

bool CRansacEllipse::isModel(TEllipse modelEllipse)
{
  // pyramide criterium - lines from models
  // main line
  pyramideDistanceLine = TLine(modelEllipse.center, vanishingPoint);

  // tree shape line, ellipse with index 0 is origin point, new space
  pyramideTreeShapedLine = TLine(Point2d(0, 0),
                                 Point2d(modelEllipse.a,
                                          getPointToPointDistance(
                                            vanishingPoint,
                                            modelEllipse.center
                                          )
                                        )
                                );

  return true;
}

void CRansacEllipse::eliminateWrongEllipses(vector<TEllipse> inputEllipses,
                                            vector<TEllipse>& outputEllipses) 
{  
  outputEllipses.clear();
  for(int i = 0; i < (int)inputEllipses.size(); i++)
  {
    double distance = getDistanceLineToPointSquared(centralLine, 
                                                    inputEllipses[i].center);
    
    if(distance > modelDistanceTrashold*modelDistanceTrashold)
    {
      continue;
    }
    
    //cout << "distance: " << distance << " > " << modelPyramideDistanceTreshold*modelPyramideDistanceTreshold << endl;
    
    // orthogonal criterium
    double angle = getSmallerIntersectionAngle( centralLine,
                                                TLine(inputEllipses[i].mainEdge,
                                                      inputEllipses[i].center)
    );
    
    if(std::abs(angle - 90) > modelAngleTreshold)
    {
      continue;
    } 
    
    //cout << "angle: " << std::abs(angle - 90) << " > " << modelAngleTreshold << endl;
    
    outputEllipses.push_back(inputEllipses[i]);
  }
}

bool CRansacEllipse::fitRansacModel(TEllipse testedEllipse)
{
  // pyramide criterium
  // intersection between distace line and its normal which goes through center
  // of tested ellipse
  Point2d intersection = getLineIntersection(pyramideDistanceLine,
                                             TLine(Vec4f(pyramideDistanceLine.a,
                                                         pyramideDistanceLine.b,
                                                         testedEllipse.center.x,
                                                         testedEllipse.center.y
                                                        )
                                                  )
                                            );

  double yCoordinate = getPointToPointDistance(vanishingPoint, 
                                               intersection);
  
  double distance = getDistanceLineToPointSquared(pyramideTreeShapedLine, 
                                                  Point2d(testedEllipse.a, 
                                                          yCoordinate));
  
  //cout << "distance: " << distance << " > " << modelPyramideDistanceTreshold*modelPyramideDistanceTreshold << endl;
  
  // check distance of the ellips main point from tree line
  if(distance > modelPyramideDistanceTreshold*modelPyramideDistanceTreshold)
  {
    return false;
  }

  return true;
}
