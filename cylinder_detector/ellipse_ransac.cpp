#include "ellipse_ransac.h"

using namespace cv;
using namespace std;

CRansacEllipse::CRansacEllipse(int _numberOfIteration,
                               int _modelDistanceTrashold,
                               int _modelPyramideDistanceTreshold,
                               double _modelAngleTreshold
                              ) :
  CRansac(2, _numberOfIteration),
  modelDistanceTrashold(_modelDistanceTrashold),
  modelPyramideDistanceTreshold(_modelPyramideDistanceTreshold),
  modelAngleTreshold(_modelAngleTreshold)

{}

int CRansacEllipse::fitEllipseRANSAC(vector<TEllipse> ellipses,
                                  vector<TEllipse>& inliers)
{
  return runRansac(ellipses, inliers);
}

bool CRansacEllipse::isModel(vector<TEllipse> ellipses)
{
  // check if we have enought ellipses
  if(ellipses.size() != 2)
  {
    cerr << "WARNING: Incorrect number of ellipses for model. Needed 2 but get "
    << ellipses.size() << "." << endl;
    return false;
  }

  // when ellipses have the same center
  if(ellipses.at(0).center == ellipses.at(1).center)
  {
    cerr  << "WARNING: Selected ellipses have equal center and cannot be used "
          << "as a model." <<endl;
    return false;
  }

  // distance criterium

  // central point
  Point2d central((ellipses.at(0).center.x - ellipses.at(1).center.x) / 2 + ellipses.at(1).center.x,
                  (ellipses.at(0).center.y - ellipses.at(1).center.y) / 2 + ellipses.at(1).center.y
                 );

  // lines made by main axises of ellipses
  TLine line1(ellipses.at(0).mainEdge, ellipses.at(0).center);
  TLine line2(ellipses.at(1).mainEdge, ellipses.at(1).center);

  // normal line to previous lines which goes trough central point
  distanceLine = TLine(Vec4d((line1.a - line2.a) / 2 + line2.a,
                              (line1.b - line2.b) / 2 + line2.b,
                              central.x,
                              central.y
                            )
                      );

  // test if ellipses are not to far from distance line
  double distance =  getDistanceLineToPointSquared(distanceLine,
                                                   ellipses.at(0).center);

  if(distance > modelDistanceTrashold*modelDistanceTrashold)
  {
    cerr  << "WARNING: Distance between model elipses centers and normal "
      << "distance line is to big. Distance is: " << distance
      << "and maximum distance is: "
      << modelDistanceTrashold*modelDistanceTrashold << endl;
    return false;
  }


  // main ellipse axis paralllel criterium
  double angle = getSmallerIntersectionAngle(line1, line2);

  if(angle > 5.0)
  {
    cerr  << "WARNING: Selected model ellipses have not parallel main axis. "
          << "Angle between them is: " << angle << endl;
    return false;
  }

  // pyramide criterium - lines from models
  // main line
  pyramideDistanceLine = TLine(ellipses.at(0).center, ellipses.at(1).center);

  // tree shape line, ellipse with index 0 is origin point, new space
  pyramideTreeShapedLine = TLine(Point2d(ellipses.at(0).a,
                                          0
                                         ),
                                  Point2d(ellipses.at(1).a,
                                          getPointToPointDistance(
                                            ellipses.at(0).center,
                                            ellipses.at(1).center
                                                                 )
                                        )
                                );

  return true;
}

bool CRansacEllipse::fitRansacModel(vector<TEllipse> ellipses, TEllipse x)
{
  // distace criterium
  if(getDistanceLineToPointSquared(distanceLine, x.center) >
    modelDistanceTrashold*modelDistanceTrashold)
  {
    return false;
  }

  // parrallel criterium
  double angle1 = getSmallerIntersectionAngle(TLine(ellipses.at(0).mainEdge,
                                                    ellipses.at(0).center),
                                              TLine(x.mainEdge,
                                                    x.center)
                                             );

  double angle2 = getSmallerIntersectionAngle(TLine(ellipses.at(1).mainEdge,
                                                    ellipses.at(1).center),
                                              TLine(x.mainEdge,
                                                    x.center)
                                             );

  if(angle1 > modelAngleTreshold || angle2 > modelAngleTreshold)
  {
    return false;
  }

  // distance from orhogonal line
  // central point

  TLine(ellipses.at(0).center,
        ellipses.at(0).mainEdge);
  TLine(ellipses.at(1).center,
        ellipses.at(1).mainEdge);

  // pyramide criterium
  // intersection between distace line and its normal which goes through center
  // of tested ellipse
  Point2d intersection = getLineIntersection(pyramideDistanceLine,
                                             TLine(Vec4f(pyramideDistanceLine.a,
                                                         pyramideDistanceLine.b,
                                                         x.center.x,
                                                         x.center.y
                                                        )
                                                  )
                                            );

  // y coordinate correction because we need correct position in y axis in new
  // space
  double yCoordinate;
  if( getPointToPointDistanceSquared(intersection, ellipses.at(0).center) <
      getPointToPointDistanceSquared(intersection, ellipses.at(1).center)
    )
  {
    yCoordinate = -getPointToPointDistance(ellipses.at(0).center, intersection);
  }
  else
  {
    yCoordinate = getPointToPointDistance(ellipses.at(0).center, intersection);
  }

  // check distance of the ellips main point from tree line
  Point2d(x.a, yCoordinate);
  if( getDistanceLineToPointSquared(pyramideTreeShapedLine, Point2d(x.a, yCoordinate)) >
      modelPyramideDistanceTreshold*modelPyramideDistanceTreshold
    )
  {
    return false;
  }

  return true;
}
