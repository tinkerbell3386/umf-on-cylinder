#include "parabolas_ransac.h"
#include "lines_fitting.h"

using namespace cv;
using namespace std;

CRansacParabola::CRansacParabola(int _numberOfIteration,
                               double _modelDistanceTrashold
) :
CRansac<TParabola>(_numberOfIteration, 2),
modelDistanceTrashold(_modelDistanceTrashold)
{
  ;
}

int CRansacParabola::fitParabolaRANSAC(vector<TParabola> parabolas,
                                     vector<TParabola>& inliers)
{  
  vector<TParabola> tmpInliers;
  tmpInliers.clear();
  
  runRansac(parabolas, tmpInliers);
  
  recomputeParams(tmpInliers);
  
  getFinalInliers(tmpInliers, inliers);
  
  recomputeParams(inliers);
  
  return (int)inliers.size();
}

void CRansacParabola::recomputeParams(vector<TParabola> inliers)
{
  vector<Point2f> points;
  for(int i = 0; i < (int)inliers.size(); i++)
  {
    points.push_back(Point2f(inliers.at(i).param, 
                             inliers.at(i).apex.y));
  }
  
  CFittingLine* fitLine = new CFittingLine();
  fitLine->fitLineFromPoints(points, testLine);
  delete fitLine;
}

void CRansacParabola::getFinalInliers(vector<TParabola> parabolas,
                                      vector<TParabola>& inliers)
{
  inliers.clear();
  for(int i = 0; i < (int)parabolas.size(); i++)
  {
    if(fitRansacModel(parabolas.at(i)))
    {
      inliers.push_back(parabolas.at(i));
    }
  }
}

bool CRansacParabola::isModel(std::vector<TParabola> modelParabola)
{
 
  testLine = TLine(Point2f(modelParabola.at(0).param, modelParabola.at(0).apex.y), 
                   Point2f(modelParabola.at(1).param, modelParabola.at(1).apex.y));

  return true;
}

bool CRansacParabola::fitRansacModel(TParabola testedParabola)
{
  double distance = getDistanceLineToPointSquared(testLine, 
                                                   Point2f(testedParabola.param, 
                                                           testedParabola.apex.y));
  
  if(distance > modelDistanceTrashold*modelDistanceTrashold)
  {
    cout << "Distance fail: " << distance << endl;
    return false;
  }
  cout << "Distance: " << modelDistanceTrashold*modelDistanceTrashold << " X " << distance << endl;
  
  return true;
}
