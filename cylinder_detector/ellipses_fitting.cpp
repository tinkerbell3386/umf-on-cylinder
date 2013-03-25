#include "ellipses_fitting.h"

using namespace std;
using namespace cv;

bool CEllipseFitting::fitEllipseFromPoints(vector<Point2f> points, TEllipse& newEllipse)
{
  if(points.size() < 5)
  {
    cerr << "WARNING: Need at least 5 points to fit an ellipse" << endl;
    return false;
  }
  
  newEllipse = TEllipse(fitEllipse(points), points.at(points.size() / 2), points.size());
  newEllipse.points = points;
  
  return true;
}

void CEllipseFitting::fitEllipsesFromLines(vector<TLine> lines, 
                                           vector< TEllipse >& ellipses)
{
  for(int i = 0; i < (int)lines.size(); i++)
  {
    TEllipse newEllipse;
    if(fitEllipseFromPoints(lines.at(i).points, newEllipse))
    {
      ellipses.push_back(newEllipse);
    }
  }
}
