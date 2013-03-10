#include <iostream>
#include "fitting.h"

using namespace cv;
using namespace std;

CLineAndEllipseFitting::CLineAndEllipseFitting(cv::Size imageSize,
                                               double _smallRatioThreshold) :
  smallRatioThreshold(_smallRatioThreshold)
{
  setSizeThreslods(imageSize);
  aShortThreshold = 50;
}

CLineAndEllipseFitting::CLineAndEllipseFitting(
  double _smallRatioThreshold):
  smallRatioThreshold(_smallRatioThreshold)
{
  aLongThreshold = 1000;
  aShortThreshold = 50;
}

void CLineAndEllipseFitting::setSizeThreslods(Size imageSize)
{
  if(imageSize.height > imageSize.width)
  {
    aLongThreshold = imageSize.height / 2;
  }
  else
  {
    aLongThreshold = imageSize.width / 2;
  }
}

bool CLineAndEllipseFitting::fitLineFromPoints(vector<Point> points,
                                                TLine &newLine)
{
  if(points.size() < 2)
  {
    cerr << "WARNING: Need at least 2 points to fit a line" << endl;
    return false;
  }

  Vec4f fittedLine;
  fitLine(points, fittedLine, CV_DIST_L2, 0, 0.01, 0.01);

  newLine = TLine(fittedLine, points.size());

  //newLine = TLine(points.at(points.size() - 1), points.at(points.size() - 2), points.size());

  newLine.endPoint1 = points.at(points.size() - 1);
  newLine.endPoint2 = points.at(points.size() - 2);

  return true;
}

bool CLineAndEllipseFitting::fitEllipseFromPoints(vector<Point> points,
                                                   TEllipse &newEllipse)
{
  if(points.size() < 5)
  {
    cerr << "WARNING: Need at least 5 points to fit an ellipse" << endl;
    return false;
  }

  newEllipse = TEllipse(fitEllipse(points), points.size());

  return true;
}

bool CLineAndEllipseFitting::isLine(double a, double b)
{
  //cout << "a: " << a << " b: " << b << " / " <<  b / a << endl;
  return smallRatioThreshold > b / a || aLongThreshold < a || aShortThreshold > a;
}

CLineAndEllipseFitting::enShapeType CLineAndEllipseFitting::fitLineOrEllipse(vector<Point> points,
                                                      TLine &newLine,
                                                      TEllipse &newEllipse)
{
  TLine tmpLine;
  TEllipse tmpEllipse;

  if(points.size() < 5)
  {
    return IS_UNKNOWN;
  }
  else if(fitEllipseFromPoints(points, tmpEllipse))
  {
    if(isLine(tmpEllipse.a, tmpEllipse.b))
    {
      if(fitLineFromPoints(points, tmpLine))
      {
        newLine = tmpLine;
        return IS_LINE;
      }
    }
    else
    {
      newEllipse = tmpEllipse;
      return IS_ELLIPSE;
    }
  }
  return IS_UNKNOWN;
}

void CLineAndEllipseFitting::fitLinesOrEllipse(vector<vector<Point> > edges,
                                                vector<TEllipse>& ellipses,
                                                vector<TLine>& lines
                                                )
{
  ellipses.clear();
  lines.clear();

  for(int i = 0; i < (int)edges.size(); i++)
  {
    TLine newLine;
    TEllipse newEllipse;
    enShapeType shapeType = fitLineOrEllipse(edges[i], newLine, newEllipse);
    if(IS_ELLIPSE == shapeType)
    {
      ellipses.push_back(newEllipse);
    }
    else if(IS_LINE == shapeType)
    {
      lines.push_back(newLine);
    }
  }
}
