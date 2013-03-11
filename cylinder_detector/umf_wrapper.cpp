  #include "umf_wrapper.h"
#include "detect_directions.h"
#include "detect_umf.h"

using namespace std;
using namespace cv;

CWrapper::CWrapper()
{
  setCenter(Point(0 ,0));
}

void CWrapper::setCenter(Point _imageCenter)
{
  imageCenter = _imageCenter;

  // lines goes trough the image center: c = - a*center.x - b*center.y;
  // vertical
  refLines[0] = lineNormalization(TLine(1, 0, - imageCenter.x));

  // 45 degree
  refLines[1] = lineNormalization(TLine(1, 1, - imageCenter.x - imageCenter.y));

  // horizontal
  refLines[2] = lineNormalization(TLine(0, 1, -imageCenter.y));

  // 135 degree
  refLines[3] = lineNormalization(TLine(-1, 1, imageCenter.x - imageCenter.y));
}

Point2d CWrapper::GetVanishingPoint(vector<TLine> lines, vector<TLine>& outputLlines, TLine& normal, Point center)
{
  setCenter(center);

  vector<TLine> linesGroup;
  vector<TLine> linesGroup2;
  int index = getLineGroups(lines, linesGroup, linesGroup2);

  vector<Line> linesGroupConverted;
  for(int i = 0; i < (int)linesGroup.size(); i++)
  {
    linesGroupConverted.push_back(convertLineWhole(linesGroup.at(i)));
  }

  vector<Line> umfLines;
  getGroup(linesGroupConverted, umfLines, convertLineBase(refLines[index]), cvPoint(imageCenter.x, imageCenter.y));
  
  vector<Line> vanishLines;
  Line tmpVanish;
  tmpVanish = getVanish(umfLines, vanishLines, cvPoint(imageCenter.x, imageCenter.y), true);

  outputLlines.clear();
  for(int i = 0; i < (int)vanishLines.size(); i++)
  {
    outputLlines.push_back(convertLineWholeReverse(vanishLines.at(i)));
  }

  normal = convertLineBaseReverse(tmpVanish);
  
  return getLineIntersection(outputLlines[0], outputLlines[1]);
}

int CWrapper::getLineGroups(vector<TLine> lines, vector<TLine>& linesGroup, vector<TLine>& linesGroup2)
{
  //create set of lines for the four directions
  vector<TLine> LineDirectionGroups[4];
  int LineDirectionScores[] = {0, 0, 0, 0};
  double LineDirectionDev[] = {0, 0, 0, 0};
  
  for(int i = 0; i < (int)lines.size(); i++)
  {
    TLine testLine = lineNormalization(lines[i]); //normalize, so we don't have to calculate length

    for(int j = 0; j < 4; j++)
    {
      //first calculate dot product3
      // skalarni soucin normalizovanych vektoru je je cos uhlu, ktery sviraji
      double dotp = std::abs(refLines[j].a * testLine.a + refLines[j].b * testLine.b);
      if(dotp < 0.5) // cos PI/3 - since there should be PI/2 to get the intersection right - this gives PI/6 allowance in both directions
      {              // je tu presah, jedna cara muze byt ve vice skupinach
        LineDirectionGroups[j].push_back(lines[i]);
        LineDirectionScores[j] += lines[i].score;
        LineDirectionDev[j] += lines[i].deviation;
      }
    }
  }
  
  double maxValue =  LineDirectionScores[0] / (LineDirectionDev[0] / (LineDirectionGroups[0].size()+1)+1);
  int index = 0;
  cout << "result value " << index << ": " << maxValue << endl;
  
  for(int i = 1; i < 4; i++)
  {
    double value =  LineDirectionScores[i] / (LineDirectionDev[i] / (LineDirectionGroups[i].size()+1)+1);
    cout << "result value " << i << ": " << value << endl;
    if(maxValue < value)
    {
      maxValue = value;
      index = i;
    }
  }
  
  // Find the max element
  //int index = max_element(LineDirectionResult, LineDirectionResult + 4 * sizeof(double)) - LineDirectionResult;

  linesGroup = LineDirectionGroups[index];
  
  linesGroup2 = LineDirectionGroups[(index + 2) % 4];
  
  return index;
}

Line CWrapper::convertLineBase(TLine inputLine)
{
  Line outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;

  return outputLine;
}

Line CWrapper::convertLineWhole(TLine inputLine)
{
  Line outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;
  outputLine.score = inputLine.score;
  outputLine.crossi = 0.0;
  outputLine.ip = 0.0;
  outputLine.ip2 = 0.0;
  outputLine.endPoint1 = cvPoint2D32f(inputLine.endPoint1.x, inputLine.endPoint1.y);
  outputLine.endPoint2 = cvPoint2D32f(inputLine.endPoint2.x, inputLine.endPoint2.y);

  return outputLine;
}

TLine CWrapper::convertLineBaseReverse(Line inputLine)
{
  TLine outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;

  return outputLine;
}

TLine CWrapper::convertLineWholeReverse(Line inputLine)
{
  TLine outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;
  outputLine.score = inputLine.score;
  outputLine.lineVector = Vec4f(-inputLine.b, inputLine.a, inputLine.endPoint1.x, inputLine.endPoint1.y);
  outputLine.endPoint1 = Point2d(inputLine.endPoint1.x, inputLine.endPoint1.y);
  outputLine.endPoint2 = Point2d(inputLine.endPoint2.x, inputLine.endPoint2.y);

  return outputLine;
}
