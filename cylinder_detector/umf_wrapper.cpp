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

Point2d CWrapper::GetVanishingPoint(vector<TLine> lines, vector<TLine>& outputLlines, Point center)
{
  setCenter(center);

  vector<TLine> linesGroup;
  int index = getLineGroups(lines, linesGroup);

  vector<Line> linesGroupConverted;
  for(int i = 0; i < (int)linesGroup.size(); i++)
  {
    linesGroupConverted.push_back(convertLineWhole(linesGroup.at(i)));
  }

  vector<Line> umfLines;
  getGroup(linesGroupConverted, umfLines, convertLineBase(refLines[index]), cvPoint(imageCenter.x, imageCenter.y));

  std::cout << "lines2: " << umfLines[0].a << std::endl;
  std::cout << "lines2: " << umfLines[0].b << std::endl;
  std::cout << "lines2: " << umfLines[0].c << std::endl;
  std::cout << "lines2: " << umfLines[0].score << std::endl;
  std::cout << "lines2: " << umfLines[0].crossi << std::endl;
  std::cout << "lines2: " << umfLines[0].ip << std::endl;
  std::cout << "lines2: " << umfLines[0].ip2 << std::endl;

  vector<Line> vanishLines;
  Line tmpVanish;
  tmpVanish = getVanish(umfLines, vanishLines, cvPoint(imageCenter.x, imageCenter.y), true);

  outputLlines.clear();
  for(int i = 0; i < (int)vanishLines.size(); i++)
  {
    outputLlines.push_back(convertLineWholeReverse(vanishLines.at(i)));
  }

  return Point2d(tmpVanish.a / tmpVanish.c, tmpVanish.b / tmpVanish.c);
}

int CWrapper::getLineGroups(vector<TLine> lines, vector<TLine>& linesGroup)
{
  //create set of lines for the four directions
  vector<TLine> LineDirectionGroups[4];
  int LineDirectionScores[] = {0, 0, 0, 0};

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
      }
    }
  }

  // Find the max element
  int index = max_element(LineDirectionScores, LineDirectionScores + 4) - LineDirectionScores;

  linesGroup = LineDirectionGroups[index];

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

  cout << "to -> lineVector: " << inputLine.lineVector << endl;

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

  cout << "back -> outputLine.lineVector: " << outputLine.lineVector << endl;

  return outputLine;
}
