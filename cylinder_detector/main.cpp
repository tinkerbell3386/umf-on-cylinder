#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "edges.h"
#include "edgels.h"
#include "fitting.h"
#include "ellipse_ransac.h"
#include "umf_wrapper.h"

using namespace cv;
using namespace std;

int scanlineStep = 40;
int searchStep = 10;
int searchRadius = 3;

int main(int argc, char** argv)
{
  CFindEdges* findEdges = new CFindEdges(scanlineStep, 15, 25);
  CFindEdgels* findEdgels = new CFindEdgels(searchStep, searchRadius, 25);
  CLineAndEllipseFitting* fitting = new CLineAndEllipseFitting();
  CRansacEllipse* ransac = new CRansacEllipse(200, 10, 10, 3.0);
  CWrapper* wrapper = new CWrapper();

  for(int x = 1; x < 1000; x++) {

    stringstream str;
    str << "../data/images3/" << x << "data.jpg";
    //str << "data/test.jpg";
    cout << str.str() << endl;

    Mat source = imread(str.str());
    Mat draw = imread(str.str());

    if(source.data == NULL)
    {
      cout << "--------- NO MORE IMAGES -------" << endl;
      break;
    }


    if( source.empty()) break;

    Mat gray;
    cvtColor(source, gray, CV_BGR2GRAY);

    // detekce hran na mrizce
    vector<Point> edges;
    findEdges->findEdges(gray, edges);

    // detekce edgelu
    vector<vector<Point> > newEdges;
    findEdgels->getEdgesFromEdgePoints(gray, edges, newEdges, draw);

    // fittovani elips a primek
    fitting->setSizeThreslods(source.size());
    vector<TLine> lines;
    vector<TEllipse> ellipses;
    fitting->fitLinesOrEllipse(newEdges, ellipses, lines);

    vector<TLine> linesSelected;
    TLine refLine;
    //wrapper->setCenter(Point(source.cols / 2, source.rows / 2));
    //wrapper->getLineGroups(lines, linesSelected);
    Point2d vanishTest = wrapper->GetVanishingPoint(lines, linesSelected, Point(source.cols / 2, source.rows / 2));

    cout << "vanishTest: " << vanishTest << endl;

////////////////////////////////////////////////////////////////////////////////
    Mat rgb;
    source.copyTo(rgb);

    // scan lines
    for(int i = findEdges->scanlineStep/2; i < rgb.rows; i += findEdges->scanlineStep)
      line(rgb, Point(0, i), Point(rgb.cols-1, i), Scalar(120, 120, 0), 1);

    for(int i = findEdges->scanlineStep/2; i < rgb.cols; i += findEdges->scanlineStep)
      line(rgb, Point(i, 0), Point(i, rgb.rows-1), Scalar(120, 120, 0), 1);


    // origin edge points
    //for(int i = 0; i < (int)edges.size(); i++)
      //circle(rgb, edges[i], 3, Scalar(125, 125,125), -1);

    // points, lines, elipses
    for(int i = 0; i < (int)newEdges.size(); i++) {
      for(int j = 0; j < (int)newEdges[i].size(); j++) {
        if(newEdges[i].size() > 10)
        {
          drawPoint(rgb, newEdges[i][j], Scalar(255, 0, 0));
          drawPoint(rgb, edges[i], Scalar(0, 255,0));
        }
      }
    }

////////////////////////////////////////////////////////////////////////////////

    Mat rgb2;
    source.copyTo(rgb2);

    // points, all ellipses
    for(int i = 0; i < 11/*(int)lines.size()*/; i++)
    {
      drawLine(rgb2, lines[i], Scalar(255, 0, 0));
      drawPoint(rgb2, lines[i].endPoint1, Scalar(255, 255, 0));
      drawPoint(rgb2, lines[i].endPoint2, Scalar(255, 255, 0));
      cout << "score: " << lines[i].score << endl;
    }

    for(int i = 0; i < (int)ellipses.size(); i++)
    {
      ellipse(rgb2, ellipses[i].boundingBox, Scalar(0, 255, 0));
    }

////////////////////////////////////////////////////////////////////////////////

    Mat rgb3;
    source.copyTo(rgb3);

    int inliersNumber;
    vector<TEllipse> inliers;

    inliersNumber = ransac->fitEllipseRANSAC(ellipses, inliers);

    vector<Point> ellipseCenters;
    for(int i = 0; i < (int)inliers.size(); i++)
      ellipseCenters.push_back(inliers.at(i).center);

    TLine ransacLine;

    if(fitting->fitLineFromPoints(ellipseCenters, ransacLine))
      drawLine(rgb3, ransacLine, Scalar(255,0,0), 2);

    cout << "Number of inliers: " << inliersNumber << endl;

    for (int i=0;i != (int)ellipses.size(); ++i)
      ellipse(rgb3, ellipses.at(i).boundingBox, Scalar(0, 0, 255));

    for (int i=0;i != (int)inliers.size(); ++i)
      ellipse(rgb3, inliers.at(i).boundingBox, Scalar(0, 255, 0), 2);

////////////////////////////////////////////////////////////////////////////////
    Mat rgb4;
    source.copyTo(rgb4);

    // origin edge points
    for(int i = 0; i < (int)linesSelected.size(); i++)
      drawLine(rgb4, linesSelected[i], Scalar(255,0,0), 1);

    drawLine(rgb4, refLine, Scalar(0, 0, 255), 2);

    //for(int i = 0; i < (int)lines.size(); i++)
    //  drawLine(rgb4, lines[i], Scalar(0, 0, 120));

////////////////////////////////////////////////////////////////////////////////
/*    Mat rgb5;
    source.copyTo(rgb5);

    for(int i = 0; i < (int)newEdges.size(); i++)
    {
      TEllipse newEllipse;
      TLine newLine;
      enShapeType shapeType = fitLineOrEllipse(newEdges[i], newLine, newEllipse);
      if(IS_ELLIPSE == shapeType)
      {
        ellipse(rgb5, newEllipse.boundingBox, Scalar(0, 255, 255));
        drawPoint(rgb5, newEllipse.center, Scalar(255, 255, 0));
        drawPoint(rgb5, newEllipse.mainEdge, Scalar(255, 0, 0));
        line(rgb5, newEllipse.center, newEllipse.mainEdge, Scalar(0, 0, 255));

        Point2f vertices[4];
        newEllipse.boundingBox.points(vertices);
        for (int i = 0; i < 4; i++)
          line(rgb5, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
      }
    }
*/
////////////////////////////////////////////////////////////////////////////////


    cout << "-------" << x << "-------" << endl;

    //imshow("Output: points, lines, elipses", rgb);
    imshow("Output: points, all ellipses", rgb2);
    //imshow("Output: ransac line, correct ellipses", rgb3);
   imshow("Output: selected lines", rgb4);
    //imshow("Output: ellipses", rgb5);
    //imshow("Output: draw", draw);

    stringstream str2;
    str2 << "1screenshot" << x << ".png";

    stringstream str3;
    str3 << "2screenshot" << x << ".png";

    imwrite(str2.str(), rgb);
    imwrite(str3.str(), rgb2);

    uchar c = (uchar)waitKey();

    if(c == 27)
    {
      break;
    }
  }

  delete findEdges;
  delete findEdgels;
  delete ransac;
  delete fitting;

  return 0;
}
