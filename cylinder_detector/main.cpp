#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "edges.h"
#include "edgels.h"
#include "fitting.h"
#include "ellipse_ransac.h"
#include "umf_wrapper.h"
#include "lines_clustering.h"

using namespace cv;
using namespace std;

int scanlineStep = 20;
int searchStep = 10;
int searchRadius = 3;

int main(int argc, char** argv)
{
  CFindEdges* findEdges = new CFindEdges(scanlineStep, 15, 25);
  CFindEdgels* findEdgels = new CFindEdgels(searchStep, searchRadius, 25);
  CLineAndEllipseFitting* fitting = new CLineAndEllipseFitting();
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

    vector<TLine> linesGrouped;
    vector<TLine> linesGrouped2;
    wrapper->setCenter(Point(source.cols / 2, source.rows / 2));
    wrapper->getLineGroups(lines, linesGrouped, linesGrouped2);
    
    vector<TLine> linesSelected;
    TLine vanishNormal;
    Point2d vanishPoint = wrapper->GetVanishingPoint(lines, linesSelected, vanishNormal, Point(source.cols / 2, source.rows / 2));

    cout << "vanishPoint: " << vanishPoint << endl;
    
    TLine centralLine = TLine(Vec4f(vanishNormal.a, vanishNormal.b, vanishPoint.x, vanishPoint.y));
    
////////////////////////////////////////////////////////////////////////////////
    Mat rgb;
    source.copyTo(rgb);

    for(int i = 0; i < (int)lines.size(); i++)
    {
      drawLine(rgb, lines[i], Scalar(255, 255, 0));
    }

////////////////////////////////////////////////////////////////////////////////

    Mat rgb2;
    source.copyTo(rgb2);
    
    // points, all ellipses    
    for(int i = 0; i < (int)linesGrouped.size(); i++)
    {
      drawLine(rgb2, linesGrouped[i], Scalar(255, 255, 0));
    }
    

////////////////////////////////////////////////////////////////////////////////

    Mat rgb3;
    source.copyTo(rgb3);
    
    drawLine(rgb3, centralLine, Scalar(255, 0, 255), 2);
    
    for(int i = 0; i < (int)linesSelected.size(); i++)
    {
      drawLine(rgb3, linesSelected[i], Scalar(255, 255, 0));
    }     

////////////////////////////////////////////////////////////////////////////////

    Mat rgb4;
    source.copyTo(rgb4);
        
    vector<TLine> finallines;
    CLineClustring* clustering = new CLineClustring(4.0);
    clustering->runLinesClustering(linesSelected, finallines);
      
    for(int i = 0; i < (int)finallines.size(); i++)
    {
      drawLine(rgb4, finallines[i], Scalar(255, 255, 0));
    }     
    
    delete clustering;

////////////////////////////////////////////////////////////////////////////////
    Mat rgb5;
    source.copyTo(rgb5);

    for (int i=0;i != (int)ellipses.size(); ++i) 
    {
      ellipse(rgb5, ellipses.at(i).boundingBox, Scalar(255, 255, 0));
    }
    
////////////////////////////////////////////////////////////////////////////////    
    Mat rgb6;
    source.copyTo(rgb6);

    CRansacEllipse* ransac = new CRansacEllipse(500, 
                                                centralLine, 
                                                vanishPoint, 10, 10, 3.0);
    
    
    vector<TEllipse> eliminatedElipses;
    ransac->eliminateWrongEllipses(ellipses, eliminatedElipses);
    
    for (int i=0;i != (int)eliminatedElipses.size(); ++i)
    { 
      ellipse(rgb6, eliminatedElipses.at(i).boundingBox, Scalar(255, 255, 0), 1);
    }
    
////////////////////////////////////////////////////////////////////////////////    
    Mat rgb7;
    source.copyTo(rgb7);   
    
    // points, all ellipses
    int inliersNumber;
    vector<TEllipse> inliers;   
    inliersNumber = ransac->fitEllipseRANSAC(ellipses, inliers);
    
    cout << "Number of inliers: " << inliersNumber << endl;
    
    for (int i=0;i != (int)inliers.size(); ++i)
    {  
      ellipse(rgb7, inliers.at(i).boundingBox, Scalar(255, 255, 0), 1);     
    }
      
    delete ransac;
 
    ////////////////////////////////////////////////////////////////////////////////    
   

    cout << "-------" << x << "-------" << endl;

    imshow("Output: All lines", rgb);
    imshow("Output: Lines grouped by direction", rgb2);
    imshow("Output: Lines after fitting vanishing point", rgb3);
    imshow("Output: lines after clustering", rgb4);
/*
    imshow("Output: All ellipses", rgb5);
    imshow("Output: Eliminated bad ellipses", rgb6);
    imshow("Output: Ellipses after RANSAC", rgb7);
*/    
    stringstream str1;
    str1 << "all-lines" << x << ".png";
    stringstream str2;
    str2 << "grouped-lines" << x << ".png";
    stringstream str3;
    str3 << "vanishing-lines" << x << ".png";
    stringstream str4;
    str4 << "clustered-lines" << x << ".png";
/*
    stringstream str5;
    str5 << "all-ellipses" << x << ".png";
    stringstream str6;
    str6 << "eliminated-ellipses" << x << ".png";
    stringstream str7;
    str7 << "ransac-ellipses" << x << ".png";
*/    
    imwrite(str1.str(), rgb);
    imwrite(str2.str(), rgb2);
    imwrite(str3.str(), rgb3);
    imwrite(str4.str(), rgb4);
/*
    imwrite(str5.str(), rgb5);
    imwrite(str6.str(), rgb6);
    imwrite(str7.str(), rgb7);
*/    
    uchar c = (uchar)waitKey();

    if(c == 27)
    {
      break;
    }
  }

  delete findEdges;
  delete findEdgels;
  delete fitting;

  return 0;
}