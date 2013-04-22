#include "detector.h"

using namespace std;
using namespace cv;

CDetector::CDetector(string parameterFileName)
{
  FileStorage fs(parameterFileName, FileStorage::READ);
  
  fs["adaptiveThreshold"] >> adaptiveThreshold; 
  fs["scanlineStep"] >> scanlineStep;
  fs["bufferSize"] >> bufferSize;
  
  fs["searchStep"] >> searchStep;
  fs["searchRadius"] >> searchRadius;
  fs["searchThreshold"] >> searchThreshold;
  
  fs["numberOfIterationEllipse"] >> numberOfIterationEllipse;
  fs["modelDistanceTrashold"] >> modelDistanceTrashold;
  fs["modelPyramideDistanceTreshold"] >> modelPyramideDistanceTreshold;
  fs["modelAngleTreshold"] >> modelAngleTreshold;
  
  fs["numberOfIterationParabola"] >> numberOfIterationParabola;
  fs["modelDistanceThresholdParameters"] >> modelDistanceThresholdParameters;
  fs["modelDistanceThresholdHorizon"] >> modelDistanceThresholdHorizon;
    
  findEdges = new CFindEdges(scanlineStep, bufferSize, adaptiveThreshold);
  findEdgels = new CFindEdgels(searchStep, searchRadius, searchThreshold);
  
  lineFitting = new CFittingLine();
  ellipseFitting = new CEllipseFitting();
  wrapper = new CWrapper();
  
  clusteringellipse = new CEllipseClustring();
  parabolaRansac = new CRansacParabola(numberOfIterationParabola, modelDistanceThresholdParameters, modelDistanceThresholdHorizon);
  parabolaClustring = new CParabolaClustring();
  supplement = new CSupplement();
  
  fs.release();
}

CDetector::~CDetector()
{
  delete findEdges;
  delete findEdgels;
  delete lineFitting;
  delete ellipseFitting;
  delete parabolaRansac;
  delete parabolaClustring;
  delete clusteringellipse;
  delete wrapper;
  delete supplement;
}

void CDetector::runDetectorTest(string fileName)
{
  Mat source = imread(fileName);
  
  Mat gray;
  cvtColor(source, gray, CV_BGR2GRAY);
  
  // detekce hran na mrizce
  vector<Point2f> edges;
  findEdges->findEdges(gray, edges);
  
  // detekce edgelu
  vector<vector<Point2f> > newEdges;   
  findEdgels->getEdgesFromEdgePoints(gray, edges, newEdges);
  
  // fittovani primek
  vector<TLine> lines;
  lineFitting->fitLines(newEdges, lines);
  
  // 2 main lines direction location
  vector<TLine> linesGrouped;
  vector<TLine> linesGrouped2;
  wrapper->setCenter(Point2f(source.cols / 2, source.rows / 2));
  int index = wrapper->getLineGroups(lines, linesGrouped, linesGrouped2);
  
  // fitting ellipses
  vector<TEllipse> ellipses;
  ellipseFitting->fitEllipsesFromLines(linesGrouped2, ellipses, source.size());
  
  // location of vanishing point and lines correction
  vector<TLine> linesSelected;
  TLine vanishNormal;
  Point2f vanishPoint = wrapper->GetVanishingPoint(linesGrouped, index, linesSelected, vanishNormal, Point2f(source.cols / 2, source.rows / 2));
  
  //cout << "vanishPoint: " << vanishPoint << endl;
  
  TLine centralLine = TLine(Vec4f(vanishNormal.a, vanishNormal.b, vanishPoint.x, vanishPoint.y));
  
  ////////////////////////////////////////////////////////////////////////////////
  
  Mat rgb14;
  source.copyTo(rgb14);
  
  for(int i = 0; i < (int)edges.size(); i++)
  {
    drawPoint(rgb14, edges.at(i), Scalar(255, 255, 0));
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  
  Mat rgb15;
  source.copyTo(rgb15);
  
  for(int i = 0; i < (int)newEdges.size() && i < 1000; i++)
  {
    for(int j = 0; j < (int)newEdges.at(i).size(); j++)
    {
      drawPoint(rgb15, newEdges.at(i).at(j), Scalar(255, 255, 0));
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  Mat rgb;
  source.copyTo(rgb);
  
  for(int i = 0; i < (int)lines.size(); i++)
  {
    drawLine(rgb, lines.at(i), Scalar(255, 255, 0));
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  Mat rgb2;
  source.copyTo(rgb2);
  
  // points, all ellipses    
  for(int i = 0; i < (int)linesGrouped.size(); i++)
  {
    drawLine(rgb2, linesGrouped.at(i), Scalar(255, 255, 0));
  }
  
  // points, all ellipses    
  for(int i = 0; i < (int)linesGrouped2.size(); i++)
  {
    drawLine(rgb2, linesGrouped2.at(i), Scalar(0, 255, 255));
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  
  Mat rgb3;
  source.copyTo(rgb3);
  
  for(int i = 0; i < (int)linesSelected.size(); i++)
  {
    drawLine(rgb3, linesSelected.at(i), Scalar(255, 255, 0));
  }     
  
  ////////////////////////////////////////////////////////////////////////////////
  
  Mat rgb5;
  source.copyTo(rgb5);
  
  for (int i=0;i < (int)ellipses.size(); ++i) 
  {
    ellipse(rgb5, ellipses.at(i).boundingBox, Scalar(255, 255, 0));
    drawPoint(rgb5, ellipses.at(i).mainEdge, Scalar(0, 0, 255));
  }
  
  ////////////////////////////////////////////////////////////////////////////////    
  Mat rgb7;
  source.copyTo(rgb7);   
  
  // points, all ellipses
  //int inliersNumber;
    
  CRansacEllipse* ransac = new CRansacEllipse(numberOfIterationEllipse, vanishPoint, modelDistanceTrashold, modelPyramideDistanceTreshold, modelAngleTreshold);
  vector<TEllipse> inliers;  
  TLine cylinderCentralLine, pyramideLine;
  ransac->fitEllipseRANSAC(ellipses, inliers, cylinderCentralLine, pyramideLine);
  
  //cout << "Number of inliers: " << inliersNumber << endl;
  
  drawLine(rgb7, cylinderCentralLine, Scalar(0, 255, 255));
  drawLine(rgb7, pyramideLine, Scalar(0, 255, 255));
  drawLine(rgb7, ransac->elipseMainAxeLine, Scalar(0, 255, 255));
  
  for (int i=0;i != (int)inliers.size(); ++i)
  {  
    ellipse(rgb7, inliers.at(i).boundingBox, Scalar(255, 255, 0), 1);  
    drawPoint(rgb7, inliers.at(i).mainEdge, Scalar(0, 0, 255));
    drawPoint(rgb7, inliers.at(i).secondaryEdge, Scalar(0, 0, 255));
  }
  
  //////////////////////////////////////////////////////////////////////////////// 
  
  Mat rgb8;
  source.copyTo(rgb8);   
  
  vector<TEllipse> finalEllipses;
  CEllipseClustring* clusteringellipse = new CEllipseClustring();
  clusteringellipse->runEllipsesClustering(inliers, finalEllipses);
  
  for(int i = 0; i < (int)finalEllipses.size(); i++)
  {
    ellipse(rgb8, finalEllipses.at(i).boundingBox, Scalar(255, 255, 0));
  }    
  
  //////////////////////////////////////////////////////////////////////////////// 
  
  Mat rgb4;
  source.copyTo(rgb4);
  
  
  vector<TLine> finallines;
  CLineClustring* clustering = new CLineClustring(cylinderCentralLine, pyramideLine, Point2f(source.cols / 2, source.rows / 2));
  
  //cout << "number of selectedLines: " << linesSelected.size() << endl; 
  
  //drawLine(rgb4, TLine(Vec4f(cylinderCentralLine.a, cylinderCentralLine.b, source.cols / 2, source.rows / 2)), Scalar(255, 0, 255));
  
  clustering->runLinesClustering(linesSelected, finallines);
  
  //cout << "number of final Lines: " << finallines.size() << endl; 
  
  for(int i = 0; i < (int)finallines.size(); i++)
  {
    drawLine(rgb4, finallines[i], Scalar(255, 255, 0));
  }     
  
  ////////////////////////////////////////////////////////////////////////////////   
  
  Mat rgb9;
  source.copyTo(rgb9);
  
  CParabolaFitting* parabolaFitting = new CParabolaFitting(cylinderCentralLine);
  
  vector<TParabola> parabolas;
  parabolaFitting->fitParabolas(linesGrouped2, parabolas);
  
  for(int i = 0; i < (int)parabolas.size(); i++)
  {
    parabolaFitting->drawParabola(rgb9, parabolas.at(i), Scalar(255, 255, 0));
  }  
  
  ////////////////////////////////////////////////////////////////////////////////   
  
  Mat rgb10;
  source.copyTo(rgb10);
  
  Mat test1(3000, 3000, CV_8UC3);
  test1.setTo(0);
  
  vector<TParabola> inliersParabola;
  parabolaRansac->fitParabolaRANSAC(parabolas, inliersParabola);
  
  //cout << "Number of inliers: " << inliersNumber << endl;
  
  Point2f referencePoint = parabolaRansac->intersection;
  
  //cout << "reference Point: " << parabolaFitting->transformPointBack(referencePoint) << endl;
  
  for(int i = 0; i < (int)inliersParabola.size(); i++)
  {
    parabolaFitting->drawParabola(rgb10, inliersParabola.at(i), Scalar(255, 255, 0));
  }
  
  drawPoint(rgb10, parabolaFitting->transformPointBack(referencePoint), Scalar(0, 0, 255));
  
  ////////////////////////////////////////////////////////////////////////////////   
  
  Mat rgb11;
  source.copyTo(rgb11);
    
  vector<TParabola> clusteredParabola;
  parabolaClustring->runParabolasClustering(inliersParabola, clusteredParabola);
  
  vector<TParabola> clusteredFinalParabola;
  
  referencePoint = parabolaClustring->recomputeClusteredParabolas(clusteredParabola, clusteredFinalParabola);
  
  //cout << "referencePoint: " << referencePoint << endl;
  
  for(int i = 0; i < (int)clusteredFinalParabola.size(); i++)
  {
    parabolaFitting->drawParabola(rgb11, clusteredFinalParabola.at(i), Scalar(255, 255, 0));
  }
  
  drawPoint(rgb11, parabolaFitting->transformPointBack(referencePoint), Scalar(0, 0, 255));
  
  
  ////////////////////////////////////////////////////////////////////////////////     
  
  Mat rgb12;
  source.copyTo(rgb12);
  
  vector<TParabola> supplementParabola;
  
  sort(clusteredFinalParabola.begin(), clusteredFinalParabola.end(), [](TParabola p1, TParabola p2)
  { 
    return p1.apex.y < p2.apex.y;
  });
  
  if(!isnan(referencePoint.x))
  {
    supplement->runSupplement(clusteredFinalParabola, supplementParabola, referencePoint);
  }
  
  for(int i = 0; i < (int)supplementParabola.size(); i++)
  {
    parabolaFitting->drawParabola(rgb12, supplementParabola.at(i), Scalar(255, 255, 0));
  }
  
  /*
   vector<TParabola> clusteredParabola2;
   for(int i = 0; i < (int)clusteredParabola.size(); i++)
   {
     clusteredParabola2.push_back(clusteredParabola.at(i));
   }
   */
  /*
   for(int i = 0; i < (int)clusteredParabola2.size(); i++)
   {
     parabolaFitting->drawParabola(rgb12, clusteredParabola2.at(i), Scalar(255, 0, 255));
   }
   */
  
  ////////////////////////////////////////////////////////////////////////////////     

  Mat rgb13;
  source.copyTo(rgb13);
  
  CFindGrid* findGrid = new CFindGrid(parabolaFitting->transformationMatrix, parabolaFitting->transformationMatrixInverse, referencePoint, pyramideLine, vanishPoint);
  
  vector<TParabola> middleParabolas;
  
  sort(supplementParabola.begin(), supplementParabola.end(), [](TParabola a, TParabola b){return a.apex.y > b.apex.y;});
  
  if(!isnan(referencePoint.x))
  {
    findGrid->findMiddleParabolas(supplementParabola, middleParabolas);
  }
  
  for(int i = 0; i < (int)middleParabolas.size(); i++)
  {
    parabolaFitting->drawParabola(rgb13, middleParabolas.at(i), Scalar(0, 255, 255));
  } 
    
  vector<TLine> middleLines;
  
  sort(finallines.begin(), finallines.end(), [&](TLine l1, TLine l2)
  {
    double angle1 = getSmallerIntersectionAngle(l1, pyramideLine);
    double angle2 = getSmallerIntersectionAngle(l2, pyramideLine);
    
    return angle1 > angle2;
  });
  
  findGrid->findMiddleLines(finallines, middleLines);
  for(int i = 0; i < (int)middleLines.size(); i++)
  {
    drawLine(rgb13, middleLines.at(i), Scalar(0, 255, 255));
  }
 
  
  vector<vector<Point2f> > gridPoints;
  findGrid->findGrid(finallines, supplementParabola, gridPoints);
  for(int i = 0; i < (int)gridPoints.size(); i++)
  {
    for(int j = 0; j < (int)gridPoints.at(i).size(); j++)
    {
      drawPoint(rgb13, gridPoints.at(i).at(j), Scalar(0, 0, 255));
    }
  } 
  
  //////////////////////////////////////////////////////////////////////////////
  
  delete ransac;
  delete clustering;
  delete parabolaFitting;
  
  //////////////////////////////////////////////////////////////////////////////
  
  stringstream str1;
  str1 << "01-all-lines.png";
  stringstream str2;
  str2 << "02-grouped-lines.png";
  stringstream str3;
  str3 << "03-vanishing-lines.png";
  stringstream str4;
  str4 << "04-clustered-lines.png";
  stringstream str5;
  str5 << "05-all-ellipses.png";
  stringstream str7;
  str7 << "07-ransac-ellipses.png";
  stringstream str8;
  str8 << "08-clustering-ellipses.png";
  stringstream str9;
  str9 << "09-parabolas-all.png";
  stringstream str10;
  str10 << "10-parabolas-ransac.png";
  stringstream str11;
  str11 << "11-parabolas-clustering.png";
  stringstream str12;
  str12 << "12-parabolas-supplement.png";
  stringstream str13;
  str13 << "13-grid.png";
  stringstream str14;
  str12 << "14-edges.png";
  stringstream str15;
  str13 << "15-edgels.png";
  
  //////////////////////////////////////////////////////////////////////////////
  /*
  imwrite(str1.str(), rgb);
  imwrite(str2.str(), rgb2);
  imwrite(str3.str(), rgb3);
  imwrite(str4.str(), rgb4);
  imwrite(str5.str(), rgb5);
  imwrite(str7.str(), rgb7);
  imwrite(str8.str(), rgb8);
  imwrite(str9.str(), rgb9);
  imwrite(str10.str(), rgb10);
  imwrite(str11.str(), rgb11);
  imwrite(str12.str(), rgb12);
  imwrite(str13.str(), rgb13);
  imwrite(str14.str(), rgb14);
  imwrite(str15.str(), rgb15);
  */
  //////////////////////////////////////////////////////////////////////////////
  
  //imshow("Output 14: Edges", rgb14);
  //imshow("Output 15: Edgels", rgb15);  
  //imshow("Output 1: All lines", rgb);
  //imshow("Output 2: Lines grouped by direction", rgb2);
  //imshow("Output 3: Lines after fitting vanishing point", rgb3);
  //imshow("Output 4: lines after clustering", rgb4);
  imshow("Output 5: All ellipses", rgb5);
  imshow("Output 7: Ellipses after RANSAC", rgb7);
  imshow("Output 8: Ellipses after clustering", rgb8);
  //imshow("Output 9: Parabolas", rgb9);
  //imshow("Output 10: Parabolas ransac", rgb10);
  //imshow("Output 11: Parabolas clustering", rgb11);  
  //imshow("Output 12: Parabolas supplement", rgb12);  
  //imshow("Output 13: Grid", rgb13);
  
}
