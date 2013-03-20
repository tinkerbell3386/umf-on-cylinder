#ifndef DP_PARABOLA_FITTING_H
#define DP_PARABOLA_FITTING_H

#include "geometry_fundamentals.h"

/**
 * class CParabolaFitting
 * 
 * This class can fit parabolas by Least Square Error Fitting.
 * It needs set of points and central line.
 * Central line defines rotation angle and translation in the direction of X, also X coord of apex.
 * 
 * Basic steps:
 * - Transform parabola that apex lies on the Y axe (apex X coord is 0) and Y is parabola axis.
 *   So it can fulfil equation: y = p*x*x + y0
 * - Use Least Square Error to fit this transformed parabola -> find p and y0
 * - To vizualize this parabola make inverse transformation
 */
class CParabolaFitting
{
public:
  CParabolaFitting(TLine centralLine);
  ~CParabolaFitting(){}

  bool fitParabola(std::vector<cv::Point2f> points, TParabola& parabola, cv::Mat draw);
  void drawParabola(cv::Mat& img, TParabola parabola, cv::Scalar color, 
                    int thickness = 1);
  
  cv::Point2f transformPointBack(cv::Point2f input);
  
  void transformPointsToY(std::vector<cv::Point2f> input, 
                          std::vector<cv::Point2f>& output);
  
  void transformPointsBack(std::vector<cv::Point2f> input, 
                           std::vector<cv::Point2f>& output);
  
private:
  void getAngleAndOrigin(TLine line);
  void setupTrasfomationMatrices();
  
  cv::Mat transformationMatrix;
  cv::Mat transformationMatrixInverse;
  cv::Point2f origin;
  double angle;
};

#endif // DP_PARABOLA_FITTING_H
