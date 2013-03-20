#ifndef DP_LINE_RANSAC__H
#define DP_LINE_RANSAC__H

#include <vector>

#include "ransac.h"
#include "geometry_fundamentals.h"

/**
 * Class CRansacLine
 *
 * Finds best line from given dataset by RANSAC algorithm and by least square
 * method.
 *
 * Inherits from generic CRansac. Provides line model.
 *
 * @author Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
 */
class CRansacEllipse : public CRansac
{
public:

  /**
   * Constructor CRansacLine
   *
   * Setup basic parameters
   *
   * @param     int _numberOfIteration          number of iteration
   * @param     int _modelTrashold              treshold for line model
   */
  CRansacEllipse(int _numberOfIteration,
                 cv::Point2f _vanishingPoint,
                 int _modelDistanceTrashold  = 20,
                 int _modelPyramideDistanceTreshold  = 20,
                 double _modelAngleTreshold = 5.0
                );

  virtual ~CRansacEllipse(){}      // virtual destructor

  /**
   * Constructor fitLineRANSAC
   *
   * Run RANSAC algorithm for finding line and fit line from computed inliers by
   * least square error method.
   *
   * @param     std::vector<TEllipse> ellipses          input dataset
   * @param     std::vector<TEllipse>& inliers          output inliers
   *
   * @result    bool                                    number of inliers
   */
  int fitEllipseRANSAC(std::vector<TEllipse> ellipses,
                       std::vector<TEllipse>& inliers,
                       TLine& finalCentralLine,
                       TLine& finalBorderLine);
  
protected:

  /**
   * Method fitRansacModel
   *
   * Define model for RANSAC. The criterium is number of ellipses in interval
   *
   * @param     TEllipse testedEllipse      tested ellipse
   * @param     TEllipse modelEllipse       model ellipse
   *
   * @result    bool                       true when model ellipse fits
   */
  virtual bool fitRansacModel(TEllipse modelEllipse);

  /**
   * Constructor fitRansacModel
   *
   * Check if data can fit the model.
   *
   * @param     TEllipse modelEllipses     model ellipses
   *
   * @result    bool                                    correct model data
   */
  virtual bool isModel(TEllipse modelEllipses);

private:
  void recomputeParams(std::vector<TEllipse> inliers);
  
  void  getFinalInliers(std::vector<TEllipse> ellipses,
                        std::vector<TEllipse>& inliers);
  
public:
  TLine centralLine;
  cv::Point2f vanishingPoint;
  
  int modelDistanceTrashold;    // treshold for line centers criterium
  int modelPyramideDistanceTreshold;
  double modelAngleTreshold;

  TLine distanceLine;
  TLine pyramideTreeShapedLine;
  TLine elipseMainAxeLine;
};

#endif // DP_LINE_RANSAC__H

