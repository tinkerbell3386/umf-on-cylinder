#ifndef DP_EDGELS_H
#define DP_EDGELS_H

#include <opencv2/opencv.hpp>
#include <vector>

class CFindEdgels
{
public:

  CFindEdgels(int _searchStep = 10, int _searchRadius = 10,
              int _searchThreshold = 25);

  ~CFindEdgels(){}

  /**
   * method getLinesFromEdgePoint2fs
   *
   * Find all edge points aroun the given edge points both directions
   *
   * @param  Mat img                              source image
   * @param  vector<Point2f> baseEdges              all edge point so far
   * @param  vector<vector<Point2f> > &newEdges     output - new edges
   * @param  Mat draw                             for debug drawing
   */
  void getEdgesFromEdgePoints(cv::Mat img,
                              std::vector<cv::Point2f> baseEdges,
                              std::vector<std::vector<cv::Point2f> > &newEdges,
                              cv::Mat &draw);

private:

  /**
   * method getSobelResponse
   *
   * Compute Sobbel responce from cross.
   * Vertical: bottom and top pixels
   * Horizontal: left and right pixels
   *
   * @param  Mat img      source image
   * @param  Point2f point  source point
   *
   * @return  Vec2f       vector with horizontal and vertical sobel responce
   */
  cv::Vec2f getSobelResponse(cv::Mat img, cv::Point2f point);

  /**
   * method binarySearch
   *
   * Provide binary search method to find an edge in interval defined by two
   * points - with lower and heigher values
   *
   * NOTE: Method expected only for edge in the interval!
   *
   * @param  Mat img                            source image
   * @param  Point2f heigher                      border point with higher value
   * @param  Point2f lower                        border point with lower value
   *
   * @return  Point2f                             edge point
   */
  cv::Point2f binarySearch(cv::Mat img,
                     cv::Point2f heigher,
                     cv::Point2f lower,
                     const int edgeTreshold,
                     cv::Mat &draw);

  /**
   * method getNewPionts
   *
   * Locates edge points around the given base edge point
   *
   * @param  Mat img                            source image
   * @param  Point2f originPoint                  origin point
   * @param  Vec2f shiftVector                  shift vector
   * @param  vector<Point2f> &newEdges            output - new edges
   * @param  Mat draw                           for debug drawing
   */
  void getNewPoints(cv::Mat img,
                    cv::Point2f originPoint,
                    cv::Vec2f shiftVector,
                    std::vector<cv::Point2f> &newEdges,
                    cv::Mat &draw, cv::Scalar color);

  /**
   * method getEdgePoint
   *
   * Provide binary search method to find an edge in interval defined by two
   * points - with lower and heigher values
   *
   * NOTE: Method expected only one edge in the interval!
   *
   * @param  Mat img                            source image
   * @param  Point2f basePoint                    base point
   * @param  Vec2f shiftVector                  shift vector
   * @param  Point2f &edge                        output - new edge point
   *
   * @return  bool                               true when edge is found
   */
  bool getEdgePoint(cv::Mat img,
                    cv::Point2f basePoint,
                    cv::Vec2f shiftVector,
                    cv::Point2f &edge,
                    cv::Mat &draw);

  int searchStep;
  int searchRadius;
  int searchThreshold;
};





#endif // DP_EDGELS_H
