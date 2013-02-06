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
   * method getLinesFromEdgePoints
   *
   * Find all edge points aroun the given edge points both directions
   *
   * @param  Mat img                              source image
   * @param  vector<Point> baseEdges              all edge point so far
   * @param  vector<vector<Point> > &newEdges     output - new edges
   * @param  Mat draw                             for debug drawing
   */
  void getEdgesFromEdgePoints(cv::Mat img,
                              std::vector<cv::Point> baseEdges,
                              std::vector<std::vector<cv::Point> > &newEdges,
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
   * @param  Point point  source point
   *
   * @return  Vec2f       vector with horizontal and vertical sobel responce
   */
  cv::Vec2f getSobelResponse(cv::Mat img, cv::Point point);

  /**
   * method binarySearch
   *
   * Provide binary search method to find an edge in interval defined by two
   * points - with lower and heigher values
   *
   * NOTE: Method expected only for edge in the interval!
   *
   * @param  Mat img                            source image
   * @param  Point heigher                      border point with higher value
   * @param  Point lower                        border point with lower value
   *
   * @return  Point                             edge point
   */
  cv::Point binarySearch(cv::Mat img,
                     cv::Point heigher,
                     cv::Point lower,
                     const int edgeTreshold,
                     cv::Mat &draw);

  /**
   * method getNewPoints
   *
   * Locates edge points around the given base edge point
   *
   * @param  Mat img                            source image
   * @param  Point originPoint                  origin point
   * @param  Vec2f shiftVector                  shift vector
   * @param  vector<Point> &newEdges            output - new edges
   * @param  Mat draw                           for debug drawing
   */
  void getNewPoints(cv::Mat img,
                    cv::Point originPoint,
                    cv::Vec2f shiftVector,
                    std::vector<cv::Point> &newEdges,
                    cv::Mat &draw);

  /**
   * method getEdgePoint
   *
   * Provide binary search method to find an edge in interval defined by two
   * points - with lower and heigher values
   *
   * NOTE: Method expected only one edge in the interval!
   *
   * @param  Mat img                            source image
   * @param  Point basePoint                    base point
   * @param  Vec2f shiftVector                  shift vector
   * @param  Point &edge                        output - new edge point
   *
   * @return  bool                               true when edge is found
   */
  bool getEdgePoint(cv::Mat img,
                    cv::Point basePoint,
                    cv::Vec2f shiftVector,
                    cv::Point &edge,
                    cv::Mat &draw);

  int searchStep;
  int searchRadius;
  int searchThreshold;
};





#endif // DP_EDGELS_H
