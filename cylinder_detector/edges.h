#ifndef DP_EDGES_H
#define DP_EDGES_H

#include <opencv2/opencv.hpp>
#include <vector>

class CFindEdges
{
public:

  CFindEdges(int _scanlineStep = 20, int _bufferSize = 15,
             int _adaptiveThreshold = 25);
  ~CFindEdges(){}

  /**
   * method findEdges
   *
   * Locates edges on the scan lines
   *
   * @param  Mat img                      source image
   * @param  vector<Point> &edges         output - vector with edges
   * @param  const int scanlineStep       step between scan lines
   */
  void findEdges(cv::Mat img, std::vector<cv::Point>& edges);

  int adaptiveThreshold;
  int scanlineStep;

private:
  /**
   * method scanLine
   *
   * Locates edges on the single scan line
   *
   * @param  Mat line                     line, column vector
   * @param  vector<Point> &edges         indexes of founded edges
   */
  void scanLine(cv::Mat line, std::vector<int>& edges);


  int bufferSize;
};

#endif // DP_EDGES_H
