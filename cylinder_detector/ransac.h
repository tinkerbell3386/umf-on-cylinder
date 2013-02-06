#ifndef DP_RANSAC_H
#define DP_RANSAC_H

#include <vector>
#include <random>

#include "geometry_fundamentals.h"

/**
 * Class CRansac
 *
 * Partly generic class, which implements RANSAC algorithm. User has to provide
 * correct RANSAC model and number of iteration.
 *
 * @author Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
 */
class CRansac
{
public:

  /**
   * Constructor CRansac
   *
   * Sets number of iteration of RANSAC algorithm and number of data defines the
   * data.
   *
   * @param     int _numberOfModelData          number of data fits the model
   * @param     int _numberOfIteration          number of iteration
   */
  CRansac(int _numberOfModelData, int _numberOfIteration);

  ~CRansac(){}

  /**
   * Constructor runRansac
   *
   * Run RANSAC algorithm
   *
   * @param     std::vector<TEllipse> data              input dataset
   * @param     std::vector<TEllipse>& inliers          output inliers
   *
   * @result    int                                     number of inliers
   */
  int runRansac(std::vector<TEllipse> data, std::vector<TEllipse>& inliers);

  int wrongModels;      // counts number of wrong models

protected:

  /**
   * pure virtual method fitRansacModel
   *
   * Defines RANSAC model. Has to be implemented by succesor.
   *
   * @param     std::vector<TEllipse> modelData         selected model data
   * @param     TEllipse x                              tested data
   *
   * @result    bool                                    true if "x" fits model
   */
  virtual bool fitRansacModel(std::vector<TEllipse> modelData,
                              TEllipse x) = 0;
  /**
   * pure virtual method isModel
   *
   * Check what if model data is good model or not.
   *
   * @param     std::vector<TEllipse> modelData         input model data
   *
   * @result    bool                                    correct model data
   */
  virtual bool isModel(std::vector<TEllipse> modelData) = 0;

private:

  /**
   * method getRandomData
   *
   * Randomly select model data from input dataset.
   *
   * @param     std::vector<TEllipse> data              input dataset
   * @param     std::vector<TEllipse>& modelData        output select data
   *
   * @result    bool                                    true if select correctly
   */
  bool getRandomData(std::vector<TEllipse> data,
                      std::vector<TEllipse>& modelData);

  /**
   * method getInliers
   *
   * Find inliers from dataset by RANSAC algorithm.
   *
   * @param     std::vector<TEllipse> data              input dataset
   * @param     std::vector<TEllipse> modelData         input model data
   * @param     std::vector<TEllipse>& inliersFlags     output inliers
   *
   * @result    int                                     number of inliers
   */
  int getInliers(std::vector<TEllipse> data,
                 std::vector<TEllipse> modelData,
                 std::vector<TEllipse>& inliers);

  int numberOfModelData;
  int numberOfIteration;
  std::default_random_engine randomGenerator;
};

#endif // DP_RANSAC_H
