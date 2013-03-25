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

template<typename T>
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
  CRansac(int _numberOfIteration, int _numberOfModelData);

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
  int runRansac(std::vector<T> data, std::vector<T>& inliers);

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
  virtual bool fitRansacModel(T testedEllipse) = 0;
  /**
   * pure virtual method isModel
   *
   * Check what if model data is good model or not.
   *
   * @param     std::vector<TEllipse> modelData         input model data
   *
   * @result    bool                                    correct model data
   */
  virtual bool isModel(std::vector<T> modelEllipses) = 0;

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
  bool getRandomData(std::vector<T> data,
                     std::vector<T>& modelData);

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
  int getInliers(std::vector<T> data,
                 std::vector<T> modelData,
                 std::vector<T>& inliers);

  int numberOfIteration;
  int numberOfModelData;
  
  std::default_random_engine randomGenerator;
};

template <typename T>
CRansac<T>::CRansac(int _numberOfIteration, int _numberOfModelData) :
wrongModels(0),
numberOfIteration(_numberOfIteration),
numberOfModelData(_numberOfModelData)
{
  ;
}

template <typename T>
int CRansac<T>::runRansac(std::vector<T> data, std::vector<T>& inliers)
{
  int bestResult = 0;
  for(int i = 0; i < numberOfIteration; i++)
  {
    std::vector<T> modelData;
    std::vector<T> temporaryInliers;
    
    if(!getRandomData(data, modelData))
      break;
    
    int temporaryResult;
    temporaryResult = getInliers(data, modelData, temporaryInliers);
    
    if(temporaryResult > bestResult)
    {
      bestResult = temporaryResult;
      inliers = temporaryInliers;
    }
  }
  
  return bestResult;
}

template <typename T>
bool CRansac<T>::getRandomData(std::vector<T> data, std::vector<T>& modelData)
{
  if((int) data.size() < 1)
  {
    std::cerr << "ERROR: There is no data in the dataset." << std::endl;
    return false;
  }
  
  std::uniform_int_distribution<int> uniformDistribution(0, data.size()-1);
  std::vector<int> duplicitycontrol;
  bool duplicitTest;
  for(int i = 0; i< numberOfModelData; i++)
  {
    int pos = uniformDistribution(randomGenerator);
    duplicitTest = false;
    for(int j = 0; j < (int)duplicitycontrol.size(); j++)
    {
      if(duplicitycontrol.at(j) == pos)
      {
        i--;
        duplicitTest = true;
      }
    }
    if(!duplicitTest)
    {
      duplicitycontrol.push_back(pos);
      modelData.push_back(data.at(pos));
    }
  }
  
  return true;
}

template <typename T>
int CRansac<T>::getInliers(std::vector<T> data, std::vector<T> modelData,
                           std::vector<T>& inliers)
{
  int counter = 0;
  
  if(!isModel(modelData))
  {
    std::cerr << "WARNING: Cannot fit model from given model data." << std::endl;
    wrongModels++;
    return counter;
  }
  
  for(int i = 0; i < (int)data.size(); i++)
  {
    if(fitRansacModel(data.at(i)))
    {
      counter += data.at(i).score;
      inliers.push_back(data.at(i));
    }
  }
  return counter;
}

#endif // DP_RANSAC_H
