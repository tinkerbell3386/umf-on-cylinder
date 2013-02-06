#include "ransac.h"

using namespace std;

CRansac::CRansac(int _numberOfModelData, int _numberOfIteration) :
        wrongModels(0),
        numberOfModelData(_numberOfModelData),
        numberOfIteration(_numberOfIteration)
{}


int CRansac::runRansac(vector<TEllipse> data, vector<TEllipse>& inliers)
{
  int bestResult = 0;
  for(int i = 0; i < numberOfIteration; i++)
  {
    vector<TEllipse> modelData;
    vector<TEllipse> temporaryInliers;

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

bool CRansac::getRandomData(vector<TEllipse> data, vector<TEllipse>& modelData)
{
  if((int) data.size() < numberOfModelData)
  {
    cerr << "ERROR: There is not enough data in the dataset. Needed at least "
    << numberOfModelData << " but got " << data.size() << "." << endl;
    return false;
  }

  uniform_int_distribution<int> uniformDistribution(0, data.size()-1);

  for(int i = 0; i < numberOfModelData; i++)
  {
    modelData.push_back(data.at(uniformDistribution(randomGenerator)));
  }

  return true;
}

int CRansac::getInliers(vector<TEllipse> data, vector<TEllipse> modelData,
                        vector<TEllipse>& inliers)
{
  int counter = 0;

  if(!isModel(modelData))
  {
    cerr << "WARNING: Cannot fit model from given model data." << endl;
    wrongModels++;
    return counter;
  }

  for(int i = 0; i < (int)data.size(); i++)
  {
    if(fitRansacModel(modelData, data.at(i)))
    {
      counter++;
      inliers.push_back(data.at(i));
    }
  }
  return counter;
}
