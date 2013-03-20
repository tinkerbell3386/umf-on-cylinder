#include "ransac.h"

using namespace std;

CRansac::CRansac(int _numberOfIteration) :
        wrongModels(0),
        numberOfIteration(_numberOfIteration)
{}


int CRansac::runRansac(vector<TEllipse> data, vector<TEllipse>& inliers)
{
  int bestResult = 0;
  for(int i = 0; i < numberOfIteration; i++)
  {
    TEllipse modelData;
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

bool CRansac::getRandomData(vector<TEllipse> data, TEllipse& modelData)
{
  if((int) data.size() < 1)
  {
    cerr << "ERROR: There is no data in the dataset." << endl;
    return false;
  }

  uniform_int_distribution<int> uniformDistribution(0, data.size()-1);

  modelData = (data.at(uniformDistribution(randomGenerator)));

  return true;
}

int CRansac::getInliers(vector<TEllipse> data, TEllipse modelData,
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
    if(fitRansacModel(data.at(i)))
    {
      counter += data.at(i).score;
      inliers.push_back(data.at(i));
    }
  }
  return counter;
}
