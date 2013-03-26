#include "parabolas_clustering.h"
#include "standart_deviation.h"

using namespace std;
using namespace cv;

CParabolaClustring::CParabolaClustring()
{
  clusters.clear();     
}

void CParabolaClustring::runParabolasClustering( vector<TParabola> inputParabolas, 
                                               vector<TParabola>& outputParabolas)
{
  transformParabolasToClusters(inputParabolas);
  
  
  //cout << "findMiximumDistance: " << findMiximumDistance() << endl
  
  int positionCluster1;
  int positionCluster2;
  double distanceMin = 0.0;
  double stdDevNew = 0.0;
  double stdDevPrev = 0.0;
  
  while(clusters.size() > 2)
  {
    findMinimumDistancePair(distanceMin, positionCluster1, positionCluster2);
    
    //cout << "first minimum distance: " << distanceMin1 << endl;
    //cout << "second minimum distance: " << distanceMin2 << endl;
    
    if(distanceMin < 30)
    {
      joinClusters(positionCluster1, positionCluster2);
      
      stdDevNew = getStdDevMean();
      
      //cout << "getStdDevSum: " << stdDevNew << endl;
    }
    else
    {
      getResultParabolas(outputParabolas);
      
      joinClusters(positionCluster1, positionCluster2);
      
      //cout << "getStdDevSum: " << getStdDevMean() << endl;
      
      stdDevNew = getStdDevMean();
      
      //cout << "getStdDevSum: " << stdDevNew << endl;
      
      if(stdDevNew > stdDevPrev * 3)
      {       
        break;
      }
    }
    
    stdDevPrev = stdDevNew;
  }
}

void CParabolaClustring::transformParabolasToClusters(vector<TParabola> parabolas)
{
  for(int i = 0; i < (int)parabolas.size(); i++)
  {
    TParabolaCluster cluster;
    cluster.centroidParabola = parabolas.at(i);
    cluster.parabolas.push_back(parabolas.at(i));
    cluster.variation = 0;
    
    clusters.push_back(cluster);
  }
}

void CParabolaClustring::findMinimumDistancePair(double& minDist, 
                                                int& positionCluster1, 
                                                int& positionCluster2)
{
  positionCluster1 = 0;
  positionCluster2 = 0;
  double minDistace = -1.0;
  double currentDistance;
  
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    for(int j = i + 1; j < (int)clusters.size(); j++)
    {
      currentDistance = computeEuclidDistanceParabolaSquared(
        clusters.at(i).centroidParabola, clusters.at(j).centroidParabola);
      
      if(currentDistance < minDistace || minDistace < 0)
      {
        minDistace = currentDistance;
        positionCluster1 = i;
        positionCluster2 = j;
      }
    }    
  }
  
  minDist = minDistace;
}

double CParabolaClustring::getStdDevMean()
{
  double result = 0;
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    result += clusters.at(i).variation;
  }
  return result / clusters.size();
}

double CParabolaClustring::computeEuclidDistanceParabolaSquared(
  TParabola parabola1, TParabola parabola2)
{
  return (parabola1.apex.y-parabola2.apex.y)*(parabola1.apex.y-parabola2.apex.y);
}

void CParabolaClustring::joinClusters(int positionCluster1, int positionCluster2)
{
  
  clusters.at(positionCluster1).parabolas.insert(clusters.at(positionCluster1).parabolas.end(), 
                                                clusters.at(positionCluster2).parabolas.begin(), 
                                                clusters.at(positionCluster2).parabolas.end());
  
  clusters.erase(clusters.begin() + positionCluster2);
  
  clusters.at(positionCluster1).centroidParabola = getCentroidParabola(clusters.at(positionCluster1).parabolas);
  clusters.at(positionCluster1).variation = getStdDev(clusters.at(positionCluster1).parabolas);
}

TParabola CParabolaClustring::getCentroidParabola(vector<TParabola> parabolas)
{
  int sumScore = 0;
  
  double sumCenterY = 0.0;
  double sumParam = 0.0;
  double sumOrigin = 0.0;
  double sumAngle = 0.0;
  
  for(int i = 0; i < (int)parabolas.size(); i++)
  {
    sumScore += parabolas.at(i).score;
    
    sumCenterY += parabolas.at(i).apex.y * parabolas.at(i).score;
    sumParam += parabolas.at(i).param * parabolas.at(i).score;
    sumOrigin += parabolas.at(i).origin * parabolas.at(i).score;
    sumAngle += parabolas.at(i).angle * parabolas.at(i).score;
  }
  
  return TParabola(Point2f(0, sumCenterY / sumScore), sumParam / sumScore, sumAngle / sumScore,  sumOrigin / sumScore);
}

double CParabolaClustring::getStdDev(vector<TParabola> parabolas)
{
  CStdDev* stdDevSecondaryEdgeY = new CStdDev();
  
  for(int i = 0; i < (int)parabolas.size(); i++)
  {
    stdDevSecondaryEdgeY->Push(parabolas.at(i).apex.y);
  }
  
  double variance = stdDevSecondaryEdgeY->Variance();
  
  delete stdDevSecondaryEdgeY;
  
  return variance;
}

void CParabolaClustring::getResultParabolas(vector<TParabola>& parabolas)
{
  parabolas.clear();
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    parabolas.push_back(clusters.at(i).centroidParabola);
  }
}

