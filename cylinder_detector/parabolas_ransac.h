#ifndef DP_PARABOLA_RANSAC__H
#define DP_PARABOLA_RANSAC__H

#include <vector>

#include "ransac.h"
#include "geometry_fundamentals.h"

class CRansacParabola : public CRansac<TParabola>
{
public:
  
  CRansacParabola(int _numberOfIteration,
                  double _modelDistanceTrashold = 5
  );
  
  virtual ~CRansacParabola(){}      // virtual destructor
  
  int fitParabolaRANSAC(std::vector<TParabola> parabolas,
                       std::vector<TParabola>& inliers);
  
protected:
  
  virtual bool fitRansacModel(TParabola modelParabola);
  
  virtual bool isModel(std::vector<TParabola> modelParabolas);
  
private:
  void recomputeParams(std::vector<TParabola> inliers);
  
  void  getFinalInliers(std::vector<TParabola> parabolas,
                        std::vector<TParabola>& inliers);
  
public:
  double modelDistanceTrashold;
  TLine testLine; 
};

#endif // DP_PARABOLA_RANSAC__H

