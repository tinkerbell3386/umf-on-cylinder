#ifndef DP_PARABOLA_FITTING_H
#define DP_PARABOLA_FITTING_H

#include "geometry_fundamentals.h"
 
 /**
  * Třída CParabolaFitting
  * 
  * Tato třída fituje paraboly pomcí metody least square error. K fitování potřebuje centrální osu válce a množinu zdrojových bodů.
  * Osa válce definuje rotaci a translaci v ose X, zároveň umožňuje nastavit X-ovou souřadnici bodu X na 0 a tímto velmi zjednodoušit fitování.
  * 
  * Kroky fitování:
  * - transformuje parabolu tak, že vrchol leží na osa Y a zároveň osa Y je i isa paraboly
  *   Takže parabola splňuje podmínky rovnice: y = p*x*x + y0, kde hledáme p a y0
  * - Parametry p a Y0 hledáme pomocí Metody nejmenších čtverců
  * - Pro vizualizace korektní paraboly je nutné výslednou parabolu transformovat zpět
  */
class CParabolaFitting
{
public:

  /**
   * Konstruktor CParabolaFitting
   * 
   * @param TLine centralLine           osa válce
   */
  CParabolaFitting(TLine centralLine);
  
  ~CParabolaFitting(){}
  
  
  bool fitParabola(std::vector<cv::Point2f> points, TParabola& parabola, cv::Mat draw);
  void drawParabola(cv::Mat& img, TParabola parabola, cv::Scalar color, 
                    int thickness = 1);
  
  cv::Point2f transformPointBack(cv::Point2f input);
  
  void transformPointsToY(std::vector<cv::Point2f> input, 
                          std::vector<cv::Point2f>& output);
  
  void transformPointsBack(std::vector<cv::Point2f> input, 
                           std::vector<cv::Point2f>& output);
  
private:
  void getAngleAndOrigin(TLine line);
  void setupTrasfomationMatrices();
  
  cv::Mat transformationMatrix;
  cv::Mat transformationMatrixInverse;
  cv::Point2f origin;
  double angle;
};

#endif // DP_PARABOLA_FITTING_H
