# include "detector.h"

using namespace cv;
using namespace std;

bool fexists(const char *filename)
{
  ifstream ifile(filename);
  return ifile.good();
}

int main(int argc, char** argv)
{
  CDetector* detector = new CDetector("default.yml");
  
  for(int x = 1; x < 1000; x++) {
    
    stringstream str;
    str << "../data/images7/" << x << "data.jpg";
    
    if(!fexists(str.str().c_str()))
    {
      break;
    }
    
    detector->runDetectorTest(str.str());
    
    uchar c = (uchar)waitKey();
    
    cout << "--------------" << x << "--------------" << endl;
    
    if(c == 27)
    {
      break;
    }
  }
  
  delete detector;
  
  return 0;
}
