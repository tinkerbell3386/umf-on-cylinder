#include <fstream>
#include <iostream>
#include <string>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

# include "detector.h"

using namespace cv;
using namespace std;

bool fexists(const char *filename)
{
  ifstream ifile(filename);
  return ifile.good();
}

bool readDirectory(string dirName, vector<string>& files)
{
  files.clear();

  string filepath;
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  
  dp = opendir( dirName.c_str() );
  if (dp == NULL)
  {
    cout << "Error(" << errno << ") opening " << dirName << endl;
    return false;
  }

  while ((dirp = readdir( dp )))
  {
    filepath = dirName + "/" + dirp->d_name;
    
    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat( filepath.c_str(), &filestat )) continue;
    if (S_ISDIR( filestat.st_mode ))         continue;
    
    // Endeavor to read a single number from the file and display it
    string ending = filepath.substr(filepath.find_last_of(".") + 1);
    if(ending == "png" || ending == "bmp" || ending == "jpg")
    {
      files.push_back(filepath);
    }    
  }

  closedir( dp );
  
  return true;
}

int main(int argc, char** argv)
{  
  if(argc < 3)
  {
    cerr << "ERROR: Some parameter is missing" << endl;
    cerr << "Using: ./testDetector CONFIGURATION_FILE_PATH DIRECTORY_WITH_IMAGES [SHOW IMAGE SWITCH: true/false]" << endl;
    return 1;
  }
    
  if(!fexists(argv[1]))
  {
    cerr << "ERROR: Cannot read configuration file" << endl;
    return 1;
  }
  
  bool showImage = false;
  if(argc > 4 && string(argv[3]) == "true")
  {
    showImage = true;
  }
  
  vector<string> files;
  if(!readDirectory(argv[2], files))
  {
    return 1;
  }
  
  CDetector* detector = new CDetector(argv[1]);
  
  uchar c = 0;
  for(int x = 0; x < (int)files.size(); x++) {
    
    cout << "-------------- start file no. " << x << " [" << files.at(x) << "]--------------" << endl;
    
    detector->runDetectorTest(files.at(x), showImage);
    
    if(showImage)
    {
      c = (uchar)waitKey();
    }
    
    cout << "-------------- end file no. " << x << " [" << files.at(x) << "]--------------" << endl;
    
    if(c == 27)
    {
      break;
    }
  }
  
  delete detector;
  
  return 0;
}
