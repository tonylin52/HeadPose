#include "HeadPose.h"
#include "HeadPoseExtern.h"

#include <iostream>
#include <unistd.h>
#include <string>
#include <fstream>
#include <streambuf>

using namespace std;
using namespace cv;

int main() {	 
         RunPupil();

         while (true) {
          std::string name;
          std::getline (std::cin,name);
          std::cout << "Main Operation (!)" << name << "\n";
     }
}