//
// Created by zlc on 2020/1/18.
//

#include "System.h"

#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace pangolin;

System::System(const std::string& sConfig_file_) : bStart_backend(true)
{
    string sConfig_file = sConfig_file_ + "euroc_config.yaml";

    cout << "1 System() sConfig_file: " << sConfig_file << endl;

    //readParameters(sConfig_file);
}