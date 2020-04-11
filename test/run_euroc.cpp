//
// Created by zlc on 2020/1/18.
//

#include <unistd.h> // unistd.h是unix std的意思，是POSIX标准定义的unix类系统定义符号常量的头文件，
                    // 包含了许多UNIX系统服务的函数原型，
                    // 例如read函数、write函数和getpid函数。 unistd.h在unix中类似于window中的windows.h。
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <iomanip>  // 作用: 从文本中提取指定个数的字符串, 并在串数组末尾添加一个空字符.
// 是I/O流控制头文件,就像C里面的格式化输出一样

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;  // 延时时间
string sData_path = "/home/zlc/VIOFiles/MH_05_difficult/mav0/";
string sConfig_path = "../config";

std::shared_ptr<System> pSystem;    // System类型的智能指针


// 发布IMU数据
void PubImuData()
{
    string sImu_data_file = sConfig_path + "MH_05_imu0.txt";   // 读取MH_05_imu0数据集的IMU数据
    cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;

    ifstream fsImu; // 准备打开文件
    fsImu.open(sImu_data_file.c_str());
    // 将C++的string转化为C的字符串数组,c_str()生成一个constchar*指针,指向字符串的首地址。

    // 打不开报错
    if ( !fsImu.is_open() )
    {
        cerr << "Fail to open imu file!" << sImu_data_file << endl;
        return;
    }

    std::string sImu_line;      // Imu数据文件中的一行
    double dStampNSec = 0.0;
    Vector3d vAcc;              // Imu数据文件中的加速度数据
    Vector3d vGyr;              // Imu数据文件中的陀螺仪数据

    while ( std::getline(fsImu, sImu_line) && !sImu_line.empty() )  // read imu data, 读取字符流
    {
        std::istringstream ssImuData(sImu_line);
        // 1403638518097829376  0.00069813170079773186  0.018849555921538759  0.07819075048934597  9.0874956666666655 -0.057205458333333334 -3.8245934999999998
        ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();

        pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);      // 发布IMU数据

        usleep(5000 * nDelayTimes);  // 5ms * 2 = 10ms
    }

    fsImu.close();  // 关闭文件
}


// 发布图像数据
void PubImageData()
{
    string sImage_file = sConfig_path + "MH_05_cam0.txt";
    cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

    ifstream fsImage;
    fsImage.open(sImage_file);

    if ( !fsImage.is_open() )
    {
        cerr << "Fail to open image file!" << sImage_file << endl;
        return;
    }

    std::string sImage_line;
    double dStampNSec;          // 每一帧图像的时间
    string sImgFileName;        // 图像数据

    // cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
    while ( std::getline(fsImage, sImage_line) && !sImage_line.empty() )
    {
        std::istringstream ssImuData(sImage_line);
        // 1403638518077829376  1403638518077829376.png
        ssImuData >> dStampNSec >> sImgFileName;
        string imagePath = sData_path + "cam0/data/" + sImgFileName;    // 去找真正的图像照片, 必须待完整的扩展名, 即后面的png

        Mat img = imread(imagePath.c_str(), 0);
        // 参数2 flags, 一个读取标记，用于选择读取图片的方式，默认值为IMREAD_COLOR，flag值的设定与用什么颜色格式读取图片有关
        if ( img.empty() )
        {
            cerr << "image is empty! path: " << imagePath << endl;
        }

        pSystem->PubImageData(dStampNSec / 1e9, img);
        // cv::imshow("SOURCE IMAGE", img);
        // cv::waitKey(0);

        usleep(50000*nDelayTimes);  // 50ms * 2 = 100ms
    }

    fsImage.close();
}

// 兼容性代码
#ifdef __APPLE__
// support for MacOS
void DrawIMGandGLinMainThrd(){
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	pSystem->InitDrawGL();
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		//pSystem->PubImageData(dStampNSec / 1e9, img);
		cv::Mat show_img;
		cv::cvtColor(img, show_img, CV_GRAY2RGB);
		if (SHOW_TRACK)
		{
			for (unsigned int j = 0; j < pSystem->trackerData[0].cur_pts.size(); j++)
			{
				double len = min(1.0, 1.0 *  pSystem->trackerData[0].track_cnt[j] / WINDOW_SIZE);
				cv::circle(show_img,  pSystem->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}

			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
		  // cv::waitKey(1);
		}

		pSystem->DrawGLFrame();
		usleep(50000*nDelayTimes);
	}
	fsImage.close();

}
#endif

int main(int argc, char* *argv)
{
    if ( argc != 3 )
    {
        cerr << "./run_eruroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n"
             << "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05-mav0 ../config/" << endl;
    }
    sData_path = argv[1];
    sConfig_path = argv[2];

    pSystem.reset(new System(sConfig_path));

    std::thread the_BackEnd(&System::ProcessBackEnd, pSystem);


    // sleep(5);
    std::thread thd_PubImuData(PubImuData);
    std::thread thd_PubImageData(PubImageData);

#ifdef __linux__
    std::thread thd_Draw(&System::Draw, pSystem);
#elif __APPLE__
    DrawIMGandGLinMainThrd();
#endif


    thd_PubImuData.join();
    thd_PubImageData.join();


    // thd_BackEnd.join();
    // thd_Draw.join();

    cout << "main end ... see you ... " << endl;

    return 0;
}


