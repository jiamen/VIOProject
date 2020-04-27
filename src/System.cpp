//
// Created by zlc on 2020/1/18.
//

#include "System.h"

#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace pangolin;

System::System(string sConfig_file_) : bStart_backend(true) // 开启后端
{
    string sConfig_file = sConfig_file_ + "euroc_config1.yaml";

    cout << "1 System() sConfig_file: " << sConfig_file << endl;        // 输出中的第一句打印： 打印配置文件 sConfig_path = ./config/euroc_config.yaml
    readParameters(sConfig_file);       // parameters.cpp 文件中的函数, 不涉及相机的部分参数, 比如后面用到的estimator.setParameter()中的相机和IMU之间的变换参数

    // 读取每个相机实例读取对应的相机内参
    trackerData[0].readIntrinsicParameter(sConfig_file);

    //readParameters(sConfig_file);

    estimator.setParameter();

    ofs_pose.open("./pose_output.txt",fstream::app | fstream::out);
    if(!ofs_pose.is_open())
    {
        cerr << "ofs_pose is not open" << endl;
    }
    // thread thd_RunBackend(&System::process,this);
    // thd_RunBackend.detach();
    cout << "2 System() end" << endl;
}


System::~System()
{
    bStart_backend = false;

    pangolin::QuitAll();

    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.clearState();
    m_estimator.unlock();

    ofs_pose.close();
}

// ==================================================================================================================

/// 4大线程之一的 发布图像数据 ☆☆☆△△△
void System::PubImageData(double dStampSec, Mat &img)
{
    if (!init_feature)  // 初始init_feature = 0
    {
        cout << "1 PubImageData skip the first detected feature, which doesn't contain optical flow speed" << endl;
        init_feature = 1;
        return;
    }

    if (first_image_flag)   // 初始first_image_flag = true
    {
        cout << "2 PubImageData first_image_flag" << endl;
        first_image_flag = false;
        first_image_time = dStampSec;   // 第一帧时间
        last_image_time = dStampSec;    // 上一帧图像时间也同样初始化为dStampSec
        return;
    }

    // 检测不稳定的图像流，
    // 如果当前的时间戳减去上次最后一帧时间戳大于1，
    // 或者当前的时间戳小于last_imgae_time，
    // 则认为i检测到不稳定的图像流，将首帧标志重置为true，
    // last_image_time置为0，发布数目置为1。
    // detect unstable camera stream
    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time)
    {
        cerr << "3 PubImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }

    last_image_time = dStampSec;        // 把当前帧图像时间赋值给上一帧

    // 控制图像发布的频率，在读取文件的方式中会有这个，在ORBSLAM中同样。
    // 如果已经发布的图像数目除以总的时间差小于等于频率，那么继续发布，否则不发布。
    // 进一步判断，如果频率相差图像发布频率的0.01倍， 就从当前帧开始重新计算频率。
    // 保证其始终小于等于FREQ。
    // frequency control
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control, 如果发布快了，则重置一下，始终保证小于等于FREQ
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }


    TicToc t_r;
    // cout << "3 PubImageData t : " << dStampSec << endl;
    trackerData[0].readImage(img, dStampSec);   // △△△ 开始引入图像特征点检测和跟踪 △△△, 注意PUB_THIS_FRAME=true / false


    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= trackerData[0].updateID(i);

        if (!completed)     // 更新点完成后，超过150点数自动退出
            break;
    }


    if (PUB_THIS_FRAME)
    {
        pub_count ++;   // 对 发布的帧数 计数
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
        feature_points->header = dStampSec;
        vector<set<int>> hash_ids(NUM_OF_CAM);

        for (int i = 0; i < NUM_OF_CAM; i++)        // 这里只有1个摄像头
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];          // 每个特征点的号
                    hash_ids[i].insert(p_id);
                    double x = un_pts[j].x;
                    double y = un_pts[j].y;
                    double z = 1;

                    // 按照定义的ImgConstPtr特征点 格式 存放
                    feature_points->points.push_back(Vector3d(x, y, z));        // 归一化坐标系下的点
                    feature_points->id_of_point.push_back(p_id * NUM_OF_CAM + i);
                    feature_points->u_of_point.push_back(cur_pts[j].x);
                    feature_points->v_of_point.push_back(cur_pts[j].y);
                    feature_points->velocity_x_of_point.push_back(pts_velocity[j].x);
                    feature_points->velocity_y_of_point.push_back(pts_velocity[j].y);
                }
            }
            //}
            // skip the first image; since no optical speed on frist image
            if (!init_pub)      // 初始发布为0,这里会执行一次
            {
                cout << "4 PubImage init_pub skip the first image!" << endl;
                init_pub = 1;
            }
            else
            {
                m_buf.lock();
                feature_buf.push(feature_points);   // 所有的特征点都保存到feature_buf中
                // cout << "5 PubImage t : " << fixed << feature_points->header
                //     << " feature_buf size: " << feature_buf.size() << endl;
                m_buf.unlock();
                con.notify_one();
            }
        }
    }


#ifdef __linux__
    cv::Mat show_img;
    cv::cvtColor(img, show_img, CV_GRAY2RGB);
    if (SHOW_TRACK)     // parameters.cpp文件中置为1
    {
        for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j ++)
        {
            double len = min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE);
            cv::circle(show_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }

        // 注意在显示特征点的时候，特征点的颜色深度与其被追踪的时长正相关。

        cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
        cv::imshow("IMAGE", show_img);
        cv::waitKey(1);
    }
#endif
    // cout << "5 PubImage" << endl;

}


///////////////////////////////////  新增PubSimImageData() 函数 ////////////////////////////////////////
void System::PubSimImageData(double dStampSec, string all_points_file_name)
{
    if (!init_feature)
    {
        cout << "1 PubImageData skip the first detected feature, which doesn't contain optical flow speed" << endl;
        init_feature = 1;
        return;
    }

    if (first_image_flag)
    {
        cout << "2 PubImageData first_image_flag" << endl;
        first_image_flag = false;
        first_image_time = dStampSec;
        last_image_time = dStampSec;
        return;
    }
    // detect unstable camera stream
    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time)
    {
        cerr << "3 PubImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }
    last_image_time = dStampSec;
    // frequency control
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    TicToc t_r;
    // cout << "3 PubImageData t : " << dStampSec << endl;
    //trackerData[0].readImage(img, dStampSec);
    trackerData[0].loadPointData(all_points_file_name, dStampSec);

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= trackerData[0].updateID(i);

        if (!completed)
            break;
    }
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
        feature_points->header = dStampSec;
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    double x = un_pts[j].x;
                    double y = un_pts[j].y;
                    double z = 1;
                    feature_points->points.push_back(Vector3d(x, y, z));    // 归一化平面
                    feature_points->id_of_point.push_back(p_id * NUM_OF_CAM + i);
                    feature_points->u_of_point.push_back(cur_pts[j].x);
                    feature_points->v_of_point.push_back(cur_pts[j].y);
                    feature_points->velocity_x_of_point.push_back(pts_velocity[j].x);
                    feature_points->velocity_y_of_point.push_back(pts_velocity[j].y);
                }
            }
            //}
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                cout << "4 PubImage init_pub skip the first image!" << endl;
                init_pub = 1;
            }
            else
            {
                m_buf.lock();
                feature_buf.push(feature_points);
                // cout << "5 PubImage t : " << fixed << feature_points->header
                //     << " feature_buf size: " << feature_buf.size() << endl;
                m_buf.unlock();
                con.notify_one();
            }
        }
    }
    // cout << "5 PubImage" << endl;

}


// IMU 与 图像的同步， 数据同步封装在本函数中
vector<pair<vector<ImuConstPtr>, ImgConstPtr>> System::getMeasurements()
{
    vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())     // 初始时刻 和 两个buffer都已经弹出完毕后，从这个条件跳出循环
        {
            // cerr << "1 imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }
        if (!(imu_buf.back()->header > feature_buf.front()->header + estimator.td))
        {   // imu 时间太小，落后
            cerr << "wait for imu, only should happen at the beginning sum_of_wait: "
                 << sum_of_wait << endl;
            sum_of_wait ++;
            // cout << "measurements imu size : " << measurements[0].first.size() << endl;
            return measurements;
        }
        if (!(imu_buf.front()->header < feature_buf.front()->header + estimator.td))
        {   // image 时间太小，落后
            cerr << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();  // 扔掉这一帧，image 继续往前赶 imu的时间
            continue;
        }

        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // cout << "1 getMeasurements IMUs size: " << IMUs.size() << endl;
        IMUs.emplace_back(imu_buf.front());     // 这里放最后一个imu，第21个

        if (IMUs.empty())
        {
            cerr << "no imu between two image" << endl;
        }
        /*cout << "1 getMeasurements img t: " << fixed << img_msg->header
             << " imu begin: "<< IMUs.front()->header
             << " end: " << IMUs.back()->header
             << endl;*/
        measurements.emplace_back(IMUs, img_msg);   // 几帧IMU 和 1帧图像
        // cout << "measurements imu size : " << measurements[0].first.size() << endl;     // 输出是21
    }

    return measurements;    // 不执行这句怎么返回的值
}


/// 4大线程之一的 发布IMU数据 ☆☆☆△△△
void System::PubImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                        const Eigen::Vector3d &vAcc)
{
    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());  // 建立智能指针
    imu_msg->header = dStampSec;                    // 读取时间戳
    imu_msg->linear_acceleration = vAcc;            // 读取xyz3个加速度值
    imu_msg->angular_velocity = vGyr;               // 最后读取xyz3个角速度值

    if (dStampSec <= last_imu_t)                    // 当前要发布的IMU时间戳 <= 上一次发布的IMU时间戳
    {
        cerr << "imu message in disorder!" << endl;
        return;
    }
    last_imu_t = dStampSec;
    // cout << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " acc: " << imu_msg->linear_acceleration.transpose()
    //     << " gyr: " << imu_msg->angular_velocity.transpose() << endl;
    m_buf.lock();
    imu_buf.push(imu_msg);
    // cout << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " imu_buf size:" << imu_buf.size() << endl;
    m_buf.unlock();
    con.notify_one();                               // 随机唤醒一个等待的线程
}


/// 4大线程之一的 后端优化线程 ☆☆☆△△△
// thread: visual-inertial odometry
void System::ProcessBackEnd()
{
    cout << "1 ProcessBackEnd start" << endl;   // 开启4大线程后的第一句打印

    while ( bStart_backend )      // System类初始化函数中, bStart_backend = true, 开启后端优化
    {
        // 首先获取观测数据，包括图像特征，IMU数据，获取数据的时候使用的是互斥锁。
        // 多线程访问同一资源时，为了保证数据的一致性，必要时需要加锁mutex。
        // cout << "1 process()" << endl;
        vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

        unique_lock<mutex> lk(m_buf);
        con.wait(lk, [&] {
            return (measurements = getMeasurements()).size() != 0;
        });

        if( measurements.size() > 1)
        {
            cout << "1 getMeasurements size: " << measurements.size()
                 << " imu sizes: " << measurements[0].first.size()
                 << " feature_buf size: " <<  feature_buf.size()
                 << " imu_buf size: " << imu_buf.size() << endl;
        }


        lk.unlock();
        m_estimator.lock();     // 状态估计过程加锁

        for (auto &measurement : measurements)      // 基本上 measuremets = 1
        {
            auto img_msg = measurement.second;                      // 取出测量值中的 1帧 图像帧
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;  //
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header;                         // imu时间戳
                double img_t = img_msg->header + estimator.td;      // imu时间阈值

                if (t <= img_t) // 如果IMU时间戳t < 图像时间戳
                {
                    if (current_time < 0)   // 第一次，初始时刻为-1
                        current_time = t;
                    double dt = t - current_time;
                    cout << "dt : " << dt << endl;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x();  // 加速度分量
                    dy = imu_msg->linear_acceleration.y();
                    dz = imu_msg->linear_acceleration.z();
                    rx = imu_msg->angular_velocity.x();     // 角速度分量
                    ry = imu_msg->angular_velocity.y();
                    rz = imu_msg->angular_velocity.z();
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else    // t > 图像时间戳img_t
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    cout << "dt_1 : " << dt_1 << endl;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    // 将加权后的IMU数据用于后面处理
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x();
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y();
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z();
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x();
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y();
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z();
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }


            // cout << "processing vision data with stamp:" << img_msg->header
            //     << " img_msg->points.size: "<< img_msg->points.size() << endl;

            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->id_of_point[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();
                double p_u = img_msg->u_of_point[i];    // 点的
                double p_v = img_msg->v_of_point[i];
                double velocity_x = img_msg->velocity_x_of_point[i];
                double velocity_y = img_msg->velocity_y_of_point[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);

            }

            TicToc t_processImage;
            estimator.processImage(image, img_msg->header);

            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[WINDOW_SIZE]);
                p_wi = estimator.Ps[WINDOW_SIZE];
                vPath_to_draw.push_back(p_wi);      // 这里保存位移，用来在后面的画轨迹线程中用

                double dStamp = estimator.Headers[WINDOW_SIZE];

                // 保存位移轨迹和旋转结果
                cout << "1 BackEnd processImage dt: " << fixed << t_processImage.toc() << " stamp: " <<  dStamp << " p_wi: " << p_wi.transpose() << endl;
                ofs_pose << fixed << dStamp << " " << p_wi.transpose() << " " << q_wi.coeffs().transpose() << endl;

            }
        }   // end for (auto &measurement : measurements)   // 基本上 measuremets = 1

        m_estimator.unlock();
    }   // end bStart_backend
}


/// 4大线程之一的 图像展示界面 ☆☆☆△△△
void System::Draw()
{
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    );

    d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // pangolin::OpenGlRenderState s_cam(
    //         pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
    //         pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    // );

    // pangolin::View &d_cam = pangolin::CreateDisplay()
    //         .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
    //         .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);

        // draw poses
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();
        for(int i = 0; i < nPath_size-1; ++i)
        {
            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i+1].x(), vPath_to_draw[i+1].y(), vPath_to_draw[i+1].z());
        }
        glEnd();

        // points
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            for(int i = 0; i < WINDOW_SIZE+1;++i)
            {
                Vector3d p_wi = estimator.Ps[i];
                glColor3f(1, 0, 0);
                glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
            }
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

#ifdef __APPLE__
    void System::InitDrawGL()
{
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    );

    d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
}

void System::DrawGLFrame()
{

    if (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);

        // draw poses
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();
        for(int i = 0; i < nPath_size-1; ++i)
        {
            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i+1].x(), vPath_to_draw[i+1].y(), vPath_to_draw[i+1].z());
        }
        glEnd();

        // points
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            for(int i = 0; i < WINDOW_SIZE+1;++i)
            {
                Vector3d p_wi = estimator.Ps[i];
                glColor3f(1, 0, 0);
                glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
            }
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
#endif

}


