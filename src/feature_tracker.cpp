//
// Created by zlc on 2020/1/18.
//

#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)    // 判断点是否在便捷内
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);

    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) // 将status判断为0的点去掉, 即没有匹配上的点
{
    int j = 0;  // j 负责统计满足状态的 向量个数

    for ( int i = 0; i < int(v.size()); i ++ )
    {
        if ( status[i] )
            v[j++] = v[i];
    }

    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{

}

void FeatureTracker::setMask()
{
    if ( FISHEYE )
    {
        mask = fisheye_mask.clone();    // 图像掩膜 = 鱼眼相机的掩膜
    }
    else
    {
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255)); // // mask 初始化为255
    }


    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for( unsigned int i = 0; i < forw_pts.size(); i ++ )
    {
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));
        // track_cnt : 当前帧forw_img中每个特征点被追踪的次数;
        // forw_pts : 光流跟踪的后一帧图像对应的图像特征点
        // ids : 能够被跟踪到的特征点的id
    }

    // 对图像中提取的 forw_pts 中的点排序, 按照track_cnt[i]由大到小排序. sort(begin, end, compare)
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point, int>> &b)
    {
        return a.first > b.first;
    });

    forw_pts.clear();   // 特征点位置
    ids.clear();        // 特征点id
    track_cnt.clear();  // 特征点跟踪次数

    // 现在把按跟踪次数多少排好序的匹配点重新放回上述三个变量中
    for( auto &it : cnt_pts_id )
    {
        if( mask.at<uchar>(it.second.first) == 255 )
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

            //在mask（为0，白色）的forw_pts特征点位置上，画一个半径为MIN_DIST（具体值怎么设定？具体为特征点之间的最小间隔，半径=特征点最小间隔？）的颜色为黑色（0）的填充圆。
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            //在mask中将当前特征点周围半径为MIN_DIST的区域设置为0(黑)，后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
        }
    }
}

void FeatureTracker::addPoints()
{
    for( auto &p : n_pts )      // n_pts 是每一帧中新提取的特征点
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1); // 重新加入的点， 跟踪次数肯定为1
    }
}

// △△△△△重中之重△△△△△
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;             /* 第1次计时 */
    cur_time = _cur_time;


    // △1. 判断图像是否需要均衡化
    if( EQUALIZE )  // 均衡化增强对比度, 默认值为1, 表示图像太亮或者太暗
    {
        // CLAHE算法增强图像效果
        // 代码中使用了cv::createCLAHE(3.0, cv::Size(8, 8))函数来增强图像的显示效果，这样便于后边的检测。
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;         /* 第2次计时 */
        clahe->apply(_img, img);
        // ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
    {
        img = _img;
    }

    // △2. 判断当前帧是否为第一次读入图像数据
    if( forw_img.empty() )      // 光流后的帧为空说明刚开始
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }
    forw_pts.clear();       // 此时forw_pts还保存的是上一帧图像中的特征点, 所以把它清除

    // △3. 判断 光流跟踪前一帧 中特征点规模是否不为0, 不为0表示有图像数据点, 对其进行光流跟踪
    if( cur_pts.size() > 0 )
    {
        TicToc t_o;         /* 第3次计时 */
        vector<uchar> status;   // status表示cur_pts和forw_pts对应点对是否跟踪成功, 无法被追踪到的点记为0
        vector<float> err;

        // calcOpticalFlowPyrLK() 从cur_pts到forw_pts做LK金字塔光流法
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // 光流跟踪结束后，判断跟踪成功的角点是否都在图像内,不在图像内的status置为0
        for( int i=0; i<int(forw_pts.size()); i ++ )
        {
            if( status[i] && !inBorder(forw_pts[i]) )
            {
                status[i] = 0;
            }
        }

        // 根据status, 把跟踪失败的点剔除
        // 不仅要从当前帧数据forw_pts中剔除, 而且还要从cur_un_pts、prev_pts和cur_pts中剔除
        // prev_pts和cur_pts中的特征点是一一对应的, 光流跟踪后的点的集合
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);          // 上一帧和上上帧 特征点数量经过剔除后一致
        reduceVector(forw_pts, status);         //
        // 将光流跟踪后的点的id和跟踪次数, 根据跟踪的状态(status)进行重组
        reduceVector(ids, status);              //
        reduceVector(cur_un_pts, status);       //
        reduceVector(track_cnt, status);        // 跟踪次数中剔除跟踪失败的点
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    // △4. 光流追踪成功后, 被成功跟踪后的特征点次数track_cnt就加1
    for(auto &n : track_cnt )   // 遍历track_cnt, 每个角点的跟踪次数, 跟踪成功的点次数+1
        n ++;

    // △5. 发布这一帧数据
    if ( PUB_THIS_FRAME )
    {
        rejectWithF();      // 通过基本矩阵剔除outliers, 删除一部分点

        // ROS_DEBUG("set mask begins");
        TicToc t_m;         /* 第4次计时 */

        setMask();          // 保证相邻的特征点之间要间隔30个像素, 设置mask, 并且在这一步中根据track_cnt对ids号进行了排序
        // ROS_DEBUG("set mask costs %fms", t_m.toc());

        // △6. 计算是否需要提取新的特征点
        // ROS_DEBUG("detect feature begins");
        TicToc t_t;         /* 第5次计时 */
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());    // 最大需要跟踪的点 - 当前帧已经跟踪的
        if( n_max_cnt > 0 )
        {
            if( mask.empty() )
                cout << "mask is empty " << endl;
            if( mask.type() != CV_8UC1 )
                cout << "mask type wrong " << endl;
            if( mask.size() != forw_img.size() )
                cout << "wrong size " << endl;

            /**
             *  cv::goodFeaturesToTrack()
             *  @brief   在mask中不为0的区域检测新的特征点
             *  @optional    ref:https://docs.opencv.org/3.1.0/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541
             *  @param[in]    InputArray _image=forw_img 输入图像
             *  @param[out]   _corners=n_pts 存放检测到的角点的vector
             *  @param[in]    maxCorners=MAX_CNT - forw_pts.size() 返回的角点的数量的最大值
             *  @param[in]    qualityLevel=0.01 角点质量水平的最低阈值（范围为0到1，质量最高角点的水平为1），小于该阈值的角点被拒绝
             *  @param[in]    minDistance=MIN_DIST 返回角点之间欧式距离的最小值
             *  @param[in]    _mask=mask 和输入图像具有相同大小，类型必须为CV_8UC1,用来描述图像中感兴趣的区域，只在感兴趣区域中检测角点
             *  @param[in]    blockSize：计算协方差矩阵时的窗口大小
             *  @param[in]    useHarrisDetector：指示是否使用Harris角点检测，如不指定，则计算shi-tomasi角点
             *  @param[in]    harrisK：Harris角点检测需要的k值
             *  @return      void
             */

            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT-forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else    // 不需要添加跟踪点
            n_pts.clear();
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // △7. 将n_pts中新提取到的角点放到forw_pts中, id初始化-1, track_cnt初始化为1.
        // ROS_DEBUG("add feature begins");
        TicToc t_a;         /* 第6次计时 */
        addPoints();
        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    // △8. 将当前帧数据传递给上一帧
    prev_img = cur_img;         // 在第一帧处理中还是等于当前帧forw_img, 上一真赋值给上上帧
    prev_pts = cur_pts;         // 在第一帧中不作处理
    prev_un_pts = cur_un_pts;   // 在第一帧中不作处理
    cur_img = forw_img;         // 将当前帧赋给上一帧
    cur_pts = forw_pts;

    // △9. 从第2张图像输入后每进行一次循环, 最后还需要对匹配的特征点对进行畸变校正和深度归一化, 计算速度
    undistortedPoints();
    prev_time = cur_time;
}



/// 作业中添加的代码：  loadPointData(points_file_name, dStampSec);
void FeatureTracker::loadPointData(string point_file_name, double _cur_time)
{
    TicToc t_r;             /* 第1次计时 */
    cur_time = _cur_time;

    // △2. 判断当前帧是否为第一次读入图像数据
    forw_pts.clear();       // 此时forw_pts还保存的是上一帧图像中的特征点, 所以把它清除
    ifstream fsImage;
    fsImage.open(point_file_name);
    if ( !fsImage.is_open() )
    {
        cerr << "can't open point_file_name: " << point_file_name << endl;
        return;
    }

    std::string sImagePoint_line;
    double tmp;     // 用于存前4维数据
    double u, v;
    cv::Point2f p;

    while ( std::getline(fsImage, sImagePoint_line) && !sImagePoint_line.empty() )
    {
        std::istringstream ssImageData(sImagePoint_line);

        for ( int i=0; i<4; i ++ )
            ssImageData >> tmp;

        ssImageData >> p.x >> p.y;

        forw_pts.push_back(p);
    }

    /*for (int i=0; i<int(forw_pts.size()); i ++)
    {
        cout << forw_pts[i] << endl;
    }*/

    // △4. 光流追踪成功后, 被成功跟踪后的特征点次数track_cnt就加1
    for(auto &n : track_cnt )   // 遍历track_cnt, 每个角点的跟踪次数, 跟踪成功的点次数+1
        n ++;

    if ( cur_pts.size()==0 )
    {
        n_pts = forw_pts;
        addPoints();
        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    // △8. 将当前帧数据传递给上一帧
    // prev_pts = cur_pts;         // 在第一帧中不作处理
    prev_un_pts = cur_un_pts;   // 在第一帧中不作处理
    cur_pts = forw_pts;

    // △9. 从第2张图像输入后每进行一次循环, 最后还需要对匹配的特征点对进行畸变校正和深度归一化, 计算速度
    undistortedSimPoints();
    prev_time = cur_time;       // 最后再赋值时间, ∵ undistortedSimPoints()函数中要用到上一帧时间 prev_time

}



// 通过基本矩阵(F)去除外点 outliers
void FeatureTracker::rejectWithF()
{
    if( forw_pts.size() >= 8 )      // 判断当前帧追踪到的角点是否大于8个, 使用8点法
    {
        // ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());   // 上一帧和当前帧图像上的角点对应的归一化平面坐标
        for(unsigned int i = 0; i < cur_pts.size(); i ++ )
        {
            Eigen::Vector3d tmp_p;
            // 将点从图像平面对应到投影空间, tmp_p为输出结果。 其实就是2d-->3d的转换过程
            // cur_pts 中保存的是上一帧图像的角点
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // 转换为归一化像素坐标, FOCAL_LENGTH 的值为460, 十四讲第86页, 后面的cx = COL/2 并且 cy = ROW/2
            /*
             *  ------------------------------------------------>
             *  |                       COL/2.0
             *  |
             *  |                       tmp.x()
             *  |  ROW/2.0              tmp.y()
             *  |
             *  ↓
             * */
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;   // 十四讲中P85页提到的 [cx, cy] 原点的平移
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;   // Puv = K Pc(归一化点)
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());              // 像素点

            // forw_pts 中保存的是当前图像中能通过光流追踪到的角点的坐标
            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        // 计算基础矩阵： 利用上一帧图像的角点和当前图像角点, 来计算基础矩阵
        /* 从两个图像中对应的3d点对来计算基础矩阵 */
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}


bool FeatureTracker::updateID(unsigned int i)
{
    if( i < ids.size() )
    {
        //cout << "before: " << ids[i] << " " << n_id << endl;
        if( ids[i] == -1 )
            ids[i] = n_id ++;
        //cout <<"after: " << ids[i] << " " << n_id << endl;
        return true;
    }
    else
        return false;
}


/* 读取相机标定的内参 */
void FeatureTracker::readIntrinsicParameter(const string& calib_file)
{
    cout << "read parameter of camera " << calib_file << endl;      // calib_file = sConfig_path = ./config/euroc_config.yaml
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);   // 根据畸变模型参数和相机内参设置相机参数
}


void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;

    for( int i = 0; i < COL; i ++ )     // 列
    {
        for( int j = 0; j < ROW;  )       // 行
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            // printf("%f, %f->%f, %f, %f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    }

    for( int i = 0; i < int(undistortedp.size()); i ++ )
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        // cout << trackerData[0].K << endl;
        // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));

        if( pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600 )
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }

    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}


void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();             // 清空当前帧归一化平面上的特征点，注意上面有 cur_pts = forw_pts;
    cur_un_pts_map.clear();

    // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for( unsigned int i = 0; i < cur_pts.size(); i ++ )
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;

        m_camera->liftProjective(a, b);    // 投影到 3 维空间
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));    // 当前帧的归一化点
        cur_un_pts_map.insert( make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())) );
        // printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }


    // calculate points velocity
    if( !prev_un_pts_map.empty() )
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();

        for(unsigned int i = 0; i < cur_un_pts.size(); i ++ )
        {
            if( ids[i] != -1 )
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if( it != prev_un_pts_map.end() )
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));      // 在归一化平面点上计算的速度
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else    // 上一帧没有速度, 所以判断为第一帧
    {
        for (unsigned int i = 0; i < cur_pts.size(); ++ i)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }

    prev_un_pts_map = cur_un_pts_map;
}


void FeatureTracker::undistortedSimPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        cur_un_pts.push_back(cv::Point2f(cur_pts[i].x,cur_pts[i].y));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(cur_pts[i].x,cur_pts[i].y)));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }

    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}





