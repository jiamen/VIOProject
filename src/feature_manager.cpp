//
// Created by zlc on 2020/1/18.
//
#include "feature_manager.h"

int FeaturePerId::endFrame() // 返回最后一次观测到这个特征点的图像帧索引ID
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[]) : Rs(_Rs)
{
    for(int i=0; i < NUM_OF_CAM; i ++)
    {
        ric[i].setIdentity();
    }
}

void FeatureManager::setRic(Matrix3d *_ric)
{
    for(int i=0; i<NUM_OF_CAM; i ++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

// 窗口被跟踪特征点的数量
int FeatureManager::getFeatureCount()
{
    int cnt = 0;

    for( auto &it : feature )   // 遍历特征点
    {
        it.used_num = it.feature_per_frame.size();  // 所有特征点被观测到的帧数

        // 如果该特征点有两帧以上观测到了 且 第一次观测到帧数不是在最后
        if( it.used_num >= 2 && it.start_frame < WINDOW_SIZE-2 )
        {
            cnt ++;   // 这个特征点是有效的
        }
    }

    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &image, double td)
{                                            // 窗口内帧的数目    // Feature Id     // Camera Id      // FeaturePerFrame: (x, y, z, u, v, vx, vy)
    // ROS_DEBUG("input feature: %d", (int)image.size());   // 特征点数量
    // ROS_DEBUG("num of feature: %d", getFeatureCount());  // 能够作为特征点的数量
    double parallax_sum = 0;// 所有特征点视差总和
    int parallax_num = 0;
    last_track_num = 0;     // 被跟踪的个数

    // 1.把image map中的所有特征点放入feature list容器中
    // 遍历特征点, 看特征点是否在特征点的列表, 如果没在, 则将<FeatureID, Start_frame>存入到Feature列表中, 否则统计数目
    for (auto& id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        // 1.1 迭代器寻找feature list中是否有这feature_id
        int feature_id = id_pts.first;  // i当前需要新加入的特征点ID
        // 第3个参数是Lambda表达式
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
        {
            return it.feature_id == feature_id;
        });

        // 1.2 如果没有则新建一个, 并在feature管理器的list容器最后添加: FeaturePerId、FeaturePerFrame、FeaturePerFrame
        if ( it == feature.end() )
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        // 1.3 之前有的话在FeaturePerFrame添加此特征点在此帧的位置和其他信息, 并统计数目.
        else if ( it->feature_id == feature_id )
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num ++;      // 此帧有多少相同特征点被跟踪
        }
    }

    // 2. 追踪次数小于20 或者 窗口内帧的数目小于2, 是关键帧
    if (frame_count <  2 || last_track_num < 20)
    {
        return true;
    }

    // 3. 计算每个特征在次新帧和次次新帧中的视差
    for(auto& it_per_id : feature)
    {
        // 观测该特征点的： 起始帧小于倒数第三帧, 终止帧要大于倒数第二帧, 保证至少有两帧能观测到
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count -1)
        {
            // 总视差: 该特征点在两帧的归一化平面上的坐标点的距离ans
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num ++;
        }
    }

    // 4.1 第一次加进去的, 是关键帧
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        // ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        // ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);

        // 4.2 平均视差大于阈值的是关键帧
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}
