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
        // 1.3 之前有的话在FeaturePerFrame添加已有的此特征点 在此帧image 的位置和其他信息, 并统计数目.
        else if ( it->feature_id == feature_id )
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num ++;      // 此帧image 有多少相同特征点被跟踪
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
        // 观测该特征点的： 起始帧小于窗口内帧数的倒数第三帧, 终止帧要大于倒数第二帧, 保证至少有两帧能观测到
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


double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax between second last frame and third last frame
    const FeaturePerFrame& frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];  // 倒数第3帧
    const FeaturePerFrame& frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];  // 倒数第2帧

    double ans = 0;

    Vector3d p_j = frame_j.point;   // 3D路标点(倒数第2帧)
    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;   // 3D路标点(倒数第3帧)
    Vector3d p_i_comp;

    // int r_i = frame_count - 2;
    // int r_j = frame_count - 1;
    // p_j_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;

    double dep_i = p_i(2);          // point.z()
    double u_i = p_i(0) / dep_i;    // point.x() / point.z()
    double v_i = p_i(1) / dep_i;    // point.y() / point.z()
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    // 求距离
    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

void FeatureManager::debugShow()
{
    // ROS_DEBUG("debug show");
    for( auto &it : feature )
    {
        assert(it.feature_per_frame.size() != 0);
        assert(it.start_frame >= 0);
        assert(it.used_num >= 0);

        // ROS_DEBUG("%d, %d, %d", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            // ROS_DEBUG("%d, " int(j.is_used));
            sum += j.is_used;
            printf("(%lf, %lf)", j.point(0), j.point(1));
        }
        assert(it.used_num == sum);
    }
}


/* 得到给定两帧之间的对应特征点3D坐标 */
vector <pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;

    for(auto &it : feature) // 遍历特征点feature的list容器
    {
        // 要找特征点的两帧在窗口范围内, 可以直接取。 窗口为:观测到当前特征点的所有图像帧
        if(it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame; // 左边帧 减 第一次观测到本特征点的帧数索引, 得到左边帧索引
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;

            corres.push_back(make_pair(a, b));
        }
    }

    return corres;
}


/* 设置特征点的逆深度估计值 */
void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1; // 先给feature ID 赋值 -1

    for(auto &it_per_id : feature)  // 遍历所有特征点
    {
        // 至少两帧观测得到这个特征点, 且首次观测到该特征点的图像帧在滑动窗范围内
        it_per_id.used_num = it_per_id.feature_per_frame.size();// 能够观测到某个特征点的所有相关帧数目
        if ( !(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE-2) )
            continue;

        // 求解逆深度
        it_per_id.estimated_depth = 1.0 / x(++feature_index);   // ??????? ???????

        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        // 深度小于0, 估计失败
        if ( it_per_id.estimated_depth < 0 )
        {
            it_per_id.solve_flag = 2;   // 失败估计
        }
        else
        {
            it_per_id.solve_flag = 1;   // 成功估计
        }
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next ++;
        if (it->solve_flag == 2)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;

    for (auto &it_per_id : feature )
    {
        // 至少两帧观测得到这个特征点, 且首次观测到该特征点的图像帧在滑动窗范围内
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if ( !(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2) )
            continue;

        // 求解逆深度
        it_per_id.estimated_depth = 1.0 / x(++feature_index);   // ??????? ???????
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;

    for (auto &it_per_id : feature )

    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if ( !(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2) )
            continue;

#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }

    return dep_vec;
}



