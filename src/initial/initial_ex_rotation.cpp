//
// Created by zlc on 2020/4/7.
//

#include "initial/initial_ex_rotation.h"

InitialEXRotation::InitialEXRotation()
{
    frame_count = 0;
    Rc.push_back(Matrix3d::Identity());     // q_ck_ck+1
    Rc_g.push_back(Matrix3d::Identity());   // q_bk_bk+1
    Rimu.push_back(Matrix3d::Identity());

    ric = Matrix3d::Identity(); // 传说中的qbc
}

bool InitialEXRotation::CalibrationExRotation(vector<pair<Vector3d, Vector3d> > corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{
    // Step1: 滑动窗口内的帧数 + 1
    frame_count ++;

    // Step2：计算前后两帧的旋转矩阵, 加到Rc向量内, 直到校准成功
    // solveRelativeR函数中可以根据corres中特征点的对应关系计算出前后两帧图像之间的旋转矩阵，加到Rc中，Rc表示相机位姿的旋转矩阵
    Rc.push_back(solveRelativeR(corres));               // 通过视觉sfM获取相邻两关键帧之间相机的相对旋转R_ck_ck+1
    // delta_q_imu为IMU预积分得到的旋转四元数，转换成矩阵形式存入Rimu当中。则Rimu中存放的是imu预积分得到的旋转矩阵   ××××××
    // 每次迭代前先用前一次估计的ric将R_bk+1^bk变换成R_ck+1^ck
    Rimu.push_back(delta_q_imu.toRotationMatrix());     // IMU预积分得到的旋转

    Rc_g.push_back(ric.inverse() * delta_q_imu * ric);  //


    Eigen::MatrixXd A( 4 * frame_count, 4);     // A  x = 0 实际就是 ( [q_bk_bk+1] - [q_ck_ck+1] ) q_bc = Q_k_k+1 q_bc =0
    A.setZero();
    int sum_ok = 0;
    for ( int i = 1; i <= frame_count; i ++ )
    {
        Quaterniond r1(Rc[i]);
        Quaterniond r2(Rc_g[i]);

        // Step3： 求取估计出的相机与IMU之间 旋转的角度残差
        double angular_distance = 180 / M_PI * r1.angularDistance(r2);      // 1弧度 = 180/π°(度)
        // ROS_DEBUG("%d %f", i, angular_distance);

        // Step4： 计算外点剔除的权重
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;   // 阈值threshold = 5.0
        ++ sum_ok;
        Matrix4d L, R;

        // Step5：求取系数矩阵, 公式(3)的系数矩阵Q, 下面代码实际求得是转置关系,
        // ( [q_ck_ck+1] - [q_bk_bk+1] ) q_cb = 0
        // 最后求出的q_bc 是经过 q_cb 的转置
        // 得到相机对极关系, 得到旋转q的左乘
        double w = Quaterniond(Rc[i]).w();      // 实数部分
        Vector3d q = Quaterniond(Rc[i]).vec();  // 虚数部分
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0,3) = q;
        L.block<1, 3>(3,0) = -q.transpose();
        L(3,3) = w;

        // 得到由IMU预积分得到的旋转q的右乘
        Quaterniond R_ij(Rimu[i]);
        w = R_ij.w();       // 对w重新赋值
        q = R_ij.vec();     // 对q重新赋值
        R.block<3, 3>(0,0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0,3) = q;
        R.block<1, 3>(3,0) = -q.transpose();
        R(3,3) = w;

        // 拼接大矩阵Q, 公式(5)的系数矩阵Q
        A.block<4,4>((i-1)*4, 4) = huber * (L - R);
    }

    // Step6： 通过SVD分解, 求取相机与IMU的相对旋转
    // 解为系数矩阵A的右奇异向量V中最小奇异值对应的特征向量
    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    Matrix<double, 4, 1> x = svd.matrixV().col(3);       // SVD原理, 右边第4列对应最小向量
    Quaterniond estimated_R(x);                             // 矩阵转四元数
    ric = estimated_R.toRotationMatrix().inverse();         // ric =  rci^-1 = rci^T   ||   q_bc = q_cb^(-1)

    // Step7：判断对于相机与IMU的相对旋转是否满足终止条件
    // 1.用来求解相对旋转的IMU-Cmaera的测量对数是否大于滑窗大小。
    // 2.A矩阵第二小的奇异值是否大于某个阈值，使A得零空间的秩为1
    Vector3d ric_cov;
    ric_cov = svd.singularValues().tail<3>();
    // 至少迭代计算了WINDOW_SIZE次，且R的奇异值大于0.25才认为标定成功
    if ( frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25 )
    {
        calib_ric_result = ric;
        return true;
    }
    else
        return false;
}


/* 根据两帧图像的归一化特征点求解两帧的旋转矩阵 */           /* 两帧中得到的匹配特征点需要选择大于9个 */
Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if ( corres.size() >= 9 )       // 需要特征点大于9对时, 否则返回单位矩阵
    {
        vector<cv::Point2f> ll, rr;
        for ( int i = 0; i < (int)corres.size(); i ++ )
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0),corres[i].second(1)));
        }
        // 求解两帧的本质矩阵E, 多于8对匹配点时, 存在误匹配的情况下, 推荐使用RANSAC
        cv::Mat E = cv::findFundamentalMat(ll, rr); // RANSAC随机采样一致性选择
        cv::Mat_<double> R1, R2, t1, t2;    // 生成指定类型矩阵, 这4个矩阵在SLAM十四讲中有提到
        decomposeE(E, R1, R2, t1, t2);

        // 如果R1行列式为负, SVD分解-E
        if ( determinant(R1) + 1.0 < 1e-09 )
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }

        // 通过三角化得到的正深度选择Rt值, 求出来的 ans_R_cv = R_ck+1_ck (具体坐标转换下标可以看十四讲P142页)
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2? R1 : R2;

        // 对R求转置, 我们要返回 R_ck_ck+1, 所以对ans_R_cv = R_ck+1_ck 进行转置
        Matrix3d ans_R_eigen;
        for ( int i = 0 ; i < 3; i ++ )
        {
            for ( int j = 0; j < 3; j ++ )
            {
                ans_R_eigen(j, i) = ans_R_cv(i, j);
            }
        }
        return ans_R_eigen;
    }

    return Matrix3d::Identity();
}

/* 铜鼓本质矩阵, 恢复出两帧相机的运动R,t */
void InitialEXRotation::decomposeE(cv::Mat E,
                cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1,  0, 0,
                  0,  0, 1);    // 绕z轴旋转矩阵Rz^T(π/2)
    /*
     *           [ cos90°   sin90°  0 ]     [  0  1  0 ]
     * Rz(π/2) = [ -sin90°  cos90°  0 ]  =  [ -1  0  0 ]  // 需要注意上面是转置
     *           [   0        0     1 ]     [  0  0  1 ]
     * */

    cv::Matx33d Wt( 0, 1, 0,
                   -1, 0, 0,
                   0,  0, 1);   // 绕z轴旋转矩阵Rz^T(-π/2)
    /*
     *           [ cos-90°    sin-90°  0 ]     [  0 -1  0 ]
     * Rz(-π/2) = [ -sin-90°  cos-90°  0 ]  =  [  1  0  0 ]  // 需要注意上面是转置
     *           [   0           0     1 ]     [  0  0  1 ]
     * */

    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = svd.u.col(2);

}

/* 通过三角化得到正深度选择, R,t解 */
double InitialEXRotation::testTriangulation(const vector<cv::Point2f> &l,
                const vector<cv::Point2f> &r,
                cv::Mat_<double> R, cv::Mat_<double> t)
{               // l和r是ck帧和ck+1帧的匹配的一堆归一化平面坐标点, R和t是R_ck+1_ck, 由前一帧旋转到后一帧的旋转矩阵
    cv::Mat pointcloud;     // 深度点云, 路标3D点坐标
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);

    cv::Matx34f P1 = cv::Matx34f(R(0,0), R(0,1), R(0,2), t(0),
                                 R(1,0), R(1,1), R(1,2), t(1),
                                 R(2,0), R(2,1), R(2,2), t(2));

    cv::triangulatePoints(P, P1, l, r, pointcloud);

    int front_count = 0;

    for ( int i = 0; i < pointcloud.cols; i ++ )
    {
        double normal_factor = pointcloud.col(i).at<float>(3);

        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);     // 第1帧相机坐标系下, 路标特征点3D坐标点, 不是归一化坐标
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);    // 通过R_ck+1_ck 转换到ck+1帧下

        if ( p_3d_l(2) > 0 && p_3d_r(2) > 0 )       // 在第1帧相机坐标系下深度>0 && 第2帧相机坐标系下深度>0
            front_count ++;
    }

    // ROS_DEBUG("MotionEstimator: %f", 1.0 * front_count / pointcloud.cols);
    return 1.0 * front_count / pointcloud.cols;
    // 在当前R1,t1、或者R1,t2/R2,ti/R2,t2下，
    // 传进来的匹配特征点有多少解出来得到路标3D点深度是正的
}
