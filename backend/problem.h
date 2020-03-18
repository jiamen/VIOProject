//
// Created by zlc on 2020/3/6.
//

#ifndef _INC_03_MYCURVEFITTING_LM_PROBLEM_H_
#define _INC_03_MYCURVEFITTING_LM_PROBLEM_H_

#include <unordered_map>
#include <map>
#include <memory>

#include "backend/eigen_types.h"
#include "backend/edge.h"
#include "backend/vertex.h"

typedef unsigned long ulong;

namespace myslam
{

namespace backend
{

class Problem
{
public:
    /**
     *  问题的类型：
     *  SLAM问题还是通用的问题
     *
     *  如果是SLAM问题, 那么pose和landmark是区分开的, Hessian 以稀疏方式存储
     *  SLAM 问题只接受一些特定的 Vertex 和 Edge
     *  如果是通用的问题, 那么hessian是稠密的, 除非用户设定某些vertex为marginalized
     * */
    enum class ProblemType
    {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };

    typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
    typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
    typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Problem(ProblemType problemType);

    Problem(const VecX &bPpSchur) : b__pp_schur_(bPpSchur) {}

    ~Problem();



    bool AddVertex(std::shared_ptr<Vertex> vertex);

    /**
     *  remove a vertex
     *  @param vertex_to_remove
     * */
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);

    bool RemoveEdge(std::shared_ptr<Edge> edge);



    /**
     * 取得在优化中被判断为outlier部分的边, 方便前端去除outlier
     * @param outlier_edges
     */
    void GetOutlierEdge(std::vector<std::shared_ptr<Edge>> &putlier_edges);

    /**
     * 求解此问题
     * @param iterations
     * @return
     */
    bool Solve(int iterations);

    // 边缘化一个frame和以它为host的landmark
    bool Marginalize(std::shared_ptr<Vertex> frameVertex,
                const std::vector<std::shared_ptr<Vertex>> &landmarkVertices);

    bool Marginalize(const std::shared_ptr<Vertex> frameVertex);

    // test compute prior
    void TestComputePrior();


private:
    double currentLambda_;
    double currentChi_;
    double stopThresholdLM_;    // LM 迭代退出阈值条件
    double ni_;                 // 控制 Lambda 缩放大小

    ProblemType problemType_;

    // 整个信息矩阵
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    // 先验部分信息
    MatXX H_prior_;
    VecX b__pp_schur_;
    MatXX Jt_prior_inv_;
    VecX err_prior_;

    // Hessian 的 Landmark 和 pose 部分
    MatXX H_pp_;
    VecX b_pp_;
    MatXX H_ll_;
    VecX b_ll;

    // all vertices
    HashVertex vertices_;

    // all edges
    HashEdge edges_;

    // 由vertex id 查询edge
    HashVertexIdToEdge vertexToEdge_;

    // Ordering related
    ulong ordering_poses_ = 0;
    ulong ordering_landmarks_ = 0;
    ulong ordering_generic_ = 0;
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;        // 以 ordering 排序的 pose顶点
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices;     // 以 ordering 排序的 landmark 顶点

    // vertices need to marg. <Ordering_id_, Vertex>
    HashVertex vertices_marg_;

    bool bDebug = false;
    double t_hessian_cost_ = 0.0;
    double t_PCGsolve_cost_ = 0.0;


    // Solve的实现, 解通用问题
    bool SolveGenericProblem(int iterations);

    // Solve的实现, 解SLAM问题
    bool SolveSLAMProblem(int iterations);

    // 设置各顶点的ordering_index
    void SetOrdering();

    // set ordering for new vertex in slam problem
    void AddOrderingSLAM(std::shared_ptr<Vertex> v);

    // 构造大H矩阵
    void MakeHessian();

    // schur求解SBA
    void SchurSBA();

    // 解线性方程
    void SolveLinearSystem();

    // 更新状态变量
    void UpdateStates();

    void RollbackStates();  // 有时候 update 后残差会变大, 需要退回去, 重来

    // 计算并更新Prior部分
    void ComputePrior();


    // 判断一个顶点是否为Pose顶点
    bool IsPoseVertex();

    // 判断一个顶点是否为 landmark 顶点
    bool IsLandmarkVertex(std::shared_ptr<Vertex> v);


    // 在新增顶点后, 需要调整几个hessian的大小
    void ResizePoseHessianWhenAddingPose(std::shared_ptr<Vertex> v);

    // 检查ordering是否正确
    bool CheckOrdering();

    void LogoutVectorSize();

    // 获取某个顶点连接到的边
    std::vector<std::shared_ptr<Edge>> GetConnectedEdges(std::shared_ptr<Vertex> vertex);


    // Levenberg
    // 计算LM算法的初始Lambda
    void ComputeLambdaInitLM();

    // Hessian 对角线加上或者减去 Lambda
    void AddLambdatoHessianLM();

    void RemoveLambdaHessianLM();

    // LM算法中用于判断 Lambda 在上次迭代中 是否可以, 以及Lambda怎么缩放
    bool IsGoodStepInLM();

    // PCG 迭代线性求解器
    VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);

};

}

}

#endif // _INC_03_MYCURVEFITTING_LM_PROBLEM_H_
