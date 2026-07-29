//
// Created by Zhang Zhimeng on 2021/3/24.
// Many thanks to https://github.com/zm0612
//

#ifndef OPTIMIZED_ICP_GN_H
#define OPTIMIZED_ICP_GN_H

#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>

// Required OpenMP header
#include <omp.h> 

class OptimizedICPGN {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;//eigen auto align
    OptimizedICPGN();

    bool SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr);

    bool Match(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr,
               const Eigen::Matrix4f &predict_pose,
               pcl::PointCloud<pcl::PointXYZI>::Ptr &transformed_source_cloud_ptr,
               Eigen::Matrix4f &result_pose);

    float GetFitnessScore(float max_range = std::numeric_limits<float>::max()) const;

    void SetMaxIterations(unsigned int iter);

    /*!
     * Max distance for nn search from the target cloud
     * note：this parameter has significant effect on the ICP result
     * @param max_correspond_distance
     */
    void SetMaxCorrespondDistance(float max_correspond_distance);

    /*!
     * Stop threshold for the iteration
     * @brief when the delta value is smaller than this value, it means the iteration is converged
     * @param transformation_epsilon
     */
    void SetTransformationEpsilon(float transformation_epsilon);

    bool HasConverged() const;

private:
    unsigned int max_iterations_{};
    float max_correspond_distance_{}, transformation_epsilon_{};

    bool has_converge_ = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_ptr_ = nullptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_ptr_ = nullptr;
    Eigen::Matrix4f final_transformation_;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_flann_ptr_ = nullptr;//In order to search
};

#endif //OPTIMIZED_ICP_GN_H
