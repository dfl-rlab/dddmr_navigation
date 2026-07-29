//
// Created by meng on 2021/3/25.
// Many thanks to https://github.com/zm0612
// OMP Implemented by Apola 2026/6/24
//
#include "opt_icp_gn/optimized_ICP_GN.h"
#include "opt_icp_gn/common.h"

OptimizedICPGN::OptimizedICPGN()
        : kdtree_flann_ptr_(new pcl::KdTreeFLANN<pcl::PointXYZI>) {
}

bool OptimizedICPGN::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr) {
    target_cloud_ptr_ = target_cloud_ptr;
    std::vector<int> indices;
    target_cloud_ptr_->is_dense = false;
    pcl::removeNaNFromPointCloud(*target_cloud_ptr_, *target_cloud_ptr_, indices);
    kdtree_flann_ptr_->setInputCloud(target_cloud_ptr);//create kd-tree for nn search
    return true;
}

bool OptimizedICPGN::Match(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr,
                           const Eigen::Matrix4f &predict_pose,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr &transformed_source_cloud_ptr,
                           Eigen::Matrix4f &result_pose) {
    has_converge_ = false;
    source_cloud_ptr_ = source_cloud_ptr;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f T = predict_pose;

    //Gauss-Newton's method solve ICP.
    for (unsigned int i = 0; i < max_iterations_; ++i) {
        pcl::transformPointCloud(*source_cloud_ptr, *transformed_cloud, T);
        Eigen::Matrix<float, 6, 6> Hessian = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> B = Eigen::Matrix<float, 6, 1>::Zero();
        
        if(transformed_cloud->size()>10000){
            //@ declare vector for omp
            std::vector<bool> valid_points(transformed_cloud->size());
            std::vector<Eigen::Matrix<float, 6, 6>> Hessians(transformed_cloud->size());
            std::vector<Eigen::Matrix<float, 6, 1>> Bs(transformed_cloud->size());

            //@ Omp tested using 932139 points, icp time from 2.926 to 2.330 on i9-11950 2.6G Hz
            //@ Omp tested using 371 points, tcp time from 0.001 to 0.008
            //@ conclusion: chose omp when points number larger than 10000
            #pragma omp parallel for
            for (unsigned int j = 0; j < transformed_cloud->size(); ++j) {
                const pcl::PointXYZI &origin_point = source_cloud_ptr->points[j];

                //delete inf
                if (!pcl::isFinite(origin_point)) {
                    valid_points[i] = false;
                    continue;
                }

                const pcl::PointXYZI &transformed_point = transformed_cloud->at(j);
                std::vector<float> resultant_distances;
                std::vector<int> indices;
                //find nearest point from the target cloud
                kdtree_flann_ptr_->nearestKSearch(transformed_point, 1, indices, resultant_distances);

                //return if the distance exceeding max_correspond_distance
                if (resultant_distances.front() > max_correspond_distance_) {
                    valid_points[i] = false;
                    continue;
                }

                Eigen::Vector3f nearest_point = Eigen::Vector3f(target_cloud_ptr_->at(indices.front()).x,
                                                                target_cloud_ptr_->at(indices.front()).y,
                                                                target_cloud_ptr_->at(indices.front()).z);

                Eigen::Vector3f point_eigen(transformed_point.x, transformed_point.y, transformed_point.z);
                Eigen::Vector3f origin_point_eigen(origin_point.x, origin_point.y, origin_point.z);
                Eigen::Vector3f error = point_eigen - nearest_point;

                Eigen::Matrix<float, 3, 6> Jacobian = Eigen::Matrix<float, 3, 6>::Zero();
                //create jacobian matrix
                Jacobian.leftCols(3) = Eigen::Matrix3f::Identity();
                Jacobian.rightCols(3) = -T.block<3, 3>(0, 0) * Hat(origin_point_eigen);

                //create hessian matrix
                //Hessian += Jacobian.transpose() * Jacobian;
                //B += -Jacobian.transpose() * error;
                Hessians[i] = Jacobian.transpose() * Jacobian;
                Bs[i] = -Jacobian.transpose() * error;
                valid_points[i] = true;
            }
            
            for(unsigned int j = 0; j < transformed_cloud->size(); ++j){
                if(valid_points[i]){
                    Hessian+=Hessians[i];
                    B+=Bs[i];
                }
            }
        }
        else{
            for (unsigned int j = 0; j < transformed_cloud->size(); ++j) {
                const pcl::PointXYZI &origin_point = source_cloud_ptr->points[j];

                //delete inf
                if (!pcl::isFinite(origin_point)) {
                    continue;
                }

                const pcl::PointXYZI &transformed_point = transformed_cloud->at(j);
                std::vector<float> resultant_distances;
                std::vector<int> indices;
                //find nearest point from the target cloud
                kdtree_flann_ptr_->nearestKSearch(transformed_point, 1, indices, resultant_distances);

                //return if the distance exceeding max_correspond_distance
                if (resultant_distances.front() > max_correspond_distance_) {
                    continue;
                }

                Eigen::Vector3f nearest_point = Eigen::Vector3f(target_cloud_ptr_->at(indices.front()).x,
                                                                target_cloud_ptr_->at(indices.front()).y,
                                                                target_cloud_ptr_->at(indices.front()).z);

                Eigen::Vector3f point_eigen(transformed_point.x, transformed_point.y, transformed_point.z);
                Eigen::Vector3f origin_point_eigen(origin_point.x, origin_point.y, origin_point.z);
                Eigen::Vector3f error = point_eigen - nearest_point;

                Eigen::Matrix<float, 3, 6> Jacobian = Eigen::Matrix<float, 3, 6>::Zero();
                //create jacobian matrix
                Jacobian.leftCols(3) = Eigen::Matrix3f::Identity();
                Jacobian.rightCols(3) = -T.block<3, 3>(0, 0) * Hat(origin_point_eigen);

                //create hessian matrix
                //@Weighted ICP, if weight of a point is introduced, then Hessian and B should include them.
                //@Hessian += w[j]*Jacobian.transpose() * Jacobian;
                //@B+=w[j]*-Jacobian.transpose() * error;
                Hessian += Jacobian.transpose() * Jacobian;
                B += -Jacobian.transpose() * error;
            }         
        }

        if (Hessian.determinant() == 0) {
            continue;
        }

        // BAD PRACTICE (by Gemini): Slow and numerically unstable
        // Eigen::Matrix<float, 6, 1> delta_x = Hessian.inverse() * B;
        // GOOD PRACTICE (by Gemini): Fast, accurate, and stable
        // LDLT is perfect for Hessian matrices because they are symmetric and positive semi-definite
        Eigen::Matrix<float, 6, 1> delta_x = Hessian.ldlt().solve(B);
        
        //@ What it means: The author is directly adding the translation increment to the existing translation vector.
        //@ The Math: If a transformation is defined as a coordinate shift \(T(\mathbf{p}) = R\mathbf{p} + \mathbf{t}\), 
        //@ the author is updating it as following: Because translation behaves linearly in Euclidean space, directly adding the components is mathematically valid and skips the overhead of full matrix multiplication.
        T.block<3, 1>(0, 3) = T.block<3, 1>(0, 3) + delta_x.head(3);

        //@ What it means: The author is applying a local (body-frame) rotation update.
        //@ The Math: Notice the *= operator. This expands to:\(R_{\text{new}}=R_{\text{old}}\cdot \Delta R\)
        T.block<3, 3>(0, 0) *= SO3Exp(delta_x.tail(3)).matrix();

        if (delta_x.norm() < transformation_epsilon_) {
            has_converge_ = true;
            break;
        }

        // debug
//        std::cout << "i= " << i << "  norm delta x= " << delta_x.norm() << std::endl;
    }

    final_transformation_ = T;
    result_pose = T;
    pcl::transformPointCloud(*source_cloud_ptr, *transformed_source_cloud_ptr, result_pose);

    return true;
}

/*
//@ This is another update step reference
// Assume these are your existing variables:
// Eigen::Matrix4f T_current; // Your current 4x4 estimation matrix
// Eigen::Vector6f delta;     // The [α, t]^T or [t, α]^T vector from the solver

void updateTransformation(Eigen::Matrix4f& T_current, const Eigen::Vector6f& delta) {
    // 1. Extract translation and rotation steps based on your block order.
    // If your Jacobian was [-Hat(p) | I], then delta is ordered as [Rotation, Translation]
    Eigen::Vector3f alpha = delta.head<3>(); // Rotational step (αx, αy, αz)
    Eigen::Vector3f t_step = delta.tail<3>(); // Translational step (tx, ty, tz)

    // Note: If you swapped your Jacobian to [I | -Hat(p)], use this instead:
    // Eigen::Vector3f t_step = delta.head<3>();
    // Eigen::Vector3f alpha  = delta.tail<3>();

    // 2. Convert the small rotation vector (alpha) into a valid 3x3 Rotation Matrix.
    // AngleAxis converts the magnitude to an angle and the normalized vector to an axis.
    float angle = alpha.norm();
    Eigen::Matrix3f R_update = Eigen::Matrix3f::Identity();
    
    if (angle > 1e-6f) { // Avoid division by zero if rotation step is near zero
        R_update = Eigen::AngleAxisf(angle, alpha / angle).toRotationMatrix();
    }

    // 3. Construct the incremental 4x4 transformation matrix (dT)
    Eigen::Matrix4f dT = Eigen::Matrix4f::Identity();
    dT.block<3, 3>(0, 0) = R_update;
    dT.block<3, 1>(0, 3) = t_step;

    // 4. Apply the global update from the LEFT side
    T_current = dT * T_current;
}
*/

float OptimizedICPGN::GetFitnessScore(float max_range) const {
    float fitness_score = 0.0f;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*source_cloud_ptr_, *transformed_cloud_ptr, final_transformation_);

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    int nr = 0;

    for (unsigned int i = 0; i < transformed_cloud_ptr->size(); ++i) {
        if(!pcl::isFinite(transformed_cloud_ptr->points[i]))
        continue;
        kdtree_flann_ptr_->nearestKSearch(transformed_cloud_ptr->points[i], 1, nn_indices, nn_dists);

        if (nn_dists.front() <= max_range) {
            fitness_score += nn_dists.front();
            nr++;
        }
    }

    if (nr > 0)
        return fitness_score / static_cast<float>(nr);
    else
        return (std::numeric_limits<float>::max());
}

bool OptimizedICPGN::HasConverged() const {
    return has_converge_;
}

void OptimizedICPGN::SetMaxIterations(unsigned int iter) {
    max_iterations_ = iter;
}

void OptimizedICPGN::SetMaxCorrespondDistance(float max_correspond_distance) {
    max_correspond_distance_ = max_correspond_distance;
}

void OptimizedICPGN::SetTransformationEpsilon(float transformation_epsilon) {
    transformation_epsilon_ = transformation_epsilon;
}
