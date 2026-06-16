//
// Created by hk on 3/26/23.
//
#include "dddmr_pcl/voxel_omp/voxel_grid_omp.h"
#include "Eigen/src/Core/Matrix.h"
#include "pcl/impl/point_types.hpp"
#include <algorithm>
#include <boost/sort/spreadsort/integer_sort.hpp>
#include <cstddef>
#include <cstring>
#include <limits>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <omp.h>
#include <cfloat>

namespace{
/*
等同于cloud_point_index_idx
cloud_point_index_idx没有默认构造函数，会导致boost的integer_sort编译不通过
*/
struct VoxelIndex{
    unsigned int idx;
    unsigned int cloud_point_index;

    VoxelIndex() = default;
    VoxelIndex (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
    bool operator < (const VoxelIndex &p) const { return (idx < p.idx); }
};
const int kSampleStep = 100;
/*
为每个线程分配它所处理的voxel index的上下界.
输入:
- index_all_thread.size()个有序vector，表示之前各个线程产生的voxel索引
- 需要为thread_num个线程分配任务
输出: 每个线程的任务
*/
size_t AssignTask(const std::vector<std::shared_ptr<std::vector<int>>>& index_all_thread, int thread_num, std::vector<std::vector<std::pair<int, int>>>* tasks){
    // 采样
    std::vector<int> sampled_data;
    size_t total = 0;
    for(auto& index: index_all_thread){
        total += index->size();
    }
    sampled_data.reserve(total/kSampleStep+index_all_thread.size()*2);
    for(auto& index: index_all_thread){
        size_t i=0;
        for(; i<index->size(); i+=kSampleStep){
            sampled_data.push_back((*index)[i]);
        }
        if(index->size()>0 && (index->size()-1)%kSampleStep>kSampleStep/2){
            sampled_data.push_back(index->back());
        }
    }
    // 确定每个进程分配voxel索引的上下界
    std::sort(sampled_data.begin(), sampled_data.end());
    tasks->resize(thread_num);
    int pad_size = sampled_data.size()%thread_num;
    int pivot_step = sampled_data.size()/thread_num;
    for(int i=0; i<thread_num; ++i){
        int start = pivot_step*i;
        start += (i>pad_size?pad_size:i);
        int pivot_start = sampled_data[start];
        int pivot_end;
        if(i+1<thread_num){
            int end = pivot_step*(i+1);
            end += (i+1>pad_size?pad_size:i+1);
            pivot_end = sampled_data[end];
        }
        auto& cur_task = (*tasks)[i];
        cur_task.resize(index_all_thread.size());
        for(size_t j=0; j<cur_task.size(); ++j){
            auto start_it = std::lower_bound(index_all_thread[j]->begin(), index_all_thread[j]->end(),pivot_start);
            cur_task[j].first = int(start_it - index_all_thread[j]->begin());
            if(i+1>=thread_num){
                cur_task[j].second = index_all_thread[j]->size();
            }else{
                auto end_it = std::lower_bound(index_all_thread[j]->begin(), index_all_thread[j]->end(),pivot_end);
                cur_task[j].second = int(end_it-index_all_thread[j]->begin());
            }
        }
    }
    return total;
}
}

void
pcl::VoxelGridOMP::setNumberOfThreads (unsigned int nr_threads)
{
    if (nr_threads == 0)
#ifdef _OPENMP
        threads_ = omp_get_num_procs();
#else
        threads_ = 1;
#endif
    else
        threads_ = nr_threads;
    printf("set number of threads: %d.\n", threads_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::VoxelGridOMP::applyFilter(PointCloud &output)
{
    //    TicToc t_bbox;
    // Has the input dataset been set already?
    if (!input_)
    {
        PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
        output.width = output.height = 0;
        output.points.clear ();
        return;
    }

    if (input_->points.size()<100)
    {
        output.width = input_->width;
        output.height = input_->height;
        output.points = input_->points;
        return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height       = 1;                    // downsampling breaks the organized structure
    output.is_dense     = true;                 // we filter out invalid points

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
        getMinMax3D<PointT> (input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
    else {
//        getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);
        getMinMax3DOMP(*input_, *indices_, min_p, max_p);
//        printf("min max OMP cost: %fms\n", t_bbox.toc());
    }

    // Check that the leaf size is not too small, given the size of the data
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

    if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
        PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
        output = *input_;
        return;
    }

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones (); // num of voxels in 3d space
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);
//    printf("bbox cost: %fms\n", t_bbox.toc());

    int centroid_size = 4; // centroid dimension
    if (downsample_all_data_) {
        centroid_size = boost::mpl::size<FieldList>::value; // dimension of all fields centroid
//        printf("downsample_all_data_\n");
    }

    // ---[ RGB special case
    std::vector<pcl::PCLPointField> fields;
    int rgba_index = -1;
    rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
    if (rgba_index == -1)
        rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
    if (rgba_index >= 0)
    {
        rgba_index = fields[rgba_index].offset;
        centroid_size += 3; // centroid  + 3 (rgb)
    }

    TicToc t_p2v;
    //global variables
    std::vector<PointCloudPtr> cloud_all_threads(threads_);
    for (size_t i = 0; i < threads_; ++i) {
        cloud_all_threads[i].reset(new PointCloud ());
    }
    std::vector<std::shared_ptr<std::vector<int>>> index_all_threads(threads_);
    std::vector<std::shared_ptr<std::vector<double>>> weight_all_threads(threads_);
    std::vector<std::shared_ptr<std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>>>> centroid_all_threads(threads_);
#pragma omp parallel num_threads(threads_)
    {
        int thread_id = omp_get_thread_num();
        PointCloudPtr& cloud_thread = cloud_all_threads[thread_id];
//        PointCloud cloud_thread;
        std::vector<VoxelIndex> index_vector; // voxel index, pointer of point
        index_vector.reserve(indices_->size() / threads_);

        int num_points = 0;
        // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
        if (!filter_field_name_.empty()) {
            // Get the distance field index
            std::vector<pcl::PCLPointField> fields;
            int distance_idx = pcl::getFieldIndex(*input_, filter_field_name_, fields);
            if (distance_idx == -1)
                PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName().c_str(),
                          distance_idx);

            // First pass: go over all points and insert them into the index_vector vector
            // with calculated idx. Points with the same idx value will contribute to the
            // same point of resulting CloudPoint

            int thread_id = omp_get_thread_num();

            for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); it += thread_id) {
                if (!input_->is_dense)
                    // Check if the point is invalid
                    if (!std::isfinite (input_->points[*it].x) ||
                        !std::isfinite (input_->points[*it].y) ||
                        !std::isfinite (input_->points[*it].z))
                        continue;

                // Get the distance value
                const uint8_t *pt_data = reinterpret_cast<const uint8_t *> (&input_->points[*it]);
                float distance_value = 0;
                memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

                if (filter_limit_negative_) {
                    // Use a threshold for cutting out points which inside the interval
                    if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                        continue;
                } else {
                    // Use a threshold for cutting out points which are too close/far away
                    if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                        continue;
                }

                int ijk0 = static_cast<int> (floor(input_->points[*it].x * inverse_leaf_size_[0]) -
                                             static_cast<float> (min_b_[0]));
                int ijk1 = static_cast<int> (floor(input_->points[*it].y * inverse_leaf_size_[1]) -
                                             static_cast<float> (min_b_[1]));
                int ijk2 = static_cast<int> (floor(input_->points[*it].z * inverse_leaf_size_[2]) -
                                             static_cast<float> (min_b_[2]));

                // Compute the centroid leaf index
                int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
                index_vector.push_back(VoxelIndex(static_cast<unsigned int> (idx), *it));
            }
        }
            // No distance filtering, process all data
        else {
            // First pass: go over all points and insert them into the index_vector vector
            // with calculated idx. Points with the same idx value will contribute to the
            // same point of resulting CloudPoint

#pragma omp for schedule(dynamic,1024)
            for (size_t i = 0; i < indices_->size(); ++i) {
                const int &point_id = indices_->at(i);
                const PointT &point = input_->points[point_id];
                if (!input_->is_dense)
                    // Check if the point is invalid
                    if (!std::isfinite (point.x) ||
                        !std::isfinite (point.y) ||
                        !std::isfinite (point.z))
                        continue;

                int ijk0 = static_cast<int> (floor(point.x * inverse_leaf_size_[0]) -
                                             static_cast<float> (min_b_[0])); // index of dimension 1
                int ijk1 = static_cast<int> (floor(point.y * inverse_leaf_size_[1]) -
                                             static_cast<float> (min_b_[1])); // index of dimension 2
                int ijk2 = static_cast<int> (floor(point.z * inverse_leaf_size_[2]) -
                                             static_cast<float> (min_b_[2])); // index of dimension 3

                // Compute the centroid leaf index
                int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2]; // voxel id the point located
                /// !!! so slow  !!!
                index_vector.emplace_back(
                        VoxelIndex(static_cast<unsigned int> (idx), point_id));
            }
        }
//        printf("thread %d points to voxel cost: %fms\n", thread_id, t_p2v.toc());

        /// !!! too slow!!!
//        TicToc t_sort;
        // Second pass: sort the index_vector vector using value representing target cell as index
        // in effect all points belonging to the same output cell will be next to each other
        auto rightshift_func = [](const VoxelIndex &x, const unsigned offset) { return x.idx >> offset; };
        boost::sort::spreadsort::integer_sort(index_vector.begin(), index_vector.end(), rightshift_func);
        // std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());
//        printf("thread %d sort cost: %fms\n", thread_id, t_sort.toc());

//    TicToc t_valid_voxel;
        // Third pass: count output cells
        // we need to skip all the same, adjacenent idx values
        unsigned int total = 0;
        unsigned int index = 0;
        // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
        // index_vector belonging to the voxel which corresponds to the i-th output point,
        // and of the first point not belonging to.
        std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
        // Worst case size
        first_and_last_indices_vector.reserve(index_vector.size());

        while (index < index_vector.size()) {
            unsigned int i = index + 1;
            while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
                ++i;
            //todo
            if (i - index >= min_points_per_voxel_)
            {
                ++total;
                first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
            }
            index = i;
        }
//        printf("thread %d compute valid voxel cost: %fms\n", thread_id, t_valid_voxel.toc());

        //    TicToc t_layout;
        // Fourth pass: compute centroids, insert them into their final position
//        cloud_thread.points.resize(total);
        size_t cur_size = first_and_last_indices_vector.size();
        auto indexes_one_thread = std::make_shared<std::vector<int>>(cur_size);
        auto weight_one_thread = std::make_shared<std::vector<double>>(cur_size);
        auto centroid_one_thread = std::make_shared<std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>>>(cur_size);
        for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
        {
            Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
            Eigen::VectorXf temporary = Eigen::VectorXf::Zero (centroid_size);
            // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
            unsigned int first_index = first_and_last_indices_vector[cp].first;
            unsigned int last_index = first_and_last_indices_vector[cp].second;
            int voxel_id = index_vector[first_index].idx;
            if (!downsample_all_data_) {
                centroid[0] = input_->points[index_vector[first_index].cloud_point_index].x;
                centroid[1] = input_->points[index_vector[first_index].cloud_point_index].y;
                centroid[2] = input_->points[index_vector[first_index].cloud_point_index].z;
            } else {
                // ---[ RGB special case
                if (rgba_index >= 0) {
                    // Fill r/g/b data, assuming that the order is BGRA
                    pcl::RGB rgb;
                    // copy memory from input to rgb
                    memcpy(&rgb,
                           reinterpret_cast<const char *> (&input_->points[index_vector[first_index].cloud_point_index]) +
                           rgba_index, sizeof(RGB));
                    centroid[centroid_size - 3] = rgb.r;
                    centroid[centroid_size - 2] = rgb.g;
                    centroid[centroid_size - 1] = rgb.b;
                }
                pcl::for_each_type<FieldList>(
                        NdCopyPointEigenFunctor<PointT>(input_->points[index_vector[first_index].cloud_point_index],
                                                        centroid));
            }

            for (unsigned int i = first_index + 1; i < last_index; ++i)
            {
                if (!downsample_all_data_) {
                    centroid[0] += input_->points[index_vector[i].cloud_point_index].x;
                    centroid[1] += input_->points[index_vector[i].cloud_point_index].y;
                    centroid[2] += input_->points[index_vector[i].cloud_point_index].z;
                } else {
                    // ---[ RGB special case
                    if (rgba_index >= 0) {
                        // Fill r/g/b data, assuming that the order is BGRA
                        pcl::RGB rgb;
                        memcpy(&rgb,
                               reinterpret_cast<const char *> (&input_->points[index_vector[i].cloud_point_index]) +
                               rgba_index, sizeof(RGB));
                        temporary[centroid_size - 3] = rgb.r;
                        temporary[centroid_size - 2] = rgb.g;
                        temporary[centroid_size - 1] = rgb.b;
                    }
                    // copying data between an Eigen type and a PointT (input point, output eigen)
                    pcl::for_each_type<FieldList>(
                            NdCopyPointEigenFunctor<PointT>(input_->points[index_vector[i].cloud_point_index],
                                                            temporary));
                    centroid += temporary; // accumulate centroid
                }
            }

            // index is centroid final position in resulting PointCloud
            if (save_leaf_layout_)
                leaf_layout_[index_vector[first_index].idx] = cp;

            (*centroid_one_thread)[cp] = centroid;
            (*weight_one_thread)[cp] = last_index-first_index;
            (*indexes_one_thread)[cp] = index_vector[first_index].idx;
        }
#pragma omp critical
        {
            centroid_all_threads[thread_id] = centroid_one_thread;
            weight_all_threads[thread_id] = weight_one_thread;
            index_all_threads[thread_id] = indexes_one_thread;
        }
//        printf("thread %d compute %d centroids cost: %fms\n", thread_id, (int)cloud_thread->points.size(), t_centriod.toc());
    }

    //merge cloud from all threads
    std::vector<std::vector<std::pair<int, int>>> tasks;
    size_t total = AssignTask(index_all_threads, threads_, &tasks);
    std::vector<PointCloudPtr> final_clouds(threads_);
#pragma omp parallel num_threads(threads_)
    {
        int thread_id = omp_get_thread_num();
        auto& task = tasks[thread_id];
        std::vector<int> cur(task.size());
        for(size_t i=0; i<cur.size(); ++i){
            cur[i]=task[i].first;
        }
        PointCloudPtr cloud(new PointCloud());
        cloud->reserve(total/threads_);
        while(true){
            bool finished = true;
            int min_voxel_index = std::numeric_limits<int>::max();
            for(size_t i=0; i<task.size(); ++i){
                if(cur[i]<task[i].second){
                    min_voxel_index = std::min(min_voxel_index, (*(index_all_threads[i]))[cur[i]]);
                    finished=false;
                }
            }
            if(finished) break;
            Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
            int weight = 0;
            for(size_t i=0; i<task.size(); ++i){
                if(cur[i]<task[i].second && (*(index_all_threads[i]))[cur[i]]==min_voxel_index){
                    centroid += (*(centroid_all_threads[i]))[cur[i]];
                    weight += (*(weight_all_threads[i]))[cur[i]];
                    ++cur[i];
                }
            }
            centroid /= float(weight);
            cloud->push_back(PointT());
            if (!downsample_all_data_){
                cloud->back().x = centroid[0];
                cloud->back().y = centroid[1];
                cloud->back().z = centroid[2];
            }else{
                //  NdCopyEigenPointFunctor( p1 the input Eigen type, p2 the output Point type)
                pcl::for_each_type<FieldList>(pcl::NdCopyEigenPointFunctor<PointT>(centroid, cloud->back()));
                // ---[ RGB special case
                if (rgba_index >= 0) {
                    // pack r/g/b into rgb
                    float r = centroid[centroid_size - 3], g = centroid[centroid_size - 2], b = centroid[centroid_size -
                                                                                                         1];
                    int rgb = (static_cast<int> (r) << 16) | (static_cast<int> (g) << 8) | static_cast<int> (b);
//                memcpy (reinterpret_cast<char*> (&output.points[index]) + rgba_index, &rgb, sizeof (float));
                    memcpy(reinterpret_cast<char *> (&(cloud->back())) + rgba_index, &rgb, sizeof(float));
                }
            }
        }
#pragma omp critical
        {
            final_clouds[thread_id] = cloud;
        }
    }
    size_t final_num = 0;
    for(auto& cloud: final_clouds){
        final_num += cloud->size();
    }
    output.resize(final_num);
    char* dst= reinterpret_cast<char*>(&output[0]);
    size_t offset = 0;
    for(auto& cloud: final_clouds){
        size_t copy_size = cloud->size()*sizeof(PointT);
        char* src = reinterpret_cast<char*>(&((cloud->points)[0]));
        std::memcpy(dst+offset, src, copy_size);
        offset+=copy_size;
    }
}

void pcl::VoxelGridOMP::getMinMax3DOMP(const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                                       Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
//    TicToc t_para;
    // prepare momery for all threads
    std::vector<Eigen::Vector4f> voxel_mins(threads_);
    std::vector<Eigen::Vector4f> voxel_maxs(threads_);

    // If the data is dense, we don't need to check for NaN
    if (cloud.is_dense)
    {
        #pragma omp parallel num_threads(threads_)
        {
            // thread private
            int thread_id = omp_get_thread_num();
            Eigen::Vector4f voxel_min_t, voxel_max_t;
            voxel_min_t.setConstant(FLT_MAX);
            voxel_max_t.setConstant(-FLT_MAX);

            // #pragma omp for
            #pragma omp for schedule(dynamic,1024)
            for (size_t i = 0; i < indices.size(); i++) {
                pcl::Array4fMapConst pt = cloud.points[indices[i]].getArray4fMap();
                voxel_min_t = voxel_min_t.array().min(pt);
                voxel_max_t = voxel_max_t.array().max(pt);
            }
            voxel_mins[thread_id] = voxel_min_t;
            voxel_maxs[thread_id] = voxel_max_t;
        }
    }
        // NaN or Inf values could exist => check for them
    else
    {
        // not be tested yet
        #pragma omp parallel num_threads(threads_)
        {
            int thread_id = omp_get_thread_num();
            Eigen::Vector4f voxel_min_t, voxel_max_t;
            voxel_min_t.setConstant(FLT_MAX); // thread private
            voxel_max_t.setConstant(-FLT_MAX);
//            #pragma omp single
            // #pragma omp for
            #pragma omp for schedule(dynamic,1024)
            for (size_t i = 0; i < indices.size(); i++) {
                // Check if the point is invalid
                if (!std::isfinite (cloud.points[indices[i]].x) ||
                    !std::isfinite (cloud.points[indices[i]].y) ||
                    !std::isfinite (cloud.points[indices[i]].z))
                    continue;
                pcl::Array4fMapConst pt = cloud.points[indices[i]].getArray4fMap();
                voxel_min_t = voxel_min_t.array().min(pt);
                voxel_max_t = voxel_max_t.array().max(pt);
            }

            voxel_mins[thread_id] = voxel_min_t;
            voxel_maxs[thread_id] = voxel_max_t;
        }
    }

    min_pt = voxel_mins[0];
    max_pt = voxel_maxs[0];
    for (size_t i = 1; i < threads_; ++i)
    {
        min_pt = min_pt.array().min(voxel_mins[i].array());
        max_pt = max_pt.array().max(voxel_maxs[i].array());
    }
//    printf("multi thread 4f cost: %fms\n", t_para.toc());
//    printf("min max:\n");
//    std::cout << min_pt.transpose() << std::endl;
//    std::cout << max_pt.transpose() << std::endl;
}