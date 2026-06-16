//
// Created by hk on 3/26/23.
//

#pragma once

#ifndef PCL_FILTERS_VOXEL_GRID_OMP_H
#define PCL_FILTERS_VOXEL_GRID_OMP_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "tic_toc.h"

namespace pcl
{
    /** \brief VoxelGridOMP assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.,
      * in parallel, using the OpenMP standard.
      * \author Kai, Huang
      */
//    template<typename PointT>
class VoxelGridOMP : public VoxelGrid<PointXYZI>
    {

    public:
        typedef typename pcl::PointXYZI PointT;
        typedef typename Filter<PointT>::PointCloud PointCloud;
        typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

    public:
        /** \brief Initialize the scheduler and set the number of threads to use.
          * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
          */
        VoxelGridOMP (unsigned int nr_threads = 0) :
                console_print(false),
                threads_(nr_threads),
                final_filter(true)
                {}

        /** \brief Initialize the scheduler and set the number of threads to use.
          * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
          */
        void
        setNumberOfThreads (unsigned int nr_threads = 0);

        /** \brief Performing the final voxel grid filtering(PCL VoxelGrid).
          * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
          */
        void
        setFinalFilter (bool flag) {final_filter = flag;}

        bool console_print;
    protected:
        /** \brief The number of threads the scheduler should use. */
        unsigned int threads_;

        /** \brief Whether to perform the final voxel grid filtering(PCL VoxelGrid) for a strict result. */
        bool final_filter;
    private:
        /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions in a given pointcloud
          * \param cloud the point cloud data message
          * \param indices the vector of point indices to use from \a cloud
          * \param min_pt the resultant minimum bounds
          * \param max_pt the resultant maximum bounds
          * \ingroup common
          */
        void
        getMinMax3DOMP (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                          Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt);

        /** \brief Downsample a Point Cloud using a voxelized grid approach
          * \param[out] output the resultant point cloud message
          */
        void
        applyFilter (PointCloud &output) override;

    };
}


//
//#define PCL_INSTANTIATE_VoxelGrid(T) template class PCL_EXPORTS pcl::VoxelGrid<T>;
//#define PCL_INSTANTIATE_getMinMax3D(T) template PCL_EXPORTS void pcl::getMinMax3D<T> (const pcl::PointCloud<T>::ConstPtr &, const std::string &, float, float, Eigen::Vector4f &, Eigen::Vector4f &, bool);


#endif //PCL_FILTERS_VOXEL_GRID_OMP_H
