/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <perception_3d/cluster_kdtree_marking.h>


namespace perception_3d
{

KDTreeMarking::~KDTreeMarking(){
  shared_data_.reset();
}

void KDTreeMarking::computeProjection(  
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& projectedptr){

  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (pcptr);
  proj.setModelCoefficients (pcplaneptr);
  proj.filter (*projectedptr);

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (projectedptr);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*projectedptr);

}

std::int16_t KDTreeMarking::safeConvert(int32_t large_value) {
  if (large_value > std::numeric_limits<std::int16_t>::max() || 
    large_value < std::numeric_limits<std::int16_t>::min()) {
    RCLCPP_ERROR(rclcpp::get_logger("cluster_kdtree_marking"),"DDDMR do not support map larger than +-1600m");
  }
  return static_cast<std::int16_t>(large_value);
}

void KDTreeMarking::addPCPtr(PointXYZU64 centroid,  
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr){

  centroid.xshort = safeConvert(centroid.x/xy_resolution_);
  centroid.yshort = safeConvert(centroid.y/xy_resolution_);
  centroid.zshort = safeConvert(centroid.z/height_resolution_);
  std::uint64_t pt_hash = int16ToUint64(centroid.xshort, centroid.yshort, centroid.zshort);
  pcl::PointCloud<pcl::PointXYZI>::Ptr projectedptr(new pcl::PointCloud<pcl::PointXYZI>);
  computeProjection(pcptr, pcplaneptr, projectedptr);
  
  auto it = marking_map_.find(pt_hash);
  if (it != marking_map_.end()) {
    //@ hash exist, do not push back pc
    per_marking pm(pcptr, pcplaneptr, projectedptr);
    marking_map_[pt_hash] = pm;
  } else {
    marking_pc_->push_back(centroid);
    per_marking pm(pcptr, pcplaneptr, projectedptr);
    marking_map_[pt_hash] = pm;
  }

}

void KDTreeMarking::removePCPtr(const PointXYZU64& centroid){

  std::uint64_t target_id = int16ToUint64(centroid.xshort, centroid.yshort, centroid.zshort);
  
  //@ Erase marking_map_ based on the input centroid
  auto map_it = marking_map_.find(target_id);
  if (map_it != marking_map_.end()) {
    map_it->second.pc_.reset();
    map_it->second.mc_.reset();
    marking_map_.erase(map_it);
  }
  
}

void KDTreeMarking::updateDGraph(const pcl::PointCloud<PointXYZU64>::Ptr& centroids_for_dgraph, 
                                  const std::vector<pcl::index_t>& ground_region_idx){
  //if ground region is empty
  //RCLCPP_INFO(rclcpp::get_logger("cluster_marking"),"ground region size: %lu", ground_region_idx.size());
  if(ground_region_idx.size()<1)
    return;

  //@clear all dgraph value in ground region
  for(auto idx_ground: ground_region_idx){
    dGraph_->clearValue(idx_ground, 9999.0);
    lethal_map_.erase(idx_ground);
  }

  //@loop marking_map_ to get projected point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_projections(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_projections_ds(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto a_pt: centroids_for_dgraph->points){
    std::uint64_t pt_hash = int16ToUint64(a_pt.xshort, a_pt.yshort, a_pt.zshort);
    *aggregated_projections += (*getMarkingCloudFromHash(pt_hash));
  }

  aggregated_projections_ds = small_gicp::voxelgrid_sampling_omp(*aggregated_projections, 0.1, 4);
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_aggregated_projections_ds(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  if(aggregated_projections_ds->points.size()>5){
    kdtree_aggregated_projections_ds->setInputCloud(aggregated_projections_ds);
  }
  else{
    RCLCPP_DEBUG(rclcpp::get_logger("cluster_marking"),"only few projection");
    return;
  }

  //@ loop ground and find closest one
  //RCLCPP_INFO(rclcpp::get_logger("cluster_marking"),"tic");
  std::vector<std::pair<pcl::index_t, double>> index_distance_pairs;

  #pragma omp parallel
  {
    std::vector<std::pair<pcl::index_t, double>> local_pairs;

    #pragma omp for nowait
    for (size_t i = 0; i < ground_region_idx.size(); ++i) {
      auto idx_ground = ground_region_idx[i];
      std::vector<int> id_tmp;
      std::vector<float> sqdist_tmp;
      kdtree_aggregated_projections_ds->nearestKSearch(shared_data_->pcl_ground_->points[idx_ground], 1, id_tmp, sqdist_tmp);
      double distance = sqrt(sqdist_tmp[0]);
      if(distance>inflation_radius_){
        //@ out of inflation range
      }
      else{
        dGraph_->setValue(idx_ground, distance);
        local_pairs.emplace_back(idx_ground, distance);
        //if(distance<=inscribed_radius_)
        //  lethal_map_[idx_ground] = distance;
      }
    }

    #pragma omp critical
    {
      index_distance_pairs.insert(index_distance_pairs.end(), local_pairs.begin(), local_pairs.end());
    }
  }

  for (const auto& pair : index_distance_pairs) {
    if (pair.second <= inscribed_radius_) {
      lethal_map_[pair.first] = pair.second;
    }
  }
  
  //RCLCPP_INFO(rclcpp::get_logger("cluster_marking"),"toc");
  /*
  //@ Below is the bottle neck of the marking, it can not be parallelized? Because we have to accessing a same address of dGrpah when inflate
  for(auto prj_pt_it=aggregated_projections_ds->points.begin();prj_pt_it!=aggregated_projections_ds->points.end();prj_pt_it++){
    pcl::PointXYZI pt;
    pt.x = (*prj_pt_it).x;
    pt.y = (*prj_pt_it).y;
    pt.z = (*prj_pt_it).z;

    std::vector<int> id_tmp;
    std::vector<float> sqdist_tmp;
    if(shared_data_->kdtree_ground_->radiusSearch(pt, inflation_radius_, id_tmp, sqdist_tmp)){
      //RCLCPP_INFO(rclcpp::get_logger("cluster_marking"),"small ground size: %lu", id_tmp.size());
      for(int i=0;i<id_tmp.size();i++){
        float dx = pt.x - shared_data_->pcl_ground_->points[id_tmp[i]].x;
        float dy = pt.y - shared_data_->pcl_ground_->points[id_tmp[i]].y;
        float dz = pt.z - shared_data_->pcl_ground_->points[id_tmp[i]].z;
        //@ Remove z value (see issue 8), we might have to review this assumption.
        float distance = sqrt(dx*dx + dy* dy + dz*dz);
        //RCLCPP_DEBUG(rclcpp::get_logger("cluster_marking"),"Distance xyz: %.2f, distance xy: %.2f", sqrt(sqdist_tmp[i]), distance);
        dGraph_->setValue(id_tmp[i], distance); //@setValue() will protect larger value being set, so we dont need to compare new and old
        if(distance<=inscribed_radius_)
          lethal_map_[id_tmp[i]] = distance;
      }
    }
  }
  */
}

void KDTreeMarking::updateKDTree(){
  if (marking_pc_->empty()){
    return;
  }
  kdtree_marking_.reset(new pcl::KdTreeFLANN<PointXYZU64>);
  kdtree_marking_->setInputCloud(marking_pc_);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KDTreeMarking::getMarkingCloudFromHash(std::uint64_t pt_hash){
  auto it = marking_map_.find(pt_hash);
  if (it != marking_map_.end()) {
    return marking_map_[pt_hash].pc_;
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    return empty_cloud;
  }
}

}//end of name space