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

}

void KDTreeMarking::computeMinDistanceFromObstacle2GroundNodes(  
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr,
  std::unordered_map<int, float>& nodes_of_min_distance){

  pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (pcptr);
  proj.setModelCoefficients (pcplaneptr);
  proj.filter (*projected_cloud_cluster);

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (projected_cloud_cluster);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*projected_cloud_cluster);

  for(auto prj_pt_it=projected_cloud_cluster->points.begin();prj_pt_it!=projected_cloud_cluster->points.end();prj_pt_it++){
    pcl::PointXYZI pt;
    pt.x = (*prj_pt_it).x;
    pt.y = (*prj_pt_it).y;
    pt.z = (*prj_pt_it).z;

    std::vector<int> id_tmp;
    std::vector<float> sqdist_tmp;
    //@ We mark lethal
    if(shared_data_->kdtree_ground_->radiusSearch(pt, inflation_radius_, id_tmp, sqdist_tmp)){
      for(int i=0;i<id_tmp.size();i++){
        float dx = pt.x - shared_data_->pcl_ground_->points[id_tmp[i]].x;
        float dy = pt.y - shared_data_->pcl_ground_->points[id_tmp[i]].y;
        float dz = pt.z - shared_data_->pcl_ground_->points[id_tmp[i]].z;
        //@ Remove z value (see issue 8), we might have to review this assumption.
        float distance = sqrt(dx*dx + dy* dy);
        //RCLCPP_DEBUG(rclcpp::get_logger("cluster_marking"),"Distance xyz: %.2f, distance xy: %.2f", sqrt(sqdist_tmp[i]), distance);
        //if(nodes_of_min_distance.insert(std::make_pair(id_tmp[i], sqrt(sqdist_tmp[i]))).second == false)
        if(nodes_of_min_distance.insert(std::make_pair(id_tmp[i], distance)).second == false)
        {
          //@ key was presented
          //nodes_of_min_distance[id_tmp[i]] = std::min(nodes_of_min_distance[id_tmp[i]], sqrt(sqdist_tmp[i]));
          nodes_of_min_distance[id_tmp[i]] = std::min(nodes_of_min_distance[id_tmp[i]], distance);
        }

      }
      
    }
  }

}


void KDTreeMarking::addPCPtr(PointXYZU64 centroid,  
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr){

  marking_id_++;
  centroid.hbyte = static_cast<std::uint32_t>(marking_id_ >> 32);
  centroid.lbyte = static_cast<std::uint32_t>(marking_id_ & 0xFFFFFFFF);
  marking_pc_->push_back(centroid);

  std::unordered_map<int, float> nodes_of_min_distance;
  computeMinDistanceFromObstacle2GroundNodes(pcptr, pcplaneptr, nodes_of_min_distance);
  per_marking pm(pcptr, pcplaneptr, nodes_of_min_distance);
  marking_map_[marking_id_] = pm;
  for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
    dGraph_->setValue((*id).first, (*id).second);
    if((*id).second<=inscribed_radius_)
      lethal_map_[(*id).first] = (*id).second;
  }

}

void KDTreeMarking::removePCPtr(const PointXYZU64& centroid){

  std::uint64_t target_id = mergeUint32ToUint64(centroid.hbyte, centroid.lbyte);

  //@ Clear corresponding values in dGraph_, lethal_map_, and marking_map_
  auto map_it = marking_map_.find(target_id);
  if (map_it != marking_map_.end()) {
    for (const auto& node : map_it->second.nodes_of_min_distance_) {
      dGraph_->clearValue(node.first, 9999.0);
      if (node.second <= inscribed_radius_) {
        lethal_map_.erase(node.first);
      }
    }
    map_it->second.pc_.reset();
    map_it->second.mc_.reset();
    marking_map_.erase(map_it);
  }
    
}


void KDTreeMarking::updateKDTree(){
  if (marking_pc_->empty()){
    return;
  }
  kdtree_marking_.reset(new pcl::KdTreeFLANN<PointXYZU64>);
  kdtree_marking_->setInputCloud(marking_pc_);
}

void KDTreeMarking::splitMarkingId(std::uint32_t& hbyte, std::uint32_t& lbyte){
  hbyte = static_cast<std::uint32_t>(marking_id_ >> 32);
  lbyte = static_cast<std::uint32_t>(marking_id_ & 0xFFFFFFFF);
}

void KDTreeMarking::splitUint64(std::uint64_t val, std::uint32_t& hbyte, std::uint32_t& lbyte){
  hbyte = static_cast<std::uint32_t>(val >> 32);
  lbyte = static_cast<std::uint32_t>(val & 0xFFFFFFFF);
}

std::uint64_t KDTreeMarking::mergeUint32ToUint64(std::uint32_t hbyte, std::uint32_t lbyte){
  return (static_cast<std::uint64_t>(hbyte) << 32) | static_cast<std::uint64_t>(lbyte);
}


}//end of name space