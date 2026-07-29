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
#ifndef PERCEPTION_3D_SHARED_DATA_H_
#define PERCEPTION_3D_SHARED_DATA_H_

#include "rclcpp/rclcpp.hpp"

/*For kd tree*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>


/*For planner*/
#include <perception_3d/dynamic_graph.h>
#include <perception_3d/static_graph.h>

namespace perception_3d
{

class SharedData{
  public:

    SharedData();
    
    //@ reset map center to request every one to update dynamic graph
    void requestAllLayersToResetDGraph();
    //@ return false if any false in dgraph_update_request
    bool isAllLayersBeenReset();
    
    //@ Variable that compute from local planner for path blocked check
    pcl::PointCloud<pcl::PointXYZI> pcl_prune_plan_;

    //@ Variable that loaded in static layer for other layer to use
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_map_; //thread safe kdtree
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_ground_; //thread safe kdtree
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ground_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map_;
    bool is_static_layer_ready_;
    std::map<std::string, bool> dgraph_update_request_;

    //@ Variable in static layer that inform new map cloud in coming
    size_t static_ground_size_;
    size_t static_map_size_;

    //@ Static graph in static layer for path planning
    std::shared_ptr<perception_3d::StaticGraph> sGraph_ptr_;    
    
    //@ We only make aggregate observation as shared
    //@ Add kd tree as shared make system unstable, because multi process are calling the tree
    //@ I am not able to make a more stable structure
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_observation_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_lethal_;

    //@ limit the current speed of trajectories
    double current_allowed_max_linear_speed_;
    
    //@ protect clear/mark/kdtree ground cb
    std::recursive_mutex ground_kdtree_cb_mutex_;
    
    bool mapping_mode_;
  private:



};


}//end of name space

#endif