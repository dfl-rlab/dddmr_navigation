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
#include <mpc_critics/collision_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::CollisionModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

CollisionModel::CollisionModel(){
  return;
  
}

void CollisionModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}

double CollisionModel::scoreTrajectory(base_trajectory::Trajectory &traj){
  
  if(shared_data_->pcl_perception_->points.size()<5){
    return 0.0;
  }
  


  for(unsigned int i=0;i<traj.getPosesSize();i++){

    base_trajectory::cuboid_min_max_t cmmt = traj.getCuboidMinMax(i);
    /*Assert for cuboids, we want user to have cuboids with the trajectory, otherwise it is dangerous*/
    assert( cmmt.first.x!=cmmt.second.x!=0 );
    
    pcl::PointXYZI pcl_traj_pose = traj.getPCLPoint(i);

    std::vector<int> id;
    std::vector<float> sqdist;
    //@ Calculate essential utils for point in cuboid test
    //@ See: https://math.stackexchange.com/questions/1472049/check-if-a-point-is-inside-a-rectangular-shaped-area-3d
    //@ See: https://stackoverflow.com/questions/52673935/check-if-3d-point-inside-a-box

    /*
    vec3 center; // Center of the box.
    vec3 dx, dy, dz; // X,Y, and Z directions, normalized.
    vec3 half; // Box size in each dimension, divided by 2.
    vec3 point; // Point to test.
    vec3 d = point - center;
    bool inside = abs(dot(d, dx)) <= half.x &&
                  abs(dot(d, dy)) <= half.y &&
                  abs(dot(d, dz)) <= half.z;
    */
    
    //@ Compute center of cuboid, leverage pcl::pointXYZ as vector/point
    pcl::PointCloud<pcl::PointXYZ> cuboid = traj.getCuboid(i);
    pcl::PointXYZ cuboid_center;
    cuboid_center.x = 0; cuboid_center.y = 0; cuboid_center.z = 0;
    for(auto a_pt = cuboid.points.begin(); a_pt!=cuboid.points.end(); a_pt++){
      cuboid_center.x += (*a_pt).x;
      cuboid_center.y += (*a_pt).y;
      cuboid_center.z += (*a_pt).z;
    }
    cuboid_center.x /= cuboid.points.size();
    cuboid_center.y /= cuboid.points.size();
    cuboid_center.z /= cuboid.points.size();
    //@ Compute dx, dy, dz. Also check --Start to parse cuboid-- section in dd_simple_trajectory_generator_theory.cpp
    pcl::PointXYZ dx;
    dx.x = cuboid.points[3].x - cuboid.points[0].x;
    dx.y = cuboid.points[3].y - cuboid.points[0].y;
    dx.z = cuboid.points[3].z - cuboid.points[0].z;

    pcl::PointXYZ dy;
    dy.x = cuboid.points[1].x - cuboid.points[0].x;
    dy.y = cuboid.points[1].y - cuboid.points[0].y;
    dy.z = cuboid.points[1].z - cuboid.points[0].z;

    pcl::PointXYZ dz;
    dz.x = cuboid.points[2].x - cuboid.points[0].x;
    dz.y = cuboid.points[2].y - cuboid.points[0].y;
    dz.z = cuboid.points[2].z - cuboid.points[0].z;

    double half_x, half_y, half_z;
    half_x = sqrt(dx.x*dx.x + dx.y*dx.y + dx.z*dx.z)/2.;
    half_y = sqrt(dy.x*dy.x + dy.y*dy.y + dy.z*dy.z)/2.;
    half_z = sqrt(dz.x*dz.x + dz.y*dz.y + dz.z*dz.z)/2.;

    dx.x/=(2.*half_x);dx.y/=(2.*half_x);dx.z/=(2.*half_x);
    dy.x/=(2.*half_y);dy.y/=(2.*half_y);dy.z/=(2.*half_y);
    dz.x/=(2.*half_z);dz.y/=(2.*half_z);dz.z/=(2.*half_z);

    //@The robot is not possible to be larger than 2 meters?
    shared_data_->pcl_perception_kdtree_->radiusSearch(pcl_traj_pose, 1.0, id, sqdist);
  
    for(auto pit=id.begin();pit!=id.end();pit++){
      auto pct_point = shared_data_->pcl_perception_->points[(*pit)];
      //@ Vector from center->point
      pcl::PointXYZ dp;
      dp.x = pct_point.x-cuboid_center.x;
      dp.y = pct_point.y-cuboid_center.y;
      dp.z = pct_point.z-cuboid_center.z;
      
      double x_value = fabs(dp.x * dx.x + dp.y * dx.y + dp.z * dx.z); //abs(dot(d, dx))
      double y_value = fabs(dp.x * dy.x + dp.y * dy.y + dp.z * dy.z); //abs(dot(d, dy))
      double z_value = fabs(dp.x * dz.x + dp.y * dz.y + dp.z * dz.z); //abs(dot(d, dz))

      if(x_value<=half_x && y_value<=half_y && z_value<=half_z)
      {
        return -1.0;
      }
      else{
        //@implement obstacle distance as rating?
        //traj.cost_ += obstacle_distance;
      }
    }
  }
  //@ there is obstacle, but the collision is passed, therefore return 0.0
  return 0.0;
}

}//end of name space
