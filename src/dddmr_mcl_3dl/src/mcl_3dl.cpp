/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mcl_3dl.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

namespace mcl_3dl
{

MCL3dlNode::MCL3dlNode(std::string name) : Node(name) 
  , global_localization_fix_cnt_(0)
  , engine_(seed_gen_())
  , is_trans_b2s_initialized_(false)
  , tf_ready_(false)
  , first_tf_(false)
{
  //supress the no intensity found log
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  
  clock_ = this->get_clock();
}

MCL3dlNode::~MCL3dlNode(){
  //tf_publish_thread_.join();
}

bool MCL3dlNode::configure(const std::shared_ptr<mcl_3dl::SubMaps>& sub_maps)
{
  sub_maps_ = sub_maps;
  params_ = std::make_shared<Parameters>(this->get_node_logging_interface(), this->get_node_parameters_interface());
  sub_maps_->knn_num_of_ground_normals_ = params_->knn_num_of_ground_normals_;
  
  lidar_measurements_ = std::make_shared<LidarMeasurementModelLikelihood>();
  lidar_measurements_->loadConfig(this->get_node_logging_interface(), this->get_node_parameters_interface());

  pf_.reset(new pf::ParticleFilter<State6DOF,
                                    float,
                                    ParticleWeightedMeanQuat,
                                    std::default_random_engine>(params_->num_particles_));
  pf_->init(params_->initial_pose_, params_->initial_pose_std_);

  f_pos_.reset(new FilterVec3(
      Filter::FILTER_LPF,
      Vec3(params_->lpf_step_, params_->lpf_step_, params_->lpf_step_),
      Vec3()));
  f_ang_.reset(new FilterVec3(
      Filter::FILTER_LPF,
      Vec3(params_->lpf_step_, params_->lpf_step_, params_->lpf_step_),
      Vec3(), true));
  f_acc_.reset(new FilterVec3(
      Filter::FILTER_LPF,
      Vec3(params_->acc_lpf_step_, params_->acc_lpf_step_, params_->acc_lpf_step_),
      Vec3()));


  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_pub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  //@Initialize transform listener and broadcaster
  tfbuf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tfbuf_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 2,
      std::bind(&MCL3dlNode::cbOdom, this, std::placeholders::_1), sub_options);

  sub_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_3d_pose", 2,
      std::bind(&MCL3dlNode::cbPosition, this, std::placeholders::_1), sub_options);

  lc_sharp_.subscribe(this, "laser_cloud_sharp");
  lc_less_sharp_.subscribe(this, "laser_cloud_less_sharp");
  lc_flat_.subscribe(this, "laser_cloud_flat");
  lc_less_flat_.subscribe(this, "laser_cloud_less_flat");
  
  syncApproximate_ = std::make_shared<message_filters::Synchronizer<LegoSyncPolicy>>(LegoSyncPolicy(5), lc_sharp_, lc_less_sharp_, lc_flat_, lc_less_flat_);
  syncApproximate_->registerCallback(&MCL3dlNode::cbLeGoFeatureCloud, this);  
  
  tf_pub_timer_ = this->create_wall_timer(50ms, std::bind(&MCL3dlNode::publishTFThread, this), tf_pub_group_);

  pub_ground_normal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_normal", 1);  
  pub_pc_ec_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("euclidean_cluster_extraction", 1);
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", 1);
  pub_particle_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", 1);
  
  has_odom_ = false;

  motion_prediction_model_ = MotionPredictionModelBase::Ptr(
      new MotionPredictionModelDifferentialDrive(params_->odom_err_integ_lin_tc_,
                                                  params_->odom_err_integ_ang_tc_));

  return true;
  /*

  sub_landmark_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/landmark", 2, &MCL3dlNode::cbLandmark, this);

  pub_pc_normal_ = pnh_.advertise<visualization_msgs::MarkerArray>("normal_marker", 2, true);
  srv_expansion_reset_ = pnh_.advertiseService("expansion_resetting", &MCL3dlNode::cbExpansionReset, this);
  
  */
}

void MCL3dlNode::cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg){

  odom_ =
      State6DOF(
          Vec3(msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z),
          Quat(msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w));

  odom_header_ = msg->header;
  odom_trans_.header = msg->header;
  odom_trans_.child_frame_id = msg->child_frame_id;
  odom_trans_.transform.translation.x = msg->pose.pose.position.x;
  odom_trans_.transform.translation.y = msg->pose.pose.position.y;
  odom_trans_.transform.translation.z = msg->pose.pose.position.z;
  odom_trans_.transform.rotation.x = msg->pose.pose.orientation.x;
  odom_trans_.transform.rotation.y = msg->pose.pose.orientation.y;
  odom_trans_.transform.rotation.z = msg->pose.pose.orientation.z;
  odom_trans_.transform.rotation.w = msg->pose.pose.orientation.w;
  if(odom_header_.stamp.sec==0 && odom_header_.stamp.nanosec == 0){
    odom_trans_.header.stamp = clock_->now();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 3000, "Odometry msg.header.timestamp = 0, use clock.now() as the timestamp");
  }
  tfb_->sendTransform(odom_trans_);

  if (!has_odom_)
  {
    odom_prev_ = odom_;
    odom_last_ = msg->header.stamp;
    has_odom_ = true;
    return;
  }
  
  double dx = odom_.pos_.x_ - odom_prev_.pos_.x_;
  double dy = odom_.pos_.y_ - odom_prev_.pos_.y_;
  double dz = odom_.pos_.z_ - odom_prev_.pos_.z_;

  tf2::Quaternion q_odom(odom_.rot_.x_, odom_.rot_.y_, odom_.rot_.z_, odom_.rot_.w_);
  tf2::Quaternion q_odom_prev_(odom_prev_.rot_.x_, odom_prev_.rot_.y_, odom_prev_.rot_.z_, odom_prev_.rot_.w_);

  double roll_odom, pitch_odom, yaw_odom;
  tf2::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

  double roll_odom_prev, pitch_odom_prev, yaw_odom_prev;
  tf2::Matrix3x3(q_odom_prev_).getRPY(roll_odom_prev, pitch_odom_prev, yaw_odom_prev);

  double droll = roll_odom - roll_odom_prev;
  double dpitch = pitch_odom - pitch_odom_prev;
  double dyaw = yaw_odom - yaw_odom_prev;
  rclcpp::Time msg_time(msg->header.stamp);
  const double dt = msg_time.seconds() - odom_last_.seconds();
  //RCLCPP_WARN(this->get_logger(), "Verify me!! I am dt : %.2f",dt);

  if(!first_tf_ || sqrt(dx*dx + dy*dy + dz*dz)>params_->update_min_d_ || sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw)>params_->update_min_a_ ){
    if(pcl_segmentations_.empty()){
      return;
    }

    motion_prediction_model_->setOdoms(odom_prev_, odom_, dt);
    auto prediction_func = [this](State6DOF& s)
    {
      motion_prediction_model_->predict(s);
    };
    pf_->predict(prediction_func);
    rclcpp::Time last_msg_time(msg->header.stamp);
    odom_last_ = last_msg_time;
    odom_prev_ = odom_;
    
    std::unique_lock<std::mutex> lock(protect_measure_in_odomcb_);
    measure(pcl_segmentations_);

    pf_->resample(State6DOF(
    Vec3(params_->resample_var_x_,
          params_->resample_var_y_,
          params_->resample_var_z_),
    Vec3(params_->resample_var_roll_,
          params_->resample_var_pitch_,
          params_->resample_var_yaw_)));

    std::normal_distribution<float> noise(0.0, 1.0);
    auto update_noise_func = [this, &noise](State6DOF& s)
    {
      s.noise_ll_ = noise(engine_) * params_->odom_err_lin_lin_;
      s.noise_la_ = noise(engine_) * params_->odom_err_lin_ang_;
      s.noise_aa_ = noise(engine_) * params_->odom_err_ang_ang_;
      s.noise_al_ = noise(engine_) * params_->odom_err_ang_lin_;
    };
    pf_->predict(update_noise_func);

    publishParticles();
  }
}

bool MCL3dlNode::getBaselink2SensorAF3(std_msgs::msg::Header sensor_header, Eigen::Affine3d& trans_b2s_af3){

  if(! is_trans_b2s_initialized_){
  
    if(sensor_header.frame_id.at(0) == '/'){
      sensor_header.frame_id.erase(0, 1);
    }

    try
    {
      trans_b2s_ = tfbuf_->lookupTransform(
          params_->frame_ids_["base_link"], sensor_header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
      return false;
    }
    //@Transform into base_link frame, and then we can perform passthrough
    is_trans_b2s_initialized_ = true;
    trans_b2s_af3 = tf2::transformToEigen(trans_b2s_);
  }
  else{
    trans_b2s_af3 = tf2::transformToEigen(trans_b2s_);
  }
  return true;
}

void MCL3dlNode::cbLeGoFeatureCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc_sharpMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_less_sharpMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_flatMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_less_flatMsg){
  
  std::unique_lock<std::mutex> lock(protect_measure_in_odomcb_);
  
  laser_header_ = pc_less_sharpMsg->header;

  if(!sub_maps_->isCurrentReady())
    return;

  Eigen::Affine3d trans_b2s_af3;
  if(! getBaselink2SensorAF3(pc_less_sharpMsg->header, trans_b2s_af3))
    return;

  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_sharp(new pcl::PointCloud<mcl_3dl::pcl_t>);
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_sharp(new pcl::PointCloud<mcl_3dl::pcl_t>);
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_flat(new pcl::PointCloud<mcl_3dl::pcl_t>); 
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_flat(new pcl::PointCloud<mcl_3dl::pcl_t>);

  //@
  //@--->pc_less_flat comprises flat
  //@--->pc_less_sharp comprises sharp
  //However pc_less_flat comprises too many points causing compution overhead
  

  //pcl::fromROSMsg(*pc_sharpMsg, *pc_sharp);
  pcl::fromROSMsg(*pc_flatMsg, *pc_flat);
  pcl::fromROSMsg(*pc_less_sharpMsg, *pc_less_sharp);
  //pcl::fromROSMsg(*pc_less_flatMsg, *pc_less_flat);

  //@Transform point cloud
  //pcl::transformPointCloud(*pc_sharp, *pc_sharp, trans_b2s_af3);
  pcl::transformPointCloud(*pc_less_sharp, *pc_less_sharp, trans_b2s_af3);
  pcl::transformPointCloud(*pc_flat, *pc_flat, trans_b2s_af3);
  pcl::transformPointCloud(*pc_less_flat, *pc_less_flat, trans_b2s_af3);
  
  //*pc_less_sharp+=*pc_less_flat;

  pc_less_sharp->header.frame_id = params_->frame_ids_["base_link"];

  RCLCPP_DEBUG(this->get_logger(), "Size pc_sharp: %lu, pc_less_sharp: %lu, pc_flat: %lu, pc_less_flat: %lu", 
                          pc_sharp->points.size(), pc_less_sharp->points.size(), pc_flat->points.size(), pc_less_flat->points.size());


  //Ground will skew the result when close to obstacle, because velodyne has blind spot of 50 cm
  pcl::VoxelGrid<mcl_3dl::pcl_t> sor;
  sor.setInputCloud (pc_flat);
  sor.setLeafSize (1.0f, 1.0f, 0.1f);
  sor.filter (*pc_flat);
  
  //@Looks like downsample less sharp only 600->550 points, not very effective
  //pcl::VoxelGrid<mcl_3dl::pcl_t> sor2;
  //sor2.setInputCloud (pc_less_sharp);
  //sor2.setLeafSize (0.1f, 0.1f, 0.1f);
  //sor2.filter (*pc_less_sharp);

  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_sharp_intensity(new pcl::PointCloud<mcl_3dl::pcl_t>);

  //@Normal estimation on observation
  pcl::PointCloud<pcl::Normal>::Ptr observation_normals;
  observation_normals.reset(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n2;
  pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree2 (new pcl::search::KdTree<mcl_3dl::pcl_t>);
  tree2->setInputCloud (pc_less_sharp);
  n2.setInputCloud (pc_less_sharp);
  n2.setSearchMethod (tree2);
  n2.setKSearch (5);
  //n2.setRadiusSearch (0.1);
  n2.compute (*observation_normals);
  
  //@ Uncomment for visualization purpose
  //pcl::PointCloud<pcl::PointXYZ> observation_xyz;
  //pcl::copyPointCloud(*pc_less_sharp, observation_xyz);
  //pcl::PointCloud<pcl::PointNormal>::Ptr observation_points_normal;
  //observation_points_normal.reset(new pcl::PointCloud<pcl::PointNormal>);
  //pcl::concatenateFields (observation_xyz, *observation_normals, *observation_points_normal);
  //normal2quaternion(observation_points_normal);
  

  double sum_normal_y = 0;
  double sum_normal_x = 0;
  for(auto it=observation_normals->points.begin(); it!=observation_normals->points.end(); it++){
    //@ compute the normals statistic
    if(!isnan((*it).normal_y))
      sum_normal_y += fabs((*it).normal_y);
    if(!isnan((*it).normal_x))
      sum_normal_x += fabs((*it).normal_x);
  }

  //@Check normal skew
  bool x_dominant = false;
  bool y_dominant = false;
  if(sum_normal_x/sum_normal_y>=1.6){
    RCLCPP_DEBUG(this->get_logger(), "Environment is x dominant: %.2f, %.2f", sum_normal_x, sum_normal_y);
    x_dominant = true;
  }
  else if(sum_normal_y/sum_normal_x>=1.6){
    RCLCPP_DEBUG(this->get_logger(), "Environment is y dominant: %.2f, %.2f", sum_normal_x, sum_normal_y);
    y_dominant = true;
  }

  if(x_dominant || y_dominant){
    size_t normal_index = 0;
    for(auto a_pt=pc_less_sharp->points.begin();a_pt!=pc_less_sharp->points.end();a_pt++){

      pcl::PointXYZI i_pt;
      i_pt.x = (*a_pt).x;
      i_pt.y = (*a_pt).y;
      i_pt.z = (*a_pt).z;   

      if(x_dominant){
        if(isnan(observation_normals->points[normal_index].normal_x)){
          i_pt.intensity = 1.0;
        }
        else{
          double y2x = observation_normals->points[normal_index].normal_y/observation_normals->points[normal_index].normal_x;
          //@cap by the ratio of sum_normal_y:sum_normal_x
          if(y2x>=0.5)
            i_pt.intensity = 0.05*sum_normal_y/sum_normal_x;
          else
            i_pt.intensity = 1.0;
        }
      }
      else if(y_dominant){
        if(isnan(observation_normals->points[normal_index].normal_x)){
          i_pt.intensity = 1.0;
        }
        else{
          double x2y = observation_normals->points[normal_index].normal_x/observation_normals->points[normal_index].normal_y;
          if(x2y>=0.5)
            i_pt.intensity = 0.05*sum_normal_x/sum_normal_y;
          else
            i_pt.intensity = 1.0;
        }      
      }
      else{
        i_pt.intensity = 1.0;
      }

      pc_less_sharp_intensity->push_back(i_pt);
      normal_index++;
    }
  }
  else{
    //@ Euclidean Distance Segmentation on less_sharp
    //@ We want to extract isolated object and give the obj the same weight instead of by each point
    //@ max size is set to 2400 for 30 degree observation as an object instead of whole object ex: a long wall
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr pc_kdtree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    pc_kdtree->setInputCloud (pc_less_sharp);

    std::vector<pcl::PointIndices> cluster_indices_segmentation;
    pcl::EuclideanClusterExtraction<mcl_3dl::pcl_t> ec_segmentation;
    ec_segmentation.setClusterTolerance (params_->euc_cluster_distance_);
    ec_segmentation.setMinClusterSize (params_->euc_cluster_min_size_);
    ec_segmentation.setMaxClusterSize (pc_less_sharp->points.size());
    ec_segmentation.setSearchMethod (pc_kdtree);
    ec_segmentation.setInputCloud (pc_less_sharp);
    ec_segmentation.extract (cluster_indices_segmentation);

    int smaller_cluster_amount = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_segmentation.begin (); it != cluster_indices_segmentation.end (); ++it)
    {
        
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        pcl::PointXYZI i_pt;
        i_pt.x = pc_less_sharp->points[*pit].x;
        i_pt.y = pc_less_sharp->points[*pit].y;
        i_pt.z = pc_less_sharp->points[*pit].z;
        //@ if cluster size is small, it is like a beam, therefore we higher its weight
        if(it->indices.size() < params_->euc_cluster_min_size_+1){
          smaller_cluster_amount++;
          i_pt.intensity = (1.0*it->indices.size())/(1.0*pc_less_sharp->points.size())/2.0;
        }
        else{
          i_pt.intensity = (1.0*it->indices.size())/(1.0*pc_less_sharp->points.size());
        }
        
        pc_less_sharp_intensity->push_back(i_pt);
        //RCLCPP_DEBUG(this->get_logger(), "%.2f,%.2f,%.2f,%.4f", i_pt.x, i_pt.y, i_pt.z, i_pt.intensity);
      } 
    }
    RCLCPP_DEBUG(this->get_logger(), "Total clusters: %lu, small clusters: %d",  cluster_indices_segmentation.size(), smaller_cluster_amount);
  }

  RCLCPP_DEBUG(this->get_logger(), "Size pc_sharp: %lu, pc_less_sharp: %lu, pc_flat: %lu, pc_less_flat: %lu", 
                          pc_sharp->points.size(), pc_less_sharp->points.size(), pc_flat->points.size(), pc_less_flat->points.size());

  //@
  //We need perpendicular features and surface.
  //Surface can correct roll and pitch
  //Perpendicular can correct yaw
  
  if (true){
    sensor_msgs::msg::PointCloud2 ec_out;
    pcl::toROSMsg(*pc_less_sharp_intensity, ec_out);
    ec_out.header = pc_less_sharpMsg->header;
    ec_out.header.frame_id = params_->frame_ids_["base_link"];
    pub_pc_ec_->publish(ec_out);
  }
  
  pcl_segmentations_[std::string("flat")] = pc_flat;
  pcl_segmentations_[std::string("less_sharp")] = pc_less_sharp_intensity;
  
}

void MCL3dlNode::measure(std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr> pcl_segmentations)
{

  if(!sub_maps_->isCurrentReady())
    return;

  if(sub_maps_->isWarmUpReady()){
    sub_maps_->swapKdTree();
  }

  const auto ts = std::chrono::high_resolution_clock::now();
  lidar_measurements_->setGlobalLocalizationStatus(
      params_->num_particles_, pf_->getParticleSize());

  float match_ratio_min = 1.0;
  float match_ratio_max = 0.0;
  auto measure_func = [this, &pcl_segmentations,
                        &match_ratio_min, &match_ratio_max](const State6DOF& s) -> float
  {
    float qualities;
    const LidarMeasurementResult result = lidar_measurements_->measure(
        sub_maps_->kdtree_map_current_, sub_maps_->kdtree_ground_current_, sub_maps_->normals_ground_current_, pcl_segmentations, s);

    qualities = result.quality;
    if (match_ratio_min > qualities)
      match_ratio_min = qualities;
    if (match_ratio_max < qualities)
      match_ratio_max = qualities;
    return result.likelihood;
  };

  //@ pf_->measure(measure_func) will loop particles
  pf_->measure(measure_func);

  //@ This block first calculate the weight (p.probability_bias_) based on the particle state
  //@ It means that the particle away from last pose has less weight
  //@ This weight is different from the weight of likelihood
  if (static_cast<int>(pf_->getParticleSize()) > params_->num_particles_)
  {
    auto bias_func = [](const State6DOF& s, float& p_bias) -> void
    {
      p_bias = 1.0;
    };
    pf_->bias(bias_func);
  }
  else
  {
    NormalLikelihood<float> nl_lin(params_->bias_var_dist_);
    NormalLikelihood<float> nl_ang(params_->bias_var_ang_);
    auto bias_func = [this, &nl_lin, &nl_ang](const State6DOF& s, float& p_bias) -> void
    {
      const float lin_diff = (s.pos_ - state_prev_.pos_).norm();
      Vec3 axis;
      float ang_diff;
      (s.rot_ * state_prev_.rot_.inv()).getAxisAng(axis, ang_diff);
      p_bias = nl_lin(lin_diff) * nl_ang(ang_diff) + 1e-6;
      assert(std::isfinite(p_bias));
    };
    //@ generate p.probability_bias_
    pf_->bias(bias_func);
  }

  //@ Weight particle based on the observation weight and p.probability_bias_
  auto e = pf_->expectationBiased();
  const auto e_max = pf_->max();

  assert(std::isfinite(e.pos_.x_));
  assert(std::isfinite(e.pos_.y_));
  assert(std::isfinite(e.pos_.z_));
  assert(std::isfinite(e.rot_.x_));
  assert(std::isfinite(e.rot_.y_));
  assert(std::isfinite(e.rot_.z_));
  assert(std::isfinite(e.rot_.w_));

  e.rot_.normalize();

  Vec3 map_pos;
  Quat map_rot;
  map_pos = e.pos_ - e.rot_ * odom_.rot_.inv() * odom_.pos_;
  map_rot = e.rot_ * odom_.rot_.inv();

  bool jump = false;
  if (static_cast<int>(pf_->getParticleSize()) > params_->num_particles_)
  {
    jump = true;
    state_prev_ = e;
  }
  else
  {
    Vec3 jump_axis;
    float jump_ang;
    float jump_dist = (e.pos_ - state_prev_.pos_).norm();
    (e.rot_.inv() * state_prev_.rot_).getAxisAng(jump_axis, jump_ang);
    if (jump_dist > params_->jump_dist_ ||
        fabs(jump_ang) > params_->jump_ang_)
    {
      RCLCPP_INFO(this->get_logger(), "Pose jumped pos:%0.3f, ang:%0.3f", jump_dist, jump_ang);
      jump = true;

      auto integ_reset_func = [](State6DOF& s)
      {
        s.odom_err_integ_lin_ = Vec3();
        s.odom_err_integ_ang_ = Vec3();
      };
      pf_->predict(integ_reset_func);
    }
    state_prev_ = e;
  }

  
  map2odom_trans_.header.stamp = laser_header_.stamp;
  map2odom_trans_.header.frame_id = params_->frame_ids_["map"];
  map2odom_trans_.child_frame_id = params_->frame_ids_["odom"];
  const auto rpy = map_rot.getRPY();
  if (jump)
  {
    f_ang_->set(rpy);
    f_pos_->set(map_pos);
  }
  map_rot.setRPY(f_ang_->in(rpy));
  map_pos = f_pos_->in(map_pos);
  map2odom_trans_.transform.translation = tf2::toMsg(tf2::Vector3(map_pos.x_, map_pos.y_, map_pos.z_));
  map2odom_trans_.transform.rotation = tf2::toMsg(tf2::Quaternion(map_rot.x_, map_rot.y_, map_rot.z_, map_rot.w_));
  tf_ready_ = true;

  // Calculate covariance from sampled particles to reduce calculation cost on global localization.
  // Use the number of original particles or at least 10% of full particles.
  auto cov = pf_->covariance(
      1.0,
      std::max(
          0.1f, static_cast<float>(params_->num_particles_) / pf_->getParticleSize()));

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = odom_last_;
  pose.header.frame_id = map2odom_trans_.header.frame_id;
  pose.pose.pose.position.x = e.pos_.x_;
  pose.pose.pose.position.y = e.pos_.y_;
  pose.pose.pose.position.z = e.pos_.z_;
  pose.pose.pose.orientation.x = e.rot_.x_;
  pose.pose.pose.orientation.y = e.rot_.y_;
  pose.pose.pose.orientation.z = e.rot_.z_;
  pose.pose.pose.orientation.w = e.rot_.w_;
  for (size_t i = 0; i < 36; i++)
  {
    pose.pose.covariance[i] = cov[i / 6][i % 6];
  }
  pub_pose_->publish(pose);
  
  //--------------------------update pose to submap
  std::unique_lock<mcl_3dl::SubMaps::sub_maps_mutex_t> lock(*(sub_maps_->getMutex()));
  sub_maps_->setPose(pose);

  const auto tnow = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "MCL (%0.3f sec.)",
            std::chrono::duration<float>(tnow - ts).count());
  const auto err_integ_map = e_max.rot_ * e_max.odom_err_integ_lin_;
  RCLCPP_DEBUG(this->get_logger(),"odom error integral lin: %0.3f, %0.3f, %0.3f, "
            "ang: %0.3f, %0.3f, %0.3f, "
            "pos: %0.3f, %0.3f, %0.3f, "
            "err on map: %0.3f, %0.3f, %0.3f",
            e_max.odom_err_integ_lin_.x_,
            e_max.odom_err_integ_lin_.y_,
            e_max.odom_err_integ_lin_.z_,
            e_max.odom_err_integ_ang_.x_,
            e_max.odom_err_integ_ang_.y_,
            e_max.odom_err_integ_ang_.z_,
            e_max.pos_.x_,
            e_max.pos_.y_,
            e_max.pos_.z_,
            err_integ_map.x_,
            err_integ_map.y_,
            err_integ_map.z_);
  RCLCPP_DEBUG(this->get_logger(),"match ratio min: %0.3f, max: %0.3f, pos: %0.3f, %0.3f, %0.3f",
            match_ratio_min,
            match_ratio_max,
            e.pos_.x_,
            e.pos_.y_,
            e.pos_.z_);

  if (match_ratio_max < params_->match_ratio_thresh_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 3000, "Low match_ratio. Expansion resetting.");
    pf_->noise(State6DOF(
        Vec3(params_->expansion_var_x_,
              params_->expansion_var_y_,
              params_->expansion_var_z_),
        Vec3(params_->expansion_var_roll_,
              params_->expansion_var_pitch_,
              params_->expansion_var_yaw_)));
  }

  if (static_cast<int>(pf_->getParticleSize()) > params_->num_particles_)
  {
    const int reduced = pf_->getParticleSize() * 0.75;
    if (reduced > params_->num_particles_)
    {
      pf_->resizeParticle(reduced);
    }
    else
    {
      pf_->resizeParticle(params_->num_particles_);
    }
    // wait 99.7% fix (three-sigma)
    global_localization_fix_cnt_ = 1 + std::ceil(params_->lpf_step_) * 3.0;
  }
  if (global_localization_fix_cnt_)
  {
    global_localization_fix_cnt_--;
  }

}

void MCL3dlNode::publishParticles()
{
  geometry_msgs::msg::PoseArray pa;
  pa.header.stamp = odom_header_.stamp;
  pa.header.frame_id = params_->frame_ids_["map"];
  for (size_t i = 0; i < pf_->getParticleSize(); i++)
  {
    geometry_msgs::msg::Pose pm;
    auto p = pf_->getParticle(i);
    p.rot_.normalize();
    pm.position.x = p.pos_.x_;
    pm.position.y = p.pos_.y_;
    pm.position.z = p.pos_.z_;
    pm.orientation.x = p.rot_.x_;
    pm.orientation.y = p.rot_.y_;
    pm.orientation.z = p.rot_.z_;
    pm.orientation.w = p.rot_.w_;
    pa.poses.push_back(pm);
  }
  pub_particle_->publish(pa);
}

void MCL3dlNode::publishTFThread()
{
  if (tf_ready_ && params_->publish_tf_){
    if(laser_header_.stamp.sec==0 && laser_header_.stamp.nanosec == 0){
      laser_header_.stamp = odom_header_.stamp;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 3000, "Laser msg.header.timestamp = 0, use odom stamp as the timestamp");
    }
    map2odom_trans_.header.stamp = laser_header_.stamp;
    tfb_->sendTransform(map2odom_trans_);
    first_tf_ = true;
  }
}

void MCL3dlNode::cbPosition(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
  const double len2 =
      msg->pose.pose.orientation.x * msg->pose.pose.orientation.x +
      msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z +
      msg->pose.pose.orientation.w * msg->pose.pose.orientation.w;
  if (std::abs(len2 - 1.0) > 0.1)
  {
    RCLCPP_ERROR(this->get_logger(), "Discarded invalid initialpose. The orientation must be unit quaternion.");
    return;
  }
  
  if(msg->header.frame_id!=params_->frame_ids_["map"]){
    RCLCPP_ERROR(this->get_logger(), "Discarded initialpose. The frame_id should be: %s", params_->frame_ids_["map"].c_str());
    return;
  }

  sub_maps_->setInitialPose(*msg);
  while(rclcpp::ok() && !sub_maps_->isWarmUpReady()){

  }
  sub_maps_->swapKdTree();
  
  //@Try to find the ground
  geometry_msgs::msg::PoseStamped pose;
  mcl_3dl::pcl_t initial_pose_pt;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  initial_pose_pt.x = msg->pose.pose.position.x;
  initial_pose_pt.y = msg->pose.pose.position.y;
  initial_pose_pt.z = msg->pose.pose.position.z;
  pose.pose.position.x = initial_pose_pt.x;
  pose.pose.position.y = initial_pose_pt.y;
  pose.pose.orientation = msg->pose.pose.orientation;
  
  for(double z=0.0; z<5.0;z+=0.1){
    initial_pose_pt.z = msg->pose.pose.position.z + z;
    if(sub_maps_->kdtree_ground_current_.radiusSearch(initial_pose_pt, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1)>0)
    {
      pose.pose.position.z = sub_maps_->ground_current_->points[pointIdxRadiusSearch[0]].z;
      RCLCPP_INFO(this->get_logger(), "Found ground at z: %.2f", pose.pose.position.z);
      break;
    }
    initial_pose_pt.z = msg->pose.pose.position.z - z;
    if(sub_maps_->kdtree_ground_current_.radiusSearch(initial_pose_pt, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1)>0)
    {
      pose.pose.position.z = sub_maps_->ground_current_->points[pointIdxRadiusSearch[0]].z;
      RCLCPP_INFO(this->get_logger(), "Found ground at z: %.2f", pose.pose.position.z);
      break;
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Set initial pose at: %.2f, %.2f, %.2f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  const State6DOF mean(Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                        Quat(pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w));
  const MultivariateNoiseGenerator<float> noise_gen(mean, msg->pose.covariance);
  pf_->initUsingNoiseGenerator(noise_gen);

  auto integ_reset_func = [](State6DOF& s)
  {
    s.odom_err_integ_lin_ = Vec3();
    s.odom_err_integ_ang_ = Vec3();
  };
  pf_->predict(integ_reset_func);

  publishParticles();
  first_tf_ = false;
}

/*
void MCL3dlNode::cbLandmark(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  NormalLikelihoodNd<float, 6> nd(
      Eigen::Matrix<double, 6, 6>(
          msg->pose.covariance.data())
          .cast<float>());
  const State6DOF measured(
      Vec3(msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z),
      Quat(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w));
  auto measure_func = [this, &measured, &nd](const State6DOF& s) -> float
  {
    State6DOF diff = s - measured;
    const Vec3 rot_rpy = diff.rot_.getRPY();
    const Eigen::Matrix<float, 6, 1> diff_vec =
        (Eigen::MatrixXf(6, 1) << diff.pos_.x_,
          diff.pos_.y_,
          diff.pos_.z_,
          rot_rpy.x_,
          rot_rpy.y_,
          rot_rpy.z_)
            .finished();

    const auto n = nd(diff_vec);
    return n;
  };
  pf_->measure(measure_func);

  pf_->resample(State6DOF(
      Vec3(params_->resample_var_x_,
            params_->resample_var_y_,
            params_->resample_var_z_),
      Vec3(params_->resample_var_roll_,
            params_->resample_var_pitch_,
            params_->resample_var_yaw_)));

  publishParticles();
}
*/
/*
bool cbResizeParticle(mcl_3dl_msgs::ResizeParticleRequest& request,
                      mcl_3dl_msgs::ResizeParticleResponse& response)
{
  pf_->resizeParticle(request.size);
  publishParticles();
  return true;
}
*/
/*
bool MCL3dlNode::cbExpansionReset(std_srvs::TriggerRequest& request,
                      std_srvs::TriggerResponse& response)
{
  pf_->noise(State6DOF(
      Vec3(params_.expansion_var_x_,
            params_.expansion_var_y_,
            params_.expansion_var_z_),
      Vec3(params_.expansion_var_roll_,
            params_.expansion_var_pitch_,
            params_.expansion_var_yaw_)));
  publishParticles();
  return true;
}
*/




/*
void MCL3dlNode::normal2quaternion(pcl::PointCloud<pcl::PointNormal>::Ptr i_normals){

  visualization_msgs::MarkerArray markerArray;
  for(size_t i=0;i<i_normals->points.size();i++){

    tf2::Vector3 axis_vector(i_normals->points[i].normal_x, i_normals->points[i].normal_y, i_normals->points[i].normal_z);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    //@Create arrow
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = i_normals->header.frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = i_normals->points[i].x;
    marker.pose.position.y = i_normals->points[i].y;
    marker.pose.position.z = i_normals->points[i].z;
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3; //scale.x is the arrow length,
    marker.scale.y = 0.05; //scale.y is the arrow width 
    marker.scale.z = 0.1; //scale.z is the arrow height. 

    double angle = atan2(i_normals->points[i].normal_z, 
                  sqrt(i_normals->points[i].normal_x*i_normals->points[i].normal_x+ i_normals->points[i].normal_y*i_normals->points[i].normal_y) ) * 180 / 3.1415926535;

    if(fabs(angle)<=10){
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.0f;      
    }
    else{
      marker.color.r = 0.0f;
      marker.color.g = 0.8f;
      marker.color.b = 0.2f; 
    }

    marker.color.a = 0.6f;   
    markerArray.markers.push_back(marker); 
  }
  pub_pc_normal_.publish(markerArray);
}
*/




}  // namespace mcl_3dl
