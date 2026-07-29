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

#include <memory>
#include <dddmr_pg_map_server.h>
#include "rclcpp/rclcpp.hpp"

class DDDMRPG_MAP_SERVER_MANAGER : public rclcpp::Node
{
  public:

    DDDMRPG_MAP_SERVER_MANAGER();

  private:
    
    void spinPGMap(std::shared_ptr<dddmr_pg_map_server::DDDMRPGMapServer> p_ptr);
};

void DDDMRPG_MAP_SERVER_MANAGER::spinPGMap(std::shared_ptr<dddmr_pg_map_server::DDDMRPGMapServer> p_ptr){
  
  auto executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(p_ptr);
  executor_->spin();
}

DDDMRPG_MAP_SERVER_MANAGER::DDDMRPG_MAP_SERVER_MANAGER():Node("dddmr_pg_map_server_manager"){
  
  std::vector<std::string> maps = {"map1"};
  this->declare_parameter("maps", maps);
  this->get_parameter("maps", maps);

  for(auto i=maps.begin(); i!=maps.end(); i++){
    RCLCPP_INFO(this->get_logger(), "Initializing map: %s", (*i).c_str());  
    auto a_pg_map_server = std::make_shared<dddmr_pg_map_server::DDDMRPGMapServer>((*i));
    std::thread{std::bind(&DDDMRPG_MAP_SERVER_MANAGER::spinPGMap, this, std::placeholders::_1), a_pg_map_server}.detach();
  }

}


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DDDMRPG_MAP_SERVER_MANAGER>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
