/* Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include "nav2_map_server/map_server.hpp"

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("map_server", "", options), map_available_(false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("topic_name", "map");
  declare_parameter("frame_id", "map");
}

MapServer::~MapServer()
{
}

nav2_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Get the name of the YAML file to use (can be empty if no initial map should be used)
  yaml_filename = get_parameter("yaml_filename").as_string();
  // std::string yaml_string = get_parameter("yaml_filenames").as_string();
  std::string topic_name = get_parameter("topic_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();

  // Example declaration, adjust the types according to your needs
  std::vector<std::array<int, 4>> zones = {
      {-0.5, -14, 4, -16.5},
      {9, 20, 12, 16},
      {-0.5, 2, 3, -1},
      {27.5, 20, 32, 16},
      {27.5, -10, 32, -14.5}
  };

  std::string file_prefix = yaml_filename.substr(0, yaml_filename.find_last_of("/"));
  std::vector<std::string> zone_yamls = {
    file_prefix + "/keepout_zone1.yaml",
    file_prefix + "/keepout_zone2.yaml",
    file_prefix + "/keepout_zone3.yaml",
    file_prefix + "/keepout_zone4.yaml",
    file_prefix + "/keepout_zone5.yaml"
  };

  // Combine the datasets
  for (size_t i = 0; i < zones.size(); ++i) {
      combined_zones.push_back({zones[i], zone_yamls[i]});
  }

  // only try to load map if parameter was set
  if (!yaml_filename.empty()) {
    // Shared pointer to LoadMap::Response is also should be initialized
    // in order to avoid null-pointer dereference
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
      std::make_shared<nav2_msgs::srv::LoadMap::Response>();

    if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
      throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }
  } else {
    RCLCPP_INFO(
      get_logger(),
      "yaml-filename parameter is empty, set map through '%s'-service",
      load_map_service_name_.c_str());
  }
  RCLCPP_INFO(
    get_logger(),
    "yaml-filename is %s",
    yaml_filename.c_str());

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  // Create a service that provides the occupancy grid
  occ_service_ = create_service<nav_msgs::srv::GetMap>(
    service_prefix + std::string(service_name_),
    std::bind(&MapServer::getMapCallback, this, _1, _2, _3));

  // Create a service that provides the occupancy grid
  update_map_service_ = create_service<nav2_msgs::srv::UpdateMap>(
    service_prefix + std::string(update_map_service_name_),
    std::bind(&MapServer::updateMapCallback, this, _1, _2, _3));

  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/acml_pose", 10, std::bind(&MapServer::poseCallback, this, _1));

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a service that loads the occupancy grid from a file
  load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
    service_prefix + std::string(load_map_service_name_),
    std::bind(&MapServer::loadMapCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Publish the map using the latched topic
  occ_pub_->on_activate();
  if (map_available_) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  occ_pub_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  occ_pub_.reset();
  occ_service_.reset();
  load_map_service_.reset();
  map_available_ = false;
  msg_ = nav_msgs::msg::OccupancyGrid();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MapServer::getMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
  std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received GetMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->map = msg_;
}

void MapServer::loadMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received LoadMap request but not in ACTIVE state, ignoring!");
    response->result = response->RESULT_UNDEFINED_FAILURE;
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling LoadMap request");
  // Load from file
  if (loadMapResponseFromYaml(request->map_url, response)) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));  // publish new map
  }
}

void MapServer::updateMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::UpdateMap::Request> /*request*/,
  std::shared_ptr<nav2_msgs::srv::UpdateMap::Response> response)
{
  RCLCPP_INFO(
    logger_, "Called the update!!");
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received UpdateMap request but not in ACTIVE state, ignoring!");
    response->result = response->RESULT_UNDEFINED_FAILURE;
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling UpdateMap request");

  // Example: Update the map based on some internal logic or data
  // This is a placeholder for your map updating logic
  bool map_updated = updateMapInternally(response);

  if (map_updated) {
    // Assuming msg_ is updated by updateMapInternally() and ready to be published
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));  // publish updated map
    response->result = response->RESULT_SUCCESS; // Indicate success
  } else {
    // Handle failure to update map
    RCLCPP_ERROR(get_logger(), "Failed to update map.");
    response->result = response->RESULT_UNDEFINED_FAILURE;
  }
}

void MapServer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = msg;
}

bool MapServer::loadMapResponseFromYaml(
  const std::string & yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  switch (loadMapFromYaml(yaml_file, msg_)) {
    case MAP_DOES_NOT_EXIST:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case INVALID_MAP_DATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case LOAD_MAP_SUCCESS:
      // Correcting msg_ header when it belongs to specific node
      updateMsgHeader();

      map_available_ = true;
      response->map = msg_;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }

  return true;
}

bool MapServer::updateMapResponseFromYaml(
  const std::string & yaml_file,
  std::shared_ptr<nav2_msgs::srv::UpdateMap::Response> response)
{
  switch (loadMapFromYaml(yaml_file, msg_)) {
    case MAP_DOES_NOT_EXIST:
      response->result = nav2_msgs::srv::UpdateMap::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::UpdateMap::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case INVALID_MAP_DATA:
      response->result = nav2_msgs::srv::UpdateMap::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case LOAD_MAP_SUCCESS:
      // Correcting msg_ header when it belongs to specific node
      updateMsgHeader();

      map_available_ = true;
      response->map = msg_;
      response->result = nav2_msgs::srv::UpdateMap::Response::RESULT_SUCCESS;
  }

  return true;
}

void MapServer::updateMsgHeader()
{
  msg_.info.map_load_time = now();
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = now();
}

bool MapServer::updateMapInternally(std::shared_ptr<nav2_msgs::srv::UpdateMap::Response> response)
{
    std::string selected_yaml;
    if (latest_pose_ != nullptr) {
        double x = latest_pose_->pose.position.x;
        double y = latest_pose_->pose.position.y;
        // Iterate over combined_zones instead of filename_zones
        for (const auto& zone_pair : combined_zones) {
            const auto& zone = zone_pair.first; // This is the std::array<int, 4>
            if (x >= zone[0] && x <= zone[2] && y >= zone[1] && y <= zone[3]) {
                selected_yaml = zone_pair.second; // This is the std::string (YAML file path)
                break;
            }
        }
    }
    if (selected_yaml.empty()) {
        // Handle the case where no YAML is selected
        response->result = nav2_msgs::srv::UpdateMap::Response::RESULT_MAP_DOES_NOT_EXIST;
        return false; // Assuming you want to return false if no YAML is selected
    }

    // Assuming updateMapResponseFromYaml is a function that returns a bool
    // indicating success or failure
    if (!updateMapResponseFromYaml(selected_yaml, response)) {
        response->result = nav2_msgs::srv::UpdateMap::Response::RESULT_INVALID_MAP_DATA;
        return false;
    }

    response->result = nav2_msgs::srv::UpdateMap::Response::RESULT_SUCCESS;
    return true;
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::MapServer)
