#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/allowed_collision_entry.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace
{

using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

std::size_t findEntry(
  const moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const std::string & name)
{
  const auto iterator = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name);
  if (iterator == matrix.entry_names.end()) {
    return matrix.entry_names.size();
  }
  return static_cast<std::size_t>(std::distance(matrix.entry_names.begin(), iterator));
}

void addEntry(
  moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const std::string & name)
{
  if (findEntry(matrix, name) != matrix.entry_names.size()) {
    return;
  }

  const std::size_t old_size = matrix.entry_names.size();
  if (matrix.entry_values.size() > old_size) {
    matrix.entry_values.resize(old_size);
  }
  matrix.entry_values.resize(old_size);
  for (auto & row : matrix.entry_values) {
    row.enabled.resize(old_size + 1, false);
  }

  matrix.entry_names.push_back(name);
  moveit_msgs::msg::AllowedCollisionEntry new_row;
  new_row.enabled.assign(old_size + 1, false);
  matrix.entry_values.push_back(std::move(new_row));
}

void allowPair(
  moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const std::string & first,
  const std::string & second)
{
  addEntry(matrix, first);
  addEntry(matrix, second);

  const std::size_t first_index = findEntry(matrix, first);
  const std::size_t second_index = findEntry(matrix, second);
  const std::size_t size = matrix.entry_names.size();
  for (auto & row : matrix.entry_values) {
    row.enabled.resize(size, false);
  }
  matrix.entry_values[first_index].enabled[second_index] = true;
  matrix.entry_values[second_index].enabled[first_index] = true;
}

moveit_msgs::msg::CollisionObject makeFloor(
  const std::string & id,
  const std::string & frame,
  double size_x,
  double size_y,
  double thickness,
  double top_z)
{
  moveit_msgs::msg::CollisionObject floor;
  floor.id = id;
  floor.header.frame_id = frame;
  floor.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size_x;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size_y;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = thickness;

  geometry_msgs::msg::Pose pose;
  pose.position.z = top_z - thickness / 2.0;
  pose.orientation.w = 1.0;

  floor.primitives.push_back(primitive);
  floor.primitive_poses.push_back(pose);
  return floor;
}

}  // namespace

class FloorSceneNode final : public rclcpp::Node
{
public:
  FloorSceneNode()
  : Node("floor_scene")
  {
    floor_id_ = declare_parameter<std::string>("floor_id", "nxp_rs_floor");
    floor_frame_ = declare_parameter<std::string>("floor_frame", "world");
    base_link_ = declare_parameter<std::string>("base_link", "base_link");
    floor_size_x_ = declare_parameter<double>("floor_size_x", 10.0);
    floor_size_y_ = declare_parameter<double>("floor_size_y", 10.0);
    floor_thickness_ = declare_parameter<double>("floor_thickness", 0.02);
    floor_top_z_ = declare_parameter<double>("floor_top_z", 0.0);

    if (floor_size_x_ <= 0.0 || floor_size_y_ <= 0.0 || floor_thickness_ <= 0.0) {
      throw std::invalid_argument("floor dimensions must be positive");
    }

    get_scene_client_ = create_client<GetPlanningScene>("/get_planning_scene");
    apply_scene_client_ = create_client<ApplyPlanningScene>("/apply_planning_scene");
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&FloorSceneNode::requestCurrentScene, this));
  }

private:
  void requestCurrentScene()
  {
    if (done_ || request_pending_ || apply_pending_) {
      return;
    }

    if (!get_scene_client_->service_is_ready() || !apply_scene_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for MoveIt planning-scene services before adding the floor");
      return;
    }

    auto request = std::make_shared<GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    request_pending_ = true;
    get_scene_client_->async_send_request(
      request,
      [this](rclcpp::Client<GetPlanningScene>::SharedFuture future) {
        request_pending_ = false;
        const auto response = future.get();
        if (!response || response->scene.allowed_collision_matrix.entry_names.empty()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "MoveIt returned no Allowed Collision Matrix yet; retrying");
          return;
        }

        moveit_msgs::msg::PlanningScene scene;
        scene.is_diff = true;
        scene.world.collision_objects.push_back(makeFloor(
          floor_id_, floor_frame_, floor_size_x_, floor_size_y_, floor_thickness_, floor_top_z_));

        // Preserve all SRDF collision exceptions and add only the expected
        // fixed base-to-floor contact exception.
        scene.allowed_collision_matrix = response->scene.allowed_collision_matrix;
        allowPair(scene.allowed_collision_matrix, base_link_, floor_id_);

        auto apply_request = std::make_shared<ApplyPlanningScene::Request>();
        apply_request->scene = std::move(scene);
        apply_pending_ = true;
        apply_scene_client_->async_send_request(
          apply_request,
          [this](rclcpp::Client<ApplyPlanningScene>::SharedFuture apply_future) {
            apply_pending_ = false;
            const auto response = apply_future.get();
            if (!response || !response->success) {
              RCLCPP_ERROR(get_logger(), "MoveIt rejected the floor planning-scene update");
              return;
            }

            done_ = true;
            retry_timer_->cancel();
            RCLCPP_INFO(
              get_logger(),
              "Added floor '%s' (top z=%.3f in %s); allowed only %s-floor contact",
              floor_id_.c_str(), floor_top_z_, floor_frame_.c_str(), base_link_.c_str());
          });
      });
  }

  std::string floor_id_;
  std::string floor_frame_;
  std::string base_link_;
  double floor_size_x_{10.0};
  double floor_size_y_{10.0};
  double floor_thickness_{0.02};
  double floor_top_z_{0.0};
  bool request_pending_{false};
  bool apply_pending_{false};
  bool done_{false};
  rclcpp::Client<GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr apply_scene_client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FloorSceneNode>());
  } catch (const std::exception & exception) {
    fprintf(stderr, "[floor_scene] %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
