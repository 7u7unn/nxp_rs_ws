// Offline FCL regression: no ROS nodes, controller, or physical hardware.
#include <fstream>
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

namespace
{
std::string read(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) { throw std::runtime_error("Cannot open " + path); }
  std::ostringstream out;
  out << stream.rdbuf();
  return out.str();
}

std::unique_ptr<planning_scene::PlanningScene> makeScene(bool legacy_base = false)
{
  const std::string root = PACKAGE_SOURCE_DIR;
  auto urdf = urdf::parseURDF(read(root + "/../nxp_rs_description/urdf/nxp_rs.xacro"));
  if (!urdf) { throw std::runtime_error("Invalid robot URDF"); }
  if (legacy_base) {
    auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(urdf->getLink("base_link")->collision->geometry);
    if (!mesh) { throw std::runtime_error("Expected base collision mesh"); }
    mesh->filename = "package://nxp_rs_description/meshes/base-link-col.STL";
  }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, read(root + "/config/nxp_rs.srdf"))) {
    throw std::runtime_error("Invalid robot SRDF");
  }
  return std::make_unique<planning_scene::PlanningScene>(
    std::make_shared<moveit::core::RobotModel>(urdf, srdf));
}

collision_detection::CollisionResult check(
  planning_scene::PlanningScene & scene, const std::vector<double> & arm, double finger)
{
  auto & state = scene.getCurrentStateNonConst();
  state.setJointGroupPositions("arm", arm);
  state.setVariablePosition("joint_right-finger", finger);
  state.update();
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.group_name = "arm";
  request.contacts = true;
  request.max_contacts = 100;
  scene.checkSelfCollision(request, result, state);
  return result;
}

const std::vector<double> measured = {
  .007454178180676951, .01303828479241087, .009043818172354001,
  1.2672792155382577, 1.8224123595264652, .24310837434099652};
const std::vector<double> desired = {
  .008046649543655263, -.05327261704304654, .007399163602867972,
  1.187140353498006, 1.826254871665399, .24421534772542555};
const std::pair<std::string, std::string> base_finger{"base_link", "right-finger"};
}  // namespace

TEST(CollisionGeometry, RecordedPoseReproducesLegacyMeshFalsePositive)
{
  auto old_scene = makeScene(true);
  auto scene = makeScene();
  // Test the recorded encoder value and the calibrated open-limit value:
  // the small gripper bounds violation is NOT the cause of this collision.
  for (double finger : {-.00027820288460854393, 0.0}) {
    EXPECT_NE(check(*old_scene, measured, finger).contacts.count(base_finger), 0u);
    EXPECT_FALSE(check(*scene, measured, finger).collision);
    EXPECT_FALSE(check(*scene, desired, finger).collision);
  }
}

TEST(CollisionGeometry, GenuineBaseFingerIntersectionStillRejected)
{
  auto scene = makeScene();
  const auto result = check(*scene,
    {-.288789, -.877655, .401449, -1.50626, -1.78133, .46042}, 0.0);
  EXPECT_TRUE(result.collision);
  EXPECT_NE(result.contacts.count(base_finger), 0u);
}

TEST(CollisionGeometry, RecordedFingerBoundsViolationRemainsVisible)
{
  auto scene = makeScene();
  check(*scene, measured, -.00027820288460854393);
  EXPECT_FALSE(scene->getCurrentState().satisfiesBounds());
  check(*scene, measured, 0.0);
  EXPECT_TRUE(scene->getCurrentState().satisfiesBounds());
}

TEST(CollisionGeometry, GraspFrameIsTheArmTipBetweenFingerRoots)
{
  auto scene = makeScene();
  const auto * arm = scene->getRobotModel()->getJointModelGroup("arm");
  ASSERT_NE(arm, nullptr);
  const auto & links = arm->getLinkModelNames();
  EXPECT_NE(std::find(links.begin(), links.end(), "grasp_frame"), links.end());
  EXPECT_EQ(links.back(), "grasp_frame");

  auto & state = scene->getCurrentStateNonConst();
  state.setJointGroupPositions("arm", std::vector<double>(6, 0.0));
  state.setVariablePosition("joint_right-finger", 0.0);
  state.update();
  const auto grasp = state.getGlobalLinkTransform("grasp_frame").translation();
  const auto right = state.getGlobalLinkTransform("right-finger").translation();
  const auto left = state.getGlobalLinkTransform("left-finger").translation();
  EXPECT_NEAR(grasp.y(), (right.y() + left.y()) / 2.0, 1e-9);
}
