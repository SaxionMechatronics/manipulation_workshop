#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

  // initialize the scene by adding a plate add the pick up location
  moveit_msgs::msg::CollisionObject create_plate(float thickness) {
  
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "plate_base";
    collision_object.id = "plate";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.2;
    primitive.dimensions[primitive.BOX_Z] = thickness;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = 0.255+thickness/2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pick_place");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Add the plate object to the scene
  float plate_thickness = 0.02;
  auto plate = create_plate(plate_thickness);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(plate);

  tf2::Quaternion q;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Set the orientation of the tool to face downwards for picking.
  q.setRPY(0,0,1.57);
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1;
  target_pose.orientation.w = 0;

  // Define a target Pose to pick up plate
  // target_pose.position.x = TODO;
  // target_pose.position.y = TODO;
  // target_pose.position.z = TODO;

  // move_group_interface.setPoseTarget(target_pose);

  // // Create a plan to that target pose
  // auto success = move_group_interface.plan(plan);

  // // Execute the plan
  // if(success) {
  //   move_group_interface.execute(plan);
  // } else {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }

  //attach plate to robot
  //TODO
  
  // Set a target Pose to place plate
  //TODO

  // release object
  // TODO

  // Set a target Pose to place plate
  // TODO

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}