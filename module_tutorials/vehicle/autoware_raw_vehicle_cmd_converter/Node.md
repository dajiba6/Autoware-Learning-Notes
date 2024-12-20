# raw_vehicle_cmd_converter
## Intro
This package converts inputs like acceleration and desired steering into vehicle control commands, including throttle, brake, and steering angle.

node:
- init
  - InterProcessPollingSubscriber sub_odometry_ in hpp: obtain odometry info
  - sub_control_cmd_ in constructor: obtain control command
    - bind with `onControlCmd`. When recieve messages it will use lookup to convert command and then publish actuation command
  - pub_actuation_cmd_ in constructor: publish actuation command

- [x] what is the frequency of sub_control_cmd_?  
ans: frequncy of sub is based on the frequncy of pub.

this node is registered as a components, which means it can be loaded without instantiation:
```cpp
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::raw_vehicle_cmd_converter::RawVehicleCommandConverterNode)
```
- [ ] lifecircle and workflow of this node?
## Constructor
Init parameters, subscribers and publishers.
- [x] difference between   
const double a = declare_parameter<double>("vgr_coef_a");  
const double kp_steer{declare_parameter<double>("steer_pid.kp")};  
**ans**:  
`=` allows narrowing conversion while `{}` dose not. so `{}` is safer.

- [x] `sub_control_cmd_ = create_subscription<Control>(
    "~/input/control_cmd", 1, std::bind(&RawVehicleCommandConverterNode::onControlCmd, this, _1));`  
    What is `~/input/control_cmd`? What is the difference of topic name beteen autoware and apollo?  
**ans**:  
Topics with the prefix ~ represent private topics, indicating that they are specific to a particular node.  
The last token is the topic or service base name, and any preceding tokens make up the namespace of the topic or service.  
In the launch folder, the .launch.xml file remaps the ~/input/control_cmd topic to $(var input_odometry), which is defined as /localization/kinematic_state. This means that the message read from ~/input/control_cmd is actually sourced from the /localization/kinematic_state topic.  
Apollo does not have the mechanisms of using ~ to represent a relative namespace and topic remapping.  
**exp1**: Relative names are appended to the namespace of the node which creates them. For example, if the node is put into a namespace /ping/pong and the topic or service name is foo/bar, then the absolute name will become /ping/pong/foo/bar. However, if the topic or service name is /foo/bar, then the absolute name will stay just /foo/bar, ignoring the node’s namespace.
  | **Input Name** | **Node: my_node NS: none** | **Node: my_node NS: /my_ns** |
  | -------------- | -------------------------- | ---------------------------- |
  | `ping`         | `/ping`                    | `/my_ns/ping`                |
  | `/ping`        | `/ping`                    | `/ping`                      |
  | `~`            | `/my_node`                 | `/my_ns/my_node`             |
  | `~/ping`       | `/my_node/ping`            | `/my_ns/my_node/ping`        |

  [topic name doc1](https://wiki.ros.org/Names)  
  [topic name doc2](https://design.ros2.org/articles/topic_and_service_names.html)


```cpp
RawVehicleCommandConverterNode::RawVehicleCommandConverterNode(
  const rclcpp::NodeOptions & node_options)
: Node("autoware_raw_vehicle_cmd_converter_node", node_options)
{
  using std::placeholders::_1;
  /* parameters for accel/brake map */
  ......
  // for steering steer controller
  ......
  // get accel_map_ and brake_map_
  ......
  // for convert_steer_cmd
  ......
  // NOTE: The steering status can be published from the vehicle side or converted in this node.
  ......
    // NOTE: Polling subscriber requires specifying the topic name at declaration,
    // so use a normal callback subscriber.
  ......
  // NOTE: some vehicles do not publish actuation status. To handle this, subscribe only when the
  // option is specified.
  ......
  // set subscriber and publisher
  sub_control_cmd_ = create_subscription<Control>(
    "~/input/control_cmd", 1, std::bind(&RawVehicleCommandConverterNode::onControlCmd, this, _1));
  pub_actuation_cmd_ = create_publisher<ActuationCommandStamped>("~/output/actuation_cmd", 1);
  debug_pub_steer_pid_ = create_publisher<Float32MultiArrayStamped>(
    "/vehicle/raw_vehicle_cmd_converter/debug/steer_pid", 1);

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
}
```
## onControlCmd
Obtain twist and cmd, then call publishActuationCmd().  
`Control::ConstSharedPtr` msg is defined in `src/core/autoware_msgs/autoware_control_msgs/msg/Control.msg`.
```cpp
void RawVehicleCommandConverterNode::onControlCmd(const Control::ConstSharedPtr msg)
{
  //? what is polling subscriber? what is the difference between polling subscriber and normal callback subscriber?
  const auto odometry_msg = sub_odometry_.takeData();
  if (odometry_msg) {
    current_twist_ptr_ = std::make_unique<TwistStamped>();
    current_twist_ptr_->header = odometry_msg->header;
    //?what does twist contain?
    current_twist_ptr_->twist = odometry_msg->twist.twist;
  }
  control_cmd_ptr_ = msg;
  publishActuationCmd();
}
```
## publishActuationCmd
Calculate desired throttle, brake, steer cmd and publish them.
```cpp
void RawVehicleCommandConverterNode::publishActuationCmd()
{
  // check if data is exist.
  ......
  // ?why we need this?
  if (need_to_subscribe_actuation_status_) {
    if (!actuation_status_ptr_) {
      RCLCPP_WARN_EXPRESSION(get_logger(), is_debugging_, "actuation status is null");
      return;
    }
  }

  double desired_accel_cmd = 0.0;
  double desired_brake_cmd = 0.0;
  double desired_steer_cmd = 0.0;
  ActuationCommandStamped actuation_cmd;
  const double acc = control_cmd_ptr_->longitudinal.acceleration;
  const double vel = current_twist_ptr_->twist.linear.x;
  const double steer = control_cmd_ptr_->lateral.steering_tire_angle;
  const double steer_rate = control_cmd_ptr_->lateral.steering_tire_rotation_rate;
  bool accel_cmd_is_zero = true;
  // 1. Calculate throttle cmd.
  if (convert_accel_cmd_) {
    desired_accel_cmd = calculateAccelMap(vel, acc, accel_cmd_is_zero);
  } else {
    // if conversion is disabled use acceleration as actuation cmd
    desired_accel_cmd = (acc >= 0) ? acc : 0;
  }
  // 2. Calculate brake cmd.
  if (convert_brake_cmd_) {
    if (accel_cmd_is_zero) {
      desired_brake_cmd = calculateBrakeMap(vel, acc);
    }
  } else if (acc < 0) {
    // if conversion is disabled use negative acceleration as brake cmd
    desired_brake_cmd = -acc;
  }
  // 3. Calculate steer cmd.
  if (!convert_steer_cmd_method_.has_value()) {
    // if conversion is disabled use steering angle as steer cmd
    desired_steer_cmd = steer;
  } else if (convert_steer_cmd_method_.value() == "vgr") {
    // NOTE: When using variable gear ratio,
    // the actuation cmd is the steering wheel angle,
    // and the actuation_status is also the steering wheel angle.
    const double current_steer_wheel = actuation_status_ptr_->status.steer_status;
    const double adaptive_gear_ratio = vgr_.calculateVariableGearRatio(vel, current_steer_wheel);
    desired_steer_cmd = steer * adaptive_gear_ratio;
  } else if (convert_steer_cmd_method_.value() == "steer_map") {
    desired_steer_cmd = calculateSteerFromMap(vel, steer, steer_rate);
  }
  // 4. publish final actuation cmd
  actuation_cmd.header.frame_id = "base_link";
  actuation_cmd.header.stamp = control_cmd_ptr_->stamp;
  actuation_cmd.actuation.accel_cmd = desired_accel_cmd;
  actuation_cmd.actuation.brake_cmd = desired_brake_cmd;
  actuation_cmd.actuation.steer_cmd = desired_steer_cmd;
  pub_actuation_cmd_->publish(actuation_cmd);
}
```
## calculateAccelMap
Obtain throttle cmd by using lookup table.
```cpp
double RawVehicleCommandConverterNode::calculateAccelMap(
  const double current_velocity, const double desired_acc, bool & accel_cmd_is_zero)
{
  double desired_accel_cmd = 0;
  if (!accel_map_.getThrottle(desired_acc, std::abs(current_velocity), desired_accel_cmd)) {
    desired_accel_cmd = 0;
  } else {
    accel_cmd_is_zero = false;
  }
  desired_accel_cmd = std::min(std::max(desired_accel_cmd, 0.0), max_accel_cmd_);
  return desired_accel_cmd;
}
```