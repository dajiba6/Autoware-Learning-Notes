# raw_vehicle_cmd_converter
## Intro
This package converts inputs like acceleration and desired steering into vehicle control commands, including throttle, brake, and steering angle.
## Constructor
Init parameters, subscribers and publishers.
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
```cpp
void RawVehicleCommandConverterNode::onControlCmd(const Control::ConstSharedPtr msg)
{
  const auto odometry_msg = sub_odometry_.takeData();
  if (odometry_msg) {
    current_twist_ptr_ = std::make_unique<TwistStamped>();
    current_twist_ptr_->header = odometry_msg->header;
    current_twist_ptr_->twist = odometry_msg->twist.twist;
  }
  control_cmd_ptr_ = msg;
  publishActuationCmd();
}
```