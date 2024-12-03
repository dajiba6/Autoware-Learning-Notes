# ROS Log
[ros2 log documentation(cn)](https://fishros.org/doc/ros2/humble/Tutorials/Demos/Logging-and-logger-configuration.html)
## add log in code
### basic
```cpp
// printf style
RCLCPP_DEBUG(node->get_logger(), "My log message %d", 4);

// C++ stream style
RCLCPP_DEBUG_STREAM(node->get_logger(), "My log message " << 4);
```