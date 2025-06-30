# Setup micro-ROS

1. Build this package with `colcon`:

   ```bash
   colcon build --packages-select spot_sensor_handler
   ```

2. Start the micro-ROS agent:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

3. Start the ESP sensor.

4. Start the plotter:

   ```bash
   ros2 run spot_sensor_handler magnet_plotter
   ```
