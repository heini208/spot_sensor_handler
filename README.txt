1. Setup microros workspace:

2. colcon build this package

colcon build --packages-select spot_sensor_handler

3. start micro_ros_agent  
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

4. start esp_sensor

5. start the plotter: 
```
ros2 run spot_sensor_handler magnet_plotter
```
