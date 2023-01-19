# provizio_radar_api_ros

ROS 1 Driver for Provizio radars.

## Provided Functionality

- Supports a single radar on the network
- Publishes live non-accumulated point clouds

## Topics

### /provizio_radar_point_cloud

Live non-accumulated point clouds as [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html).

Point fields present:

- `x`: Forward, radar relative, meters, float (32 bit)
- `y`: Left, radar relative, meters, float (32 bit)
- `z`: Up, radar relative, meters, float (32 bit)
- `radar_relative_radial_velocity`: Forward, radar relative, meters/second, float (32 bit)
- `signal_to_noise_ratio`: float (32 bit)
- `ground_relative_radial_velocity`: Forward, ground relative, meters/second, float (32 bit), may be a valid value or NaN depending on a radar configuration

## Native API

The complete native API written in C can be [found here](https://github.com/provizio/provizio_radar_api_core).

## License

Open-source as Apache License 2.0 (see [LICENSE](LICENSE))
