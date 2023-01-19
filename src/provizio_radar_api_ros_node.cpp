// Copyright 2022 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <inttypes.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "provizio/radar_api/core.h"

namespace
{
    constexpr const char *topic_name = "/provizio_radar_point_cloud";
    constexpr std::uint64_t receive_timeout_ns = 100000000; // 0.1s

    std::size_t point_clouds_sent = 0;
    std::size_t points_sent = 0;

    // Checks if the host is big endian
    constexpr bool is_host_big_endian()
    {
        union
        {
            uint32_t as_int;
            char as_chars[4];
        } testint = {0x01020304};

        return testint.as_chars[0] == 1;
    }

    // Gets invoked every time a complete or incomplete (in case of lost packets) point cloud is received
    void point_cloud_callback(const provizio_radar_point_cloud *point_cloud,
                              struct provizio_radar_point_cloud_api_context *context)
    {
        // As defined in http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html
        constexpr std::uint8_t float_field_type = 7;
        constexpr std::size_t num_fields_per_point = 6;
        constexpr std::size_t field_x = 0;
        constexpr std::size_t field_y = 1;
        constexpr std::size_t field_z = 2;
        constexpr std::size_t field_radar_relative_radial_velocity = 3;
        constexpr std::size_t field_signal_to_noise_ratio = 4;
        constexpr std::size_t field_ground_relative_radial_velocity = 5; // May be valid or NaN, depending on radar configuration

        // Convert provizio_radar_point_cloud to sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 ros_point_cloud;
        ros_point_cloud.header.seq = static_cast<std::uint32_t>(point_clouds_sent++);
        ros_point_cloud.header.stamp.fromNSec(point_cloud->timestamp);
        ros_point_cloud.header.frame_id = "provizio_radar_" + std::to_string(point_cloud->radar_position_id); // Defines a reference frame, not a frame number
        ros_point_cloud.height = 1;
        ros_point_cloud.width = point_cloud->num_points_received;
        ros_point_cloud.fields.resize(num_fields_per_point);
        ros_point_cloud.fields[field_x].name = "x";
        ros_point_cloud.fields[field_x].offset = offsetof(provizio_radar_point, x_meters);
        ros_point_cloud.fields[field_x].datatype = float_field_type;
        ros_point_cloud.fields[field_x].count = 1;
        ros_point_cloud.fields[field_y].name = "y";
        ros_point_cloud.fields[field_y].offset = offsetof(provizio_radar_point, y_meters);
        ros_point_cloud.fields[field_y].datatype = float_field_type;
        ros_point_cloud.fields[field_y].count = 1;
        ros_point_cloud.fields[field_z].name = "z";
        ros_point_cloud.fields[field_z].offset = offsetof(provizio_radar_point, z_meters);
        ros_point_cloud.fields[field_z].datatype = float_field_type;
        ros_point_cloud.fields[field_z].count = 1;
        ros_point_cloud.fields[field_radar_relative_radial_velocity].name = "radar_relative_radial_velocity";
        ros_point_cloud.fields[field_radar_relative_radial_velocity].offset = offsetof(provizio_radar_point, radar_relative_radial_velocity_m_s);
        ros_point_cloud.fields[field_radar_relative_radial_velocity].datatype = float_field_type;
        ros_point_cloud.fields[field_radar_relative_radial_velocity].count = 1;
        ros_point_cloud.fields[field_signal_to_noise_ratio].name = "signal_to_noise_ratio";
        ros_point_cloud.fields[field_signal_to_noise_ratio].offset = offsetof(provizio_radar_point, signal_to_noise_ratio);
        ros_point_cloud.fields[field_signal_to_noise_ratio].datatype = float_field_type;
        ros_point_cloud.fields[field_signal_to_noise_ratio].count = 1;
        ros_point_cloud.fields[field_ground_relative_radial_velocity].name = "ground_relative_radial_velocity";
        ros_point_cloud.fields[field_ground_relative_radial_velocity].offset = offsetof(provizio_radar_point, ground_relative_radial_velocity_m_s);
        ros_point_cloud.fields[field_ground_relative_radial_velocity].datatype = float_field_type;
        ros_point_cloud.fields[field_ground_relative_radial_velocity].count = 1;
        ros_point_cloud.is_bigendian = is_host_big_endian();
        ros_point_cloud.point_step = sizeof(provizio_radar_point);
        ros_point_cloud.row_step = sizeof(provizio_radar_point) * point_cloud->num_points_received;
        ros_point_cloud.data.resize(ros_point_cloud.row_step);
        std::memcpy(ros_point_cloud.data.data(), point_cloud->radar_points, ros_point_cloud.data.size());
        ros_point_cloud.is_dense = true;

        // Good to publish now
        static_cast<ros::Publisher *>(context->user_data)->publish(ros_point_cloud);
        points_sent += point_cloud->num_points_received;
    }
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "provizio_radar_api_ros_node");

    ros::NodeHandle node;

    // TODO: Support multiple radars use case, each publishing to their respective topics
    ros::Publisher publisher = node.advertise<sensor_msgs::PointCloud2>(topic_name, 10);

    // Initialize the Provizio Radar API Context
    auto radar_api_context = std::make_unique<provizio_radar_point_cloud_api_context>();
    provizio_radar_point_cloud_api_context_init(&point_cloud_callback, &publisher, radar_api_context.get());

    // Open a live Provizio radar connection
    provizio_radar_api_connection radar_api_connection;
    std::memset(&radar_api_connection, 0, sizeof(&radar_api_connection));
    auto status = provizio_open_radar_connection(0, receive_timeout_ns, 0, radar_api_context.get(), &radar_api_connection);
    if (status != 0)
    {
        ROS_ERROR("provizio_open_radar_connection failed. Error code: %d", status);
        return status;
    }

    ROS_INFO("provizio_radar_api_ros_node is running");

    try
    {
        while (ros::ok())
        {
            // Receive the next radar packet if any (with timeout specified in receive_timeout_ns)
            status = provizio_radar_api_receive_packet(&radar_api_connection);
            if (status != 0 && status != PROVIZIO_E_TIMEOUT && status != PROVIZIO_E_SKIPPED)
            {
                if (status != EINTR)
                {
                    // Genuine error occured
                    ROS_ERROR("provizio_radar_api_receive_packet failed. Error code: %d", status);
                }
                else
                {
                    // Interrupted by user, not really an error
                }

                break;
            }

            ros::spinOnce();
        }
    }
    catch (...)
    {
        // Unexpected exception. Make sure to close the connection just in case.
        provizio_close_radar_connection(&radar_api_connection);
        throw;
    }

    if (status == PROVIZIO_E_TIMEOUT || status == PROVIZIO_E_SKIPPED || status == EINTR)
    {
        // Not really an error, they're all normal working statuses
        status = 0;
    }

    ROS_INFO("provizio_radar_api_ros_node finished with status: %d. Point clouds sent: %" PRId64 ". Total points sent: %" PRId64 "\n", static_cast<int>(status), point_clouds_sent, points_sent);

    provizio_close_radar_connection(&radar_api_connection);

    return status;
}
