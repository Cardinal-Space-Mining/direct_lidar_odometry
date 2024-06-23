/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "rclcpp/rclcpp.hpp"

namespace dlo {

    template <typename T>
    struct identity { typedef T type; };

    template <typename T>
    void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value) {
        node->declare_parameter(param_name, default_value);
        node->get_parameter(param_name, param);
    }

}


#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tf2
{
    template<>
    inline void doTransform(
        const sensor_msgs::msg::Imu& t_in,
        sensor_msgs::msg::Imu& t_out,
        const geometry_msgs::msg::TransformStamped& transform)
    {
        t_out.header.stamp = transform.header.stamp;
        t_out.header.frame_id = transform.header.frame_id;

        doTransform(t_in.orientation, t_out.orientation, transform);
        t_out.orientation_covariance = t_in.orientation_covariance;

        doTransform(t_in.angular_velocity, t_out.angular_velocity, transform);
        t_out.angular_velocity_covariance = t_in.angular_velocity_covariance;

        doTransform(t_in.linear_acceleration, t_out.linear_acceleration, transform);
        t_out.linear_acceleration_covariance = t_in.linear_acceleration_covariance;
    }
}
