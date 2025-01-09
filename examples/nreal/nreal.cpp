#include "vision_core/nreal_light.hpp"
#include <iostream>
#include <fstream> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>




int imu()
{
    NrealLight glasses;

    rclcpp::PublisherOptions mPubOpt;
    rclcpp::QoS mQos(10);
    std::cout << "Initialized NrealLight" << std::endl;

    mPubOpt.qos_overriding_options =
        rclcpp::QosOverridingOptions::with_default_policies();
    // Send initial heartbeat
    glasses.sendHeartbeatIfNeeded();
    std::cout << "Sent initial heartbeat" << std::endl;

    // Get the current display mode
    DisplayMode currentMode = glasses.getDisplayMode();
    std::cout << "Current display mode: " << static_cast<int>(currentMode) << std::endl;

    // Set a new display mode (example: Stereo mode)
    Error setResult = glasses.setDisplayMode(DisplayMode::Stereo);
    if (setResult == Error::None) {
        std::cout << "Display mode set to Stereo successfully." << std::endl;
    } else {
        std::cerr << "Failed to set display mode to Stereo." << std::endl;
    }

    // Verify the new display mode
    DisplayMode newMode = glasses.getDisplayMode();
    std::cout << "New display mode: " << static_cast<int>(newMode) << std::endl;

    setResult = glasses.setDisplayMode(DisplayMode::SameOnBoth);
    if (setResult == Error::None) {
        std::cout << "Display mode set to SameOnBoth successfully." << std::endl;
    } else {
        std::cerr << "Failed to set display mode to SameOnBoth." << std::endl;
    }

    setResult = glasses.setDisplayMode(DisplayMode::HighRefreshRate);
    if (setResult == Error::None) {
        std::cout << "Display mode set to HighRefreshRate successfully." << std::endl;
    } else {
        std::cerr << "Failed to set display mode to HighRefreshRate." << std::endl;
    }
    // rclcpp::init(0, nullptr);
    // IMU data publisher
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("imu_publisher");
    rclcpp::spin_some(node);
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::Imu>("/xreal/imu/data",  mQos, mPubOpt);
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu_msg.angular_velocity_covariance = {0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02};
    imu_msg.linear_acceleration_covariance = {0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04};
    float x, y, z, w;
    // Main loop for IMU data reading
    while (rclcpp::ok()) {
        auto start_time = std::chrono::high_resolution_clock::now();
        glasses.sendHeartbeatIfNeeded();

        auto imu_data = glasses.readIMUData();
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        if (imu_data) {
            auto [gyro, acc] = imu_data.value();
            roll = atan2(acc.y, acc.z);
            pitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z));
            yaw = atan2(gyro.y, gyro.x);
            auto end_time = std::chrono::high_resolution_clock::now();
            // std::cout << "FPS = " << 1000 / std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() << std::endl;
            imu_msg.header.stamp = node->now();
            // calculate quaternion directly from gyro and acc
            x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
            y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
            z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
            w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
            imu_msg.orientation.x = x;
            imu_msg.orientation.y = y;
            imu_msg.orientation.z = z;
            imu_msg.orientation.w = w;
            imu_msg.angular_velocity.x = gyro.x;
            imu_msg.angular_velocity.y = gyro.y;
            imu_msg.angular_velocity.z = gyro.z;
            imu_msg.linear_acceleration.x = acc.x;
            imu_msg.linear_acceleration.y = acc.y;
            imu_msg.linear_acceleration.z = acc.z;
            publisher->publish(imu_msg);
            // std::cout << "Gyroscope: (" << gyro.x << ", " << gyro.y << ", " << gyro.z << ")" << endl;
            // std::cout << "Accelerometer: (" << acc.x << ", " << acc.y << ", " << acc.z << ")" << endl;
            std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << endl;
        } else {
            std::cerr << "Failed to read IMU data." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}
int main(int argc, char **argv)
{   
    std::thread imu_thread(imu);
    imu_thread.join();
    return 0;
}
