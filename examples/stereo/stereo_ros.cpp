#include <iostream>
#include <fstream> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include<chrono>

#include"vision_core/stereo.hpp"


cv::Mat heatmap(cv::Mat&disparity)
{
    //max min
    cv::Mat image_re = disparity.reshape(1);
    double minValue, maxValue;   
    cv::Point  minIdx, maxIdx;     
    cv::minMaxLoc(image_re, &minValue, &maxValue, &minIdx, &maxIdx);
    
    cv::Mat mean_mat(cv::Size(disparity.cols,disparity.rows), CV_32FC1, minValue);
    cv::Mat std_mat(cv::Size(disparity.cols,disparity.rows), CV_32FC1, (maxValue-minValue)/255);

    cv::Mat norm_disparity_map = (disparity - mean_mat) / std_mat;
    cv::Mat heap_map,abs_map;
    cv::convertScaleAbs(norm_disparity_map,abs_map,1);
    cv::applyColorMap(abs_map,heap_map,cv::COLORMAP_JET);
    return heap_map;

}

rclcpp::QoS mQos(10);
rclcpp::PublisherOptions mPubOpt;

void stereo(int argc, char **argv)
{
    rclcpp::init(0, nullptr);

    // set calibration path based on arg camera type
    std::string camera_type = (argc > 1) ? argv[1] : "default";
    std::cout << "Camera type: " << camera_type << std::endl;
    char* stereo_calibration_path;
    if (camera_type == "zed") {
        std::cout << "Using ZED calibration" << std::endl;
        stereo_calibration_path = "zed_calib_params.yml";
    } else if (camera_type == "nreal") {
        std::cout << "Using Nreal calibration" << std::endl;
        stereo_calibration_path = "nreal_calib_params.yml";
    } else {
        std::cout << "Using default calibration" << std::endl;
        stereo_calibration_path = "stereo_calibration.yml";
    }

    // pointcloud publisher
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("nreal_node");
    mQos.transient_local();

    rclcpp::spin_some(node);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/xreal/pointcloud",  mQos);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image = node->create_publisher<sensor_msgs::msg::Image>("/xreal/depth",  mQos);
    // left right image publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left = node->create_publisher<sensor_msgs::msg::Image>("/xreal/left/raw",   mQos);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right = node->create_publisher<sensor_msgs::msg::Image>("/xreal/right/raw",   mQos);
    // camera info left right publisher
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_left_info = node->create_publisher<sensor_msgs::msg::CameraInfo>("/xreal/left/camera_info",   mQos);
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_right_info = node->create_publisher<sensor_msgs::msg::CameraInfo>("/xreal/right/camera_info",   mQos);
    
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    sensor_msgs::msg::Image image_msg, left_image_msg, right_image_msg;
    cv_bridge::CvImage out_msg;
    pointcloud_msg.header.frame_id = "map";
    pointcloud_msg.height = 1;
    pointcloud_msg.width = 640 * 480;
    pointcloud_msg.fields.resize(6);
    std::vector<std::string> field_names = {"x", "y", "z", "r", "g", "b"};
    
    int offset = 0;
    for (const auto& name : field_names) {
        pointcloud_msg.fields[offset].name = name;
        pointcloud_msg.fields[offset].offset = offset * 4;
        pointcloud_msg.fields[offset].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[offset].count = 1;
        offset++;
    }
    pointcloud_msg.is_bigendian = false;
    pointcloud_msg.point_step = 24;
    pointcloud_msg.row_step = 640 * 480 * 24;
    pointcloud_msg.is_dense = true;
    std::vector<float> data_buffer(pointcloud_msg.width * pointcloud_msg.point_step / sizeof(float), 0.0f);


    image_msg.height = 480;
    image_msg.width = 640;
    image_msg.encoding = "rgb8";//"mono8";
    image_msg.is_bigendian = false;
    image_msg.step = 640;
    image_msg.data.resize(640 * 480 * 3);
    left_image_msg = image_msg;
    right_image_msg = image_msg;
    out_msg.header.frame_id = "map";
    left_image_msg.header.frame_id = "left_camera";
    right_image_msg.header.frame_id = "right_camera";

    // left_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    // right_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    left_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    right_image_msg.encoding = sensor_msgs::image_encodings::RGB8;

    // extract camera informatio from stereo opencv calibration file
    cv::FileStorage fs(stereo_calibration_path, cv::FileStorage::READ);
    cv::Mat camera_matrix1, camera_matrix2, dist_coeffs1, dist_coeffs2, R, T;
    fs["intrinsic_left"] >> camera_matrix1;
    fs["intrinsic_right"] >> camera_matrix2;
    fs["distCoeffs_left"] >> dist_coeffs1;
    fs["distCoeffs_right"] >> dist_coeffs2;
    fs["R"] >> R;
    fs["T"] >> T;
    fs.release();
    sensor_msgs::msg::CameraInfo left_info, right_info;
    left_info.header.frame_id = "left_camera";
    right_info.header.frame_id = "right_camera";
    left_info.height = 480;
    left_info.width = 640;
    right_info.height = 480;
    right_info.width = 640;
    left_info.distortion_model = "plumb_bob";
    right_info.distortion_model = "plumb_bob";
    left_info.d.resize(5);
    right_info.d.resize(5);
    for (int i = 0; i < 5; i++) {
        left_info.d[i] = dist_coeffs1.at<double>(i);
        right_info.d[i] = dist_coeffs2.at<double>(i);
    }
    left_info.k.fill(0.0);
    right_info.k.fill(0.0);
    left_info.k[0] = camera_matrix1.at<double>(0, 0);
    left_info.k[2] = camera_matrix1.at<double>(0, 2);
    left_info.k[4] = camera_matrix1.at<double>(1, 1);
    left_info.k[5] = camera_matrix1.at<double>(1, 2);
    left_info.k[8] = 1.0;
    right_info.k[0] = camera_matrix2.at<double>(0, 0);
    right_info.k[2] = camera_matrix2.at<double>(0, 2);
    right_info.k[4] = camera_matrix2.at<double>(1, 1);
    right_info.k[5] = camera_matrix2.at<double>(1, 2);
    right_info.k[8] = 1.0;
    left_info.r.fill(0.0);
    right_info.r.fill(0.0);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            left_info.r[i * 3 + j] = R.at<double>(i, j);
            right_info.r[i * 3 + j] = R.at<double>(i, j);
        }
    }
    left_info.p.fill(0.0);
    right_info.p.fill(0.0);
    left_info.p[0] = camera_matrix1.at<double>(0, 0);
    left_info.p[2] = camera_matrix1.at<double>(0, 2);
    left_info.p[5] = camera_matrix1.at<double>(1, 1);
    left_info.p[6] = camera_matrix1.at<double>(1, 2);
    left_info.p[10] = 1.0;
    right_info.p[0] = camera_matrix2.at<double>(0, 0);
    right_info.p[2] = camera_matrix2.at<double>(0, 2);
    right_info.p[5] = camera_matrix2.at<double>(1, 1);
    right_info.p[6] = camera_matrix2.at<double>(1, 2);
    right_info.p[10] = 1.0;
    left_info.header.frame_id = "left_camera";
    right_info.header.frame_id = "right_camera";

    // extract the transformation between the camera frames for static transform publisher
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(camera_matrix1, dist_coeffs1, camera_matrix2, dist_coeffs2, cv::Size(640, 480), R, T, R1, R2, P1, P2, Q);
    cv::Mat R1_inv = R1.inv();
    cv::Mat R2_inv = R2.inv();
    cv::Mat T_inv = -R1_inv * T;
    cv::Mat R1_vec, R2_vec;
    cv::Rodrigues(R1_inv, R1_vec);
    cv::Rodrigues(R2_inv, R2_vec);

    // static transform publisher
    rclcpp::Node::SharedPtr static_transform_node = rclcpp::Node::make_shared("static_transform_publisher");
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr static_transform_publisher = static_transform_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static",   mQos);
    tf2_msgs::msg::TFMessage static_transform_msg;
    geometry_msgs::msg::TransformStamped left_transform, right_transform;
    left_transform.header.frame_id = "imu_link"; // center of the glasses
    left_transform.header.stamp = node->now();
    left_transform.child_frame_id = "left_camera";
    right_transform.header.frame_id = "imu_link";
    right_transform.header.stamp = node->now();
    right_transform.child_frame_id = "right_camera";
    left_transform.transform.translation.x = T_inv.at<double>(0);
    left_transform.transform.translation.y = T_inv.at<double>(1);
    left_transform.transform.translation.z = T_inv.at<double>(2);
    right_transform.transform.translation.x = T_inv.at<double>(0);
    right_transform.transform.translation.y = T_inv.at<double>(1);
    right_transform.transform.translation.z = T_inv.at<double>(2);
    left_transform.transform.rotation.x = R1_vec.at<double>(0);
    left_transform.transform.rotation.y = R1_vec.at<double>(1);
    left_transform.transform.rotation.z = R1_vec.at<double>(2);
    left_transform.transform.rotation.w = 1.0;
    right_transform.transform.rotation.x = R2_vec.at<double>(0);
    right_transform.transform.rotation.y = R2_vec.at<double>(1);
    right_transform.transform.rotation.z = R2_vec.at<double>(2);
    right_transform.transform.rotation.w = 1.0;
    static_transform_msg.transforms.push_back(left_transform);
    static_transform_msg.transforms.push_back(right_transform);
    static_transform_publisher->publish(static_transform_msg);

    // TODO: use URDF to define the static transform

    cv::Mat imageL, imageR;
    cv::Mat frame, heat_map, depth_map, disparity, rgb, smoothedFrame;
    
    //init
    void * stereo=vision_core::Initialize(stereo_calibration_path);

    //x,y,z,r,g,b
    float* pointcloud=new float[640*480*6];

    int device_id = argc > 2 ? std::stoi(argv[2]) : 0;
    int device_rgb_id = argc > 3 ? std::stoi(argv[3]) : 2;
    // cv::VideoCapture cap(device_id), rgb_cap;
    cv::VideoCapture cap("out.avi"), rgb_cap;
    if (argc > 3){
        rgb_cap.open(device_rgb_id);
        rgb_cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);
        // set width and height
        rgb_cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        rgb_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        // set fps
        rgb_cap.set(cv::CAP_PROP_FPS, 60);
    }
        
    //, rgb_cap(2);
    // cap.set(cv::CAP_PROP_FORMAT, CV_8UC1);
    // cap.set(cv::CAP_PROP_CONVERT_RGB, false);
    // cap.set(cv::CAP_PROP_FPS, 25);
    double exposureValue = -5; // Usually, negative values are used for manual exposure


    double alpha = 0.5;  // Smoothing factor

    cap >> frame;
    smoothedFrame = frame.clone();
    smoothedFrame.convertTo(smoothedFrame, CV_32F);
    while (cap.isOpened() && rclcpp::ok()){
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat floatFrame;
        frame.convertTo(floatFrame, CV_32F);
        smoothedFrame = alpha * floatFrame + (1 - alpha) * smoothedFrame;
        cv::Mat displayFrame;
        smoothedFrame.convertTo(displayFrame, CV_8U);
        // cv::cvtColor(displayFrame, displayFrame, cv::COLOR_BGR2GRAY);
        // rgb_cap >> rgb;
        // convert to gray
        // split frame to left and right
        cv::Mat imageL = cv::Mat(displayFrame, cv::Rect(0, 0, frame.cols / 2, frame.rows));
        cv::Mat imageR = cv::Mat(displayFrame, cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
        // resize to 640x480
        cv::resize(imageL, imageL, cv::Size(640, 480));
        cv::resize(imageR, imageR, cv::Size(640, 480));
        //need RectifyImage
        vision_core::getDisparity(stereo,imageL,imageR,pointcloud,disparity);
        

        
        heat_map = heatmap(disparity);
        
        // normalize depth map
        
        // depth map publisher
        depth_map = 0.1 * 320 / disparity;
        image_msg.data.clear();
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        out_msg.header.stamp = node->now();
        out_msg.image = depth_map;
        out_msg.toImageMsg(image_msg);
        publisher_image->publish(image_msg);


        //  to gray
        // cv::cvtColor(imageL, imageL, cv::COLOR_BGR2GRAY);
        // cv::cvtColor(imageR, imageR, cv::COLOR_BGR2GRAY);
        // left image publisher
        left_image_msg.data.clear();
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;//MONO8;
        out_msg.image = imageL;
        out_msg.toImageMsg(left_image_msg);
        left_image_msg.header.stamp = node->now();
        publisher_left->publish(left_image_msg);

        // right image publisher
        right_image_msg.data.clear();
        out_msg.image = imageR;
        out_msg.toImageMsg(right_image_msg);
        right_image_msg.header.stamp = node->now();
        publisher_right->publish(right_image_msg);

        // pointcloud publisher
        pointcloud_msg.data.clear();
        pointcloud_msg.data.resize(640 * 480 * 24);
        data_buffer.clear();
        for (int i = 0; i < 640 * 480 * 6; i += 6) {
            data_buffer.push_back(pointcloud[i +2]/1000);
            data_buffer.push_back(-pointcloud[i]/1000);
            data_buffer.push_back(-pointcloud[i + 1]/1000);
            data_buffer.push_back(pointcloud[i + 3]);
            data_buffer.push_back(pointcloud[i + 4]);
            data_buffer.push_back(pointcloud[i + 5]);
        }
        memcpy(pointcloud_msg.data.data(), data_buffer.data(), pointcloud_msg.data.size());
        pointcloud_msg.header.stamp = node->now();
        publisher_->publish(pointcloud_msg);

        // cv::normalize(depth_map, depth_map, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        // cv::imshow("frame", displayFrame);
        // cv::imshow("heatmap", heat_map);
        // cv::imshow("depth_map", depth_map);
        // cv::waitKey(1);
        left_info.header.stamp = node->now();
        publisher_left_info->publish(left_info);
        right_info.header.stamp = node->now();
        publisher_right_info->publish(right_info);
        // static_transform_publisher->publish(static_transform_msg);

    }

    vision_core::Release(stereo);
    delete []pointcloud;
    pointcloud=nullptr;
}

int main(int argc, char **argv)
{   
    std::thread stereo_thread(stereo, argc, argv);

    stereo_thread.join();
    return 0;
}