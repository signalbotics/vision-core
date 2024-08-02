// #include <opencv4/opencv2/opencv.hpp>
// #include <cmath>
// #include "mono.hpp"
// #include "vision_core/mono_depth.h"
// #include <opencv2/surface_matching.hpp> // Include the ICP module


// // Using std and sl namespaces
// using namespace std;
// // using namespace sl;

// // void parseArgs(int argc, char **argv, sl::InitParameters& param);
// // cv::Mat slMat2cvMat(sl::Mat& input);
// float cx = 611.0380249023438, cy = 402.2382507324219, fx = 749.0894775390625, fy = 749.0894775390625;




// MonoDepth depth_model;
// sensor_msgs::msg::Image ros_image_left, ros_image_right;
// sensor_msgs::msg::PointCloud2 rosPointCloud;

// sensor_msgs::msg::PointCloud2 visualizePointCloud(cv::Mat& point_cloud, cv::Mat& color_image) {
//     sensor_msgs::msg::PointCloud2 ros_pt2;
//     ros_pt2.header.frame_id = "zedm_left_camera_frame";
//     ros_pt2.height = 1; // Unordered point cloud has height of 1
//     ros_pt2.width = point_cloud.rows * point_cloud.cols;
//     ros_pt2.is_bigendian = false;
//     ros_pt2.is_dense = true;
//     ros_pt2.point_step = 16; // Each point consists of 12 bytes for XYZ (3x4 bytes) + 12 bytes for RGB (3x4 bytes)
//     ros_pt2.row_step = ros_pt2.point_step * ros_pt2.width;
//     // Clear any existing fields to prevent duplicates
//     ros_pt2.fields.clear();

//     // Define and add all necessary fields
//     std::vector<std::string> field_names = {"x", "y", "z", "rgb"};
//     int offset = 0;
//     for (const auto& name : field_names) {
//         sensor_msgs::msg::PointField field;
//         field.name = name;
//         field.offset = offset;
//         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
//         field.count = 1;
//         ros_pt2.fields.push_back(field);
//         offset += 4; // Move to the next field location
//     }

//     std::vector<float> data_buffer(ros_pt2.width * ros_pt2.point_step / sizeof(float), 0.0f);

//     for (int y = 0; y < point_cloud.rows; ++y) {
//         for (int x = 0; x < point_cloud.cols; ++x) {
//             cv::Vec3f pt = point_cloud.at<cv::Vec3f>(y, x);
//             // Conversion to ROS "z-up" coordinate system
//             int index = (y * point_cloud.cols + x) * 4; // 6 floats per point (XYZRGB)
//             data_buffer[index + 0] = pt[2];
//             data_buffer[index + 1] = -pt[0];
//             data_buffer[index + 2] = -pt[1];

//             cv::Vec3b color = color_image.at<cv::Vec3b>(y, x);

//             uint8_t r = color[2];
//             uint8_t g = color[1];
//             uint8_t b = color[0];
//             uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//             float* rgb_float = reinterpret_cast<float*>(&data_buffer[index + 3]);
//             *rgb_float = *reinterpret_cast<float*>(&rgb);
//         }
//     }

//     ros_pt2.data = std::vector<uint8_t>(reinterpret_cast<uint8_t*>(data_buffer.data()), reinterpret_cast<uint8_t*>(data_buffer.data() + data_buffer.size()));

//     return ros_pt2;
// }

// sensor_msgs::msg::PointCloud2 depthMapToPointCloud(cv::Mat& depthMap, cv::Mat& colorImage, const double fx, const double fy) {
//     // resize the depth map to the same size as the color image
//     // cv::imshow("colorImage", colorImage);
//     // cv::waitKey(1);
//     sensor_msgs::msg::PointCloud2 ros_pt2;
//     ros_pt2.header.frame_id = "zedm_left_camera_frame";
//     ros_pt2.height = 1; // Unordered point cloud has height of 1
//     ros_pt2.width = depthMap.rows * depthMap.cols;
//     // get the max depth 
//     float max_depth = 0;
//     // int x_, y_;
//     // for (int y = 0; y < depthMap.rows; ++y) {
//     //     for (int x = 0; x < depthMap.cols; ++x) {
//     //         if (depthMap.at<float>(y, x) > max_depth) {
//     //             max_depth = depthMap.at<float>(y, x);
//     //             x_ = x;
//     //             y_ = y;
//     //         }
//     //     }
//     // }
//     // float slop = 0.5*max_depth;
//     ros_pt2.is_bigendian = false;
//     ros_pt2.is_dense = true;
//     ros_pt2.point_step = 16; // Each point consists of 12 bytes for XYZ (3x4 bytes) + 12 bytes for RGB (3x4 bytes)
//     ros_pt2.row_step = ros_pt2.point_step * ros_pt2.width;
//     // Clear any existing fields to prevent duplicates
//     ros_pt2.fields.clear();

//     // Define and add all necessary fields
//     std::vector<std::string> field_names = {"x", "y", "z", "rgb"};
//     int offset = 0;
//     for (const auto& name : field_names) {
//         sensor_msgs::msg::PointField field;
//         field.name = name;
//         field.offset = offset;
//         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
//         field.count = 1;
//         ros_pt2.fields.push_back(field);
//         offset += 4; // Move to the next field location
//     }

//     std::vector<float> data_buffer(ros_pt2.width * ros_pt2.point_step / sizeof(float), 0.0f);

//     for (int y = 0; y < depthMap.rows; ++y) {
//         for (int x = 0; x < depthMap.cols; ++x) {
//             float Z = depthMap.at<float>(y, x);
//             float X = (x - depthMap.cols / 2) * Z / fx;
//             float Y = (y - depthMap.rows / 2) * Z / fy;

//             // Conversion to ROS "z-up" coordinate system
//             int index = (y * depthMap.cols + x) * 4; // 6 floats per point (XYZRGB)
//             data_buffer[index + 0] = Z;
//             data_buffer[index + 1] = -X;
//             data_buffer[index + 2] = -Y;

//             cv::Vec3b color = colorImage.at<cv::Vec3b>(y, x);

//             uint8_t r = color[2];
//             uint8_t g = color[1];
//             uint8_t b = color[0];
//             uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//             float* rgb_float = reinterpret_cast<float*>(&data_buffer[index + 3]);
//             *rgb_float = *reinterpret_cast<float*>(&rgb);
            
//         }
//     }

//     ros_pt2.data = std::vector<uint8_t>(reinterpret_cast<uint8_t*>(data_buffer.data()), reinterpret_cast<uint8_t*>(data_buffer.data() + data_buffer.size()));

//     return ros_pt2;
// }
// cv::Matx44d  alignPointClouds(const cv::Mat& src, const cv::Mat& dst) {
//     // Initialize ICP
//     cv::ppf_match_3d::ICP icp(10000, 200);
//     // icp.setMaxIterations(100);
//     // icp.setTolerance(0.05);

//     // Perform ICP
//     std::vector<cv::ppf_match_3d::Pose3DPtr> poses;

//     // Perform ICP alignment
//     int result = icp.registerModelToScene(src, dst, poses);


//     if (result >= 0 && !poses.empty()) {
//         std::cout << "ICP succeeded, number of poses: " << poses.size() << std::endl;
//         // Assuming the first pose is the best one
//         return poses.front()->pose;
//     } else {
//         return cv::Matx44d::eye(); // Return identity matrix if ICP fails
//     }


// }

// void applyTransformationToPointCloud(cv::Mat& pointCloud, const cv::Matx44d& transformation) {
//     // Ensure the point cloud is in the correct format, i.e., CV_32FC3
//     for (int i = 0; i < pointCloud.rows; ++i) {
//         for (int j = 0; j < pointCloud.cols; ++j) {
//             cv::Vec3f& point = pointCloud.at<cv::Vec3f>(i, j);
//             cv::Vec4f pointHomogeneous(point[0], point[1], point[2], 1.0f); // Convert to homogeneous coordinates

//             // Apply the transformation matrix
//             cv::Vec4f transformedPoint = transformation * pointHomogeneous;

//             // Convert back to 3D point and store in the point cloud
//             point = cv::Vec3f(transformedPoint[0], transformedPoint[1], transformedPoint[2]);
//         }
//     }
// }
// cv::Mat applyCustomTransformationToPointCloud(cv::Mat& pointCloud, double scaleX, double scaleY, double scaleZ, const cv::Point3f& translation) {
//     cv::Mat transformed_point_cloud(pointCloud.size(), CV_32FC3);
//     for (int i = 0; i < pointCloud.rows; ++i) {
//         for (int j = 0; j < pointCloud.cols; ++j) {
//             cv::Vec3f& point = pointCloud.at<cv::Vec3f>(i, j);
//             // Apply different scaling and translation to each dimension
//             transformed_point_cloud.at<cv::Vec3f>(i, j) = cv::Vec3f(
//                 point[0] * scaleX + translation.x,
//                 point[1] * scaleY + translation.y,
//                 (point[2]<0?-point[2]:point[2]) * scaleZ + translation.z
//             );

        
//         }
//     }
//     return transformed_point_cloud;
// }



// int main(int argc, char **argv) {

//     // Camera zed;
//     // Set configuration parameters for the ZED
//     // InitParameters init_parameters;
//     // init_parameters.depth_mode = DEPTH_MODE::ULTRA;
//     // init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
//     // init_parameters.sdk_verbose = 1;
//     // parseArgs(argc, argv, init_parameters);
//     // capture the video from the camera
//     // cv::VideoCapture cap(0);
//     // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//     // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
//     // if (!cap.isOpened()) {
//     //     std::cerr << "Error opening the camera" << std::endl;
//     //     return -1;
//     // }
//     std::cout << "Loading model from " << "depth_anything_vits14.engine" << "..." << std::endl;
//     cv::Mat frame, result_d,depth;

//     // if (frame.empty()) {
//     //     std::cerr << "Error capturing the frame" << std::endl;
//     //     return -1;
//     // }
//     // Open the camera
//     // auto returned_state = zed.open(init_parameters);
//     // if (returned_state != ERROR_CODE::SUCCESS) {
//     //     print("Camera Open", returned_state, "Exit program.");
//     //     return EXIT_FAILURE;
//     // }

//     // auto camera_config = zed.getCameraInformation().camera_configuration;
//     // float image_aspect_ratio = camera_config.resolution.width / (1.f * camera_config.resolution.height);
//     // int requested_low_res_w = min(720, (int)camera_config.resolution.width);
//     // sl::Resolution res(requested_low_res_w, requested_low_res_w / image_aspect_ratio);

//     // auto stream = zed.getCUDAStream();
//     // SensorsData sensors_data;

//     // RuntimeParameters runParameters;
//     // // Setting the depth confidence parameters
//     // runParameters.confidence_threshold = 50;
//     // runParameters.texture_confidence_threshold = 100;

//     // Allocation of 4 channels of float on GPU
//     // Mat image(res.width, res.height, sl::MAT_TYPE::U8_C4);
//     // Mat point_cloud(res.width, res.height, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

//     // // Main Loop
//     // while (cap.isOpened() && rclcpp::ok()) {     
//     //     // Check that a new image is successfully acquired
//     //     // if (zed.grab(runParameters) == ERROR_CODE::SUCCESS) {
//     //         // retrieve the current 3D coloread point cloud in GPU
//     //         // zed.retrieveImage(image, VIEW::LEFT);
//     //         // cv::Mat image_ocv = slMat2cvMat(image);
//     //         // cv::Mat result_d,depth;

//     //         cap >> frame;

//     //     //     auto pred = depth_model.predict(frame);
//     //     //     // auto pred = depth_model.predict(image_ocv);
//     //     //     result_d = pred.first;
//     //     //     depth = pred.second;

//     //     //    // Convert depth map and color image to point cloud
//     //     //     sensor_msgs::msg::PointCloud2 rosPointCloud = depthMapToPointCloud(depth, frame, 702.7350, 702.735);
//     //     //     // Visualize the point cloud
//     //     //     pub->publish(rosPointCloud);

//     //     //     sensor_msgs::msg::Image rosImage;
//     //     //     rosImage.header.frame_id = "map";
//     //     //     rosImage.height = depth.rows;
//     //     //     rosImage.width = depth.cols;
//     //     //     rosImage.encoding = "32FC1";
//     //     //     rosImage.is_bigendian = false;
//     //     //     rosImage.step = sizeof(float) * depth.cols;
//     //     //     rosImage.data = std::vector<uint8_t>(depth.data, depth.data + depth.total() * depth.elemSize());
//     //     //     pub_img->publish(rosImage);
//     //         // Check for ESC key press or ctrl + c
//     //         if (cv::waitKey(5) == 27) // Exit loop when 'Esc' key (ASCII code 27) is pressed
//     //             break;
//     //         if (cv::waitKey(5) == 27) // Exit loop when 'Esc' key (ASCII code 27) is pressed
//     //             break;
//     //     // }
//     // }

//     // Free allocated memory before closing the ZED
//     // Close the ZED
//     // zed.close();
//     rclcpp::shutdown();

//     return 0;
// }

