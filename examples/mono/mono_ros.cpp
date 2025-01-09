
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <cmath>
#include "vision_core/mono_depth.hpp"
#include <opencv2/surface_matching.hpp> // Include the ICP module
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// Using std and sl namespaces
using namespace std;
// using namespace sl;

// void parseArgs(int argc, char **argv, sl::InitParameters& param);
// cv::Mat slMat2cvMat(sl::Mat& input);
// float fx = 749.0894775390625, fy = 749.0894775390625, cx = 611.0380249023438, cy = 402.2382507324219, pitch = 0;
float fx = 749.0894775390625, fy = 749.0894775390625, cx = 611.0380249023438, cy = 402.2382507324219, pitch = 0;


MonoDepth depth_model;
sensor_msgs::msg::Image ros_image_left, ros_image_right;
sensor_msgs::msg::PointCloud2 rosPointCloud;

sensor_msgs::msg::PointCloud2 visualizePointCloud(cv::Mat& point_cloud, cv::Mat& color_image) {
    sensor_msgs::msg::PointCloud2 ros_pt2;
    ros_pt2.header.frame_id = "zedm_left_camera_frame";
    ros_pt2.height = 1; // Unordered point cloud has height of 1
    ros_pt2.width = point_cloud.rows * point_cloud.cols;
    ros_pt2.is_bigendian = false;
    ros_pt2.is_dense = true;
    ros_pt2.point_step = 16; // Each point consists of 12 bytes for XYZ (3x4 bytes) + 12 bytes for RGB (3x4 bytes)
    ros_pt2.row_step = ros_pt2.point_step * ros_pt2.width;
    // Clear any existing fields to prevent duplicates
    ros_pt2.fields.clear();

    // Define and add all necessary fields
    std::vector<std::string> field_names = {"x", "y", "z", "rgb"};
    int offset = 0;
    for (const auto& name : field_names) {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        ros_pt2.fields.push_back(field);
        offset += 4; // Move to the next field location
    }

    std::vector<float> data_buffer(ros_pt2.width * ros_pt2.point_step / sizeof(float), 0.0f);

    for (int y = 0; y < point_cloud.rows; ++y) {
        for (int x = 0; x < point_cloud.cols; ++x) {
            cv::Vec3f pt = point_cloud.at<cv::Vec3f>(y, x);
            // Conversion to ROS "z-up" coordinate system
            int index = (y * point_cloud.cols + x) * 4; // 6 floats per point (XYZRGB)
            data_buffer[index + 0] = pt[2];
            data_buffer[index + 1] = -pt[0];
            data_buffer[index + 2] = -pt[1];

            cv::Vec3b color = color_image.at<cv::Vec3b>(y, x);

            uint8_t r = color[2];
            uint8_t g = color[1];
            uint8_t b = color[0];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            float* rgb_float = reinterpret_cast<float*>(&data_buffer[index + 3]);
            *rgb_float = *reinterpret_cast<float*>(&rgb);
        }
    }

    ros_pt2.data = std::vector<uint8_t>(reinterpret_cast<uint8_t*>(data_buffer.data()), reinterpret_cast<uint8_t*>(data_buffer.data() + data_buffer.size()));
    std::cout << "point cloud size: " << ros_pt2.data.size() << std::endl;
    return ros_pt2;
}

sensor_msgs::msg::PointCloud2 depthMapToPointCloud(cv::Mat& depthMap, cv::Mat& colorImage, const double fx, const double fy) {
    // resize the depth map to the same size as the color image
    // cv::imshow("colorImage", colorImage);
    // cv::waitKey(1);
    sensor_msgs::msg::PointCloud2 ros_pt2;
    ros_pt2.header.frame_id = "zedm_left_camera_frame";
    ros_pt2.height = 1; // Unordered point cloud has height of 1
    ros_pt2.width = depthMap.rows * depthMap.cols;
    // get the max depth 
    float max_depth = 0;
    // int x_, y_;
    // for (int y = 0; y < depthMap.rows; ++y) {
    //     for (int x = 0; x < depthMap.cols; ++x) {
    //         if (depthMap.at<float>(y, x) > max_depth) {
    //             max_depth = depthMap.at<float>(y, x);
    //             x_ = x;
    //             y_ = y;
    //         }
    //     }
    // }
    // float slop = 0.5*max_depth;
    ros_pt2.is_bigendian = false;
    ros_pt2.is_dense = true;
    ros_pt2.point_step = 16; // Each point consists of 12 bytes for XYZ (3x4 bytes) + 12 bytes for RGB (3x4 bytes)
    ros_pt2.row_step = ros_pt2.point_step * ros_pt2.width;
    // Clear any existing fields to prevent duplicates
    ros_pt2.fields.clear();

    // Define and add all necessary fields
    std::vector<std::string> field_names = {"x", "y", "z", "rgb"};
    int offset = 0;
    for (const auto& name : field_names) {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        ros_pt2.fields.push_back(field);
        offset += 4; // Move to the next field location
    }

    std::vector<float> data_buffer(ros_pt2.width * ros_pt2.point_step / sizeof(float), 0.0f);

    for (int y = 0; y < depthMap.rows; ++y) {
        for (int x = 0; x < depthMap.cols; ++x) {
            float Z = depthMap.at<float>(y, x);
            float X = (x - depthMap.cols / 2) * Z / fx;
            float Y = (y - depthMap.rows / 2) * Z / fy;

            // Conversion to ROS "z-up" coordinate system
            int index = (y * depthMap.cols + x) * 4; // 6 floats per point (XYZRGB)
            data_buffer[index + 0] = Z;
            data_buffer[index + 1] = -X;
            data_buffer[index + 2] = -Y;

            cv::Vec3b color = colorImage.at<cv::Vec3b>(y, x);

            uint8_t r = color[2];
            uint8_t g = color[1];
            uint8_t b = color[0];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            float* rgb_float = reinterpret_cast<float*>(&data_buffer[index + 3]);
            *rgb_float = *reinterpret_cast<float*>(&rgb);
            
        }
    }

    ros_pt2.data = std::vector<uint8_t>(reinterpret_cast<uint8_t*>(data_buffer.data()), reinterpret_cast<uint8_t*>(data_buffer.data() + data_buffer.size()));

    return ros_pt2;
}
cv::Matx44d  alignPointClouds(const cv::Mat& src, const cv::Mat& dst) {
    // Initialize ICP
    cv::ppf_match_3d::ICP icp(10000, 200);
    // icp.setMaxIterations(100);
    // icp.setTolerance(0.05);

    // Perform ICP
    std::vector<cv::ppf_match_3d::Pose3DPtr> poses;

    // Perform ICP alignment
    int result = icp.registerModelToScene(src, dst, poses);


    if (result >= 0 && !poses.empty()) {
        std::cout << "ICP succeeded, number of poses: " << poses.size() << std::endl;
        // Assuming the first pose is the best one
        return poses.front()->pose;
    } else {
        return cv::Matx44d::eye(); // Return identity matrix if ICP fails
    }


}

void applyTransformationToPointCloud(cv::Mat& pointCloud, const cv::Matx44d& transformation) {
    // Ensure the point cloud is in the correct format, i.e., CV_32FC3
    for (int i = 0; i < pointCloud.rows; ++i) {
        for (int j = 0; j < pointCloud.cols; ++j) {
            cv::Vec3f& point = pointCloud.at<cv::Vec3f>(i, j);
            cv::Vec4f pointHomogeneous(point[0], point[1], point[2], 1.0f); // Convert to homogeneous coordinates

            // Apply the transformation matrix
            cv::Vec4f transformedPoint = transformation * pointHomogeneous;

            // Convert back to 3D point and store in the point cloud
            point = cv::Vec3f(transformedPoint[0], transformedPoint[1], transformedPoint[2]);
        }
    }
}
cv::Mat applyCustomTransformationToPointCloud(cv::Mat& pointCloud, double scaleX, double scaleY, double scaleZ, const cv::Point3f& translation) {
    cv::Mat transformed_point_cloud(pointCloud.size(), CV_32FC3);
    for (int i = 0; i < pointCloud.rows; ++i) {
        for (int j = 0; j < pointCloud.cols; ++j) {
            cv::Vec3f& point = pointCloud.at<cv::Vec3f>(i, j);
            // Apply different scaling and translation to each dimension
            transformed_point_cloud.at<cv::Vec3f>(i, j) = cv::Vec3f(
                point[0] * scaleX + translation.x,
                point[1] * scaleY + translation.y,
                (point[2]<0?-point[2]:point[2]) * scaleZ + translation.z
            );

        
        }
    }
    return transformed_point_cloud;
}

void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr image_l, const sensor_msgs::msg::Image::ConstSharedPtr image_r, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_l, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_r) {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received stereo images");
    cv::Mat left_image = cv_bridge::toCvCopy(image_l, "bgr8")->image;
    cv::Mat right_image = cv_bridge::toCvCopy(image_r, "bgr8")->image;
    auto start_time = std::chrono::high_resolution_clock::now();
    cv::Mat depth_l, depth_r, color_l, color_r;
    auto pred = depth_model.predict(left_image);
    color_l = pred.first;
    // depth_l = pred.second /1.5;
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Time taken to predict depth: " << duration.count() << "ms" << std::endl;
    std::cout << "Depth map size: " << depth_l.size() << std::endl;
    std::cout << "intrinsics: " << fx << " " << fy << " " << cx << " " << cy << std::endl;
    cv::Mat mono_pt_cloud = cv::Mat::zeros(left_image.size(), CV_32FC3);
    for (int i = 0; i < depth_l.rows; i++) {
        for (int j = 0; j < depth_l.cols; j++) {
            mono_pt_cloud.at<cv::Vec3f>(i, j) =  cv::Vec3f(
                                                (j - 518/2) * depth_l.at<float>(i, j) / fx, 
                                                (i - 518/2) * depth_l.at<float>(i, j) / fy, 
                                                depth_l.at<float>(i, j)
                                                );
            

        }
    }
    if (pitch != 0) {
        cv::Matx33f R;
        R << 1, 0, 0,
            0, cos(pitch), -sin(pitch),
            0, sin(pitch), cos(pitch);

        // Reshape mono_pt_cloud to a 2D matrix for matrix multiplication
        cv::Mat reshaped_pt_cloud = mono_pt_cloud.reshape(1, mono_pt_cloud.rows * mono_pt_cloud.cols);
        cv::Mat rotated_pt_cloud = reshaped_pt_cloud * cv::Mat(R).t();
        mono_pt_cloud = rotated_pt_cloud.reshape(3, mono_pt_cloud.rows);
    }
    // start_time = std::chrono::high_resolution_clock::now();
    // cv::Ptr<cv::ORB> orb = cv::ORB::create();
    // std::vector<cv::KeyPoint> keypoints_l, keypoints_r;
    // cv::Mat descriptors_l, descriptors_r;
    // orb->detectAndCompute(left_image, cv::noArray(), keypoints_l, descriptors_l);
    // orb->detectAndCompute(right_image, cv::noArray(), keypoints_r, descriptors_r);
    // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    // std::vector<cv::DMatch> matches;
    // matcher->match(descriptors_l, descriptors_r, matches);
    // std::vector<cv::DMatch> good_matches;
    // double max_dist = 0; double min_dist = 100;
    // for (int i = 0; i < descriptors_l.rows; i++) {
    //     double dist = matches[i].distance;
    //     if (dist < min_dist) min_dist = dist;
    //     if (dist > max_dist) max_dist = dist;
    // }
    // for (int i = 0; i < descriptors_l.rows; i++) {
    //     if (matches[i].distance <= std::max(2 * min_dist, 0.01)) {
    //         good_matches.push_back(matches[i]);
    //     }
    // }
    // // cv::Mat img_matches;
    // // cv::drawMatches(left_image, keypoints_l, right_image, keypoints_r, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // // cv::imshow("Matches", img_matches);
    // // cv::waitKey(1);
    // // cout << "Number of matches: " << good_matches.size() << endl;
    // // get the pixel disparity for each match
    // end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // std::cout << "Time taken to match keypoints: " << duration.count() << "ms" << std::endl;
    
    // start_time = std::chrono::high_resolution_clock::now();
    // cv::Mat depth_map = cv::Mat::zeros(left_image.size(), CV_32F);
    // cv::Mat mono_pt_cloud = cv::Mat::zeros(left_image.size(), CV_32FC3);
    // for (int i = 0; i < depth_l.rows; i++) {
    //     for (int j = 0; j < depth_l.cols; j++) {
    //         mono_pt_cloud.at<cv::Vec3f>(i, j) =  cv::Vec3f(
    //                                             (j - cx) * depth_l.at<float>(i, j) / fx, 
    //                                             (i - cy) * depth_l.at<float>(i, j) / fy, 
    //                                             depth_l.at<float>(i, j)
    //                                             );
    //     }
    // }

    // // float scale_factor = 0;
    // double sumScaleX = 0, sumScaleY = 0, sumScaleZ = 0;

    // std::vector<cv::Point3f> points_mono, points_stereo;
    // cv::Mat points_mono_mat(good_matches.size(), 3, CV_32F);
    // cv::Mat points_stereo_mat(good_matches.size(), 3, CV_32F);
    // // cv::Mat src_points_mat(good_matches.size(), 3, CV_32F);
    // cv::Mat dst_points_mat(good_matches.size(), 3, CV_32F);

    // for (int i = 0; i < good_matches.size(); i++) {
    //     cv::Point2f pt_left = keypoints_l[good_matches[i].queryIdx].pt;
    //     cv::Point2f pt_right = keypoints_r[good_matches[i].trainIdx].pt;
    //     float d = pt_left.x - pt_right.x;
    //     if (d < 0) continue;
    //     float dp = 0.06 * fx/d;

    //     dst_points_mat.at<float>(i, 0)  = (pt_left.x - cx) * dp / fx;
    //     dst_points_mat.at<float>(i, 1)  = (pt_left.y - cy) * dp / fy;
    //     dst_points_mat.at<float>(i, 2)  = dp;
    //     cv::Point3f pt_stereo(dst_points_mat.at<float>(i, 0), dst_points_mat.at<float>(i, 1), dst_points_mat.at<float>(i, 2));

    //     points_stereo.push_back(pt_stereo);
    //     points_mono.push_back(cv::Point3f(mono_pt_cloud.at<cv::Vec3f>(pt_left.y, pt_left.x)));// Convert image coordinates to 3D points
    //     points_mono_mat.at<cv::Vec3f>(i) = mono_pt_cloud.at<cv::Vec3f>(pt_left.y, pt_left.x);
    //     points_stereo_mat.at<cv::Vec3f>(i) = cv::Vec3f(
    //                                                 (pt_left.x - cx) * dp / fx, 
    //                                                 (pt_left.y - cy) * dp / fy, 
    //                                                 dp
    //                                             );
    
    // }

    // // Compute centroids
    // cv::Scalar centroid_mono = cv::mean(points_mono_mat);
    // cv::Scalar centroid_stereo = cv::mean(points_stereo_mat);

    // // Subtract centroids// Assuming points_mono_mat and points_stereo_mat are of type CV_32FC3
    // cv::Mat centroid_mono_mat(points_mono_mat.size(), points_mono_mat.type(), centroid_mono);
    // cv::Mat centroid_stereo_mat(points_stereo_mat.size(), points_stereo_mat.type(), centroid_stereo);

    // points_mono_mat -= centroid_mono_mat;
    // points_stereo_mat -= centroid_stereo_mat;
    
    // // Compute matrix W = points_mono_mat^T * points_stereo_mat
    // cv::Mat W;
    // cv::gemm(points_mono_mat.t(), points_stereo_mat, 1, cv::Mat(), 0, W);
    // // cv::gemm(points_mono_mat, points_stereo_mat.t(), 1.0, cv::Mat(), 0.0, W);  // now A * B^T

    // // Perform SVD
    // cv::SVD svd(W);

    // // Compute scale factor as the average of the singular values
    // cv::Mat singular_values = svd.w;
    // double scale_factor = cv::sum(singular_values)[0] / singular_values.rows;
    // cout << "Scale factor: " << scale_factor << endl;
    // // Apply the scale factor to the monocular point cloud
    // for (int i = 0; i < mono_pt_cloud.rows; i++) {
    //     mono_pt_cloud.at<cv::Vec3f>(i)[0] /= scale_factor;  // Scale x-coordinate
    //     mono_pt_cloud.at<cv::Vec3f>(i)[1] /= scale_factor;  // Scale y-coordinate
    //     mono_pt_cloud.at<cv::Vec3f>(i)[2] /= scale_factor;  // Scale z-coordinate (depth)
    // }



    // cv::Mat transformed_point_cloud(mono_pt_cloud.size(), CV_32FC3);

    // // cv::Matx44d transformation = alignPointClouds(src_points_mat, dst_points_mat);
    // // if (transformation == cv::Matx44d::eye()) {
    // //     std::cerr << "ICP failed to find a valid transformation." << std::endl;
    // //     return;
    // // }
    // // applyTransformationToPointCloud(mono_pt_cloud, transformation);


    // // Compute the centroids of the points
    // cv::Point3f centroid_mono(0, 0, 0), centroid_stereo(0, 0, 0);
    // for (int i = 0; i < points_mono.size(); ++i) {
    //     centroid_mono += points_mono[i];
    //     centroid_stereo += points_stereo[i];
    // }
    // centroid_mono *= (1.0 / points_mono.size());
    // centroid_stereo *= (1.0 / points_stereo.size());

    // // Subtract the centroids from the points
    // std::vector<cv::Point3f> centered_mono, centered_stereo;
    // for (int i = 0; i < points_mono.size(); ++i) {
    //     centered_mono.push_back(points_mono[i] - centroid_mono);
    //     centered_stereo.push_back(points_stereo[i] - centroid_stereo);
    // }

    // // Compute the scaling factor
    // for (int i = 0; i < centered_mono.size(); ++i) {
    //     auto scale = cv::norm(centered_stereo[i]) / cv::norm(centered_mono[i]);
    //     if (scale < 1 || scale > 30 || std::isnan(scale)) {
    //         std::cerr << "Invalid scale factor." << std::endl;
    //         continue;
    //     }
    //     scale_factor += scale;
    //     sumScaleX += centered_stereo[i].x / centered_mono[i].x;
    //     sumScaleY += centered_stereo[i].y / centered_mono[i].y;
    //     sumScaleZ += centered_stereo[i].z / centered_mono[i].z;
    // }
    // scale_factor /= centered_mono.size();
    // double scaleX = sumScaleX / centered_mono.size();
    // double scaleY = sumScaleY / centered_mono.size();
    // double scaleZ = sumScaleZ / centered_mono.size();
    // // Compute the translation vector
    // cv::Point3f translation = centroid_stereo - centroid_mono * scale_factor;
    // // transformed_point_cloud = applyCustomTransformationToPointCloud(mono_pt_cloud, scaleX, scaleY, scaleZ, translation);
    // for (int i = 0; i < mono_pt_cloud.rows; ++i) {
    //     for (int j = 0; j < mono_pt_cloud.cols; ++j) {
    //         cv::Point3f pt_mono = mono_pt_cloud.at<cv::Vec3f>(i, j);
    //         cv::Point3f pt_ = pt_mono *scale_factor + translation;
    //         transformed_point_cloud.at<cv::Vec3f>(i, j) = cv::Vec3f(pt_);
    //     }
    // }
    // end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // std::cout << "Time taken to compute transformation: " << duration.count() << "ms" << std::endl;


    auto ros_image_left = cv_bridge::CvImage(image_l->header, "bgr8", color_l).toImageMsg();
    auto ros_image_right = cv_bridge::CvImage(image_r->header, "bgr8", color_r).toImageMsg();
    pub_img_l->publish(*ros_image_left);
    pub_img_r->publish(*ros_image_right);
    // Convert depth map and color image to point cloud
    rosPointCloud = visualizePointCloud(mono_pt_cloud, left_image);
    // roate the point cloud by pitch angle
    
    
    // rosPointCloud = depthMapToPointCloud(depth_l, left_image, 702.7350, 702.735);
    pub->publish(rosPointCloud);
}

int main(int argc, char **argv) {
    bool use_topic = false;
    // parse for command line arguments
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "--use_topic") == 0) {
            use_topic = true;
        }
    }
    if (use_topic) {
        std::cout << "Using ROS2 topics for image acquisition" << std::endl;
    } else {
        std::cout << "Using OpenCV for image acquisition" << std::endl;
    }
    
    if (!use_topic) {
        cv::Mat frame, result_d,depth;
        cv::Mat point_cloud;

        cv::VideoCapture cap(2);
        // Main Loop
            pcl::visualization::CloudViewer viewer("Point Cloud");
        while (cap.isOpened() && !viewer.wasStopped()) {     

            cap >> frame;

            auto pred = depth_model.predict(frame);
            
            result_d = pred.first;
            depth = pred.second;
            point_cloud = cv::Mat::zeros(frame.size(), CV_32FC3);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud->width = point_cloud.cols;
            cloud->height = point_cloud.rows;
            cloud->is_dense = false;
            cloud->points.resize(cloud->width * cloud->height);

            for (int i = 0; i < depth.rows; i++) {
                for (int j = 0; j < depth.cols; j++) {
                    float z = depth.at<float>(i, j);
                    float x = (j - cx) * z / fx;
                    float y = (i - cy) * z / fy;
                    point_cloud.at<cv::Vec3f>(i, j) = cv::Vec3f(x, y, z);

                    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                        cloud->points[i * point_cloud.cols + j].x = x;
                        cloud->points[i * point_cloud.cols + j].y = y;
                        cloud->points[i * point_cloud.cols + j].z = z;
                    } else {
                        cloud->points[i * point_cloud.cols + j].x = x;
                        cloud->points[i * point_cloud.cols + j].y = y;
                        cloud->points[i * point_cloud.cols + j].z = std::numeric_limits<float>::quiet_NaN();
                    }
                }
            }

            cv::imshow("depth", result_d);
            // Check for ESC key press or ctrl + c
            if (cv::waitKey(5) == 27) // Exit loop when 'Esc' key (ASCII code 27) is pressed
                break;
            viewer.showCloud(cloud);
        }

        } else if (use_topic) {
        rclcpp::init(argc, argv);
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("mono_depth");
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("mono_depth/point_cloud", 10);
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_l = node->create_publisher<sensor_msgs::msg::Image>("mono_depth/left/depth", 10);
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_r = node->create_publisher<sensor_msgs::msg::Image>("mono_depth/right/depth", 10);

        message_filters::Subscriber<sensor_msgs::msg::Image> image_l_sub_  = message_filters::Subscriber<sensor_msgs::msg::Image>(node, "/zedm/zed_node/left/image_rect_color");
        message_filters::Subscriber<sensor_msgs::msg::Image> image_r_sub_  = message_filters::Subscriber<sensor_msgs::msg::Image>(node, "/zedm/zed_node/right/image_rect_color");
        
        // subscribe to imu messages
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu = node->create_subscription<sensor_msgs::msg::Imu>("/zedm/zed_node/imu/data", 10, [](const sensor_msgs::msg::Imu::SharedPtr msg) {
            // std::cout << "IMU Data: " << msg->linear_acceleration.x << std::endl;
            pitch = atan2(-msg->linear_acceleration.x, sqrt(pow(msg->linear_acceleration.y, 2) + pow(msg->linear_acceleration.z, 2)));
        });


        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,sensor_msgs::msg::Image>> sync_;

        sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,sensor_msgs::msg::Image>>(image_l_sub_, image_r_sub_, 10);
        sync_->registerCallback(std::bind(&stereoCallback, std::placeholders::_1, std::placeholders::_2, pub, pub_img_l, pub_img_r));
        
        
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info = node->create_subscription<sensor_msgs::msg::CameraInfo>("/zedm/zed_node/left/camera_info", 10, [](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            // std::cout << "Camera Info: " << msg->header.frame_id << std::endl;
            cx = msg->k[2];
            cy = msg->k[5];
            fx = msg->k[0];
            fy = msg->k[4];
        });
        

        rclcpp::spin(node);
    }
   
    rclcpp::shutdown();

    return 0;
}


