#include <opencv2/opencv.hpp>
#include <cmath>
#include "vision_core/mono_depth.hpp"


// Using std and sl namespaces
using namespace std;
// using namespace sl;

// void parseArgs(int argc, char **argv, sl::InitParameters& param);
// cv::Mat slMat2cvMat(sl::Mat& input);
float cx = 611.0380249023438, cy = 402.2382507324219, fx = 749.0894775390625, fy = 749.0894775390625;




int main(int argc, char **argv) {
    // Create MonoDepth instance here instead of globally
    MonoDepth depth_model;

    std::cout << "Loading model from " << "depth_anything_vits14.engine" << "..." << std::endl;
    cv::Mat frame, result_d,depth;
    cv::Mat point_cloud;

    cv::VideoCapture cap(0);
    // Main Loop
    while (cap.isOpened()) {     

      cap >> frame;

      depth = depth_model.predict(frame);

      point_cloud = cv::Mat::zeros(frame.size(), CV_32FC3);
      for (int i = 0; i < depth.rows; i++) {
          for (int j = 0; j < depth.cols; j++) {
              point_cloud.at<cv::Vec3f>(i, j) =  cv::Vec3f(
                                                  (j - cx) * depth.at<float>(i, j) / fx, 
                                                  (i - cy) * depth.at<float>(i, j) / fy, 
                                                  depth.at<float>(i, j)
                                                  );
          }
      }
      cv::imshow("depth", depth);
    //   cv::imshow("frame", frame);
            // Check for ESC key press or ctrl + c
      if (cv::waitKey(5) == 27) // Exit loop when 'Esc' key (ASCII code 27) is pressed
          break;
    }

    // Free allocated memory before closing the ZED
    // Close the ZED
    // zed.close();

    return 0;
}


