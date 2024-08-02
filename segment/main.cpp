#include "segment/segment.h"
#include "utils.h"

void segmentClickedPoint(Segment& segment) {

    // auto image = imread(imagePath);
    // Data structure to hold clicked point
    PointData pointData;
    pointData.clicked = false;

    // Create a window to display the image
    cv::namedWindow("Image");
    // Set the callback function for mouse events on the displayed cloned image
    cv::setMouseCallback("Image", onMouse, &pointData);


    cv::Mat clonedImage, image;
    int clickCount = 0;

    // Loop until Esc key is pressed
    cv::VideoCapture cap(0);
    while (cap.isOpened())
    {
        // Display the original image
        cap >> image;   
        clonedImage = image.clone();     
        cv::imshow("Image", image);

        if (pointData.clicked)
        {   
            std::cout << "Point: " << pointData.point << std::endl;
            pointData.clicked = false; // Reset clicked flag            

            auto mask = segment.predict(image, { pointData.point }, { 1.0f });

            cv::circle(image, pointData.point, 5, cv::Scalar(0, 0, 255), -1);

            if (clickCount >= CITYSCAPES_COLORS.size()) clickCount = 0;
            overlay(image, mask, CITYSCAPES_COLORS[clickCount * 9]);
            clickCount++;
            cv::imshow("mask", image);
        }

        // Check for Esc key press
        char key = cv::waitKey(1);
        if (key == 27) // ASCII code for Esc key
        {
            clonedImage = image.clone();
            cap.release();
        }
    }
    cv::destroyAllWindows();
}

void segmentBbox(Segment& segment, string imagePath, string outputPath, vector<Point> bbox)
{
    auto image = imread(imagePath);

    // 2 : Bounding box top-left, 3 : Bounding box bottom-right
    vector<float> labels = { 2, 3 }; 

    auto mask = segment.predict(image, bbox, labels);

    overlay(image, mask);

    rectangle(image, bbox[0], bbox[1], cv::Scalar(255, 255, 0), 3);

    imwrite(outputPath, image);
}

void segmentWithPoint(Segment& segment, string imagePath, string outputPath, Point promptPoint)
{
    auto image = imread(imagePath);
    
    // 1 : Foreground
    vector<float> labels = { 1.0f }; 

    auto mask = segment.predict(image, { promptPoint }, labels);

    overlay(image, mask);

    imwrite(outputPath, image);
}

int main(int argc, char** argv)
{
    /* 1. Load engine examples */

    // Option 1: Load the engines
    //Segment segment("data/resnet18_image_encoder.engine",  "data/mobile_sam_mask_decoder.engine");
    // Option 2: Build the engines from onnx files


    // model path
    std::string model_path_encoder = argv[1];
    std::string model_path_decoder = argv[2];
    Segment segment( model_path_encoder, model_path_decoder);
    // Segment segment("../data/resnet18_image_encoder.onnx", "../data/mobile_sam_mask_decoder.onnx");

    /* 2. Segmentation examples */
    
    // // Demo 1: Segment using a point
    // segmentWithPoint(segment, "../assets/dog.jpg", "../assets/dog_mask.jpg", Point(1300, 900));
    
    // // Demo 2: Segment using a bounding box
    // segmentBbox(segment, "../assets/dogs.jpg", "../assets/dogs_mask.jpg", { Point(100, 100), Point(750, 759) });

    // Demo 3: Segment the clicked object
    segmentClickedPoint(segment);

    return 0;
}
