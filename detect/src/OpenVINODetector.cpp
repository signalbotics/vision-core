#include "detect/OpenVINODetector.hpp"
#include <opencv2/imgproc.hpp>

namespace vision_core{


#ifdef WITH_OPENVINO

OpenVINODetector::OpenVINODetector(const Configs& config) : BaseDetector(config) {
    try {
        auto model = core.read_model(config.openvinoModelXML, config.openvinoModelBIN);
        compiled_model = core.compile_model(model, config.openvinoDevice);
        infer_request = compiled_model.create_infer_request();
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize OpenVINODetector: " << e.what() << std::endl;
        exit(1);
    }
}

void OpenVINODetector::detect(const cv::Mat& frame, std::vector<cv::Rect>& boxes,
                              std::vector<float>& confidences, std::vector<int>& classIds) {

    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, cv::Size(416, 416));
    // resizedFrame.convertTo(resizedFrame, CV_32F, 1/255.0);
    cv::Mat blob = cv::dnn::blobFromImage(resizedFrame, 1.0, cv::Size(416, 416), cv::Scalar(), true, false);

    infer_request.set_input_tensor(ov::Tensor{ov::element::f32, {1, 416, 416, 3}, blob.data});

    infer_request.infer();

    auto output_tensor_52 = infer_request.get_output_tensor(2); // 52x52
    auto output_tensor_26 = infer_request.get_output_tensor(1); // 26x26
    auto output_tensor_13 = infer_request.get_output_tensor(0); // 13x13

    decodeYOLOOutput(output_tensor_52, 52, frame, boxes, confidences, classIds);
    decodeYOLOOutput(output_tensor_26, 26, frame, boxes, confidences, classIds);
    decodeYOLOOutput(output_tensor_13, 13, frame, boxes, confidences, classIds);

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidenceThreshold, nmsThreshold, indices);

    std::vector<cv::Rect> filteredBoxes;
    std::vector<float> filteredConfidences;
    std::vector<int> filteredClassIds;
    for (int idx : indices) {
        filteredBoxes.push_back(boxes[idx]);
        filteredConfidences.push_back(confidences[idx]);
        filteredClassIds.push_back(classIds[idx]);
    }

    boxes = std::move(filteredBoxes);
    confidences = std::move(filteredConfidences);
    classIds = std::move(filteredClassIds);
}

void OpenVINODetector::decodeYOLOOutput(const ov::Tensor& output_tensor, int grid_size, const cv::Mat& frame,
                                        std::vector<cv::Rect>& boxes, std::vector<float>& confidences, 
                                        std::vector<int>& classIds) {
    const float* data = output_tensor.data<float>();
    int num_classes = 80;  
    int anchor_boxes = 3;

    for (int i = 0; i < grid_size * grid_size * anchor_boxes; ++i) {
        int offset = i * (5 + num_classes);

        float x = (data[offset + 0] + i % grid_size) / grid_size;
        float y = (data[offset + 1] + i / grid_size) / grid_size;
        float width = data[offset + 2];
        float height = data[offset + 3];
        float confidence = data[offset + 4];
        if (confidence > confidenceThreshold) {
            int left = static_cast<int>((x - width / 2) * frame.cols);
            int top = static_cast<int>((y - height / 2) * frame.rows);
            int box_width = static_cast<int>(width * frame.cols);
            int box_height = static_cast<int>(height * frame.rows);

            boxes.emplace_back(left, top, box_width, box_height);
            confidences.push_back(confidence);

            int class_id = -1;
            float max_class_score = 0.0f;
            for (int c = 0; c < num_classes; ++c) {
                float class_score = data[offset + 5 + c];
                if (class_score > max_class_score) {
                    max_class_score = class_score;
                    class_id = c;
                }
            }
            classIds.push_back(class_id);
        }
    }
}

#endif // WITH_OPENVINO

} // namespace vision_core