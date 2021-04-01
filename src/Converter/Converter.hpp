#pragma once
#include <opencv2/opencv.hpp>
#include "../Calibrater/Calibrater.hpp"
#include "../Initializer/Initializer.hpp"

class Converter{
    private:
        Initializer& initializer_;
    public:
        Converter(Initializer& initializer);

        cv::Mat imagesToImage(const std::vector<cv::Mat>& images, const cv::Size2d& fieldImageSize);
};
