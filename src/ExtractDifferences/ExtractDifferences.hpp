#pragma once
#include <opencv2/opencv.hpp>

class ExtractDifferences{
    public:
        ExtractDifferences(cv::Mat& standardImg_);
        cv::Mat getDifferences(const cv::Mat& img);
    private:
        cv::Mat standardImg;
        cv::Mat grayStandardImg;

};
