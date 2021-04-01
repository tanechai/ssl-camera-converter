#pragma once
#include <opencv2/opencv.hpp>
#include "../Initializer/Initializer.hpp"
#include "../Converter/Converter.hpp"

class Controller{
    private:
        std::vector<cv::VideoCapture> cameras_;
        Initializer initializer_;
        Converter converter_;

    public:
        Controller(std::vector<cv::VideoCapture>& cameras, Initializer& initializer);
        bool initialize();
        void execute();

};
