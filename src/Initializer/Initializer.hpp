#pragma once
#include <opencv2/opencv.hpp>
#include "../Calibrater/Calibrater.hpp"

class Initializer{
    private:
        std::vector<cv::VideoCapture> cameras_;
        CameraCalibrater calibrater_;
        double robotHeight_;
        cv::Size2d patternSize_;
        cv::Size2d squareSize_;
        cv::Size2d fieldImageSize_;
        
        std::vector<cv::Mat> homographyMatrixs_;
        std::vector<cv::Mat> defaultFields_;
        cv::Mat fieldImage_;

    public:
        Initializer(std::vector<cv::VideoCapture>& cameras, const double& robotHeight, const cv::Size2d& patternSize, const cv::Size2d& squareSize, const cv::Size2d& fieldImageSize);

        // 初期設定
        void initializeIntrinsicParameters(const int& captureSize);
        void initializeIntrinsicParameters(std::vector<cv::FileStorage>& fileStrages);
        void initializeExtrinsicParameters();
        void initializeHomography();
        void initializeDefaultFields();
        void initializeFieldRange();

        std::vector<model::Camera> parameters()const;
        std::vector<cv::Mat> homographyMatrixs()const;
        std::vector<cv::Mat> defaultFields()const;
        cv::Mat field()const;
        cv::Size2d fieldImageSize()const;

};
