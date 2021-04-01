#pragma once
#include <opencv2/opencv.hpp>
#include "../model/Coordinates.hpp"
#include "../model/Camera.hpp"

class CameraCalibrater{
private:
    std::vector<model::Camera> parameters_;
    cv::Size patternSize_;
    cv::Size2d squareSize_;
public:
    CameraCalibrater(const cv::Size2d& patternSize, const cv::Size2d& squareSize);

    // 内部パラメータの計算
    void calculateIntrinsicParameters(std::vector<std::vector<cv::Mat>>& checkerImages);
    
    // 内部パラメータの読み込み
    void loadIntrinsicParameters(std::vector<cv::FileStorage>& fileStrages);

    // 外部パラメータの計算
    void calculateExtrinsicParameters(std::vector<cv::Mat>& checkerImage);

    // パラメータ取得
    std::vector<model::Camera> parameters()const;

    // キャリブレーションボードからのコーナーの取得
    std::vector<std::vector<util::ImageCoordinates>> findCorners(std::vector<cv::Mat>& checkerImages);
    std::vector<util::ImageCoordinates> findCorners(cv::Mat& checkerImage);

    //createParameterFile();
};