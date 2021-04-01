#pragma once
#include <opencv2/opencv.hpp>

namespace model
{

class Camera{
    private:
        cv::Mat cameraMatrix_;
        cv::Mat distCoeffs_;
        cv::Mat rvecs_;
        cv::Mat tvecs_;
    public:
        Camera();
        Camera(cv::Mat& cameraMatrix,cv::Mat& distCoeffs);
        Camera(cv::Mat& cameraMatrix,cv::Mat& distCoeffs,cv::Mat& rvecs,cv::Mat& tvecs);
        
        cv::Mat cameraMatrix()const;
        cv::Mat distCoeffs()const;
        cv::Mat rvecs()const;
        cv::Mat tvecs()const;

        void setCameraMatrix(cv::Mat& cameraMatrix);
        void setDistCoeffs(cv::Mat& distCoeffs);
        void setRvecs(cv::Mat& rvecs);
        void setTvecs(cv::Mat& tvecs);
};

} // namespace model

