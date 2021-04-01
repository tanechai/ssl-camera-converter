#include <opencv2/opencv.hpp>
#include "Camera.hpp"

namespace model
{


    Camera::Camera(){};
    Camera::Camera(cv::Mat& cameraMatrix, cv::Mat& distCoeffs):cameraMatrix_(cameraMatrix),distCoeffs_(distCoeffs){};
    Camera::Camera(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rvecs, cv::Mat& tvecs):cameraMatrix_(cameraMatrix),distCoeffs_(distCoeffs),rvecs_(rvecs),tvecs_(tvecs){};
    
    
    cv::Mat Camera::cameraMatrix()const{
        return cameraMatrix_;
    };

    cv::Mat Camera::distCoeffs()const{
        return distCoeffs_;
    };

    cv::Mat Camera::rvecs()const{
        return rvecs_;
    };

    cv::Mat Camera::tvecs()const{
        return tvecs_;
    };


    void Camera::setCameraMatrix(cv::Mat& cameraMatrix){
        cameraMatrix_ = cameraMatrix;
    };

    void Camera::setDistCoeffs(cv::Mat& distCoeffs){
        distCoeffs_ = distCoeffs;
    };

    void Camera::setRvecs(cv::Mat& rvecs){
        rvecs_ = rvecs;
    };

    void Camera::setTvecs(cv::Mat& tvecs){
        tvecs_ = tvecs;
    };


} // namespace model

