#include "Controller.hpp"

Controller::Controller(std::vector<cv::VideoCapture>& cameras, Initializer& initializer):cameras_(cameras),initializer_(initializer),converter_(initializer_){}

bool Controller::initialize(){
    
    const int CV_WAITKEY_ENTER = 13;
    std::cout << "If you want to use the camera parameters file, press enter." << std::endl;
    if(cv::waitKey() == CV_WAITKEY_ENTER){
        std::vector<cv::FileStorage> fileStrages;

        for(int i = 0; i < cameras_.size(); i++){
            std::string filePath = "../prm/camera" + std::to_string(i) + ".xml";
            cv::FileStorage fileStrage;

            if(fileStrage.open(filePath, cv::FileStorage::READ)){
                fileStrages.emplace_back(fileStrage);
            }
        }

        initializer_.initializeIntrinsicParameters(fileStrages);
    }else{
        initializer_.initializeIntrinsicParameters(10);
    }
    
    initializer_.initializeExtrinsicParameters();
    initializer_.initializeHomography();
    initializer_.initializeDefaultFields();
    initializer_.initializeFieldRange();
    return true;
}

void Controller::execute(){
    std::vector<cv::Mat> frames;
    int i = 0;
    for(auto camera : cameras_){
        cv::Mat frame;
        camera.read(frame);
        frames.emplace_back(frame);
        cv::imshow("camera"+std::to_string(i),frame);
        i++;
    }
    cv::Mat skyImage;
    skyImage = converter_.imagesToImage(frames,initializer_.fieldImageSize());
    cv::Mat sky;
    cv::Size windowSize(960,540);
    cv::resize(skyImage, sky, windowSize);
    cv::waitKey(10);
    cv::imshow("image",sky);
}