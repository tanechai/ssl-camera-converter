#include "Controller/Controller.hpp"
#include "Initializer/Initializer.hpp"
#include "Calibrater/Calibrater.hpp"

int main(){
    const double robotHeight = 120;
    const cv::Size2d patternSize(10,7);
    const cv::Size2d squareSize(67,67);
    const cv::Size2d fieldImageSize(4000,2000);
    const int maxCameras = 5;
    std::vector<cv::VideoCapture> cameras;
    for(int i = 0; i < maxCameras; i++){
        cv::VideoCapture camera(i);
        if(camera.isOpened()){
            cameras.emplace_back(camera);
            std::cout << "Camera " << i << " is opened." << std::endl;
        }
    }
    
    std::string windowName = "main window";
    cv::namedWindow(windowName);

    CameraCalibrater calibrater(patternSize,squareSize);
    Initializer initializer(cameras,robotHeight,patternSize,squareSize,fieldImageSize);
    Controller controller(cameras,initializer);
    
    if(controller.initialize()){
        std::cout << "All settings have been completed." << std::endl;
    }
    
    while(true){
        controller.execute();
    }

}