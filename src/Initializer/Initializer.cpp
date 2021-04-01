#include "Initializer.hpp"
#include "../model/Coordinates.hpp"

Initializer::Initializer(std::vector<cv::VideoCapture>& cameras, const double& robotHeight, const cv::Size2d& patternSize, const cv::Size2d& squareSize, const cv::Size2d& fieldImageSize):cameras_(cameras),calibrater_({patternSize,squareSize}),robotHeight_(robotHeight),patternSize_(patternSize),squareSize_(squareSize),fieldImageSize_(fieldImageSize){}

void Initializer::initializeIntrinsicParameters(const int& captureSize){
    
    std::cout << "start IntrinsicParameters" << std::endl;
    // 全カメラのキャリブレーションに用いる画像群
    std::vector<std::vector<cv::Mat>> allFrames;

    // 各カメラで指定数の画像を撮影
    for(auto camera : cameras_){

        // 各カメラのキャリブレーションに用いる画像群
        std::vector<cv::Mat> frames;

        std::cout << "press any key for capture image" << std::endl;
        // 画像を１枚ずつ指定数まで撮影
        for(int i = 0; i < captureSize;i++){
            cv::Mat frame;
            std::cout << captureSize - i << " more times" << std::endl;
            while(true){
                camera.read(frame);
                cv::imshow("IntrinsicParameters",frame);
                if(cv::waitKey(1)>0){
                    break;
                }
            }
            
            
            frames.emplace_back(frame);
        }

        // 各カメラの画像群を挿入
        allFrames.emplace_back(frames);
    }

    // 撮影した画像を元に内部パラメータを計算
    calibrater_.calculateIntrinsicParameters(allFrames);
}

void Initializer::initializeIntrinsicParameters(std::vector<cv::FileStorage>& fileStrages){
    
    std::cout << "load IntrinsicParameters" << std::endl;
    calibrater_.loadIntrinsicParameters(fileStrages);
    
}

void Initializer::initializeExtrinsicParameters(){

    // 全カメラのキャリブレーションに用いる画像群
    std::vector<cv::Mat> allFrames;

    // 各カメラで指定数の画像を撮影
    int i = 0;
    for(auto camera : cameras_){

        cv::Mat frame;

        std::cout << "press any key for capture image" << std::endl;
        while(true){
            camera.read(frame);
            cv::imshow("ExtrinsicParameters",frame);
            cv::Mat grayFrame;
            std::vector<util::ImageCoordinates> corners(1);
            cv::cvtColor(frame,grayFrame,cv::COLOR_RGB2GRAY);
            if (cv::findChessboardCorners(grayFrame, patternSize_, corners)){
                std::cout << "\r" << "\e[42m" << "find!" << "\e[m"<< std::string(20, ' ');
                break;
            }else{
                std::cout << "\r" << "\e[41m" << "not find!..." << "\e[m"<< std::string(20, ' ');
            }
            if(cv::waitKey(1)>0){
                break;
            }
        }
        //frame = cv::imread("../"+std::to_string(i)+".jpg");
        i++;
        // 各カメラの画像を挿入
        allFrames.emplace_back(frame);
    }
    
    // 撮影した画像を元に外部パラメータを計算
    calibrater_.calculateExtrinsicParameters(allFrames);
}

void Initializer::initializeHomography(){

    // 実世界の空間上での座標(キャリブレーションボードに対して x方向:右 y方向:上 z方向:手前)
    const std::vector<util::WorldCoordinates> srcPoints{
        util::WorldCoordinates(-(patternSize_.width-1)/2 * squareSize_.width, (patternSize_.height-1)/2 * squareSize_.height,robotHeight_),
        util::WorldCoordinates(-(patternSize_.width-1)/2 * squareSize_.width,-(patternSize_.height-1)/2 * squareSize_.height,robotHeight_),
        util::WorldCoordinates( (patternSize_.width-1)/2 * squareSize_.width, (patternSize_.height-1)/2 * squareSize_.height,robotHeight_),
        util::WorldCoordinates( (patternSize_.width-1)/2 * squareSize_.width,-(patternSize_.height-1)/2 * squareSize_.height,robotHeight_)
    };

    // 最終的な画像に投影された際の座標(投影後の画像に対して x方向:右 y方向:下)
    const std::vector<util::ImageCoordinates> dstPoints{
        util::ImageCoordinates(fieldImageSize_.width/2 - patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 - patternSize_.height/2 * squareSize_.height),
        util::ImageCoordinates(fieldImageSize_.width/2 - patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 + patternSize_.height/2 * squareSize_.height),
        util::ImageCoordinates(fieldImageSize_.width/2 + patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 - patternSize_.height/2 * squareSize_.height),
        util::ImageCoordinates(fieldImageSize_.width/2 + patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 + patternSize_.height/2 * squareSize_.height)
    };

    std::vector<model::Camera> cameraParameters = parameters();

    for(int i = 0; i < cameras_.size(); i++){
        
        std::vector<util::ImageCoordinates> projectedPoints(4);
        // カメラのパラメータを用いて、実世界の空間上での座標が画像上ではどの座標になるかを計算
        cv::projectPoints(srcPoints,cameraParameters.at(i).rvecs(),cameraParameters.at(i).tvecs(),cameraParameters.at(i).cameraMatrix(),cameraParameters.at(i).distCoeffs(),projectedPoints);
        
        // その座標をカメラの歪みの分ずらす
        std::vector<util::ImageCoordinates> undistortedPoints(4);
        cv::undistortPoints(projectedPoints, undistortedPoints, cameraParameters.at(i).cameraMatrix(), cameraParameters.at(i).distCoeffs(),cv::noArray(),cameraParameters.at(i).cameraMatrix());

        // homography 行列を計算
        cv::Mat homography = cv::getPerspectiveTransform(undistortedPoints.data(),dstPoints.data());
        
        homographyMatrixs_.emplace_back(homography);
    }
    
}

void Initializer::initializeDefaultFields(){
    
    /// 各カメラでフィールドの画像を撮影 ///

    for(auto& camera : cameras_){

        cv::Mat frame;

        std::cout << "press any key for capture default field image" << std::endl;
        while(true){
            camera.read(frame);
            cv::imshow("DefaultFields",frame);
            if(cv::waitKey(1)>0){
                break;
            }
        }
        // 各カメラの画像を挿入
        defaultFields_.emplace_back(frame);
    }

    /// フィールドを合成するためのホモグラフィ行列を取得 ///

    // 実世界の空間上での座標(キャリブレーションボードに対して x方向:右 y方向:上 z方向:手前)
    const std::vector<util::WorldCoordinates> srcPoints{
        util::WorldCoordinates(-(patternSize_.width-1)/2 * squareSize_.width, (patternSize_.height-1)/2 * squareSize_.height,0),
        util::WorldCoordinates(-(patternSize_.width-1)/2 * squareSize_.width,-(patternSize_.height-1)/2 * squareSize_.height,0),
        util::WorldCoordinates( (patternSize_.width-1)/2 * squareSize_.width, (patternSize_.height-1)/2 * squareSize_.height,0),
        util::WorldCoordinates( (patternSize_.width-1)/2 * squareSize_.width,-(patternSize_.height-1)/2 * squareSize_.height,0)
    };

    // 最終的な画像に投影された際の座標(投影後の画像に対して x方向:右 y方向:下)
    const std::vector<util::ImageCoordinates> dstPoints{
        util::ImageCoordinates(fieldImageSize_.width/2 - patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 - patternSize_.height/2 * squareSize_.height),
        util::ImageCoordinates(fieldImageSize_.width/2 - patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 + patternSize_.height/2 * squareSize_.height),
        util::ImageCoordinates(fieldImageSize_.width/2 + patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 - patternSize_.height/2 * squareSize_.height),
        util::ImageCoordinates(fieldImageSize_.width/2 + patternSize_.width/2 * squareSize_.width, fieldImageSize_.height/2 + patternSize_.height/2 * squareSize_.height)
    };

    std::vector<model::Camera> cameraParameters = parameters();
    std::vector<cv::Mat> fieldHomographyMatrixs;
    for(int i = 0; i < cameras_.size(); i++){
        
        std::vector<util::ImageCoordinates> projectedPoints(4);
        // カメラのパラメータを用いて、実世界の空間上での座標が画像上ではどの座標になるかを計算
        cv::projectPoints(srcPoints,cameraParameters.at(i).rvecs(),cameraParameters.at(i).tvecs(),cameraParameters.at(i).cameraMatrix(),cameraParameters.at(i).distCoeffs(),projectedPoints);
        
        // その座標をカメラの歪みの分ずらす
        std::vector<util::ImageCoordinates> undistortedPoints(4);
        cv::undistortPoints(projectedPoints, undistortedPoints, cameraParameters.at(i).cameraMatrix(), cameraParameters.at(i).distCoeffs(),cv::noArray(),cameraParameters.at(i).cameraMatrix());

        // homography 行列を計算
        cv::Mat homography = cv::getPerspectiveTransform(undistortedPoints.data(),dstPoints.data());
        
        fieldHomographyMatrixs.emplace_back(homography);
    }
    
    /// 各カメラからの画像を合成してフィールド画像を作成 ///

    for (int i = 0; i < defaultFields_.size(); i++) {
        
        cv::Mat undistortedImage = cv::Mat::zeros(fieldImageSize_.height, fieldImageSize_.width, CV_8UC3);
        cv::Mat warpedImage = cv::Mat::zeros(fieldImageSize_.height, fieldImageSize_.width, CV_8UC3);
        cv::Mat leanImage = defaultFields_.at(i);

        cv::undistort(leanImage, undistortedImage, cameraParameters.at(i).cameraMatrix(), cameraParameters.at(i).distCoeffs(), cameraParameters.at(i).cameraMatrix());
        cv::warpPerspective(undistortedImage, warpedImage, fieldHomographyMatrixs.at(i), warpedImage.size());

        if(fieldImage_.empty()){
            fieldImage_ = warpedImage;
        }else{
            // 合成割合を決める定数を決める
            double alpha = 1 / static_cast<double>(i+1);
            double beta = 1 - alpha;
            // 合成元をコピー
            cv::Mat tmp = fieldImage_;
            cv::addWeighted(warpedImage, alpha, tmp, beta, 0,fieldImage_);
        }
    }
}

void Initializer::initializeFieldRange(){

}


std::vector<model::Camera> Initializer::parameters()const{
    return calibrater_.parameters();
}

std::vector<cv::Mat> Initializer::homographyMatrixs()const{
    return homographyMatrixs_;
}

std::vector<cv::Mat> Initializer::defaultFields()const{
    return defaultFields_;
}

cv::Mat Initializer::field()const{
    return fieldImage_;
}

cv::Size2d Initializer::fieldImageSize()const{
    return fieldImageSize_;
}