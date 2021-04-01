#include <opencv2/core.hpp>
#include "Calibrater.hpp"

CameraCalibrater::CameraCalibrater(const cv::Size2d& patternSize, const cv::Size2d& squareSize):patternSize_(patternSize),squareSize_(squareSize){}

// カメラパラメータの取得
std::vector<model::Camera> CameraCalibrater::parameters()const{
    return parameters_;
}

// 内部パラメータの計算
void CameraCalibrater::calculateIntrinsicParameters(std::vector<std::vector<cv::Mat>>& checkerImages){
    
    // 各カメラごとの画像群で計算
    for(auto& images : checkerImages){

        // キャリブレーションに必要な画像数に達しているかの確認
        if(images.size()<5){
            cv::error(cv::Error::StsBadSize, "too little", CV_Func, __FILE__, __LINE__);
            return;
        }

        // 各画像に対しての回転・並進ベクトル
        std::vector<cv::Mat> rotationVectors;
        std::vector<cv::Mat> translationVectors;

        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;

        // 画像座標系でのコーナー座標の取得
        std::vector<std::vector<util::ImageCoordinates>> corners(images.size());
        corners = findCorners(images);

        // 世界座標系でのコーナー座標の定義
        std::vector<std::vector<util::WorldCoordinates>> basedCorners(images.size());
        for (int i = 0; i < images.size(); i++) {
            for (int j = 0; j < patternSize_.area(); j++) {
                basedCorners.at(i).emplace_back(util::WorldCoordinates(static_cast<float>(j % patternSize_.width - static_cast<float>(patternSize_.width - 1)/2.0) * squareSize_.width,
                 static_cast<float>(-j / patternSize_.width + static_cast<float>(patternSize_.height - 1)/2.0) * squareSize_.height,
                 0.0));
            }
        }

        // カメラ行列、歪みパラメータの取得
        cv::calibrateCamera(basedCorners, corners, images.at(0).size(), cameraMatrix, distCoeffs, rotationVectors, translationVectors);

        // model::Cameraの形に変更
        model::Camera parameter;
        parameter.setCameraMatrix(cameraMatrix);
        parameter.setDistCoeffs(distCoeffs);
        
        //カメラごとにパラメータをセット
        parameters_.emplace_back(parameter);
    }
    
}

// ファイルから内部パラメータの読み込み
void CameraCalibrater::loadIntrinsicParameters(std::vector<cv::FileStorage>& fileStrages){
    for(auto& fileStrage:fileStrages){
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        fileStrage["cameraMatrix"] >> cameraMatrix;
        fileStrage["distCoeffs"] >> distCoeffs;

        // model::Cameraの形に変更
        model::Camera parameter;
        parameter.setCameraMatrix(cameraMatrix);
        parameter.setDistCoeffs(distCoeffs);
        
        //カメラごとにパラメータをセット
        parameters_.emplace_back(parameter);
        
    }
}

// 外部パラメータの計算
void CameraCalibrater::calculateExtrinsicParameters(std::vector<cv::Mat>& checkerImages){
    // 各カメラごとの画像で計算
    for(int i = 0; i < checkerImages.size(); i++){
        // 画像座標系でのコーナー座標の取得
        std::vector<util::ImageCoordinates> corners = findCorners(checkerImages.at(i));
        
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 20, 0.001);
        cv::Mat rotationVectors;
        cv::Mat translationVectors;

        // 世界座標系でのコーナー座標の定義
        std::vector<cv::Point3d> basedCorners(patternSize_.area());
        for(int j = 0;j<patternSize_.height;j++){
            for(int k = 0;k<patternSize_.width;k++){
                basedCorners[j*patternSize_.width+k] = cv::Point3d((k-static_cast<double>(patternSize_.width-1)/2)*squareSize_.width ,-(j-static_cast<double>(patternSize_.height-1)/2)*squareSize_.height,0);
            }
        }

        // 回転・並進ベクトルの計算
        cv::solvePnPRansac(basedCorners,corners,parameters_.at(i).cameraMatrix(), parameters_.at(i).distCoeffs(),rotationVectors,translationVectors);
        
        parameters_.at(i).setTvecs(translationVectors);
        parameters_.at(i).setRvecs(rotationVectors);

    }
}

// キャリブレーションボードからのコーナーの取得
std::vector<std::vector<util::ImageCoordinates>> CameraCalibrater::findCorners(std::vector<cv::Mat>& checkerImages){
    
    std::vector<std::vector<util::ImageCoordinates>> corners(checkerImages.size());
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 20, 0.001);
    std::vector<cv::Mat> grayImages(checkerImages.size());

    for (int i = 0; i < checkerImages.size(); i++){
        // 画像の色をRGBからGRAYに変換
        cv::cvtColor(checkerImages[i],grayImages[i],cv::COLOR_RGB2GRAY);
        if (cv::findChessboardCorners(grayImages[i], patternSize_, corners[i])) {
            // サブピクセル精度に変更
            cv::cornerSubPix(grayImages[i], corners[i], cv::Size(11, 11), cv::Size(-1, -1), criteria);
        } else {
            // コーナーが見つからなかったときの例外処理
            cv::error(cv::Error::StsBadSize, "...at least 1 corner not found.", CV_Func, __FILE__, __LINE__);
        }
    }
    return corners;
}

// キャリブレーションボードからのコーナーの取得
std::vector<util::ImageCoordinates> CameraCalibrater::findCorners(cv::Mat& checkerImage){
    
    std::vector<util::ImageCoordinates> corners(1);
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 20, 0.001);
    
    // 画像の色をRGBからGRAYに変換
    cv::Mat grayImage;
    cv::cvtColor(checkerImage,grayImage,cv::COLOR_RGB2GRAY);

    if (cv::findChessboardCorners(grayImage, patternSize_, corners)) {
        // サブピクセル精度に変更
        cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    } else {
        // コーナーが見つからなかったときの例外処理
        cv::error(cv::Error::StsBadSize, "...at least 1 corner not found.", CV_Func, __FILE__, __LINE__);
    }
    return corners;
}
